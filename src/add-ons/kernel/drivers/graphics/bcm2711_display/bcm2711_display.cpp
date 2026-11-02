/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The Raspberry Pi 4's display as its VideoCore firmware drives it: the
	framebuffer the boot loader set up, the monitor's EDID as the firmware read
	it, and a vertical blank semaphore. The firmware signals each vertical
	blank through the SMI block's interrupt (as Linux's vc4_firmware_kms
	expects); without it, a thread waits on the firmware's SET_VSYNC call.

	Overlays are firmware planes 1-6 showing buffers this driver allocates
	(contiguous, below 1 GiB). The firmware has no color key, but blends with
	per-pixel alpha: while an overlay is visible, the desktop is scanned out on
	plane 0 as ARGB8888 above the overlays, and app_server writes it opaque
	except where the overlay's key color is (B_OVERLAY_KEYING_USES_ALPHA).
*/


#include <bcm2711_display.h>
#include <bcm2835_mailbox.h>

#include <stdlib.h>
#include <string.h>

#include <boot_item.h>
#include <driver_settings.h>
#include <frame_buffer_console.h>
#include <graphic_driver.h>
#include <KernelExport.h>
#include <lock.h>
#include <util/AutoLock.h>
#include <AutoDeleter.h>
#include <arch/cpu.h>
#include <kernel.h>
#include <team.h>
#include <vm/vm.h>

extern "C" {
#include <libfdt.h>
}


#define TRACE_ALWAYS(x...)	dprintf("bcm2711_display: " x)
#define ERROR(x...)			dprintf("bcm2711_display: " x)


#define DEVICE_NAME "graphics/bcm2711_display"


extern void* gFDT;


int32 api_version = B_CUR_DRIVER_API_VERSION;


// SMI registers, as the firmware uses them to signal vertical blanks
enum {
	SMI_CS		= 0x00 / 4,
	SMI_DSW0	= 0x14 / 4,
	SMI_DSW1	= 0x1c / 4,
};
// SMI_CS: the interrupt bits the firmware sets (INTD, INTT, INTR)
static const uint32 kSmiInterrupts = 7 << 9;
// SMI_DSWn: a newer firmware's per-display vertical blank flag (bit 0) comes
// with this tag, which also acknowledges it
static const uint32 kSmiDisplayTag = 0xabcd0000;

// how long the SMI interrupt gets to show up before the SET_VSYNC thread
// takes over
static const bigtime_t kVblankProbeTime = 100000;

// The firmware's planes: 8 per display, 0 = primary, 7 = the cursor; the
// firmware framebuffer is under them, at layer -127
static const uint8 kPrimaryPlane = 0;
static const int8 kPrimaryLayer = 0;
static const uint8 kCursorPlane = 7;
static const int8 kCursorLayer = 127;
// Overlays lie under the desktop while it is keyed (scanned out with alpha),
// or above it (the "overlay_above_desktop" setting)
static const int8 kOverlayLayer = 16;
static const int8 kKeyedDesktopLayer = 64;
static const int8 kOverlayAboveLayer = 100;
// VC_IMAGE_XRGB8888 / ARGB8888: B, G, R, X/A in memory, as B_RGB32 / B_RGBA32
static const uint8 kImageXRGB8888 = BCM2711_DISPLAY_IMAGE_XRGB8888;
static const uint8 kImageARGB8888 = 43;
// The firmware scans out from the low 1 GiB only (it keeps 30 address bits)
static const phys_addr_t kScanOutLimit = (phys_addr_t)1 << 30;

// overlay buffers: how many of the native mode's size are reserved when the
// device is opened (the "overlay_reserve" setting), and at most in all
static const uint32 kDefaultOverlayReserve = 3;
static const uint32 kMaxOverlayBuffers = 32;
static const size_t kMaxOverlayBufferSize = 64 * 1024 * 1024;

// a plane update within this time before the predicted vertical blank may
// be latched one frame late: the firmware takes it at an update point ~1.6 ms
// before the blank (measured on the Phoenix-RTOS port of this board)
static const bigtime_t kLateUpdateMargin = 2000;
// the per-plane statistics are logged this often while the plane changes
static const bigtime_t kOverlayReportInterval = 10000000;


/*!	The value buffer of the firmware's SET_PLANE property (as Linux's
	vc4_firmware_kms): source rectangle in 16.16 fixed point.
*/
struct firmware_plane {
	uint8	display;
	uint8	plane_id;
	uint8	image_type;
	int8	layer;
	uint16	width;
	uint16	height;
	uint16	pitch;
	uint16	vpitch;
	uint32	src_x;
	uint32	src_y;
	uint32	src_w;
	uint32	src_h;
	int16	dst_x;
	int16	dst_y;
	uint16	dst_w;
	uint16	dst_h;
	uint8	alpha;
	uint8	num_planes;
	uint8	is_vu;
	uint8	color_encoding;
	uint32	planes[4];
	uint32	transform;
} _PACKED;

static_assert(sizeof(firmware_plane) == 60, "SET_PLANE takes 60 bytes");


struct display_info {
	int32			open_count;
	bcm2711_display_shared_info* shared_info;
	area_id			shared_area;

	area_id			frame_buffer_area;
		// the one the accelerant clones: boot_area or mode_area
	addr_t			frame_buffer;

	// the boot loader's frame buffer, scanned out by the firmware itself, in
	// the native mode
	area_id			boot_area;
	addr_t			boot_frame_buffer;
	uint32			boot_bytes_per_row;
	uint32			depth;

	// a smaller mode's frame buffer, scaled up by a firmware plane: one of the
	// native mode's size, allocated once and kept (see reserve_mode_buffer())
	area_id			mode_area;
	addr_t			mode_frame_buffer;
	phys_addr_t		mode_physical;

	uint32			display_id;

	// the hardware cursor: a firmware plane over everything
	area_id			cursor_area;
	uint32*			cursor_bits;
	phys_addr_t		cursor_physical;
	uint16			cursor_width;
	uint16			cursor_height;
	uint16			cursor_hot_x;
	uint16			cursor_hot_y;
	uint16			cursor_x;
	uint16			cursor_y;
	bool			cursor_visible;

	// the boot loader's frame buffer, when the desktop is plane 0 (keyed)
	phys_addr_t		boot_physical;

	struct overlay_memory {
		area_id		area;		// < 0: unused slot
		void*		address;
		phys_addr_t	physical;
		size_t		size;
		bool		reserved;	// kept for the next allocation when freed
		bool		in_use;
	} overlay_buffers[kMaxOverlayBuffers];

	struct overlay_plane {
		bool		visible;
		phys_addr_t	physical;
		size_t		size;		// of what it scans out from physical

		// statistics of the plane's updates since the last report
		uint32		updates;
		int64		first_vblank;
		int64		last_vblank;
		uint32		late;
		uint32		gaps;
		uint32		superseded;
		bigtime_t	last_report;
	} overlays[BCM2711_DISPLAY_OVERLAY_PLANES];
	uint32			visible_overlays;

	bigtime_t		vblank_period;
		// measured, in microseconds

	volatile uint32* smi;
	area_id			smi_area;
	int32			smi_interrupt;
	bool			interrupt_installed;

	thread_id		vsync_thread;
	bool			quit;
};


static const char* sDeviceNames[] = { DEVICE_NAME, NULL };
static display_info sInfo;
static mutex sLock = MUTEX_INITIALIZER("bcm2711 display");
static bcm2835_mailbox_module_info* sMailbox;


//	#pragma mark - device tree


static uint64
read_cells(const uint32* cells, uint32 count)
{
	uint64 value = 0;
	for (uint32 i = 0; i < count; i++)
		value = (value << 32) | fdt32_to_cpu(cells[i]);
	return value;
}


static uint32
address_cells(int node)
{
	const uint32* prop = (const uint32*)fdt_getprop(gFDT, node,
		"#address-cells", NULL);
	return prop != NULL ? fdt32_to_cpu(*prop) : 2;
}


static uint32
size_cells(int node)
{
	const uint32* prop = (const uint32*)fdt_getprop(gFDT, node, "#size-cells",
		NULL);
	return prop != NULL ? fdt32_to_cpu(*prop) : 1;
}


/*!	Translates \a address with the first matching entry of the "ranges" of
	\a bus.
*/
static bool
apply_ranges(int bus, uint64& address)
{
	int length;
	const uint32* ranges = (const uint32*)fdt_getprop(gFDT, bus, "ranges",
		&length);
	if (ranges == NULL || length == 0)
		return true;

	uint32 childCells = address_cells(bus);
	uint32 parentCells = address_cells(fdt_parent_offset(gFDT, bus));
	uint32 entryCells = childCells + parentCells + size_cells(bus);
	for (uint32 i = 0; (i + 1) * entryCells * 4 <= (uint32)length; i++) {
		const uint32* entry = ranges + i * entryCells;
		uint64 child = read_cells(entry, childCells);
		uint64 parent = read_cells(entry + childCells, parentCells);
		uint64 size = read_cells(entry + childCells + parentCells,
			size_cells(bus));
		if (address >= child && address - child < size) {
			address = address - child + parent;
			return true;
		}
	}
	return false;
}


/*!	The SMI block: its registers' physical address and its interrupt. */
static status_t
find_smi(phys_addr_t& registers, int32& interrupt)
{
	int node = fdt_node_offset_by_compatible(gFDT, -1, "brcm,bcm2835-smi");
	if (node < 0)
		return B_DEVICE_NOT_FOUND;

	int bus = fdt_parent_offset(gFDT, node);
	const uint32* reg = (const uint32*)fdt_getprop(gFDT, node, "reg", NULL);
	int length;
	const uint32* interrupts = (const uint32*)fdt_getprop(gFDT, node,
		"interrupts", &length);
	if (bus < 0 || reg == NULL || interrupts == NULL
		|| length < 3 * (int)sizeof(uint32)) {
		return B_BAD_DATA;
	}

	uint64 address = read_cells(reg, address_cells(bus));
	for (int parent = bus; fdt_parent_offset(gFDT, parent) >= 0;
			parent = fdt_parent_offset(gFDT, parent)) {
		if (!apply_ranges(parent, address))
			return B_BAD_DATA;
	}
	registers = address;

	// GIC: <type number flags>, type 0 = SPI (from 32), 1 = PPI (from 16)
	interrupt = fdt32_to_cpu(interrupts[1])
		+ (fdt32_to_cpu(interrupts[0]) == 0 ? 32 : 16);
	return B_OK;
}


//	#pragma mark - vertical blank


static void
vertical_blank(display_info& info)
{
	const bigtime_t now = system_time();
	const bigtime_t period = now - info.shared_info->vblank_time;
	if (period > 5000 && period < 50000)
		info.vblank_period = (info.vblank_period * 7 + period) / 8;
	info.shared_info->vblank_time = now;
	atomic_add64(&info.shared_info->vblank_count, 1);

	int32 count;
	if (get_sem_count(info.shared_info->vblank_semaphore, &count) == B_OK
		&& count < 0) {
		release_sem_etc(info.shared_info->vblank_semaphore, -count,
			B_DO_NOT_RESCHEDULE);
	}
}


static int32
smi_interrupt(void* data)
{
	display_info& info = *(display_info*)data;
	if ((info.smi[SMI_CS] & kSmiInterrupts) == 0)
		return B_UNHANDLED_INTERRUPT;
	info.smi[SMI_CS] = 0;

	// An older firmware signals one event for all displays; a newer one flags
	// each display and wants it acknowledged
	bool blank = true;
	uint32 status = info.smi[SMI_DSW0];
	if ((status & 0xffff0000) == kSmiDisplayTag) {
		blank = false;
		if ((status & 1) != 0) {
			info.smi[SMI_DSW0] = kSmiDisplayTag;
			blank = true;
		}
		status = info.smi[SMI_DSW1];
		if ((status & 1) != 0) {
			info.smi[SMI_DSW1] = kSmiDisplayTag;
			blank = true;
		}
	}

	if (blank)
		vertical_blank(info);
	return B_INVOKE_SCHEDULER;
}


static status_t
vsync_thread(void* data)
{
	display_info& info = *(display_info*)data;
	while (!info.quit) {
		uint32 value = 0;
		if (sMailbox->property_tag(BCM2835_TAG_SET_VSYNC, &value,
				sizeof(value), sizeof(value), NULL) != B_OK) {
			snooze(16667);
		}
		vertical_blank(info);
	}
	return B_OK;
}


/*!	Takes the vertical blanks from the SMI interrupt if the firmware raises it,
	and from a thread waiting on SET_VSYNC otherwise.
*/
static void
start_vertical_blank(display_info& info)
{
	phys_addr_t registers;
	int32 interrupt;
	status_t status = find_smi(registers, interrupt);
	if (status == B_OK) {
		void* address;
		info.smi_area = map_physical_memory("bcm2711 display smi",
			registers & ~(phys_addr_t)(B_PAGE_SIZE - 1), B_PAGE_SIZE,
			B_ANY_KERNEL_ADDRESS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
			&address);
		status = info.smi_area >= 0 ? B_OK : info.smi_area;
		if (status == B_OK) {
			info.smi = (volatile uint32*)((addr_t)address
				+ (registers & (B_PAGE_SIZE - 1)));
			info.smi[SMI_CS] = 0;
			info.smi_interrupt = interrupt;
			status = install_io_interrupt_handler(interrupt, &smi_interrupt,
				&info, 0);
			info.interrupt_installed = status == B_OK;
		}
	}

	if (status == B_OK) {
		// three vertical blanks show that the interrupt works: ~50 ms at
		// 60 Hz; give up after kVblankProbeTime
		bigtime_t deadline = system_time() + kVblankProbeTime;
		int64 count = 0;
		while ((count = atomic_get64(&info.shared_info->vblank_count)) < 3
			&& system_time() < deadline) {
			snooze(5000);
		}
		if (count >= 3) {
			info.shared_info->vblank_from_interrupt = true;
			TRACE_ALWAYS("vertical blank: SMI interrupt %" B_PRId32 ", %"
				B_PRId64 " within %" B_PRId64 " ms\n", interrupt, count,
				kVblankProbeTime / 1000);
			return;
		}
		remove_io_interrupt_handler(interrupt, &smi_interrupt, &info);
		info.interrupt_installed = false;
		TRACE_ALWAYS("vertical blank: no SMI interrupt (%" B_PRId64 " in %"
			B_PRId64 " ms)\n", count, kVblankProbeTime / 1000);
	} else
		TRACE_ALWAYS("vertical blank: no SMI block: %s\n", strerror(status));

	info.quit = false;
	info.vsync_thread = spawn_kernel_thread(&vsync_thread,
		"bcm2711 display vsync", B_URGENT_DISPLAY_PRIORITY, &info);
	if (info.vsync_thread >= 0) {
		resume_thread(info.vsync_thread);
		TRACE_ALWAYS("vertical blank: firmware SET_VSYNC\n");
	}
}


static void
stop_vertical_blank(display_info& info)
{
	if (info.interrupt_installed) {
		remove_io_interrupt_handler(info.smi_interrupt, &smi_interrupt, &info);
		info.interrupt_installed = false;
	}
	if (info.vsync_thread >= 0) {
		info.quit = true;
		status_t result;
		wait_for_thread(info.vsync_thread, &result);
		info.vsync_thread = -1;
	}
	if (info.smi_area >= 0) {
		delete_area(info.smi_area);
		info.smi_area = -1;
	}
}


//	#pragma mark - display


static uint32
color_space_for_depth(uint32 depth)
{
	switch (depth) {
		case 8:
			return B_CMAP8;
		case 15:
			return B_RGB15;
		case 16:
			return B_RGB16;
		case 24:
			return B_RGB24;
		case 32:
			return B_RGB32;
	}
	return 0;
}


static void
read_edid(display_info& info)
{
	// block number, status, then the block's 128 bytes
	uint32 values[2 + sizeof(edid1_raw) / 4] = { 0 };
	status_t status = sMailbox->property_tag(BCM2835_TAG_GET_EDID_BLOCK,
		values, sizeof(values), sizeof(uint32), NULL);
	if (status != B_OK || values[1] != 0) {
		TRACE_ALWAYS("no EDID from the firmware: %s, status %" B_PRIu32 "\n",
			strerror(status), values[1]);
		return;
	}
	memcpy(&info.shared_info->edid_raw, values + 2, sizeof(edid1_raw));
	info.shared_info->has_edid = true;
}


static status_t reserve_mode_buffer(display_info& info);
static void overlays_init(display_info& info, void* settings);
static void overlays_uninit(display_info& info);


static status_t
display_init(display_info& info)
{
	frame_buffer_boot_info* bufferInfo
		= (frame_buffer_boot_info*)get_boot_item(FRAME_BUFFER_BOOT_INFO, NULL);
	if (bufferInfo == NULL)
		return B_ERROR;

	status_t status = get_module(BCM2835_MAILBOX_MODULE_NAME,
		(module_info**)&sMailbox);
	if (status != B_OK)
		return status;

	info.shared_area = create_area("bcm2711 display shared info",
		(void**)&info.shared_info, B_ANY_KERNEL_ADDRESS,
		(sizeof(bcm2711_display_shared_info) + B_PAGE_SIZE - 1)
			& ~(B_PAGE_SIZE - 1), B_FULL_LOCK,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA | B_CLONEABLE_AREA);
	if (info.shared_area < 0) {
		put_module(BCM2835_MAILBOX_MODULE_NAME);
		return info.shared_area;
	}

	bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	memset(&sharedInfo, 0, sizeof(sharedInfo));
	sharedInfo.mode_list_area = -1;

	// The boot loader's mapping stays (the kernel debugger's console uses
	// it); this one is write-combining
	info.boot_area = map_physical_memory("bcm2711 display frame buffer",
		bufferInfo->physical_frame_buffer,
		bufferInfo->bytes_per_row * bufferInfo->height, B_ANY_KERNEL_ADDRESS,
		B_READ_AREA | B_WRITE_AREA, (void**)&info.boot_frame_buffer);
	if (info.boot_area < 0) {
		delete_area(info.shared_area);
		put_module(BCM2835_MAILBOX_MODULE_NAME);
		return info.boot_area;
	}
	vm_set_area_memory_type(info.boot_area, bufferInfo->physical_frame_buffer,
		B_WRITE_COMBINING_MEMORY);
	info.boot_physical = bufferInfo->physical_frame_buffer;
	info.boot_bytes_per_row = bufferInfo->bytes_per_row;
	info.depth = bufferInfo->depth;
	info.frame_buffer_area = info.boot_area;
	info.frame_buffer = info.boot_frame_buffer;
	info.mode_area = -1;
	info.cursor_area = -1;
	info.cursor_width = 0;
	info.cursor_visible = false;

	sharedInfo.bytes_per_row = bufferInfo->bytes_per_row;
	sharedInfo.native_width = bufferInfo->width;
	sharedInfo.native_height = bufferInfo->height;
	sharedInfo.current_mode.virtual_width = bufferInfo->width;
	sharedInfo.current_mode.virtual_height = bufferInfo->height;
	sharedInfo.current_mode.space = color_space_for_depth(bufferInfo->depth);

	// while memory is not yet fragmented (a failure is retried by set_mode())
	reserve_mode_buffer(info);

	void* settings = load_driver_settings("bcm2711_display");
	overlays_init(info, settings);
	unload_driver_settings(settings);

	// the firmware's id of the first display (HDMI 0)
	uint32 display = 0;
	if (sMailbox->property_tag(BCM2835_TAG_GET_DISPLAY_ID, &display,
			sizeof(display), sizeof(display), NULL) == B_OK) {
		info.display_id = display;
	}

	read_edid(info);

	sharedInfo.vblank_semaphore = create_sem(0, "bcm2711 display vblank");
	info.smi_area = -1;
	info.vsync_thread = -1;
	if (sharedInfo.vblank_semaphore >= 0) {
		// Userland may not acquire a kernel semaphore: it belongs to the
		// team that opened the device (app_server), as intel_extreme's
		set_sem_owner(sharedInfo.vblank_semaphore, team_get_current_team_id());
		start_vertical_blank(info);
	}

	TRACE_ALWAYS("%" B_PRIu32 "x%" B_PRIu32 ", %" B_PRIu32 " bits, frame buffer "
		"%#" B_PRIxPHYSADDR ", EDID %s\n", bufferInfo->width, bufferInfo->height,
		(uint32)bufferInfo->depth, bufferInfo->physical_frame_buffer,
		sharedInfo.has_edid ? "yes" : "no");
	return B_OK;
}


static status_t update_cursor(display_info& info);
static void blank_firmware_frame_buffer(bool blank);


/*!	Where the current mode appears on the native one: scaled up as far as it
	fits, centered (black borders).
*/
static void
mode_on_screen(const display_info& info, int32& offsetX, int32& offsetY,
	int32& width, int32& height)
{
	const bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	const int32 modeWidth = sharedInfo.current_mode.virtual_width;
	const int32 modeHeight = sharedInfo.current_mode.virtual_height;
	const int32 nativeWidth = sharedInfo.native_width;
	const int32 nativeHeight = sharedInfo.native_height;

	width = nativeWidth;
	height = (int64)modeHeight * nativeWidth / modeWidth;
	if (height > nativeHeight) {
		height = nativeHeight;
		width = (int64)modeWidth * nativeHeight / modeHeight;
	}
	offsetX = (nativeWidth - width) / 2;
	offsetY = (nativeHeight - height) / 2;
}


static bool
desktop_keyed(const display_info& info)
{
	return info.visible_overlays > 0 && info.shared_info->overlays_keyed;
}


/*!	Scans the desktop out as the mode and the overlays need it: the native
	mode without visible overlays is the firmware's own frame buffer; a smaller
	mode is the mode buffer on plane 0, scaled up; with an overlay under it,
	the desktop is plane 0 with per-pixel alpha (in the native mode, the boot
	loader's frame buffer, the firmware's own blanked).
*/
static status_t
update_primary_plane(display_info& info)
{
	const bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	const uint32 width = sharedInfo.current_mode.virtual_width;
	const uint32 height = sharedInfo.current_mode.virtual_height;
	const bool native = width == sharedInfo.native_width
		&& height == sharedInfo.native_height;
	const bool keyed = desktop_keyed(info);

	firmware_plane plane = {};
	plane.display = info.display_id;
	plane.plane_id = kPrimaryPlane;
	if (native && !keyed) {
		// only display and plane: unsets the plane
		status_t status = sMailbox->property_tag(BCM2835_TAG_SET_PLANE, &plane,
			sizeof(plane), sizeof(plane), NULL);
		blank_firmware_frame_buffer(false);
		return status;
	}

	int32 dstX, dstY, dstWidth, dstHeight;
	mode_on_screen(info, dstX, dstY, dstWidth, dstHeight);

	plane.image_type = keyed ? kImageARGB8888 : kImageXRGB8888;
	plane.layer = keyed ? kKeyedDesktopLayer : kPrimaryLayer;
	plane.width = width;
	plane.height = height;
	plane.pitch = native ? info.boot_bytes_per_row : width * 4;
	plane.vpitch = height;
	plane.src_w = width << 16;
	plane.src_h = height << 16;
	plane.dst_x = dstX;
	plane.dst_y = dstY;
	plane.dst_w = dstWidth;
	plane.dst_h = dstHeight;
	plane.alpha = 0xff;
	plane.num_planes = 1;
	plane.planes[0] = native ? info.boot_physical : info.mode_physical;

	status_t status = sMailbox->property_tag(BCM2835_TAG_SET_PLANE, &plane,
		sizeof(plane), sizeof(plane), NULL);
	if (status == B_OK)
		blank_firmware_frame_buffer(true);
	return status;
}


static void
blank_firmware_frame_buffer(bool blank)
{
	uint32 value = blank ? 1 : 0;
	sMailbox->property_tag(BCM2835_TAG_FRAMEBUFFER_BLANK, &value,
		sizeof(value), sizeof(value), NULL);
}


/*!	Allocates memory the firmware scans out: physically contiguous below
	kScanOutLimit, mapped write-combining. The pages were cleared through a
	cacheable mapping: their lines are written back and dropped first, so that
	the firmware never scans stale memory and no dirty line is evicted later
	over what was written through the write-combining mapping.
*/
static area_id
create_scan_out_area(const char* name, size_t size, uint32 protection,
	void** _address, phys_addr_t* _physical)
{
	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = kScanOutLimit;
	area_id area = create_area_etc(B_SYSTEM_TEAM, name, size, B_CONTIGUOUS,
		protection, 0, 0, &virtualRestrictions, &physicalRestrictions,
		_address);
	if (area < 0)
		return area;

	physical_entry entry;
	status_t status = get_memory_map(*_address, B_PAGE_SIZE, &entry, 1);
	if (status == B_OK) {
		arm64_dcache_flush_range((addr_t)*_address, size);
		status = vm_set_area_memory_type(area, entry.address,
			B_WRITE_COMBINING_MEMORY);
	}
	if (status != B_OK) {
		delete_area(area);
		return status;
	}

	*_physical = entry.address;
	return area;
}


/*!	Allocates the frame buffer of the smaller modes: the native mode's size,
	so that every smaller mode fits. It is allocated once, early, and kept: a
	contiguous run of a few MB in the first GB is often no longer to be had
	after the system has run for a while (a game's mode change then failed
	with B_NO_MEMORY: KNOWN-ISSUES W15).
*/
static status_t
reserve_mode_buffer(display_info& info)
{
	if (info.mode_area >= 0)
		return B_OK;

	bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	const size_t size = (sharedInfo.native_width * 4 * sharedInfo.native_height
		+ B_PAGE_SIZE - 1) & ~(B_PAGE_SIZE - 1);
	void* address;
	area_id area = create_scan_out_area("bcm2711 display mode frame buffer",
		size, B_READ_AREA | B_WRITE_AREA, &address, &info.mode_physical);
	if (area < 0)
		return area;

	info.mode_area = area;
	info.mode_frame_buffer = (addr_t)address;
	return B_OK;
}


//	#pragma mark - overlays


/*!	Hands out an overlay buffer of at least \a size bytes: a reserved one
	that fits, or a new one. Either is a kernel area that every team may
	access: the overlay API gives the clients its address as it is.
*/
static status_t
allocate_overlay_buffer(display_info& info,
	bcm2711_display_overlay_buffer& request)
{
	if (info.shared_info->overlay_planes == 0)
		return B_NOT_SUPPORTED;
	const size_t size = ROUNDUP((size_t)request.size, B_PAGE_SIZE);
	if (size == 0 || size > kMaxOverlayBufferSize)
		return B_BAD_VALUE;

	display_info::overlay_memory* buffer = NULL;
	for (uint32 i = 0; i < kMaxOverlayBuffers; i++) {
		display_info::overlay_memory& slot = info.overlay_buffers[i];
		if (slot.area >= 0 && !slot.in_use && slot.size >= size
			&& (buffer == NULL || slot.size < buffer->size)) {
			buffer = &slot;
		}
	}
	if (buffer != NULL) {
		// what its last user left in it
		memset(buffer->address, 0, size);
	} else {
		for (uint32 i = 0; i < kMaxOverlayBuffers; i++) {
			if (info.overlay_buffers[i].area < 0) {
				buffer = &info.overlay_buffers[i];
				break;
			}
		}
		if (buffer == NULL)
			return B_NO_MEMORY;

		buffer->area = create_scan_out_area("bcm2711 display overlay buffer",
			size, B_READ_AREA | B_WRITE_AREA | B_CLONEABLE_AREA,
			&buffer->address, &buffer->physical);
		if (buffer->area < 0) {
			status_t status = buffer->area;
			buffer->area = -1;
			return status;
		}
		buffer->size = size;
		buffer->reserved = false;
	}

	buffer->in_use = true;
	request.area = buffer->area;
	request.address = buffer->address;
	request.physical = buffer->physical;
	return B_OK;
}


static display_info::overlay_memory*
find_overlay_buffer(display_info& info, phys_addr_t physical, size_t size)
{
	for (uint32 i = 0; i < kMaxOverlayBuffers; i++) {
		display_info::overlay_memory& buffer = info.overlay_buffers[i];
		if (buffer.area >= 0 && buffer.in_use && physical >= buffer.physical
			&& physical - buffer.physical <= buffer.size
			&& size <= buffer.size - (physical - buffer.physical)) {
			return &buffer;
		}
	}
	return NULL;
}


static void
report_overlay(display_info& info, uint32 index)
{
	display_info::overlay_plane& overlay = info.overlays[index];
	if (overlay.updates > 0) {
		TRACE_ALWAYS("plane %" B_PRIu32 ": %" B_PRIu32 " updates in %" B_PRId64
			" vblanks, %" B_PRIu32 " late, %" B_PRIu32 " gaps, %" B_PRIu32
			" superseded (vblank %" B_PRId64 " us)\n",
			index + BCM2711_DISPLAY_FIRST_OVERLAY_PLANE, overlay.updates,
			overlay.last_vblank - overlay.first_vblank + 1, overlay.late,
			overlay.gaps, overlay.superseded, info.vblank_period);
	}
	overlay.updates = 0;
	overlay.late = 0;
	overlay.gaps = 0;
	overlay.superseded = 0;
	overlay.last_report = system_time();
}


/*!	Counts a plane update (after SET_PLANE returned) against the vertical
	blanks: one per blank is a flip on time; an update shortly before the
	predicted blank may be latched a frame late; blanks without an update
	in a run of them are missed flips (when the client meant to flip every
	frame); several in one blank are superseded flips (unsynchronized).
*/
static void
count_overlay_update(display_info& info, uint32 index)
{
	display_info::overlay_plane& overlay = info.overlays[index];
	const bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	const int64 vblank = atomic_get64(&info.shared_info->vblank_count);
	const bigtime_t now = system_time();

	const bigtime_t sinceBlank = now - sharedInfo.vblank_time;
	if (sinceBlank < info.vblank_period
		&& info.vblank_period - sinceBlank < kLateUpdateMargin) {
		overlay.late++;
	}

	if (overlay.updates == 0)
		overlay.first_vblank = vblank;
	else if (vblank == overlay.last_vblank)
		overlay.superseded++;
	else if (vblank - overlay.last_vblank > 1
		&& vblank - overlay.last_vblank <= 30) {
		// (a longer pause is not a run of flips)
		overlay.gaps += vblank - overlay.last_vblank - 1;
	}
	overlay.last_vblank = vblank;
	overlay.updates++;

	if (now - overlay.last_report >= kOverlayReportInterval)
		report_overlay(info, index);
}


static status_t
hide_overlay(display_info& info, uint32 plane)
{
	if (plane < BCM2711_DISPLAY_FIRST_OVERLAY_PLANE
		|| plane >= BCM2711_DISPLAY_FIRST_OVERLAY_PLANE
			+ BCM2711_DISPLAY_OVERLAY_PLANES) {
		return B_BAD_VALUE;
	}
	const uint32 index = plane - BCM2711_DISPLAY_FIRST_OVERLAY_PLANE;
	display_info::overlay_plane& overlay = info.overlays[index];
	if (!overlay.visible)
		return B_OK;

	firmware_plane request = {};
	request.display = info.display_id;
	request.plane_id = plane;
	status_t status = sMailbox->property_tag(BCM2835_TAG_SET_PLANE, &request,
		sizeof(request), sizeof(request), NULL);

	overlay.visible = false;
	report_overlay(info, index);
	if (--info.visible_overlays == 0 && info.shared_info->overlays_keyed) {
		// the desktop is opaque again
		update_primary_plane(info);
	}
	return status;
}


static uint32
overlay_extent(const bcm2711_display_overlay& request)
{
	const uint32 size = request.bytes_per_row * request.height;
	if (request.image_type == BCM2711_DISPLAY_IMAGE_YUV420)
		return size + size / 2;
	return size;
}


/*!	Shows a buffer on an overlay plane: the destination rectangle (the
	mode's coordinates, unclipped) is clipped to the mode, with the source
	cropped in proportion, then scaled to the native coordinates the
	firmware works in.
*/
static status_t
set_overlay(display_info& info, const bcm2711_display_overlay& request)
{
	if (request.plane < BCM2711_DISPLAY_FIRST_OVERLAY_PLANE
		|| request.plane >= BCM2711_DISPLAY_FIRST_OVERLAY_PLANE
			+ BCM2711_DISPLAY_OVERLAY_PLANES
		|| info.shared_info->overlay_planes == 0) {
		return B_BAD_VALUE;
	}
	switch (request.image_type) {
		case BCM2711_DISPLAY_IMAGE_XRGB8888:
		case BCM2711_DISPLAY_IMAGE_YUV422_YUYV:
		case BCM2711_DISPLAY_IMAGE_YUV420:
			break;
		default:
			return B_BAD_VALUE;
	}
	if (request.width == 0 || request.height == 0
		|| request.bytes_per_row > UINT16_MAX
		|| request.source_width == 0 || request.source_height == 0
		|| request.source_x + request.source_width
			> ((uint32)request.width << 16)
		|| request.source_y + request.source_height
			> ((uint32)request.height << 16)
		|| request.destination_width == 0
		|| request.destination_height == 0) {
		return B_BAD_VALUE;
	}
	// only what we handed out scans out
	const uint32 extent = overlay_extent(request);
	if (find_overlay_buffer(info, request.physical, extent) == NULL)
		return B_BAD_ADDRESS;

	const uint32 index = request.plane - BCM2711_DISPLAY_FIRST_OVERLAY_PLANE;
	display_info::overlay_plane& overlay = info.overlays[index];

	// clip the destination to the mode, crop the source in proportion
	const bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	const int64 modeWidth = sharedInfo.current_mode.virtual_width;
	const int64 modeHeight = sharedInfo.current_mode.virtual_height;
	int64 left = request.x;
	int64 top = request.y;
	int64 right = left + request.destination_width;
	int64 bottom = top + request.destination_height;
	int64 srcX = request.source_x;
	int64 srcY = request.source_y;
	int64 srcWidth = request.source_width;
	int64 srcHeight = request.source_height;
	if (left < 0) {
		const int64 crop = -left * srcWidth / request.destination_width;
		srcX += crop;
		srcWidth -= crop;
		left = 0;
	}
	if (top < 0) {
		const int64 crop = -top * srcHeight / request.destination_height;
		srcY += crop;
		srcHeight -= crop;
		top = 0;
	}
	if (right > modeWidth) {
		srcWidth -= (right - modeWidth) * request.source_width
			/ request.destination_width;
		right = modeWidth;
	}
	if (bottom > modeHeight) {
		srcHeight -= (bottom - modeHeight) * request.source_height
			/ request.destination_height;
		bottom = modeHeight;
	}
	if (right <= left || bottom <= top || srcWidth <= 0 || srcHeight <= 0)
		return hide_overlay(info, request.plane);

	// the keyed desktop goes up before the overlay shows under it
	const bool shown = !overlay.visible;
	if (shown) {
		overlay.visible = true;
		overlay.last_report = system_time();
		if (info.visible_overlays++ == 0 && sharedInfo.overlays_keyed)
			update_primary_plane(info);
	}

	int32 offsetX, offsetY, screenWidth, screenHeight;
	mode_on_screen(info, offsetX, offsetY, screenWidth, screenHeight);

	firmware_plane plane = {};
	plane.display = info.display_id;
	plane.plane_id = request.plane;
	plane.image_type = request.image_type;
	plane.layer = (sharedInfo.overlays_keyed ? kOverlayLayer
		: kOverlayAboveLayer) + request.plane;
	plane.width = request.width;
	plane.height = request.height;
	plane.pitch = request.bytes_per_row;
	plane.vpitch = request.height;
	plane.src_x = srcX;
	plane.src_y = srcY;
	plane.src_w = srcWidth;
	plane.src_h = srcHeight;
	plane.dst_x = offsetX + left * screenWidth / modeWidth;
	plane.dst_y = offsetY + top * screenHeight / modeHeight;
	plane.dst_w = max_c((right - left) * screenWidth / modeWidth, 1);
	plane.dst_h = max_c((bottom - top) * screenHeight / modeHeight, 1);
	plane.alpha = 0xff;
	plane.color_encoding = request.color_encoding;
	plane.planes[0] = request.physical;
	if (request.image_type == BCM2711_DISPLAY_IMAGE_YUV420) {
		// Y, then U and V at half the pitch and height (the firmware has
		// one pitch: it derives theirs)
		const uint32 lumaSize = request.bytes_per_row * request.height;
		plane.num_planes = 3;
		plane.planes[1] = request.physical + lumaSize;
		plane.planes[2] = plane.planes[1] + lumaSize / 4;
	} else
		plane.num_planes = 1;

	status_t status = sMailbox->property_tag(BCM2835_TAG_SET_PLANE, &plane,
		sizeof(plane), sizeof(plane), NULL);
	if (status != B_OK) {
		if (shown) {
			overlay.visible = false;
			if (--info.visible_overlays == 0 && sharedInfo.overlays_keyed)
				update_primary_plane(info);
		}
		return status;
	}

	overlay.physical = request.physical;
	overlay.size = extent;
	count_overlay_update(info, index);
	return B_OK;
}


static status_t
free_overlay_buffer(display_info& info, area_id area)
{
	for (uint32 i = 0; i < kMaxOverlayBuffers; i++) {
		display_info::overlay_memory& buffer = info.overlay_buffers[i];
		if (buffer.area != area || !buffer.in_use)
			continue;

		// never scan out what is freed
		for (uint32 j = 0; j < BCM2711_DISPLAY_OVERLAY_PLANES; j++) {
			const display_info::overlay_plane& overlay = info.overlays[j];
			if (overlay.visible && overlay.physical >= buffer.physical
				&& overlay.physical < buffer.physical + buffer.size) {
				hide_overlay(info, j + BCM2711_DISPLAY_FIRST_OVERLAY_PLANE);
			}
		}

		buffer.in_use = false;
		if (!buffer.reserved) {
			delete_area(buffer.area);
			buffer.area = -1;
		}
		return B_OK;
	}
	return B_BAD_VALUE;
}


/*!	Overlays need every plane buffer below kScanOutLimit, and for the keyed
	desktop in the native mode the boot loader's frame buffer too. The
	reserved buffers are allocated while memory is not yet fragmented.
*/
static void
overlays_init(display_info& info, void* settings)
{
	bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	for (uint32 i = 0; i < kMaxOverlayBuffers; i++)
		info.overlay_buffers[i].area = -1;
	info.vblank_period = 16667;

	const bool above = get_driver_boolean_parameter(settings,
		"overlay_above_desktop", false, true);
	if (info.depth != 32 || (!above && (info.boot_physical >= kScanOutLimit
			|| info.boot_physical + (phys_addr_t)info.boot_bytes_per_row
				* sharedInfo.native_height > kScanOutLimit))) {
		TRACE_ALWAYS("no overlays: the frame buffer is not below 1 GiB\n");
		return;
	}
	sharedInfo.overlay_planes = BCM2711_DISPLAY_OVERLAY_PLANES;
	sharedInfo.overlays_keyed = !above;

	const char* value = get_driver_parameter(settings, "overlay_reserve",
		NULL, NULL);
	uint32 reserve = value != NULL ? strtoul(value, NULL, 0)
		: kDefaultOverlayReserve;
	reserve = min_c(reserve, kMaxOverlayBuffers / 2);
	const size_t size = ROUNDUP((size_t)sharedInfo.native_width * 4
		* sharedInfo.native_height, B_PAGE_SIZE);
	uint32 reserved = 0;
	for (; reserved < reserve; reserved++) {
		display_info::overlay_memory& buffer = info.overlay_buffers[reserved];
		buffer.area = create_scan_out_area("bcm2711 display overlay buffer",
			size, B_READ_AREA | B_WRITE_AREA | B_CLONEABLE_AREA,
			&buffer.address, &buffer.physical);
		if (buffer.area < 0) {
			buffer.area = -1;
			break;
		}
		buffer.size = size;
		buffer.reserved = true;
		buffer.in_use = false;
	}

	TRACE_ALWAYS("overlays: planes %d-%d %s the desktop, %" B_PRIu32
		" buffers of %" B_PRIuSIZE " KiB reserved\n",
		BCM2711_DISPLAY_FIRST_OVERLAY_PLANE,
		BCM2711_DISPLAY_FIRST_OVERLAY_PLANE + BCM2711_DISPLAY_OVERLAY_PLANES
			- 1, above ? "above" : "under (keyed)", reserved, size / 1024);
}


static void
overlays_uninit(display_info& info)
{
	for (uint32 plane = BCM2711_DISPLAY_FIRST_OVERLAY_PLANE;
			plane < BCM2711_DISPLAY_FIRST_OVERLAY_PLANE
				+ BCM2711_DISPLAY_OVERLAY_PLANES; plane++) {
		hide_overlay(info, plane);
	}
	for (uint32 i = 0; i < kMaxOverlayBuffers; i++) {
		display_info::overlay_memory& buffer = info.overlay_buffers[i];
		if (buffer.area >= 0) {
			delete_area(buffer.area);
			buffer.area = -1;
		}
	}
}


/*!	Switches to \a width x \a height: the native mode is the firmware's own
	frame buffer, a smaller one the mode buffer that a firmware plane scales
	up. The accelerant's clone of the old buffer becomes a null area.
*/
static status_t
set_mode(display_info& info, uint32 width, uint32 height)
{
	bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	if (width == sharedInfo.current_mode.virtual_width
		&& height == sharedInfo.current_mode.virtual_height) {
		return B_OK;
	}
	if (info.depth != 32 || width < 320 || height < 200
		|| width > sharedInfo.native_width
		|| height > sharedInfo.native_height) {
		return B_BAD_VALUE;
	}

	const bool native = width == sharedInfo.native_width
		&& height == sharedInfo.native_height;
	area_id oldArea = info.frame_buffer_area;

	if (!native) {
		status_t status = reserve_mode_buffer(info);
		if (status != B_OK)
			return status;
		memset((void*)info.mode_frame_buffer, 0, width * 4 * height);
	}

	const uint32 oldWidth = sharedInfo.current_mode.virtual_width;
	const uint32 oldHeight = sharedInfo.current_mode.virtual_height;
	sharedInfo.current_mode.virtual_width = width;
	sharedInfo.current_mode.virtual_height = height;
	status_t status = update_primary_plane(info);
	if (status != B_OK) {
		sharedInfo.current_mode.virtual_width = oldWidth;
		sharedInfo.current_mode.virtual_height = oldHeight;
		update_primary_plane(info);
		return status;
	}

	if (native) {
		info.frame_buffer_area = info.boot_area;
		info.frame_buffer = info.boot_frame_buffer;
		sharedInfo.bytes_per_row = info.boot_bytes_per_row;
	} else {
		info.frame_buffer_area = info.mode_area;
		info.frame_buffer = info.mode_frame_buffer;
		sharedInfo.bytes_per_row = width * 4;
	}

	// the kernel debugger's console follows
	frame_buffer_update(info.frame_buffer, width, height, info.depth,
		sharedInfo.bytes_per_row);

	// the clones of the old buffer (the accelerant clones the new one): of the
	// mode buffer too when only the size changed, as its layout did
	vm_change_clones_to_null_areas(oldArea);

	if (info.cursor_visible)
		update_cursor(info);
	TRACE_ALWAYS("mode %" B_PRIu32 "x%" B_PRIu32 "%s\n", width, height,
		native ? " (native)" : ", scaled by a firmware plane");
	return B_OK;
}


/*!	Puts the cursor plane where the cursor is: the mode's coordinates scaled
	to the native ones, clipped to the screen; hidden, the plane is unset.
*/
static status_t
update_cursor(display_info& info)
{
	firmware_plane plane = {};
	plane.display = info.display_id;
	plane.plane_id = kCursorPlane;

	const bcm2711_display_shared_info& sharedInfo = *info.shared_info;
	const int32 modeWidth = sharedInfo.current_mode.virtual_width;
	const int32 modeHeight = sharedInfo.current_mode.virtual_height;
	if (info.cursor_visible && info.cursor_width != 0 && modeWidth > 0) {
		// the mode as the primary plane shows it
		int32 offsetX, offsetY, dstWidth, dstHeight;
		mode_on_screen(info, offsetX, offsetY, dstWidth, dstHeight);

		// the cursor's rectangle in mode coordinates, clipped to the mode
		int32 left = (int32)info.cursor_x - info.cursor_hot_x;
		int32 top = (int32)info.cursor_y - info.cursor_hot_y;
		int32 srcX = 0, srcY = 0;
		int32 width = info.cursor_width, height = info.cursor_height;
		if (left < 0) {
			srcX = -left;
			width += left;
			left = 0;
		}
		if (top < 0) {
			srcY = -top;
			height += top;
			top = 0;
		}
		if (left + width > modeWidth)
			width = modeWidth - left;
		if (top + height > modeHeight)
			height = modeHeight - top;

		if (width > 0 && height > 0) {
			plane.image_type = kImageARGB8888;
			plane.layer = kCursorLayer;
			plane.width = info.cursor_width;
			plane.height = info.cursor_height;
			plane.pitch = BCM2711_DISPLAY_CURSOR_SIZE * 4;
			plane.vpitch = info.cursor_height;
			plane.src_x = srcX << 16;
			plane.src_y = srcY << 16;
			plane.src_w = width << 16;
			plane.src_h = height << 16;
			plane.dst_x = offsetX + (int64)left * dstWidth / modeWidth;
			plane.dst_y = offsetY + (int64)top * dstHeight / modeHeight;
			plane.dst_w = max_c((int64)width * dstWidth / modeWidth, 1);
			plane.dst_h = max_c((int64)height * dstHeight / modeHeight, 1);
			plane.alpha = 0xff;
			plane.num_planes = 1;
			plane.planes[0] = info.cursor_physical;
		}
	}

	return sMailbox->property_tag(BCM2835_TAG_SET_PLANE, &plane, sizeof(plane),
		sizeof(plane), NULL);
}


static status_t
set_cursor(display_info& info, const bcm2711_display_cursor& cursor)
{
	if (cursor.width == 0 || cursor.height == 0
		|| cursor.width > BCM2711_DISPLAY_CURSOR_SIZE
		|| cursor.height > BCM2711_DISPLAY_CURSOR_SIZE) {
		return B_BAD_VALUE;
	}

	if (info.cursor_area < 0) {
		info.cursor_area = create_scan_out_area("bcm2711 display cursor",
			sizeof(cursor.bits), B_READ_AREA | B_WRITE_AREA,
			(void**)&info.cursor_bits, &info.cursor_physical);
		if (info.cursor_area < 0)
			return info.cursor_area;
	}

	// rows of the full cursor size: an aligned pitch for the plane
	memset(info.cursor_bits, 0, sizeof(cursor.bits));
	for (uint32 y = 0; y < cursor.height; y++) {
		memcpy(info.cursor_bits + y * BCM2711_DISPLAY_CURSOR_SIZE,
			cursor.bits + y * cursor.width, cursor.width * 4);
	}
	info.cursor_width = cursor.width;
	info.cursor_height = cursor.height;
	info.cursor_hot_x = cursor.hot_x;
	info.cursor_hot_y = cursor.hot_y;
	return update_cursor(info);
}


static void
display_uninit(display_info& info)
{
	overlays_uninit(info);
	set_mode(info, info.shared_info->native_width,
		info.shared_info->native_height);
	if (info.cursor_area >= 0) {
		info.cursor_visible = false;
		update_cursor(info);
		delete_area(info.cursor_area);
		info.cursor_area = -1;
	}
	stop_vertical_blank(info);
	delete_sem(info.shared_info->vblank_semaphore);
	vm_change_clones_to_null_areas(info.boot_area);
	if (info.mode_area >= 0) {
		vm_change_clones_to_null_areas(info.mode_area);
		delete_area(info.mode_area);
		info.mode_area = -1;
	}
	delete_area(info.boot_area);
	delete_area(info.shared_area);
	put_module(BCM2835_MAILBOX_MODULE_NAME);
}


//	#pragma mark - device hooks


static status_t
device_open(const char* name, uint32 flags, void** _cookie)
{
	if (strcmp(name, DEVICE_NAME) != 0)
		return B_BAD_VALUE;

	MutexLocker locker(sLock);
	if (sInfo.open_count == 0) {
		status_t status = display_init(sInfo);
		if (status != B_OK)
			return status;
	}
	sInfo.open_count++;
	*_cookie = &sInfo;
	return B_OK;
}


static status_t
device_close(void* cookie)
{
	return B_OK;
}


static status_t
device_free(void* cookie)
{
	MutexLocker locker(sLock);
	if (--sInfo.open_count == 0)
		display_uninit(sInfo);
	return B_OK;
}


static status_t
device_ioctl(void* cookie, uint32 op, void* buffer, size_t length)
{
	display_info& info = *(display_info*)cookie;

	switch (op) {
		case B_GET_ACCELERANT_SIGNATURE:
			if (user_strlcpy((char*)buffer, BCM2711_DISPLAY_ACCELERANT_NAME,
					B_FILE_NAME_LENGTH) < B_OK) {
				return B_BAD_ADDRESS;
			}
			return B_OK;

		case BCM2711_DISPLAY_GET_PRIVATE_DATA:
			return user_memcpy(buffer, &info.shared_area, sizeof(area_id));

		case BCM2711_DISPLAY_CLONE_FRAME_BUFFER:
		{
			void* address;
			area_id area = vm_clone_area(B_CURRENT_TEAM,
				"bcm2711 display frame buffer", &address, B_ANY_ADDRESS,
				B_READ_AREA | B_WRITE_AREA, 0, info.frame_buffer_area, true);
			if (area < 0)
				return area;

			// A clone of the boot frame buffer (physical memory) keeps its
			// write-combining type, but a clone of a mode's frame buffer (RAM)
			// would be cacheable: the firmware would scan out stale memory.
			area_info source;
			physical_entry entry;
			if (get_area_info(info.frame_buffer_area, &source) == B_OK
				&& get_memory_map(source.address, B_PAGE_SIZE, &entry, 1)
					== B_OK) {
				vm_set_area_memory_type(area, entry.address,
					B_WRITE_COMBINING_MEMORY);
			}
			return _user_get_area_info(area, (area_info*)buffer);
		}

		case BCM2711_DISPLAY_SET_MODE:
		{
			bcm2711_display_mode mode;
			if (user_memcpy(&mode, buffer, sizeof(mode)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			return set_mode(info, mode.width, mode.height);
		}

		case BCM2711_DISPLAY_SET_CURSOR:
		{
			bcm2711_display_cursor* cursor
				= (bcm2711_display_cursor*)malloc(sizeof(*cursor));
			if (cursor == NULL)
				return B_NO_MEMORY;
			MemoryDeleter cursorDeleter(cursor);
			if (user_memcpy(cursor, buffer, sizeof(*cursor)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			return set_cursor(info, *cursor);
		}

		case BCM2711_DISPLAY_MOVE_CURSOR:
		{
			bcm2711_display_cursor_position position;
			if (user_memcpy(&position, buffer, sizeof(position)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			info.cursor_x = position.x;
			info.cursor_y = position.y;
			return info.cursor_visible ? update_cursor(info) : B_OK;
		}

		case BCM2711_DISPLAY_SHOW_CURSOR:
		{
			bool visible;
			if (user_memcpy(&visible, buffer, sizeof(visible)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			if (visible == info.cursor_visible)
				return B_OK;
			info.cursor_visible = visible;
			TRACE_ALWAYS("hardware cursor %s\n", visible ? "shown" : "hidden");
			return update_cursor(info);
		}

		case BCM2711_DISPLAY_ALLOCATE_OVERLAY_BUFFER:
		{
			bcm2711_display_overlay_buffer request;
			if (user_memcpy(&request, buffer, sizeof(request)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			status_t status = allocate_overlay_buffer(info, request);
			if (status != B_OK)
				return status;
			if (user_memcpy(buffer, &request, sizeof(request)) != B_OK) {
				free_overlay_buffer(info, request.area);
				return B_BAD_ADDRESS;
			}
			return B_OK;
		}

		case BCM2711_DISPLAY_FREE_OVERLAY_BUFFER:
		{
			area_id area;
			if (user_memcpy(&area, buffer, sizeof(area)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			return free_overlay_buffer(info, area);
		}

		case BCM2711_DISPLAY_SET_OVERLAY:
		{
			bcm2711_display_overlay request;
			if (user_memcpy(&request, buffer, sizeof(request)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			return set_overlay(info, request);
		}

		case BCM2711_DISPLAY_HIDE_OVERLAY:
		{
			uint32 plane;
			if (user_memcpy(&plane, buffer, sizeof(plane)) != B_OK)
				return B_BAD_ADDRESS;
			MutexLocker locker(sLock);
			return hide_overlay(info, plane);
		}

		case BCM2711_DISPLAY_GET_DEVICE_NAME:
			if (user_strlcpy((char*)buffer, DEVICE_NAME, B_PATH_NAME_LENGTH)
					< B_OK) {
				return B_BAD_ADDRESS;
			}
			return B_OK;
	}

	return B_DEV_INVALID_IOCTL;
}


static status_t
device_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static status_t
device_write(void* cookie, off_t position, const void* buffer,
	size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static device_hooks sDeviceHooks = {
	device_open,
	device_close,
	device_free,
	device_ioctl,
	device_read,
	device_write,
	NULL,
	NULL,
	NULL,
	NULL
};


//	#pragma mark - driver


extern "C" status_t
init_hardware(void)
{
	if (gFDT == NULL
		|| fdt_node_check_compatible(gFDT, 0, "brcm,bcm2711") != 0
		|| get_boot_item(FRAME_BUFFER_BOOT_INFO, NULL) == NULL) {
		return B_ERROR;
	}
	return B_OK;
}


extern "C" status_t
init_driver(void)
{
	return B_OK;
}


extern "C" void
uninit_driver(void)
{
}


extern "C" const char**
publish_devices(void)
{
	return sDeviceNames;
}


extern "C" device_hooks*
find_device(const char* name)
{
	return strcmp(name, DEVICE_NAME) == 0 ? &sDeviceHooks : NULL;
}
