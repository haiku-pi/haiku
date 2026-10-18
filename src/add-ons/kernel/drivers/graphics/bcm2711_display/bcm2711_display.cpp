/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The Raspberry Pi 4's display as its VideoCore firmware drives it: the
	framebuffer the boot loader set up, the monitor's EDID as the firmware read
	it, and a vertical blank semaphore. The firmware signals each vertical
	blank through the SMI block's interrupt (as Linux's vc4_firmware_kms
	expects); without it, a thread waits on the firmware's SET_VSYNC call.
*/


#include <bcm2711_display.h>
#include <bcm2835_mailbox.h>

#include <stdlib.h>
#include <string.h>

#include <boot_item.h>
#include <frame_buffer_console.h>
#include <graphic_driver.h>
#include <KernelExport.h>
#include <lock.h>
#include <util/AutoLock.h>
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
// VC_IMAGE_XRGB8888: B, G, R, X in memory, as B_RGB32
static const uint8 kImageXRGB8888 = 44;
// The firmware scans out from the low 1 GiB only (it keeps 30 address bits)
static const phys_addr_t kScanOutLimit = (phys_addr_t)1 << 30;


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

	// a smaller mode's frame buffer, scaled up by a firmware plane
	area_id			mode_area;
	addr_t			mode_frame_buffer;

	uint32			display_id;

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
		snooze(kVblankProbeTime);
		int64 count = atomic_get64(&info.shared_info->vblank_count);
		if (count >= 3) {
			info.shared_info->vblank_from_interrupt = true;
			TRACE_ALWAYS("vertical blank: SMI interrupt %" B_PRId32 ", %"
				B_PRId64 " in %" B_PRId64 " ms\n", interrupt, count,
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
	info.boot_bytes_per_row = bufferInfo->bytes_per_row;
	info.depth = bufferInfo->depth;
	info.frame_buffer_area = info.boot_area;
	info.frame_buffer = info.boot_frame_buffer;
	info.mode_area = -1;

	sharedInfo.bytes_per_row = bufferInfo->bytes_per_row;
	sharedInfo.native_width = bufferInfo->width;
	sharedInfo.native_height = bufferInfo->height;
	sharedInfo.current_mode.virtual_width = bufferInfo->width;
	sharedInfo.current_mode.virtual_height = bufferInfo->height;
	sharedInfo.current_mode.space = color_space_for_depth(bufferInfo->depth);

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


static status_t
set_plane(display_info& info, uint32 width, uint32 height, phys_addr_t buffer)
{
	firmware_plane plane = {};
	plane.display = info.display_id;
	plane.plane_id = kPrimaryPlane;
	if (width != 0) {
		const uint32 nativeWidth = info.shared_info->native_width;
		const uint32 nativeHeight = info.shared_info->native_height;

		// scaled up as far as it fits, centered (black borders)
		uint32 dstWidth = nativeWidth;
		uint32 dstHeight = (uint64)height * nativeWidth / width;
		if (dstHeight > nativeHeight) {
			dstHeight = nativeHeight;
			dstWidth = (uint64)width * nativeHeight / height;
		}

		plane.image_type = kImageXRGB8888;
		plane.layer = kPrimaryLayer;
		plane.width = width;
		plane.height = height;
		plane.pitch = width * 4;
		plane.vpitch = height;
		plane.src_w = width << 16;
		plane.src_h = height << 16;
		plane.dst_x = (nativeWidth - dstWidth) / 2;
		plane.dst_y = (nativeHeight - dstHeight) / 2;
		plane.dst_w = dstWidth;
		plane.dst_h = dstHeight;
		plane.alpha = 0xff;
		plane.num_planes = 1;
		plane.planes[0] = buffer;
	}
		// else: only display and plane: unsets the plane

	return sMailbox->property_tag(BCM2835_TAG_SET_PLANE, &plane, sizeof(plane),
		sizeof(plane), NULL);
}


static void
blank_firmware_frame_buffer(bool blank)
{
	uint32 value = blank ? 1 : 0;
	sMailbox->property_tag(BCM2835_TAG_FRAMEBUFFER_BLANK, &value,
		sizeof(value), sizeof(value), NULL);
}


/*!	Switches to \a width x \a height: the native mode is the firmware's own
	frame buffer, a smaller one gets a new buffer that a firmware plane scales
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
	area_id oldModeArea = info.mode_area;

	if (native) {
		set_plane(info, 0, 0, 0);
		blank_firmware_frame_buffer(false);
		info.mode_area = -1;
		info.frame_buffer_area = info.boot_area;
		info.frame_buffer = info.boot_frame_buffer;
		sharedInfo.bytes_per_row = info.boot_bytes_per_row;
	} else {
		const uint32 bytesPerRow = width * 4;
		const size_t size = (bytesPerRow * height + B_PAGE_SIZE - 1)
			& ~(B_PAGE_SIZE - 1);
		virtual_address_restrictions virtualRestrictions = {};
		virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
		physical_address_restrictions physicalRestrictions = {};
		physicalRestrictions.high_address = kScanOutLimit;
		void* address;
		area_id area = create_area_etc(B_SYSTEM_TEAM,
			"bcm2711 display mode frame buffer", size, B_CONTIGUOUS,
			B_READ_AREA | B_WRITE_AREA, 0, 0, &virtualRestrictions,
			&physicalRestrictions, &address);
		if (area < 0)
			return area;
		physical_entry entry;
		get_memory_map(address, B_PAGE_SIZE, &entry, 1);
		vm_set_area_memory_type(area, entry.address, B_WRITE_COMBINING_MEMORY);
		memset(address, 0, size);

		status_t status = set_plane(info, width, height, entry.address);
		if (status != B_OK) {
			delete_area(area);
			return status;
		}
		blank_firmware_frame_buffer(true);

		info.mode_area = area;
		info.mode_frame_buffer = (addr_t)address;
		info.frame_buffer_area = area;
		info.frame_buffer = (addr_t)address;
		sharedInfo.bytes_per_row = bytesPerRow;
	}

	// the kernel debugger's console follows
	frame_buffer_update(info.frame_buffer, width, height, info.depth,
		sharedInfo.bytes_per_row);

	if (oldModeArea >= 0) {
		vm_change_clones_to_null_areas(oldModeArea);
		delete_area(oldModeArea);
	} else
		vm_change_clones_to_null_areas(info.boot_area);

	sharedInfo.current_mode.virtual_width = width;
	sharedInfo.current_mode.virtual_height = height;
	TRACE_ALWAYS("mode %" B_PRIu32 "x%" B_PRIu32 "%s\n", width, height,
		native ? " (native)" : ", scaled by a firmware plane");
	return B_OK;
}


static void
display_uninit(display_info& info)
{
	set_mode(info, info.shared_info->native_width,
		info.shared_info->native_height);
	stop_vertical_blank(info);
	delete_sem(info.shared_info->vblank_semaphore);
	vm_change_clones_to_null_areas(info.boot_area);
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
