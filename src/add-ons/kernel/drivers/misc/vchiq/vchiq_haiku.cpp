/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	VCHIQ, the message queue interface to the Raspberry Pi's VideoCore
	firmware ("brcm,bcm2711-vchiq", "brcm,bcm2835-vchiq"): the services of the
	firmware (gencmd, MMAL's codecs, camera and ISP, audio...) are reached
	through it. The core is FreeBSD's port of Broadcom's code
	(interface/vchiq_arm, BSD-3-Clause); this file is the driver around it,
	in the place of FreeBSD's vchiq_kmod.c, and the Haiku side of
	vchiq_haiku.h.

	It publishes /dev/vchiq with the ioctls of Linux and FreeBSD
	(vchiq_ioctl.h), for Broadcom's userland libvchiq. Reading the device
	returns the driver's state, a few gencmd answers of the firmware and the
	core's state dump.

	Nothing happens while the system boots: the first open() allocates the
	shared memory (the "slots"), starts the core's threads and hands the
	slots to the firmware (mailbox property VCHIQ_INIT). That cannot be undone:
	the firmware keeps the slots' address, so the driver stays loaded and
	never frees them.

	The VideoCore rings the ARM through the doorbell register BELL0 (its
	interrupt); the ARM rings the VideoCore through BELL2. Memory the
	VideoCore reads or writes is below the end of the bus's DMA window and
	addressed through it (on the Pi 4: 0xc0000000 + physical, below 1 GiB),
	except the pages of bulk transfers on the BCM2711, which page lists give
	by physical address.
*/


#include "vchiq_haiku.h"

#include <new>

#include <ByteOrder.h>
#include <Drivers.h>
#include <arch/cpu.h>
#include <bus/FDT.h>
#include <Notifications.h>
#include <kernel.h>
#include <team.h>
#include <util/AutoLock.h>
#include <vm/vm.h>

#include <bcm2835_mailbox.h>

#include <sys/ioccom.h>
extern "C" {
#include "vchiq_ioctl.h"
}


#define VCHIQ_MODULE_NAME "drivers/misc/vchiq/driver_v1"
#define VCHIQ_DEVICE_MODULE_NAME "drivers/misc/vchiq/device_v1"
#define VCHIQ_DEVICE_NAME "vchiq"

#define TRACE_ALWAYS(x...)	dprintf("vchiq: " x)
#define ERROR(x...)			dprintf("vchiq: " x)

// Mailbox property tag: hand the slots' bus address to the firmware
#define BCM2835_TAG_VCHIQ_INIT	0x00048010

// Doorbells, from the node's "reg"
enum {
	kBell0			= 0x00 / 4,
		// VideoCore to ARM; reading returns the status and clears it
	kBell2			= 0x08 / 4,
		// ARM to VideoCore; writing rings it
};
#define BELL0_RUNG		(1u << 2)

static const size_t kMaxReportSize = 32 * 1024;
static const char* const kGencmdCommands[] = {
	"version",
	"get_mem arm",
	"get_mem gpu",
	"measure_temp",
};


struct vchiq_device {
	device_node*		node;

	phys_addr_t			registers_physical;
	area_id				registers_area;
	volatile uint32*	registers;
	uint64				interrupt;
	int64				bus_offset;
	phys_addr_t			dma_end;
	bool				long_bulk_addresses;

	mutex				lock;
	bool				initialized;
		// tried once: the firmware may hold the slots from then on
	status_t			init_status;
	int32				interrupts;
};

struct vchiq_file {
	vchiq_device*		device;
	void*				instance;
	char*				report;
	size_t				report_size;
};


static device_manager_info* sDeviceManager;
static bcm2835_mailbox_module_info* sMailbox;
static vchiq_device* sDevice;


//	#pragma mark - memory (vchiq_haiku.h)


status_t
vchiq_haiku_dma_alloc(const char* name, size_t size, void** _address,
	phys_addr_t* _physical, area_id* _area)
{
	size = ROUNDUP(size, B_PAGE_SIZE);

	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = sDevice->dma_end;

	void* address;
	area_id area = create_area_etc(B_SYSTEM_TEAM, name, size, B_CONTIGUOUS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0, &virtualRestrictions,
		&physicalRestrictions, &address);
	if (area < 0)
		return area;

	physical_entry entry;
	uint32 count = 1;
	status_t status = get_memory_map_etc(B_SYSTEM_TEAM, address, B_PAGE_SIZE,
		&entry, &count);
	if (status == B_BUFFER_OVERFLOW)
		status = B_OK;
	if (status == B_OK) {
		// The pages were cleared through the cacheable mapping: write those
		// lines back and drop them, so that no dirty line can be evicted over
		// what the VideoCore writes later.
		arm64_dcache_flush_range((addr_t)address, size);
		status = vm_set_area_memory_type(area, entry.address,
			B_WRITE_COMBINING_MEMORY);
	}
	if (status == B_OK && entry.address + size > sDevice->dma_end)
		status = B_NO_MEMORY;
	if (status != B_OK) {
		delete_area(area);
		return status;
	}

	*_address = address;
	*_physical = entry.address;
	*_area = area;
	return B_OK;
}


void
vchiq_haiku_dma_free(area_id area)
{
	delete_area(area);
}


uint32
vchiq_haiku_bus_address(phys_addr_t physical)
{
	return (uint32)(physical + sDevice->bus_offset);
}


bool
vchiq_haiku_long_bulk_addresses(void)
{
	return sDevice->long_bulk_addresses;
}


uint32
vchiq_haiku_cache_line_size(void)
{
	return arm64_dcache_line_size();
}


/*!	Bulk buffers of user teams. The VideoCore may still be reading or
	writing one when its team goes away, and a team's areas are deleted
	before its files are closed (and the transfers with them): when a team
	is removed, its buffers move to kernel clones of their areas, which keep
	the pages alive until the transfers end (as Linux keeps its page
	references).
*/
static mutex sBuffersLock = MUTEX_INITIALIZER("vchiq buffers");
static vchiq_haiku_buffer* sBuffers;


static uint32
lock_flags(vchiq_haiku_buffer* buffer)
{
	return buffer->device_writes ? B_READ_DEVICE : 0;
}


/*!	The buffer's pages, from its team's area to a kernel clone of that area,
	without being unlocked in between. Called with sBuffersLock held.
*/
static void
adopt_buffer(vchiq_haiku_buffer* buffer)
{
	area_info info;
	status_t status = get_area_info(buffer->area, &info);
	void* cloneBase = NULL;
	area_id clone = -1;
	if (status == B_OK) {
		clone = vm_clone_area(B_SYSTEM_TEAM, "vchiq bulk buffer", &cloneBase,
			B_ANY_KERNEL_ADDRESS, B_KERNEL_READ_AREA
				| (buffer->device_writes ? B_KERNEL_WRITE_AREA : 0),
			REGION_NO_PRIVATE_MAP, buffer->area, true);
		status = clone >= 0 ? B_OK : clone;
	}
	void* address = (uint8*)cloneBase
		+ ((addr_t)buffer->address - (addr_t)info.address);
	if (status == B_OK) {
		status = lock_memory_etc(B_SYSTEM_TEAM, address, buffer->size,
			lock_flags(buffer));
		if (status != B_OK)
			delete_area(clone);
	}
	if (status != B_OK) {
		ERROR("cannot keep a bulk buffer of team %" B_PRId32 " (area %" B_PRId32
			"): %s\n", buffer->team, buffer->area, strerror(status));
		return;
	}

	unlock_memory_etc(buffer->team, buffer->address, buffer->size,
		lock_flags(buffer));
	buffer->clone = clone;
	buffer->clone_address = address;
}


class TeamListener : public NotificationListener {
public:
	virtual void EventOccurred(NotificationService& service,
		const KMessage* event)
	{
		int32 eventCode;
		team_id team;
		if (event->FindInt32("event", &eventCode) != B_OK
			|| eventCode != TEAM_REMOVED
			|| event->FindInt32("team", &team) != B_OK) {
			return;
		}

		MutexLocker locker(sBuffersLock);
		for (vchiq_haiku_buffer* buffer = sBuffers; buffer != NULL;
				buffer = buffer->next) {
			if (buffer->team == team && buffer->clone < 0)
				adopt_buffer(buffer);
		}
	}
};

static TeamListener* sTeamListener;


status_t
vchiq_haiku_buffer_lock(void* address, size_t size, bool deviceWrites,
	vchiq_haiku_buffer** _buffer)
{
	const size_t pageOffset = (addr_t)address & (B_PAGE_SIZE - 1);
	const uint32 maxRuns = (pageOffset + size + B_PAGE_SIZE - 1) / B_PAGE_SIZE;
	vchiq_haiku_buffer* buffer = (vchiq_haiku_buffer*)calloc(1,
		sizeof(vchiq_haiku_buffer) + maxRuns * sizeof(physical_entry));
	if (buffer == NULL)
		return B_NO_MEMORY;

	buffer->team = B_SYSTEM_TEAM;
	buffer->address = address;
	buffer->size = size;
	buffer->device_writes = deviceWrites;
	buffer->area = buffer->clone = -1;
	if (size == 0) {
		*_buffer = buffer;
		return B_OK;
	}

	if (IS_USER_ADDRESS(address)) {
		// one area, so that it can be cloned (a malloc()ed buffer is)
		buffer->team = team_get_current_team_id();
		buffer->area = area_for(address);
		area_info info;
		if (buffer->area < 0 || get_area_info(buffer->area, &info) != B_OK
			|| (addr_t)address + size > (addr_t)info.address + info.size) {
			ERROR("bulk buffer %p (%zu bytes) is not in one area\n", address,
				size);
			free(buffer);
			return B_BAD_VALUE;
		}
	}

	status_t status = lock_memory_etc(buffer->team, address, size,
		lock_flags(buffer));
	if (status != B_OK) {
		free(buffer);
		return status;
	}

	uint32 count = maxRuns;
	status = get_memory_map_etc(buffer->team, address, size, buffer->runs,
		&count);
	if (status != B_OK) {
		unlock_memory_etc(buffer->team, address, size, lock_flags(buffer));
		free(buffer);
		return status;
	}
	buffer->run_count = count;

	if (buffer->team != B_SYSTEM_TEAM) {
		MutexLocker locker(sBuffersLock);
		buffer->next = sBuffers;
		if (sBuffers != NULL)
			sBuffers->previous = buffer;
		sBuffers = buffer;
	}

	*_buffer = buffer;
	return B_OK;
}


void
vchiq_haiku_buffer_unlock(vchiq_haiku_buffer* buffer)
{
	if (buffer->team != B_SYSTEM_TEAM) {
		MutexLocker locker(sBuffersLock);
		if (buffer->previous != NULL)
			buffer->previous->next = buffer->next;
		else
			sBuffers = buffer->next;
		if (buffer->next != NULL)
			buffer->next->previous = buffer->previous;
	}

	if (buffer->clone >= 0) {
		unlock_memory_etc(B_SYSTEM_TEAM, buffer->clone_address, buffer->size,
			lock_flags(buffer));
		delete_area(buffer->clone);
	} else if (buffer->size > 0) {
		unlock_memory_etc(buffer->team, buffer->address, buffer->size,
			lock_flags(buffer));
	}
	free(buffer);
}


/*!	Calls \a function for the parts of the buffer's physical runs from
	\a offset on, \a size bytes, with their address in the physical map.
*/
template<typename Function>
static void
for_each_range(vchiq_haiku_buffer* buffer, size_t offset, size_t size,
	Function function)
{
	for (uint32 i = 0; i < buffer->run_count && size > 0; i++) {
		size_t runSize = buffer->runs[i].size;
		if (offset >= runSize) {
			offset -= runSize;
			continue;
		}
		size_t length = min_c(runSize - offset, size);
		function(KERNEL_PMAP_BASE + buffer->runs[i].address + offset, length);
		size -= length;
		offset = 0;
	}
}


void
vchiq_haiku_buffer_clean(vchiq_haiku_buffer* buffer, size_t offset,
	size_t size)
{
	for_each_range(buffer, offset, size, [](addr_t address, size_t length) {
		arm64_dcache_clean_range(address, length);
	});
}


void
vchiq_haiku_buffer_flush(vchiq_haiku_buffer* buffer, size_t offset,
	size_t size)
{
	for_each_range(buffer, offset, size, [](addr_t address, size_t length) {
		arm64_dcache_flush_range(address, length);
	});
}


void
vchiq_haiku_buffer_write(vchiq_haiku_buffer* buffer, size_t offset,
	const void* data, size_t size)
{
	const uint8* source = (const uint8*)data;
	for_each_range(buffer, offset, size,
		[&source](addr_t address, size_t length) {
			memcpy((void*)address, source, length);
			source += length;
		});
}


//	#pragma mark - firmware and doorbells (vchiq_haiku.h)


status_t
vchiq_haiku_firmware_init(uint32 slotsBusAddress)
{
	uint32 value = slotsBusAddress;
	status_t status = sMailbox->property_tag(BCM2835_TAG_VCHIQ_INIT, &value,
		sizeof(value), sizeof(value), NULL);
	if (status == B_OK && value != 0) {
		// the firmware answers 0 when it took the slots
		status = B_ERROR;
	}
	TRACE_ALWAYS("VCHIQ_INIT with slots at bus %#" B_PRIx32 ": %s (answer %#"
		B_PRIx32 ")\n", slotsBusAddress, strerror(status), value);
	return status;
}


void
vchiq_haiku_ring_doorbell(void)
{
	sDevice->registers[kBell2] = 0;
}


static int32
vchiq_interrupt(void* data)
{
	vchiq_device* device = (vchiq_device*)data;

	uint32 status = device->registers[kBell0];
	if ((status & BELL0_RUNG) == 0)
		return B_UNHANDLED_INTERRUPT;

	atomic_add(&device->interrupts, 1);
	vchiq_haiku_doorbell();
	return B_INVOKE_SCHEDULER;
}


/*!	The slots, the core and the doorbell interrupt, once: whatever the
	outcome, the firmware may hold the slots afterwards. Called with the
	lock held.
*/
static status_t
init_vchiq(vchiq_device* device)
{
	if (device->initialized)
		return device->init_status;
	device->initialized = true;

	const phys_addr_t pageBase = device->registers_physical
		& ~(phys_addr_t)(B_PAGE_SIZE - 1);
	void* address;
	device->registers_area = map_physical_memory("vchiq doorbells", pageBase,
		B_PAGE_SIZE, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &address);
	status_t status = device->registers_area;
	if (status >= B_OK) {
		device->registers = (volatile uint32*)((addr_t)address
			+ (device->registers_physical - pageBase));
		status = vchiq_haiku_init();
	}
	if (status == B_OK) {
		status = install_io_interrupt_handler(device->interrupt,
			&vchiq_interrupt, device, 0);
	}
	if (status == B_OK) {
		sTeamListener = new(std::nothrow) TeamListener;
		status = sTeamListener != NULL
			? NotificationManager::Manager().AddListener("teams", TEAM_REMOVED,
				*sTeamListener)
			: B_NO_MEMORY;
	}

	device->init_status = status;
	if (status != B_OK)
		ERROR("initialisation failed: %s\n", strerror(status));
	else
		TRACE_ALWAYS("connected to the VideoCore\n");
	return status;
}


//	#pragma mark - device module API


static status_t
vchiq_init_device(void* driverCookie, void** _cookie)
{
	*_cookie = driverCookie;
	return B_OK;
}


static void
vchiq_uninit_device(void* cookie)
{
}


static status_t
vchiq_open(void* deviceCookie, const char* path, int openMode, void** _cookie)
{
	vchiq_device* device = (vchiq_device*)deviceCookie;

	vchiq_file* file = (vchiq_file*)calloc(1, sizeof(vchiq_file));
	if (file == NULL)
		return B_NO_MEMORY;
	file->device = device;

	MutexLocker locker(device->lock);
	status_t status = init_vchiq(device);
	locker.Unlock();
	if (status != B_OK) {
		free(file);
		return status;
	}

	// Without a connection to the VideoCore, the file still reads (the
	// state dump says why); its ioctls fail.
	if (vchiq_haiku_open(team_get_current_team_id(), &file->instance) != B_OK)
		file->instance = NULL;

	*_cookie = file;
	return B_OK;
}


static status_t
vchiq_close(void* cookie)
{
	return B_OK;
}


static status_t
vchiq_free(void* cookie)
{
	vchiq_file* file = (vchiq_file*)cookie;

	if (file->instance != NULL)
		vchiq_haiku_free(file->instance);
	free(file->report);
	free(file);
	return B_OK;
}


static void
append(char* report, size_t& length, const char* format, ...)
{
	if (length >= kMaxReportSize - 1)
		return;

	va_list args;
	va_start(args, format);
	int added = vsnprintf(report + length, kMaxReportSize - length, format,
		args);
	va_end(args);
	if (added > 0)
		length = min_c(length + added, kMaxReportSize - 1);
}


static status_t
make_report(vchiq_file* file)
{
	vchiq_device* device = file->device;

	char* report = (char*)malloc(kMaxReportSize);
	if (report == NULL)
		return B_NO_MEMORY;

	size_t length = 0;
	char reply[256];
	for (size_t i = 0; i < B_COUNT_OF(kGencmdCommands); i++) {
		int32 result = 0;
		status_t status = vchiq_haiku_gencmd(kGencmdCommands[i], reply,
			sizeof(reply), &result);
		if (status == B_OK) {
			append(report, length, "gencmd %s: (%" B_PRId32 ") %s\n",
				kGencmdCommands[i], result, reply);
		} else {
			append(report, length, "gencmd %s: %s\n", kGencmdCommands[i],
				strerror(status));
		}
	}

	append(report, length, "vchiq: doorbells %#" B_PRIxPHYSADDR ", interrupt %"
		B_PRIu64 " (%" B_PRId32 " taken), DMA offset %#" B_PRIx64 " below %#"
		B_PRIxPHYSADDR "%s\n", device->registers_physical, device->interrupt,
		atomic_get(&device->interrupts), device->bus_offset, device->dma_end,
		device->long_bulk_addresses ? ", 36-bit bulk addresses" : "");

	length += vchiq_haiku_dump_state(report + length,
		kMaxReportSize - 1 - length);
	report[length] = '\0';

	free(file->report);
	file->report = report;
	file->report_size = length;
	return B_OK;
}


static status_t
vchiq_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	vchiq_file* file = (vchiq_file*)cookie;

	if (position == 0 || file->report == NULL) {
		status_t status = make_report(file);
		if (status != B_OK)
			return status;
	}

	if (position < 0 || (size_t)position >= file->report_size) {
		*_length = 0;
		return B_OK;
	}
	size_t count = min_c(*_length, file->report_size - (size_t)position);
	if (user_memcpy(buffer, file->report + position, count) != B_OK)
		return B_BAD_ADDRESS;
	*_length = count;
	return B_OK;
}


static status_t
vchiq_write(void* cookie, off_t position, const void* buffer,
	size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


/*!	The command number says the argument's size and direction (BSD's
	<sys/ioccom.h> encoding): the "length" argument is not used, as
	libvchiq does not pass one. For _IO commands the argument is the value
	itself.
*/
static status_t
vchiq_control(void* cookie, uint32 op, void* arg, size_t length)
{
	vchiq_file* file = (vchiq_file*)cookie;

	if (IOCGROUP(op) != VCHIQ_IOC_MAGIC)
		return B_DEV_INVALID_IOCTL;
	if (file->instance == NULL)
		return ENOTCONN;

	union {
		VCHIQ_CREATE_SERVICE_T			create_service;
		VCHIQ_QUEUE_MESSAGE_T			queue_message;
		VCHIQ_QUEUE_BULK_TRANSFER_T		queue_bulk_transfer;
		VCHIQ_AWAIT_COMPLETION_T		await_completion;
		VCHIQ_DEQUEUE_MESSAGE_T			dequeue_message;
		VCHIQ_GET_CONFIG_T				get_config;
		VCHIQ_SET_SERVICE_OPTION_T		set_service_option;
		VCHIQ_DUMP_MEM_T				dump_mem;
		size_t							value;
	} data;

	const size_t size = IOCPARM_LEN(op);
	if (size > sizeof(data))
		return B_BAD_VALUE;

	if (size == 0) {
		data.value = (addr_t)arg;
	} else {
		if (!IS_USER_ADDRESS(arg))
			return B_BAD_ADDRESS;
		if ((op & IOC_IN) != 0) {
			if (user_memcpy(&data, arg, size) != B_OK)
				return B_BAD_ADDRESS;
		} else
			memset(&data, 0, size);
	}

	int result = vchiq_haiku_ioctl(file->instance, op, &data);

	// Also after an error: an interrupted blocking bulk transfer reports
	// in its "mode" that it waits (libvchiq repeats the call with it).
	if (size > 0 && (op & IOC_OUT) != 0
		&& user_memcpy(arg, &data, size) != B_OK && result >= 0) {
		return B_BAD_ADDRESS;
	}
	return result;
}


//	#pragma mark - driver module API


static float
vchiq_support(device_node* parent)
{
	const char* bus;
	if (sDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
			!= B_OK || strcmp(bus, "fdt") != 0) {
		return 0.0f;
	}

	const char* compatible;
	if (sDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible,
			false) != B_OK) {
		return 0.0f;
	}
	if (strcmp(compatible, "brcm,bcm2711-vchiq") != 0
		&& strcmp(compatible, "brcm,bcm2835-vchiq") != 0) {
		return 0.0f;
	}
	return 0.6f;
}


static status_t
vchiq_register_device(device_node* node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Broadcom VideoCore VCHIQ" } },
		// the firmware keeps the slots once they are set up
		{ B_DEVICE_FLAGS, B_UINT32_TYPE, { .ui32 = B_KEEP_DRIVER_LOADED } },
		{ NULL }
	};

	return sDeviceManager->register_node(node, VCHIQ_MODULE_NAME, attrs, NULL,
		NULL);
}


static status_t
vchiq_init_driver(device_node* node, void** _driverCookie)
{
	if (sDevice != NULL)
		return B_BUSY;

	device_node* parent = sDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&fdtDevice);
	const char* compatible = NULL;
	sDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible,
		false);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	vchiq_device* device = (vchiq_device*)calloc(1, sizeof(vchiq_device));
	if (device == NULL)
		return B_NO_MEMORY;

	uint64 registers, size;
	if (!fdt->get_reg(fdtDevice, 0, &registers, &size)
		|| size < 3 * sizeof(uint32)
		|| !fdt->get_interrupt(fdtDevice, 0, NULL, &device->interrupt)) {
		free(device);
		return B_DEVICE_NOT_FOUND;
	}

	// The VideoCore sees RAM through the bus's DMA window (on the Pi 4's
	// /soc: bus 0xc0000000 = physical 0, 1 GiB). Without one, the classic
	// alias of the BCM2835.
	int64 busOffset = 0xc0000000;
	phys_addr_t dmaEnd = 0x40000000;
	fdt->get_dma_window(fdtDevice, &busOffset, &dmaEnd);
	dmaEnd = min_c(dmaEnd, (phys_addr_t)0x40000000);

	device->node = node;
	device->registers_physical = registers;
	device->registers_area = -1;
	device->bus_offset = busOffset;
	device->dma_end = dmaEnd;
	device->long_bulk_addresses = compatible != NULL
		&& strcmp(compatible, "brcm,bcm2711-vchiq") == 0;
	mutex_init(&device->lock, "vchiq");

	TRACE_ALWAYS("doorbells %#" B_PRIxPHYSADDR ", interrupt %" B_PRIu64
		", DMA offset %#" B_PRIx64 " below %#" B_PRIxPHYSADDR "%s\n",
		device->registers_physical, device->interrupt, device->bus_offset,
		device->dma_end,
		device->long_bulk_addresses ? ", 36-bit bulk addresses" : "");

	sDevice = device;
	*_driverCookie = device;
	return B_OK;
}


static void
vchiq_uninit_driver(void* driverCookie)
{
	// B_KEEP_DRIVER_LOADED: only when the device goes away, before it was
	// ever opened
	vchiq_device* device = (vchiq_device*)driverCookie;
	if (device->initialized)
		panic("vchiq: unloaded while the firmware holds its memory");

	mutex_destroy(&device->lock);
	free(device);
	sDevice = NULL;
}


static status_t
vchiq_register_child_devices(void* driverCookie)
{
	vchiq_device* device = (vchiq_device*)driverCookie;
	return sDeviceManager->publish_device(device->node, VCHIQ_DEVICE_NAME,
		VCHIQ_DEVICE_MODULE_NAME);
}


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{ BCM2835_MAILBOX_MODULE_NAME, (module_info**)&sMailbox },
	{}
};


driver_module_info vchiq_driver_module = {
	{
		VCHIQ_MODULE_NAME,
		0,
		NULL
	},

	vchiq_support,
	vchiq_register_device,
	vchiq_init_driver,
	vchiq_uninit_driver,
	vchiq_register_child_devices,
	NULL,	// rescan
	NULL,	// removed
};


struct device_module_info vchiq_device_module = {
	{
		VCHIQ_DEVICE_MODULE_NAME,
		0,
		NULL
	},

	vchiq_init_device,
	vchiq_uninit_device,
	NULL,

	vchiq_open,
	vchiq_close,
	vchiq_free,
	vchiq_read,
	vchiq_write,
	NULL,
	vchiq_control,

	NULL,
	NULL
};


module_info* modules[] = {
	(module_info*)&vchiq_driver_module,
	(module_info*)&vchiq_device_module,
	NULL
};
