/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The Raspberry Pi 4's 3D GPU, V3D 4.2 ("brcm,2711-v3d").

	The driver publishes /dev/dri/renderD128 and speaks the Linux v3d uAPI
	over ioctl(): buffer objects, control list jobs, syncobjs - what Mesa's
	v3d driver needs. Reading the device returns the identification
	registers.

	Nothing touches the hardware while the system boots: an MMIO access to an
	unclocked V3D never completes. The first open() powers the GPU on, sets
	up its MMU and installs the interrupt handler; it stays on from then on.

	Power-on, as the VideoCore firmware does not do it by itself: QPU enable,
	power domain and clock through the firmware mailbox, then the V3D reset
	line in the power management block (PM_GRAFX.V3DRSTN) and the two
	asynchronous bus bridges (ASB) between the GPU and the bus.
*/


#include "v3d_private.h"

#include <Drivers.h>
#include <Errors.h>
#include <bus/FDT.h>
#include <kernel.h>
#include <ByteOrder.h>
#include <util/AutoLock.h>

#include <stdio.h>
#include <sys/ioccom.h>

#include <bcm2835_mailbox.h>


#define V3D_MODULE_NAME "drivers/graphics/v3d/driver_v1"
#define V3D_DEVICE_MODULE_NAME "drivers/graphics/v3d/device_v1"
#define V3D_DEVICE_PATH "dri/renderD128"


// Power management block and bus bridges (PM "pm" and "rpivid_asb")
enum {
	kPmGraphics			= 0x010c,
	kPmV3dResetN		= 1 << 6,
	kPmPassword			= 0x5a000000,

	kAsbSlaveControl	= 0x08,
	kAsbMasterControl	= 0x0c,
	kAsbRequestStop		= 1 << 0,
	kAsbAcknowledge		= 1 << 1,
};

static const uint32 kV3dClock = 5;
	// the firmware's ID of the V3D clock

// "V3D" in the low bytes of CORE0_IDENT0; 0xdeadbeef means unpowered
static const uint32 kCoreIdentMagic = 0x443356;


static device_manager_info* sDeviceManager;
static bcm2835_mailbox_module_info* sMailbox;


//	#pragma mark - power


static status_t
firmware_call(uint32 tag, uint32 id, uint32 value, uint32 requestWords,
	uint32* _answer)
{
	uint32 values[2] = { id, value };
	status_t status = sMailbox->property_tag(tag, values, sizeof(values),
		requestWords * sizeof(uint32), NULL);
	if (status == B_OK && _answer != NULL)
		*_answer = values[1];
	return status;
}


static status_t
set_clock(bool on)
{
	return firmware_call(BCM2835_TAG_SET_CLOCK_STATE, kV3dClock, on ? 1 : 0, 2,
		NULL);
}


/*!	Turns the V3D clock on, and only returns B_OK when the firmware says it
	is on: the next step would be an MMIO access.
*/
static status_t
set_clock_on_confirmed()
{
	status_t setStatus = B_ERROR, getStatus = B_ERROR;
	uint32 state = 0;
	for (int32 i = 0; i < 20; i++) {
		setStatus = set_clock(true);
		getStatus = firmware_call(BCM2835_TAG_GET_CLOCK_STATE, kV3dClock, 0, 1,
			&state);
		if (getStatus == B_OK && (state & 1) != 0)
			return B_OK;
		snooze(200);
	}
	uint32 rate = 0;
	firmware_call(BCM2835_TAG_GET_CLOCK_RATE, kV3dClock, 0, 1, &rate);
	ERROR("clock does not come on: set %s, get %s, state %#" B_PRIx32
		", rate %" B_PRIu32 "\n", strerror(setStatus), strerror(getStatus),
		state, rate);
	return B_TIMED_OUT;
}


static status_t
map_registers(const char* name, phys_addr_t physical, area_id& area,
	volatile uint32*& registers)
{
	void* address;
	area = map_physical_memory(name, physical & ~(phys_addr_t)(B_PAGE_SIZE - 1),
		B_PAGE_SIZE * 4, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &address);
	if (area < 0)
		return area;
	registers = (volatile uint32*)((addr_t)address
		+ (physical & (B_PAGE_SIZE - 1)));
	return B_OK;
}


/*!	Releases the stop request of a bus bridge and waits until it acknowledges
	(the acknowledge bit clears).
*/
static status_t
enable_bridge(volatile uint32* asb, uint32 offset)
{
	asb[offset / 4] = kPmPassword | (asb[offset / 4] & ~kAsbRequestStop);
	for (int32 i = 0; i < 100000; i++) {
		if ((asb[offset / 4] & kAsbAcknowledge) == 0)
			return B_OK;
	}
	return B_TIMED_OUT;
}


/*!	Requests a bus bridge to stop and waits until it acknowledges. */
static status_t
stop_bridge(volatile uint32* asb, uint32 offset)
{
	asb[offset / 4] = kPmPassword | asb[offset / 4] | kAsbRequestStop;
	for (int32 i = 0; i < 100000; i++) {
		if ((asb[offset / 4] & kAsbAcknowledge) != 0)
			return B_OK;
	}
	return B_TIMED_OUT;
}


status_t
v3d_power_on(v3d_device* device)
{
	uint32 answer;
	uint32 qpu[1] = { 1 };
	status_t qpuStatus = sMailbox->property_tag(BCM2835_TAG_SET_QPU_ENABLE,
		qpu, sizeof(qpu), sizeof(qpu), NULL);
	status_t domainStatus = firmware_call(BCM2835_TAG_SET_DOMAIN_STATE,
		BCM2835_DOMAIN_V3D, 1, 2, &answer);
	set_clock(true);

	area_id pmArea, asbArea;
	volatile uint32* pm;
	volatile uint32* asb;
	status_t status = map_registers("v3d pm", device->pm_physical, pmArea, pm);
	if (status != B_OK)
		return status;
	status = map_registers("v3d asb", device->asb_physical, asbArea, asb);
	if (status != B_OK) {
		delete_area(pmArea);
		return status;
	}

	// Release the reset with the clock briefly on, then off
	set_clock(true);
	snooze(50);
	set_clock(false);
	const uint32 graphics = pm[kPmGraphics / 4];
	pm[kPmGraphics / 4] = kPmPassword | graphics | kPmV3dResetN;

	status_t clockStatus = set_clock_on_confirmed();
	snooze(50);

	status_t masterStatus = B_NO_INIT, slaveStatus = B_NO_INIT;
	if (clockStatus == B_OK) {
		masterStatus = enable_bridge(asb, kAsbMasterControl);
		slaveStatus = enable_bridge(asb, kAsbSlaveControl);
	}
	const uint32 graphicsAfter = pm[kPmGraphics / 4];
	snooze(2000);

	delete_area(asbArea);
	delete_area(pmArea);

	TRACE_ALWAYS("power on: qpu %s, domain %s, clock %s, PM_GRAFX %#" B_PRIx32
		" -> %#" B_PRIx32 ", bridges %s/%s\n", strerror(qpuStatus),
		strerror(domainStatus), strerror(clockStatus), graphics, graphicsAfter,
		strerror(masterStatus), strerror(slaveStatus));

	if (clockStatus != B_OK || masterStatus != B_OK || slaveStatus != B_OK)
		return B_ERROR;
	return B_OK;
}


/*!	A true reset of a wedged GPU (Phoenix's proven recovery): stop both bus
	bridges, hold the V3D reset line, then the whole power-on again. The
	caller has masked the GPU's interrupts and stopped its GMP.
*/
status_t
v3d_power_reset(v3d_device* device)
{
	area_id pmArea, asbArea;
	volatile uint32* pm;
	volatile uint32* asb;
	status_t status = map_registers("v3d pm", device->pm_physical, pmArea, pm);
	if (status != B_OK)
		return status;
	status = map_registers("v3d asb", device->asb_physical, asbArea, asb);
	if (status != B_OK) {
		delete_area(pmArea);
		return status;
	}

	status_t masterStatus = stop_bridge(asb, kAsbMasterControl);
	status_t slaveStatus = stop_bridge(asb, kAsbSlaveControl);
	pm[kPmGraphics / 4] = kPmPassword
		| (pm[kPmGraphics / 4] & ~(uint32)kPmV3dResetN);
	snooze(100);

	delete_area(asbArea);
	delete_area(pmArea);

	TRACE_ALWAYS("reset: bridges stopped %s/%s\n", strerror(masterStatus),
		strerror(slaveStatus));
	return v3d_power_on(device);
}


/*!	Powers the GPU on (once), masks its interrupts, records its
	identification for read(), then sets up the MMU and the interrupt
	handler.
*/
static status_t
start_gpu(v3d_device* device)
{
	if (device->started)
		return B_OK;
	if (device->broken)
		return B_DEVICE_NOT_FOUND;

	if (!device->powered) {
		status_t status = v3d_power_on(device);
		if (status != B_OK)
			return status;

		status = map_registers("v3d hub", device->hub_physical,
			device->hub_area, device->hub);
		if (status != B_OK)
			return status;
		status = map_registers("v3d core", device->core_physical,
			device->core_area, device->core);
		if (status != B_OK) {
			delete_area(device->hub_area);
			device->hub_area = -1;
			return status;
		}

		// No interrupts until there is a handler for them
		core_write(device, kCoreInterruptMaskSet, ~0u);
		core_write(device, kCoreInterruptClear, ~0u);
		hub_write(device, kHubInterruptMaskSet, ~0u);
		hub_write(device, kHubInterruptClear, ~0u);

		uint32 rate = 0, measured = 0;
		firmware_call(BCM2835_TAG_GET_CLOCK_RATE, kV3dClock, 0, 1, &rate);
		firmware_call(BCM2835_TAG_GET_CLOCK_MEASURED, kV3dClock, 0, 1,
			&measured);

		device->report_length = snprintf(device->report,
			sizeof(device->report),
			"V3D: core ident %#" B_PRIx32 "/%#" B_PRIx32 "/%#" B_PRIx32
			", hub ident %#" B_PRIx32 "/%#" B_PRIx32 "/%#" B_PRIx32 "/%#"
			B_PRIx32 ", uif %#" B_PRIx32 ", mmu debug %#" B_PRIx32
			", clock %" B_PRIu32 " MHz (measured %" B_PRIu32 " MHz)"
			", interrupt %" B_PRIu64 "\n",
			core_read(device, kCoreIdent0), core_read(device, kCoreIdent1),
			core_read(device, kCoreIdent2), hub_read(device, kHubIdent0),
			hub_read(device, kHubIdent1), hub_read(device, kHubIdent2),
			hub_read(device, kHubIdent3), hub_read(device, kHubUifConfig),
			hub_read(device, kMmuDebugInfo), rate / 1000000,
			measured / 1000000, device->interrupt);
		TRACE_ALWAYS("%s", device->report);

		device->powered = true;
	}

	// Do not drive a GPU that does not identify itself.
	if ((core_read(device, kCoreIdent0) & 0xffffff) != kCoreIdentMagic) {
		ERROR("unexpected identification %#" B_PRIx32 ", not using the GPU\n",
			core_read(device, kCoreIdent0));
		return B_DEVICE_NOT_FOUND;
	}

	status_t status = B_OK;
	if (device->page_table_area < 0)
		status = v3d_page_table_alloc(device);
	if (status == B_OK)
		status = v3d_mmu_init(device);
	if (status != B_OK) {
		ERROR("MMU set-up failed: %s\n", strerror(status));
		return status;
	}
	status = v3d_jobs_start(device);
	if (status != B_OK) {
		ERROR("interrupt set-up failed: %s\n", strerror(status));
		return status;
	}

	device->started = true;
	return B_OK;
}


//	#pragma mark - ioctls


static status_t
get_param(v3d_file* file, drm_v3d_get_param* args)
{
	v3d_device* device = file->device;
	if (args->pad != 0)
		return B_BAD_VALUE;

	MutexLocker locker(device->lock);
	switch (args->param) {
		case DRM_V3D_PARAM_SUPPORTS_TFU:
		case DRM_V3D_PARAM_SUPPORTS_CSD:
		case DRM_V3D_PARAM_SUPPORTS_CACHE_FLUSH:
			args->value = 1;
			return B_OK;
		case DRM_V3D_PARAM_SUPPORTS_PERFMON:
		case DRM_V3D_PARAM_SUPPORTS_MULTISYNC_EXT:
		case DRM_V3D_PARAM_SUPPORTS_CPU_QUEUE:
		case DRM_V3D_PARAM_MAX_PERF_COUNTERS:
		case DRM_V3D_PARAM_SUPPORTS_SUPER_PAGES:
		case DRM_V3D_PARAM_CONTEXT_RESET_COUNTER:
			args->value = 0;
			return B_OK;
		case DRM_V3D_PARAM_GLOBAL_RESET_COUNTER:
			args->value = device->resets;
			return B_OK;
	}

	// The identification, read live from the hardware.
	if (device->broken)
		return B_DEVICE_NOT_FOUND;
	switch (args->param) {
		case DRM_V3D_PARAM_V3D_UIFCFG:
			args->value = hub_read(device, kHubUifConfig);
			return B_OK;
		case DRM_V3D_PARAM_V3D_HUB_IDENT1:
			args->value = hub_read(device, kHubIdent1);
			return B_OK;
		case DRM_V3D_PARAM_V3D_HUB_IDENT2:
			args->value = hub_read(device, kHubIdent2);
			return B_OK;
		case DRM_V3D_PARAM_V3D_HUB_IDENT3:
			args->value = hub_read(device, kHubIdent3);
			return B_OK;
		case DRM_V3D_PARAM_V3D_CORE0_IDENT0:
			args->value = core_read(device, kCoreIdent0);
			return B_OK;
		case DRM_V3D_PARAM_V3D_CORE0_IDENT1:
			args->value = core_read(device, kCoreIdent1);
			return B_OK;
		case DRM_V3D_PARAM_V3D_CORE0_IDENT2:
			args->value = core_read(device, kCoreIdent2);
			return B_OK;
	}
	return B_BAD_VALUE;
}


/*!	As Linux drm_copy_field(): copies as much as fits, without a NUL, and
	returns the full length.
*/
static status_t
copy_string_out(char* buffer, __kernel_size_t& length, const char* string)
{
	const size_t stringLength = strlen(string);
	if (buffer != NULL && length > 0) {
		if (!IS_USER_ADDRESS(buffer)
			|| user_memcpy(buffer, string, min_c(length, stringLength))
				!= B_OK) {
			return B_BAD_ADDRESS;
		}
	}
	length = stringLength;
	return B_OK;
}


static status_t
get_version(drm_version* args)
{
	// what Linux' v3d reports
	args->version_major = 1;
	args->version_minor = 0;
	args->version_patchlevel = 0;
	status_t status = copy_string_out(args->name, args->name_len, "v3d");
	if (status == B_OK)
		status = copy_string_out(args->date, args->date_len, "20180419");
	if (status == B_OK)
		status = copy_string_out(args->desc, args->desc_len, "Broadcom V3D");
	return status;
}


static status_t
get_cap(drm_get_cap* args)
{
	switch (args->capability) {
		case DRM_CAP_SYNCOBJ:
		case DRM_CAP_TIMESTAMP_MONOTONIC:
			args->value = 1;
			return B_OK;
		case DRM_CAP_PRIME:
		case DRM_CAP_SYNCOBJ_TIMELINE:
			args->value = 0;
			return B_OK;
	}
	return B_BAD_VALUE;
}


//	#pragma mark - device module API


static status_t
v3d_init_device(void* driverCookie, void** _cookie)
{
	*_cookie = driverCookie;
	return B_OK;
}


static void
v3d_uninit_device(void* cookie)
{
}


static status_t
v3d_open(void* deviceCookie, const char* path, int openMode, void** _cookie)
{
	v3d_device* device = (v3d_device*)deviceCookie;

	v3d_file* file = (v3d_file*)calloc(1, sizeof(v3d_file));
	if (file == NULL)
		return B_NO_MEMORY;
	file->device = device;

	MutexLocker locker(device->lock);
	status_t status = start_gpu(device);
	if (status != B_OK) {
		free(file);
		return status;
	}

	*_cookie = file;
	return B_OK;
}


static status_t
v3d_close(void* cookie)
{
	v3d_file* file = (v3d_file*)cookie;
	v3d_device* device = file->device;

	// Let this file's waiters go; the objects go in free().
	MutexLocker locker(device->lock);
	file->closing = true;
	device->job_done.NotifyAll();
	return B_OK;
}


/*!	Everything the file still holds goes, so a client that dies leaks
	nothing: its jobs still run (they hold their BOs), then the BOs follow.
	Its mappings were areas of its team, deleted with it.
*/
static status_t
v3d_free(void* cookie)
{
	v3d_file* file = (v3d_file*)cookie;
	v3d_device* device = file->device;

	MutexLocker locker(device->lock);
	v3d_file_close_bos(file);
	v3d_file_close_syncobjs(file);
	locker.Unlock();

	free(file);
	return B_OK;
}


static status_t
v3d_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	v3d_device* device = ((v3d_file*)cookie)->device;
	if (position >= device->report_length) {
		*_length = 0;
		return B_OK;
	}

	size_t length = min_c(*_length, (size_t)(device->report_length - position));
	if (user_memcpy(buffer, device->report + position, length) != B_OK)
		return B_BAD_ADDRESS;
	*_length = length;
	return B_OK;
}


static status_t
v3d_write(void* cookie, off_t position, const void* buffer, size_t* _length)
{
	return B_NOT_ALLOWED;
}


/*!	DRM ioctls, numbered as drm.h does on Haiku (<sys/ioccom.h>): group 'd',
	the command in the low byte, the argument size in bits 16-28. ioctl()
	passes no length, so the size comes from the number; as Linux, only what
	both sides know is copied (the rest reads as zero), and the argument is
	copied back for everything that returns data.
*/
static status_t
v3d_control(void* cookie, uint32 op, void* arg, size_t length)
{
	v3d_file* file = (v3d_file*)cookie;
	if (IOCGROUP(op) != DRM_IOCTL_BASE)
		return B_DEV_INVALID_IOCTL;

	const uint32 nr = op & 0xff;
	size_t size;
	bool copyOut = true;
	switch (nr) {
		case 0x00:
			size = sizeof(drm_version);
			break;
		case 0x09:
			size = sizeof(drm_gem_close);
			copyOut = false;
			break;
		case 0x0c:
			size = sizeof(drm_get_cap);
			break;
		case 0xbf:
			size = sizeof(drm_syncobj_create);
			break;
		case 0xc0:
			size = sizeof(drm_syncobj_destroy);
			break;
		case 0xc3:
			size = sizeof(drm_syncobj_wait);
			break;
		case 0xc4:
		case 0xc5:
			size = sizeof(drm_syncobj_array);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_SUBMIT_CL:
			size = sizeof(drm_v3d_submit_cl);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_SUBMIT_TFU:
			size = sizeof(drm_v3d_submit_tfu);
			copyOut = false;
			break;
		case DRM_COMMAND_BASE + DRM_V3D_WAIT_BO:
			size = sizeof(drm_v3d_wait_bo);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_CREATE_BO:
			size = sizeof(drm_v3d_create_bo);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_MMAP_BO:
			size = sizeof(drm_v3d_mmap_bo);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_GET_PARAM:
			size = sizeof(drm_v3d_get_param);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_GET_BO_OFFSET:
			size = sizeof(drm_v3d_get_bo_offset);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_HAIKU_IMPORT_MEMORY:
			size = sizeof(drm_v3d_haiku_import_memory);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_HAIKU_SYNC_BO:
			size = sizeof(drm_v3d_haiku_sync_bo);
			break;
		default:
			// SUBMIT_CSD, perfmon, CPU jobs, PRIME, FLINK: not yet
			return B_DEV_INVALID_IOCTL;
	}

	union {
		drm_version				version;
		drm_gem_close			gem_close;
		drm_get_cap				get_cap;
		drm_syncobj_create		syncobj_create;
		drm_syncobj_destroy		syncobj_destroy;
		drm_syncobj_wait		syncobj_wait;
		drm_syncobj_array		syncobj_array;
		drm_v3d_submit_cl		submit_cl;
		drm_v3d_submit_tfu		submit_tfu;
		drm_v3d_wait_bo			wait_bo;
		drm_v3d_create_bo		create_bo;
		drm_v3d_mmap_bo			mmap_bo;
		drm_v3d_get_param		get_param;
		drm_v3d_get_bo_offset	get_bo_offset;
		drm_v3d_haiku_import_memory import_memory;
		drm_v3d_haiku_sync_bo	sync_bo;
	} data;
	memset(&data, 0, sizeof(data));

	const size_t copySize = min_c(size, (size_t)IOCPARM_LEN(op));
	if (!IS_USER_ADDRESS(arg) || user_memcpy(&data, arg, copySize) != B_OK)
		return B_BAD_ADDRESS;

	status_t status;
	switch (nr) {
		case 0x00:
			status = get_version(&data.version);
			break;
		case 0x0c:
			status = get_cap(&data.get_cap);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_GET_PARAM:
			status = get_param(file, &data.get_param);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_SUBMIT_CL:
			status = v3d_submit_cl(file, &data.submit_cl);
			break;
		case DRM_COMMAND_BASE + DRM_V3D_SUBMIT_TFU:
			status = v3d_submit_tfu(file, &data.submit_tfu);
			break;
		case 0xbf:
		case 0xc0:
		case 0xc3:
		case 0xc4:
		case 0xc5:
			status = v3d_syncobj_ioctl(file, nr, &data);
			break;
		default:
			status = v3d_bo_ioctl(file, nr, &data);
			break;
	}

	if (copyOut && user_memcpy(arg, &data, copySize) != B_OK)
		return B_BAD_ADDRESS;
	return status;
}


//	#pragma mark - driver module API


static float
v3d_support(device_node* parent)
{
	const char* bus;
	if (sDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
			!= B_OK || strcmp(bus, "fdt") != 0) {
		return 0.0f;
	}

	// The firmware's device tree marks the node disabled unless one of its vc4
	// overlays is used; it describes the hardware correctly either way.
	const char* compatible;
	if (sDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible,
			false) != B_OK) {
		return 0.0f;
	}
	return strcmp(compatible, "brcm,2711-v3d") == 0 ? 0.6f : 0.0f;
}


static status_t
v3d_register_device(device_node* node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Broadcom V3D 4.2 GPU" } },
		// Once powered, the GPU keeps its page table, interrupt handler and
		// worker: the driver must not be unloaded after the last close.
		{ B_DEVICE_FLAGS, B_UINT32_TYPE, { .ui32 = B_KEEP_DRIVER_LOADED } },
		{ NULL }
	};

	return sDeviceManager->register_node(node, V3D_MODULE_NAME, attrs, NULL,
		NULL);
}


static status_t
get_fdt_device(device_node* node, fdt_device_module_info** _module,
	fdt_device** _device)
{
	return sDeviceManager->get_driver(node, (driver_module_info**)_module,
		(void**)_device);
}


/*!	The power management block is the GPU's "power-domains" provider. */
static status_t
find_power_management(fdt_device_module_info* fdt, fdt_device* fdtDevice,
	phys_addr_t& pm, phys_addr_t& asb)
{
	int length;
	const uint32* domains = (const uint32*)fdt->get_prop(fdtDevice,
		"power-domains", &length);
	if (domains == NULL || length < (int)sizeof(uint32))
		return B_DEVICE_NOT_FOUND;

	device_node* busNode = fdt->get_bus(fdtDevice);
	fdt_bus_module_info* busModule;
	fdt_bus* bus;
	status_t status = sDeviceManager->get_driver(busNode,
		(driver_module_info**)&busModule, (void**)&bus);
	if (status != B_OK)
		return status;

	device_node* pmNode = busModule->node_by_phandle(bus,
		B_BENDIAN_TO_HOST_INT32(domains[0]));
	if (pmNode == NULL)
		return B_DEVICE_NOT_FOUND;

	fdt_device_module_info* pmModule;
	fdt_device* pmDevice;
	status = get_fdt_device(pmNode, &pmModule, &pmDevice);
	if (status != B_OK)
		return status;

	// "pm", "asb", "rpivid_asb": the GPU's bridges are the last ones
	uint64 address, size;
	if (!pmModule->get_reg(pmDevice, 0, &address, &size))
		return B_DEVICE_NOT_FOUND;
	pm = address;
	if (!pmModule->get_reg(pmDevice, 2, &address, &size))
		return B_DEVICE_NOT_FOUND;
	asb = address;
	return B_OK;
}


static status_t
v3d_init_driver(device_node* node, void** _driverCookie)
{
	device_node* parent = sDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	status_t status = get_fdt_device(parent, &fdt, &fdtDevice);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	v3d_device* device = (v3d_device*)calloc(1, sizeof(v3d_device));
	if (device == NULL)
		return B_NO_MEMORY;

	uint64 hub, core, size;
	if (!fdt->get_reg(fdtDevice, 0, &hub, &size)
		|| !fdt->get_reg(fdtDevice, 1, &core, &size)
		|| !fdt->get_interrupt(fdtDevice, 0, NULL, &device->interrupt)) {
		free(device);
		return B_DEVICE_NOT_FOUND;
	}
	status = find_power_management(fdt, fdtDevice, device->pm_physical,
		device->asb_physical);
	if (status != B_OK) {
		ERROR("no power management block: %s\n", strerror(status));
		free(device);
		return status;
	}

	device->node = node;
	device->hub_physical = hub;
	device->core_physical = core;
	device->hub_area = device->core_area = -1;
	device->overflow_area = -1;
	device->worker = -1;
	mutex_init(&device->lock, "v3d");
	B_INITIALIZE_SPINLOCK(&device->irq_lock);
	device->job_done.Init(device, "v3d job done");

	// Memory only; the GPU is not touched before the first open(). Should
	// this fail, the first open() tries again.
	device->page_table_area = device->scratch_area = -1;
	status = v3d_page_table_alloc(device);
	if (status != B_OK)
		ERROR("no memory for the GPU page table yet: %s\n", strerror(status));

	TRACE_ALWAYS("hub %#" B_PRIxPHYSADDR ", core %#" B_PRIxPHYSADDR
		", interrupt %" B_PRIu64 ", pm %#" B_PRIxPHYSADDR ", bridges %#"
		B_PRIxPHYSADDR "\n", device->hub_physical, device->core_physical,
		device->interrupt, device->pm_physical, device->asb_physical);

	*_driverCookie = device;
	return B_OK;
}


static void
v3d_uninit_driver(void* driverCookie)
{
	v3d_device* device = (v3d_device*)driverCookie;

	// Interrupts off and the handler gone before the registers go.
	if (device->started)
		v3d_jobs_stop(device);

	{
		MutexLocker locker(device->lock);
		device->completed_seqno = device->submitted_seqno;
		v3d_bo_release_quarantine(device);
	}

	v3d_mmu_uninit(device);
	if (device->core_area >= 0)
		delete_area(device->core_area);
	if (device->hub_area >= 0)
		delete_area(device->hub_area);
	mutex_destroy(&device->lock);
	free(device);
}


static status_t
v3d_register_child_devices(void* driverCookie)
{
	v3d_device* device = (v3d_device*)driverCookie;
	return sDeviceManager->publish_device(device->node, V3D_DEVICE_PATH,
		V3D_DEVICE_MODULE_NAME);
}


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{ BCM2835_MAILBOX_MODULE_NAME, (module_info**)&sMailbox },
	{}
};


driver_module_info v3d_driver_module = {
	{
		V3D_MODULE_NAME,
		0,
		NULL
	},

	v3d_support,
	v3d_register_device,
	v3d_init_driver,
	v3d_uninit_driver,
	v3d_register_child_devices,
	NULL,	// rescan
	NULL,	// removed
};


struct device_module_info v3d_device_module = {
	{
		V3D_DEVICE_MODULE_NAME,
		0,
		NULL
	},

	v3d_init_device,
	v3d_uninit_device,
	NULL,

	v3d_open,
	v3d_close,
	v3d_free,
	v3d_read,
	v3d_write,
	NULL,
	v3d_control,

	NULL,
	NULL
};


module_info* modules[] = {
	(module_info*)&v3d_driver_module,
	(module_info*)&v3d_device_module,
	NULL
};
