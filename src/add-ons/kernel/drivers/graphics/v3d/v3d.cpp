/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The Raspberry Pi 4's 3D GPU, V3D 4.2 ("brcm,2711-v3d").

	So far this only powers the GPU on and identifies it: the first open() of
	/dev/dri/renderD128 runs the power-on sequence, and reading the device
	returns the identification registers. Nothing touches the hardware while
	the system boots: an MMIO access to an unclocked V3D never completes.

	Power-on, as the VideoCore firmware does not do it by itself: QPU enable,
	power domain and clock through the firmware mailbox, then the V3D reset
	line in the power management block (PM_GRAFX.V3DRSTN) and the two
	asynchronous bus bridges (ASB) between the GPU and the bus.
*/


#include <Drivers.h>
#include <Errors.h>
#include <KernelExport.h>
#include <bus/FDT.h>
#include <kernel.h>
#include <ByteOrder.h>
#include <lock.h>
#include <util/AutoLock.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <bcm2835_mailbox.h>


#define V3D_MODULE_NAME "drivers/graphics/v3d/driver_v1"
#define V3D_DEVICE_MODULE_NAME "drivers/graphics/v3d/device_v1"
#define V3D_DEVICE_PATH "dri/renderD128"

#define TRACE_ALWAYS(x...)	dprintf("v3d: " x)
#define ERROR(x...)			dprintf("v3d: " x)


// Hub registers
enum {
	kHubUifConfig		= 0x0004,
	kHubIdent0			= 0x0008,
	kHubIdent1			= 0x000c,
	kHubIdent2			= 0x0010,
	kHubIdent3			= 0x0014,
	kHubInterruptClear	= 0x0058,
	kHubInterruptMaskSet = 0x0060,
	kHubMmuDebugInfo	= 0x1238,
};

// Core registers
enum {
	kCoreIdent0			= 0x0000,
	kCoreIdent1			= 0x0004,
	kCoreIdent2			= 0x0008,
	kCoreInterruptClear	= 0x0058,
	kCoreInterruptMaskSet = 0x0060,
};

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


struct v3d_device {
	device_node*		node;
	mutex				lock;

	phys_addr_t			hub_physical;
	phys_addr_t			core_physical;
	phys_addr_t			pm_physical;
	phys_addr_t			asb_physical;
	uint64				interrupt;

	bool				powered;
	area_id				hub_area;
	area_id				core_area;
	volatile uint32*	hub;
	volatile uint32*	core;

	char				report[512];
	int32				report_length;
};


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


static status_t
power_on(v3d_device* device)
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


/*!	Powers the GPU on (once), masks its interrupts and records its
	identification for read().
*/
static status_t
start_gpu(v3d_device* device)
{
	if (device->powered)
		return B_OK;

	status_t status = power_on(device);
	if (status != B_OK)
		return status;

	status = map_registers("v3d hub", device->hub_physical, device->hub_area,
		device->hub);
	if (status != B_OK)
		return status;
	status = map_registers("v3d core", device->core_physical,
		device->core_area, device->core);
	if (status != B_OK) {
		delete_area(device->hub_area);
		return status;
	}

	volatile uint32* hub = device->hub;
	volatile uint32* core = device->core;

	// No interrupts until there is a handler for them
	core[kCoreInterruptMaskSet / 4] = ~0u;
	core[kCoreInterruptClear / 4] = ~0u;
	hub[kHubInterruptMaskSet / 4] = ~0u;
	hub[kHubInterruptClear / 4] = ~0u;

	uint32 rate = 0, measured = 0;
	firmware_call(BCM2835_TAG_GET_CLOCK_RATE, kV3dClock, 0, 1, &rate);
	firmware_call(BCM2835_TAG_GET_CLOCK_MEASURED, kV3dClock, 0, 1, &measured);

	device->report_length = snprintf(device->report, sizeof(device->report),
		"V3D: core ident %#" B_PRIx32 "/%#" B_PRIx32 "/%#" B_PRIx32
		", hub ident %#" B_PRIx32 "/%#" B_PRIx32 "/%#" B_PRIx32 "/%#" B_PRIx32
		", uif %#" B_PRIx32 ", mmu debug %#" B_PRIx32
		", clock %" B_PRIu32 " MHz (measured %" B_PRIu32 " MHz)"
		", interrupt %" B_PRIu64 "\n",
		core[kCoreIdent0 / 4], core[kCoreIdent1 / 4], core[kCoreIdent2 / 4],
		hub[kHubIdent0 / 4], hub[kHubIdent1 / 4], hub[kHubIdent2 / 4],
		hub[kHubIdent3 / 4], hub[kHubUifConfig / 4],
		hub[kHubMmuDebugInfo / 4], rate / 1000000, measured / 1000000,
		device->interrupt);
	TRACE_ALWAYS("%s", device->report);

	device->powered = true;
	return B_OK;
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

	MutexLocker locker(device->lock);
	status_t status = start_gpu(device);
	if (status != B_OK)
		return status;

	*_cookie = device;
	return B_OK;
}


static status_t
v3d_close(void* cookie)
{
	return B_OK;
}


static status_t
v3d_free(void* cookie)
{
	return B_OK;
}


static status_t
v3d_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	v3d_device* device = (v3d_device*)cookie;
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


static status_t
v3d_control(void* cookie, uint32 op, void* arg, size_t length)
{
	return B_DEV_INVALID_IOCTL;
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
	mutex_init(&device->lock, "v3d");

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
