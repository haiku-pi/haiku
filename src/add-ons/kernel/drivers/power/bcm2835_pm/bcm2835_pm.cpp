/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Reboot and power-off for the Raspberry Pi (BCM2835 to BCM2711), through
	the watchdog of the SoC's power management block (the device tree's
	"brcm,bcm2835-pm-wdt" node). The kernel calls it from arch_cpu_shutdown()
	when the firmware offers no PSCI, as under U-Boot on the Pi 4.

	Reboot: the watchdog is started with a timeout of a few ticks and set up
	for a full reset. Power-off is the same reset, with "partition 63" in the
	reset status register: the Raspberry Pi firmware does not boot from that
	partition but halts the board.

	The registers and both sequences are hardware facts, as NetBSD's
	bcm2835_pmwdog.c and FreeBSD's bcm2835_wdog.c (both BSD-2-Clause) and
	Linux's bcm2835_wdt.c use them; this driver was written from those facts,
	no code was copied.
*/


#include <new>
#include <string.h>

#include <KernelExport.h>
#include <bus/FDT.h>
#include <cpu.h>


#define DRIVER_NAME "bcm2835_pm"
#define TRACE_ALWAYS(x...)	dprintf(DRIVER_NAME ": " x)

#define BCM2835_PM_MODULE_NAME "drivers/power/bcm2835_pm/driver_v1"


// registers ("pm", the first "reg" entry)
enum {
	PM_RSTC	= 0x1c,		// reset control
	PM_RSTS	= 0x20,		// reset status; the firmware reads the partition
	PM_WDOG	= 0x24,		// watchdog timeout, in ticks of 1/65536 s
};

// Every write needs the password in the top byte.
static const uint32 kPassword = 0x5a000000;
static const uint32 kPasswordMask = 0xff000000;

static const uint32 kResetConfigMask = 0x00000030;
static const uint32 kResetConfigFullReset = 0x00000020;

// The partition number's six bits are spread over the even bits 0-10.
static const uint32 kPartitionMask = 0x00000555;
static const uint32 kHaltPartition = 63;

static const uint32 kResetTicks = 10;
	// about 150 us


struct pm_device {
	area_id				registersArea;
	volatile uint32*	registers;
	bool				canPowerOff;
};


static device_manager_info* sDeviceManager;
static pm_device* sDevice;


static inline uint32
partition_bits(uint32 partition)
{
	uint32 bits = 0;
	for (int bit = 0; bit < 6; bit++) {
		if ((partition & (1 << bit)) != 0)
			bits |= 1 << (bit * 2);
	}
	return bits;
}


static status_t
pm_shutdown(bool reboot)
{
	pm_device* device = sDevice;
	if (device == NULL)
		return B_ERROR;
	if (!reboot && !device->canPowerOff)
		return B_NOT_SUPPORTED;

	volatile uint32* registers = device->registers;
	cpu_status state = disable_interrupts();

	// the partition the firmware boots from next: 0, or 63 to halt
	uint32 status = registers[PM_RSTS / 4]
		& ~(kPasswordMask | kPartitionMask);
	registers[PM_RSTS / 4] = kPassword | status
		| partition_bits(reboot ? 0 : kHaltPartition);

	registers[PM_WDOG / 4] = kPassword | kResetTicks;
	uint32 control = registers[PM_RSTC / 4]
		& ~(kPasswordMask | kResetConfigMask);
	registers[PM_RSTC / 4] = kPassword | control | kResetConfigFullReset;

	// The reset comes after the few ticks; nothing should run until then.
	for (int i = 0; i < 1000; i++)
		spin(1000);

	restore_interrupts(state);
	return B_ERROR;
}


//	#pragma mark - driver module API


static float
pm_supports_device(device_node* parent)
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

	return strcmp(compatible, "brcm,bcm2711-pm") == 0
		|| strcmp(compatible, "brcm,bcm2835-pm") == 0
		|| strcmp(compatible, "brcm,bcm2835-pm-wdt") == 0 ? 0.6f : 0.0f;
}


static status_t
pm_register_device(device_node* node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Broadcom BCM2835 power management watchdog" } },
		{ B_DEVICE_FLAGS, B_UINT32_TYPE, { .ui32 = B_KEEP_DRIVER_LOADED } },
		{ NULL }
	};

	return sDeviceManager->register_node(node, BCM2835_PM_MODULE_NAME, attrs,
		NULL, NULL);
}


static status_t
pm_init_driver(device_node* node, void** _driverCookie)
{
	if (sDevice != NULL)
		return B_BUSY;

	device_node* parent = sDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&fdtDevice);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	uint64 address, size;
	if (!fdt->get_reg(fdtDevice, 0, &address, &size)
		|| size < PM_WDOG + sizeof(uint32)) {
		return B_DEVICE_NOT_FOUND;
	}

	pm_device* device = new(std::nothrow) pm_device;
	if (device == NULL)
		return B_NO_MEMORY;

	const phys_addr_t base = ROUNDDOWN(address, B_PAGE_SIZE);
	void* virtualBase;
	device->registersArea = map_physical_memory("bcm2835 pm registers", base,
		ROUNDUP(address + size, B_PAGE_SIZE) - base, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &virtualBase);
	if (device->registersArea < 0) {
		status = device->registersArea;
		delete device;
		return status;
	}
	device->registers = (volatile uint32*)((addr_t)virtualBase
		+ (address - base));

	// as Linux: only the node marked as the system's power controller may
	// power it off
	int length;
	device->canPowerOff = fdt->get_prop(fdtDevice, "system-power-controller",
		&length) != NULL;

	sDevice = device;
	arm64_set_shutdown_hook(pm_shutdown);

	TRACE_ALWAYS("at %#" B_PRIx64 ": reboot%s through the watchdog\n",
		address, device->canPowerOff ? " and power-off" : "");
	*_driverCookie = device;
	return B_OK;
}


static void
pm_uninit_driver(void* driverCookie)
{
	pm_device* device = (pm_device*)driverCookie;
	arm64_set_shutdown_hook(NULL);
	sDevice = NULL;
	delete_area(device->registersArea);
	delete device;
}


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{}
};


static driver_module_info sPmDriverModule = {
	{
		BCM2835_PM_MODULE_NAME,
		0,
		NULL
	},

	pm_supports_device,
	pm_register_device,
	pm_init_driver,
	pm_uninit_driver,
	NULL,	// register child devices
	NULL,	// rescan
	NULL,	// device removed
};


module_info* modules[] = {
	(module_info*)&sPmDriverModule,
	NULL
};
