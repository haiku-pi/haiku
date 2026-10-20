/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The BCM2711's hardware random number generator (RNG200, as in Broadcom's
	iProc SoCs; the Raspberry Pi 4). It feeds the random module's pool, as the
	other random controllers do: a timer queues a DPC that drains the
	generator's FIFO into it.
*/


#include <new>
#include <stdlib.h>
#include <string.h>

#include <KernelExport.h>
#include <bus/FDT.h>
#include <dpc.h>

#include "random.h"


#define DRIVER_NAME "bcm2711_rng200"
#define TRACE_ALWAYS(x...)	dprintf(DRIVER_NAME ": " x)
#define ERROR(x...)			dprintf(DRIVER_NAME ": " x)

#define BCM2711_RNG200_MODULE_NAME "busses/random/bcm2711_rng200/driver_v1"

static const char* const kCompatible = "brcm,bcm2711-rng200";

// registers
enum {
	RNG_CTRL		= 0x00,
	RNG_SOFT_RESET	= 0x04,
	RBG_SOFT_RESET	= 0x08,
	RNG_INT_STATUS	= 0x18,
	RNG_FIFO_DATA	= 0x20,
	RNG_FIFO_COUNT	= 0x24,
};

#define RNG_CTRL_RBGEN_MASK		0x00001fff
#define RNG_CTRL_RBGEN_ENABLE	0x00000001
#define RNG_SOFT_RESET_BIT		0x00000001
#define RNG_FIFO_COUNT_MASK		0x000000ff
#define RNG_INT_STATUS_ERRORS	0x80000020
	// master fail lockout, NIST failure

// how much to take from the FIFO (32-bit words), and how often
static const uint32 kWordsPerRun = 16;
static const bigtime_t kInterval = 1000000;


struct rng200_device {
	device_node*	node;
	area_id			registersArea;
	volatile uint32* registers;
	void*			dpcHandle;
	timer			extractTimer;
	uint32			restarts;
};


static device_manager_info* sDeviceManager;
static random_for_controller_interface* sRandom;
static dpc_module_info* sDPC;


static inline uint32
read_reg(rng200_device* device, uint32 offset)
{
	return device->registers[offset / 4];
}


static inline void
write_reg(rng200_device* device, uint32 offset, uint32 value)
{
	device->registers[offset / 4] = value;
}


static void
set_enabled(rng200_device* device, bool enabled)
{
	uint32 control = read_reg(device, RNG_CTRL) & ~RNG_CTRL_RBGEN_MASK;
	if (enabled)
		control |= RNG_CTRL_RBGEN_ENABLE;
	write_reg(device, RNG_CTRL, control);
}


/*!	Disables the generator, clears its interrupt status, resets both of its
	parts and enables it again (Linux's iproc_rng200_restart()).
*/
static void
restart(rng200_device* device)
{
	set_enabled(device, false);
	write_reg(device, RNG_INT_STATUS, 0xffffffff);

	write_reg(device, RBG_SOFT_RESET,
		read_reg(device, RBG_SOFT_RESET) | RNG_SOFT_RESET_BIT);
	write_reg(device, RNG_SOFT_RESET,
		read_reg(device, RNG_SOFT_RESET) | RNG_SOFT_RESET_BIT);
	write_reg(device, RNG_SOFT_RESET,
		read_reg(device, RNG_SOFT_RESET) & ~RNG_SOFT_RESET_BIT);
	write_reg(device, RBG_SOFT_RESET,
		read_reg(device, RBG_SOFT_RESET) & ~RNG_SOFT_RESET_BIT);

	set_enabled(device, true);
}


static void
extract(void* cookie)
{
	rng200_device* device = (rng200_device*)cookie;

	if ((read_reg(device, RNG_INT_STATUS) & RNG_INT_STATUS_ERRORS) != 0) {
		// the generator stopped itself after a failed health test
		if (device->restarts++ < 10)
			ERROR("health test failed, restarting\n");
		restart(device);
		return;
	}

	uint32 words = read_reg(device, RNG_FIFO_COUNT) & RNG_FIFO_COUNT_MASK;
	if (words > kWordsPerRun)
		words = kWordsPerRun;
	for (; words >= 2; words -= 2) {
		uint64 value = read_reg(device, RNG_FIFO_DATA);
		value |= (uint64)read_reg(device, RNG_FIFO_DATA) << 32;
		sRandom->queue_randomness(value);
	}
}


static int32
extract_timer(timer* timer)
{
	rng200_device* device = (rng200_device*)timer->user_data;
	sDPC->queue_dpc(device->dpcHandle, extract, device);
	return B_HANDLED_INTERRUPT;
}


//	#pragma mark - driver module API


static float
rng200_supports_device(device_node* parent)
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
	return strcmp(compatible, kCompatible) == 0 ? 0.6f : 0.0f;
}


static status_t
rng200_register_device(device_node* node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Broadcom BCM2711 random number generator" } },
		{ B_DEVICE_FLAGS, B_UINT32_TYPE, { .ui32 = B_KEEP_DRIVER_LOADED } },
		{ NULL }
	};

	return sDeviceManager->register_node(node, BCM2711_RNG200_MODULE_NAME,
		attrs, NULL, NULL);
}


static status_t
rng200_init_driver(device_node* node, void** _driverCookie)
{
	device_node* parent = sDeviceManager->get_parent_node(node);
	fdt_device_module_info* fdt;
	fdt_device* fdtDevice;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&fdtDevice);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	uint64 address, size;
	if (!fdt->get_reg(fdtDevice, 0, &address, &size) || size < 0x28)
		return B_DEVICE_NOT_FOUND;

	rng200_device* device = new(std::nothrow) rng200_device;
	if (device == NULL)
		return B_NO_MEMORY;
	memset(device, 0, sizeof(*device));
	device->node = node;

	device->registersArea = map_physical_memory("bcm2711 rng200 registers",
		address, B_PAGE_SIZE, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
		(void**)&device->registers);
	if (device->registersArea < 0) {
		status = device->registersArea;
		delete device;
		return status;
	}

	restart(device);

	// it takes a moment to produce its first words
	uint32 words = 0;
	for (int i = 0; i < 100 && words == 0; i++) {
		snooze(1000);
		words = read_reg(device, RNG_FIFO_COUNT) & RNG_FIFO_COUNT_MASK;
	}
	if (words == 0) {
		ERROR("the generator produces nothing\n");
		set_enabled(device, false);
		delete_area(device->registersArea);
		delete device;
		return B_DEVICE_NOT_FOUND;
	}

	status = sDPC->new_dpc_queue(&device->dpcHandle, "rng200",
		B_LOW_PRIORITY);
	if (status != B_OK) {
		delete_area(device->registersArea);
		delete device;
		return status;
	}

	device->extractTimer.user_data = device;
	add_timer(&device->extractTimer, extract_timer, kInterval,
		B_PERIODIC_TIMER);

	TRACE_ALWAYS("at %#" B_PRIx64 ", %" B_PRIu32 " words ready, feeding the "
		"random pool\n", address, words);
	*_driverCookie = device;
	return B_OK;
}


static void
rng200_uninit_driver(void* driverCookie)
{
	rng200_device* device = (rng200_device*)driverCookie;
	cancel_timer(&device->extractTimer);
	sDPC->delete_dpc_queue(device->dpcHandle);
	set_enabled(device, false);
	delete_area(device->registersArea);
	delete device;
}


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{ RANDOM_FOR_CONTROLLER_MODULE_NAME, (module_info**)&sRandom },
	{ B_DPC_MODULE_NAME, (module_info**)&sDPC },
	{}
};


static driver_module_info sRng200DriverModule = {
	{
		BCM2711_RNG200_MODULE_NAME,
		0,
		NULL
	},

	rng200_supports_device,
	rng200_register_device,
	rng200_init_driver,
	rng200_uninit_driver,
	NULL,	// register child devices
	NULL,	// rescan
	NULL,	// device removed
};


module_info* modules[] = {
	(module_info*)&sRng200DriverModule,
	NULL
};
