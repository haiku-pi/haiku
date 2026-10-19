/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Temperature of the Raspberry Pi's SoC, and whether the VideoCore firmware
	throttles it (for heat or under-voltage). Both come from the firmware,
	which owns the sensor, through its mailbox property interface.
*/


#include <Drivers.h>
#include <Errors.h>
#include <KernelExport.h>
#include <bus/FDT.h>
#include <kernel.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <bcm2835_mailbox.h>

#include "bcm2835_thermal.h"


#define BCM2835_THERMAL_MODULE_NAME "drivers/power/bcm2835_thermal/driver_v1"
#define BCM2835_THERMAL_DEVICE_MODULE_NAME \
	"drivers/power/bcm2835_thermal/device_v1"
#define BCM2835_THERMAL_BASENAME "power/bcm2835_thermal/%d"
#define BCM2835_THERMAL_PATHID_GENERATOR "bcm2835_thermal/path_id"

#define ERROR(x...)	dprintf("bcm2835_thermal: " x)


// GET_THROTTLED bits: the conditions now (low half) and since boot (high)
enum {
	kThrottledUnderVoltage		= 1 << 0,
	kThrottledArmFrequencyCapped = 1 << 1,
	kThrottledThrottled			= 1 << 2,
	kThrottledSoftTempLimit		= 1 << 3,
	kThrottledSinceBootShift	= 16,
};


static device_manager_info* sDeviceManager;
static bcm2835_mailbox_module_info* sMailbox;


static status_t
firmware_value(uint32 tag, uint32 id, uint32& value)
{
	// [id] -> [id, value]; GET_THROTTLED takes and returns one word
	uint32 values[2] = { id, 0 };
	uint32 requestSize = tag == BCM2835_TAG_GET_THROTTLED ? 0 : sizeof(uint32);
	status_t status = sMailbox->property_tag(tag, values, sizeof(values),
		requestSize, NULL);
	if (status != B_OK)
		return status;
	value = tag == BCM2835_TAG_GET_THROTTLED ? values[0] : values[1];
	return B_OK;
}


/*!	The board, from the firmware's revision code (new style: bit 23 set; see
	the Raspberry Pi documentation's "revision codes"), e.g. 0xc03111:
	"Raspberry Pi 4 Model B rev 1.1, 4 GB, BCM2711".
*/
static void
describe_board(char* buffer, size_t size)
{
	uint32 values[1] = {};
	if (sMailbox->property_tag(BCM2835_TAG_GET_BOARD_REVISION, values,
			sizeof(values), 0, NULL) != B_OK) {
		snprintf(buffer, size, "unknown");
		return;
	}
	const uint32 code = values[0];
	if ((code & (1 << 23)) == 0) {
		snprintf(buffer, size, "revision %#" B_PRIx32, code);
		return;
	}

	const uint32 type = (code >> 4) & 0xff;
	const uint32 processor = (code >> 12) & 0xf;
	const uint32 memory = (code >> 20) & 0x7;
	const char* typeName = type == 0x11 ? "4 Model B" : type == 0x13 ? "400"
		: type == 0x14 ? "Compute Module 4" : type == 0x17 ? "5" : NULL;
	static const char* const kProcessors[] = { "BCM2835", "BCM2836", "BCM2837",
		"BCM2711", "BCM2712" };
	char typeBuffer[16];
	if (typeName == NULL) {
		snprintf(typeBuffer, sizeof(typeBuffer), "type %#" B_PRIx32, type);
		typeName = typeBuffer;
	}
	snprintf(buffer, size, "Raspberry Pi %s rev 1.%" B_PRIu32 ", %" B_PRIu32
		" MB, %s (%#" B_PRIx32 ")", typeName, code & 0xf, 256u << memory,
		processor < B_COUNT_OF(kProcessors) ? kProcessors[processor] : "?",
		code);
}


static status_t
get_thermal(bcm2835_thermal_type& info)
{
	uint32 temperature, maxTemperature, throttled;
	status_t status = firmware_value(BCM2835_TAG_GET_TEMPERATURE, 0,
		temperature);
	if (status == B_OK) {
		status = firmware_value(BCM2835_TAG_GET_MAX_TEMPERATURE, 0,
			maxTemperature);
	}
	if (status == B_OK)
		status = firmware_value(BCM2835_TAG_GET_THROTTLED, 0, throttled);
	if (status != B_OK)
		return status;

	// the firmware reports thousandths of a degree Celsius
	info.current_temp = temperature / 100;
	info.critical_temp = maxTemperature / 100;
	info.throttled = throttled;
	return B_OK;
}


//	#pragma mark - device module API


static status_t
bcm2835_thermal_init_device(void* driverCookie, void** _cookie)
{
	*_cookie = driverCookie;
	return B_OK;
}


static void
bcm2835_thermal_uninit_device(void* cookie)
{
}


static status_t
bcm2835_thermal_open(void* deviceCookie, const char* path, int openMode,
	void** _cookie)
{
	if ((openMode & O_RWMASK) != O_RDONLY)
		return B_NOT_ALLOWED;

	*_cookie = deviceCookie;
	return B_OK;
}


static status_t
bcm2835_thermal_close(void* cookie)
{
	return B_OK;
}


static status_t
bcm2835_thermal_free(void* cookie)
{
	return B_OK;
}


static status_t
bcm2835_thermal_read(void* cookie, off_t position, void* buffer,
	size_t* _length)
{
	if (position != 0) {
		*_length = 0;
		return B_OK;
	}

	bcm2835_thermal_type info;
	status_t status = get_thermal(info);
	if (status != B_OK)
		return status;

	// the clocks the firmware runs now (it lowers them when hot)
	const uint32 clockIDs[3] = { BCM2835_CLOCK_ARM, BCM2835_CLOCK_CORE,
		BCM2835_CLOCK_V3D };
	uint32 clocks[3] = {};
	for (uint32 i = 0; i < 3; i++)
		firmware_value(BCM2835_TAG_GET_CLOCK_MEASURED, clockIDs[i], clocks[i]);

	char board[96];
	describe_board(board, sizeof(board));

	const uint32 now = info.throttled & 0xffff;
	const uint32 sinceBoot = info.throttled >> kThrottledSinceBootShift;
	char text[384];
	int32 length = snprintf(text, sizeof(text),
		"  Board: %s\n"
		"  Current Temperature: %" B_PRIu32 ".%" B_PRIu32 " C\n"
		"  Critical Temperature: %" B_PRIu32 ".%" B_PRIu32 " C\n"
		"  Throttled: %#" B_PRIx32 "%s%s%s%s (since boot: %#" B_PRIx32 ")\n"
		"  Clocks: ARM %" B_PRIu32 " MHz, core %" B_PRIu32 " MHz, V3D %"
		B_PRIu32 " MHz\n",
		board, info.current_temp / 10, info.current_temp % 10,
		info.critical_temp / 10, info.critical_temp % 10,
		info.throttled,
		(now & kThrottledUnderVoltage) != 0 ? ", under-voltage" : "",
		(now & kThrottledArmFrequencyCapped) != 0 ? ", frequency capped" : "",
		(now & kThrottledThrottled) != 0 ? ", throttled" : "",
		(now & kThrottledSoftTempLimit) != 0 ? ", soft temperature limit" : "",
		sinceBoot, clocks[0] / 1000000, clocks[1] / 1000000,
		clocks[2] / 1000000);

	length = min_c((int32)*_length, length + 1);
	if (user_memcpy(buffer, text, length) != B_OK)
		return B_BAD_ADDRESS;
	*_length = length;
	return B_OK;
}


static status_t
bcm2835_thermal_write(void* cookie, off_t position, const void* buffer,
	size_t* _length)
{
	return B_NOT_ALLOWED;
}


static status_t
bcm2835_thermal_control(void* cookie, uint32 op, void* arg, size_t length)
{
	switch (op) {
		case drvOpGetThermalType:
		{
			bcm2835_thermal_type info;
			status_t status = get_thermal(info);
			if (status != B_OK)
				return status;
			if (!IS_USER_ADDRESS(arg)) {
				memcpy(arg, &info, sizeof(info));
				return B_OK;
			}
			return user_memcpy(arg, &info, sizeof(info));
		}
	}
	return B_DEV_INVALID_IOCTL;
}


//	#pragma mark - driver module API


static float
bcm2835_thermal_support(device_node* parent)
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

	return strcmp(compatible, "brcm,bcm2711-thermal") == 0
		|| strcmp(compatible, "brcm,bcm2835-thermal") == 0 ? 0.6f : 0.0f;
}


static status_t
bcm2835_thermal_register_device(device_node* node)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Raspberry Pi Thermal" } },
		{ NULL }
	};

	return sDeviceManager->register_node(node, BCM2835_THERMAL_MODULE_NAME,
		attrs, NULL, NULL);
}


static status_t
bcm2835_thermal_init_driver(device_node* node, void** _driverCookie)
{
	*_driverCookie = node;
	return B_OK;
}


static void
bcm2835_thermal_uninit_driver(void* driverCookie)
{
}


static status_t
bcm2835_thermal_register_child_devices(void* driverCookie)
{
	device_node* node = (device_node*)driverCookie;

	int pathID = sDeviceManager->create_id(BCM2835_THERMAL_PATHID_GENERATOR);
	if (pathID < 0) {
		ERROR("couldn't create a path ID\n");
		return B_ERROR;
	}

	char name[B_DEV_NAME_LENGTH];
	snprintf(name, sizeof(name), BCM2835_THERMAL_BASENAME, pathID);
	return sDeviceManager->publish_device(node, name,
		BCM2835_THERMAL_DEVICE_MODULE_NAME);
}


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{ BCM2835_MAILBOX_MODULE_NAME, (module_info**)&sMailbox },
	{}
};


driver_module_info bcm2835_thermal_driver_module = {
	{
		BCM2835_THERMAL_MODULE_NAME,
		0,
		NULL
	},

	bcm2835_thermal_support,
	bcm2835_thermal_register_device,
	bcm2835_thermal_init_driver,
	bcm2835_thermal_uninit_driver,
	bcm2835_thermal_register_child_devices,
	NULL,	// rescan
	NULL,	// removed
};


struct device_module_info bcm2835_thermal_device_module = {
	{
		BCM2835_THERMAL_DEVICE_MODULE_NAME,
		0,
		NULL
	},

	bcm2835_thermal_init_device,
	bcm2835_thermal_uninit_device,
	NULL,

	bcm2835_thermal_open,
	bcm2835_thermal_close,
	bcm2835_thermal_free,
	bcm2835_thermal_read,
	bcm2835_thermal_write,
	NULL,
	bcm2835_thermal_control,

	NULL,
	NULL
};


module_info* modules[] = {
	(module_info*)&bcm2835_thermal_driver_module,
	(module_info*)&bcm2835_thermal_device_module,
	NULL
};
