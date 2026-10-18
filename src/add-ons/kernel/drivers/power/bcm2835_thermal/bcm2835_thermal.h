/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _BCM2835_THERMAL_H
#define _BCM2835_THERMAL_H


#include <Drivers.h>


enum { /* ioctl op-codes */
	drvOpGetThermalType = B_DEVICE_OP_CODES_END + 10001,
};


struct bcm2835_thermal_type {
	/* Required fields for thermal devices */
	uint32 current_temp;
		// tenths of a degree Celsius
	uint32 critical_temp;
		// the firmware's throttling limit, tenths of a degree Celsius

	uint32 throttled;
		// the firmware's GET_THROTTLED bits
};


#endif // _BCM2835_THERMAL_H
