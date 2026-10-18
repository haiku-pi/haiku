/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef BCM2711_DISPLAY_H
#define BCM2711_DISPLAY_H


#include <Accelerant.h>
#include <Drivers.h>
#include <edid_raw.h>


#define BCM2711_DISPLAY_ACCELERANT_NAME "bcm2711_display.accelerant"


/*!	Shared between the Raspberry Pi 4 display driver and its accelerant: the
	display the VideoCore firmware scans out, set up by the boot loader.
*/
struct bcm2711_display_shared_info {
	area_id			mode_list_area;
	uint32			mode_count;
	display_mode	current_mode;
	uint32			bytes_per_row;

	edid1_raw		edid_raw;
		// as the firmware read it from the monitor (decoded by the accelerant)
	bool			has_edid;

	// released at every vertical blank (all waiters)
	sem_id			vblank_semaphore;
	// vertical blanks so far, and where they come from
	int64			vblank_count;
	bool			vblank_from_interrupt;
		// the firmware's SMI interrupt; otherwise a thread waiting for the
		// firmware's vertical sync (the SET_VSYNC property)
};


enum {
	BCM2711_DISPLAY_GET_PRIVATE_DATA = B_DEVICE_OP_CODES_END + 1,
	BCM2711_DISPLAY_CLONE_FRAME_BUFFER,
	BCM2711_DISPLAY_GET_DEVICE_NAME,
};


#endif	// BCM2711_DISPLAY_H
