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

	// the mode the firmware drives the monitor with; smaller ones are scaled
	// up to it by a firmware plane
	uint32			native_width;
	uint32			native_height;

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
	BCM2711_DISPLAY_SET_MODE,
		// struct bcm2711_display_mode; the frame buffer changes, so clone it
		// again afterwards
	BCM2711_DISPLAY_SET_CURSOR,
		// struct bcm2711_display_cursor
	BCM2711_DISPLAY_MOVE_CURSOR,
		// struct bcm2711_display_cursor_position
	BCM2711_DISPLAY_SHOW_CURSOR,
		// a bool
};


#define BCM2711_DISPLAY_CURSOR_SIZE 64


/*!	A cursor image: B, G, R, A bytes (B_RGBA32), not premultiplied. */
struct bcm2711_display_cursor {
	uint16			width;
	uint16			height;
	uint16			hot_x;
	uint16			hot_y;
	uint32			bits[BCM2711_DISPLAY_CURSOR_SIZE
						* BCM2711_DISPLAY_CURSOR_SIZE];
		// width x height, rows of width pixels
};


struct bcm2711_display_cursor_position {
	uint16			x;
	uint16			y;
};


struct bcm2711_display_mode {
	uint32			width;
	uint32			height;
};


#endif	// BCM2711_DISPLAY_H
