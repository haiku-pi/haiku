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
	bigtime_t		vblank_time;
		// system_time() of the last vertical blank

	uint32			overlay_planes;
		// firmware planes available for overlays (0: no overlay support)
	bool			overlays_keyed;
		// the desktop covers the overlays, transparent where its alpha is 0
		// (B_OVERLAY_KEYING_USES_ALPHA); else they lie on top of it
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
	BCM2711_DISPLAY_ALLOCATE_OVERLAY_BUFFER,
		// struct bcm2711_display_overlay_buffer
	BCM2711_DISPLAY_FREE_OVERLAY_BUFFER,
		// an area_id; a plane showing the buffer is hidden first
	BCM2711_DISPLAY_SET_OVERLAY,
		// struct bcm2711_display_overlay
	BCM2711_DISPLAY_HIDE_OVERLAY,
		// a uint32: the plane
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


/*!	An overlay buffer: physically contiguous, scanned out by the firmware,
	write-combining. A kernel area that every team may access (the overlay API
	hands its address to the clients as it is), and clone.
*/
struct bcm2711_display_overlay_buffer {
	uint32			size;		// in: bytes
	area_id			area;		// out
	void*			address;	// out: valid in every team
	uint32			physical;	// out: below 1 GiB
};


#define BCM2711_DISPLAY_FIRST_OVERLAY_PLANE	1
#define BCM2711_DISPLAY_OVERLAY_PLANES		6

// the firmware's image types (VC_IMAGE_*) of the overlay color spaces
#define BCM2711_DISPLAY_IMAGE_YUV420		3
	// planar: Y, then U and V of half the pitch and height
#define BCM2711_DISPLAY_IMAGE_XRGB8888		44
#define BCM2711_DISPLAY_IMAGE_YUV422_YUYV	45
#define BCM2711_DISPLAY_IMAGE_YUV_UV		19
	// SAND128 NV12 (the HEVC decoder's output; experimental): columns of 128
	// bytes, bytes_per_row is the column height in lines, the chroma at
	// chroma_offset

// the firmware's color encodings of YUV images
#define BCM2711_DISPLAY_ENCODING_BT601		1
#define BCM2711_DISPLAY_ENCODING_BT709		2


/*!	Shows a buffer on an overlay plane:  source (16.16 fixed point, in
	buffer pixels) scaled to  destination (the current mode's coordinates;
	clipped to the screen by the driver).
*/
struct bcm2711_display_overlay {
	uint32			plane;
		// BCM2711_DISPLAY_FIRST_OVERLAY_PLANE ... + OVERLAY_PLANES - 1
	uint32			image_type;
	uint32			color_encoding;
	uint32			physical;	// the buffer (all of its planes)
	uint16			width;		// of the buffer, in pixels
	uint16			height;
	uint32			bytes_per_row;
		// of the (first) plane
	uint32			chroma_offset;
		// BCM2711_DISPLAY_IMAGE_YUV_UV: from physical
	uint32			vpitch;
		// 0: the height (the firmware's "vpitch"; its meaning for SAND is
		// unknown)
	uint32			source_x;
	uint32			source_y;
	uint32			source_width;
	uint32			source_height;
	int32			x;
	int32			y;
	uint32			destination_width;
	uint32			destination_height;
};


#endif	// BCM2711_DISPLAY_H
