/*
 * Copyright 2005-2008, Axel Dörfler, axeld@pinc-software.de. All rights reserved.
 * Copyright 2016, Jessica Hamilton, jessica.l.hamilton@gmail.com.
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef BCM2711_DISPLAY_ACCELERANT_H
#define BCM2711_DISPLAY_ACCELERANT_H


#include <bcm2711_display.h>


typedef struct accelerant_info {
	int					device;
	bool				is_clone;

	area_id				shared_info_area;
	bcm2711_display_shared_info	*shared_info;

	area_id				mode_list_area;
		// cloned list of standard display modes
	display_mode		*mode_list;
	uint16				current_mode;

	area_id				frame_buffer_area;
		// cloned framebuffer
	void*				frame_buffer;
} accelerant_info;

extern accelerant_info *gInfo;

extern status_t create_mode_list(void);

#endif	/* BCM2711_DISPLAY_ACCELERANT_H */
