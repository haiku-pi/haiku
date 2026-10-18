/*
 * Copyright 2005-2008, Axel Dörfler, axeld@pinc-software.de.
 * Copyright 2016, Jessica Hamilton, jessica.l.hamilton@gmail.com.
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * All rights reserved. Distributed under the terms of the MIT License.
 */
#ifndef _ACCELERANT_PROTOS_H
#define _ACCELERANT_PROTOS_H


#include <Accelerant.h>
#include "video_overlay.h"


#ifdef __cplusplus
extern "C" {
#endif

// general
status_t bcm2711_init_accelerant(int fd);
ssize_t bcm2711_accelerant_clone_info_size(void);
void bcm2711_get_accelerant_clone_info(void *data);
status_t bcm2711_clone_accelerant(void *data);
void bcm2711_uninit_accelerant(void);
status_t bcm2711_get_accelerant_device_info(accelerant_device_info *adi);
sem_id bcm2711_accelerant_retrace_semaphore(void);

// modes & constraints
uint32 bcm2711_accelerant_mode_count(void);
status_t bcm2711_get_mode_list(display_mode *dm);
status_t bcm2711_set_display_mode(display_mode *modeToSet);
status_t bcm2711_get_display_mode(display_mode *currentMode);
status_t bcm2711_get_edid_info(void *info, size_t size, uint32 *_version);
status_t bcm2711_get_frame_buffer_config(frame_buffer_config *config);
status_t bcm2711_get_pixel_clock_limits(display_mode *dm, uint32 *low,
	uint32 *high);

#ifdef __cplusplus
}
#endif

#endif	/* _ACCELERANT_PROTOS_H */
