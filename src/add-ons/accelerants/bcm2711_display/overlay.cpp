/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Overlays: firmware planes 1-6, showing buffers the driver allocates.

	The firmware has no color key. It blends planes with per-pixel alpha
	instead: while an overlay is visible, the driver scans the desktop out
	above the overlays, and app_server keeps the desktop opaque except where
	it holds the overlay's key color (B_OVERLAY_KEYING_USES_ALPHA).

	Color spaces: B_RGB32/B_RGBA32, and B_YUV420 as planar I420 (what video
	decoders put out): Y, then Cb and Cr of half the width and height. Its
	bytes_per_row is that of the whole picture, as for any B_YUV420 bitmap
	(3 bytes per 2 pixels, so that bytes_per_row * height is its size): the
	luma rows are 2/3 of it, the chroma rows 1/3. The display's scaler has no
	packed 4:2:2 format, so B_YCbCr422 is not offered.
*/


#include "accelerant_protos.h"
#include "accelerant.h"

#include <errno.h>
#include <new>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


struct overlay {
	overlay_buffer	buffer;
	area_id			area;
	uint32			physical;
	uint32			image_type;
	uint32			pitch;
		// of the first plane: the firmware derives the chroma planes' pitch
};

// A token is the address of its plane's slot. app_server may release a
// token twice (an overlay that handed its channel over keeps the token), and
// keep configuring with it: a released plane is taken again by configuring.
struct plane_slot {
	bool			used;
};

static plane_slot sPlanes[BCM2711_DISPLAY_OVERLAY_PLANES];
static pthread_mutex_t sLock = PTHREAD_MUTEX_INITIALIZER;

static const uint32 kSupportedSpaces[] = {
	B_RGB32, B_RGBA32, B_YUV420, 0
};


static uint32
plane_for_token(overlay_token token)
{
	plane_slot* slot = (plane_slot*)token;
	if (slot < sPlanes || slot >= sPlanes + BCM2711_DISPLAY_OVERLAY_PLANES)
		return 0;
	return BCM2711_DISPLAY_FIRST_OVERLAY_PLANE + (slot - sPlanes);
}


static void
hide(uint32 plane)
{
	ioctl(gInfo->device, BCM2711_DISPLAY_HIDE_OVERLAY, &plane, sizeof(plane));
}


//	#pragma mark - public accelerant functions


uint32
bcm2711_overlay_count(const display_mode* mode)
{
	return gInfo->shared_info->overlay_planes;
}


const uint32*
bcm2711_overlay_supported_spaces(const display_mode* mode)
{
	return kSupportedSpaces;
}


uint32
bcm2711_overlay_supported_features(uint32 colorSpace)
{
	// the firmware filters whenever it scales
	uint32 features = B_OVERLAY_COLOR_KEY | B_OVERLAY_HORIZONTAL_FILTERING
		| B_OVERLAY_VERTICAL_FILTERING;
	if (gInfo->shared_info->overlays_keyed)
		features |= B_OVERLAY_KEYING_USES_ALPHA;
	return features;
}


const overlay_buffer*
bcm2711_allocate_overlay_buffer(color_space colorSpace, uint16 width,
	uint16 height)
{
	uint32 imageType;
	switch (colorSpace) {
		case B_RGB32:
		case B_RGBA32:
			// the alpha channel is ignored: frames are opaque
			imageType = BCM2711_DISPLAY_IMAGE_XRGB8888;
			break;
		case B_YUV420:
			imageType = BCM2711_DISPLAY_IMAGE_YUV420;
			break;
		default:
			return NULL;
	}
	if (width < 2 || height < 2 || gInfo->shared_info->overlay_planes == 0)
		return NULL;

	// 64-byte rows (the chroma rows of 4:2:0 are still 32-byte aligned)
	uint32 pitch;
	uint32 bytesPerRow;
	if (colorSpace == B_YUV420) {
		width = (width + 1) & ~1;
		height = (height + 1) & ~1;
		pitch = (width + 63) & ~63;
		bytesPerRow = pitch * 3 / 2;
	} else {
		pitch = (width * 4 + 63) & ~63;
		bytesPerRow = pitch;
	}
	const uint32 size = bytesPerRow * height;

	overlay* info = new(std::nothrow) overlay;
	if (info == NULL)
		return NULL;

	bcm2711_display_overlay_buffer request = {};
	request.size = size;
	if (ioctl(gInfo->device, BCM2711_DISPLAY_ALLOCATE_OVERLAY_BUFFER, &request,
			sizeof(request)) != 0) {
		delete info;
		return NULL;
	}

	info->buffer.space = colorSpace;
	info->buffer.width = width;
	info->buffer.height = height;
	info->buffer.bytes_per_row = bytesPerRow;
	info->buffer.buffer = request.address;
	info->buffer.buffer_dma = (void*)(addr_t)request.physical;
	info->area = request.area;
	info->physical = request.physical;
	info->image_type = imageType;
	info->pitch = pitch;
	return &info->buffer;
}


status_t
bcm2711_release_overlay_buffer(const overlay_buffer* buffer)
{
	if (buffer == NULL)
		return B_BAD_VALUE;

	overlay* info = (overlay*)buffer;
	// (the driver hides a plane showing it)
	status_t status = ioctl(gInfo->device,
		BCM2711_DISPLAY_FREE_OVERLAY_BUFFER, &info->area, sizeof(info->area));
	delete info;
	return status == 0 ? B_OK : errno;
}


status_t
bcm2711_get_overlay_constraints(const display_mode* mode,
	const overlay_buffer* buffer, overlay_constraints* constraints)
{
	if (mode == NULL || buffer == NULL || constraints == NULL)
		return B_BAD_VALUE;

	memset(constraints, 0, sizeof(overlay_constraints));

	// the source rectangle, in buffer pixels: 4:2:x chroma comes in pairs
	const bool yuv = buffer->space == B_YUV420;
	constraints->view.h_alignment = yuv ? 1 : 0;
	constraints->view.v_alignment = yuv ? 1 : 0;
	constraints->view.width_alignment = yuv ? 1 : 0;
	constraints->view.height_alignment = yuv ? 1 : 0;
	constraints->view.width.min = 2;
	constraints->view.width.max = buffer->width;
	constraints->view.height.min = 2;
	constraints->view.height.max = buffer->height;

	// the destination, in the mode's coordinates
	constraints->window.width.min = 2;
	constraints->window.width.max = mode->virtual_width;
	constraints->window.height.min = 2;
	constraints->window.height.max = mode->virtual_height;

	// the scaler's range (Linux allows down to 1/8)
	constraints->h_scale.min = 1.0f / 8;
	constraints->h_scale.max = 16.0f;
	constraints->v_scale.min = 1.0f / 8;
	constraints->v_scale.max = 16.0f;
	return B_OK;
}


overlay_token
bcm2711_allocate_overlay(void)
{
	if (gInfo->shared_info->overlay_planes == 0)
		return NULL;

	pthread_mutex_lock(&sLock);
	overlay_token token = NULL;
	for (uint32 i = 0; i < BCM2711_DISPLAY_OVERLAY_PLANES; i++) {
		if (!sPlanes[i].used) {
			sPlanes[i].used = true;
			token = &sPlanes[i];
			break;
		}
	}
	pthread_mutex_unlock(&sLock);
	return token;
}


status_t
bcm2711_release_overlay(overlay_token token)
{
	const uint32 plane = plane_for_token(token);
	if (plane == 0)
		return B_BAD_VALUE;

	pthread_mutex_lock(&sLock);
	((plane_slot*)token)->used = false;
	pthread_mutex_unlock(&sLock);

	hide(plane);
	return B_OK;
}


/*!	Shows \a buffer on the token's plane, or hides the plane (\a window or
	\a view NULL). The window rectangle is in the mode's coordinates and not
	clipped (the driver clips it); the view is the part of the buffer shown.
	A new buffer in the same place is a page flip: the firmware takes the
	plane at its next update point, ~1.6 ms before the vertical blank.
*/
status_t
bcm2711_configure_overlay(overlay_token token, const overlay_buffer* buffer,
	const overlay_window* window, const overlay_view* view)
{
	const uint32 plane = plane_for_token(token);
	if (plane == 0)
		return B_BAD_VALUE;

	if (buffer == NULL || window == NULL || view == NULL) {
		hide(plane);
		return B_OK;
	}

	pthread_mutex_lock(&sLock);
	// a token released while still in use (see plane_slot)
	((plane_slot*)token)->used = true;
	pthread_mutex_unlock(&sLock);

	const overlay* info = (const overlay*)buffer;
	if (view->width == 0 || view->height == 0 || window->width == 0
		|| window->height == 0
		|| view->h_start + view->width > buffer->width
		|| view->v_start + view->height > buffer->height) {
		hide(plane);
		return B_BAD_VALUE;
	}

	bcm2711_display_overlay request = {};
	request.plane = plane;
	request.image_type = info->image_type;
	if (info->image_type != BCM2711_DISPLAY_IMAGE_XRGB8888) {
		// as video is encoded: BT.709 for HD, BT.601 for SD
		request.color_encoding = buffer->height >= 720
			? BCM2711_DISPLAY_ENCODING_BT709 : BCM2711_DISPLAY_ENCODING_BT601;
	}
	request.physical = info->physical;
	request.width = buffer->width;
	request.height = buffer->height;
	request.bytes_per_row = info->pitch;
	request.source_x = (uint32)view->h_start << 16;
	request.source_y = (uint32)view->v_start << 16;
	request.source_width = (uint32)view->width << 16;
	request.source_height = (uint32)view->height << 16;
	request.x = window->h_start + window->offset_left;
	request.y = window->v_start + window->offset_top;
	request.destination_width = window->width - window->offset_left
		- window->offset_right;
	request.destination_height = window->height - window->offset_top
		- window->offset_bottom;
	if (window->offset_left + window->offset_right >= window->width
		|| window->offset_top + window->offset_bottom >= window->height) {
		hide(plane);
		return B_OK;
	}
	if (window->offset_left != 0 || window->offset_right != 0
		|| window->offset_top != 0 || window->offset_bottom != 0) {
		// only part of the window shows: crop the view in proportion
		request.source_x += (uint64)window->offset_left
			* request.source_width / window->width;
		request.source_y += (uint64)window->offset_top
			* request.source_height / window->height;
		request.source_width = (uint64)request.source_width
			* request.destination_width / window->width;
		request.source_height = (uint64)request.source_height
			* request.destination_height / window->height;
	}

	if (ioctl(gInfo->device, BCM2711_DISPLAY_SET_OVERLAY, &request,
			sizeof(request)) != 0) {
		return errno;
	}
	return B_OK;
}
