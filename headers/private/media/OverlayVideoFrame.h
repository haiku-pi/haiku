/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _OVERLAY_VIDEO_FRAME_H
#define _OVERLAY_VIDEO_FRAME_H


#include <GraphicsDefs.h>
#include <OS.h>


/*!	A decoded video frame that stays in the decoder's memory, to be shown on
	an overlay as it is: what a decoder writes into the caller's buffer
	instead of the pixels when its output color space is one that only
	overlays show (B_YUV420_SAND128, B_YUV420_SAND128_10). The consumer
	shows it with an overlay bitmap of that memory:
	BBitmap(area, offset, BRect(0, 0, width - 1, height - 1),
	B_BITMAP_WILL_OVERLAY, space, bytes_per_row).

	The decoder keeps the memory unchanged until it has put out
	B_OVERLAY_VIDEO_FRAME_HOLD more frames: a consumer shows a frame before
	that (MediaPlayer: at most three frames queued, one shown, one being
	replaced on the screen).
*/
struct overlay_video_frame {
	uint32		magic;			// B_OVERLAY_VIDEO_FRAME_MAGIC
	uint32		size;			// of this structure
	area_id		area;			// in the decoder's (and consumer's) team
	uint32		offset;
	uint32		width;			// of the picture: the bitmap's size
	uint32		height;
	uint32		bytes_per_row;	// the bitmap's
	color_space	space;
	uint32		serial;			// counts the decoder's frames
	uint32		_reserved[3];
};

#define B_OVERLAY_VIDEO_FRAME_MAGIC	'OVfr'
#define B_OVERLAY_VIDEO_FRAME_HOLD	8


#endif	// _OVERLAY_VIDEO_FRAME_H
