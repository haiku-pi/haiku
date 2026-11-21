/*
 * Copyright 2026, Haiku, Inc.
 * Distributed under the terms of the MIT License.
 */
#ifndef OVERLAY_READ_BACK_H
#define OVERLAY_READ_BACK_H


#include <video_overlay.h>


/*!	What an overlay shows, for reading the screen back (a screenshot): the
	overlay hardware composes its buffer into the picture on the way to the
	display, so the frame buffer holds only the key where it shows.
*/
struct overlay_read_back {
	const overlay_buffer*	buffer;
	overlay_view			view;
	overlay_window			window;
	bool					keyed_by_alpha;
		// B_OVERLAY_KEYING_USES_ALPHA: the overlay shows where the frame
		// buffer's alpha is 0, else where it holds the key color
		// (window.flags B_OVERLAY_COLOR_KEY), else in all of its window
};


// Draws what \a overlay shows into \a pixels: B_RGB32 pixels, \a width x
// \a height of them, of the screen rectangle at \a left, \a top, holding the
// frame buffer's pixels there (which tell where the key lets the overlay
// show). Returns B_OK, or an error if the buffer's color space is unknown
// or memory is short; \a pixels are unchanged then.
status_t read_back_overlay(const overlay_read_back& overlay, uint32* pixels,
	int32 bytesPerRow, int32 left, int32 top, int32 width, int32 height);


#endif	// OVERLAY_READ_BACK_H
