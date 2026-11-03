/*
 * Copyright 2026, Haiku, Inc.
 * Distributed under the terms of the MIT License.
 */
#ifndef OVERLAY_KEYING_H
#define OVERLAY_KEYING_H


#include <SupportDefs.h>


/*!	Keying overlays by alpha (B_OVERLAY_KEYING_USES_ALPHA): the display blends
	the desktop over the overlays by the alpha of its pixels. The desktop is
	written opaque, except the key color inside an overlay's rectangle, which
	becomes transparent: 0x00000000, nothing whether the display takes alpha as
	premultiplied or not.
*/
struct overlay_key_rect {
	int32	left;
	int32	top;
	int32	right;
	int32	bottom;
	uint32	key;
		// the key color as a B_RGB32 pixel without alpha: 0x00RRGGBB
};


// copies \a count pixels, alpha set to 0xff
void copy_row_opaque(uint32* destination, const uint32* source, int32 count);

// copies \a count pixels: the key color transparent, the others opaque
void copy_row_keyed(uint32* destination, const uint32* source, int32 count,
	uint32 key);

// copies the \a count pixels of row \a y from \a x on: keyed where they lie in
// one of the \a keyCount rectangles, opaque elsewhere
void write_keyed_row(uint32* destination, const uint32* source, int32 x,
	int32 y, int32 count, const overlay_key_rect* keys, int32 keyCount);


#endif	// OVERLAY_KEYING_H
