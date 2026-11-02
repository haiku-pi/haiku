/*
 * Copyright 2026, Haiku, Inc.
 * Distributed under the terms of the MIT License.
 */


#include "OverlayKeying.h"


static const uint32 kAlpha = 0xff000000;
static const uint32 kColor = 0x00ffffff;
static const int32 kMaxSpans = 8;


void
copy_row_opaque(uint32* destination, const uint32* source, int32 count)
{
	int32 i = 0;
	for (; i + 4 <= count; i += 4) {
		const uint32 a = source[i];
		const uint32 b = source[i + 1];
		const uint32 c = source[i + 2];
		const uint32 d = source[i + 3];
		destination[i] = a | kAlpha;
		destination[i + 1] = b | kAlpha;
		destination[i + 2] = c | kAlpha;
		destination[i + 3] = d | kAlpha;
	}
	for (; i < count; i++)
		destination[i] = source[i] | kAlpha;
}


void
copy_row_keyed(uint32* destination, const uint32* source, int32 count,
	uint32 key)
{
	key &= kColor;
	for (int32 i = 0; i < count; i++) {
		const uint32 pixel = source[i];
		destination[i] = (pixel & kColor) == key ? 0 : pixel | kAlpha;
	}
}


void
write_keyed_row(uint32* destination, const uint32* source, int32 x, int32 y,
	int32 count, const overlay_key_rect* keys, int32 keyCount)
{
	const int32 right = x + count - 1;

	// the parts of the row in a rectangle, ordered by their left edge
	struct span {
		int32	left;
		int32	right;
		uint32	key;
	} spans[kMaxSpans];
	int32 spanCount = 0;
	for (int32 i = 0; i < keyCount && spanCount < kMaxSpans; i++) {
		const overlay_key_rect& rect = keys[i];
		if (y < rect.top || y > rect.bottom || rect.right < x
			|| rect.left > right) {
			continue;
		}
		span newSpan = { rect.left > x ? rect.left : x,
			rect.right < right ? rect.right : right, rect.key };
		int32 index = spanCount++;
		while (index > 0 && spans[index - 1].left > newSpan.left) {
			spans[index] = spans[index - 1];
			index--;
		}
		spans[index] = newSpan;
	}

	int32 position = x;
	for (int32 i = 0; i < spanCount; i++) {
		int32 left = spans[i].left;
		if (left < position)
			left = position;
			// overlapping rectangles: the first one keys the overlap
		if (spans[i].right < left)
			continue;
		if (left > position) {
			copy_row_opaque(destination + (position - x),
				source + (position - x), left - position);
		}
		copy_row_keyed(destination + (left - x), source + (left - x),
			spans[i].right - left + 1, spans[i].key);
		position = spans[i].right + 1;
	}
	if (position <= right) {
		copy_row_opaque(destination + (position - x), source + (position - x),
			right - position + 1);
	}
}
