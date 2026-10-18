/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The hardware cursor: the driver shows it on a firmware plane. */


#include "accelerant_protos.h"
#include "accelerant.h"

#include <new>
#include <string.h>
#include <unistd.h>


static status_t
set_cursor(bcm2711_display_cursor& cursor)
{
	return ioctl(gInfo->device, BCM2711_DISPLAY_SET_CURSOR, &cursor,
		sizeof(cursor)) == 0 ? B_OK : B_ERROR;
}


/*!	A BeOS cursor: per pixel, AND 1 + XOR 0 is transparent, AND 0 + XOR 0
	white, AND 0 + XOR 1 black; AND 1 + XOR 1 (invert) is shown black.
*/
status_t
bcm2711_set_cursor_shape(uint16 width, uint16 height, uint16 hotX, uint16 hotY,
	const uint8* andMask, const uint8* xorMask)
{
	if (width > BCM2711_DISPLAY_CURSOR_SIZE
		|| height > BCM2711_DISPLAY_CURSOR_SIZE || (width % 8) != 0) {
		return B_BAD_VALUE;
	}

	bcm2711_display_cursor* cursor
		= new(std::nothrow) bcm2711_display_cursor;
	if (cursor == NULL)
		return B_NO_MEMORY;

	cursor->width = width;
	cursor->height = height;
	cursor->hot_x = hotX;
	cursor->hot_y = hotY;
	const uint32 bytesPerRow = width / 8;
	for (uint32 y = 0; y < height; y++) {
		for (uint32 x = 0; x < width; x++) {
			const uint32 byte = y * bytesPerRow + x / 8;
			const uint8 bit = 0x80 >> (x % 8);
			const bool transparent = (andMask[byte] & bit) != 0
				&& (xorMask[byte] & bit) == 0;
			const bool black = (xorMask[byte] & bit) != 0;
			cursor->bits[y * width + x] = transparent ? 0
				: black ? 0xff000000 : 0xffffffff;
		}
	}

	status_t status = set_cursor(*cursor);
	delete cursor;
	return status;
}


status_t
bcm2711_set_cursor_bitmap(uint16 width, uint16 height, uint16 hotX,
	uint16 hotY, color_space colorSpace, uint16 bytesPerRow,
	const uint8* bitmapData)
{
	if (colorSpace != B_RGBA32 || width > BCM2711_DISPLAY_CURSOR_SIZE
		|| height > BCM2711_DISPLAY_CURSOR_SIZE || bytesPerRow < width * 4) {
		return B_BAD_VALUE;
	}

	bcm2711_display_cursor* cursor
		= new(std::nothrow) bcm2711_display_cursor;
	if (cursor == NULL)
		return B_NO_MEMORY;

	cursor->width = width;
	cursor->height = height;
	cursor->hot_x = hotX;
	cursor->hot_y = hotY;
	for (uint32 y = 0; y < height; y++)
		memcpy(cursor->bits + y * width, bitmapData + y * bytesPerRow, width * 4);

	status_t status = set_cursor(*cursor);
	delete cursor;
	return status;
}


void
bcm2711_move_cursor(uint16 x, uint16 y)
{
	bcm2711_display_cursor_position position = { x, y };
	ioctl(gInfo->device, BCM2711_DISPLAY_MOVE_CURSOR, &position,
		sizeof(position));
}


void
bcm2711_show_cursor(bool isVisible)
{
	ioctl(gInfo->device, BCM2711_DISPLAY_SHOW_CURSOR, &isVisible,
		sizeof(isVisible));
}
