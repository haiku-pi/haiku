/*
 * Copyright 2026, Haiku, Inc.
 * Distributed under the terms of the MIT License.
 */


/*!	Reading back what an overlay shows (see OverlayReadBack.h): its buffer is
	unpacked into planes of 16-bit samples (R, G, B, or Y, Cb, Cr), scaled
	from its view to its window bilinearly, as overlay scalers filter, and
	converted to RGB with the buffer's YCbCr encoding. It runs once per
	screenshot, so it is written for clarity, not speed; but overlay buffers
	are often write-combining memory, which is slow to read in small pieces,
	so they are read in whole rows (or columns) first.
*/


#include "OverlayReadBack.h"

#include <new>
#include <stdlib.h>
#include <string.h>


namespace {


struct plane {
	uint16*	samples;
	int32	width;
	int32	height;
	int32	shift_x;
		// 1: half the luma's horizontal resolution, sited with the even
		// luma samples (as in MPEG-2, H.264 and HEVC)
	int32	shift_y;
		// 1: half the luma's vertical resolution, sited between two rows
};


struct picture {
	plane	planes[3];
	bool	rgb;
		// R, G, B; else Y, Cb, Cr
	int32	bits;
		// per sample

	picture()
	{
		memset(planes, 0, sizeof(planes));
		rgb = false;
		bits = 8;
	}

	~picture()
	{
		for (int32 i = 0; i < 3; i++)
			free(planes[i].samples);
	}

	bool Allocate(int32 width, int32 height, int32 chromaShiftX,
		int32 chromaShiftY)
	{
		for (int32 i = 0; i < 3; i++) {
			plane& p = planes[i];
			p.shift_x = i == 0 ? 0 : chromaShiftX;
			p.shift_y = i == 0 ? 0 : chromaShiftY;
			p.width = (width + (1 << p.shift_x) - 1) >> p.shift_x;
			p.height = (height + (1 << p.shift_y) - 1) >> p.shift_y;
			p.samples = (uint16*)malloc(
				(size_t)p.width * p.height * sizeof(uint16));
			if (p.samples == NULL)
				return false;
		}
		return true;
	}

	uint16* Row(int32 index, int32 y) const
	{
		return planes[index].samples + (size_t)y * planes[index].width;
	}
};


static inline uint16
expand5(uint32 value)
{
	value &= 0x1f;
	return (uint16)((value << 3) | (value >> 2));
}


static inline uint16
expand6(uint32 value)
{
	value &= 0x3f;
	return (uint16)((value << 2) | (value >> 4));
}


static status_t
unpack_rgb(const overlay_buffer& buffer, picture& out, uint8* row)
{
	const int32 width = buffer.width;
	const int32 height = buffer.height;
	if (!out.Allocate(width, height, 0, 0))
		return B_NO_MEMORY;
	out.rgb = true;

	const uint8* bits = (const uint8*)buffer.buffer;
	const int32 bytesPerPixel = buffer.space == B_RGB32
		|| buffer.space == B_RGBA32 ? 4 : 2;
	for (int32 y = 0; y < height; y++) {
		memcpy(row, bits + (size_t)y * buffer.bytes_per_row,
			width * bytesPerPixel);
		uint16* red = out.Row(0, y);
		uint16* green = out.Row(1, y);
		uint16* blue = out.Row(2, y);
		for (int32 x = 0; x < width; x++) {
			if (bytesPerPixel == 4) {
				// B, G, R, A in memory; the alpha is not shown
				blue[x] = row[x * 4];
				green[x] = row[x * 4 + 1];
				red[x] = row[x * 4 + 2];
				continue;
			}
			const uint32 value = row[x * 2] | (row[x * 2 + 1] << 8);
			if (buffer.space == B_RGB16) {
				red[x] = expand5(value >> 11);
				green[x] = expand6(value >> 5);
			} else {
				red[x] = expand5(value >> 10);
				green[x] = expand5(value >> 5);
			}
			blue[x] = expand5(value);
		}
	}
	return B_OK;
}


/*!	B_YCbCr422: Y0 Cb0 Y1 Cr0, ... */
static status_t
unpack_ycbcr422(const overlay_buffer& buffer, picture& out, uint8* row)
{
	const int32 width = buffer.width & ~1;
	const int32 height = buffer.height;
	if (width < 2 || !out.Allocate(width, height, 1, 0))
		return B_BAD_VALUE;

	const uint8* bits = (const uint8*)buffer.buffer;
	for (int32 y = 0; y < height; y++) {
		memcpy(row, bits + (size_t)y * buffer.bytes_per_row, width * 2);
		uint16* luma = out.Row(0, y);
		uint16* cb = out.Row(1, y);
		uint16* cr = out.Row(2, y);
		for (int32 x = 0; x < width; x += 2) {
			luma[x] = row[x * 2];
			cb[x / 2] = row[x * 2 + 1];
			luma[x + 1] = row[x * 2 + 2];
			cr[x / 2] = row[x * 2 + 3];
		}
	}
	return B_OK;
}


/*!	B_YUV420, planar: Y, then Cb and Cr of half the width and height. Its
	bytes_per_row is that of the whole picture (3 bytes per 2 pixels): the
	luma rows are 2/3 of it, the chroma rows 1/3.
*/
static status_t
unpack_yuv420(const overlay_buffer& buffer, picture& out, uint8* row)
{
	const int32 width = buffer.width & ~1;
	const int32 height = buffer.height & ~1;
	if (width < 2 || height < 2 || !out.Allocate(width, height, 1, 1))
		return B_BAD_VALUE;

	const size_t lumaPitch = buffer.bytes_per_row / 3 * 2;
	const size_t chromaPitch = lumaPitch / 2;
	const uint8* bits = (const uint8*)buffer.buffer;
	const uint8* planes[3] = {
		bits,
		bits + lumaPitch * height,
		bits + lumaPitch * height + chromaPitch * (height / 2)
	};
	for (int32 i = 0; i < 3; i++) {
		const plane& p = out.planes[i];
		const size_t pitch = i == 0 ? lumaPitch : chromaPitch;
		for (int32 y = 0; y < p.height; y++) {
			memcpy(row, planes[i] + pitch * y, p.width);
			uint16* samples = out.Row(i, y);
			for (int32 x = 0; x < p.width; x++)
				samples[x] = row[x];
		}
	}
	return B_OK;
}


/*!	B_YUV420_SAND128(_10): luma, then the interleaved CbCr rows, in columns
	128 bytes wide; bytes_per_row is the column height in rows, the CbCr rows
	follow ((height + 15) & ~15) rows of luma in each column. 8 bit: 128
	samples a column row; 10 bit: 96, three in each little-endian 32-bit word,
	lowest first.
*/
static status_t
unpack_sand(const overlay_buffer& buffer, picture& out)
{
	const bool tenBit = buffer.space == B_YUV420_SAND128_10;
	const int32 width = buffer.width & ~1;
	const int32 height = buffer.height & ~1;
	const size_t chromaOffset = (size_t)((buffer.height + 15) & ~15) * 128;
	if (width < 2 || height < 2
		|| buffer.bytes_per_row < (uint32)((buffer.height + 15) & ~15) * 3 / 2
		|| !out.Allocate(width, height, 1, 1)) {
		return B_BAD_VALUE;
	}
	out.bits = tenBit ? 10 : 8;

	const uint8* bits = (const uint8*)buffer.buffer;
	const size_t columnStride = (size_t)buffer.bytes_per_row * 128;
	const int32 samplesPerColumn = tenBit ? 96 : 128;
	uint8 block[128];
	uint16 samples[128];

	// the luma rows, and the chroma rows with Cb and Cr alternating
	for (int32 part = 0; part < 2; part++) {
		const int32 rows = part == 0 ? height : height / 2;
		const size_t offset = part == 0 ? 0 : chromaOffset;
		for (int32 y = 0; y < rows; y++) {
			for (int32 x = 0; x < width; x += samplesPerColumn) {
				memcpy(block, bits + offset + x / samplesPerColumn
					* columnStride + (size_t)y * 128, 128);
				if (tenBit) {
					for (int32 k = 0; k < 96; k++) {
						const uint32 word = block[k / 3 * 4]
							| (block[k / 3 * 4 + 1] << 8)
							| (block[k / 3 * 4 + 2] << 16)
							| ((uint32)block[k / 3 * 4 + 3] << 24);
						samples[k] = (word >> (k % 3 * 10)) & 0x3ff;
					}
				} else {
					for (int32 k = 0; k < 128; k++)
						samples[k] = block[k];
				}
				const int32 count = width - x < samplesPerColumn
					? width - x : samplesPerColumn;
				if (part == 0) {
					memcpy(out.Row(0, y) + x, samples,
						count * sizeof(uint16));
					continue;
				}
				uint16* cb = out.Row(1, y);
				uint16* cr = out.Row(2, y);
				for (int32 k = 0; k < count; k += 2) {
					cb[(x + k) / 2] = samples[k];
					cr[(x + k) / 2] = samples[k + 1];
				}
			}
		}
	}
	return B_OK;
}


static status_t
unpack(const overlay_buffer& buffer, picture& out)
{
	if (buffer.buffer == NULL || buffer.width == 0 || buffer.height == 0)
		return B_BAD_VALUE;

	uint8* row = (uint8*)malloc((size_t)buffer.width * 4);
	if (row == NULL)
		return B_NO_MEMORY;

	status_t status;
	switch (buffer.space) {
		case B_RGB32:
		case B_RGBA32:
		case B_RGB16:
		case B_RGB15:
		case B_RGBA15:
			status = unpack_rgb(buffer, out, row);
			break;
		case B_YCbCr422:
			status = unpack_ycbcr422(buffer, out, row);
			break;
		case B_YUV420:
			status = unpack_yuv420(buffer, out, row);
			break;
		case B_YUV420_SAND128:
		case B_YUV420_SAND128_10:
			status = unpack_sand(buffer, out);
			break;
		default:
			status = B_NOT_SUPPORTED;
			break;
	}
	free(row);
	return status;
}


/*!	From YCbCr samples of \a bits to RGB in 0...1: the encoding of
	overlay_window.flags. Without a matrix, the one overlays use for video
	without one (as the accelerants that show YCbCr do): BT.709 from 720
	lines on, BT.601 below.
*/
struct ycbcr_to_rgb {
	float	luma_offset;
	float	luma_scale;
	float	chroma_offset;
	float	chroma_scale;
	float	red_cr;
	float	green_cb;
	float	green_cr;
	float	blue_cb;

	ycbcr_to_rgb(uint32 flags, int32 height, int32 bits)
	{
		uint32 matrix = flags & B_OVERLAY_COLOR_MATRIX_MASK;
		if (matrix == 0) {
			matrix = height >= 720
				? B_OVERLAY_COLOR_BT709 : B_OVERLAY_COLOR_BT601;
		}
		float kr = 0.299f;
		float kb = 0.114f;
		if (matrix == B_OVERLAY_COLOR_BT709) {
			kr = 0.2126f;
			kb = 0.0722f;
		} else if (matrix == B_OVERLAY_COLOR_BT2020) {
			kr = 0.2627f;
			kb = 0.0593f;
		}
		const float kg = 1 - kr - kb;
		red_cr = 2 * (1 - kr);
		blue_cb = 2 * (1 - kb);
		green_cb = -2 * kb * (1 - kb) / kg;
		green_cr = -2 * kr * (1 - kr) / kg;

		const float unit = (float)(1 << (bits - 8));
		if ((flags & B_OVERLAY_COLOR_FULL_RANGE) != 0) {
			const float maximum = (float)((1 << bits) - 1);
			luma_offset = 0;
			luma_scale = 1 / maximum;
			chroma_offset = 128 * unit;
			chroma_scale = 1 / maximum;
		} else {
			luma_offset = 16 * unit;
			luma_scale = 1 / (219 * unit);
			chroma_offset = 128 * unit;
			chroma_scale = 1 / (224 * unit);
		}
	}
};


static inline uint32
to_8_bits(float value)
{
	// \a value in 0...1, rounded
	const float scaled = value * 255 + 0.5f;
	if (scaled <= 0)
		return 0;
	if (scaled >= 255)
		return 255;
	return (uint32)scaled;
}


/*!	Bilinear sample of \a p at (\a x, \a y), in its own coordinates; the
	neighbours are clamped to the rectangle \a minX ... \a maxY (the view's
	part of the plane), as the scaler repeats the edge of its source.
*/
static float
sample(const plane& p, float x, float y, int32 minX, int32 maxX, int32 minY,
	int32 maxY)
{
	if (x < minX)
		x = minX;
	else if (x > maxX)
		x = maxX;
	if (y < minY)
		y = minY;
	else if (y > maxY)
		y = maxY;

	const int32 x0 = (int32)x;
	const int32 y0 = (int32)y;
	const int32 x1 = x0 < maxX ? x0 + 1 : x0;
	const int32 y1 = y0 < maxY ? y0 + 1 : y0;
	const float fx = x - x0;
	const float fy = y - y0;

	const uint16* row0 = p.samples + (size_t)y0 * p.width;
	const uint16* row1 = p.samples + (size_t)y1 * p.width;
	const float top = row0[x0] + (row0[x1] - row0[x0]) * fx;
	const float bottom = row1[x0] + (row1[x1] - row1[x0]) * fx;
	return top + (bottom - top) * fy;
}


static inline int32
min_int(int32 a, int32 b)
{
	return a < b ? a : b;
}


static inline int32
max_int(int32 a, int32 b)
{
	return a > b ? a : b;
}


}	// namespace


status_t
read_back_overlay(const overlay_read_back& overlay, uint32* pixels,
	int32 bytesPerRow, int32 left, int32 top, int32 width, int32 height)
{
	const overlay_buffer* buffer = overlay.buffer;
	const overlay_view& view = overlay.view;
	const overlay_window& window = overlay.window;
	if (buffer == NULL || view.width == 0 || view.height == 0
		|| window.width == 0 || window.height == 0
		|| view.h_start + view.width > buffer->width
		|| view.v_start + view.height > buffer->height) {
		return B_BAD_VALUE;
	}

	// the part of the window that is shown, and of that what is asked for
	const int32 windowLeft = window.h_start;
	const int32 windowTop = window.v_start;
	const int32 fromX = max_int(windowLeft + window.offset_left, left);
	const int32 toX = min_int(windowLeft + window.width - window.offset_right,
		left + width);
	const int32 fromY = max_int(windowTop + window.offset_top, top);
	const int32 toY = min_int(windowTop + window.height - window.offset_bottom,
		top + height);
	if (fromX >= toX || fromY >= toY)
		return B_OK;

	picture source;
	status_t status = unpack(*buffer, source);
	if (status != B_OK)
		return status;

	const ycbcr_to_rgb encoding(window.flags, buffer->height, source.bits);
	const bool mirrored = (window.flags & B_OVERLAY_HORIZONTAL_MIRRORING) != 0;
	const bool colorKey = (window.flags & B_OVERLAY_COLOR_KEY) != 0;
	const uint32 keyMask = (uint32)window.red.mask << 16
		| (uint32)window.green.mask << 8 | window.blue.mask;
	const uint32 key = ((uint32)window.red.value << 16
		| (uint32)window.green.value << 8 | window.blue.value) & keyMask;

	// the view's samples in each plane (chroma: the pairs it touches)
	int32 minX[3], maxX[3], minY[3], maxY[3];
	for (int32 i = 0; i < 3; i++) {
		const plane& p = source.planes[i];
		minX[i] = view.h_start >> p.shift_x;
		maxX[i] = min_int(((view.h_start + view.width - 1) >> p.shift_x),
			p.width - 1);
		minY[i] = view.v_start >> p.shift_y;
		maxY[i] = min_int(((view.v_start + view.height - 1) >> p.shift_y),
			p.height - 1);
	}

	const float scaleX = (float)view.width / window.width;
	const float scaleY = (float)view.height / window.height;

	for (int32 y = fromY; y < toY; y++) {
		uint32* row = (uint32*)((uint8*)pixels
			+ (size_t)(y - top) * bytesPerRow);
		// the luma position of this row's centre
		const float sourceY = view.v_start + (y - windowTop + 0.5f) * scaleY
			- 0.5f;

		for (int32 x = fromX; x < toX; x++) {
			const uint32 pixel = row[x - left];
			if (overlay.keyed_by_alpha) {
				if ((pixel >> 24) != 0)
					continue;
			} else if (colorKey && (pixel & keyMask) != key)
				continue;

			int32 column = x - windowLeft;
			if (mirrored)
				column = window.width - 1 - column;
			const float sourceX = view.h_start + (column + 0.5f) * scaleX
				- 0.5f;

			float values[3];
			for (int32 i = 0; i < 3; i++) {
				const plane& p = source.planes[i];
				const float planeX = p.shift_x != 0 ? sourceX / 2 : sourceX;
				const float planeY = p.shift_y != 0
					? (sourceY - 0.5f) / 2 : sourceY;
				values[i] = sample(p, planeX, planeY, minX[i], maxX[i],
					minY[i], maxY[i]);
			}

			float red, green, blue;
			if (source.rgb) {
				red = values[0] / 255;
				green = values[1] / 255;
				blue = values[2] / 255;
			} else {
				const float luma = (values[0] - encoding.luma_offset)
					* encoding.luma_scale;
				const float cb = (values[1] - encoding.chroma_offset)
					* encoding.chroma_scale;
				const float cr = (values[2] - encoding.chroma_offset)
					* encoding.chroma_scale;
				red = luma + encoding.red_cr * cr;
				green = luma + encoding.green_cb * cb + encoding.green_cr * cr;
				blue = luma + encoding.blue_cb * cb;
			}
			row[x - left] = 0xff000000 | to_8_bits(red) << 16
				| to_8_bits(green) << 8 | to_8_bits(blue);
		}
	}
	return B_OK;
}
