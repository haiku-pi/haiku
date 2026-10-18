/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Exercises the V3D driver through the Linux v3d uAPI on
	/dev/dri/renderD128: identification, buffer objects, a clear-and-store
	job on the GPU, waits and syncobjs.

	The job is the smallest real one: a binner list without draws and a
	render list that clears the 64x64 RGBA8 tile buffer to a colour and
	stores it to a linear buffer, which the CPU then checks pixel by pixel.
	The packets are encoded from Mesa's src/broadcom/cle/v3d_packet.xml
	(V3D 4.2), in the order Mesa's v3dx_rcl.c emits a clear-only frame.

	Usage: v3dtest [-d] [test...]
		tests: info bo clear wait syncobj (the default, "all"), and
		hang    a binner list that never finishes: the watchdog must reset
		        the GPU, after which a clear must work again
		orphan  submits a hanging job and exits without cleaning up; a
		        later "areas" shows whether the driver freed everything
		areas   prints the number of kernel areas of BOs
		-d      also writes every line to the syslog (/dev/dprintf)
	Exit status: 0 if every check passed.
*/


#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <OS.h>

#include <drm/v3d_drm.h>


static const char* kDevicePath = "/dev/dri/renderD128";

static const uint32 kWidth = 64;
static const uint32 kHeight = 64;
static const uint32 kPixels = kWidth * kHeight;
static const uint32 kStride = kWidth * 4;
static const uint32 kSentinel = 0xdeadbeef;

// 1 tile of 64x64: tile allocation = align(128 B per tile, 4 KiB) + 8 KiB
// (Mesa v3d_tile_alloc_sizes()), tile state = 256 B per tile.
static const uint32 kTileAllocSize = 12288;
static const uint32 kTileStateSize = 4096;
static const uint32 kListSize = 4096;
	// a control list buffer; the CLE reads up to 256 bytes ahead
static const uint32 kGenericListOffset = 256;
	// where the per-tile list goes in the render list buffer


static int sDevice = -1;
static FILE* sSyslog = NULL;
static int sFailures = 0;
static int sChecks = 0;


static void
report(const char* format, ...)
{
	char line[512];
	va_list args;
	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);

	printf("v3dtest: %s\n", line);
	fflush(stdout);
	if (sSyslog != NULL) {
		fprintf(sSyslog, "v3dtest: %s\n", line);
		fflush(sSyslog);
	}
}


static bool
check(bool ok, const char* format, ...)
{
	char line[480];
	va_list args;
	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);

	sChecks++;
	if (!ok)
		sFailures++;
	report("%s %s", line, ok ? "OK" : "FAILED");
	return ok;
}


/*!	As libdrm's drmIoctl(): restarts interrupted calls. Returns 0 or the
	error code.
*/
static int
drm_ioctl(unsigned long request, void* argument)
{
	int result;
	do {
		result = ioctl(sDevice, request, argument);
	} while (result == -1 && (errno == EINTR || errno == EAGAIN));
	return result == -1 ? errno : 0;
}


static uint64
get_param(uint32 param, int* _error = NULL)
{
	drm_v3d_get_param args = {};
	args.param = param;
	int error = drm_ioctl(DRM_IOCTL_V3D_GET_PARAM, &args);
	if (_error != NULL)
		*_error = error;
	return error == 0 ? args.value : 0;
}


static int64
monotonic_ns(bigtime_t fromNow)
{
	// CLOCK_MONOTONIC on Haiku is system_time()
	return (system_time() + fromNow) * 1000;
}


static int32
count_bo_areas()
{
	area_info info;
	ssize_t cookie = 0;
	int32 count = 0;
	while (get_next_area_info(B_SYSTEM_TEAM, &cookie, &info) == B_OK) {
		if (strcmp(info.name, "v3d bo") == 0)
			count++;
	}
	return count;
}


//	#pragma mark - buffer objects


struct Buffer {
	uint32	handle;
	uint32	gpu_address;
	uint32	size;
	uint8*	data;
};


static int
buffer_close(uint32 handle)
{
	drm_gem_close args = {};
	args.handle = handle;
	return drm_ioctl(DRM_IOCTL_GEM_CLOSE, &args);
}


static int
buffer_create(Buffer& buffer, uint32 size)
{
	memset(&buffer, 0, sizeof(buffer));

	drm_v3d_create_bo create = {};
	create.size = size;
	int error = drm_ioctl(DRM_IOCTL_V3D_CREATE_BO, &create);
	if (error != 0)
		return error;
	buffer.handle = create.handle;
	buffer.gpu_address = create.offset;
	buffer.size = size;

	drm_v3d_mmap_bo map = {};
	map.handle = create.handle;
	error = drm_ioctl(DRM_IOCTL_V3D_MMAP_BO, &map);
	if (error != 0) {
		buffer_close(buffer.handle);
		return error;
	}
	// On Haiku, "offset" is the mapping itself: a device cannot be mmap()ed.
	buffer.data = (uint8*)(addr_t)map.offset;
	return 0;
}


static void
buffer_destroy(Buffer& buffer)
{
	if (buffer.data != NULL && munmap(buffer.data, buffer.size) != 0) {
		report("munmap() of a BO mapping failed: %s, deleting the area",
			strerror(errno));
		delete_area(area_for(buffer.data));
	}
	if (buffer.handle != 0)
		buffer_close(buffer.handle);
	memset(&buffer, 0, sizeof(buffer));
}


//	#pragma mark - control lists


struct ControlList {
	uint8	data[kGenericListOffset];
	uint32	length;
};


/*!	Appends a packet: its opcode, then \a length - 1 bytes of fields, all
	zero. Returns the fields.
*/
static uint8*
emit(ControlList& list, uint8 opcode, uint32 length)
{
	if (list.length + length > sizeof(list.data)) {
		report("control list overflow");
		exit(1);
	}
	uint8* packet = list.data + list.length;
	memset(packet, 0, length);
	packet[0] = opcode;
	list.length += length;
	return packet + 1;
}


/*!	Sets a field of a packet; \a start and \a width in bits, counted from the
	first field byte, little endian, as the XML describes them. Addresses are
	set as whole 32-bit words: their low bits are zero by alignment, which is
	where some packets keep other fields.
*/
static void
set_field(uint8* fields, uint32 start, uint32 width, uint64 value)
{
	for (uint32 bit = 0; bit < width; bit++) {
		if (((value >> bit) & 1) != 0)
			fields[(start + bit) / 8] |= 1 << ((start + bit) % 8);
	}
}


// Opcodes (v3d_packet.xml)
enum {
	kFlush						= 4,
	kStartTileBinning			= 6,
	kWaitOnSemaphore			= 8,
	kEndOfRendering				= 13,
	kReturnFromSubList			= 18,
	kFlushVcdCache				= 19,
	kStartAddressOfGenericTileList = 20,
	kBranchToImplicitTileList	= 21,
	kSupertileCoordinates		= 23,
	kClearTileBuffers			= 25,
	kEndOfLoads					= 26,
	kEndOfTileMarker			= 27,
	kStoreTileBufferGeneral		= 29,
	kSetInstanceId				= 54,
	kPrimListFormat				= 56,
	kOcclusionQueryCounter		= 92,
	kNumberOfLayers				= 119,
	kTileBinningModeConfig		= 120,
	kTileRenderingModeConfig	= 121,
	kMulticoreSupertileConfig	= 122,
	kMulticoreTileListBase		= 123,
	kTileCoordinates			= 124,
	kTileCoordinatesImplicit	= 125,
	kTileListInitialBlockSize	= 126,
};

static const uint32 kStoreNone = 8;
static const uint32 kOutputFormatRGBA8 = 27;
static const uint32 kInternalType8 = 2;
static const uint32 kBlockSize64 = 0;
static const uint32 kBlockSize128 = 1;


/*!	The binner list of a frame without draws (Mesa v3dX(start_binning)() and
	v3dX(bcl_epilogue)()): binning set up, nothing binned, and FLUSH caps
	every tile list. With \a hang, it first waits for a semaphore only a
	render list could increment: the binner stops with its current address
	unchanged, which only the watchdog ends.
*/
static void
build_binner_list(ControlList& list, bool hang)
{
	if (hang)
		emit(list, kWaitOnSemaphore, 1);

	uint8* fields = emit(list, kNumberOfLayers, 2);
	set_field(fields, 0, 8, 1 - 1);

	fields = emit(list, kTileBinningModeConfig, 9);
	set_field(fields, 48, 16, kHeight - 1);
	set_field(fields, 32, 16, kWidth - 1);
	// maximum bpp 32 (0), one render target (0), no MSAA, no double buffer
	set_field(fields, 4, 2, kBlockSize64);
	set_field(fields, 2, 2, kBlockSize128);
		// the initial block size; the render list must say the same

	emit(list, kFlushVcdCache, 1);
	emit(list, kOcclusionQueryCounter, 5);
		// address 0: no occlusion query
	emit(list, kStartTileBinning, 1);
	emit(list, kFlush, 1);
}


/*!	STORE_TILE_BUFFER_GENERAL of render target 0 to a linear RGBA8 image
	(Mesa store_general()), or a store of nothing.
*/
static void
emit_store(ControlList& list, bool renderTarget, uint32 address)
{
	uint8* fields = emit(list, kStoreTileBufferGeneral, 13);
	if (!renderTarget) {
		set_field(fields, 0, 4, kStoreNone);
		return;
	}
	set_field(fields, 0, 4, 0);
		// render target 0, raster memory format (0), no dither, sample 0
	set_field(fields, 12, 6, kOutputFormatRGBA8);
	set_field(fields, 28, 20, kStride);
	set_field(fields, 64, 32, address);
}


static void
emit_clear_tile_buffers(ControlList& list)
{
	uint8* fields = emit(list, kClearTileBuffers, 2);
	set_field(fields, 1, 1, 1);
		// Z/stencil
	set_field(fields, 0, 1, 1);
		// all render targets
}


/*!	The per-tile list the render list branches to for every tile
	(Mesa v3d_rcl_emit_generic_per_tile_list()): no loads, the binner's tile
	list, store render target 0, clear the tile buffer for the next tile
	(GFXH-1461).
*/
static void
build_generic_tile_list(ControlList& list, uint32 target)
{
	emit(list, kTileCoordinatesImplicit, 1);
	emit(list, kEndOfLoads, 1);

	uint8* fields = emit(list, kPrimListFormat, 2);
	set_field(fields, 0, 6, 2);
		// triangles, which the binner assumes
	emit(list, kSetInstanceId, 5);
		// instance 0: the binner assumes it, the hardware does not set it

	emit(list, kBranchToImplicitTileList, 2);
	emit_store(list, true, target);
	emit_clear_tile_buffers(list);
	emit(list, kEndOfTileMarker, 1);
	emit(list, kReturnFromSubList, 1);
}


/*!	The render list of a clear-only frame (Mesa v3dX(emit_rcl)() for V3D 4.2,
	one 64x64 tile, one supertile).
*/
static void
build_render_list(ControlList& list, uint32 color, uint32 tileAlloc,
	uint32 genericStart, uint32 genericEnd)
{
	// Tile rendering mode: "Common" must come first, "ZS clear values" last.
	uint8* fields = emit(list, kTileRenderingModeConfig, 9);
	set_field(fields, 0, 4, 0);
		// sub-id: common
	set_field(fields, 4, 4, 1 - 1);
		// one render target
	set_field(fields, 8, 16, kWidth);
	set_field(fields, 24, 16, kHeight);
	set_field(fields, 46, 1, 1);
		// early Z off: there is no depth buffer

	fields = emit(list, kTileRenderingModeConfig, 9);
	set_field(fields, 0, 4, 3);
		// sub-id: clear colours part 1, render target 0
	set_field(fields, 8, 32, color);
		// an 8-bit internal type takes the packed RGBA8 colour

	fields = emit(list, kTileRenderingModeConfig, 9);
	set_field(fields, 0, 4, 1);
		// sub-id: colour
	set_field(fields, 6, 4, kInternalType8);
		// render target 0: internal bpp 32 (0), type 8, no clamp

	fields = emit(list, kTileRenderingModeConfig, 9);
	set_field(fields, 0, 4, 2);
		// sub-id: Z/stencil clear values, both 0

	fields = emit(list, kTileListInitialBlockSize, 2);
	set_field(fields, 0, 2, kBlockSize128);
	set_field(fields, 2, 1, 1);
		// auto-chained tile lists

	fields = emit(list, kMulticoreTileListBase, 5);
	set_field(fields, 0, 32, tileAlloc);
		// tile list set 0 in the low bits, the address is 64 byte aligned

	fields = emit(list, kMulticoreSupertileConfig, 9);
	set_field(fields, 0, 8, 1 - 1);
	set_field(fields, 8, 8, 1 - 1);
		// supertiles of 1x1 tiles
	set_field(fields, 16, 8, 1);
	set_field(fields, 24, 8, 1);
		// a frame of 1x1 supertiles
	set_field(fields, 32, 12, 1);
	set_field(fields, 44, 12, 1);
		// a frame of 1x1 tiles; one bin tile list (0)

	// Clear the tile buffer once, then a second dummy store (GFXH-1742).
	for (int32 i = 0; i < 2; i++) {
		emit(list, kTileCoordinates, 4);
			// tile 0, 0
		emit(list, kEndOfLoads, 1);
		emit_store(list, false, 0);
		if (i == 0)
			emit_clear_tile_buffers(list);
		emit(list, kEndOfTileMarker, 1);
	}

	emit(list, kFlushVcdCache, 1);

	fields = emit(list, kStartAddressOfGenericTileList, 9);
	set_field(fields, 0, 32, genericStart);
	set_field(fields, 32, 32, genericEnd);

	emit(list, kSupertileCoordinates, 3);
		// supertile 0, 0: renders it
	emit(list, kEndOfRendering, 1);
}


//	#pragma mark - the clear job


struct ClearJob {
	Buffer	binner;
	Buffer	render;
	Buffer	tile_alloc;
	Buffer	tile_state;
	Buffer	target;
	uint32	color;
};


static void
clear_job_destroy(ClearJob& job)
{
	buffer_destroy(job.target);
	buffer_destroy(job.tile_state);
	buffer_destroy(job.tile_alloc);
	buffer_destroy(job.render);
	buffer_destroy(job.binner);
}


static int
clear_job_submit(ClearJob& job, uint32 color, uint32 outSync, bool hang)
{
	memset(&job, 0, sizeof(job));
	job.color = color;

	int error = buffer_create(job.binner, kListSize);
	if (error == 0)
		error = buffer_create(job.render, kListSize);
	if (error == 0)
		error = buffer_create(job.tile_alloc, kTileAllocSize);
	if (error == 0)
		error = buffer_create(job.tile_state, kTileStateSize);
	if (error == 0)
		error = buffer_create(job.target, kPixels * 4);
	if (error != 0) {
		clear_job_destroy(job);
		return error;
	}

	uint32* pixels = (uint32*)job.target.data;
	for (uint32 i = 0; i < kPixels; i++)
		pixels[i] = kSentinel;

	ControlList binner = {}, render = {}, generic = {};
	build_binner_list(binner, hang);
	const uint32 genericStart = job.render.gpu_address + kGenericListOffset;
	build_generic_tile_list(generic, job.target.gpu_address);
	build_render_list(render, color, job.tile_alloc.gpu_address, genericStart,
		genericStart + generic.length);

	memcpy(job.binner.data, binner.data, binner.length);
	memcpy(job.render.data, render.data, render.length);
	memcpy(job.render.data + kGenericListOffset, generic.data,
		generic.length);
	// The lists and the sentinel reach memory before the GPU reads them.
	__asm__ __volatile__("dsb sy" : : : "memory");

	uint32 handles[5] = { job.binner.handle, job.render.handle,
		job.tile_alloc.handle, job.tile_state.handle, job.target.handle };

	drm_v3d_submit_cl submit = {};
	submit.bcl_start = job.binner.gpu_address;
	submit.bcl_end = job.binner.gpu_address + binner.length;
	submit.rcl_start = job.render.gpu_address;
	submit.rcl_end = job.render.gpu_address + render.length;
	submit.qma = job.tile_alloc.gpu_address;
	submit.qms = job.tile_alloc.size;
	submit.qts = job.tile_state.gpu_address;
	submit.bo_handles = (uint64)(addr_t)handles;
	submit.bo_handle_count = 5;
	submit.out_sync = outSync;
	error = drm_ioctl(DRM_IOCTL_V3D_SUBMIT_CL, &submit);
	if (error != 0)
		clear_job_destroy(job);
	return error;
}


static int
wait_bo(uint32 handle, uint64 timeout)
{
	drm_v3d_wait_bo args = {};
	args.handle = handle;
	args.timeout_ns = timeout;
	return drm_ioctl(DRM_IOCTL_V3D_WAIT_BO, &args);
}


/*!	Counts the pixels that are not the clear colour, and reports the first
	few.
*/
static uint32
clear_job_check(ClearJob& job)
{
	const uint32* pixels = (const uint32*)job.target.data;
	uint32 bad = 0;
	for (uint32 i = 0; i < kPixels; i++) {
		if (pixels[i] == job.color)
			continue;
		if (bad++ < 4) {
			report("  pixel %" B_PRIu32 ",%" B_PRIu32 " is %#010" B_PRIx32
				", expected %#010" B_PRIx32, i % kWidth, i / kWidth, pixels[i],
				job.color);
		}
	}
	return bad;
}


//	#pragma mark - tests


static void
test_info()
{
	char name[32] = {}, date[32] = {}, description[64] = {};
	drm_version version = {};
	version.name = name;
	version.name_len = sizeof(name) - 1;
	version.date = date;
	version.date_len = sizeof(date) - 1;
	version.desc = description;
	version.desc_len = sizeof(description) - 1;
	int error = drm_ioctl(DRM_IOCTL_VERSION, &version);
	check(error == 0 && strcmp(name, "v3d") == 0,
		"version: %s %d.%d.%d %s (%s), %s", name, version.version_major,
		version.version_minor, version.version_patchlevel, date, description,
		strerror(error));

	drm_get_cap cap = {};
	cap.capability = DRM_CAP_SYNCOBJ;
	error = drm_ioctl(DRM_IOCTL_GET_CAP, &cap);
	check(error == 0 && cap.value == 1, "syncobj capability %" B_PRIu64 ", %s",
		cap.value, strerror(error));

	const uint64 ident0 = get_param(DRM_V3D_PARAM_V3D_CORE0_IDENT0, &error);
	check(error == 0 && (ident0 & 0xffffff) == 0x443356,
		"ident core %#" B_PRIx64 "/%#" B_PRIx64 "/%#" B_PRIx64 " uif %#"
		B_PRIx64 " hub %#" B_PRIx64 "/%#" B_PRIx64 "/%#" B_PRIx64
		", version %" B_PRIu64 ".%" B_PRIu64, ident0,
		get_param(DRM_V3D_PARAM_V3D_CORE0_IDENT1),
		get_param(DRM_V3D_PARAM_V3D_CORE0_IDENT2),
		get_param(DRM_V3D_PARAM_V3D_UIFCFG),
		get_param(DRM_V3D_PARAM_V3D_HUB_IDENT1),
		get_param(DRM_V3D_PARAM_V3D_HUB_IDENT2),
		get_param(DRM_V3D_PARAM_V3D_HUB_IDENT3), ident0 >> 28,
		(get_param(DRM_V3D_PARAM_V3D_CORE0_IDENT1) >> 28) & 0xf);

	report("supports tfu %" B_PRIu64 " csd %" B_PRIu64 " cache flush %"
		B_PRIu64 " perfmon %" B_PRIu64 " multisync %" B_PRIu64 " cpu queue %"
		B_PRIu64 ", resets so far %" B_PRIu64,
		get_param(DRM_V3D_PARAM_SUPPORTS_TFU),
		get_param(DRM_V3D_PARAM_SUPPORTS_CSD),
		get_param(DRM_V3D_PARAM_SUPPORTS_CACHE_FLUSH),
		get_param(DRM_V3D_PARAM_SUPPORTS_PERFMON),
		get_param(DRM_V3D_PARAM_SUPPORTS_MULTISYNC_EXT),
		get_param(DRM_V3D_PARAM_SUPPORTS_CPU_QUEUE),
		get_param(DRM_V3D_PARAM_GLOBAL_RESET_COUNTER));

	get_param(0x1000, &error);
	check(error == EINVAL, "unknown parameter refused (%s)", strerror(error));
}


static void
test_bo()
{
	Buffer first, second;
	int error = buffer_create(first, 64 * 1024);
	if (!check(error == 0, "create + map 64 KiB BO: %s", strerror(error)))
		return;
	error = buffer_create(second, 4096);
	if (!check(error == 0, "create + map 4 KiB BO: %s", strerror(error))) {
		buffer_destroy(first);
		return;
	}

	check(first.gpu_address != 0 && (first.gpu_address & 4095) == 0
			&& (second.gpu_address >= first.gpu_address + first.size
				|| second.gpu_address + second.size <= first.gpu_address),
		"GPU addresses %#" B_PRIx32 " and %#" B_PRIx32 " are page aligned and "
		"apart", first.gpu_address, second.gpu_address);

	drm_v3d_get_bo_offset offset = {};
	offset.handle = first.handle;
	error = drm_ioctl(DRM_IOCTL_V3D_GET_BO_OFFSET, &offset);
	check(error == 0 && offset.offset == first.gpu_address,
		"GET_BO_OFFSET %#" B_PRIx32 " (%s)", offset.offset, strerror(error));

	uint32 nonZero = 0;
	for (uint32 i = 0; i < first.size; i++) {
		if (first.data[i] != 0)
			nonZero++;
	}
	check(nonZero == 0, "new BO is zeroed (%" B_PRIu32 " bytes not)", nonZero);

	uint32* words = (uint32*)first.data;
	for (uint32 i = 0; i < first.size / 4; i++)
		words[i] = i * 2654435761u;
	uint32 wrong = 0;
	for (uint32 i = 0; i < first.size / 4; i++) {
		if (words[i] != i * 2654435761u)
			wrong++;
	}
	check(wrong == 0, "CPU write/read through the mapping (%" B_PRIu32
		" words wrong)", wrong);

	// A second mapping of the same BO sees the same memory.
	drm_v3d_mmap_bo map = {};
	map.handle = first.handle;
	error = drm_ioctl(DRM_IOCTL_V3D_MMAP_BO, &map);
	if (check(error == 0, "second mapping: %s", strerror(error))) {
		const uint32* other = (const uint32*)(addr_t)map.offset;
		check(other[123] == words[123] && (void*)other != (void*)words,
			"second mapping shares the pages");
		munmap((void*)other, first.size);
	}

	const uint32 handle = first.handle;
	buffer_destroy(first);
	error = buffer_close(handle);
	check(error != 0, "closing a closed handle refused (%s)", strerror(error));
	error = wait_bo(handle, 0);
	check(error != 0, "waiting for a closed handle refused (%s)",
		strerror(error));
	buffer_destroy(second);
}


static bool
run_clear(uint32 color, uint32 outSync, const char* label)
{
	ClearJob job;
	const bigtime_t start = system_time();
	int error = clear_job_submit(job, color, outSync, false);
	if (!check(error == 0, "%s: submit: %s", label, strerror(error)))
		return false;

	error = wait_bo(job.target.handle, 2000000000ull);
	const bigtime_t elapsed = system_time() - start;
	report("%s to %#010" B_PRIx32 ": wait %s after %" B_PRId64 " us", label,
		color, strerror(error), elapsed);
	const uint32 bad = clear_job_check(job);
	const bool ok = check(error == 0 && bad == 0, "%s %" B_PRIu32 "/%"
		B_PRIu32 " pixels", label, kPixels - bad, kPixels);
	clear_job_destroy(job);
	return ok;
}


static void
test_clear()
{
	static const uint32 kColors[] = { 0x80402010, 0xff00ff00, 0x12345678 };
	for (uint32 i = 0; i < B_COUNT_OF(kColors); i++)
		run_clear(kColors[i], 0, "clear");
}


static void
test_wait()
{
	ClearJob job;
	int error = clear_job_submit(job, 0xff0000ff, 0, false);
	if (!check(error == 0, "wait: submit: %s", strerror(error)))
		return;

	// The job has most likely not run yet; either answer is valid.
	error = wait_bo(job.target.handle, 0);
	report("WAIT_BO without time right after submit: %s",
		error == ETIME ? "ETIME (job pending)" : strerror(error));
	check(error == 0 || error == ETIME, "WAIT_BO polls");

	drm_v3d_wait_bo args = {};
	args.handle = job.target.handle;
	args.timeout_ns = 1000000000ull;
	error = drm_ioctl(DRM_IOCTL_V3D_WAIT_BO, &args);
	check(error == 0 && args.timeout_ns <= 1000000000ull,
		"WAIT_BO 1 s: %s, %" B_PRIu64 " ns left", strerror(error),
		args.timeout_ns);
	check(clear_job_check(job) == 0, "wait: pixels");

	error = wait_bo(job.target.handle, 0);
	check(error == 0, "WAIT_BO on an idle BO: %s", strerror(error));
	clear_job_destroy(job);
}


static int
syncobj_create(uint32 flags, uint32* _handle)
{
	drm_syncobj_create args = {};
	args.flags = flags;
	int error = drm_ioctl(DRM_IOCTL_SYNCOBJ_CREATE, &args);
	*_handle = args.handle;
	return error;
}


static int
syncobj_wait(uint32 handle, uint32 flags, int64 deadline)
{
	drm_syncobj_wait args = {};
	args.handles = (uint64)(addr_t)&handle;
	args.count_handles = 1;
	args.flags = flags;
	args.timeout_nsec = deadline;
	return drm_ioctl(DRM_IOCTL_SYNCOBJ_WAIT, &args);
}


static int
syncobj_array(unsigned long request, uint32 handle)
{
	drm_syncobj_array args = {};
	args.handles = (uint64)(addr_t)&handle;
	args.count_handles = 1;
	return drm_ioctl(request, &args);
}


static int
syncobj_destroy(uint32 handle)
{
	drm_syncobj_destroy args = {};
	args.handle = handle;
	return drm_ioctl(DRM_IOCTL_SYNCOBJ_DESTROY, &args);
}


static void
test_syncobj()
{
	uint32 syncobj;
	int error = syncobj_create(DRM_SYNCOBJ_CREATE_SIGNALED, &syncobj);
	if (!check(error == 0 && syncobj != 0, "syncobj create: %s",
			strerror(error)))
		return;

	error = syncobj_wait(syncobj, 0, 0);
	check(error == 0, "created signalled: %s", strerror(error));

	error = syncobj_array(DRM_IOCTL_SYNCOBJ_RESET, syncobj);
	check(error == 0, "reset: %s", strerror(error));
	error = syncobj_wait(syncobj, 0, monotonic_ns(10000));
	check(error == EINVAL, "waiting for nothing refused (%s)",
		strerror(error));
	error = syncobj_wait(syncobj, DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT,
		monotonic_ns(20000));
	check(error == ETIME, "WAIT_FOR_SUBMIT times out (%s)", strerror(error));

	error = syncobj_array(DRM_IOCTL_SYNCOBJ_SIGNAL, syncobj);
	check(error == 0, "signal: %s", strerror(error));
	error = syncobj_wait(syncobj, 0, 0);
	check(error == 0, "signalled: %s", strerror(error));

	// A job's out-sync, waited for with an absolute deadline.
	error = syncobj_array(DRM_IOCTL_SYNCOBJ_RESET, syncobj);
	ClearJob job;
	error = clear_job_submit(job, 0x00ff00ff, syncobj, false);
	if (check(error == 0, "syncobj: submit: %s", strerror(error))) {
		error = syncobj_wait(syncobj, 0, monotonic_ns(2000000));
		const uint32 bad = clear_job_check(job);
		check(error == 0 && bad == 0, "job out-sync: %s, %" B_PRIu32
			" pixels wrong", strerror(error), bad);
		clear_job_destroy(job);
	}

	error = syncobj_destroy(syncobj);
	check(error == 0, "destroy: %s", strerror(error));
	error = syncobj_destroy(syncobj);
	check(error != 0, "destroying twice refused (%s)", strerror(error));
}


static void
test_hang()
{
	int error;
	const uint64 resetsBefore = get_param(
		DRM_V3D_PARAM_GLOBAL_RESET_COUNTER, &error);

	ClearJob job;
	error = clear_job_submit(job, 0x11223344, 0, true);
	if (!check(error == 0, "hang: submit: %s", strerror(error)))
		return;

	error = wait_bo(job.target.handle, 100000000ull);
	check(error == ETIME, "hanging job times out a 100 ms wait (%s)",
		strerror(error));

	// The watchdog gives up after 500 ms without progress, resets the GPU
	// and drops the job, which completes its fence.
	const bigtime_t start = system_time();
	error = wait_bo(job.target.handle, 5000000000ull);
	check(error == 0, "watchdog ended the job after %" B_PRId64 " ms (%s)",
		(system_time() - start) / 1000, strerror(error));
	const uint64 resetsAfter
		= get_param(DRM_V3D_PARAM_GLOBAL_RESET_COUNTER);
	check(resetsAfter == resetsBefore + 1, "GPU resets %" B_PRIu64 " -> %"
		B_PRIu64, resetsBefore, resetsAfter);
	clear_job_destroy(job);

	run_clear(0x55aa55aa, 0, "clear after reset");
}


static void
test_orphan()
{
	ClearJob job;
	int error = clear_job_submit(job, 0x11223344, 0, true);
	check(error == 0, "orphan: submitted a hanging job, exiting without "
		"cleaning up (%s)", strerror(error));
	// The team's death closes the device; its areas go with it.
	exit(error == 0 ? 0 : 1);
}


static void
usage()
{
	fprintf(stderr, "usage: v3dtest [-d] [all|info|bo|clear|wait|syncobj|"
		"hang|orphan|areas]...\n"
		"  -d  also log to the syslog\n"
		"  all = info bo clear wait syncobj (the default)\n");
	exit(2);
}


int
main(int argc, char** argv)
{
	int first = 1;
	if (argc > 1 && strcmp(argv[1], "-d") == 0) {
		sSyslog = fopen("/dev/dprintf", "w");
		first = 2;
	}
	const char* defaults[] = { "all" };
	const char** tests = (const char**)argv + first;
	int testCount = argc - first;
	if (testCount == 0) {
		tests = defaults;
		testCount = 1;
	}

	sDevice = open(kDevicePath, O_RDWR);
	if (sDevice < 0) {
		report("cannot open %s: %s", kDevicePath, strerror(errno));
		return 1;
	}

	const int32 areasBefore = count_bo_areas();
	bool checkAreas = false;
	for (int i = 0; i < testCount; i++) {
		const char* test = tests[i];
		const bool all = strcmp(test, "all") == 0;
		if (all || strcmp(test, "info") == 0)
			test_info();
		if (all || strcmp(test, "bo") == 0)
			test_bo();
		if (all || strcmp(test, "clear") == 0)
			test_clear();
		if (all || strcmp(test, "wait") == 0)
			test_wait();
		if (all || strcmp(test, "syncobj") == 0)
			test_syncobj();
		if (strcmp(test, "hang") == 0)
			test_hang();
		if (strcmp(test, "orphan") == 0)
			test_orphan();
		if (strcmp(test, "areas") == 0)
			report("%" B_PRId32 " BO areas", count_bo_areas());
		else
			checkAreas = true;
		if (!all && strcmp(test, "info") != 0 && strcmp(test, "bo") != 0
			&& strcmp(test, "clear") != 0 && strcmp(test, "wait") != 0
			&& strcmp(test, "syncobj") != 0 && strcmp(test, "hang") != 0
			&& strcmp(test, "areas") != 0) {
			usage();
		}
	}

	if (checkAreas) {
		// Everything was closed; quarantined BOs go when their jobs are done.
		int32 areasAfter = count_bo_areas();
		for (int32 i = 0; i < 20 && areasAfter > areasBefore; i++) {
			snooze(100000);
			areasAfter = count_bo_areas();
		}
		check(areasAfter == areasBefore, "BO areas %" B_PRId32 " -> %" B_PRId32,
			areasBefore, areasAfter);
	}

	close(sDevice);
	report("%d of %d checks passed%s", sChecks - sFailures, sChecks,
		sFailures == 0 ? "" : " - FAILURES");
	if (sSyslog != NULL)
		fclose(sSyslog);
	return sFailures == 0 ? 0 : 1;
}
