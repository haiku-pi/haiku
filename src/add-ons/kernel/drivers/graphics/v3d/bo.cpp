/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Buffer objects (BOs).

	A BO is a wired kernel area (not physically contiguous: the GPU has an
	MMU), cleared and mapped write-combining, and entered into the GPU page
	table at a fixed GPU address for its whole life: Mesa writes that address
	straight into its control lists.

	Haiku cannot mmap() a device, so MMAP_BO clones the BO's area into the
	calling team and returns the address of the clone in "offset"; the client
	uses it as its mapping and deletes it with munmap() (or delete_area()).

	Freeing a BO never returns memory the GPU may still write: its page table
	entries are cleared first, and its pages and GPU addresses are released
	only once every job submitted before the free has completed - including
	jobs that use the BO without listing it.
*/


#include "v3d_private.h"

#include <arch/cpu.h>
#include <kernel.h>
#include <team.h>
#include <util/AutoLock.h>
#include <vm/vm.h>


static const size_t kMaxBOSize = 512 * 1024 * 1024;


static void
bo_release(v3d_device* device, v3d_bo* bo)
{
	v3d_va_free(device, bo->gpu_address, bo->size);
	if (bo->area >= 0)
		delete_area(bo->area);
	else {
		unlock_memory_etc(bo->imported_team, bo->address, bo->size,
			B_READ_DEVICE);
	}
	free(bo);
}


/*!	The last reference is gone: unmap the BO from the GPU now, release its
	memory once the GPU is done with every job that could still use it.
*/
static void
bo_free(v3d_device* device, v3d_bo* bo)
{
	v3d_unmap_range(device, bo->gpu_address, bo->size);
	if (device->active == NULL)
		v3d_mmu_flush(device);
	// else: the TLB may still hold the old entries for the running job, but
	// its pages stay until it is done, and every job flushes the TLB first.

	bo->release_after = device->submitted_seqno;
	if (device->completed_seqno >= bo->release_after) {
		bo_release(device, bo);
		return;
	}

	bo->next = device->quarantine;
	device->quarantine = bo;
}


void
v3d_bo_put(v3d_device* device, v3d_bo* bo)
{
	if (--bo->references == 0)
		bo_free(device, bo);
}


void
v3d_bo_release_quarantine(v3d_device* device)
{
	v3d_bo** link = &device->quarantine;
	while (*link != NULL) {
		v3d_bo* bo = *link;
		if (device->completed_seqno >= bo->release_after) {
			*link = bo->next;
			bo_release(device, bo);
		} else
			link = &bo->next;
	}
}


v3d_bo*
v3d_bo_lookup(v3d_file* file, uint32 handle)
{
	return v3d_handle_lookup(file->bos, file->bo_slots, handle);
}


void
v3d_file_close_bos(v3d_file* file)
{
	for (uint32 i = 0; i < file->bo_slots; i++) {
		if (file->bos[i] != NULL)
			v3d_bo_put(file->device, file->bos[i]);
	}
	free(file->bos);
	file->bos = NULL;
	file->bo_slots = 0;
}


//	#pragma mark - ioctls


static status_t
create_bo(v3d_file* file, drm_v3d_create_bo* args)
{
	v3d_device* device = file->device;
	if (args->flags != 0)
		return B_BAD_VALUE;
	const size_t size = ROUNDUP((size_t)args->size, B_PAGE_SIZE);
	if (size == 0 || size > kMaxBOSize)
		return B_BAD_VALUE;

	v3d_bo* bo = (v3d_bo*)calloc(1, sizeof(v3d_bo));
	if (bo == NULL)
		return B_NO_MEMORY;
	bo->size = size;
	bo->references = 1;

	// The pages arrive cleared: a BO the GPU sees first must hold zeroes
	// (Phoenix: a garbage binner BO wedged the render CLE).
	bo->area = v3d_create_gpu_memory("v3d bo", size, false, &bo->address,
		NULL);
	if (bo->area < 0) {
		status_t status = bo->area;
		free(bo);
		return status;
	}

	MutexLocker locker(device->lock);
	status_t status = v3d_va_alloc(device, size, &bo->gpu_address);
	if (status == B_OK) {
		status = v3d_map_area(device, bo->address, size, bo->gpu_address);
		if (status != B_OK)
			v3d_va_free(device, bo->gpu_address, size);
	}
	uint32 handle = 0;
	if (status == B_OK) {
		status = v3d_handle_alloc(file->bos, file->bo_slots, bo, &handle);
		if (status != B_OK) {
			// never entered into the TLB: no flush needed
			v3d_unmap_range(device, bo->gpu_address, size);
			v3d_va_free(device, bo->gpu_address, size);
		}
	}
	if (status != B_OK) {
		delete_area(bo->area);
		free(bo);
		return status;
	}
	// The next job flushes the MMU before it starts.

	args->handle = handle;
	args->offset = bo->gpu_address;
	return B_OK;
}


static status_t
mmap_bo(v3d_file* file, drm_v3d_mmap_bo* args)
{
	v3d_device* device = file->device;
	if (args->flags != 0)
		return B_BAD_VALUE;

	MutexLocker locker(device->lock);
	v3d_bo* bo = v3d_bo_lookup(file, args->handle);
	if (bo == NULL)
		return B_ENTRY_NOT_FOUND;
	if (bo->area < 0)
		return B_NOT_ALLOWED;
			// imported: the client has it mapped already
	bo->references++;
		// keeps the BO while we clone it unlocked
	locker.Unlock();

	void* address = NULL;
	area_id clone = vm_clone_area(team_get_current_team_id(), "v3d bo map",
		&address, B_ANY_ADDRESS, B_READ_AREA | B_WRITE_AREA,
		REGION_NO_PRIVATE_MAP, bo->area, true);
	status_t status = clone >= 0 ? B_OK : clone;
	if (status == B_OK) {
		// A clone of RAM does not inherit the memory type of its source.
		physical_entry entry;
		uint32 count = 1;
		status = get_memory_map_etc(B_SYSTEM_TEAM, bo->address, B_PAGE_SIZE,
			&entry, &count);
		if (status == B_OK || status == B_BUFFER_OVERFLOW) {
			status = vm_set_area_memory_type(clone, entry.address,
				B_WRITE_COMBINING_MEMORY);
		}
		if (status == B_OK) {
			// The clone was mapped cacheable for a moment: drop whatever
			// lines that may have pulled in (they are clean).
			arm64_dcache_flush_range((addr_t)bo->address, bo->size);
		} else
			vm_delete_area(team_get_current_team_id(), clone, true);
	}

	locker.Lock();
	v3d_bo_put(device, bo);
	if (status != B_OK)
		return status;

	args->offset = (addr_t)address;
	return B_OK;
}


static status_t
get_bo_offset(v3d_file* file, drm_v3d_get_bo_offset* args)
{
	MutexLocker locker(file->device->lock);
	v3d_bo* bo = v3d_bo_lookup(file, args->handle);
	if (bo == NULL)
		return B_ENTRY_NOT_FOUND;
	args->offset = bo->gpu_address;
	return B_OK;
}


static status_t
gem_close(v3d_file* file, drm_gem_close* args)
{
	MutexLocker locker(file->device->lock);
	v3d_bo* bo = v3d_bo_lookup(file, args->handle);
	if (bo == NULL)
		return B_BAD_VALUE;
	file->bos[args->handle - 1] = NULL;
	v3d_bo_put(file->device, bo);
	return B_OK;
}


/*!	A range of the calling team's memory as a BO (a Haiku extension): the
	pages are wired for the BO's life and entered into the GPU page table, so
	that the GPU renders straight into, say, a BBitmap. The range need not be
	page aligned: offset is the GPU address of its first byte.
*/
static status_t
import_memory(v3d_file* file, drm_v3d_haiku_import_memory* args)
{
	v3d_device* device = file->device;
	const addr_t start = ROUNDDOWN((addr_t)args->address, B_PAGE_SIZE);
	const addr_t end = ROUNDUP((addr_t)(args->address + args->size), B_PAGE_SIZE);
	const size_t size = end - start;
	if (args->size == 0 || size > kMaxBOSize || end < start
		|| !IS_USER_ADDRESS(start) || !IS_USER_ADDRESS(end - 1)) {
		return B_BAD_VALUE;
	}

	v3d_bo* bo = (v3d_bo*)calloc(1, sizeof(v3d_bo));
	if (bo == NULL)
		return B_NO_MEMORY;
	bo->size = size;
	bo->references = 1;
	bo->area = -1;
	bo->address = (void*)start;
	bo->imported_team = team_get_current_team_id();

	// The GPU writes it: as a device reading into memory
	status_t status = lock_memory_etc(bo->imported_team, bo->address, size,
		B_READ_DEVICE);
	if (status != B_OK) {
		free(bo);
		return status;
	}

	MutexLocker locker(device->lock);
	status = v3d_va_alloc(device, size, &bo->gpu_address);
	if (status == B_OK) {
		status = v3d_map_memory(device, bo->imported_team, bo->address, size,
			bo->gpu_address);
		if (status != B_OK)
			v3d_va_free(device, bo->gpu_address, size);
	}
	uint32 handle = 0;
	if (status == B_OK) {
		status = v3d_handle_alloc(file->bos, file->bo_slots, bo, &handle);
		if (status != B_OK) {
			v3d_unmap_range(device, bo->gpu_address, size);
			v3d_va_free(device, bo->gpu_address, size);
		}
	}
	if (status != B_OK) {
		unlock_memory_etc(bo->imported_team, bo->address, size, B_READ_DEVICE);
		free(bo);
		return status;
	}

	args->handle = handle;
	args->offset = bo->gpu_address + (uint32)(args->address - start);
	return B_OK;
}


/*!	Writes back and drops the CPU's cache lines of an imported BO (through the
	physical map: the GPU does not snoop the caches).
*/
static status_t
sync_bo(v3d_file* file, drm_v3d_haiku_sync_bo* args)
{
	MutexLocker locker(file->device->lock);
	v3d_bo* bo = v3d_bo_lookup(file, args->handle);
	if (bo == NULL)
		return B_BAD_VALUE;
	if (bo->area >= 0)
		return B_OK;
			// write-combining: nothing cached

	size_t offset = 0;
	while (offset < bo->size) {
		physical_entry ranges[16];
		uint32 count = B_COUNT_OF(ranges);
		status_t status = get_memory_map_etc(bo->imported_team,
			(uint8*)bo->address + offset, bo->size - offset, ranges, &count);
		if (status != B_OK && status != B_BUFFER_OVERFLOW)
			return status;
		for (uint32 i = 0; i < count; i++) {
			arm64_dcache_flush_range(KERNEL_PMAP_BASE + ranges[i].address,
				ranges[i].size);
			offset += ranges[i].size;
		}
	}
	return B_OK;
}


/*!	Waits until the last job using the BO completed. The timeout is
	relative; as Linux, the time not used is returned in timeout_ns.
*/
static status_t
wait_bo(v3d_file* file, drm_v3d_wait_bo* args)
{
	v3d_device* device = file->device;
	if (args->pad != 0)
		return B_BAD_VALUE;

	const bigtime_t start = system_time();
	bigtime_t deadline;
	if (args->timeout_ns >= (uint64)INT64_MAX / 2)
		deadline = B_INFINITE_TIMEOUT;
	else
		deadline = start + (bigtime_t)((args->timeout_ns + 999) / 1000);

	MutexLocker locker(device->lock);
	v3d_bo* bo = v3d_bo_lookup(file, args->handle);
	if (bo == NULL)
		return B_BAD_VALUE;
	status_t status = v3d_wait_seqno(device, file, bo->last_seqno,
		args->timeout_ns == 0 ? 0 : deadline);
	locker.Unlock();

	if (deadline != B_INFINITE_TIMEOUT) {
		const uint64 used = (uint64)(system_time() - start) * 1000;
		args->timeout_ns = used < args->timeout_ns
			? args->timeout_ns - used : 0;
	}
	return status;
}


status_t
v3d_bo_ioctl(v3d_file* file, uint32 nr, void* data)
{
	switch (nr) {
		case DRM_COMMAND_BASE + DRM_V3D_CREATE_BO:
			return create_bo(file, (drm_v3d_create_bo*)data);
		case DRM_COMMAND_BASE + DRM_V3D_MMAP_BO:
			return mmap_bo(file, (drm_v3d_mmap_bo*)data);
		case DRM_COMMAND_BASE + DRM_V3D_GET_BO_OFFSET:
			return get_bo_offset(file, (drm_v3d_get_bo_offset*)data);
		case DRM_COMMAND_BASE + DRM_V3D_WAIT_BO:
			return wait_bo(file, (drm_v3d_wait_bo*)data);
		case DRM_COMMAND_BASE + DRM_V3D_HAIKU_IMPORT_MEMORY:
			return import_memory(file, (drm_v3d_haiku_import_memory*)data);
		case DRM_COMMAND_BASE + DRM_V3D_HAIKU_SYNC_BO:
			return sync_bo(file, (drm_v3d_haiku_sync_bo*)data);
		case 0x09:
			// DRM_IOCTL_GEM_CLOSE
			return gem_close(file, (drm_gem_close*)data);
	}
	return B_DEV_INVALID_IOCTL;
}
