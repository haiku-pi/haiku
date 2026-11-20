/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The scan-out pool: memory below 1 GiB, taken while the system boots, for
	the pictures the display shows as they are (RPIVID_PICTURE_SCAN_OUT).

	The display keeps 30 address bits, and every page the system allocates
	comes from the lowest memory first (the free page list starts in address
	order, and freed pages are reused first): after the first minutes of use,
	the memory below 1 GiB is in use or scattered, and a 4K stream's pictures
	(15 runs of 12.7 MiB) are not to be had any more. The pool is taken when
	the driver is initialised, while that memory is still in one piece.

	Each picture gets an area of its own over its part of the pool (a
	physical mapping: the client's and app_server's clones work as for any
	area), cleared before it is handed out. A picture that does not fit is
	allocated as before. A clone of a physical mapping does not keep the
	memory: a picture's part of the pool is handed out again only once no
	clone of its area is left (a client could otherwise keep reading the
	next stream's pictures, and the display could scan out memory that is
	being reused), see pool_sweep().

	Settings (driver settings file "rpivid"): scan_out_pool <MiB>; the
	default is sized for one 4K 8-bit stream, at most 1/16 of the memory;
	0 turns the pool off.
*/


#include "rpivid_private.h"

#include <arch/cpu.h>
#include <driver_settings.h>
#include <util/AutoLock.h>
#include <vm/VMArea.h>
#include <vm/VMCache.h>
#include <vm/vm.h>

#include <stdlib.h>


// one 3840x2160 8-bit stream: its DPB, the pictures a player holds, the one
// being decoded and the next (15 of 12.7 MiB)
static const size_t kDefaultPoolSize = 192 * 1024 * 1024;
static const phys_addr_t kScanOutLimit = (phys_addr_t)1 << 30;


/*!	Whether \a id, an area of ours, still has clones. */
static bool
area_is_cloned(area_id id)
{
	VMArea* area = VMAreas::Lookup(id);
	if (area == NULL)
		return false;
	VMCache* cache = vm_area_get_locked_cache(area);
	const bool cloned = cache->areas.Head() != cache->areas.Tail();
	vm_area_put_locked_cache(cache);
	return cloned;
}


/*!	Removes the extent at \a offset. The pool is locked. */
static void
remove_extent(rpivid_pool& pool, size_t offset)
{
	for (uint32 i = 0; i < pool.count; i++) {
		if (pool.extents[i].offset == offset) {
			pool.used -= pool.extents[i].size;
			memmove(&pool.extents[i], &pool.extents[i + 1],
				(pool.count - i - 1) * sizeof(rpivid_pool::extent));
			pool.count--;
			return;
		}
	}
	panic("rpivid: offset %#" B_PRIxSIZE " is not in the scan-out pool",
		offset);
}


/*!	Gives back the released pictures' parts of the pool whose areas have no
	clone left.
*/
static void
pool_sweep(rpivid_pool& pool)
{
	MutexLocker locker(pool.lock);
	for (uint32 i = 0; i < pool.count;) {
		rpivid_pool::extent& extent = pool.extents[i];
		if (!extent.released || area_is_cloned(extent.area)) {
			i++;
			continue;
		}
		delete_area(extent.area);
		remove_extent(pool, extent.offset);
	}
}


status_t
rpivid_pool_init(rpivid_device* device)
{
	rpivid_pool& pool = device->pool;
	pool.area = -1;
	mutex_init(&pool.lock, "rpivid scan-out pool");

	size_t size = kDefaultPoolSize;
	system_info info;
	if (get_system_info(&info) == B_OK)
		size = min_c(size, (size_t)(info.max_pages * B_PAGE_SIZE / 16));
	void* settings = load_driver_settings("rpivid");
	const char* value = get_driver_parameter(settings, "scan_out_pool", NULL,
		NULL);
	if (value != NULL)
		size = (size_t)strtoul(value, NULL, 0) * 1024 * 1024;
	unload_driver_settings(settings);
	size = ROUNDDOWN(size, B_PAGE_SIZE);
	if (size == 0)
		return B_OK;

	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = min_c(kScanOutLimit, device->dma_end);
	void* address;
	area_id area = create_area_etc(B_SYSTEM_TEAM, "rpivid scan-out pool",
		size, B_CONTIGUOUS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0,
		&virtualRestrictions, &physicalRestrictions, &address);
	if (area < 0) {
		ERROR("no scan-out pool of %" B_PRIuSIZE " MiB: %s\n",
			size / 1024 / 1024, strerror(area));
		return area;
	}
	physical_entry entry;
	uint32 count = 1;
	status_t status = get_memory_map_etc(B_SYSTEM_TEAM, address, B_PAGE_SIZE,
		&entry, &count);
	if (status == B_BUFFER_OVERFLOW)
		status = B_OK;
	if (status != B_OK) {
		delete_area(area);
		return status;
	}

	pool.area = area;
	pool.address = (uint8*)address;
	pool.physical = entry.address;
	pool.size = size;
	TRACE_ALWAYS("scan-out pool: %" B_PRIuSIZE " MiB at %#" B_PRIxPHYSADDR
		"\n", size / 1024 / 1024, pool.physical);
	return B_OK;
}


void
rpivid_pool_uninit(rpivid_device* device)
{
	rpivid_pool& pool = device->pool;
	// every picture is gone with the files; the driver goes only when no
	// file is open, and the clients' clones went with their teams
	for (uint32 i = 0; i < pool.count; i++) {
		if (pool.extents[i].area >= 0)
			delete_area(pool.extents[i].area);
	}
	pool.count = 0;
	if (pool.area >= 0)
		delete_area(pool.area);
	pool.area = -1;
	mutex_destroy(&pool.lock);
}


/*!	A cleared part of the pool of \a size bytes as \a memory (its own area,
	\a memoryType B_WRITE_COMBINING_MEMORY or 0, cached), or B_NO_MEMORY.
*/
status_t
rpivid_pool_alloc(rpivid_device* device, const char* name, size_t size,
	uint32 memoryType, dma_memory& memory)
{
	rpivid_pool& pool = device->pool;
	size = ROUNDUP(size, B_PAGE_SIZE);
	if (pool.area < 0 || size > pool.size)
		return B_NO_MEMORY;

	pool_sweep(pool);
	MutexLocker locker(pool.lock);
	if (pool.count == kMaxPoolExtents)
		return B_NO_MEMORY;
	// first fit between the extents in use (sorted by offset)
	size_t offset = 0;
	uint32 index = 0;
	for (; index < pool.count; index++) {
		if (pool.extents[index].offset - offset >= size)
			break;
		offset = pool.extents[index].offset + pool.extents[index].size;
	}
	if (offset + size > pool.size) {
		pool.misses++;
		return B_NO_MEMORY;
	}
	memmove(&pool.extents[index + 1], &pool.extents[index],
		(pool.count - index) * sizeof(rpivid_pool::extent));
	pool.extents[index].offset = offset;
	pool.extents[index].size = size;
	pool.extents[index].area = -1;
	pool.extents[index].released = false;
	pool.count++;
	pool.used += size;
	pool.peak = max_c(pool.peak, pool.used);
	locker.Unlock();

	// what an earlier client had there must not show through
	memset(pool.address + offset, 0, size);
	arm64_dcache_flush_range((addr_t)pool.address + offset, size);

	void* address;
	area_id area = map_physical_memory(name, pool.physical + offset, size,
		B_ANY_KERNEL_ADDRESS
			| (memoryType != 0 ? memoryType : B_WRITE_BACK_MEMORY),
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &address);
	locker.Lock();
	if (area < 0) {
		remove_extent(pool, offset);
		return area;
	}
	for (uint32 i = 0; i < pool.count; i++) {
		if (pool.extents[i].offset == offset)
			pool.extents[i].area = area;
	}
	locker.Unlock();

	memory.area = area;
	memory.address = address;
	memory.physical = pool.physical + offset;
	memory.size = size;
	memory.pool = &pool;
	return B_OK;
}


/*!	The picture of \a memory is gone: its part of the pool goes back once
	its area has no clone left (now, or when a later allocation sweeps).
*/
void
rpivid_pool_release(rpivid_pool& pool, dma_memory& memory)
{
	{
		MutexLocker locker(pool.lock);
		const size_t offset = memory.physical - pool.physical;
		for (uint32 i = 0; i < pool.count; i++) {
			if (pool.extents[i].offset == offset)
				pool.extents[i].released = true;
		}
	}
	pool_sweep(pool);
}


/*!	How many released pictures still wait for their clones to go. */
uint32
rpivid_pool_released(rpivid_pool& pool)
{
	pool_sweep(pool);
	MutexLocker locker(pool.lock);
	uint32 count = 0;
	for (uint32 i = 0; i < pool.count; i++) {
		if (pool.extents[i].released)
			count++;
	}
	return count;
}
