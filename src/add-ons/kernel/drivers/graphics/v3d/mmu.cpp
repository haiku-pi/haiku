/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The V3D MMU and the memory the GPU sees.

	All clients share one 4 GiB GPU address space with a single-level page
	table (4 MiB, physically contiguous), as Linux does: the page table base
	register is global, and switching tables per client would cost a TLB
	flush per job anyway. Accesses to unmapped pages are redirected to a
	scratch page and raise an MMU interrupt.

	The GPU does not snoop the CPU caches. Everything it reads or writes is
	mapped write-combining (Normal non-cacheable) on the CPU side, so CPU
	stores only need a barrier before a job starts.
*/


#include "v3d_private.h"

#include <arch/cpu.h>
#include <vm/vm.h>


static const uint32 kScratchSize = B_PAGE_SIZE;


/*!	Allocates wired memory for the GPU, cleared and mapped write-combining.
	Contiguous memory (page table, scratch page) is taken below 4 GiB.
*/
area_id
v3d_create_gpu_memory(const char* name, size_t size, bool contiguous,
	void** _address, phys_addr_t* _physical)
{
	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	physical_address_restrictions physicalRestrictions = {};
	if (contiguous)
		physicalRestrictions.high_address = (phys_addr_t)1 << 32;

	area_id area = create_area_etc(B_SYSTEM_TEAM, name, size,
		contiguous ? B_CONTIGUOUS : B_FULL_LOCK,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0, &virtualRestrictions,
		&physicalRestrictions, _address);
	if (area < 0)
		return area;

	physical_entry entry;
	uint32 count = 1;
	status_t status = get_memory_map_etc(B_SYSTEM_TEAM, *_address,
		B_PAGE_SIZE, &entry, &count);
	if (status == B_OK || status == B_BUFFER_OVERFLOW) {
		// The pages were cleared through the cacheable physical map: write
		// those lines back and drop them, so that neither the GPU nor a
		// non-cacheable mapping sees stale memory, and no dirty line can be
		// evicted over what the GPU writes later.
		arm64_dcache_flush_range((addr_t)*_address, size);
		status = vm_set_area_memory_type(area, entry.address,
			B_WRITE_COMBINING_MEMORY);
	}
	if (status != B_OK) {
		delete_area(area);
		return status;
	}

	if (_physical != NULL)
		*_physical = entry.address;
	return area;
}


/*!	The page table and the scratch page are allocated when the driver is
	initialized (the GPU is not touched then): 4 MiB of contiguous memory are
	easy to find at boot, and not necessarily later.
*/
status_t
v3d_page_table_alloc(v3d_device* device)
{
	void* address;
	device->page_table_area = v3d_create_gpu_memory("v3d page table",
		kPageTableSize, true, &address, &device->page_table_physical);
	if (device->page_table_area < 0)
		return device->page_table_area;
	device->page_table = (uint32*)address;

	device->scratch_area = v3d_create_gpu_memory("v3d scratch page",
		kScratchSize, true, &address, &device->scratch_physical);
	if (device->scratch_area < 0) {
		delete_area(device->page_table_area);
		device->page_table_area = -1;
		return device->scratch_area;
	}
	return B_OK;
}


//	#pragma mark - GPU virtual addresses


static inline bool
va_page_used(v3d_device* device, uint32 page)
{
	return (device->va_bitmap[page / 32] & (1u << (page % 32))) != 0;
}


static void
va_mark(v3d_device* device, uint32 first, uint32 count, bool used)
{
	for (uint32 page = first; page < first + count; page++) {
		if (used)
			device->va_bitmap[page / 32] |= 1u << (page % 32);
		else
			device->va_bitmap[page / 32] &= ~(1u << (page % 32));
	}
}


/*!	First fit. va_hint is the lowest page that may be free. */
status_t
v3d_va_alloc(v3d_device* device, size_t size, uint32* _address)
{
	const uint32 pages = size >> kGpuPageShift;
	if (pages == 0 || pages >= kPageTableEntries)
		return B_BAD_VALUE;

	uint32 runStart = device->va_hint;
	uint32 page = runStart;
	while (page < kPageTableEntries) {
		if (page % 32 == 0 && device->va_bitmap[page / 32] == ~0u) {
			page += 32;
			runStart = page;
			continue;
		}
		if (va_page_used(device, page)) {
			runStart = ++page;
			continue;
		}
		if (++page - runStart < pages)
			continue;

		va_mark(device, runStart, pages, true);
		if (runStart == device->va_hint)
			device->va_hint = page;
		*_address = runStart << kGpuPageShift;
		return B_OK;
	}
	return B_NO_MEMORY;
}


void
v3d_va_free(v3d_device* device, uint32 address, size_t size)
{
	const uint32 first = address >> kGpuPageShift;
	va_mark(device, first, size >> kGpuPageShift, false);
	if (first < device->va_hint)
		device->va_hint = first;
}


//	#pragma mark - page table


/*!	Enters the pages of a wired kernel area into the page table at
	\a gpuAddress. The caller flushes the MMU before the GPU may use them.
*/
status_t
v3d_map_area(v3d_device* device, void* address, size_t size,
	uint32 gpuAddress)
{
	uint32 entry = gpuAddress >> kGpuPageShift;
	size_t offset = 0;
	while (offset < size) {
		physical_entry ranges[16];
		uint32 count = B_COUNT_OF(ranges);
		status_t status = get_memory_map_etc(B_SYSTEM_TEAM,
			(uint8*)address + offset, size - offset, ranges, &count);
		if (status != B_OK && status != B_BUFFER_OVERFLOW)
			return status;

		for (uint32 i = 0; i < count; i++) {
			for (phys_size_t done = 0; done < ranges[i].size;
					done += B_PAGE_SIZE) {
				phys_addr_t physical = ranges[i].address + done;
				if (physical + B_PAGE_SIZE > device->physical_limit) {
					ERROR("page %#" B_PRIxPHYSADDR " is beyond the GPU's "
						"reach\n", physical);
					v3d_unmap_range(device, gpuAddress, offset);
					return B_NO_MEMORY;
				}
				device->page_table[entry++] = (uint32)(physical
					>> kGpuPageShift) | kPteValid | kPteWritable;
				offset += B_PAGE_SIZE;
			}
		}
	}
	return B_OK;
}


void
v3d_unmap_range(v3d_device* device, uint32 gpuAddress, size_t size)
{
	uint32 first = gpuAddress >> kGpuPageShift;
	for (uint32 i = 0; i < size >> kGpuPageShift; i++)
		device->page_table[first + i] = 0;
}


/*!	Waits (bounded, 100 ms as Linux) until \a bit of a register clears. */
bool
v3d_wait_for_clear(volatile uint32* registers, uint32 offset, uint32 bit,
	const char* what)
{
	bigtime_t timeout = system_time() + 100000;
	while ((registers[offset / 4] & bit) != 0) {
		if (system_time() > timeout) {
			ERROR("%s does not finish\n", what);
			return false;
		}
	}
	return true;
}


/*!	Makes page table changes visible to the GPU: flush the MMU's PTE cache,
	then clear its TLB (as Linux v3d_mmu.c).
*/
void
v3d_mmu_flush(v3d_device* device)
{
	if (device->broken)
		return;

	// The PTE stores (non-cacheable) must reach memory first.
	v3d_barrier();

	hub_write(device, kMmuCacheControl, kMmuCacheFlush | kMmuCacheEnable);
	v3d_wait_for_clear(device->hub, kMmuCacheControl, kMmuCacheFlushing,
		"MMU cache flush");

	hub_write(device, kMmuControl,
		hub_read(device, kMmuControl) | kMmuTlbClear);
	v3d_wait_for_clear(device->hub, kMmuControl, kMmuTlbClearing, "TLB clear");
}


/*!	The invariant state of the GPU, after power-on and after every reset.
	The register set is the one proven on this board by the Phoenix-RTOS
	server (its apply_core_regs): the MMU as Linux v3d_mmu_set_page_table(),
	caches and GFXH-1383 as Linux v3d_init_core(), and a MISCCFG that also
	limits QRMAXCNT.
*/
void
v3d_apply_core_registers(v3d_device* device)
{
	hub_write(device, kMmuPageTableBase,
		(uint32)(device->page_table_physical >> kGpuPageShift));
	hub_write(device, kMmuControl, kMmuEnable | kMmuInvalidPteEnable
		| kMmuInvalidPteAbort | kMmuInvalidPteInt | kMmuWriteViolationAbort
		| kMmuWriteViolationInt | kMmuCapExceededAbort
		| kMmuCapExceededInt);
	hub_write(device, kMmuIllegalAddress,
		(uint32)(device->scratch_physical >> kGpuPageShift)
			| kMmuIllegalAddressEnable);
	hub_write(device, kMmuCacheControl, kMmuCacheEnable);

	core_write(device, kCoreL2CacheControl, kL2CacheClear | kL2CacheEnable);
	core_write(device, kCoreL2TFlushStart, 0);
	core_write(device, kCoreL2TFlushEnd, ~0u);

	// GFXH-1383: limit AXI bursts
	hub_write(device, kHubAxiConfig, 0xf);

	core_write(device, kCoreMiscConfig, (2 << kMiscQrMaxCountShift)
		| kMiscOverrideTmuOutput);
}


/*!	Called once, the GPU powered and its interrupts masked. */
status_t
v3d_mmu_init(v3d_device* device)
{
	if (device->va_bitmap != NULL) {
		// an earlier start failed later on
		v3d_apply_core_registers(device);
		v3d_mmu_flush(device);
		return B_OK;
	}

	// The GPU's physical address width, as Linux v3d_drv.c reads it
	const uint32 debugInfo = hub_read(device, kMmuDebugInfo);
	device->physical_limit = (uint64)1 << (30 + ((debugInfo >> 8) & 0xf));

	device->va_bitmap = (uint32*)calloc(kPageTableEntries / 32,
		sizeof(uint32));
	if (device->va_bitmap == NULL)
		return B_NO_MEMORY;
	// As Linux: 0 is never a valid GPU address (some units treat it
	// specially), and a BO never ends at the top of the address space.
	va_mark(device, 0, 1, true);
	va_mark(device, kPageTableEntries - 1, 1, true);
	device->va_hint = 1;

	// Binner overflow memory, mapped once: the interrupt handler hands out
	// chunks of it without touching the page table.
	void* address;
	const size_t overflowSize = kOverflowChunkSize * kOverflowChunks;
	device->overflow_area = v3d_create_gpu_memory("v3d binner overflow",
		overflowSize, false, &address, NULL);
	if (device->overflow_area < 0)
		return device->overflow_area;
	status_t status = v3d_va_alloc(device, overflowSize,
		&device->overflow_gpu_address);
	if (status == B_OK) {
		status = v3d_map_area(device, address, overflowSize,
			device->overflow_gpu_address);
	}
	if (status != B_OK) {
		delete_area(device->overflow_area);
		device->overflow_area = -1;
		return status;
	}
	device->overflow_free = ~0u >> (32 - kOverflowChunks);

	v3d_apply_core_registers(device);
	v3d_mmu_flush(device);

	TRACE_ALWAYS("MMU: page table %#" B_PRIxPHYSADDR ", scratch %#"
		B_PRIxPHYSADDR ", physical limit %#" B_PRIx64 ", overflow pool at %#"
		B_PRIx32 "\n", device->page_table_physical, device->scratch_physical,
		device->physical_limit, device->overflow_gpu_address);
	return B_OK;
}


void
v3d_mmu_uninit(v3d_device* device)
{
	if (device->overflow_area >= 0)
		delete_area(device->overflow_area);
	device->overflow_area = -1;
	free(device->va_bitmap);
	device->va_bitmap = NULL;

	if (device->scratch_area >= 0)
		delete_area(device->scratch_area);
	if (device->page_table_area >= 0)
		delete_area(device->page_table_area);
	device->scratch_area = device->page_table_area = -1;
}
