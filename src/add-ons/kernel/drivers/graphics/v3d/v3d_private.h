/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef V3D_PRIVATE_H
#define V3D_PRIVATE_H


#include <stdlib.h>
#include <string.h>

#include <OS.h>
#include <KernelExport.h>
#include <device_manager.h>

#include <condition_variable.h>
#include <kernel.h>
#include <lock.h>

#include <drm/v3d_drm.h>


#define TRACE_ALWAYS(x...)	dprintf("v3d: " x)
#define ERROR(x...)			dprintf("v3d: " x)


// Hub registers
enum {
	kHubAxiConfig			= 0x0000,
	kHubUifConfig			= 0x0004,
	kHubIdent0				= 0x0008,
	kHubIdent1				= 0x000c,
	kHubIdent2				= 0x0010,
	kHubIdent3				= 0x0014,
	kHubInterruptStatus		= 0x0050,
	kHubInterruptClear		= 0x0058,
	kHubInterruptMaskSet	= 0x0060,
	kHubInterruptMaskClear	= 0x0064,

	// texture formatting unit (V3D 4.x offsets)
	kTfuControlStatus		= 0x0400,
	kTfuInputConfig			= 0x0408,
	kTfuInputImageAddress	= 0x040c,
	kTfuInputChromaAddress	= 0x0410,
	kTfuInputImageStride	= 0x0414,
	kTfuInputUAddress		= 0x0418,
	kTfuOutputAddress		= 0x041c,
	kTfuOutputSize			= 0x0420,
	kTfuCoefficient0		= 0x0424,
	kTfuCoefficient1		= 0x0428,
	kTfuCoefficient2		= 0x042c,
	kTfuCoefficient3		= 0x0430,

	kMmuCacheControl		= 0x1000,
	kMmuControl				= 0x1200,
	kMmuPageTableBase		= 0x1204,
	kMmuViolationId			= 0x122c,
	kMmuIllegalAddress		= 0x1230,
	kMmuViolationAddress	= 0x1234,
	kMmuDebugInfo			= 0x1238,
};

// TFU bits
enum {
	kTfuInterruptOnCompletion = 1 << 0,
	kTfuUseCoefficients		= 1u << 31,
};

// Hub interrupts
enum {
	kHubTfuDone				= 1 << 1,
	kHubMmuCapExceeded		= 1 << 3,
	kHubMmuInvalidPte		= 1 << 4,
	kHubMmuWriteViolation	= 1 << 5,
	kHubMmuFaults			= kHubMmuCapExceeded | kHubMmuInvalidPte
		| kHubMmuWriteViolation,
	// As Linux: TFU failures are not an interrupt source.
	kHubInterrupts			= kHubMmuFaults | kHubTfuDone,
};

// MMU bits
enum {
	kMmuCacheEnable			= 1 << 0,
	kMmuCacheFlush			= 1 << 1,
	kMmuCacheFlushing		= 1 << 2,

	kMmuEnable				= 1 << 0,
	kMmuTlbClear			= 1 << 2,
	kMmuTlbClearing			= 1 << 7,
	kMmuWriteViolationInt	= 1 << 10,
	kMmuWriteViolationAbort	= 1 << 11,
	kMmuInvalidPteEnable	= 1 << 16,
	kMmuInvalidPteInt		= 1 << 18,
	kMmuInvalidPteAbort		= 1 << 19,
	kMmuCapExceededInt		= 1 << 25,
	kMmuCapExceededAbort	= 1 << 26,

	kMmuIllegalAddressEnable = 1u << 31,

	kPteValid				= 1 << 28,
	kPteWritable			= 1 << 29,
};

// Core registers
enum {
	kCoreIdent0				= 0x0000,
	kCoreIdent1				= 0x0004,
	kCoreIdent2				= 0x0008,
	kCoreMiscConfig			= 0x0018,
	kCoreL2CacheControl		= 0x0020,
	kCoreSliceCacheControl	= 0x0024,
	kCoreL2TCacheControl	= 0x0030,
	kCoreL2TFlushStart		= 0x0034,
	kCoreL2TFlushEnd		= 0x0038,
	kCoreInterruptStatus	= 0x0050,
	kCoreInterruptClear		= 0x0058,
	kCoreInterruptMaskSet	= 0x0060,
	kCoreInterruptMaskClear	= 0x0064,

	kCle0Status				= 0x0100,
	kCle1Status				= 0x0104,
	kCle0EndAddress			= 0x0108,
	kCle1EndAddress			= 0x010c,
	kCle0CurrentAddress		= 0x0110,
	kCle1CurrentAddress		= 0x0114,
	kCle0ReturnAddress		= 0x0118,
	kCle1ReturnAddress		= 0x011c,
	kCle0QueuedTileState	= 0x015c,
	kCle0QueuedBase			= 0x0160,
	kCle1QueuedBase			= 0x0164,
	kCle0QueuedEnd			= 0x0168,
	kCle1QueuedEnd			= 0x016c,
	kCle0QueuedTileMemory	= 0x0170,
	kCle0QueuedTileMemorySize = 0x0174,

	kBinnerOverflowAddress	= 0x0308,
	kBinnerOverflowSize		= 0x030c,

	kGmpStatus				= 0x0800,
	kGmpConfig				= 0x0804,
	kGmpViolationAddress	= 0x0808,

	kErrorFdbgo				= 0x0f04,
	kErrorFdbgb				= 0x0f08,
	kErrorFdbgr				= 0x0f0c,
	kErrorFdbgs				= 0x0f10,
	kErrorStatus			= 0x0f20,
};

// Core bits
enum {
	kMiscOverrideTmuOutput	= 1 << 0,
	kMiscQrMaxCountShift	= 1,

	kL2CacheEnable			= 1 << 0,
	kL2CacheClear			= 1 << 2,

	kSliceInvalidateAll		= 0x0f0f0f0f,

	kL2TFlush				= 1 << 0,
	kL2TModeClean			= 2 << 1,
	kL2TTmuWriteFlush		= 1 << 8,

	kQueuedTileStateEnable	= 1 << 1,

	kGmpConfigBusy			= 1 << 3,
	kGmpReadWriteCount		= 0x7f7f0000,
	kGmpStopRequest			= 1 << 1,
};

// Core interrupts
enum {
	kRenderDone				= 1 << 0,
	kBinDone				= 1 << 1,
	kOutOfMemory			= 1 << 2,
	kGmpViolation			= 1 << 5,
	kComputeDone			= 1 << 7,
	kQpuInterrupts			= 0xfff << 16,
	// As Linux v3d_irq.c
	kCoreInterrupts			= kOutOfMemory | kBinDone | kRenderDone
		| kComputeDone | kGmpViolation,
};


// GPU virtual address space: one 4 GiB space for all clients, as Linux.
static const uint32 kGpuPageShift = 12;
static const uint32 kPageTableEntries = 1u << (32 - kGpuPageShift);
static const size_t kPageTableSize = kPageTableEntries * sizeof(uint32);

// Binner overflow memory: 32 chunks of 1 MiB, mapped once (Phoenix's pool).
static const uint32 kOverflowChunkSize = 1024 * 1024;
static const uint32 kOverflowChunks = 32;

// A job that makes no progress for this long is declared wedged.
static const bigtime_t kWatchdogTick = 125000;
static const bigtime_t kWatchdogTimeout = 500000;
static const bigtime_t kWatchdogHardLimit = 10000000;


struct v3d_device;


struct v3d_bo {
	int32				references;
		// one per handle, one per job using it
	area_id				area;
	void*				address;
		// the kernel's (write-combining) mapping; for an imported BO (area
		// < 0): the client's, wired, in imported_team
	team_id				imported_team;
	size_t				size;
	uint32				gpu_address;
	uint64				last_seqno;
		// the last job that uses this BO
	uint64				release_after;
	v3d_bo*				next;
		// quarantine list
};


struct v3d_syncobj {
	bool				has_fence;
	uint64				seqno;
		// signalled once job "seqno" completed (0: always)
};


struct v3d_file {
	v3d_device*			device;
	v3d_bo**			bos;
	uint32				bo_slots;
	v3d_syncobj**		syncobjs;
	uint32				syncobj_slots;
	bool				closing;
};


enum v3d_job_stage {
	V3D_JOB_QUEUED,
	V3D_JOB_BIN,
	V3D_JOB_RENDER,
	V3D_JOB_TFU,
};


struct v3d_job {
	v3d_job*			next;
	uint64				seqno;
	v3d_job_stage		stage;
	bool				is_tfu;
		// a texture formatting job (tfu below), else a control list job

	drm_v3d_submit_tfu	tfu;

	uint32				bcl_start;
	uint32				bcl_end;
	uint32				rcl_start;
	uint32				rcl_end;
	uint32				qma;
	uint32				qms;
	uint32				qts;
	uint32				flags;

	v3d_bo**			bos;
	uint32				bo_count;
	uint32				overflow_chunks;
		// bit mask of the overflow chunks the binner got

	bigtime_t			kicked;
	bigtime_t			last_progress;
	bigtime_t			next_check;
	uint32				current_address;
	uint32				return_address;
	uint32				qpu_acks;
};


struct v3d_device {
	device_node*		node;
	mutex				lock;
		// everything below that is not marked otherwise

	phys_addr_t			hub_physical;
	phys_addr_t			core_physical;
	phys_addr_t			pm_physical;
	phys_addr_t			asb_physical;
	uint64				interrupt;

	bool				powered;
	bool				started;
		// MMU, interrupt handler and worker are set up
	bool				broken;
		// a reset could not turn the clock back on: no MMIO any more
	area_id				hub_area;
	area_id				core_area;
	volatile uint32*	hub;
	volatile uint32*	core;

	char				report[512];
	int32				report_length;

	// MMU
	area_id				page_table_area;
	uint32*				page_table;
	phys_addr_t			page_table_physical;
	area_id				scratch_area;
	phys_addr_t			scratch_physical;
	uint64				physical_limit;
	uint32*				va_bitmap;
	uint32				va_hint;

	// binner overflow pool
	area_id				overflow_area;
	uint32				overflow_gpu_address;
	uint32				overflow_free;
		// bit mask of free chunks (the staged one is not free)

	// shared with the interrupt handler, under irq_lock
	spinlock			irq_lock;
	int32				pending_core;
	int32				pending_hub;
	int32				staged_chunk;
	int32				consumed_chunk;
		// chunk index + 1, 0 = none
	int32				oom_missed;
	int32				interrupts_since_worker;
	int32				spurious_interrupts;
	int32				storm;
	uint32				mmu_control_seen;

	// jobs
	v3d_job*			queue_head;
	v3d_job*			queue_tail;
	v3d_job*			active;
	uint64				submitted_seqno;
	uint64				completed_seqno;
	ConditionVariable	job_done;
	v3d_bo*				quarantine;
	uint32				resets;

	sem_id				worker_sem;
	thread_id			worker;
	bool				quit;
};


// v3d.cpp
status_t v3d_power_on(v3d_device* device);
status_t v3d_power_reset(v3d_device* device);

// mmu.cpp
status_t v3d_page_table_alloc(v3d_device* device);
status_t v3d_mmu_init(v3d_device* device);
void v3d_mmu_uninit(v3d_device* device);
void v3d_apply_core_registers(v3d_device* device);
void v3d_mmu_flush(v3d_device* device);
bool v3d_wait_for_clear(volatile uint32* registers, uint32 offset, uint32 bit,
	const char* what);
status_t v3d_va_alloc(v3d_device* device, size_t size, uint32* _address);
void v3d_va_free(v3d_device* device, uint32 address, size_t size);
status_t v3d_map_memory(v3d_device* device, team_id team, void* address,
	size_t size, uint32 gpuAddress);
status_t v3d_map_area(v3d_device* device, void* address, size_t size,
	uint32 gpuAddress);
void v3d_unmap_range(v3d_device* device, uint32 gpuAddress, size_t size);
area_id v3d_create_gpu_memory(const char* name, size_t size, bool contiguous,
	void** _address, phys_addr_t* _physical);

// bo.cpp
status_t v3d_bo_ioctl(v3d_file* file, uint32 nr, void* data);
v3d_bo* v3d_bo_lookup(v3d_file* file, uint32 handle);
void v3d_bo_put(v3d_device* device, v3d_bo* bo);
void v3d_bo_release_quarantine(v3d_device* device);
void v3d_file_close_bos(v3d_file* file);

// jobs.cpp
status_t v3d_jobs_start(v3d_device* device);
void v3d_jobs_stop(v3d_device* device);
status_t v3d_submit_cl(v3d_file* file, drm_v3d_submit_cl* args);
status_t v3d_submit_tfu(v3d_file* file, drm_v3d_submit_tfu* args);
status_t v3d_wait_seqno(v3d_device* device, v3d_file* file, uint64 seqno,
	bigtime_t deadline);
status_t v3d_syncobj_ioctl(v3d_file* file, uint32 nr, void* data);
void v3d_file_close_syncobjs(v3d_file* file);


static inline uint32
hub_read(v3d_device* device, uint32 offset)
{
	return device->hub[offset / 4];
}


static inline void
hub_write(v3d_device* device, uint32 offset, uint32 value)
{
	device->hub[offset / 4] = value;
}


static inline uint32
core_read(v3d_device* device, uint32 offset)
{
	return device->core[offset / 4];
}


static inline void
core_write(v3d_device* device, uint32 offset, uint32 value)
{
	device->core[offset / 4] = value;
}


static inline void
v3d_barrier()
{
	// Normal-NC stores (BOs, page table) reach memory before the next
	// device register write.
	__asm__ __volatile__("dsb sy" : : : "memory");
}


/*!	Stores \a object in the first free slot of a per-file handle table and
	returns its handle, the slot index + 1 (0 is never a valid handle).
*/
template<typename Type>
static inline status_t
v3d_handle_alloc(Type**& slots, uint32& slotCount, Type* object,
	uint32* _handle)
{
	for (uint32 i = 0; i < slotCount; i++) {
		if (slots[i] == NULL) {
			slots[i] = object;
			*_handle = i + 1;
			return B_OK;
		}
	}

	uint32 newCount = slotCount == 0 ? 16 : slotCount * 2;
	Type** newSlots = (Type**)calloc(newCount, sizeof(Type*));
	if (newSlots == NULL)
		return B_NO_MEMORY;
	if (slotCount > 0)
		memcpy(newSlots, slots, slotCount * sizeof(Type*));
	free(slots);

	newSlots[slotCount] = object;
	*_handle = slotCount + 1;
	slots = newSlots;
	slotCount = newCount;
	return B_OK;
}


template<typename Type>
static inline Type*
v3d_handle_lookup(Type** slots, uint32 slotCount, uint32 handle)
{
	if (handle == 0 || handle > slotCount)
		return NULL;
	return slots[handle - 1];
}


#endif	// V3D_PRIVATE_H
