/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef RPIVID_PRIVATE_H
#define RPIVID_PRIVATE_H


#include <stdlib.h>
#include <string.h>

#include <OS.h>
#include <KernelExport.h>
#include <device_manager.h>

#include <condition_variable.h>
#include <kernel.h>
#include <lock.h>

#include <rpivid.h>


#define TRACE_ALWAYS(x...)	dprintf("rpivid: " x)
#define ERROR(x...)			dprintf("rpivid: " x)


// Block registers, byte offsets. Phase 1 registers are written through the
// command buffer, the others directly.
enum {
	kSps0				= 0x00,
	kStatus				= 0x38,
	kVersion			= 0x3c,
	kBitstreamBase		= 0x40,	// BFBASE
	kBitstreamLength	= 0x44,	// BFNUM
	kPuWriteBase		= 0x50,
	kPuWriteStride		= 0x54,
	kCoeffWriteBase		= 0x58,
	kCoeffWriteStride	= 0x5c,
	kCommandBase		= 0x6c,	// CFBASE: starts phase 1
	kCommandCount		= 0x70,	// CFNUM
	kCommandStatus		= 0x74,	// CFSTATUS

	kProbabilities		= 0x1000,
	kProbabilitiesEnd	= 0x1000 + 156,
	kScalingFactors		= 0x2000,
	kScalingFactorsEnd	= 0x2000 + 4064,
	kSliceMessages		= 0x4000,
	kSliceMessagesEnd	= 0x4000 + 4 * 259,
		// 2 + 16 references * 2 lists * 8 words + 1 (as Linux SLICE_MSGS_MAX)

	kPuReadBase			= 0x8000,
	kPuReadStride		= 0x8004,
	kCoeffReadBase		= 0x8008,
	kCoeffReadStride	= 0x800c,
	kNumRows			= 0x8010,	// starts phase 2
	kConfig2			= 0x8014,
	kOutLumaBase		= 0x8018,
	kOutLumaStride		= 0x801c,
	kOutChromaBase		= 0x8020,
	kOutChromaStride	= 0x8024,
	kFrameSize			= 0x802c,
	kMvBase				= 0x8030,
	kMvStride			= 0x8034,
	kColBase			= 0x8038,
	kColStride			= 0x803c,
	kCurrentPoc			= 0x8040,
	kReferences			= 0x9000,	// 16 slots: luma base, stride, chroma base, stride
	kReferenceSize		= 16,

	kBlockSize			= 0x10000,
};

static const uint32 kExpectedVersion = 0x202;

// STATUS after phase 1: a scratch buffer was too small
static const uint32 kStatusCoeffExhausted = 1 << 3;
static const uint32 kStatusPuExhausted = 1 << 4;

// CONFIG2 fields the driver checks against the buffers
static const uint32 kConfig2BitDepths = 0x3ff;
static const uint32 kConfig2Log2CtbShift = 10;
static const uint32 kConfig2Log2CtbMask = 7 << 10;
static const uint32 kConfig2WriteMv = 1 << 15;
static const uint32 kConfig2ReadMv = 1 << 19;

// The interrupt controller ("intc"), its control register at 0
enum {
	kActive1Interrupt	= 1 << 0,	// phase 1 done
	kActive1Enable		= 1 << 2,
	kActive2Interrupt	= 1 << 4,	// phase 2 done
	kActive2Enable		= 1 << 6,
	kInterruptsZeroMask	= (0xff << 12) | (1 << 11),
		// written as zero (writing 1 does something else)
	kActiveInterrupts	= kActive1Interrupt | kActive2Interrupt,
};


// Contiguous memory the block reaches
struct dma_memory {
	area_id			area;
	void*			address;
	phys_addr_t		physical;
	size_t			size;
};


enum rpivid_buffer_type {
	kBitstreamBuffer,
	kPictureBuffer,
};

struct rpivid_buffer {
	rpivid_buffer_type type;
	dma_memory		memory;
	area_id			client_area;
		// the mapping in the client's team (deleted with the buffer if the
		// client deletes it by ioctl)
	team_id			client_team;
	int32			references;
		// the file's handle and every job naming it: the memory goes with
		// the last (under the device lock)
	bool			pinned;
		// handed to a block that stopped responding: never freed

	// pictures
	uint32			width;
	uint32			height;
	uint32			bit_depth;
	uint32			flags;
	uint32			columns;
	uint32			luma_stride;
	uint32			chroma_stride;
	size_t			chroma_offset;
	size_t			planes_size;
		// luma and chroma: what the CPU reads
	size_t			mv_offset;
	uint32			mv_stride;
};


struct rpivid_file;

enum rpivid_job_state {
	kJobQueued,
	kJobPhase1,
	kJobPhase1Done,		// waits for phase 2
	kJobPhase2,
	kJobDone,
};

// One submitted picture. It holds a reference to each buffer it names until
// it is waited for (or its file is closed).
struct rpivid_job {
	rpivid_job*		next;		// in the device's queue (submission order)
	rpivid_job*		file_next;	// in the file's jobs
	rpivid_file*	file;
	uint64			id;
	rpivid_job_state state;

	uint64*			entries;	// until phase 1 starts
	uint32			command_count;
	rpivid_buffer*	bitstream;
	rpivid_buffer*	output;
	rpivid_buffer*	collocated;
	rpivid_buffer*	references[RPIVID_MAX_REFERENCES];
	uint32			config2;
	uint32			current_poc;
	uint32			ctb_rows;
	bool			write_mv;
	bool			read_mv;

	int32			scratch;	// the scratch set from phase 1 to phase 2 end
	uint32			pu_stride;
	uint32			coeff_stride;

	bigtime_t		submitted;
	bigtime_t		phase1_start;
	bigtime_t		phase2_start;

	// results
	status_t		result;
	uint32			status;
	uint32			command_status;
	uint32			phase1_runs;
	bigtime_t		queue_time;
	bigtime_t		phase1_time;
	bigtime_t		phase2_time;
};


// Phase 1 of a picture writes a scratch set that phase 2 of the picture
// reads: with two, phase 1 of a picture runs during phase 2 of the one
// before it.
static const uint32 kScratchSets = 2;

struct rpivid_scratch {
	dma_memory		pu;
	dma_memory		coeff;
	size_t			pu_size;
		// the logical sizes the strides derive from
	size_t			coeff_size;
	rpivid_job*		owner;
};


struct rpivid_device {
	device_node*	node;
	phys_addr_t		block_physical;
	phys_addr_t		intc_physical;
	uint64			interrupt;
	uint32			clock_id;
	int64			bus_offset;
		// bus address = physical address + bus_offset
	phys_addr_t		dma_end;
		// the block reaches physical memory below this

	mutex			lock;
		// open count, power, buffers, jobs, the block's registers
	int32			open_count;
	bool			powered;
	bool			broken;
	uint32			version;
	uint32			clock_rate;
	area_id			block_area;
	area_id			intc_area;
	volatile uint32* block;
	volatile uint32* intc;

	spinlock		irq_lock;
	uint32			pending;
		// completions the interrupt handler saw (kActive*Interrupt)
	sem_id			irq_sem;
		// released by the interrupt handler and by RPIVID_SUBMIT: wakes the
		// decode thread
	uint32			interrupts;

	// the pipeline, run by the decode thread
	thread_id		thread;
	bool			quit;
	rpivid_job*		queue;		// jobs not done, in submission order
	rpivid_job*		queue_tail;
	rpivid_job*		phase1;		// the job in phase 1
	rpivid_job*		phase2;		// the job in phase 2
	bigtime_t		phase1_deadline;
	bigtime_t		phase2_deadline;
	ConditionVariable job_done;

	// device memory: the block's alone
	dma_memory		commands;
	rpivid_scratch	scratch[kScratchSets];
	dma_memory		zero_mv;

	uint64			decoded;
	uint64			failed;
	uint64			overlaps;
};


struct rpivid_file {
	rpivid_device*	device;
	rpivid_buffer**	buffers;
	uint32			buffer_slots;
	rpivid_job*		jobs;		// submitted, not yet waited for
	uint32			job_count;
	uint32			active_jobs;	// of these, not done
	uint64			next_job;
};


static inline uint32
block_read(rpivid_device* device, uint32 offset)
{
	return device->block[offset / 4];
}


static inline void
block_write(rpivid_device* device, uint32 offset, uint32 value)
{
	device->block[offset / 4] = value;
}


// rpivid.cpp
status_t	rpivid_dma_alloc(rpivid_device* device, const char* name,
				size_t size, uint32 memoryType, dma_memory& memory);
void		rpivid_dma_free(dma_memory& memory);
uint32		rpivid_bus_address(rpivid_device* device, phys_addr_t physical);
rpivid_buffer* rpivid_buffer_lookup(rpivid_file* file, uint32 handle,
				rpivid_buffer_type type);
void		rpivid_buffer_release(rpivid_buffer* buffer);
void		rpivid_power_off(rpivid_device* device);

// decode.cpp
int32		rpivid_interrupt(void* data);
status_t	rpivid_decode_thread(void* data);
status_t	rpivid_submit_ioctl(rpivid_file* file, rpivid_decode* args);
status_t	rpivid_wait_ioctl(rpivid_file* file, rpivid_wait* args);
status_t	rpivid_decode_ioctl(rpivid_file* file, rpivid_decode* args);
void		rpivid_file_jobs_free(rpivid_file* file);
void		rpivid_decode_free_memory(rpivid_device* device);


#endif	// RPIVID_PRIVATE_H
