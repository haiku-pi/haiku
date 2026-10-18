/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Control list jobs, the interrupt handler, the watchdog, and syncobjs.

	One job is on the GPU at a time (its binner list, then its render list),
	but the caller does not wait for it: SUBMIT_CL queues the job and returns.
	A worker thread owns the hardware: it kicks jobs, takes their completion
	from the interrupt handler, runs the watchdog and resets a wedged GPU.
	Waits (WAIT_BO, SYNCOBJ_WAIT) sleep until a job's sequence number has
	completed.

	Jobs run in submission order, one at a time, so every fence a new job may
	depend on - its in-syncs, the last job of each of its BOs - belongs to an
	earlier job and has signalled by the time the new job starts: in-syncs
	need no tracking yet. Running the queues concurrently will change that.

	The cache and TLB maintenance around each job is the sequence proven on
	this board by the Phoenix-RTOS V3D server; every step there is load-
	bearing, and each is marked below.
*/


#include "v3d_private.h"

#include <util/AutoLock.h>


static const uint32 kMaxJobBOs = 16384;
static const int32 kSpuriousLimit = 1000;
static const int32 kBurstLimit = 200000;


static void reset_gpu(v3d_device* device, const char* reason);


//	#pragma mark - interrupts


static void
mask_interrupts(v3d_device* device)
{
	core_write(device, kCoreInterruptMaskSet, ~0u);
	hub_write(device, kHubInterruptMaskSet, ~0u);
}


static void
clear_interrupts(v3d_device* device)
{
	core_write(device, kCoreInterruptClear, ~0u);
	hub_write(device, kHubInterruptClear, ~0u);
}


static void
unmask_interrupts(v3d_device* device)
{
	core_write(device, kCoreInterruptMaskClear, kCoreInterrupts);
	hub_write(device, kHubInterruptMaskClear, kHubInterrupts);
}


static inline uint32
overflow_chunk_address(v3d_device* device, int32 index)
{
	return device->overflow_gpu_address + index * kOverflowChunkSize;
}


/*!	Reads and acknowledges both status words, hands the binner its staged
	overflow chunk and records what happened for the worker. Runs with
	irq_lock held and interrupts disabled: from the handler, and from the
	worker when it has to poll.
*/
static void
service_status(v3d_device* device, uint32& _core, uint32& _hub)
{
	// BCM2711 has one interrupt line for hub and core: always read both.
	const uint32 core = core_read(device, kCoreInterruptStatus);
	const uint32 hub = hub_read(device, kHubInterruptStatus);

	if (core != 0) {
		// The raw status, QPU bits (27:16) included, even though they are
		// masked: an unacknowledged QPU interrupt stalls fragment shading
		// (as Linux; on Phoenix this made a render wedge go away).
		core_write(device, kCoreInterruptClear, core);
	}
	if (hub != 0) {
		if ((hub & kHubMmuFaults) != 0) {
			// The fault bits in MMU_CTL are write-one-to-clear (as Linux).
			const uint32 control = hub_read(device, kMmuControl);
			device->mmu_control_seen = control;
			hub_write(device, kMmuControl, control);
		}
		hub_write(device, kHubInterruptClear, hub);
	}

	if ((core & kOutOfMemory) != 0) {
		// The binner stalls until it gets memory: hand it the staged chunk
		// right away instead of waking the worker first.
		const int32 chunk = device->staged_chunk;
		if (chunk != 0) {
			device->staged_chunk = 0;
			core_write(device, kBinnerOverflowAddress,
				overflow_chunk_address(device, chunk - 1));
			core_write(device, kBinnerOverflowSize, kOverflowChunkSize);
			device->consumed_chunk = chunk;
		} else
			device->oom_missed++;
	}

	device->pending_core |= core;
	device->pending_hub |= hub;
	_core = core;
	_hub = hub;
}


/*!	The line is level triggered: whatever asserts it must be acknowledged
	here, or it fires again at once. Two guards turn a stuck source into a
	logged fallback to polling instead of a CPU lost to interrupts.
*/
static int32
v3d_interrupt(void* data)
{
	v3d_device* device = (v3d_device*)data;
	if (device->broken)
		return B_UNHANDLED_INTERRUPT;

	SpinLocker locker(device->irq_lock);

	uint32 core, hub;
	service_status(device, core, hub);

	if ((core | hub) == 0) {
		// Fires with nothing to acknowledge: something we do not clear keeps
		// the line asserted.
		if (++device->spurious_interrupts > kSpuriousLimit
			&& device->storm == 0) {
			mask_interrupts(device);
			device->storm = 1;
			locker.Unlock();
			release_sem_etc(device->worker_sem, 1, B_DO_NOT_RESCHEDULE);
		}
		return B_UNHANDLED_INTERRUPT;
	}
	device->spurious_interrupts = 0;

	if (++device->interrupts_since_worker > kBurstLimit
		&& device->storm == 0) {
		// A source latches again as fast as it is cleared, and the worker
		// never gets to run.
		mask_interrupts(device);
		device->storm = 2;
	}

	locker.Unlock();
	release_sem_etc(device->worker_sem, 1, B_DO_NOT_RESCHEDULE);
	return B_INVOKE_SCHEDULER;
}


//	#pragma mark - binner overflow memory


/*!	Arms one free chunk for the interrupt handler. */
static void
stage_overflow_chunk(v3d_device* device)
{
	InterruptsSpinLocker locker(device->irq_lock);
	if (device->staged_chunk != 0 || device->overflow_free == 0)
		return;

	const int32 index = __builtin_ctz(device->overflow_free);
	device->overflow_free &= ~(1u << index);
	device->staged_chunk = index + 1;
}


/*!	Gives the stalled binner a chunk directly: it ran out of memory with
	nothing staged. Returns false if the pool is empty.
*/
static bool
grant_overflow_chunk(v3d_device* device, v3d_job* job)
{
	InterruptsSpinLocker locker(device->irq_lock);
	if (device->overflow_free == 0)
		return false;

	const int32 index = __builtin_ctz(device->overflow_free);
	device->overflow_free &= ~(1u << index);
	core_write(device, kBinnerOverflowAddress,
		overflow_chunk_address(device, index));
	core_write(device, kBinnerOverflowSize, kOverflowChunkSize);
	job->overflow_chunks |= 1u << index;
	return true;
}


static void
return_overflow_chunks(v3d_device* device, uint32 chunks)
{
	InterruptsSpinLocker locker(device->irq_lock);
	device->overflow_free |= chunks;
}


//	#pragma mark - cache maintenance


static void
wait_l2t(v3d_device* device)
{
	v3d_wait_for_clear(device->core, kCoreL2TCacheControl, kL2TFlush,
		"L2T flush");
}


/*!	A flush of the texture L2 cache. GFXH-1897: never start one while
	another is pending.
*/
static void
flush_l2t(v3d_device* device, bool waitForIt)
{
	wait_l2t(device);
	core_write(device, kCoreL2TCacheControl, kL2TFlush);
	if (waitForIt)
		wait_l2t(device);
}


/*!	Writes back what the TMU wrote, before the job is reported done (as
	Linux v3d_clean_caches(), for DRM_V3D_SUBMIT_CL_FLUSH_CACHE).
*/
static void
clean_caches(v3d_device* device)
{
	wait_l2t(device);
	core_write(device, kCoreL2TCacheControl, kL2TTmuWriteFlush);
	v3d_wait_for_clear(device->core, kCoreL2TCacheControl, kL2TTmuWriteFlush,
		"TMU write combiner flush");
	core_write(device, kCoreL2TCacheControl, kL2TFlush | kL2TModeClean);
	wait_l2t(device);
}


/*!	The per-job prologue (Phoenix's proven default; dropping steps there
	regressed).
*/
static void
prepare_caches(v3d_device* device)
{
	// CPU stores to control lists, uniforms and page table reach memory.
	v3d_barrier();
	// Invalidate the slices early, so that the waits below are its settle
	// time; without it shaders read stale uniforms.
	core_write(device, kCoreSliceCacheControl, kSliceInvalidateAll);
	v3d_mmu_flush(device);
	// A waited L2T flush, twice: removing the second one brought render
	// list time-outs back on Phoenix.
	flush_l2t(device, true);
	flush_l2t(device, true);
}


/*!	Drops completion bits of the list about to be started that are still
	latched from earlier jobs: one job is in flight, they cannot belong to
	anything else. OUTOMEM stays with the handler.
*/
static void
drain_stale(v3d_device* device, uint32 bits)
{
	InterruptsSpinLocker locker(device->irq_lock);
	const uint32 status = core_read(device, kCoreInterruptStatus) & bits;
	if (status != 0)
		core_write(device, kCoreInterruptClear, status);
	device->pending_core &= ~bits;
}


//	#pragma mark - kick and completion


static void
start_watchdog(v3d_job* job, uint32 currentAddress, uint32 returnAddress)
{
	const bigtime_t now = system_time();
	job->kicked = job->last_progress = now;
	job->next_check = now + kWatchdogTick;
	job->current_address = currentAddress;
	job->return_address = returnAddress;
}


static void
kick_bin(v3d_device* device, v3d_job* job)
{
	prepare_caches(device);
	drain_stale(device, kBinDone);

	{
		// No overflow memory from a previous job (as Linux v3d_sched.c).
		InterruptsSpinLocker locker(device->irq_lock);
		core_write(device, kBinnerOverflowSize, 0);
	}
	stage_overflow_chunk(device);

	if (job->qma != 0) {
		core_write(device, kCle0QueuedTileMemory, job->qma);
		core_write(device, kCle0QueuedTileMemorySize, job->qms);
	}
	if (job->qts != 0) {
		core_write(device, kCle0QueuedTileState,
			kQueuedTileStateEnable | job->qts);
	}

	job->stage = V3D_JOB_BIN;
	start_watchdog(job, job->bcl_start, 0);
	core_write(device, kCle0QueuedBase, job->bcl_start);
	// writing the end address starts the list
	core_write(device, kCle0QueuedEnd, job->bcl_end);
}


static void
kick_render(v3d_device* device, v3d_job* job)
{
	if (job->stage == V3D_JOB_QUEUED)
		prepare_caches(device);

	// The hand-off from the binner: its tile lists reach memory before the
	// render list reads them, then the slices start over.
	flush_l2t(device, true);
	core_write(device, kCoreSliceCacheControl, kSliceInvalidateAll);
	drain_stale(device, kRenderDone);

	job->stage = V3D_JOB_RENDER;
	start_watchdog(job, job->rcl_start, 0);
	core_write(device, kCle1QueuedBase, job->rcl_start);
	core_write(device, kCle1QueuedEnd, job->rcl_end);
}


static void
schedule(v3d_device* device)
{
	if (device->active != NULL || device->queue_head == NULL || device->broken)
		return;

	v3d_job* job = device->queue_head;
	device->queue_head = job->next;
	if (device->queue_head == NULL)
		device->queue_tail = NULL;
	job->next = NULL;
	device->active = job;

	if (job->bcl_start != job->bcl_end)
		kick_bin(device, job);
	else
		kick_render(device, job);
}


static void
free_job(v3d_device* device, v3d_job* job)
{
	for (uint32 i = 0; i < job->bo_count; i++)
		v3d_bo_put(device, job->bos[i]);
	free(job->bos);
	free(job);
}


/*!	The job is done, or was dropped by a reset (its fence signals anyway:
	resubmitting a job that wedged the GPU wedges it again).
*/
static void
finish_job(v3d_device* device, v3d_job* job)
{
	device->active = NULL;
	device->completed_seqno = job->seqno;

	// The render list read the tile lists in these chunks; now they are free.
	return_overflow_chunks(device, job->overflow_chunks);

	free_job(device, job);
	v3d_bo_release_quarantine(device);
	device->job_done.NotifyAll();
}


static void
render_done(v3d_device* device, v3d_job* job)
{
	// Write back the texture L2 cache, not waited for: the next prologue
	// waits for it before it starts its own flush.
	wait_l2t(device);
	core_write(device, kCoreL2TCacheControl, kL2TFlush | kL2TModeClean);

	if ((job->flags & DRM_V3D_SUBMIT_CL_FLUSH_CACHE) != 0)
		clean_caches(device);

	finish_job(device, job);
}


//	#pragma mark - watchdog and reset


static void
progress_registers(v3d_device* device, v3d_job* job, uint32& current,
	uint32& _return)
{
	if (job->stage == V3D_JOB_BIN) {
		current = core_read(device, kCle0CurrentAddress);
		_return = core_read(device, kCle0ReturnAddress);
	} else {
		current = core_read(device, kCle1CurrentAddress);
		_return = core_read(device, kCle1ReturnAddress);
	}
}


static void
dump_job(v3d_device* device, v3d_job* job)
{
	ERROR("%s list of job %" B_PRIu64 " wedged: int %#" B_PRIx32
		", ct0 cs %#" B_PRIx32 " ca %#" B_PRIx32 " [%#" B_PRIx32 "..%#"
		B_PRIx32 "] ra %#" B_PRIx32 ", ct1 cs %#" B_PRIx32 " ca %#" B_PRIx32
		" [%#" B_PRIx32 "..%#" B_PRIx32 "], bpoa %#" B_PRIx32 " bpos %#"
		B_PRIx32 ", overflow free %#" B_PRIx32 "\n",
		job->stage == V3D_JOB_BIN ? "binner" : "render", job->seqno,
		core_read(device, kCoreInterruptStatus),
		core_read(device, kCle0Status), core_read(device, kCle0CurrentAddress),
		job->bcl_start, job->bcl_end,
		core_read(device, kCle0ReturnAddress),
		core_read(device, kCle1Status), core_read(device, kCle1CurrentAddress),
		job->rcl_start, job->rcl_end,
		core_read(device, kBinnerOverflowAddress),
		core_read(device, kBinnerOverflowSize), device->overflow_free);
	ERROR("  gmp %#" B_PRIx32 " violation %#" B_PRIx32 ", mmu illegal %#"
		B_PRIx32 ", fdbg %#" B_PRIx32 "/%#" B_PRIx32 "/%#" B_PRIx32 "/%#"
		B_PRIx32 ", error %#" B_PRIx32 ", qpu acks %" B_PRIu32 "\n",
		core_read(device, kGmpStatus), core_read(device, kGmpViolationAddress),
		hub_read(device, kMmuIllegalAddress), core_read(device, kErrorFdbgo),
		core_read(device, kErrorFdbgb), core_read(device, kErrorFdbgr),
		core_read(device, kErrorFdbgs), core_read(device, kErrorStatus),
		job->qpu_acks);
}


/*!	A true reset (Phoenix's proven recovery): stop the GMP so that no AXI
	transaction is cut in half, hold the GPU in reset and power it on again,
	then restore its invariant state. The page table survives in memory.
	The job on the GPU is dropped; queued jobs run afterwards.
*/
static void
reset_gpu(v3d_device* device, const char* reason)
{
	v3d_job* job = device->active;
	if (job != NULL)
		dump_job(device, job);

	{
		InterruptsSpinLocker locker(device->irq_lock);
		mask_interrupts(device);
	}

	// As Linux v3d_idle_axi()
	core_write(device, kGmpConfig, kGmpStopRequest);
	bigtime_t timeout = system_time() + 100000;
	while ((core_read(device, kGmpStatus)
			& (kGmpReadWriteCount | kGmpConfigBusy)) != 0
		&& system_time() < timeout) {
	}

	status_t status = v3d_power_reset(device);
	if (status != B_OK) {
		// The clock may be off: another register access could hang the CPU.
		device->broken = true;
		ERROR("GPU reset after %s failed: %s; the GPU is out of service\n",
			reason, strerror(status));
	} else {
		v3d_apply_core_registers(device);

		InterruptsSpinLocker locker(device->irq_lock);
		mask_interrupts(device);
		clear_interrupts(device);
		if (device->staged_chunk != 0)
			device->overflow_free |= 1u << (device->staged_chunk - 1);
		if (device->consumed_chunk != 0)
			device->overflow_free |= 1u << (device->consumed_chunk - 1);
		device->staged_chunk = device->consumed_chunk = 0;
		device->oom_missed = 0;
		device->pending_core = device->pending_hub = 0;
		locker.Unlock();

		v3d_mmu_flush(device);
	}

	device->resets++;
	if (job != NULL)
		finish_job(device, job);

	if (device->broken) {
		// Nothing will run any more: let every waiter go.
		while (device->queue_head != NULL) {
			v3d_job* queued = device->queue_head;
			device->queue_head = queued->next;
			device->active = queued;
			finish_job(device, queued);
		}
		device->queue_tail = NULL;
		return;
	}

	InterruptsSpinLocker locker(device->irq_lock);
	if (device->storm == 0)
		unmask_interrupts(device);
	locker.Unlock();

	ERROR("GPU reset after %s (job %" B_PRIu64 " dropped), %" B_PRIu32
		" resets so far\n", reason, job != NULL ? job->seqno : 0,
		device->resets);
}


static void
check_watchdog(v3d_device* device)
{
	v3d_job* job = device->active;
	const bigtime_t now = system_time();
	if (job == NULL || now < job->next_check)
		return;
	job->next_check = now + kWatchdogTick;

	// A list that still moves is extended, as Linux v3d_cl_job_timedout().
	uint32 current, returnAddress;
	progress_registers(device, job, current, returnAddress);
	if (current != job->current_address
		|| returnAddress != job->return_address) {
		job->current_address = current;
		job->return_address = returnAddress;
		job->last_progress = now;
	}

	if (job->stage == V3D_JOB_RENDER) {
		// The QPU sources are masked, so the handler only acknowledges them
		// when something else fires: do it here during a long render.
		InterruptsSpinLocker locker(device->irq_lock);
		const uint32 qpu = core_read(device, kCoreInterruptStatus)
			& kQpuInterrupts;
		if (qpu != 0) {
			core_write(device, kCoreInterruptClear, qpu);
			job->qpu_acks++;
		}
	}

	if (now - job->last_progress >= kWatchdogTimeout)
		reset_gpu(device, "a stalled control list");
	else if (now - job->kicked >= kWatchdogHardLimit)
		reset_gpu(device, "a runaway control list");
}


//	#pragma mark - worker


static void
process_events(v3d_device* device)
{
	uint32 core, hub;
	int32 consumed, missed, storm;
	uint32 mmuControl;
	{
		InterruptsSpinLocker locker(device->irq_lock);
		device->interrupts_since_worker = 0;
		if (device->storm != 0)
			service_status(device, core, hub);

		core = device->pending_core;
		hub = device->pending_hub;
		device->pending_core = device->pending_hub = 0;
		consumed = device->consumed_chunk;
		device->consumed_chunk = 0;
		missed = device->oom_missed;
		device->oom_missed = 0;
		storm = device->storm;
		mmuControl = device->mmu_control_seen;
	}

	static int32 sReportedStorm = 0;
	if (storm != sReportedStorm) {
		ERROR("interrupt storm (%s): interrupts masked, polling\n",
			storm == 1 ? "line stuck" : "source re-latches");
		sReportedStorm = storm;
	}

	v3d_job* job = device->active;

	// Overflow memory first: a chunk the binner got belongs to this job even
	// if its FLDONE arrived in the same batch.
	if (consumed != 0) {
		if (job != NULL && job->stage == V3D_JOB_BIN)
			job->overflow_chunks |= 1u << (consumed - 1);
		else
			return_overflow_chunks(device, 1u << (consumed - 1));
	}
	if (job != NULL && job->stage == V3D_JOB_BIN) {
		for (; missed > 0; missed--) {
			if (!grant_overflow_chunk(device, job)) {
				ERROR("binner overflow pool (%" B_PRIu32 " MiB) exhausted\n",
					kOverflowChunks * kOverflowChunkSize / 1024 / 1024);
				reset_gpu(device, "running out of binner memory");
				return;
			}
		}
		stage_overflow_chunk(device);
	}

	if (job != NULL && job->stage == V3D_JOB_BIN && (core & kBinDone) != 0)
		kick_render(device, job);
	else if (job != NULL && job->stage == V3D_JOB_RENDER
		&& (core & kRenderDone) != 0)
		render_done(device, job);

	if ((core & kGmpViolation) != 0) {
		ERROR("GMP violation at %#" B_PRIx32 "\n",
			core_read(device, kGmpViolationAddress));
	}
	if ((hub & kHubMmuFaults) != 0) {
		static int32 sReported = 0;
		if (sReported++ < 16) {
			ERROR("MMU fault: hub %#" B_PRIx32 ", id %#" B_PRIx32
				", address %#" B_PRIx32 "(<<8), control %#" B_PRIx32 "%s%s%s\n",
				hub, hub_read(device, kMmuViolationId),
				hub_read(device, kMmuViolationAddress), mmuControl,
				(hub & kHubMmuWriteViolation) != 0 ? " write-violation" : "",
				(hub & kHubMmuInvalidPte) != 0 ? " invalid-pte" : "",
				(hub & kHubMmuCapExceeded) != 0 ? " cap-exceeded" : "");
		}
	}

	check_watchdog(device);
	schedule(device);
}


static status_t
worker_thread(void* data)
{
	v3d_device* device = (v3d_device*)data;

	while (true) {
		bigtime_t timeout = B_INFINITE_TIMEOUT;
		{
			MutexLocker locker(device->lock);
			if (device->quit)
				break;
			if (device->active != NULL) {
				InterruptsSpinLocker irqLocker(device->irq_lock);
				timeout = device->storm != 0 ? 1000 : kWatchdogTick;
			}
		}

		acquire_sem_etc(device->worker_sem, 1,
			timeout == B_INFINITE_TIMEOUT ? 0 : B_RELATIVE_TIMEOUT, timeout);

		MutexLocker locker(device->lock);
		if (device->quit)
			break;
		if (!device->broken)
			process_events(device);
	}
	return B_OK;
}


/*!	Installs the interrupt handler in the storm-proof order: every source
	masked and cleared before the handler exists, then the ones it serves
	unmasked (as Linux v3d_irq_enable()). The GPU is powered, the MMU set up.
*/
status_t
v3d_jobs_start(v3d_device* device)
{
	device->worker_sem = create_sem(0, "v3d worker");
	if (device->worker_sem < 0)
		return device->worker_sem;

	{
		InterruptsSpinLocker locker(device->irq_lock);
		mask_interrupts(device);
		clear_interrupts(device);
	}

	status_t status = install_io_interrupt_handler(device->interrupt,
		&v3d_interrupt, device, 0);
	if (status != B_OK) {
		delete_sem(device->worker_sem);
		return status;
	}

	device->worker = spawn_kernel_thread(&worker_thread, "v3d worker",
		B_URGENT_DISPLAY_PRIORITY, device);
	if (device->worker < 0) {
		remove_io_interrupt_handler(device->interrupt, &v3d_interrupt, device);
		delete_sem(device->worker_sem);
		return device->worker;
	}
	resume_thread(device->worker);

	InterruptsSpinLocker locker(device->irq_lock);
	unmask_interrupts(device);
	return B_OK;
}


/*!	Called without the device lock. */
void
v3d_jobs_stop(v3d_device* device)
{
	{
		MutexLocker locker(device->lock);
		device->quit = true;
	}
	release_sem(device->worker_sem);
	status_t result;
	wait_for_thread(device->worker, &result);

	if (!device->broken) {
		InterruptsSpinLocker locker(device->irq_lock);
		mask_interrupts(device);
		clear_interrupts(device);
	}
	remove_io_interrupt_handler(device->interrupt, &v3d_interrupt, device);
	delete_sem(device->worker_sem);

	MutexLocker locker(device->lock);
	if (device->active != NULL) {
		free_job(device, device->active);
		device->active = NULL;
	}
	while (device->queue_head != NULL) {
		v3d_job* job = device->queue_head;
		device->queue_head = job->next;
		free_job(device, job);
	}
	device->queue_tail = NULL;
}


//	#pragma mark - submission and waits


status_t
v3d_submit_cl(v3d_file* file, drm_v3d_submit_cl* args)
{
	v3d_device* device = file->device;

	// The client wrote its control lists through a write-combining mapping
	// on this CPU; the worker that starts the job may run on another one,
	// and its barrier does not drain this CPU's write buffer.
	v3d_barrier();

	// No extensions, no multisync, no perfmon: GET_PARAM says so.
	if ((args->flags & ~DRM_V3D_SUBMIT_CL_FLUSH_CACHE) != 0
		|| args->extensions != 0 || args->perfmon_id != 0 || args->pad != 0)
		return B_BAD_VALUE;
	// bcl_start == bcl_end means "render only"; a render list there must be.
	if (args->rcl_start == args->rcl_end)
		return B_BAD_VALUE;
	if (args->bo_handle_count > kMaxJobBOs)
		return B_BAD_VALUE;

	v3d_job* job = (v3d_job*)calloc(1, sizeof(v3d_job));
	if (job == NULL)
		return B_NO_MEMORY;
	job->bcl_start = args->bcl_start;
	job->bcl_end = args->bcl_end;
	job->rcl_start = args->rcl_start;
	job->rcl_end = args->rcl_end;
	job->qma = args->qma;
	job->qms = args->qms;
	job->qts = args->qts;
	job->flags = args->flags;

	const uint32 count = args->bo_handle_count;
	uint32* handles = NULL;
	if (count > 0) {
		const void* userHandles = (const void*)(addr_t)args->bo_handles;
		handles = (uint32*)malloc(count * sizeof(uint32));
		job->bos = (v3d_bo**)calloc(count, sizeof(v3d_bo*));
		if (handles == NULL || job->bos == NULL) {
			free(handles);
			free(job->bos);
			free(job);
			return B_NO_MEMORY;
		}
		if (!IS_USER_ADDRESS(userHandles)
			|| user_memcpy(handles, userHandles, count * sizeof(uint32))
				!= B_OK) {
			free(handles);
			free(job->bos);
			free(job);
			return B_BAD_ADDRESS;
		}
	}

	MutexLocker locker(device->lock);
	if (device->broken) {
		free(handles);
		free_job(device, job);
		return B_DEVICE_NOT_FOUND;
	}

	for (uint32 i = 0; i < count; i++) {
		v3d_bo* bo = v3d_bo_lookup(file, handles[i]);
		if (bo == NULL) {
			free(handles);
			free_job(device, job);
			return B_ENTRY_NOT_FOUND;
		}
		bo->references++;
		job->bos[job->bo_count++] = bo;
	}
	free(handles);

	job->seqno = ++device->submitted_seqno;
	for (uint32 i = 0; i < job->bo_count; i++)
		job->bos[i]->last_seqno = job->seqno;

	// The out-sync gets the job's fence now (DRM semantics); an unknown
	// handle is ignored, as Linux does.
	v3d_syncobj* syncobj = v3d_handle_lookup(file->syncobjs,
		file->syncobj_slots, args->out_sync);
	if (syncobj != NULL) {
		syncobj->has_fence = true;
		syncobj->seqno = job->seqno;
	}

	if (device->queue_tail != NULL)
		device->queue_tail->next = job;
	else
		device->queue_head = job;
	device->queue_tail = job;

	// for SYNCOBJ_WAIT with WAIT_FOR_SUBMIT
	device->job_done.NotifyAll();
	locker.Unlock();

	release_sem(device->worker_sem);
	return B_OK;
}


/*!	Waits until job \a seqno completed. \a deadline is absolute, 0 to only
	check, or B_INFINITE_TIMEOUT. Returns ETIME on time-out (Mesa checks for
	it). Called with the device lock held.
*/
status_t
v3d_wait_seqno(v3d_device* device, v3d_file* file, uint64 seqno,
	bigtime_t deadline)
{
	while (device->completed_seqno < seqno) {
		if (file->closing)
			return B_FILE_ERROR;
		if (deadline == 0
			|| (deadline != B_INFINITE_TIMEOUT && system_time() >= deadline))
			return ETIME;

		status_t status = device->job_done.Wait(&device->lock,
			B_CAN_INTERRUPT
				| (deadline != B_INFINITE_TIMEOUT ? B_ABSOLUTE_TIMEOUT : 0),
			deadline);
		if (status == B_INTERRUPTED)
			return status;
	}
	return B_OK;
}


//	#pragma mark - syncobjs


static status_t
syncobj_create(v3d_file* file, drm_syncobj_create* args)
{
	if ((args->flags & ~DRM_SYNCOBJ_CREATE_SIGNALED) != 0)
		return B_BAD_VALUE;

	v3d_syncobj* syncobj = (v3d_syncobj*)calloc(1, sizeof(v3d_syncobj));
	if (syncobj == NULL)
		return B_NO_MEMORY;
	// Signalled = a fence that is always signalled (Mesa creates its
	// out-sync this way and waits for it before its first submit).
	syncobj->has_fence = (args->flags & DRM_SYNCOBJ_CREATE_SIGNALED) != 0;

	MutexLocker locker(file->device->lock);
	status_t status = v3d_handle_alloc(file->syncobjs, file->syncobj_slots,
		syncobj, &args->handle);
	if (status != B_OK)
		free(syncobj);
	return status;
}


static status_t
syncobj_destroy(v3d_file* file, drm_syncobj_destroy* args)
{
	if (args->pad != 0)
		return B_BAD_VALUE;

	MutexLocker locker(file->device->lock);
	v3d_syncobj* syncobj = v3d_handle_lookup(file->syncobjs,
		file->syncobj_slots, args->handle);
	if (syncobj == NULL)
		return B_BAD_VALUE;
	file->syncobjs[args->handle - 1] = NULL;
	free(syncobj);
	return B_OK;
}


static status_t
copy_handles(uint64 userAddress, uint32 count, uint32** _handles)
{
	if (count == 0 || count > kMaxJobBOs)
		return B_BAD_VALUE;
	const void* source = (const void*)(addr_t)userAddress;
	uint32* handles = (uint32*)malloc(count * sizeof(uint32));
	if (handles == NULL)
		return B_NO_MEMORY;
	if (!IS_USER_ADDRESS(source)
		|| user_memcpy(handles, source, count * sizeof(uint32)) != B_OK) {
		free(handles);
		return B_BAD_ADDRESS;
	}
	*_handles = handles;
	return B_OK;
}


/*!	RESET drops the fence, SIGNAL replaces it with a signalled one. */
static status_t
syncobj_reset_or_signal(v3d_file* file, drm_syncobj_array* args, bool signal)
{
	if (args->pad != 0)
		return B_BAD_VALUE;
	uint32* handles;
	status_t status = copy_handles(args->handles, args->count_handles,
		&handles);
	if (status != B_OK)
		return status;

	MutexLocker locker(file->device->lock);
	for (uint32 i = 0; i < args->count_handles && status == B_OK; i++) {
		if (v3d_handle_lookup(file->syncobjs, file->syncobj_slots, handles[i])
				== NULL)
			status = B_BAD_VALUE;
	}
	for (uint32 i = 0; i < args->count_handles && status == B_OK; i++) {
		v3d_syncobj* syncobj = file->syncobjs[handles[i] - 1];
		syncobj->has_fence = signal;
		syncobj->seqno = 0;
	}
	if (status == B_OK && signal)
		file->device->job_done.NotifyAll();
	free(handles);
	return status;
}


/*!	The time-out is an absolute CLOCK_MONOTONIC time in nanoseconds, which on
	Haiku is system_time() * 1000.
*/
static status_t
syncobj_wait(v3d_file* file, drm_syncobj_wait* args)
{
	v3d_device* device = file->device;
	const uint32 knownFlags = DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL
		| DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT
		| DRM_SYNCOBJ_WAIT_FLAGS_WAIT_DEADLINE;
	if ((args->flags & ~knownFlags) != 0 || args->pad != 0)
		return B_BAD_VALUE;
	const bool waitAll = (args->flags & DRM_SYNCOBJ_WAIT_FLAGS_WAIT_ALL) != 0;
	const bool forSubmit
		= (args->flags & DRM_SYNCOBJ_WAIT_FLAGS_WAIT_FOR_SUBMIT) != 0;

	bigtime_t deadline;
	if (args->timeout_nsec <= 0)
		deadline = 0;
	else if (args->timeout_nsec >= INT64_MAX / 2)
		deadline = B_INFINITE_TIMEOUT;
	else
		deadline = args->timeout_nsec / 1000;

	uint32* handles;
	status_t status = copy_handles(args->handles, args->count_handles,
		&handles);
	if (status != B_OK)
		return status;

	MutexLocker locker(device->lock);
	while (true) {
		uint32 signalled = 0;
		int32 first = -1;
		for (uint32 i = 0; i < args->count_handles; i++) {
			v3d_syncobj* syncobj = v3d_handle_lookup(file->syncobjs,
				file->syncobj_slots, handles[i]);
			if (syncobj == NULL) {
				status = B_ENTRY_NOT_FOUND;
				break;
			}
			if (!syncobj->has_fence) {
				if (!forSubmit) {
					// as Linux: nothing to wait for is an error
					status = B_BAD_VALUE;
					break;
				}
				continue;
			}
			if (device->completed_seqno >= syncobj->seqno) {
				signalled++;
				if (first < 0)
					first = i;
			}
		}
		if (status != B_OK)
			break;
		if (waitAll ? signalled == args->count_handles : signalled > 0) {
			args->first_signaled = first;
			break;
		}

		if (file->closing) {
			status = B_FILE_ERROR;
			break;
		}
		if (deadline == 0
			|| (deadline != B_INFINITE_TIMEOUT && system_time() >= deadline)) {
			status = ETIME;
			break;
		}
		status = device->job_done.Wait(&device->lock,
			B_CAN_INTERRUPT
				| (deadline != B_INFINITE_TIMEOUT ? B_ABSOLUTE_TIMEOUT : 0),
			deadline);
		if (status == B_INTERRUPTED)
			break;
		status = B_OK;
	}

	free(handles);
	return status;
}


status_t
v3d_syncobj_ioctl(v3d_file* file, uint32 nr, void* data)
{
	switch (nr) {
		case 0xbf:
			return syncobj_create(file, (drm_syncobj_create*)data);
		case 0xc0:
			return syncobj_destroy(file, (drm_syncobj_destroy*)data);
		case 0xc3:
			return syncobj_wait(file, (drm_syncobj_wait*)data);
		case 0xc4:
			return syncobj_reset_or_signal(file, (drm_syncobj_array*)data,
				false);
		case 0xc5:
			return syncobj_reset_or_signal(file, (drm_syncobj_array*)data,
				true);
	}
	return B_DEV_INVALID_IOCTL;
}


void
v3d_file_close_syncobjs(v3d_file* file)
{
	for (uint32 i = 0; i < file->syncobj_slots; i++)
		free(file->syncobjs[i]);
	free(file->syncobjs);
	file->syncobjs = NULL;
	file->syncobj_slots = 0;
}
