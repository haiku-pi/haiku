/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Jobs, the interrupt handler, the watchdog, and syncobjs.

	One job is on the GPU at a time (its binner list, then its render list;
	a texture formatting unit job; or a compute shader dispatch), but the
	caller does not wait for it: the SUBMIT ioctls queue the job and return.
	CPU jobs (SUBMIT_CPU: timestamp queries, indirect dispatch set-up) take
	their turn in the same queue and run on the worker when they get it.
	A worker thread owns the hardware: it kicks jobs, takes their completion
	from the interrupt handler, runs the watchdog and resets a wedged GPU.
	Waits (WAIT_BO, SYNCOBJ_WAIT) sleep until a job's sequence number has
	completed.

	Jobs run in submission order, one at a time, so every fence a new job may
	depend on - its in-syncs, the last job of each of its BOs - belongs to an
	earlier job and has signalled by the time the new job starts: in-syncs
	(also those of the multisync extension) need no tracking yet, and its
	wait stage does not matter. Running the queues concurrently will change
	that.

	The cache and TLB maintenance around each job is the sequence proven on
	this board by the Phoenix-RTOS V3D server; every step there is load-
	bearing, and each is marked below.
*/


#include "v3d_private.h"

#include <driver_settings.h>

#include <util/AutoLock.h>


static const uint32 kMaxJobBOs = 16384;
static const uint32 kMaxArrayCount = 65536;
	// handles, offsets and syncs of one submission
static const int32 kMaxExtensions = 16;
static const int32 kSpuriousLimit = 1000;
static const int32 kBurstLimit = 200000;
static const bigtime_t kStatsInterval = 5000000;


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
	job->render_started = system_time();
	start_watchdog(job, job->rcl_start, 0);
	core_write(device, kCle1QueuedBase, job->rcl_start);
	core_write(device, kCle1QueuedEnd, job->rcl_end);
}


/*!	The TFU reads and writes memory through the MMU like the other units:
	the prologue's barrier and TLB flush apply. Writing the input
	configuration starts it.
*/
static void
kick_tfu(v3d_device* device, v3d_job* job)
{
	prepare_caches(device);
	{
		// a completion still latched from the previous TFU job
		InterruptsSpinLocker locker(device->irq_lock);
		if ((hub_read(device, kHubInterruptStatus) & kHubTfuDone) != 0)
			hub_write(device, kHubInterruptClear, kHubTfuDone);
		device->pending_hub &= ~kHubTfuDone;
	}

	const drm_v3d_submit_tfu& tfu = job->tfu;
	hub_write(device, kTfuInputImageAddress, tfu.iia);
	hub_write(device, kTfuInputImageStride, tfu.iis);
	hub_write(device, kTfuInputChromaAddress, tfu.ica);
	hub_write(device, kTfuInputUAddress, tfu.iua);
	hub_write(device, kTfuOutputAddress, tfu.ioa);
	hub_write(device, kTfuOutputSize, tfu.ios);
	hub_write(device, kTfuCoefficient0, tfu.coef[0]);
	if ((tfu.coef[0] & kTfuUseCoefficients) != 0) {
		hub_write(device, kTfuCoefficient1, tfu.coef[1]);
		hub_write(device, kTfuCoefficient2, tfu.coef[2]);
		hub_write(device, kTfuCoefficient3, tfu.coef[3]);
	}

	job->stage = V3D_JOB_TFU;
	start_watchdog(job, hub_read(device, kTfuControlStatus), 0);
	hub_write(device, kTfuInputConfig, tfu.icfg | kTfuInterruptOnCompletion);
}


/*!	A compute shader dispatch (as Linux v3d_csd_job_run()): the caches are
	invalidated as for a control list, then the queued configuration is
	written; CFG0 last, which starts it.
*/
static void
kick_csd(v3d_device* device, v3d_job* job)
{
	prepare_caches(device);
	drain_stale(device, kComputeDone);

	job->stage = V3D_JOB_CSD;
	start_watchdog(job, ~0u, 0);
	for (int32 i = 1; i <= 6; i++)
		core_write(device, kCsdQueuedConfig0 + 4 * i, job->csd_config[i]);
	core_write(device, kCsdQueuedConfig0, job->csd_config[0]);
}


static void finish_job(v3d_device* device, v3d_job* job);


static void
schedule(v3d_device* device)
{
	while (device->active == NULL && device->queue_head != NULL
		&& !device->broken) {
		v3d_job* job = device->queue_head;
		device->queue_head = job->next;
		if (device->queue_head == NULL)
			device->queue_tail = NULL;
		job->next = NULL;
		device->active = job;
		job->started = system_time();

		switch (job->type) {
			case V3D_JOB_TYPE_CL:
				if (job->bcl_start != job->bcl_end)
					kick_bin(device, job);
				else
					kick_render(device, job);
				break;
			case V3D_JOB_TYPE_TFU:
				kick_tfu(device, job);
				break;
			case V3D_JOB_TYPE_CSD:
				if (job->skip)
					finish_job(device, job);
				else
					kick_csd(device, job);
				break;
			case V3D_JOB_TYPE_CPU:
				// every earlier job is done: its turn has come
				v3d_cpu_job_run(device, job);
				finish_job(device, job);
				break;
		}
	}
}


/*!	Frees a job that is done, dropped, or was never queued. */
void
v3d_job_free(v3d_device* device, v3d_job* job)
{
	for (uint32 i = 0; i < job->bo_count; i++)
		v3d_bo_put(device, job->bos[i]);
	free(job->bos);
	free(job->handles);
	if (job->cpu != NULL)
		v3d_cpu_job_free(device, job->cpu);
	free(job);
}


/*!	The job is done, or was dropped by a reset (its fence signals anyway:
	resubmitting a job that wedged the GPU wedges it again).
*/
static void
account_job(v3d_device* device, v3d_job* job)
{
	auto& stats = device->stats;
	const bigtime_t now = system_time();
	switch (job->type) {
		case V3D_JOB_TYPE_CL:
			stats.cl_jobs++;
			if (job->render_started != 0) {
				stats.bin_time += job->render_started - job->started;
				stats.render_time += now - job->render_started;
			}
			break;
		case V3D_JOB_TYPE_TFU:
			stats.tfu_jobs++;
			break;
		case V3D_JOB_TYPE_CSD:
			stats.csd_jobs++;
			break;
		case V3D_JOB_TYPE_CPU:
			stats.cpu_jobs++;
			break;
	}
	stats.gpu_time += now - job->started;
	const bigtime_t latency = now - job->submitted;
	stats.latency += latency;
	if (latency > stats.max_latency)
		stats.max_latency = latency;

	if (now - stats.window_start < kStatsInterval)
		return;
	const uint32 jobs = stats.cl_jobs + stats.tfu_jobs + stats.csd_jobs
		+ stats.cpu_jobs;
	TRACE_ALWAYS("stats %" B_PRId64 " ms: %" B_PRIu32 " CL + %" B_PRIu32 " TFU + %"
		B_PRIu32 " CSD + %" B_PRIu32 " CPU jobs, GPU %" B_PRId64 " us/job, submit to done %"
		B_PRId64 " us/job (max %" B_PRId64 " us), %" B_PRIu32 " waits for %" B_PRId64
		" ms\n", (now - stats.window_start) / 1000, stats.cl_jobs, stats.tfu_jobs,
		stats.csd_jobs, stats.cpu_jobs, stats.gpu_time / jobs, stats.latency / jobs,
		stats.max_latency, stats.waits, stats.wait_time / 1000);
	if (stats.cl_jobs > 0) {
		TRACE_ALWAYS("stats: control lists bin %" B_PRId64 " + render %" B_PRId64
			" us/job; GPU busy %" B_PRId64 "%%\n", stats.bin_time / stats.cl_jobs,
			stats.render_time / stats.cl_jobs,
			stats.gpu_time * 100 / (now - stats.window_start));
	}
	if (stats.syncs > 0) {
		TRACE_ALWAYS("stats: %" B_PRIu32 " cache syncs of imported memory, %"
			B_PRIu64 " KiB and %" B_PRId64 " us each\n", stats.syncs,
			stats.sync_bytes / stats.syncs / 1024, stats.sync_time / stats.syncs);
	}
	const bool enabled = stats.enabled;
	memset(&stats, 0, sizeof(stats));
	stats.enabled = enabled;
	stats.window_start = now;
}


static void
finish_job(v3d_device* device, v3d_job* job)
{
	if (device->stats.enabled)
		account_job(device, job);
	device->active = NULL;
	atomic_set64((int64*)&device->completed_seqno, job->seqno);

	// The render list read the tile lists in these chunks; now they are free.
	return_overflow_chunks(device, job->overflow_chunks);

	v3d_job_free(device, job);
	v3d_bo_release_quarantine(device);
	device->job_done.NotifyAll();
	v3d_sync_file_notify(device);
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


static void
csd_done(v3d_device* device, v3d_job* job)
{
	// What the shaders wrote through the TMU reaches memory before anyone
	// is told (Linux queues a cache clean job after every dispatch).
	clean_caches(device);
	finish_job(device, job);
}


//	#pragma mark - watchdog and reset


static void
progress_registers(v3d_device* device, v3d_job* job, uint32& current,
	uint32& _return)
{
	if (job->stage == V3D_JOB_TFU) {
		// its conversion count moves while it works
		current = hub_read(device, kTfuControlStatus);
		_return = 0;
	} else if (job->stage == V3D_JOB_CSD) {
		// the batches still to go
		current = core_read(device, kCsdCurrentConfig4);
		_return = 0;
	} else if (job->stage == V3D_JOB_BIN) {
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
	if (job->type == V3D_JOB_TYPE_CSD) {
		ERROR("compute job %" B_PRIu64 " wedged: status %#" B_PRIx32
			", batches left %#" B_PRIx32 ", cfg %#" B_PRIx32 " %#" B_PRIx32
			" %#" B_PRIx32 " %#" B_PRIx32 " %#" B_PRIx32 " %#" B_PRIx32 " %#"
			B_PRIx32 ", int %#" B_PRIx32 ", mmu illegal %#" B_PRIx32 "\n",
			job->seqno, core_read(device, kCsdStatus),
			core_read(device, kCsdCurrentConfig4), job->csd_config[0],
			job->csd_config[1], job->csd_config[2], job->csd_config[3],
			job->csd_config[4], job->csd_config[5], job->csd_config[6],
			core_read(device, kCoreInterruptStatus),
			hub_read(device, kMmuIllegalAddress));
		return;
	}
	if (job->type == V3D_JOB_TYPE_TFU) {
		const drm_v3d_submit_tfu& tfu = job->tfu;
		ERROR("TFU job %" B_PRIu64 " wedged: cs %#" B_PRIx32 ", icfg %#"
			B_PRIx32 " iia %#" B_PRIx32 " iis %#" B_PRIx32 " ioa %#" B_PRIx32
			" ios %#" B_PRIx32 ", hub int %#" B_PRIx32 ", mmu illegal %#"
			B_PRIx32 "\n", job->seqno, hub_read(device, kTfuControlStatus),
			tfu.icfg, tfu.iia, tfu.iis, tfu.ioa, tfu.ios,
			hub_read(device, kHubInterruptStatus),
			hub_read(device, kMmuIllegalAddress));
		return;
	}

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
	ERROR("  qma %#" B_PRIx32 " qms %#" B_PRIx32 " qts %#" B_PRIx32
		", flags %#" B_PRIx32 ", %" B_PRIu32 " buffers:\n", job->qma, job->qms,
		job->qts, job->flags, job->bo_count);
	for (uint32 i = 0; i < job->bo_count; i++) {
		const v3d_bo* bo = job->bos[i];
		ERROR("    [%#" B_PRIx32 "..%#" B_PRIx32 ") last job %" B_PRIu64
			", %" B_PRId32 " references\n", bo->gpu_address,
			bo->gpu_address + (uint32)bo->size, bo->last_seqno, bo->references);
	}
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
	const uint64 droppedJob = job != NULL ? job->seqno : 0;
		// (finish_job() releases the job)

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
		" resets so far\n", reason, droppedJob, device->resets);
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

	if (job->stage == V3D_JOB_RENDER || job->stage == V3D_JOB_CSD) {
		// The QPU sources are masked, so the handler only acknowledges them
		// when something else fires: do it here while shaders run long.
		InterruptsSpinLocker locker(device->irq_lock);
		const uint32 qpu = core_read(device, kCoreInterruptStatus)
			& kQpuInterrupts;
		if (qpu != 0) {
			core_write(device, kCoreInterruptClear, qpu);
			job->qpu_acks++;
		}
	}

	if (now - job->last_progress >= kWatchdogTimeout)
		reset_gpu(device, "a stalled job");
	else if (now - job->kicked >= kWatchdogHardLimit)
		reset_gpu(device, "a runaway job");
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
	else if (job != NULL && job->stage == V3D_JOB_TFU
		&& (hub & kHubTfuDone) != 0)
		finish_job(device, job);
	else if (job != NULL && job->stage == V3D_JOB_CSD
		&& (core & kComputeDone) != 0)
		csd_done(device, job);

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

	void* settings = load_driver_settings("kernel");
	device->stats.enabled = get_driver_boolean_parameter(settings, "v3d_stats", false, true);
	unload_driver_settings(settings);
	device->stats.window_start = system_time();

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
		v3d_job_free(device, device->active);
		device->active = NULL;
	}
	while (device->queue_head != NULL) {
		v3d_job* job = device->queue_head;
		device->queue_head = job->next;
		v3d_job_free(device, job);
	}
	device->queue_tail = NULL;
}


//	#pragma mark - submission and waits


/*!	Copies an array of \a count 32-bit values (handles, offsets) from the
	caller; none for a count of 0.
*/
status_t
v3d_copy_user_array(uint64 userAddress, uint32 count, uint32** _array)
{
	*_array = NULL;
	if (count == 0)
		return B_OK;
	if (count > kMaxArrayCount)
		return B_BAD_VALUE;

	const void* source = (const void*)(addr_t)userAddress;
	uint32* array = (uint32*)malloc(count * sizeof(uint32));
	if (array == NULL)
		return B_NO_MEMORY;
	if (!IS_USER_ADDRESS(source)
		|| user_memcpy(array, source, count * sizeof(uint32)) != B_OK) {
		free(array);
		return B_BAD_ADDRESS;
	}
	*_array = array;
	return B_OK;
}


static void
free_extensions(v3d_submit_ext& ext)
{
	free(ext.out_syncs);
	ext.out_syncs = NULL;
	ext.out_sync_count = 0;
}


/*!	The handles of a multisync extension's out-syncs. Its in-syncs need no
	tracking (see the top of this file).
*/
static status_t
read_out_syncs(const drm_v3d_multi_sync& multisync, v3d_submit_ext& ext)
{
	const uint32 count = multisync.out_sync_count;
	if (count == 0)
		return B_OK;
	if (count > kMaxArrayCount)
		return B_BAD_VALUE;

	ext.out_syncs = (uint32*)malloc(count * sizeof(uint32));
	if (ext.out_syncs == NULL)
		return B_NO_MEMORY;
	ext.out_sync_count = count;

	const drm_v3d_sem* user = (const drm_v3d_sem*)(addr_t)multisync.out_syncs;
	for (uint32 i = 0; i < count; i++) {
		drm_v3d_sem sem;
		if (!IS_USER_ADDRESS(user + i)
			|| user_memcpy(&sem, user + i, sizeof(sem)) != B_OK) {
			return B_BAD_ADDRESS;
		}
		ext.out_syncs[i] = sem.handle;
	}
	return B_OK;
}


/*!	Reads the extension chain of a submit ioctl (as Linux
	v3d_get_extensions()): a multisync extension replaces the single in- and
	out-sync; a CPU job extension (only for SUBMIT_CPU, \a cpuJob) is left
	to v3d_cpu_job_create(). \a ext must be freed with free_extensions().
*/
static status_t
read_extensions(uint32 flags, uint64 userAddress, bool cpuJob,
	v3d_submit_ext& ext)
{
	memset(&ext, 0, sizeof(ext));
	if ((flags & DRM_V3D_SUBMIT_EXTENSION) == 0)
		return B_OK;

	for (int32 count = 0; userAddress != 0; count++) {
		const void* user = (const void*)(addr_t)userAddress;
		drm_v3d_extension header;
		if (count == kMaxExtensions)
			return B_BAD_VALUE;
		if (!IS_USER_ADDRESS(user)
			|| user_memcpy(&header, user, sizeof(header)) != B_OK) {
			return B_BAD_ADDRESS;
		}

		switch (header.id) {
			case DRM_V3D_EXT_ID_MULTI_SYNC:
			{
				drm_v3d_multi_sync multisync;
				if (ext.multisync)
					return B_BAD_VALUE;
				if (user_memcpy(&multisync, user, sizeof(multisync)) != B_OK)
					return B_BAD_ADDRESS;
				if (multisync.pad != 0 || (multisync.in_sync_count == 0
						&& multisync.out_sync_count == 0)) {
					return B_BAD_VALUE;
				}
				ext.multisync = true;
				status_t status = read_out_syncs(multisync, ext);
				if (status != B_OK)
					return status;
				break;
			}

			case DRM_V3D_EXT_ID_CPU_INDIRECT_CSD:
			case DRM_V3D_EXT_ID_CPU_TIMESTAMP_QUERY:
			case DRM_V3D_EXT_ID_CPU_RESET_TIMESTAMP_QUERY:
			case DRM_V3D_EXT_ID_CPU_COPY_TIMESTAMP_QUERY:
			case DRM_V3D_EXT_ID_CPU_RESET_PERFORMANCE_QUERY:
			case DRM_V3D_EXT_ID_CPU_COPY_PERFORMANCE_QUERY:
				// one per CPU job, and only there
				if (!cpuJob || ext.cpu_type != 0)
					return B_BAD_VALUE;
				ext.cpu_type = header.id;
				ext.cpu_extension = userAddress;
				break;

			default:
				return B_BAD_VALUE;
		}
		userAddress = header.next;
	}
	return B_OK;
}


/*!	Takes the buffers of \a first and of the jobs chained to it (next), gives
	them the next sequence numbers and the out-syncs the fence of the last
	one, and queues them in one go: nothing can come in between. Owns the
	jobs. With a multisync extension its out-syncs replace \a outSync.
*/
static status_t
queue_jobs(v3d_file* file, v3d_job* first, const v3d_submit_ext& ext,
	uint32 outSync)
{
	v3d_device* device = file->device;

	status_t status = B_OK;
	for (v3d_job* job = first; job != NULL && status == B_OK;
			job = job->next) {
		if (job->handle_count > kMaxJobBOs)
			status = B_BAD_VALUE;
		else if (job->handle_count > 0) {
			job->bos = (v3d_bo**)calloc(job->handle_count, sizeof(v3d_bo*));
			if (job->bos == NULL)
				status = B_NO_MEMORY;
		}
	}

	MutexLocker locker(device->lock);
	if (status == B_OK && device->broken)
		status = B_DEVICE_NOT_FOUND;

	v3d_job* last = NULL;
	for (v3d_job* job = first; job != NULL && status == B_OK;
			job = job->next) {
		for (uint32 i = 0; i < job->handle_count; i++) {
			v3d_bo* bo = v3d_bo_lookup(file, job->handles[i]);
			if (bo == NULL) {
				status = B_ENTRY_NOT_FOUND;
				break;
			}
			bo->references++;
			job->bos[job->bo_count++] = bo;
		}
		if (status == B_OK && job->type == V3D_JOB_TYPE_CPU)
			status = v3d_cpu_job_resolve(file, job);
		last = job;
	}

	// As Linux, an unknown out-sync of the multisync extension fails the
	// submission; an unknown single one is ignored.
	for (uint32 i = 0; i < ext.out_sync_count && status == B_OK; i++) {
		if (v3d_syncobj_lookup(file, ext.out_syncs[i]) == NULL)
			status = B_BAD_VALUE;
	}

	if (status != B_OK) {
		while (first != NULL) {
			v3d_job* next = first->next;
			v3d_job_free(device, first);
			first = next;
		}
		return status;
	}

	const bigtime_t now = system_time();
	for (v3d_job* job = first; job != NULL; job = job->next) {
		job->seqno = ++device->submitted_seqno;
		job->submitted = now;
		for (uint32 i = 0; i < job->bo_count; i++)
			job->bos[i]->last_seqno = job->seqno;
		free(job->handles);
		job->handles = NULL;
	}

	// The out-syncs get the fence of the last job now (DRM semantics).
	if (ext.multisync) {
		for (uint32 i = 0; i < ext.out_sync_count; i++) {
			v3d_syncobj* syncobj = v3d_syncobj_lookup(file, ext.out_syncs[i]);
			syncobj->has_fence = true;
			syncobj->seqno = last->seqno;
		}
	} else {
		v3d_syncobj* syncobj = v3d_syncobj_lookup(file, outSync);
		if (syncobj != NULL) {
			syncobj->has_fence = true;
			syncobj->seqno = last->seqno;
		}
	}

	if (device->queue_tail != NULL)
		device->queue_tail->next = first;
	else
		device->queue_head = first;
	device->queue_tail = last;

	// for SYNCOBJ_WAIT with WAIT_FOR_SUBMIT
	device->job_done.NotifyAll();
	locker.Unlock();

	release_sem(device->worker_sem);
	return B_OK;
}


status_t
v3d_submit_cl(v3d_file* file, drm_v3d_submit_cl* args)
{
	// The client wrote its control lists through a write-combining mapping
	// on this CPU; the worker that starts the job may run on another one,
	// and its barrier does not drain this CPU's write buffer.
	v3d_barrier();

	// No perfmon: GET_PARAM says so.
	const uint32 knownFlags = DRM_V3D_SUBMIT_CL_FLUSH_CACHE
		| DRM_V3D_SUBMIT_EXTENSION;
	if ((args->flags & ~knownFlags) != 0 || args->perfmon_id != 0
		|| args->pad != 0) {
		return B_BAD_VALUE;
	}
	// bcl_start == bcl_end means "render only"; a render list there must be.
	if (args->rcl_start == args->rcl_end)
		return B_BAD_VALUE;

	v3d_job* job = (v3d_job*)calloc(1, sizeof(v3d_job));
	if (job == NULL)
		return B_NO_MEMORY;
	job->type = V3D_JOB_TYPE_CL;
	job->bcl_start = args->bcl_start;
	job->bcl_end = args->bcl_end;
	job->rcl_start = args->rcl_start;
	job->rcl_end = args->rcl_end;
	job->qma = args->qma;
	job->qms = args->qms;
	job->qts = args->qts;
	job->flags = args->flags;
	job->handle_count = args->bo_handle_count;

	v3d_submit_ext ext;
	status_t status = read_extensions(args->flags, args->extensions, false,
		ext);
	if (status == B_OK) {
		status = v3d_copy_user_array(args->bo_handles, args->bo_handle_count,
			&job->handles);
	}
	if (status == B_OK)
		status = queue_jobs(file, job, ext, args->out_sync);
	else {
		free(job->handles);
		free(job);
	}
	free_extensions(ext);
	return status;
}


/*!	A texture formatting unit job (texture uploads, mipmap generation): the
	TFU converts the input image into the output BO's tiled layout and
	writes the requested mip levels.
*/
status_t
v3d_submit_tfu(v3d_file* file, drm_v3d_submit_tfu* args)
{
	// the source image was written through a write-combining mapping
	v3d_barrier();

	if ((args->flags & ~DRM_V3D_SUBMIT_EXTENSION) != 0
		|| args->bo_handles[0] == 0) {
		return B_BAD_VALUE;
	}

	v3d_job* job = (v3d_job*)calloc(1, sizeof(v3d_job));
	if (job == NULL)
		return B_NO_MEMORY;
	job->type = V3D_JOB_TYPE_TFU;
	job->tfu = *args;

	// the output BO first, then the inputs; 0 ends the list
	uint32 count = 0;
	while (count < B_COUNT_OF(args->bo_handles) && args->bo_handles[count] != 0)
		count++;
	job->handles = (uint32*)malloc(count * sizeof(uint32));
	if (job->handles == NULL) {
		free(job);
		return B_NO_MEMORY;
	}
	memcpy(job->handles, args->bo_handles, count * sizeof(uint32));
	job->handle_count = count;

	v3d_submit_ext ext;
	status_t status = read_extensions(args->flags, args->extensions, false,
		ext);
	if (status == B_OK)
		status = queue_jobs(file, job, ext, args->out_sync);
	else {
		free(job->handles);
		free(job);
	}
	free_extensions(ext);
	return status;
}


/*!	A compute shader dispatch job of \a args (also the one an indirect
	dispatch CPU job carries), with its BO handles, not queued yet.
*/
status_t
v3d_csd_job_create(const drm_v3d_submit_csd& args, v3d_job** _job)
{
	// No perfmon: GET_PARAM says so.
	if ((args.flags & ~DRM_V3D_SUBMIT_EXTENSION) != 0 || args.pad != 0
		|| args.perfmon_id != 0) {
		return B_BAD_VALUE;
	}

	v3d_job* job = (v3d_job*)calloc(1, sizeof(v3d_job));
	if (job == NULL)
		return B_NO_MEMORY;
	job->type = V3D_JOB_TYPE_CSD;
	memcpy(job->csd_config, args.cfg, sizeof(job->csd_config));
	job->handle_count = args.bo_handle_count;

	status_t status = v3d_copy_user_array(args.bo_handles,
		args.bo_handle_count, &job->handles);
	if (status != B_OK) {
		free(job);
		return status;
	}
	*_job = job;
	return B_OK;
}


status_t
v3d_submit_csd(v3d_file* file, drm_v3d_submit_csd* args)
{
	// uniforms and the shader were written through write-combining mappings
	v3d_barrier();

	v3d_job* job;
	status_t status = v3d_csd_job_create(*args, &job);
	if (status != B_OK)
		return status;

	v3d_submit_ext ext;
	status = read_extensions(args->flags, args->extensions, false, ext);
	if (status == B_OK)
		status = queue_jobs(file, job, ext, args->out_sync);
	else {
		free(job->handles);
		free(job);
	}
	free_extensions(ext);
	return status;
}


/*!	Work the CPU does in queue order (cpu_jobs.cpp). The job type comes from
	its extension, which must be there; so must, in practice, a multisync
	extension, for the CPU job has no single out-sync.
*/
status_t
v3d_submit_cpu(v3d_file* file, drm_v3d_submit_cpu* args)
{
	v3d_barrier();

	if (args->flags != DRM_V3D_SUBMIT_EXTENSION)
		return B_BAD_VALUE;

	v3d_submit_ext ext;
	status_t status = read_extensions(args->flags, args->extensions, true,
		ext);
	if (status == B_OK && ext.cpu_type == 0)
		status = B_BAD_VALUE;

	v3d_job* first = NULL;
	if (status == B_OK)
		status = v3d_cpu_job_create(*args, ext, &first);
	if (status == B_OK)
		status = queue_jobs(file, first, ext, 0);
	free_extensions(ext);
	return status;
}


/*!	Waits until job \a seqno completed. \a deadline is absolute, 0 to only
	check, or B_INFINITE_TIMEOUT. Returns ETIME on time-out (Mesa checks for
	it). Called with the device lock held.
*/
status_t
v3d_wait_seqno(v3d_device* device, v3d_file* file, uint64 seqno,
	bigtime_t deadline)
{
	const bigtime_t start = system_time();
	bool waited = false;
	while (device->completed_seqno < seqno) {
		waited = true;
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
	if (waited && device->stats.enabled) {
		device->stats.waits++;
		device->stats.wait_time += system_time() - start;
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
	syncobj->references = 1;
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
	v3d_syncobj* syncobj = v3d_syncobj_lookup(file, args->handle);
	if (syncobj == NULL)
		return B_BAD_VALUE;
	file->syncobjs[args->handle - 1] = NULL;
	v3d_syncobj_put(syncobj);
	return B_OK;
}


/*!	Exports the syncobj's fence as a sync file (a new file descriptor of the
	caller's). As Linux, a syncobj without a fence has nothing to export:
	Vulkan's runtime relies on that to learn whether a wait was submitted.
	Opaque syncobj file descriptors (sharing the syncobj itself) are not
	supported.
*/
static status_t
syncobj_handle_to_fd(v3d_file* file, drm_syncobj_handle* args)
{
	if (args->flags != DRM_SYNCOBJ_HANDLE_TO_FD_FLAGS_EXPORT_SYNC_FILE)
		return B_NOT_SUPPORTED;
	if (args->pad != 0)
		return B_BAD_VALUE;

	MutexLocker locker(file->device->lock);
	v3d_syncobj* syncobj = v3d_syncobj_lookup(file, args->handle);
	if (syncobj == NULL || !syncobj->has_fence)
		return B_BAD_VALUE;
	const uint64 seqno = syncobj->seqno;
	locker.Unlock();
		// no device lock while the VFS locks the caller's I/O context

	int fd;
	status_t status = v3d_sync_file_create(file->device, seqno, &fd);
	if (status != B_OK)
		return status;
	args->fd = fd;
	return B_OK;
}


/*!	Replaces the syncobj's fence with the one of a sync file. */
static status_t
syncobj_fd_to_handle(v3d_file* file, drm_syncobj_handle* args)
{
	if (args->flags != DRM_SYNCOBJ_FD_TO_HANDLE_FLAGS_IMPORT_SYNC_FILE)
		return B_NOT_SUPPORTED;
	if (args->pad != 0)
		return B_BAD_VALUE;

	uint64 seqno;
	status_t status = v3d_sync_file_get_seqno(file->device, args->fd, &seqno);
	if (status != B_OK)
		return status;

	MutexLocker locker(file->device->lock);
	v3d_syncobj* syncobj = v3d_syncobj_lookup(file, args->handle);
	if (syncobj == NULL)
		return B_BAD_VALUE;
	syncobj->has_fence = true;
	syncobj->seqno = seqno;
	// for SYNCOBJ_WAIT with WAIT_FOR_SUBMIT
	file->device->job_done.NotifyAll();
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
		if (v3d_syncobj_lookup(file, handles[i]) == NULL)
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
			v3d_syncobj* syncobj = v3d_syncobj_lookup(file, handles[i]);
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
		case 0xc1:
			return syncobj_handle_to_fd(file, (drm_syncobj_handle*)data);
		case 0xc2:
			return syncobj_fd_to_handle(file, (drm_syncobj_handle*)data);
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


v3d_syncobj*
v3d_syncobj_lookup(v3d_file* file, uint32 handle)
{
	return v3d_handle_lookup(file->syncobjs, file->syncobj_slots, handle);
}


/*!	Called with the device lock held. */
void
v3d_syncobj_put(v3d_syncobj* syncobj)
{
	if (--syncobj->references == 0)
		free(syncobj);
}


void
v3d_file_close_syncobjs(v3d_file* file)
{
	for (uint32 i = 0; i < file->syncobj_slots; i++) {
		if (file->syncobjs[i] != NULL)
			v3d_syncobj_put(file->syncobjs[i]);
	}
	free(file->syncobjs);
	file->syncobjs = NULL;
	file->syncobj_slots = 0;
}
