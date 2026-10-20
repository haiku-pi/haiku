/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Decoding pictures: a two-stage pipeline.

	Phase 1 (entropy decode) executes the client's command buffer and writes
	prediction units and coefficients into a scratch set the driver owns, a
	stride per CTB row; when a buffer of the set is too small, the block stops
	with an "exhausted" status bit and phase 1 runs again with a larger one.
	Phase 2 (reconstruction) is programmed with direct register writes: the
	scratch set, the output and reference pictures, the collocated motion
	vectors. Each phase ends with an interrupt from the block's interrupt
	controller.

	The phases are separate engines of the block: phase 1 of a picture runs
	while phase 2 reconstructs the picture before it (as Linux's driver does),
	each with its own scratch set. RPIVID_SUBMIT checks a picture and queues
	it; the decode thread starts each phase as soon as its engine and a scratch
	set are free, and handles completions, reruns and timeouts; RPIVID_WAIT
	returns a picture's result. Phase 1 takes the pictures in submission order
	and phase 2 in phase-1 order, so a picture's references - pictures
	submitted before it - are complete before its phase 2 reads them, and a
	picture buffer that becomes the output of a later picture is written only
	after every earlier picture is done with it.

	The command entries are checked before the block sees them: they may
	only set phase-1 parameters, the bitstream addresses in them are offsets
	into the client's bitstream buffer (rebased here), and the bit depth and
	CTB size they program must be the pictures' - so the block reads and
	writes only memory the client was given. A job holds a reference to every
	buffer it names: a buffer the client deletes stays until no job needs it.
*/


#include "rpivid_private.h"

#include <arch/cpu.h>
#include <util/AutoLock.h>


// A picture's command buffer beyond this is not a picture
static const uint32 kMaxCommands = 1 << 18;
static const uint32 kMaxPhase1Runs = 5;
static const bigtime_t kPhase1Timeout = 1000000;
static const bigtime_t kPhase2Timeout = 2000000;
// While a phase runs, the decode thread re-checks the interrupt controller
// this often, so a lost interrupt costs this, not the picture.
static const bigtime_t kPollInterval = 10000;
// The zero motion vectors read for a collocated picture the client does not
// have: for the largest picture (4096 x 4096), allocated once
static const size_t kZeroMvSize = 4096 * (4096 / 16);


//	#pragma mark - completion


/*!	The block's interrupt line is level triggered: the completion bits are
	acknowledged here, and recorded for the decode thread.
*/
int32
rpivid_interrupt(void* data)
{
	rpivid_device* device = (rpivid_device*)data;

	SpinLocker locker(device->irq_lock);
	if (device->intc == NULL)
		return B_UNHANDLED_INTERRUPT;

	const uint32 control = device->intc[0];
	const uint32 active = control & kActiveInterrupts;
	if (active == 0)
		return B_UNHANDLED_INTERRUPT;

	// write 1 to clear; the enables are written back as they are
	device->intc[0] = control & ~(uint32)kInterruptsZeroMask;
	device->pending |= active;
	device->interrupts++;
	locker.Unlock();

	release_sem_etc(device->irq_sem, 1, B_DO_NOT_RESCHEDULE);
	return B_INVOKE_SCHEDULER;
}


/*!	A completion can be seen twice, by the handler and by the decode thread
	polling the controller: the handler's mark may land after the thread took
	the completion. Left there, it would end the next run of that phase at
	once, before the block finished. So before a phase starts, a pending
	completion of it is forgotten (the other phase's is left alone).
*/
static void
clear_completion(rpivid_device* device, uint32 bit)
{
	InterruptsSpinLocker locker(device->irq_lock);
	device->pending &= ~bit;
	const uint32 control = device->intc[0];
	if ((control & bit) != 0) {
		device->intc[0] = (control & ~(uint32)kInterruptsZeroMask
			& ~(uint32)kActiveInterrupts) | bit;
	}
}


static bool
take_completion(rpivid_device* device, uint32 bit)
{
	InterruptsSpinLocker locker(device->irq_lock);
	if ((device->pending & bit) != 0) {
		device->pending &= ~bit;
		return true;
	}
	const uint32 control = device->intc[0];
	if ((control & bit) != 0) {
		device->intc[0] = (control & ~(uint32)kInterruptsZeroMask
			& ~(uint32)kActiveInterrupts) | bit;
		return true;
	}
	return false;
}


//	#pragma mark - device memory


/*!	The scratch buffer sizes as Linux grows them: never below 768 bytes,
	then three times a power of two.
*/
static size_t
round_up_size(size_t size)
{
	uint32 log2 = 8;
	if (size >= 256) {
		log2 = 0;
		while ((size >> (log2 + 1)) != 0)
			log2++;
	}
	return size >= ((size_t)3 << log2) ? (size_t)4 << log2
		: (size_t)3 << log2;
}


/*!	At least \a size bytes of device memory; the contents need not survive. */
static status_t
ensure_memory(rpivid_device* device, dma_memory& memory, size_t size,
	uint32 memoryType, const char* name)
{
	if (memory.area >= 0 && memory.size >= size)
		return B_OK;
	rpivid_dma_free(memory);
	return rpivid_dma_alloc(device, name, size, memoryType, memory);
}


void
rpivid_decode_free_memory(rpivid_device* device)
{
	rpivid_dma_free(device->commands);
	for (uint32 i = 0; i < kScratchSets; i++) {
		rpivid_dma_free(device->scratch[i].pu);
		rpivid_dma_free(device->scratch[i].coeff);
		device->scratch[i].pu_size = device->scratch[i].coeff_size = 0;
	}
	rpivid_dma_free(device->zero_mv);
}


//	#pragma mark - command checks


static bool
phase1_register(uint32 offset)
{
	switch (offset) {
		case 0x00:	// SPS0
		case 0x04:	// SPS1
		case 0x08:	// PPS
		case 0x0c:	// SLICE
		case 0x10:	// TILESTART
		case 0x14:	// TILEEND
		case 0x18:	// SLICESTART
		case 0x1c:	// MODE
		case 0x30:	// QP
		case 0x34:	// CONTROL
		case 0x38:	// STATUS
		case 0x40:	// BFBASE
		case 0x44:	// BFNUM
		case 0x48:	// BFCONTROL
		case 0x60:	// SLICECMDS
		case 0x64:	// BEGINTILEEND
		case 0x68:	// TRANSFER
			return true;
	}
	return (offset >= kProbabilities && offset < kProbabilitiesEnd)
		|| (offset >= kScalingFactors && offset < kScalingFactorsEnd)
		|| (offset >= kSliceMessages && offset < kSliceMessagesEnd);
}


/*!	Checks the entries and rebases their bitstream addresses (offsets in the
	bitstream buffer, in 64-byte units) to bus addresses.
*/
static status_t
check_commands(rpivid_device* device, uint64* entries, uint32 count,
	const rpivid_buffer* bitstream, const rpivid_buffer* output,
	uint32 log2CtbSize)
{
	const uint64 bitstreamUnits = bitstream->memory.size >> 6;
	const uint32 bitstreamBase = rpivid_bus_address(device,
		bitstream->memory.physical);
	const uint32 bitDepths = (output->bit_depth << 16)
		| (output->bit_depth << 20);
	uint64 sliceBase = UINT64_MAX;

	for (uint32 i = 0; i < count; i++) {
		const uint32 offset = (uint32)entries[i];
		uint32 value = (uint32)(entries[i] >> 32);
		if ((offset & 3) != 0 || !phase1_register(offset)) {
			ERROR("command %" B_PRIu32 ": register %#" B_PRIx32 " not allowed\n",
				i, offset);
			return B_NOT_ALLOWED;
		}

		switch (offset) {
			case kSps0:
				if ((value & 0xff0000) != bitDepths
					|| ((value >> 4) & 0xf) != log2CtbSize) {
					ERROR("command %" B_PRIu32 ": SPS0 %#" B_PRIx32 " does not "
						"match the %" B_PRIu32 "-bit pictures, CTB %d\n", i,
						value, output->bit_depth, 1 << log2CtbSize);
					return B_BAD_VALUE;
				}
				break;

			case kBitstreamBase:
				if (value >= bitstreamUnits) {
					ERROR("command %" B_PRIu32 ": slice at %#" B_PRIx64
						" outside the bitstream buffer\n", i, (uint64)value << 6);
					return B_BAD_VALUE;
				}
				sliceBase = value;
				value += bitstreamBase;
				break;

			case kBitstreamLength:
				// the data starts at most 63 bytes into the unit (BFCONTROL)
				if (sliceBase == UINT64_MAX
					|| (sliceBase << 6) + 63 + value
						> bitstream->memory.size) {
					ERROR("command %" B_PRIu32 ": %" B_PRIu32 " bytes of slice "
						"data outside the bitstream buffer\n", i, value);
					return B_BAD_VALUE;
				}
				break;
		}
		entries[i] = offset | ((uint64)value << 32);
	}
	return B_OK;
}


static bool
same_geometry(const rpivid_buffer* a, const rpivid_buffer* b)
{
	return a->width == b->width && a->height == b->height
		&& a->bit_depth == b->bit_depth;
}


//	#pragma mark - jobs


static void
job_free(rpivid_job* job)
{
	rpivid_buffer* buffers[] = { job->bitstream, job->output, job->collocated };
	for (uint32 i = 0; i < B_COUNT_OF(buffers); i++) {
		if (buffers[i] != NULL)
			rpivid_buffer_release(buffers[i]);
	}
	for (uint32 i = 0; i < RPIVID_MAX_REFERENCES; i++) {
		if (job->references[i] != NULL)
			rpivid_buffer_release(job->references[i]);
	}
	free(job->entries);
	free(job);
}


static rpivid_buffer*
job_reference(rpivid_buffer* buffer)
{
	if (buffer != NULL)
		buffer->references++;
	return buffer;
}


/*!	The job is done: out of the queue and the phases, its scratch set free,
	its result there for RPIVID_WAIT.
*/
static void
job_finished(rpivid_device* device, rpivid_job* job, status_t result)
{
	rpivid_job* previous = NULL;
	for (rpivid_job* other = device->queue; other != NULL;
			previous = other, other = other->next) {
		if (other != job)
			continue;
		if (previous != NULL)
			previous->next = job->next;
		else
			device->queue = job->next;
		if (device->queue_tail == job)
			device->queue_tail = previous;
		break;
	}
	job->next = NULL;

	if (job->scratch >= 0)
		device->scratch[job->scratch].owner = NULL;
	job->scratch = -1;
	if (device->phase1 == job)
		device->phase1 = NULL;
	if (device->phase2 == job)
		device->phase2 = NULL;
	free(job->entries);
	job->entries = NULL;

	job->result = result;
	job->state = kJobDone;
	job->file->active_jobs--;
	if (result == B_OK)
		device->decoded++;
	else
		device->failed++;
	device->job_done.NotifyAll();
}


//	#pragma mark - the phases


/*!	A phase did not finish: the block is not used again. Its interrupts and
	its clock go off; the memory it was given stays allocated for good, as a
	block that stopped may still write it (the device memory too: it is not
	freed once the device is broken). Every job fails.
*/
static void
block_stopped(rpivid_device* device, int phase)
{
	ERROR("phase %d did not finish (CFSTATUS %" B_PRIu32 ", CFNUM %" B_PRIu32
		", STATUS %#" B_PRIx32 ", interrupt control %#" B_PRIx32 "): the block "
		"is not used again\n", phase, block_read(device, kCommandStatus),
		block_read(device, kCommandCount), block_read(device, kStatus),
		device->intc[0]);

	rpivid_job* running[] = { device->phase1, device->phase2 };
	for (uint32 i = 0; i < B_COUNT_OF(running); i++) {
		rpivid_job* job = running[i];
		if (job == NULL)
			continue;
		rpivid_buffer* buffers[] = { job->bitstream, job->output,
			job->collocated };
		for (uint32 j = 0; j < B_COUNT_OF(buffers); j++) {
			if (buffers[j] != NULL)
				buffers[j]->pinned = true;
		}
		for (uint32 j = 0; j < RPIVID_MAX_REFERENCES; j++)
			job->references[j]->pinned = true;
	}

	device->broken = true;
	rpivid_power_off(device);
	while (device->queue != NULL)
		job_finished(device, device->queue, B_IO_ERROR);
}


static void
kick_phase1(rpivid_device* device, rpivid_job* job)
{
	const rpivid_scratch& scratch = device->scratch[job->scratch];
	job->phase1_runs++;
	job->pu_stride = ROUNDDOWN(scratch.pu_size / job->ctb_rows, 64);
	job->coeff_stride = ROUNDDOWN(scratch.coeff_size / job->ctb_rows, 64);

	// the command buffer is write-combining: in memory before the kick
	arm64_dsb(sy);
	block_write(device, kPuWriteBase,
		rpivid_bus_address(device, scratch.pu.physical));
	block_write(device, kPuWriteStride, (job->pu_stride + 63) >> 6);
	block_write(device, kCoeffWriteBase,
		rpivid_bus_address(device, scratch.coeff.physical));
	block_write(device, kCoeffWriteStride, (job->coeff_stride + 63) >> 6);
	block_write(device, kCommandCount, job->command_count);
	clear_completion(device, kActive1Interrupt);
	block_write(device, kCommandBase,
		rpivid_bus_address(device, device->commands.physical));
}


/*!	Starts phase 1 of \a job in a free scratch set, sized for its picture.
	B_BUSY: no set is free (both pictures before it are not done yet).
*/
static status_t
start_phase1(rpivid_device* device, rpivid_job* job)
{
	int32 set = -1;
	for (uint32 i = 0; i < kScratchSets && set < 0; i++) {
		if (device->scratch[i].owner == NULL)
			set = i;
	}
	if (set < 0)
		return B_BUSY;
	rpivid_scratch& scratch = device->scratch[set];

	// Only phase 1 uses the command buffer, and only its job the set: both
	// may be reallocated here.
	const size_t pixels = (size_t)job->output->width * job->output->height;
	status_t status = ensure_memory(device, device->commands,
		max_c((size_t)job->command_count * 8, (size_t)64 * 1024),
		B_WRITE_COMBINING_MEMORY, "rpivid commands");
	if (status == B_OK && scratch.pu_size < round_up_size(pixels / 4)) {
		scratch.pu_size = 0;
		status = ensure_memory(device, scratch.pu, round_up_size(pixels / 4), 0,
			"rpivid pu");
		if (status == B_OK)
			scratch.pu_size = round_up_size(pixels / 4);
	}
	if (status == B_OK && scratch.coeff_size < round_up_size(pixels)) {
		scratch.coeff_size = 0;
		status = ensure_memory(device, scratch.coeff, round_up_size(pixels), 0,
			"rpivid coefficients");
		if (status == B_OK)
			scratch.coeff_size = round_up_size(pixels);
	}
	if (status != B_OK)
		return status;

	memcpy(device->commands.address, job->entries,
		(size_t)job->command_count * 8);
	free(job->entries);
	job->entries = NULL;

	scratch.owner = job;
	job->scratch = set;
	job->state = kJobPhase1;
	job->phase1_start = system_time();
	job->queue_time = job->phase1_start - job->submitted;
	device->phase1 = job;
	device->phase1_deadline = job->phase1_start + kPhase1Timeout;
	if (device->phase2 != NULL)
		device->overlaps++;
	kick_phase1(device, job);
	return B_OK;
}


/*!	Phase 1 of the running job ended: done, or again with a larger scratch
	buffer, or failed.
*/
static void
phase1_done(rpivid_device* device)
{
	rpivid_job* job = device->phase1;
	job->command_status = block_read(device, kCommandStatus);
	job->status = block_read(device, kStatus);
	if (job->command_status == block_read(device, kCommandCount)) {
		job->phase1_time = system_time() - job->phase1_start;
		job->state = kJobPhase1Done;
		device->phase1 = NULL;
		return;
	}

	const uint32 exhausted = job->status
		& (kStatusPuExhausted | kStatusCoeffExhausted);
	if (exhausted == 0 || job->phase1_runs >= kMaxPhase1Runs) {
		job->phase1_time = system_time() - job->phase1_start;
		job_finished(device, job, B_BAD_DATA);
		return;
	}

	rpivid_scratch& scratch = device->scratch[job->scratch];
	if ((exhausted & kStatusPuExhausted) != 0) {
		const size_t size = round_up_size(scratch.pu_size + 1);
		if (ensure_memory(device, scratch.pu, size, 0, "rpivid pu") != B_OK) {
			scratch.pu_size = 0;
			job_finished(device, job, B_NO_MEMORY);
			return;
		}
		scratch.pu_size = size;
	}
	if ((exhausted & kStatusCoeffExhausted) != 0) {
		const size_t size = round_up_size(scratch.coeff_size + 1);
		if (ensure_memory(device, scratch.coeff, size, 0,
				"rpivid coefficients") != B_OK) {
			scratch.coeff_size = 0;
			job_finished(device, job, B_NO_MEMORY);
			return;
		}
		scratch.coeff_size = size;
	}
	device->phase1_deadline = system_time() + kPhase1Timeout;
	kick_phase1(device, job);
}


static uint32
picture_address(rpivid_device* device, const rpivid_buffer* picture,
	size_t offset)
{
	return rpivid_bus_address(device, picture->memory.physical + offset);
}


static void
start_phase2(rpivid_device* device, rpivid_job* job)
{
	const rpivid_scratch& scratch = device->scratch[job->scratch];
	const rpivid_buffer* output = job->output;

	block_write(device, kPuReadBase,
		rpivid_bus_address(device, scratch.pu.physical));
	block_write(device, kPuReadStride, (job->pu_stride + 63) >> 6);
	block_write(device, kCoeffReadBase,
		rpivid_bus_address(device, scratch.coeff.physical));
	block_write(device, kCoeffReadStride, (job->coeff_stride + 63) >> 6);

	block_write(device, kOutLumaBase, picture_address(device, output, 0));
	block_write(device, kOutChromaBase,
		picture_address(device, output, output->chroma_offset));
	block_write(device, kOutLumaStride, output->luma_stride >> 6);
	block_write(device, kOutChromaStride, output->chroma_stride >> 6);
	for (uint32 i = 0; i < RPIVID_MAX_REFERENCES; i++) {
		const rpivid_buffer* reference = job->references[i];
		const uint32 slot = kReferences + i * kReferenceSize;
		block_write(device, slot + 0, picture_address(device, reference, 0));
		block_write(device, slot + 4, output->luma_stride >> 6);
		block_write(device, slot + 8, picture_address(device, reference,
			reference->chroma_offset));
		block_write(device, slot + 12, output->chroma_stride >> 6);
	}

	block_write(device, kConfig2, job->config2);
	block_write(device, kFrameSize, (output->height << 16) | output->width);
	block_write(device, kCurrentPoc, job->current_poc);

	const uint32 mvStride = job->write_mv || job->read_mv
		? output->mv_stride : 0;
	block_write(device, kColStride, (mvStride + 63) >> 6);
	block_write(device, kMvStride, (mvStride + 63) >> 6);
	block_write(device, kMvBase, job->write_mv
		? picture_address(device, output, output->mv_offset) : 0);
	uint32 colBase = 0;
	if (job->read_mv) {
		colBase = job->collocated != NULL
			? picture_address(device, job->collocated,
				job->collocated->mv_offset)
			: rpivid_bus_address(device, device->zero_mv.physical);
	}
	block_write(device, kColBase, colBase);

	job->state = kJobPhase2;
	job->phase2_start = system_time();
	device->phase2 = job;
	device->phase2_deadline = job->phase2_start + kPhase2Timeout;
	if (device->phase1 != NULL)
		device->overlaps++;

	clear_completion(device, kActive2Interrupt);
	block_write(device, kNumRows, job->ctb_rows);
}


/*!	One step of the pipeline: completions, timeouts, then what can start -
	phase 2 of the oldest picture whose phase 1 is done, phase 1 of the oldest
	queued one.
*/
static void
advance(rpivid_device* device)
{
	if (device->broken || !device->powered) {
		while (device->queue != NULL)
			job_finished(device, device->queue, B_DEVICE_NOT_FOUND);
		return;
	}

	if (device->phase1 != NULL && take_completion(device, kActive1Interrupt)) {
		// the block is a non-coherent bus master: its writes are complete
		// before the status registers are read
		arm64_dsb(sy);
		phase1_done(device);
	}
	if (device->phase2 != NULL && take_completion(device, kActive2Interrupt)) {
		arm64_dsb(sy);
		rpivid_job* job = device->phase2;
		job->phase2_time = system_time() - job->phase2_start;
		job_finished(device, job, B_OK);
	}

	const bigtime_t now = system_time();
	if (device->phase1 != NULL && now >= device->phase1_deadline) {
		block_stopped(device, 1);
		return;
	}
	if (device->phase2 != NULL && now >= device->phase2_deadline) {
		block_stopped(device, 2);
		return;
	}

	// the queue holds, in order: the job in phase 2 or waiting for it, the
	// job in phase 1, the queued ones
	if (device->phase2 == NULL && device->queue != NULL
		&& device->queue->state == kJobPhase1Done) {
		start_phase2(device, device->queue);
	}
	while (device->phase1 == NULL) {
		rpivid_job* job = device->queue;
		while (job != NULL && job->state != kJobQueued)
			job = job->next;
		if (job == NULL)
			break;
		const status_t status = start_phase1(device, job);
		if (status == B_BUSY)
			break;
		if (status != B_OK)
			job_finished(device, job, status);
	}
}


/*!	Runs the pipeline: woken by the interrupt handler and by RPIVID_SUBMIT,
	and every kPollInterval while a phase runs.
*/
status_t
rpivid_decode_thread(void* data)
{
	rpivid_device* device = (rpivid_device*)data;

	MutexLocker locker(device->lock);
	while (!device->quit) {
		advance(device);

		uint32 flags = 0;
		bigtime_t timeout = B_INFINITE_TIMEOUT;
		if (device->phase1 != NULL || device->phase2 != NULL) {
			flags = B_RELATIVE_TIMEOUT;
			timeout = kPollInterval;
		}
		locker.Unlock();
		acquire_sem_etc(device->irq_sem, 1, flags, timeout);
		locker.Lock();
	}
	return B_OK;
}


//	#pragma mark - ioctls


/*!	Checks a picture and queues it. Takes \a entries on success. */
static status_t
submit(rpivid_file* file, rpivid_decode* args, uint64* entries)
{
	rpivid_device* device = file->device;
	if (device->broken || !device->powered)
		return B_DEVICE_NOT_FOUND;
	if (file->job_count >= RPIVID_MAX_JOBS)
		return B_BUSY;

	// the buffers, all the client's
	rpivid_buffer* bitstream = rpivid_buffer_lookup(file, args->bitstream,
		kBitstreamBuffer);
	rpivid_buffer* output = rpivid_buffer_lookup(file, args->output,
		kPictureBuffer);
	if (bitstream == NULL || output == NULL)
		return B_BAD_VALUE;

	rpivid_buffer* references[RPIVID_MAX_REFERENCES];
	for (uint32 i = 0; i < RPIVID_MAX_REFERENCES; i++) {
		references[i] = args->references[i] == 0 ? output
			: rpivid_buffer_lookup(file, args->references[i], kPictureBuffer);
		if (references[i] == NULL || !same_geometry(references[i], output))
			return B_BAD_VALUE;
	}

	// CONFIG2: bit depths and CTB size as the pictures, motion vectors only
	// with buffers for them
	const uint32 bitDepthFlags = output->bit_depth != 8 ? (3 << 8) : 0;
	if ((args->config2 & kConfig2BitDepths)
			!= (output->bit_depth | (output->bit_depth << 4) | bitDepthFlags)
		|| (args->config2 & kConfig2Log2CtbMask)
			!= (args->log2_ctb_size << kConfig2Log2CtbShift)) {
		return B_BAD_VALUE;
	}
	const bool writeMv = (args->config2 & kConfig2WriteMv) != 0;
	const bool readMv = (args->config2 & kConfig2ReadMv) != 0;
	if (writeMv && (output->flags & RPIVID_PICTURE_MOTION_VECTORS) == 0)
		return B_BAD_VALUE;
	rpivid_buffer* collocated = NULL;
	if (args->collocated != 0) {
		collocated = rpivid_buffer_lookup(file, args->collocated,
			kPictureBuffer);
		if (collocated == NULL || !same_geometry(collocated, output)
			|| (collocated->flags & RPIVID_PICTURE_MOTION_VECTORS) == 0) {
			return B_BAD_VALUE;
		}
	}

	status_t status = check_commands(device, entries, args->command_count,
		bitstream, output, args->log2_ctb_size);
	if (status != B_OK)
		return status;

	if (readMv && collocated == NULL && device->zero_mv.area < 0) {
		// a collocated picture the client does not have: zero motion
		status = rpivid_dma_alloc(device, "rpivid zero motion vectors",
			kZeroMvSize, 0, device->zero_mv);
		if (status != B_OK)
			return status;
	}

	rpivid_job* job = (rpivid_job*)calloc(1, sizeof(rpivid_job));
	if (job == NULL)
		return B_NO_MEMORY;
	job->file = file;
	job->id = ++file->next_job;
	job->state = kJobQueued;
	job->entries = entries;
	job->command_count = args->command_count;
	job->bitstream = job_reference(bitstream);
	job->output = job_reference(output);
	job->collocated = job_reference(collocated);
	for (uint32 i = 0; i < RPIVID_MAX_REFERENCES; i++)
		job->references[i] = job_reference(references[i]);
	job->config2 = args->config2;
	job->current_poc = args->current_poc;
	const uint32 ctbSize = 1 << args->log2_ctb_size;
	job->ctb_rows = (output->height + ctbSize - 1) / ctbSize;
	job->write_mv = writeMv;
	job->read_mv = readMv;
	job->scratch = -1;
	job->submitted = system_time();

	if (device->queue_tail != NULL)
		device->queue_tail->next = job;
	else
		device->queue = job;
	device->queue_tail = job;
	job->file_next = file->jobs;
	file->jobs = job;
	file->job_count++;
	file->active_jobs++;

	args->job = job->id;
	return B_OK;
}


status_t
rpivid_submit_ioctl(rpivid_file* file, rpivid_decode* args)
{
	rpivid_device* device = file->device;
	args->job = 0;
	args->status = args->command_status = args->phase1_runs = 0;
	args->queue_time = args->phase1_time = args->phase2_time = 0;
	if (args->command_count == 0 || args->command_count > kMaxCommands
		|| args->log2_ctb_size < 4 || args->log2_ctb_size > 6) {
		return B_BAD_VALUE;
	}

	// copied before taking the lock: reading the client's memory may fault
	const size_t size = (size_t)args->command_count * sizeof(uint64);
	uint64* entries = (uint64*)malloc(size);
	if (entries == NULL)
		return B_NO_MEMORY;
	if (!IS_USER_ADDRESS(args->commands)
		|| user_memcpy(entries, (const void*)(addr_t)args->commands, size)
			!= B_OK) {
		free(entries);
		return B_BAD_ADDRESS;
	}

	MutexLocker locker(device->lock);
	status_t status = submit(file, args, entries);
	if (status != B_OK) {
		device->failed++;
		free(entries);
	}
	locker.Unlock();

	// after unlocking: the decode thread runs at once, and finds the lock free
	if (status == B_OK)
		release_sem(device->irq_sem);
	return status;
}


static rpivid_job*
find_job(rpivid_file* file, uint64 id, rpivid_job*** _link)
{
	for (rpivid_job** link = &file->jobs; *link != NULL;
			link = &(*link)->file_next) {
		if ((*link)->id == id) {
			*_link = link;
			return *link;
		}
	}
	return NULL;
}


/*!	Waits for a job, returns its result and forgets it. Not interruptible:
	every job is done within the phase timeouts.
*/
status_t
rpivid_wait_ioctl(rpivid_file* file, rpivid_wait* args)
{
	rpivid_device* device = file->device;
	const bigtime_t deadline = args->timeout > 0
		? system_time() + args->timeout : 0;

	MutexLocker locker(device->lock);
	rpivid_job** link;
	rpivid_job* job;
	for (;;) {
		// looked up again after each wait: another thread may have taken it
		job = find_job(file, args->job, &link);
		if (job == NULL)
			return B_BAD_VALUE;
		if (job->state == kJobDone)
			break;
		if (args->timeout == 0)
			return B_WOULD_BLOCK;
		status_t status = device->job_done.Wait(&device->lock,
			args->timeout > 0 ? B_ABSOLUTE_TIMEOUT : 0, deadline);
		if (status == B_TIMED_OUT) {
			job = find_job(file, args->job, &link);
			if (job == NULL || job->state != kJobDone)
				return B_TIMED_OUT;
			break;
		}
	}

	args->result = job->result;
	args->status = job->status;
	args->command_status = job->command_status;
	args->phase1_runs = job->phase1_runs;
	args->queue_time = job->queue_time;
	args->phase1_time = job->phase1_time;
	args->phase2_time = job->phase2_time;

	*link = job->file_next;
	file->job_count--;
	rpivid_buffer* output = NULL;
	if (job->result == B_OK
		&& (job->output->flags & RPIVID_PICTURE_UNCACHED) == 0) {
		output = job_reference(job->output);
	}
	job_free(job);
	locker.Unlock();

	if (output != NULL) {
		// Drop what the CPU has cached of the picture (clean lines fetched
		// before or, speculatively, during the decode): the client reads it
		// next. In the client's time, and without the lock: the block goes on.
		arm64_dcache_flush_range((addr_t)output->memory.address,
			output->planes_size);
		locker.Lock();
		rpivid_buffer_release(output);
	}
	return B_OK;
}


status_t
rpivid_decode_ioctl(rpivid_file* file, rpivid_decode* args)
{
	status_t status = rpivid_submit_ioctl(file, args);
	if (status != B_OK)
		return status;

	rpivid_wait wait = {};
	wait.job = args->job;
	wait.timeout = -1;
	status = rpivid_wait_ioctl(file, &wait);
	if (status != B_OK)
		return status;
	args->status = wait.status;
	args->command_status = wait.command_status;
	args->phase1_runs = wait.phase1_runs;
	args->queue_time = wait.queue_time;
	args->phase1_time = wait.phase1_time;
	args->phase2_time = wait.phase2_time;
	return wait.result;
}


/*!	The file is going: waits for its jobs to finish (the block may still
	use its buffers) and forgets them. Called with the device lock held.
*/
void
rpivid_file_jobs_free(rpivid_file* file)
{
	rpivid_device* device = file->device;
	while (file->active_jobs > 0)
		device->job_done.Wait(&device->lock);

	while (file->jobs != NULL) {
		rpivid_job* job = file->jobs;
		file->jobs = job->file_next;
		job_free(job);
	}
	file->job_count = 0;
}
