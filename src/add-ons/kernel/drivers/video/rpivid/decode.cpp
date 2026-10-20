/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Decoding one picture.

	Phase 1 (entropy decode) executes the client's command buffer and writes
	prediction units and coefficients into two scratch buffers the driver
	owns, a stride per CTB row; when one is too small, the block stops with
	an "exhausted" status bit and phase 1 runs again with a larger one.
	Phase 2 (reconstruction) is programmed with direct register writes: the
	scratch buffers, the output and reference pictures, the collocated motion
	vectors. Each phase ends with an interrupt from the block's interrupt
	controller.

	The command entries are checked before the block sees them: they may
	only set phase-1 parameters, the bitstream addresses in them are offsets
	into the client's bitstream buffer (rebased here), and the bit depth and
	CTB size they program must be the pictures' - so the block reads and
	writes only memory the client was given.
*/


#include "rpivid_private.h"

#include <arch/cpu.h>
#include <util/AutoLock.h>


// A picture's command buffer beyond this is not a picture
static const uint32 kMaxCommands = 1 << 18;
static const uint32 kMaxPhase1Runs = 5;
static const bigtime_t kPhase1Timeout = 1000000;
static const bigtime_t kPhase2Timeout = 2000000;
// A wait re-checks the interrupt controller this often, so a lost
// interrupt costs this, not the picture.
static const bigtime_t kPollInterval = 10000;


//	#pragma mark - completion


/*!	The block's interrupt line is level triggered: the completion bits are
	acknowledged here, and recorded for the waiter.
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


/*!	A completion can be seen twice, by the handler and by a waiter that
	polls: the handler's mark may land after the waiter took the completion.
	Left there, it would end the next wait for that phase at once, before
	the block finished. So before a phase starts, a pending completion of it
	is forgotten.
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


static status_t
wait_for_completion(rpivid_device* device, uint32 bit, bigtime_t timeout)
{
	const bigtime_t deadline = system_time() + timeout;
	while (!take_completion(device, bit)) {
		const bigtime_t now = system_time();
		if (now >= deadline)
			return B_TIMED_OUT;
		acquire_sem_etc(device->irq_sem, 1, B_RELATIVE_TIMEOUT,
			min_c(deadline - now, kPollInterval));
	}

	// The block is a non-coherent bus master: its writes are complete before
	// anything reads the output.
	arm64_dsb(sy);
	return B_OK;
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
	rpivid_dma_free(device->pu);
	rpivid_dma_free(device->coeff);
	rpivid_dma_free(device->zero_mv);
	device->pu_size = device->coeff_size = 0;
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


//	#pragma mark - decode


/*!	A phase did not finish: the block is not used again. Its interrupts and
	its clock go off; the memory it was given stays allocated for good, as a
	block that stopped may still write it (the device memory too: it is not
	freed once the device is broken).
*/
static void
block_stopped(rpivid_device* device, int phase, rpivid_buffer** buffers,
	uint32 bufferCount)
{
	ERROR("phase %d did not finish (CFSTATUS %" B_PRIu32 ", CFNUM %" B_PRIu32
		", STATUS %#" B_PRIx32 ", interrupt control %#" B_PRIx32 "): the block "
		"is not used again\n", phase, block_read(device, kCommandStatus),
		block_read(device, kCommandCount), block_read(device, kStatus),
		device->intc[0]);

	for (uint32 i = 0; i < bufferCount; i++) {
		if (buffers[i] != NULL)
			buffers[i]->pinned = true;
	}
	device->broken = true;
	rpivid_power_off(device);
}


/*!	Phase 1, run again with a larger scratch buffer when one ran out. */
static status_t
run_phase1(rpivid_device* device, rpivid_decode* args, uint32 ctbRows,
	uint32& puStride, uint32& coeffStride)
{
	for (uint32 run = 1;; run++) {
		args->phase1_runs = run;
		puStride = ROUNDDOWN(device->pu_size / ctbRows, 64);
		coeffStride = ROUNDDOWN(device->coeff_size / ctbRows, 64);

		// the command buffer is write-combining: in memory before the kick
		arm64_dsb(sy);
		block_write(device, kPuWriteBase,
			rpivid_bus_address(device, device->pu.physical));
		block_write(device, kPuWriteStride, (puStride + 63) >> 6);
		block_write(device, kCoeffWriteBase,
			rpivid_bus_address(device, device->coeff.physical));
		block_write(device, kCoeffWriteStride, (coeffStride + 63) >> 6);
		block_write(device, kCommandCount, args->command_count);
		clear_completion(device, kActive1Interrupt);
		block_write(device, kCommandBase,
			rpivid_bus_address(device, device->commands.physical));

		status_t status = wait_for_completion(device, kActive1Interrupt,
			kPhase1Timeout);
		if (status != B_OK)
			return status;

		args->command_status = block_read(device, kCommandStatus);
		args->status = block_read(device, kStatus);
		if (args->command_status == block_read(device, kCommandCount))
			return B_OK;

		const uint32 exhausted = args->status
			& (kStatusPuExhausted | kStatusCoeffExhausted);
		if (exhausted == 0 || run >= kMaxPhase1Runs)
			return B_BAD_DATA;

		if ((exhausted & kStatusPuExhausted) != 0) {
			const size_t size = round_up_size(device->pu_size + 1);
			if (ensure_memory(device, device->pu, size, 0, "rpivid pu")
					!= B_OK) {
				device->pu_size = 0;
				return B_NO_MEMORY;
			}
			device->pu_size = size;
		}
		if ((exhausted & kStatusCoeffExhausted) != 0) {
			const size_t size = round_up_size(device->coeff_size + 1);
			if (ensure_memory(device, device->coeff, size, 0,
					"rpivid coefficients") != B_OK) {
				device->coeff_size = 0;
				return B_NO_MEMORY;
			}
			device->coeff_size = size;
		}
	}
}


static uint32
picture_address(rpivid_device* device, const rpivid_buffer* picture,
	size_t offset)
{
	return rpivid_bus_address(device, picture->memory.physical + offset);
}


static status_t
decode(rpivid_file* file, rpivid_decode* args, uint64* entries)
{
	rpivid_device* device = file->device;
	if (device->broken || !device->powered)
		return B_DEVICE_NOT_FOUND;

	// the buffers, all the client's
	rpivid_buffer* buffers[3 + RPIVID_MAX_REFERENCES] = {};
	rpivid_buffer* bitstream = rpivid_buffer_lookup(file, args->bitstream,
		kBitstreamBuffer);
	rpivid_buffer* output = rpivid_buffer_lookup(file, args->output,
		kPictureBuffer);
	if (bitstream == NULL || output == NULL)
		return B_BAD_VALUE;
	buffers[0] = bitstream;
	buffers[1] = output;

	rpivid_buffer* references[RPIVID_MAX_REFERENCES];
	for (uint32 i = 0; i < RPIVID_MAX_REFERENCES; i++) {
		references[i] = args->references[i] == 0 ? output
			: rpivid_buffer_lookup(file, args->references[i], kPictureBuffer);
		if (references[i] == NULL || !same_geometry(references[i], output))
			return B_BAD_VALUE;
		buffers[3 + i] = references[i];
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
		buffers[2] = collocated;
	}

	status_t status = check_commands(device, entries, args->command_count,
		bitstream, output, args->log2_ctb_size);
	if (status != B_OK)
		return status;

	// device memory
	const size_t pixels = (size_t)output->width * output->height;
	status = ensure_memory(device, device->commands,
		max_c((size_t)args->command_count * 8, (size_t)64 * 1024),
		B_WRITE_COMBINING_MEMORY, "rpivid commands");
	if (status == B_OK && device->pu_size < round_up_size(pixels / 4)) {
		device->pu_size = 0;
		status = ensure_memory(device, device->pu, round_up_size(pixels / 4), 0,
			"rpivid pu");
		if (status == B_OK)
			device->pu_size = round_up_size(pixels / 4);
	}
	if (status == B_OK && device->coeff_size < round_up_size(pixels)) {
		device->coeff_size = 0;
		status = ensure_memory(device, device->coeff, round_up_size(pixels), 0,
			"rpivid coefficients");
		if (status == B_OK)
			device->coeff_size = round_up_size(pixels);
	}
	if (status == B_OK && readMv && collocated == NULL) {
		// a collocated picture the client does not have: zero motion
		status = ensure_memory(device, device->zero_mv,
			(size_t)ROUNDUP(output->width, 64) * (ROUNDUP(output->height, 64)
				/ 16), 0, "rpivid zero motion vectors");
	}
	if (status != B_OK)
		return status;
	memcpy(device->commands.address, entries,
		(size_t)args->command_count * 8);

	const uint32 ctbSize = 1 << args->log2_ctb_size;
	const uint32 ctbRows = (output->height + ctbSize - 1) / ctbSize;

	// phase 1
	bigtime_t start = system_time();
	uint32 puStride, coeffStride;
	status = run_phase1(device, args, ctbRows, puStride, coeffStride);
	bigtime_t end = system_time();
	args->phase1_time = end - start;
	if (status == B_TIMED_OUT) {
		block_stopped(device, 1, buffers, B_COUNT_OF(buffers));
		return B_IO_ERROR;
	}
	if (status != B_OK)
		return status;

	// phase 2
	start = end;
	block_write(device, kPuReadBase,
		rpivid_bus_address(device, device->pu.physical));
	block_write(device, kPuReadStride, (puStride + 63) >> 6);
	block_write(device, kCoeffReadBase,
		rpivid_bus_address(device, device->coeff.physical));
	block_write(device, kCoeffReadStride, (coeffStride + 63) >> 6);

	block_write(device, kOutLumaBase, picture_address(device, output, 0));
	block_write(device, kOutChromaBase,
		picture_address(device, output, output->chroma_offset));
	block_write(device, kOutLumaStride, output->luma_stride >> 6);
	block_write(device, kOutChromaStride, output->chroma_stride >> 6);
	for (uint32 i = 0; i < RPIVID_MAX_REFERENCES; i++) {
		const uint32 slot = kReferences + i * kReferenceSize;
		block_write(device, slot + 0, picture_address(device, references[i], 0));
		block_write(device, slot + 4, output->luma_stride >> 6);
		block_write(device, slot + 8, picture_address(device, references[i],
			references[i]->chroma_offset));
		block_write(device, slot + 12, output->chroma_stride >> 6);
	}

	block_write(device, kConfig2, args->config2);
	block_write(device, kFrameSize, (output->height << 16) | output->width);
	block_write(device, kCurrentPoc, args->current_poc);

	const uint32 mvStride = writeMv || readMv ? output->mv_stride : 0;
	block_write(device, kColStride, (mvStride + 63) >> 6);
	block_write(device, kMvStride, (mvStride + 63) >> 6);
	block_write(device, kMvBase,
		writeMv ? picture_address(device, output, output->mv_offset) : 0);
	uint32 colBase = 0;
	if (readMv) {
		colBase = collocated != NULL
			? picture_address(device, collocated, collocated->mv_offset)
			: rpivid_bus_address(device, device->zero_mv.physical);
	}
	block_write(device, kColBase, colBase);

	clear_completion(device, kActive2Interrupt);
	block_write(device, kNumRows, ctbRows);
	status = wait_for_completion(device, kActive2Interrupt, kPhase2Timeout);
	args->phase2_time = system_time() - start;
	if (status != B_OK) {
		block_stopped(device, 2, buffers, B_COUNT_OF(buffers));
		return B_IO_ERROR;
	}

	// Drop what the CPU has cached of the picture (clean lines fetched before
	// or, speculatively, during the decode): the client reads it next.
	if ((output->flags & RPIVID_PICTURE_UNCACHED) == 0) {
		arm64_dcache_flush_range((addr_t)output->memory.address,
			output->planes_size);
	}
	return B_OK;
}


status_t
rpivid_decode_ioctl(rpivid_file* file, rpivid_decode* args)
{
	rpivid_device* device = file->device;
	args->status = args->command_status = args->phase1_runs = 0;
	args->phase1_time = args->phase2_time = 0;
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
	status_t status = decode(file, args, entries);
	if (status == B_OK)
		device->decoded++;
	else
		device->failed++;
	locker.Unlock();

	free(entries);
	return status;
}
