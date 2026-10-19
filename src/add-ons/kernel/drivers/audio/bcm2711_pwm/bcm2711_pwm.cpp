/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The Raspberry Pi 4's 3.5 mm headphone jack, as a multi_audio device.

	The jack is driven by the second PWM block (PWM1) on GPIO 40 and 41: both
	channels run in mark/space mode from the PWM FIFO, which alternates between
	them, so interleaved stereo needs no reordering. Each sample becomes one
	FIFO word, its duty cycle (0 to kRange). A legacy DMA channel, paced by the
	PWM's DREQ, feeds the FIFO from a ring of such words in two segments, one
	per multi_audio buffer; the control blocks chain the segments into a loop
	and interrupt at the end of each. buffer_exchange() converts the PCM buffer
	the media add-on has just filled into its segment.

	The engine never stops while the device is open: a segment that has been
	played is overwritten with silence by the interrupt handler, so when the
	client stops delivering, the jack idles at mid-scale instead of repeating
	the last buffers. Stopping the PWM would drop the output to a rail, which
	is a click; the output is ramped up and down instead.
*/


#include <stdlib.h>
#include <string.h>

#include <Drivers.h>
#include <KernelExport.h>
#include <arch/cpu.h>
#include <kernel.h>
#include <lock.h>
#include <util/AutoLock.h>
#include <vm/vm.h>

// after the kernel's ASSERT, or it pulls in the userland <Debug.h>
#include <hmulti_audio.h>

extern "C" {
#include <libfdt.h>
}


//#define TRACE_BCM2711_PWM
#ifdef TRACE_BCM2711_PWM
#	define TRACE(x...)		dprintf("bcm2711_pwm: " x)
#else
#	define TRACE(x...)
#endif
#define TRACE_ALWAYS(x...)	dprintf("bcm2711_pwm: " x)
#define ERROR(x...)			dprintf("bcm2711_pwm: " x)


#define DEVICE_NAME "audio/hmulti/bcm2711_pwm/1"


extern void* gFDT;


int32 api_version = B_CUR_DRIVER_API_VERSION;


// PWM registers (BCM2711 ARM Peripherals, chapter 8), as word indices
enum {
	PWM_CTL		= 0x00 / 4,
	PWM_STA		= 0x04 / 4,
	PWM_DMAC	= 0x08 / 4,
	PWM_RNG1	= 0x10 / 4,
	PWM_FIF1	= 0x18 / 4,
	PWM_RNG2	= 0x20 / 4,
};
static const uint32 kPwmEnable1 = 1 << 0;
static const uint32 kPwmUseFifo1 = 1 << 5;
static const uint32 kPwmClearFifo = 1 << 6;
static const uint32 kPwmMarkSpace1 = 1 << 7;
static const uint32 kPwmEnable2 = 1 << 8;
static const uint32 kPwmUseFifo2 = 1 << 13;
static const uint32 kPwmMarkSpace2 = 1 << 15;
static const uint32 kPwmChannels = kPwmEnable1 | kPwmUseFifo1 | kPwmMarkSpace1
	| kPwmEnable2 | kPwmUseFifo2 | kPwmMarkSpace2;
// PWM_STA: the sticky error bits (write 1 to clear), bus error included
static const uint32 kPwmStatusErrors = 0x1fc;
// PWM_DMAC: enable, panic threshold 8 and DREQ threshold 4 (as the
// Phoenix-RTOS driver that was proven on this board)
static const uint32 kPwmDmaControl = (1u << 31) | (8 << 8) | 4;

// The bus address of PWM1, which drives the jack; PWM0 is compatible with it
static const uint64 kPwm1BusAddress = 0x7e20c800;

// The PWM clock in the clock manager (CPRMAN), as word indices
enum {
	CM_PWMCTL	= 0xa0 / 4,
	CM_PWMDIV	= 0xa4 / 4,
};
static const uint32 kClockPassword = 0x5a000000;
static const uint32 kClockPasswordMask = 0xff000000;
static const uint32 kClockSourceMask = 0xf;
static const uint32 kClockSourceOscillator = 1;
static const uint32 kClockEnable = 1 << 4;
// Linux sets this "gate" bit with every enable; it does not read back here
static const uint32 kClockGate = 1 << 6;
static const uint32 kClockBusy = 1 << 7;
static const uint32 kClockMash = 3 << 9;

// GPIO: function select of pins 40 to 49, 3 bits each; ALT0 is PWM1
static const uint32 kGpioFunctionSelect4 = 0x10 / 4;
static const uint32 kGpioAlt0 = 4;

// Legacy DMA channel registers, as word indices
enum {
	DMA_CS			= 0x00 / 4,
	DMA_CONBLK_AD	= 0x04 / 4,
	DMA_SOURCE_AD	= 0x0c / 4,
	DMA_DEST_AD		= 0x10 / 4,
	DMA_TXFR_LEN	= 0x14 / 4,
	DMA_DEBUG		= 0x20 / 4,
};
static const uint32 kDmaActive = 1 << 0;
static const uint32 kDmaInterrupt = 1 << 2;
static const uint32 kDmaError = 1 << 8;
static const uint32 kDmaReset = 1u << 31;
static const uint32 kDmaDebugErrors = 7;
	// read last not set, FIFO and read error; write 1 to clear
// Transfer information of a control block
static const uint32 kDmaInterruptEnable = 1 << 0;
static const uint32 kDmaWaitResponse = 1 << 3;
static const uint32 kDmaDestinationDreq = 1 << 6;
static const uint32 kDmaSourceIncrement = 1 << 8;
static const uint32 kDmaPermapShift = 16;
static const uint32 kDmaDreqPwm1 = 1;
	// on the BCM2711, DREQ 1 is PWM1 (5 is PWM0)

// Channel 5 is free for the ARM in the firmware's channel mask, has an
// interrupt of its own, and reaches the low 1 GiB like all legacy channels.
static const uint32 kDmaChannel = 5;
static const phys_addr_t kDmaMemoryLimit = (phys_addr_t)1 << 30;

// The sample rate: the 54 MHz oscillator divided by 2 is a 27 MHz PWM clock,
// and one FIFO word lasts kRange clocks: 27 MHz / 612 = 44117.647 Hz, 0.04%
// above the 44100 Hz that is advertised (multi_audio has no closer rate, and
// the media add-on does not take a variable one). played_frames_count comes
// from the hardware, so the time source follows the true rate.
static const uint32 kOscillatorRate = 54000000;
static const uint32 kClockDivisor = 2;
static const uint32 kRange = 612;
static const uint32 kFrameRateMilliHertz
	= (uint64)kOscillatorRate * 1000 / (kClockDivisor * kRange);

// PCM 0 maps to the middle of the range; that is also the level the ring
// idles at, so starting and stopping a stream does not move the output.
static const uint32 kDutySilence = kRange / 2;
static_assert((32768 * kRange) / 65536 == kDutySilence,
	"PCM 0 must convert to the idle duty, or every stop clicks");

static const uint32 kChannelCount = 2;
static const uint32 kBufferCount = 2;
static const uint32 kFramesPerBuffer = 1024;
static const uint32 kSegmentWords = kFramesPerBuffer * kChannelCount;
static const uint32 kSegmentBytes = kSegmentWords * sizeof(uint32);
static const bigtime_t kPeriod = (bigtime_t)kFramesPerBuffer * 1000000
	* kClockDivisor * kRange / kOscillatorRate;
	// ~23.2 ms

// A buffer exchange that waits this long has a stalled engine; it restarts it
// (and says so, the first few times).
static const bigtime_t kExchangeTimeout = 10 * kPeriod;
static const uint32 kTimeoutsLogged = 4;
// Only a channel that moves the read cursor is streaming: one that is active
// but parked waits for a DREQ that does not come. 20 ms are ~1760 words.
static const bigtime_t kStartSettleTime = 20000;
static const uint32 kStartMinimumWords = 64;
static const uint32 kStartAttempts = 3;
// The measured frame rate is logged once, after this many periods (~3 s).
static const uint32 kRateReportPeriods = 128;


struct dma_control_block {
	uint32	transfer_info;
	uint32	source;
	uint32	destination;
	uint32	length;
	uint32	stride;
	uint32	next;
	uint32	reserved[2];
} _PACKED;

static_assert(sizeof(dma_control_block) == 32, "control blocks are 32 bytes");


struct pwm_audio {
	mutex				lock;
		// open count, engine start and stop
	int32				open_count;

	area_id				pwm_area;
	area_id				clock_area;
	area_id				gpio_area;
	area_id				dma_area;
	volatile uint32*	pwm;
	volatile uint32*	clock;
	volatile uint32*	gpio;
	volatile uint32*	dma;
	uint32				fifo_bus_address;
	int32				interrupt;

	// what the media add-on writes: kBufferCount interleaved 16 bit buffers
	area_id				pcm_area;
	int16*				pcm;

	// what the DMA reads, write-combining: the ring of duty words in
	// kBufferCount segments, then one control block per segment
	area_id				ring_area;
	volatile uint32*	ring;
	volatile dma_control_block* control_blocks;
	uint32				ring_bus_address;
	uint32				control_blocks_bus_address;

	bool				running;
	bool				failed;
		// the engine would not start; not retried before the next open
	sem_id				period_sem;
	int32				fill_cycle;
		// the buffer the client fills, converted on its next exchange

	// shared with the interrupt handler
	spinlock			state_lock;
	int32				playing;
	bool				filled[kBufferCount];
	bool				primed;
	uint32				idle_duty;
	bigtime_t			real_time;
	int64				frames_count;
	bigtime_t			first_period_time;
	uint32				periods;
	uint32				underruns;
	uint32				dma_errors;

	uint32				timeouts;
	bool				rate_reported;
};

static pwm_audio sDevice;


//	#pragma mark - device tree


static uint64
read_cells(const uint32* cells, uint32 count)
{
	uint64 value = 0;
	for (uint32 i = 0; i < count; i++)
		value = (value << 32) | fdt32_to_cpu(cells[i]);
	return value;
}


static uint32
address_cells(int node)
{
	const uint32* prop = (const uint32*)fdt_getprop(gFDT, node,
		"#address-cells", NULL);
	return prop != NULL ? fdt32_to_cpu(*prop) : 2;
}


static uint32
size_cells(int node)
{
	const uint32* prop = (const uint32*)fdt_getprop(gFDT, node, "#size-cells",
		NULL);
	return prop != NULL ? fdt32_to_cpu(*prop) : 1;
}


/*!	Applies the first matching entry of the "ranges" (or "dma-ranges") of
	\a bus to \a address, in the direction given by \a toParent.
*/
static bool
apply_ranges(int bus, const char* property, bool toParent, uint64& address)
{
	int length;
	const uint32* ranges = (const uint32*)fdt_getprop(gFDT, bus, property,
		&length);
	if (ranges == NULL || length == 0)
		return true;
			// no translation

	uint32 childCells = address_cells(bus);
	uint32 parentCells = address_cells(fdt_parent_offset(gFDT, bus));
	uint32 sizeCells = size_cells(bus);
	uint32 entryCells = childCells + parentCells + sizeCells;
	for (uint32 i = 0; (i + 1) * entryCells * 4 <= (uint32)length; i++) {
		const uint32* entry = ranges + i * entryCells;
		uint64 child = read_cells(entry, childCells);
		uint64 parent = read_cells(entry + childCells, parentCells);
		uint64 size = read_cells(entry + childCells + parentCells, sizeCells);
		uint64 from = toParent ? child : parent;
		uint64 to = toParent ? parent : child;
		if (address >= from && address - from < size) {
			address = address - from + to;
			return true;
		}
	}
	return false;
}


/*!	The first register block of \a node: its address on the node's bus (what
	the DMA engine uses) and the CPU's physical address of it.
*/
static status_t
node_registers(int node, uint64& busAddress, phys_addr_t& physical)
{
	int bus = fdt_parent_offset(gFDT, node);
	const uint32* reg = (const uint32*)fdt_getprop(gFDT, node, "reg", NULL);
	if (bus < 0 || reg == NULL)
		return B_BAD_DATA;

	busAddress = read_cells(reg, address_cells(bus));
	uint64 address = busAddress;
	for (int parent = bus; fdt_parent_offset(gFDT, parent) >= 0;
			parent = fdt_parent_offset(gFDT, parent)) {
		if (!apply_ranges(parent, "ranges", true, address))
			return B_BAD_DATA;
	}
	physical = address;
	return B_OK;
}


static int
find_compatible(const char* compatible, uint64 busAddress = 0)
{
	for (int node = fdt_node_offset_by_compatible(gFDT, -1, compatible);
			node >= 0;
			node = fdt_node_offset_by_compatible(gFDT, node, compatible)) {
		uint64 address;
		phys_addr_t physical;
		if (busAddress == 0 || (node_registers(node, address, physical) == B_OK
				&& address == busAddress)) {
			return node;
		}
	}
	return -1;
}


/*!	Maps the page holding the registers of \a node, at \a offset into them. */
static status_t
map_registers(int node, uint32 offset, const char* name, area_id& area,
	volatile uint32*& registers, uint64* _busAddress = NULL)
{
	uint64 busAddress;
	phys_addr_t physical;
	if (node < 0 || node_registers(node, busAddress, physical) != B_OK)
		return B_DEVICE_NOT_FOUND;
	physical += offset;

	void* address;
	area = map_physical_memory(name, physical & ~(phys_addr_t)(B_PAGE_SIZE - 1),
		B_PAGE_SIZE, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &address);
	if (area < 0)
		return area;

	registers = (volatile uint32*)((addr_t)address
		+ (physical & (B_PAGE_SIZE - 1)));
	if (_busAddress != NULL)
		*_busAddress = busAddress + offset;
	return B_OK;
}


/*!	The DMA channel's interrupt, and whether the firmware leaves the channel
	to the ARM.
*/
static status_t
find_dma_interrupt(int node, int32& interrupt)
{
	const uint32* mask = (const uint32*)fdt_getprop(gFDT, node,
		"brcm,dma-channel-mask", NULL);
	if (mask == NULL || (fdt32_to_cpu(*mask) & (1 << kDmaChannel)) == 0)
		return B_BUSY;

	// GIC: <type number flags> per channel, type 0 = SPI (from 32)
	int length;
	const uint32* interrupts = (const uint32*)fdt_getprop(gFDT, node,
		"interrupts", &length);
	if (interrupts == NULL
		|| length < (int)((kDmaChannel + 1) * 3 * sizeof(uint32))) {
		return B_BAD_DATA;
	}
	const uint32* entry = interrupts + kDmaChannel * 3;
	interrupt = fdt32_to_cpu(entry[1]) + (fdt32_to_cpu(entry[0]) == 0 ? 32 : 16);
	return B_OK;
}


/*!	The bus address at which the DMA engine sees the physical address. */
static status_t
dma_bus_address(int dmaNode, phys_addr_t physical, uint32& busAddress)
{
	uint64 address = physical;
	if (!apply_ranges(fdt_parent_offset(gFDT, dmaNode), "dma-ranges", false,
			address) || address > UINT32_MAX) {
		return B_BAD_ADDRESS;
	}
	busAddress = (uint32)address;
	return B_OK;
}


//	#pragma mark - hardware


static bool
wait_for(volatile uint32* reg, uint32 mask, bool set)
{
	bigtime_t timeout = system_time() + 10000;
	while (((*reg & mask) != 0) != set) {
		if (system_time() > timeout)
			return false;
		spin(1);
	}
	return true;
}


/*!	PWM register writes that follow each other closely do not all take
	(PWM_STA reports a bus error), so each is read back, which retires it, and
	given time to cross into the PWM clock domain.
*/
static void
write_pwm(uint32 index, uint32 value)
{
	sDevice.pwm[index] = value;
	(void)sDevice.pwm[index];
	spin(10);
}


/*!	Runs the PWM clock from the oscillator at kClockDivisor. The order is the
	one Linux uses: stop the generator, change the source while it is
	disabled, then the divisor, and enable it in a write of its own; changing
	source and enable at once races the generator's clock domain crossing.
*/
static status_t
start_clock()
{
	volatile uint32* clock = sDevice.clock;
	clock[CM_PWMCTL] = kClockPassword
		| (clock[CM_PWMCTL] & ~(kClockPasswordMask | kClockEnable));
	if (!wait_for(&clock[CM_PWMCTL], kClockBusy, false)) {
		ERROR("PWM clock does not stop (CM_PWMCTL %#" B_PRIx32 ")\n",
			clock[CM_PWMCTL]);
	}

	// The divisor has no fraction, so MASH stays off (the firmware leaves it
	// on).
	uint32 control = (clock[CM_PWMCTL] & ~(kClockPasswordMask | kClockEnable
		| kClockMash | kClockSourceMask)) | kClockSourceOscillator;
	clock[CM_PWMCTL] = kClockPassword | control;
	clock[CM_PWMDIV] = kClockPassword | (kClockDivisor << 12);
	clock[CM_PWMCTL] = kClockPassword | control | kClockEnable | kClockGate;

	if (!wait_for(&clock[CM_PWMCTL], kClockBusy, true)) {
		ERROR("PWM clock does not start (CM_PWMCTL %#" B_PRIx32 ")\n",
			clock[CM_PWMCTL]);
		return B_IO_ERROR;
	}
	return B_OK;
}


static status_t
start_pwm()
{
	// GPIO 40 and 41 to PWM1
	uint32 select = sDevice.gpio[kGpioFunctionSelect4];
	select &= ~((7u << 0) | (7u << 3));
	select |= (kGpioAlt0 << 0) | (kGpioAlt0 << 3);
	sDevice.gpio[kGpioFunctionSelect4] = select;

	write_pwm(PWM_CTL, 0);
	write_pwm(PWM_RNG1, kRange);
	write_pwm(PWM_RNG2, kRange);
	write_pwm(PWM_CTL, kPwmClearFifo);
	write_pwm(PWM_CTL, kPwmChannels);
	sDevice.pwm[PWM_STA] = kPwmStatusErrors;

	if (sDevice.pwm[PWM_RNG1] != kRange
		|| (sDevice.pwm[PWM_CTL] & kPwmChannels) != kPwmChannels) {
		ERROR("PWM does not take its setup (CTL %#" B_PRIx32 ", RNG1 %"
			B_PRIu32 ")\n", sDevice.pwm[PWM_CTL], sDevice.pwm[PWM_RNG1]);
		return B_IO_ERROR;
	}
	return B_OK;
}


static void
stop_pwm()
{
	write_pwm(PWM_DMAC, 0);
	write_pwm(PWM_CTL, 0);
}


static void
reset_dma()
{
	sDevice.dma[DMA_CS] = kDmaReset;
	wait_for(&sDevice.dma[DMA_CS], kDmaReset, false);
}


static void
dump_state(const char* why)
{
	ERROR("%s: DMA CS %#" B_PRIx32 " CONBLK %#" B_PRIx32 " SOURCE %#" B_PRIx32
		" DEST %#" B_PRIx32 " LEN %" B_PRIu32 " DEBUG %#" B_PRIx32 "; PWM CTL %#"
		B_PRIx32 " STA %#" B_PRIx32 " DMAC %#" B_PRIx32 " RNG1 %" B_PRIu32
		"; CM_PWMCTL %#" B_PRIx32 " CM_PWMDIV %#" B_PRIx32 "\n", why,
		sDevice.dma[DMA_CS], sDevice.dma[DMA_CONBLK_AD],
		sDevice.dma[DMA_SOURCE_AD], sDevice.dma[DMA_DEST_AD],
		sDevice.dma[DMA_TXFR_LEN], sDevice.dma[DMA_DEBUG],
		sDevice.pwm[PWM_CTL], sDevice.pwm[PWM_STA], sDevice.pwm[PWM_DMAC],
		sDevice.pwm[PWM_RNG1], sDevice.clock[CM_PWMCTL],
		sDevice.clock[CM_PWMDIV]);
}


/*!	The words of the ring the DMA engine has read since it was armed, which
	starts it at the ring's base.
*/
static uint32
dma_words_read()
{
	return (sDevice.dma[DMA_SOURCE_AD] - sDevice.ring_bus_address)
		/ sizeof(uint32);
}


/*!	Starts the DMA at the ring's base and checks that it streams. */
static status_t
arm_dma()
{
	// The ring and control blocks are write-combining, the registers are
	// device memory: those writes are not ordered against each other, and the
	// engine must not fetch a control block before it reached memory.
	arm64_dsb(sy);

	sDevice.pwm[PWM_DMAC] = kPwmDmaControl;
	reset_dma();
	sDevice.dma[DMA_DEBUG] = kDmaDebugErrors;
	sDevice.dma[DMA_CONBLK_AD] = sDevice.control_blocks_bus_address;
	sDevice.dma[DMA_CS] = kDmaActive;

	snooze(kStartSettleTime);
	uint32 words = dma_words_read();
	if (words < kStartMinimumWords || words > kBufferCount * kSegmentWords) {
		dump_state("DMA does not stream");
		return B_IO_ERROR;
	}
	TRACE("DMA streams: %" B_PRIu32 " words in %" B_PRIdBIGTIME " us\n", words,
		kStartSettleTime);
	return B_OK;
}


static inline uint32
duty(int16 sample)
{
	return (uint32)((int32)sample + 32768) * kRange / 65536;
}


static void
fill_segment(int32 segment, uint32 value)
{
	volatile uint32* words = sDevice.ring + segment * kSegmentWords;
	for (uint32 i = 0; i < kSegmentWords; i++)
		words[i] = value;
}


/*!	Moves the output from \a from to \a to over a segment, so that powering
	the PWM up or down is not a click.
*/
static void
fill_ramp(int32 segment, uint32 from, uint32 to)
{
	volatile uint32* words = sDevice.ring + segment * kSegmentWords;
	for (uint32 frame = 0; frame < kFramesPerBuffer; frame++) {
		uint32 value = (from * (kFramesPerBuffer - frame) + to * frame)
			/ kFramesPerBuffer;
		for (uint32 channel = 0; channel < kChannelCount; channel++)
			words[frame * kChannelCount + channel] = value;
	}
}


static void
convert_buffer(int32 index)
{
	const int16* samples = sDevice.pcm + index * kSegmentWords;
	volatile uint32* words = sDevice.ring + index * kSegmentWords;
	for (uint32 i = 0; i < kSegmentWords; i++)
		words[i] = duty(samples[i]);
	arm64_dsb(sy);
}


static int32
dma_interrupt(void* data)
{
	uint32 status = sDevice.dma[DMA_CS];
	if ((status & kDmaInterrupt) == 0)
		return B_UNHANDLED_INTERRUPT;

	// Acknowledge, but keep the channel active: writing ACTIVE as 0 pauses it
	sDevice.dma[DMA_CS] = kDmaInterrupt | kDmaActive;

	// The segments are adjacent, so the read cursor names the segment that
	// plays now, whether or not the engine loaded its control block yet. The
	// slack covers a cursor that lags the interrupt by a few words; it limits
	// the interrupt latency this tolerates to 7/8 of a period.
	const int32 playing = ((dma_words_read() + kSegmentWords / 8)
		/ kSegmentWords) % kBufferCount;
	const int32 finished = (playing + 1) % kBufferCount;

	SpinLocker locker(sDevice.state_lock);

	// Silence what has been played, so that it does not play again if the
	// client does not deliver in time.
	fill_segment(finished, sDevice.idle_duty);
	arm64_dsb(sy);

	if (sDevice.primed && !sDevice.filled[playing])
		sDevice.underruns++;
	sDevice.filled[finished] = false;
	if ((status & kDmaError) != 0)
		sDevice.dma_errors++;

	sDevice.playing = playing;
	sDevice.real_time = system_time();
	sDevice.frames_count += kFramesPerBuffer;
	if (sDevice.periods++ == 0)
		sDevice.first_period_time = sDevice.real_time;

	locker.Unlock();

	// One pending period is enough: a client that fell behind should not then
	// get several exchanges in a row that all report the same buffer.
	int32 count;
	if (get_sem_count(sDevice.period_sem, &count) == B_OK && count <= 0)
		release_sem_etc(sDevice.period_sem, 1, B_DO_NOT_RESCHEDULE);

	return B_INVOKE_SCHEDULER;
}


static status_t
start_engine()
{
	if (sDevice.running)
		return B_OK;

	status_t status = start_clock();
	if (status == B_OK)
		status = start_pwm();
	if (status != B_OK)
		return status;

	// A restart must not play what the client left before it
	memset(sDevice.pcm, 0, kBufferCount * kSegmentWords * sizeof(int16));

	// The PWM output idles low until it streams: ramp up to the silence
	// level in the first segment.
	fill_ramp(0, 0, kDutySilence);
	fill_segment(1, kDutySilence);
	for (uint32 i = 0; i < kBufferCount; i++) {
		volatile dma_control_block& block = sDevice.control_blocks[i];
		block.transfer_info = kDmaInterruptEnable | kDmaWaitResponse
			| kDmaDestinationDreq | kDmaSourceIncrement
			| (kDmaDreqPwm1 << kDmaPermapShift);
		block.source = sDevice.ring_bus_address + i * kSegmentBytes;
		block.destination = sDevice.fifo_bus_address;
		block.length = kSegmentBytes;
		block.stride = 0;
		block.next = sDevice.control_blocks_bus_address
			+ ((i + 1) % kBufferCount) * sizeof(dma_control_block);
		block.reserved[0] = block.reserved[1] = 0;
	}

	{
		InterruptsSpinLocker locker(sDevice.state_lock);
		sDevice.playing = 0;
		sDevice.filled[0] = sDevice.filled[1] = false;
		sDevice.primed = false;
		sDevice.idle_duty = kDutySilence;
		sDevice.real_time = system_time();
		sDevice.frames_count = 0;
		sDevice.periods = 0;
		sDevice.underruns = 0;
		sDevice.dma_errors = 0;
	}
	sDevice.fill_cycle = -1;
	sDevice.rate_reported = false;

	status = install_io_interrupt_handler(sDevice.interrupt, &dma_interrupt,
		NULL, 0);
	if (status != B_OK) {
		ERROR("cannot install interrupt %" B_PRId32 ": %s\n",
			sDevice.interrupt, strerror(status));
		stop_pwm();
		return status;
	}

	// A channel that comes up parked has been seen to stream after the PWM
	// was set up again (on Phoenix-RTOS, on this board).
	for (uint32 attempt = 1; attempt <= kStartAttempts; attempt++) {
		status = arm_dma();
		if (status == B_OK)
			break;
		reset_dma();
		if (attempt < kStartAttempts)
			start_pwm();
	}
	if (status != B_OK) {
		ERROR("DMA does not stream after %" B_PRIu32 " attempts\n",
			kStartAttempts);
		stop_pwm();
		remove_io_interrupt_handler(sDevice.interrupt, &dma_interrupt, NULL);
		return status;
	}

	sDevice.running = true;
	TRACE_ALWAYS("streaming at %" B_PRIu32 ".%03" B_PRIu32 " Hz (PWM range %"
		B_PRIu32 "), DMA channel %" B_PRIu32 ", interrupt %" B_PRId32 "\n",
		kFrameRateMilliHertz / 1000, kFrameRateMilliHertz % 1000, kRange,
		kDmaChannel, sDevice.interrupt);
	return B_OK;
}


/*!	With \a ramp, the output is first brought down to the PWM's idle level,
	which takes two periods: the segment that plays when this is called, then
	the ramp. Without, the engine is stopped at once (it stalled anyway).
*/
static void
stop_engine(bool ramp)
{
	if (!sDevice.running)
		return;

	if (ramp) {
		InterruptsSpinLocker locker(sDevice.state_lock);
		const int32 next = (sDevice.playing + 1) % kBufferCount;
		sDevice.idle_duty = 0;
		locker.Unlock();

		fill_ramp(next, kDutySilence, 0);
		arm64_dsb(sy);
		snooze(2 * kPeriod + kPeriod / 4);
	}

	reset_dma();
	stop_pwm();
	remove_io_interrupt_handler(sDevice.interrupt, &dma_interrupt, NULL);
	sDevice.running = false;

	TRACE_ALWAYS("stopped after %" B_PRIu32 " periods: %" B_PRIu32
		" underruns, %" B_PRIu32 " exchange timeouts, %" B_PRIu32
		" DMA errors\n", sDevice.periods, sDevice.underruns, sDevice.timeouts,
		sDevice.dma_errors);
}


//	#pragma mark - multi_audio


static const multi_channel_info kChannels[] = {
	{ 0, B_MULTI_OUTPUT_CHANNEL, B_CHANNEL_LEFT | B_CHANNEL_STEREO_BUS, 0 },
	{ 1, B_MULTI_OUTPUT_CHANNEL, B_CHANNEL_RIGHT | B_CHANNEL_STEREO_BUS, 0 },
	{ 2, B_MULTI_OUTPUT_BUS, B_CHANNEL_LEFT | B_CHANNEL_STEREO_BUS,
		B_CHANNEL_MINI_JACK_STEREO },
	{ 3, B_MULTI_OUTPUT_BUS, B_CHANNEL_RIGHT | B_CHANNEL_STEREO_BUS,
		B_CHANNEL_MINI_JACK_STEREO },
};
static const int32 kChannelInfoCount = B_COUNT_OF(kChannels);


static status_t
get_description(void* arg)
{
	multi_description description;
	if (user_memcpy(&description, arg, sizeof(description)) != B_OK)
		return B_BAD_ADDRESS;

	multi_channel_info* channels = description.channels;
	int32 channelCount = min_c(description.request_channel_count,
		kChannelInfoCount);
	if (channelCount > 0 && (channels == NULL
			|| user_memcpy(channels, kChannels,
				channelCount * sizeof(multi_channel_info)) != B_OK)) {
		return B_BAD_ADDRESS;
	}

	// Everything not set below (inputs, CVSR rates, timecode) is 0; only the
	// caller's own fields are kept.
	const size_t infoSize = description.info_size;
	const int32 requestCount = description.request_channel_count;
	memset(&description, 0, sizeof(description));
	description.info_size = infoSize;
	description.request_channel_count = requestCount;
	description.channels = channels;

	description.interface_version = B_CURRENT_INTERFACE_VERSION;
	description.interface_minimum = B_CURRENT_INTERFACE_VERSION;
	strlcpy(description.friendly_name, "Raspberry Pi headphones",
		sizeof(description.friendly_name));
	strlcpy(description.vendor_info, "Haiku",
		sizeof(description.vendor_info));

	description.output_channel_count = kChannelCount;
	description.output_bus_channel_count = kChannelCount;
	description.output_rates = B_SR_44100;
	description.output_formats = B_FMT_16BIT;
	description.lock_sources = B_MULTI_LOCK_INTERNAL;
	description.interface_flags = B_MULTI_INTERFACE_PLAYBACK;
	description.start_latency = kStartSettleTime + kPeriod;

	return user_memcpy(arg, &description, sizeof(description));
}


static status_t
get_enabled_channels(void* arg)
{
	multi_channel_enable enable;
	if (user_memcpy(&enable, arg, sizeof(enable)) != B_OK)
		return B_BAD_ADDRESS;

	// all of them, always
	uchar bits = (1 << kChannelInfoCount) - 1;
	if (enable.enable_bits == NULL
		|| user_memcpy(enable.enable_bits, &bits, sizeof(bits)) != B_OK) {
		return B_BAD_ADDRESS;
	}

	enable.lock_source = B_MULTI_LOCK_INTERNAL;
	return user_memcpy(arg, &enable, sizeof(enable));
}


static status_t
get_global_format(void* arg)
{
	multi_format_info info;
	if (user_memcpy(&info, arg, sizeof(info)) != B_OK)
		return B_BAD_ADDRESS;

	info.output_latency = kPeriod;
	info.input_latency = 0;
	info.timecode_kind = 0;
	memset(&info.input, 0, sizeof(info.input));
	memset(&info.output, 0, sizeof(info.output));
	info.output.rate = B_SR_44100;
	info.output.format = B_FMT_16BIT;

	return user_memcpy(arg, &info, sizeof(info));
}


static status_t
list_mix_controls(void* arg)
{
	// no mixer: the media kit's own gain applies
	multi_mix_control_info info;
	if (user_memcpy(&info, arg, sizeof(info)) != B_OK)
		return B_BAD_ADDRESS;

	info.control_count = 0;
	return user_memcpy(arg, &info, sizeof(info));
}


static status_t
list_mix_connections(void* arg)
{
	multi_mix_connection_info info;
	if (user_memcpy(&info, arg, sizeof(info)) != B_OK)
		return B_BAD_ADDRESS;

	info.actual_count = 0;
	return user_memcpy(arg, &info, sizeof(info));
}


static status_t
get_buffers(void* arg)
{
	multi_buffer_list list;
	if (user_memcpy(&list, arg, sizeof(list)) != B_OK)
		return B_BAD_ADDRESS;

	// The buffers are fixed; the client needs room for all of them
	if (list.request_playback_buffers < (int32)kBufferCount
		|| list.request_playback_channels < (int32)kChannelCount
		|| list.playback_buffers == NULL) {
		return B_BAD_VALUE;
	}

	for (uint32 buffer = 0; buffer < kBufferCount; buffer++) {
		buffer_desc* row;
		if (user_memcpy(&row, &list.playback_buffers[buffer], sizeof(row))
				!= B_OK || row == NULL) {
			return B_BAD_ADDRESS;
		}

		buffer_desc descriptors[kChannelCount];
		memset(descriptors, 0, sizeof(descriptors));
		for (uint32 channel = 0; channel < kChannelCount; channel++) {
			descriptors[channel].base = (char*)(sDevice.pcm
				+ buffer * kSegmentWords + channel);
			descriptors[channel].stride = kChannelCount * sizeof(int16);
		}
		if (user_memcpy(row, descriptors, sizeof(descriptors)) != B_OK)
			return B_BAD_ADDRESS;
	}

	list.flags = B_MULTI_BUFFER_PLAYBACK;
	list.return_playback_buffers = kBufferCount;
	list.return_playback_channels = kChannelCount;
	list.return_playback_buffer_size = kFramesPerBuffer;
	list.return_record_buffers = 0;
	list.return_record_channels = 0;
	list.return_record_buffer_size = 0;

	return user_memcpy(arg, &list, sizeof(list));
}


/*!	The media add-on fills the buffer before the one we report as playing:
	that is the one that has just been played. It is converted on the next
	exchange, which then waits for the end of the current period.
*/
static status_t
buffer_exchange(void* arg)
{
	multi_buffer_info info;
	if (user_memcpy(&info, arg, sizeof(info)) != B_OK)
		return B_BAD_ADDRESS;

	{
		MutexLocker locker(sDevice.lock);
		status_t status = sDevice.failed ? B_IO_ERROR : start_engine();
		if (status != B_OK) {
			// The engine did not come up; it is tried again on the next open.
			// Until then the client is paced as if it played, so that its
			// thread does not spin on the failing exchange.
			sDevice.failed = true;
			locker.Unlock();
			snooze(kPeriod);
			return status;
		}
	}

	if (sDevice.fill_cycle >= 0) {
		convert_buffer(sDevice.fill_cycle);
		InterruptsSpinLocker locker(sDevice.state_lock);
		sDevice.filled[sDevice.fill_cycle] = true;
		sDevice.primed = true;
	}

	status_t status = acquire_sem_etc(sDevice.period_sem, 1,
		B_CAN_INTERRUPT | B_RELATIVE_TIMEOUT, kExchangeTimeout);
	if (status == B_TIMED_OUT) {
		MutexLocker locker(sDevice.lock);
		if (sDevice.running) {
			if (sDevice.timeouts++ < kTimeoutsLogged)
				dump_state("no period ended, restarting");
			stop_engine(false);
		}
		return status;
	}
	if (status != B_OK)
		return status;

	InterruptsSpinLocker locker(sDevice.state_lock);
	info.playback_buffer_cycle = sDevice.playing;
	info.played_real_time = sDevice.real_time;
	info.played_frames_count = sDevice.frames_count;
	const uint32 periods = sDevice.periods;
	const bigtime_t elapsed = sDevice.real_time - sDevice.first_period_time;
	const uint32 underruns = sDevice.underruns;
	locker.Unlock();

	sDevice.fill_cycle = (info.playback_buffer_cycle + kBufferCount - 1)
		% kBufferCount;

	if (!sDevice.rate_reported && periods >= kRateReportPeriods
		&& elapsed > 0) {
		// the rate the hardware actually plays at, from the interrupt times
		const uint64 milliHertz = (uint64)(periods - 1) * kFramesPerBuffer
			* 1000000000 / elapsed;
		TRACE_ALWAYS("measured %" B_PRIu64 ".%03" B_PRIu64 " Hz over %"
			B_PRIu32 " periods, %" B_PRIu32 " underruns\n", milliHertz / 1000,
			milliHertz % 1000, periods, underruns);
		sDevice.rate_reported = true;
	}

	return user_memcpy(arg, &info, sizeof(info));
}


/*!	The engine keeps running over silence until the device is closed: stopping
	the PWM would click.
*/
static status_t
buffer_force_stop()
{
	MutexLocker locker(sDevice.lock);
	if (!sDevice.running)
		return B_OK;

	InterruptsSpinLocker stateLocker(sDevice.state_lock);
	for (uint32 i = 0; i < kBufferCount; i++) {
		fill_segment(i, kDutySilence);
		sDevice.filled[i] = false;
	}
	sDevice.primed = false;
	stateLocker.Unlock();

	sDevice.fill_cycle = -1;
	return B_OK;
}


//	#pragma mark - device hooks


static status_t
bcm2711_pwm_open(const char* name, uint32 flags, void** _cookie)
{
	MutexLocker locker(sDevice.lock);
	sDevice.open_count++;
	*_cookie = &sDevice;
	return B_OK;
}


static status_t
bcm2711_pwm_close(void* cookie)
{
	return B_OK;
}


static status_t
bcm2711_pwm_free(void* cookie)
{
	MutexLocker locker(sDevice.lock);
	if (--sDevice.open_count == 0) {
		stop_engine(true);
		sDevice.failed = false;
	}
	return B_OK;
}


static status_t
bcm2711_pwm_control(void* cookie, uint32 op, void* arg, size_t length)
{
	switch (op) {
		case B_MULTI_GET_DESCRIPTION:
			return get_description(arg);
		case B_MULTI_GET_ENABLED_CHANNELS:
			return get_enabled_channels(arg);
		case B_MULTI_SET_ENABLED_CHANNELS:
			return B_OK;
		case B_MULTI_GET_GLOBAL_FORMAT:
			return get_global_format(arg);
		case B_MULTI_SET_GLOBAL_FORMAT:
			// There is only one format; like the other drivers, this does not
			// fail, B_MULTI_GET_GLOBAL_FORMAT tells what is in effect.
			return B_OK;
		case B_MULTI_GET_MIX:
		case B_MULTI_SET_MIX:
		case B_MULTI_LIST_MIX_CHANNELS:
			return B_OK;
		case B_MULTI_LIST_MIX_CONTROLS:
			return list_mix_controls(arg);
		case B_MULTI_LIST_MIX_CONNECTIONS:
			return list_mix_connections(arg);
		case B_MULTI_GET_BUFFERS:
			return get_buffers(arg);
		case B_MULTI_BUFFER_EXCHANGE:
			return buffer_exchange(arg);
		case B_MULTI_BUFFER_FORCE_STOP:
			return buffer_force_stop();

		case B_MULTI_GET_EVENT_INFO:
		case B_MULTI_SET_EVENT_INFO:
		case B_MULTI_GET_EVENT:
		case B_MULTI_GET_CHANNEL_FORMATS:
		case B_MULTI_SET_CHANNEL_FORMATS:
		case B_MULTI_SET_BUFFERS:
		case B_MULTI_SET_START_TIME:
			return B_ERROR;
	}
	return B_DEV_INVALID_IOCTL;
}


static status_t
bcm2711_pwm_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static status_t
bcm2711_pwm_write(void* cookie, off_t position, const void* buffer,
	size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static device_hooks sDeviceHooks = {
	bcm2711_pwm_open,
	bcm2711_pwm_close,
	bcm2711_pwm_free,
	bcm2711_pwm_control,
	bcm2711_pwm_read,
	bcm2711_pwm_write,
};

static const char* sDeviceNames[] = {
	DEVICE_NAME,
	NULL
};


//	#pragma mark - driver


/*!	The PCM buffers the media add-on writes (user accessible, cached: only the
	CPU reads them), and the ring and control blocks for the DMA engine, in
	contiguous memory it reaches and mapped write-combining, as it does not
	snoop the CPU caches.
*/
static status_t
allocate_buffers(int dmaNode)
{
	sDevice.pcm_area = create_area("bcm2711_pwm buffers",
		(void**)&sDevice.pcm, B_ANY_KERNEL_ADDRESS,
		ROUNDUP(kBufferCount * kSegmentWords * sizeof(int16), B_PAGE_SIZE),
		B_FULL_LOCK, B_READ_AREA | B_WRITE_AREA);
	if (sDevice.pcm_area < 0)
		return sDevice.pcm_area;
	memset(sDevice.pcm, 0, kBufferCount * kSegmentWords * sizeof(int16));

	const size_t size = ROUNDUP(kBufferCount * kSegmentBytes
		+ kBufferCount * sizeof(dma_control_block), B_PAGE_SIZE);
	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = kDmaMemoryLimit;
	void* address;
	sDevice.ring_area = create_area_etc(B_SYSTEM_TEAM, "bcm2711_pwm dma", size,
		B_CONTIGUOUS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0,
		&virtualRestrictions, &physicalRestrictions, &address);
	if (sDevice.ring_area < 0)
		return sDevice.ring_area;

	physical_entry entry;
	uint32 count = 1;
	status_t status = get_memory_map_etc(B_SYSTEM_TEAM, address, B_PAGE_SIZE,
		&entry, &count);
	if (status == B_OK || status == B_BUFFER_OVERFLOW) {
		// The pages were cleared through the cacheable physical map: write
		// those lines back and drop them, so that none is evicted over what
		// the write-combining mapping stores later.
		arm64_dcache_flush_range((addr_t)address, size);
		status = vm_set_area_memory_type(sDevice.ring_area, entry.address,
			B_WRITE_COMBINING_MEMORY);
	}
	if (status == B_OK)
		status = dma_bus_address(dmaNode, entry.address,
			sDevice.ring_bus_address);
	if (status != B_OK)
		return status;

	sDevice.ring = (volatile uint32*)address;
	sDevice.control_blocks = (volatile dma_control_block*)((addr_t)address
		+ kBufferCount * kSegmentBytes);
	sDevice.control_blocks_bus_address = sDevice.ring_bus_address
		+ kBufferCount * kSegmentBytes;
	return B_OK;
}


static void
free_resources()
{
	delete_sem(sDevice.period_sem);
	delete_area(sDevice.ring_area);
	delete_area(sDevice.pcm_area);
	delete_area(sDevice.dma_area);
	delete_area(sDevice.gpio_area);
	delete_area(sDevice.clock_area);
	delete_area(sDevice.pwm_area);
	mutex_destroy(&sDevice.lock);
}


extern "C" status_t
init_hardware(void)
{
	if (gFDT == NULL || fdt_node_check_compatible(gFDT, 0, "brcm,bcm2711") != 0
		|| find_compatible("brcm,bcm2835-pwm", kPwm1BusAddress) < 0
		|| find_compatible("brcm,bcm2835-dma") < 0) {
		return B_DEVICE_NOT_FOUND;
	}
	return B_OK;
}


extern "C" status_t
init_driver(void)
{
	memset(&sDevice, 0, sizeof(sDevice));
	sDevice.pwm_area = sDevice.clock_area = sDevice.gpio_area
		= sDevice.dma_area = sDevice.pcm_area = sDevice.ring_area = -1;
	sDevice.period_sem = -1;
	B_INITIALIZE_SPINLOCK(&sDevice.state_lock);
	mutex_init(&sDevice.lock, "bcm2711_pwm");

	const int dmaNode = find_compatible("brcm,bcm2835-dma");
	uint64 fifoBusAddress = 0;
	status_t status = map_registers(
		find_compatible("brcm,bcm2835-pwm", kPwm1BusAddress), 0, "bcm2711_pwm",
		sDevice.pwm_area, sDevice.pwm, &fifoBusAddress);
	if (status == B_OK) {
		status = map_registers(find_compatible("brcm,bcm2711-cprman"), 0,
			"bcm2711_pwm clock", sDevice.clock_area, sDevice.clock);
	}
	if (status == B_OK) {
		status = map_registers(find_compatible("brcm,bcm2711-gpio"), 0,
			"bcm2711_pwm gpio", sDevice.gpio_area, sDevice.gpio);
	}
	if (status == B_OK) {
		status = map_registers(dmaNode, kDmaChannel * 0x100, "bcm2711_pwm dma",
			sDevice.dma_area, sDevice.dma);
	}
	if (status == B_OK)
		status = find_dma_interrupt(dmaNode, sDevice.interrupt);
	if (status == B_OK)
		status = allocate_buffers(dmaNode);
	if (status == B_OK) {
		sDevice.period_sem = create_sem(0, "bcm2711_pwm period");
		if (sDevice.period_sem < 0)
			status = sDevice.period_sem;
	}
	if (status != B_OK) {
		ERROR("init failed: %s\n", strerror(status));
		free_resources();
		return status;
	}

	sDevice.fifo_bus_address = (uint32)fifoBusAddress + PWM_FIF1 * 4;
	return B_OK;
}


extern "C" void
uninit_driver(void)
{
	free_resources();
}


extern "C" const char**
publish_devices(void)
{
	return sDeviceNames;
}


extern "C" device_hooks*
find_device(const char* name)
{
	return strcmp(name, DEVICE_NAME) == 0 ? &sDeviceHooks : NULL;
}
