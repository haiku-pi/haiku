/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	HDMI audio on the Raspberry Pi 4 (BCM2711, the first HDMI port), as a
	multi_audio device: two channels, 16 bit, 48 kHz.

	The firmware sets up the HDMI port and its video mode; this driver only
	drives the port's audio path, which is separate from the video one: the MAI
	FIFO, its sample clock (from a fixed 108 MHz source), the audio clock
	regeneration values (N, CTS) and an Audio InfoFrame in a packet slot the
	firmware does not use. The FIFO takes IEC 60958 subframes, one 32-bit word
	per sample: a preamble, the sample in bits 27 to 12, a channel status bit
	(of a 192-frame block) and even parity.

	A legacy DMA channel, paced by the MAI's DREQ, feeds the FIFO from a ring
	of such words in two segments, one per multi_audio buffer, as in the
	bcm2711_pwm driver this one follows. A segment is a whole number of
	192-frame blocks, so every segment starts a block and the preamble and
	status bit of each word depend only on its position. The engine plays
	digital silence while the device is open and no buffer arrives.

	HDMI audio needs a monitor that takes it: the driver publishes its device
	only when the firmware drives the port in HDMI (not DVI) mode with the
	state machine clock running. Without a monitor at boot, the port's
	registers must not be touched at all (a read stalls the bus), which the
	measured state machine clock tells beforehand.
	See docs/design/hdmi-audio.md in the haiku-rpi repository.
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

#include <bcm2835_mailbox.h>

// after the kernel's ASSERT, or it pulls in the userland <Debug.h>
#include <hmulti_audio.h>

extern "C" {
#include <libfdt.h>
}


//#define TRACE_BCM2711_HDMI_AUDIO
#ifdef TRACE_BCM2711_HDMI_AUDIO
#	define TRACE(x...)		dprintf("bcm2711_hdmi_audio: " x)
#else
#	define TRACE(x...)
#endif
#define TRACE_ALWAYS(x...)	dprintf("bcm2711_hdmi_audio: " x)
#define ERROR(x...)			dprintf("bcm2711_hdmi_audio: " x)


#define DEVICE_NAME "audio/hmulti/bcm2711_hdmi_audio/1"


extern void* gFDT;


int32 api_version = B_CUR_DRIVER_API_VERSION;


// HDMI core registers, as word indices
enum {
	HDMI_MAI_CHANNEL_MAP		= 0x09c / 4,
	HDMI_MAI_CONFIG				= 0x0a0 / 4,
	HDMI_AUDIO_PACKET_CONFIG	= 0x0b8 / 4,
	HDMI_RAM_PACKET_CONFIG		= 0x0bc / 4,
	HDMI_RAM_PACKET_STATUS		= 0x0c4 / 4,
	HDMI_CRP_CFG				= 0x0c8 / 4,
	HDMI_CTS_0					= 0x0cc / 4,
	HDMI_CTS_1					= 0x0d0 / 4,
	HDMI_SCHEDULER_CONTROL		= 0x0e0 / 4,
	HDMI_HORZA					= 0x0e4 / 4,
	HDMI_HORZB					= 0x0e8 / 4,
	HDMI_VERTA0					= 0x0ec / 4,
	HDMI_VERTB0					= 0x0f0 / 4,
	HDMI_HOTPLUG				= 0x1a8 / 4,
};
// "hd" block registers of the first HDMI port, as word indices
enum {
	HD_MAI_CTL		= 0x10 / 4,
	HD_MAI_THR		= 0x14 / 4,
	HD_MAI_FMT		= 0x18 / 4,
	HD_MAI_DATA		= 0x1c / 4,
	HD_MAI_SMP		= 0x20 / 4,
	HD_FRAME_COUNT	= 0x60 / 4,
};
static const uint32 kHdMaiDataOffset = 0x1c;
static const uint32 kPhyPowerdownControl = 0x004 / 4;
static const uint32 kPhyRandomGeneratorPowerdown = 1 << 4;
static const uint32 kDvpMiscConfig = 0x08 / 4;
static const uint32 kDvpAudioClockGate = 1 << 3;
	// set: the first port's 108 MHz audio clock is stopped

static const uint32 kSchedulerHdmiMode = (1 << 0) | (1 << 1);
	// HDMI (not DVI) mode, and active
static const uint32 kRamPacketEnable = 1 << 16;
static const uint32 kHotplugConnected = 1 << 0;

// MAI_CTL
static const uint32 kMaiReset = 1 << 0;
static const uint32 kMaiErrorFull = 1 << 1;
static const uint32 kMaiErrorEmpty = 1 << 2;
static const uint32 kMaiEnable = 1 << 3;
static const uint32 kMaiChannels2 = 2 << 4;
static const uint32 kMaiFlush = 1 << 9;
static const uint32 kMaiWholeSample = 1 << 12;
static const uint32 kMaiChannelAlign = 1 << 13;
static const uint32 kMaiDataLate = 1 << 15;
static const uint32 kMaiStart = kMaiReset | kMaiErrorFull | kMaiErrorEmpty
	| kMaiFlush | kMaiDataLate;
static const uint32 kMaiRun = kMaiChannels2 | kMaiWholeSample
	| kMaiChannelAlign | kMaiEnable;
static const uint32 kMaiStop = kMaiDataLate | kMaiErrorEmpty | kMaiErrorFull;

// The values for 2-channel 16-bit PCM at 48 kHz (docs/design/hdmi-audio.md 3)
static const uint32 kMaiFormat48k = (2 << 16) | (9 << 8);
	// PCM, 48 kHz
static const uint32 kMaiThresholds = 0x10101c1c;
static const uint32 kMaiSampleClock48k = (2250 << 8) | (1 - 1);
	// 108 MHz * 1 / 2250
static const uint32 kMaiConfig = (1 << 27) | (1 << 26) | 0x3;
	// format and bit reversal, channels 0 and 1
static const uint32 kMaiChannelMap = 0x10;
static const uint32 kAudioPacketConfig = (1 << 29) | (1 << 24) | (8 << 10)
	| 0x3;
	// zero data on flat samples and inactive channels, B frame preamble 8,
	// channels 0 and 1
static const uint32 kAudioClockN48k = 6144;
	// 128 * 48 kHz / 1 kHz
static const uint32 kCrpExternalCts = 1 << 24;

// The firmware fills packet slots 0, 2 and 4 (its SPD InfoFrame is in 4);
// the Audio InfoFrame goes into slot 1.
static const uint32 kInfoFrameSlot = 1;
static const uint32 kPacketSlotWords = 0x24 / 4;
// type 0x84, version 1, length 10; checksum 0x70, 2 channels
static const uint32 kAudioInfoFrame[] = { 0x000a0184, 0x00000170 };

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
// Transfer information of a control block
static const uint32 kDmaInterruptEnable = 1 << 0;
static const uint32 kDmaWaitResponse = 1 << 3;
static const uint32 kDmaDestinationDreq = 1 << 6;
static const uint32 kDmaSourceIncrement = 1 << 8;
static const uint32 kDmaPermapShift = 16;
static const uint32 kDmaDreqHdmi0 = 10;

// Channel 4 is free for the ARM in the firmware's channel mask (the PWM
// driver uses 5), has an interrupt of its own, and reaches the low 1 GiB.
static const uint32 kDmaChannel = 4;
static const phys_addr_t kDmaMemoryLimit = (phys_addr_t)1 << 30;

// the firmware clock of the HDMI state machine
static const uint32 kClockHdmiStateMachine = 13;

static const uint32 kFrameRate = 48000;
static const uint32 kChannelCount = 2;
static const uint32 kBufferCount = 2;
static const uint32 kFramesPerBlock = 192;
	// of channel status
static const uint32 kFramesPerBuffer = 5 * kFramesPerBlock;
static const uint32 kSegmentWords = kFramesPerBuffer * kChannelCount;
static const uint32 kSegmentBytes = kSegmentWords * sizeof(uint32);
static const bigtime_t kPeriod = (bigtime_t)kFramesPerBuffer * 1000000
	/ kFrameRate;
	// 20 ms

// consumer, PCM, category "PCM coder" (original), 48 kHz, 16 bit
static const uint8 kChannelStatus[kFramesPerBlock / 8] = {
	0x04, 0x82, 0x00, 0x02, 0x02
};

static const bigtime_t kExchangeTimeout = 10 * kPeriod;
static const uint32 kTimeoutsLogged = 4;
static const bigtime_t kStartSettleTime = 20000;
static const uint32 kStartMinimumWords = 64;
static const uint32 kRateReportPeriods = 150;
	// ~3 s


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


struct hdmi_audio {
	mutex				lock;
		// open count, engine start and stop
	int32				open_count;

	area_id				core_area;
	area_id				hd_area;
	area_id				packet_area;
	area_id				phy_area;
	area_id				dvp_area;
	area_id				dma_area;
	volatile uint32*	core;
	volatile uint32*	hd;
	volatile uint32*	packet;
	volatile uint32*	phy;
	volatile uint32*	dvp;
	volatile uint32*	dma;
	uint32				fifo_bus_address;
	int32				interrupt;
	uint32				pixel_clock;
		// kHz, for the audio clock regeneration (CTS)

	area_id				pcm_area;
	int16*				pcm;

	// what the DMA reads, write-combining: the ring of subframes in
	// kBufferCount segments, then one control block per segment
	area_id				ring_area;
	volatile uint32*	ring;
	volatile dma_control_block* control_blocks;
	uint32				ring_bus_address;
	uint32				control_blocks_bus_address;

	bool				running;
	bool				failed;
	sem_id				period_sem;
	int32				fill_cycle;

	// shared with the interrupt handler
	spinlock			state_lock;
	int32				playing;
	bool				filled[kBufferCount];
	bool				primed;
	bigtime_t			real_time;
	int64				frames_count;
	bigtime_t			first_period_time;
	uint32				periods;
	uint32				underruns;
	uint32				dma_errors;

	uint32				timeouts;
	bool				rate_reported;
};

static hdmi_audio sDevice;

// the subframe of each word of a segment with a zero sample: preamble,
// channel status bit and parity
static uint32 sSilence[kSegmentWords];


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


/*!	The index of \a name in the "reg-names" of \a node, -1 if it is not there.
*/
static int
register_index(int node, const char* name)
{
	int length;
	const char* names = (const char*)fdt_getprop(gFDT, node, "reg-names",
		&length);
	if (names == NULL)
		return name == NULL ? 0 : -1;
	if (name == NULL)
		return 0;

	int index = 0;
	for (int offset = 0; offset < length; index++) {
		if (strcmp(names + offset, name) == 0)
			return index;
		offset += strlen(names + offset) + 1;
	}
	return -1;
}


/*!	A register block of \a node, by its name in "reg-names" (NULL: the first):
	its address on the node's bus (what the DMA engine uses) and the CPU's
	physical address of it.
*/
static status_t
node_registers(int node, const char* name, uint64& busAddress,
	phys_addr_t& physical)
{
	int bus = fdt_parent_offset(gFDT, node);
	int length;
	const uint32* reg = (const uint32*)fdt_getprop(gFDT, node, "reg", &length);
	int index = register_index(node, name);
	if (bus < 0 || reg == NULL || index < 0)
		return B_BAD_DATA;

	uint32 entryCells = address_cells(bus) + size_cells(bus);
	if ((uint32)length < (index + 1) * entryCells * sizeof(uint32))
		return B_BAD_DATA;
	busAddress = read_cells(reg + index * entryCells, address_cells(bus));

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
find_compatible(const char* compatible)
{
	return fdt_node_offset_by_compatible(gFDT, -1, compatible);
}


/*!	Maps the page holding a register block of \a node, at \a offset into it.
*/
static status_t
map_registers(int node, const char* blockName, uint32 offset,
	const char* areaName, area_id& area, volatile uint32*& registers,
	uint64* _busAddress = NULL)
{
	uint64 busAddress;
	phys_addr_t physical;
	if (node < 0 || node_registers(node, blockName, busAddress, physical)
			!= B_OK) {
		return B_DEVICE_NOT_FOUND;
	}
	physical += offset;

	void* address;
	area = map_physical_memory(areaName,
		physical & ~(phys_addr_t)(B_PAGE_SIZE - 1), B_PAGE_SIZE,
		B_ANY_KERNEL_ADDRESS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
		&address);
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


//	#pragma mark - firmware


/*!	Whether the HDMI state machine runs, measured: the firmware reports no set
	rate for it. Without a monitor at boot it does not, and then a read of
	any HDMI register stalls the bus.
*/
static bool
hdmi_clock_runs()
{
	bcm2835_mailbox_module_info* mailbox;
	if (get_module(BCM2835_MAILBOX_MODULE_NAME, (module_info**)&mailbox)
			!= B_OK) {
		return false;
	}
	uint32 clock[2] = { kClockHdmiStateMachine, 0 };
	status_t status = mailbox->property_tag(BCM2835_TAG_GET_CLOCK_MEASURED,
		clock, sizeof(clock), sizeof(uint32), NULL);
	put_module(BCM2835_MAILBOX_MODULE_NAME);
	TRACE("HDMI state machine clock: %s, %" B_PRIu32 " Hz\n",
		strerror(status), clock[1]);
	return status == B_OK && clock[1] != 0;
}


//	#pragma mark - hardware


static bool
wait_for(volatile uint32* reg, uint32 mask, bool set)
{
	bigtime_t timeout = system_time() + 100000;
	while (((*reg & mask) != 0) != set) {
		if (system_time() > timeout)
			return false;
		snooze(100);
	}
	return true;
}


/*!	Whether the firmware drives the port so that it can carry audio. */
static bool
port_takes_audio()
{
	return (sDevice.core[HDMI_HOTPLUG] & kHotplugConnected) != 0
		&& (sDevice.core[HDMI_SCHEDULER_CONTROL] & kSchedulerHdmiMode)
			== kSchedulerHdmiMode
		&& (sDevice.core[HDMI_RAM_PACKET_CONFIG] & kRamPacketEnable) != 0
		&& (sDevice.dvp[kDvpMiscConfig] & kDvpAudioClockGate) == 0;
}


/*!	The pixel clock of the current mode, in kHz, from the timing the firmware
	programmed and the frame rate counted over a while; snapped to the CEA
	clocks it is close to, as those are what monitors expect.
*/
static uint32
measure_pixel_clock()
{
	const uint32 horza = sDevice.core[HDMI_HORZA];
	const uint32 horzb = sDevice.core[HDMI_HORZB];
	const uint32 verta = sDevice.core[HDMI_VERTA0];
	const uint32 vertb = sDevice.core[HDMI_VERTB0];
	const uint32 totalWidth = (horza & 0x3fff) + ((horza >> 16) & 0x1fff)
		+ ((horzb >> 16) & 0x7ff) + (horzb & 0x7ff);
	const uint32 totalHeight = (verta & 0x1fff) + ((verta >> 16) & 0x7f)
		+ ((verta >> 24) & 0x1f) + (vertb & 0x1ff);

	// Time kFrames frames from one frame counter step to another; polling
	// every 50 us makes that good to better than 0.01 %.
	static const uint32 kFrames = 30;
	uint32 count = sDevice.hd[HD_FRAME_COUNT];
	bigtime_t start = 0, elapsed = 0;
	const bigtime_t timeout = system_time() + 1000000;
	for (uint32 frames = 0; frames <= kFrames && system_time() < timeout;) {
		const uint32 now = sDevice.hd[HD_FRAME_COUNT];
		if (now == count) {
			spin(50);
			continue;
		}
		count = now;
		if (frames++ == 0)
			start = system_time();
		else
			elapsed = system_time() - start;
	}
	const uint32 frames = kFrames;
	if (elapsed == 0) {
		ERROR("the frame counter does not count\n");
		return 0;
	}

	uint64 clock = (uint64)totalWidth * totalHeight * frames * 1000
		/ elapsed;
		// kHz
	static const uint32 kStandardClocks[] = {
		25175, 25200, 27000, 27027, 54000, 74176, 74250, 148352, 148500,
		296703, 297000, 594000
	};
	uint64 closest = 0;
	for (uint32 standard : kStandardClocks) {
		const uint64 distance = clock > standard ? clock - standard
			: standard - clock;
		if (distance * 200 < standard && (closest == 0
				|| distance < (clock > closest ? clock - closest
					: closest - clock))) {
			closest = standard;
		}
	}
	if (closest != 0)
		clock = closest;
	TRACE_ALWAYS("mode %" B_PRIu32 " x %" B_PRIu32 " total, %" B_PRIu32
		" frames in %" B_PRIdBIGTIME " us: pixel clock %" B_PRIu64 " kHz\n",
		totalWidth, totalHeight, frames, elapsed, clock);
	return (uint32)clock;
}


static status_t
write_info_frame()
{
	volatile uint32* slot = sDevice.packet + kInfoFrameSlot * kPacketSlotWords;
	sDevice.core[HDMI_RAM_PACKET_CONFIG] &= ~(1u << kInfoFrameSlot);
	if (!wait_for(&sDevice.core[HDMI_RAM_PACKET_STATUS], 1u << kInfoFrameSlot,
			false)) {
		return B_TIMED_OUT;
	}
	for (uint32 i = 0; i < kPacketSlotWords; i++)
		slot[i] = i < B_COUNT_OF(kAudioInfoFrame) ? kAudioInfoFrame[i] : 0;
	sDevice.core[HDMI_RAM_PACKET_CONFIG] |= 1u << kInfoFrameSlot;
	if (!wait_for(&sDevice.core[HDMI_RAM_PACKET_STATUS], 1u << kInfoFrameSlot,
			true)) {
		return B_TIMED_OUT;
	}
	return B_OK;
}


static status_t
start_mai()
{
	if (!hdmi_clock_runs() || !port_takes_audio()) {
		ERROR("the HDMI port does not take audio now\n");
		return B_DEVICE_NOT_FOUND;
	}

	sDevice.hd[HD_MAI_CTL] = kMaiStart;
	sDevice.phy[kPhyPowerdownControl] &= ~kPhyRandomGeneratorPowerdown;
	sDevice.hd[HD_MAI_SMP] = kMaiSampleClock48k;
	sDevice.hd[HD_MAI_CTL] = kMaiRun;
	sDevice.hd[HD_MAI_FMT] = kMaiFormat48k;
	sDevice.hd[HD_MAI_THR] = kMaiThresholds;
	sDevice.core[HDMI_MAI_CONFIG] = kMaiConfig;
	sDevice.core[HDMI_MAI_CHANNEL_MAP] = kMaiChannelMap;
	sDevice.core[HDMI_AUDIO_PACKET_CONFIG] = kAudioPacketConfig;

	// CTS = pixel clock * N / (128 * sample rate)
	const uint32 cts = (uint64)sDevice.pixel_clock * 1000 * kAudioClockN48k
		/ (128 * kFrameRate);
	sDevice.core[HDMI_CRP_CFG] = kCrpExternalCts | kAudioClockN48k;
	sDevice.core[HDMI_CTS_0] = cts;
	sDevice.core[HDMI_CTS_1] = cts;

	status_t status = write_info_frame();
	if (status != B_OK)
		ERROR("the Audio InfoFrame does not take: %s\n", strerror(status));
	return B_OK;
}


static void
stop_mai()
{
	sDevice.hd[HD_MAI_CTL] = kMaiStop;
	sDevice.phy[kPhyPowerdownControl] |= kPhyRandomGeneratorPowerdown;
	sDevice.core[HDMI_RAM_PACKET_CONFIG] &= ~(1u << kInfoFrameSlot);
	sDevice.hd[HD_MAI_CTL] = kMaiReset;
	sDevice.hd[HD_MAI_CTL] = kMaiErrorFull;
	sDevice.hd[HD_MAI_CTL] = kMaiFlush;
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
		" DEST %#" B_PRIx32 " LEN %" B_PRIu32 " DEBUG %#" B_PRIx32 "; MAI_CTL %#"
		B_PRIx32 "; SCHEDULER_CONTROL %#" B_PRIx32 " HOTPLUG %#" B_PRIx32 "\n",
		why, sDevice.dma[DMA_CS], sDevice.dma[DMA_CONBLK_AD],
		sDevice.dma[DMA_SOURCE_AD], sDevice.dma[DMA_DEST_AD],
		sDevice.dma[DMA_TXFR_LEN], sDevice.dma[DMA_DEBUG],
		sDevice.hd[HD_MAI_CTL], sDevice.core[HDMI_SCHEDULER_CONTROL],
		sDevice.core[HDMI_HOTPLUG]);
}


static uint32
dma_words_read()
{
	return (sDevice.dma[DMA_SOURCE_AD] - sDevice.ring_bus_address)
		/ sizeof(uint32);
}


static status_t
arm_dma()
{
	// The ring and control blocks are write-combining, the registers are
	// device memory: those writes are not ordered against each other.
	arm64_dsb(sy);

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
	return B_OK;
}


/*!	The subframe for \a sample at word \a index of a segment: bits 27..12 take
	the sample, and parity covers bits 4..31.
*/
static inline uint32
subframe(uint32 index, int16 sample)
{
	uint32 word = (sSilence[index] & 0x7fffffff)
		| ((uint32)(uint16)sample << 12);
	uint32 bits = word >> 4;
	bits ^= bits >> 16;
	bits ^= bits >> 8;
	bits ^= bits >> 4;
	bits ^= bits >> 2;
	bits ^= bits >> 1;
	return word | ((bits & 1) << 31);
}


static void
init_silence()
{
	for (uint32 frame = 0; frame < kFramesPerBuffer; frame++) {
		const uint32 position = frame % kFramesPerBlock;
		const uint32 status = (kChannelStatus[position / 8] >> (position % 8))
			& 1;
		for (uint32 channel = 0; channel < kChannelCount; channel++) {
			// Z starts a block, X is any other left sample, Y a right one
			uint32 preamble = channel != 0 ? 0x4 : (position == 0 ? 0x8 : 0x2);
			sSilence[frame * kChannelCount + channel] = preamble
				| (status << 30);
		}
	}
	for (uint32 i = 0; i < kSegmentWords; i++)
		sSilence[i] = subframe(i, 0);
}


static void
fill_silence(int32 segment)
{
	volatile uint32* words = sDevice.ring + segment * kSegmentWords;
	for (uint32 i = 0; i < kSegmentWords; i++)
		words[i] = sSilence[i];
}


static void
convert_buffer(int32 index)
{
	const int16* samples = sDevice.pcm + index * kSegmentWords;
	volatile uint32* words = sDevice.ring + index * kSegmentWords;
	for (uint32 i = 0; i < kSegmentWords; i++)
		words[i] = subframe(i, samples[i]);
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
	// plays now (see bcm2711_pwm).
	const int32 playing = ((dma_words_read() + kSegmentWords / 8)
		/ kSegmentWords) % kBufferCount;
	const int32 finished = (playing + 1) % kBufferCount;

	SpinLocker locker(sDevice.state_lock);

	// Silence what has been played, so that it does not play again if the
	// client does not deliver in time.
	fill_silence(finished);
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

	status_t status = start_mai();
	if (status != B_OK)
		return status;

	memset(sDevice.pcm, 0, kBufferCount * kSegmentWords * sizeof(int16));
	for (uint32 i = 0; i < kBufferCount; i++) {
		fill_silence(i);
		volatile dma_control_block& block = sDevice.control_blocks[i];
		block.transfer_info = kDmaInterruptEnable | kDmaWaitResponse
			| kDmaDestinationDreq | kDmaSourceIncrement
			| (kDmaDreqHdmi0 << kDmaPermapShift);
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
		stop_mai();
		return status;
	}

	status = arm_dma();
	if (status != B_OK) {
		reset_dma();
		stop_mai();
		remove_io_interrupt_handler(sDevice.interrupt, &dma_interrupt, NULL);
		return status;
	}

	sDevice.running = true;
	TRACE_ALWAYS("streaming at %" B_PRIu32 " Hz, DMA channel %" B_PRIu32
		", interrupt %" B_PRId32 ", pixel clock %" B_PRIu32 " kHz\n", kFrameRate,
		kDmaChannel, sDevice.interrupt, sDevice.pixel_clock);
	return B_OK;
}


static void
stop_engine()
{
	if (!sDevice.running)
		return;

	reset_dma();
	stop_mai();
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
		0 },
	{ 3, B_MULTI_OUTPUT_BUS, B_CHANNEL_RIGHT | B_CHANNEL_STEREO_BUS,
		0 },
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

	const size_t infoSize = description.info_size;
	const int32 requestCount = description.request_channel_count;
	memset(&description, 0, sizeof(description));
	description.info_size = infoSize;
	description.request_channel_count = requestCount;
	description.channels = channels;

	description.interface_version = B_CURRENT_INTERFACE_VERSION;
	description.interface_minimum = B_CURRENT_INTERFACE_VERSION;
	strlcpy(description.friendly_name, "Raspberry Pi HDMI",
		sizeof(description.friendly_name));
	strlcpy(description.vendor_info, "Haiku",
		sizeof(description.vendor_info));

	description.output_channel_count = kChannelCount;
	description.output_bus_channel_count = kChannelCount;
	description.output_rates = B_SR_48000;
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
	info.output.rate = B_SR_48000;
	info.output.format = B_FMT_16BIT;

	return user_memcpy(arg, &info, sizeof(info));
}


static status_t
list_mix_controls(void* arg)
{
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
			// Tried again on the next open; until then the client is paced
			// as if it played.
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
			stop_engine();
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
		const uint64 milliHertz = (uint64)(periods - 1) * kFramesPerBuffer
			* 1000000000 / elapsed;
		TRACE_ALWAYS("measured %" B_PRIu64 ".%03" B_PRIu64 " Hz over %"
			B_PRIu32 " periods, %" B_PRIu32 " underruns\n", milliHertz / 1000,
			milliHertz % 1000, periods, underruns);
		sDevice.rate_reported = true;
	}

	return user_memcpy(arg, &info, sizeof(info));
}


static status_t
buffer_force_stop()
{
	MutexLocker locker(sDevice.lock);
	if (!sDevice.running)
		return B_OK;

	InterruptsSpinLocker stateLocker(sDevice.state_lock);
	for (uint32 i = 0; i < kBufferCount; i++) {
		fill_silence(i);
		sDevice.filled[i] = false;
	}
	sDevice.primed = false;
	stateLocker.Unlock();

	sDevice.fill_cycle = -1;
	return B_OK;
}


//	#pragma mark - device hooks


static status_t
hdmi_audio_open(const char* name, uint32 flags, void** _cookie)
{
	MutexLocker locker(sDevice.lock);
	sDevice.open_count++;
	*_cookie = &sDevice;
	return B_OK;
}


static status_t
hdmi_audio_close(void* cookie)
{
	return B_OK;
}


static status_t
hdmi_audio_free(void* cookie)
{
	MutexLocker locker(sDevice.lock);
	if (--sDevice.open_count == 0) {
		stop_engine();
		sDevice.failed = false;
	}
	return B_OK;
}


static status_t
hdmi_audio_control(void* cookie, uint32 op, void* arg, size_t length)
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
			// one format only; B_MULTI_GET_GLOBAL_FORMAT tells what is in effect
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
hdmi_audio_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static status_t
hdmi_audio_write(void* cookie, off_t position, const void* buffer,
	size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static device_hooks sDeviceHooks = {
	hdmi_audio_open,
	hdmi_audio_close,
	hdmi_audio_free,
	hdmi_audio_control,
	hdmi_audio_read,
	hdmi_audio_write,
};

static const char* sDeviceNames[] = {
	DEVICE_NAME,
	NULL
};


//	#pragma mark - driver


/*!	The PCM buffers the media add-on writes, and the ring and control blocks
	for the DMA engine, in contiguous memory it reaches and mapped
	write-combining, as it does not snoop the CPU caches (see bcm2711_pwm).
*/
static status_t
allocate_buffers(int dmaNode)
{
	sDevice.pcm_area = create_area("bcm2711_hdmi_audio buffers",
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
	sDevice.ring_area = create_area_etc(B_SYSTEM_TEAM,
		"bcm2711_hdmi_audio dma", size, B_CONTIGUOUS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0, &virtualRestrictions,
		&physicalRestrictions, &address);
	if (sDevice.ring_area < 0)
		return sDevice.ring_area;

	physical_entry entry;
	uint32 count = 1;
	status_t status = get_memory_map_etc(B_SYSTEM_TEAM, address, B_PAGE_SIZE,
		&entry, &count);
	if (status == B_OK || status == B_BUFFER_OVERFLOW) {
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
	delete_area(sDevice.dvp_area);
	delete_area(sDevice.phy_area);
	delete_area(sDevice.packet_area);
	delete_area(sDevice.hd_area);
	delete_area(sDevice.core_area);
	mutex_destroy(&sDevice.lock);
}


extern "C" status_t
init_hardware(void)
{
	if (gFDT == NULL || fdt_node_check_compatible(gFDT, 0, "brcm,bcm2711") != 0
		|| find_compatible("brcm,bcm2711-hdmi0") < 0
		|| find_compatible("brcm,brcm2711-dvp") < 0
		|| find_compatible("brcm,bcm2835-dma") < 0) {
		return B_DEVICE_NOT_FOUND;
	}
	// no monitor: the port's registers must not be touched
	return hdmi_clock_runs() ? B_OK : B_DEVICE_NOT_FOUND;
}


extern "C" status_t
init_driver(void)
{
	memset(&sDevice, 0, sizeof(sDevice));
	sDevice.core_area = sDevice.hd_area = sDevice.packet_area
		= sDevice.phy_area = sDevice.dvp_area = sDevice.dma_area
		= sDevice.pcm_area = sDevice.ring_area = -1;
	sDevice.period_sem = -1;
	B_INITIALIZE_SPINLOCK(&sDevice.state_lock);
	mutex_init(&sDevice.lock, "bcm2711_hdmi_audio");

	if (!hdmi_clock_runs()) {
		free_resources();
		return B_DEVICE_NOT_FOUND;
	}

	const int hdmiNode = find_compatible("brcm,bcm2711-hdmi0");
	const int dmaNode = find_compatible("brcm,bcm2835-dma");
	uint64 hdBusAddress = 0;
	status_t status = map_registers(hdmiNode, "hdmi", 0, "hdmi0 core",
		sDevice.core_area, sDevice.core);
	if (status == B_OK) {
		status = map_registers(hdmiNode, "hd", 0, "hdmi hd", sDevice.hd_area,
			sDevice.hd, &hdBusAddress);
	}
	if (status == B_OK) {
		status = map_registers(hdmiNode, "packet", 0, "hdmi0 packet",
			sDevice.packet_area, sDevice.packet);
	}
	if (status == B_OK) {
		status = map_registers(hdmiNode, "phy", 0, "hdmi0 phy",
			sDevice.phy_area, sDevice.phy);
	}
	if (status == B_OK) {
		status = map_registers(find_compatible("brcm,brcm2711-dvp"), NULL, 0,
			"hdmi dvp", sDevice.dvp_area, sDevice.dvp);
	}
	if (status == B_OK && !port_takes_audio()) {
		TRACE_ALWAYS("the HDMI port carries no audio (DVI mode or no monitor)"
			"\n");
		status = B_DEVICE_NOT_FOUND;
	}
	if (status == B_OK) {
		status = map_registers(dmaNode, NULL, kDmaChannel * 0x100,
			"bcm2711_hdmi_audio dma", sDevice.dma_area, sDevice.dma);
	}
	if (status == B_OK)
		status = find_dma_interrupt(dmaNode, sDevice.interrupt);
	if (status == B_OK)
		status = allocate_buffers(dmaNode);
	if (status == B_OK) {
		sDevice.period_sem = create_sem(0, "bcm2711_hdmi_audio period");
		if (sDevice.period_sem < 0)
			status = sDevice.period_sem;
	}
	if (status != B_OK) {
		if (status != B_DEVICE_NOT_FOUND)
			ERROR("init failed: %s\n", strerror(status));
		free_resources();
		return status;
	}

	sDevice.fifo_bus_address = (uint32)hdBusAddress + kHdMaiDataOffset;
	sDevice.pixel_clock = measure_pixel_clock();
	init_silence();
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
