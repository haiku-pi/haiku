/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	Driver for the Broadcom GENET v5 Gigabit Ethernet controller of the
	Raspberry Pi 4 (BCM2711), with its external BCM54213PE PHY attached over
	RGMII and managed through the GENET's own UniMAC MDIO interface.

	The controller is described by the flattened device tree
	("brcm,bcm2711-genet-v5"); the driver attaches to that FDT node and
	publishes net/genet/<n> for the "ethernet" network device module.

	Design, see docs/design/genet-driver.md for the reasoning:
	- Only the descriptor ring (ring 16) is used, in both directions, with all
	  256 hardware descriptors. Every descriptor owns one fixed 2 KiB buffer;
	  received frames are copied out, frames to send are copied in.
	- The buffers live in one physically contiguous area mapped
	  non-cacheable, since the GENET does not snoop the CPU caches.
	- Ring 16 signals its completions on the first interrupt line
	  (INTRL2_0). A thread that runs out of work unmasks the interrupt it
	  needs and sleeps; the handler masks it again and wakes the thread, so a
	  burst of frames costs a single interrupt.
	- The link is polled on demand: the ethernet module asks for the link
	  state when the interface comes up and every second afterwards.
*/


#include <new>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include <ByteOrder.h>
#include <Drivers.h>
#include <KernelExport.h>
#include <bus/FDT.h>
#include <device_manager.h>
#include <net/if_media.h>

#include <arch/cpu.h>
#include <bcm2835_mailbox.h>
#include <condition_variable.h>
#include <ether_driver.h>
#include <lock.h>
#include <util/AutoLock.h>
#include <vm/vm.h>

#include "genet.h"


//#define TRACE_GENET
#ifdef TRACE_GENET
#	define TRACE(x...) dprintf("genet: " x)
#else
#	define TRACE(x...) ;
#endif
#define TRACE_ALWAYS(x...)	dprintf("genet: " x)
#define ERROR(x...)			dprintf("\33[33mgenet:\33[0m " x)


#define GENET_DRIVER_MODULE_NAME	"drivers/network/genet/driver_v1"
#define GENET_DEVICE_MODULE_NAME	"drivers/network/genet/device_v1"
#define GENET_DEVICE_ID_GENERATOR	"genet/device_id"


static const char* const kCompatible = "brcm,bcm2711-genet-v5";

// One buffer per descriptor, sized like Linux' RX_BUF_LENGTH; the hardware
// is told this size through DMA_RING_BUF_SIZE.
static const size_t kBufferSize = 2048;
static const uint32 kRxBufferCount = DMA_DESC_COUNT;
static const uint32 kTxBufferCount = DMA_DESC_COUNT;
// Ethernet frame sizes without the FCS, which the MAC strips on receive and
// appends on transmit.
static const size_t kMaxFrameSize = 1514;
static const size_t kMinFrameSize = 60;
static const size_t kHeaderSize = 14;

/*	The /scb bus "dma-ranges" of the Raspberry Pi 4 device tree has two
	entries: <0x4 0x7c000000  0x0 0xfc000000  0x0 0x03800000> (the peripheral
	window) and <0x0 0x00000000  0x0 0x00000000  0x4 0x00000000>, an identity
	mapping of the first 16 GiB. The GENET therefore reaches all of RAM with
	DMA address = physical address, and its descriptors hold 40-bit addresses.
	Staying below 4 GiB keeps the high address word zero and costs nothing.
	fdt_device_module_info::get_dma_window() cannot be used here: it reports
	only the first entry, which for /scb is the peripheral window.
*/
static const phys_addr_t kDmaHighAddress = 0x100000000LL;

// The PHY address on the Raspberry Pi 4 ("ethernet-phy@1" below the GENET's
// "mdio@e14" node). Following the "phy-handle" phandle would require the
// device node of the PHY to be initialized, which nothing guarantees, so the
// address is assumed, verified by reading the PHY ID, and the bus scanned if
// nothing answers there.
static const uint32 kDefaultPhyAddress = 1;

static const bigtime_t kMdioTimeout = 20000;
static const bigtime_t kDmaDisableTimeout = 5000;
static const bigtime_t kTxTimeout = 1000000;

/*	How long a reader sleeps before it looks at the ring again even without
	an interrupt. It bounds the latency of a frame whose completion interrupt
	went missing: the Phoenix-RTOS driver for the same controller observed
	frames that were in the ring without a pending interrupt at gigabit
	speed, which otherwise stay there until the next frame arrives (often a
	TCP retransmission a second later).
*/
static const bigtime_t kRxPollInterval = 10000;

/*	The same backstop for a writer waiting for a full transmit ring to drain.
	That ring 16 signals transmit completions with INTRL2_0 bit 16 comes from
	Linux and FreeBSD; unlike the receive bit it has not been seen working on
	this board yet, and a wrong bit should only make the wait slower.
*/
static const bigtime_t kTxPollInterval = 10000;

// VideoCore property tag: the board's MAC address (6 bytes, network order).
static const uint32 kTagGetBoardMacAddress = 0x00010003;

static const uint32 kMaxMulticastAddresses = 32;


static device_manager_info* sDeviceManager;


enum phy_mode {
	PHY_MODE_RGMII,
	PHY_MODE_RGMII_TXID,
	PHY_MODE_RGMII_RXID,
};


class GenetDevice {
public:
								GenetDevice(device_node* node);
								~GenetDevice();

			status_t			InitDriver();
			status_t			AllocateBuffers();
			void				FreeBuffers();

			status_t			Open();
			void				Close();

			status_t			Receive(void* buffer, size_t* _length,
									bool nonBlocking);
			status_t			Send(const void* buffer, size_t length,
									bool nonBlocking);

			const uint8*		MacAddress() const { return fMacAddress; }
			status_t			GetLinkState(ether_link_state* state);
			void				SetLinkStateSemaphore(sem_id semaphore)
									{ fLinkStateSemaphore = semaphore; }
			status_t			SetPromiscuous(bool promiscuous);
			status_t			AddMulticast(const uint8* address);
			status_t			RemoveMulticast(const uint8* address);

			device_node*		Node() const { return fNode; }

private:
	static	int32				_HandleInterrupt0(void* data);
	static	int32				_HandleInterrupt1(void* data);
			int32				_Interrupt0();
			int32				_Interrupt1();

	inline	uint32				_Read(uint32 offset) const;
	inline	void				_Write(uint32 offset, uint32 value);

			void				_ReadMacAddress();
			void				_WriteMacAddress();
			status_t			_ReadPhyMode();

			status_t			_Start();
			void				_Stop();
			void				_DisableInterrupts();
			void				_DisableDma();
			void				_ResetUmac();
			void				_InitUmac();
			void				_InitRxRing();
			void				_InitTxRing();
			void				_SetRxFilter();
			void				_WriteFilterAddress(uint32 index,
									const uint8* address);

			status_t			_MdioWait(uint32* _command);
			status_t			_MdioRead(uint32 reg, uint16* _value);
			status_t			_MdioWrite(uint32 reg, uint16 value);
			uint16				_PhyRead(uint32 reg);
			void				_PhyWrite(uint32 reg, uint16 value);
			uint16				_PhyReadMmd(uint32 device, uint32 reg);
			void				_PhyWriteMmd(uint32 device, uint32 reg,
									uint16 value);
			uint32				_ReadPhyId();
			status_t			_InitPhy();
			void				_ConfigureBroadcomPhy();
			void				_ConfigureRgmii();
			bool				_ReadLink(uint32* _speed, bool* _fullDuplex);
			void				_ConfigureMacForLink();

			device_node*		fNode;

			area_id				fRegistersArea;
			volatile uint8*		fRegisters;
			uint64				fRegistersPhysical;
			uint32				fIrq[2];
			bool				fInterruptsInstalled[2];

			area_id				fBuffersArea;
				// the transmit buffers
			area_id				fRxBuffersArea;
			uint8*				fRxBuffers;
			uint8*				fTxBuffers;
			phys_addr_t			fRxBuffersPhysical;
			phys_addr_t			fTxBuffersPhysical;

			uint8				fMacAddress[6];
			phy_mode			fPhyMode;
			uint32				fPhyAddress;
			bool				fBroadcomPhy;

			// fLock serializes Open() and Close(); it is taken before any of
			// the other locks.
			mutex				fLock;
			int32				fOpenCount;
			bool				fRunning;

			mutex				fRxLock;
			uint32				fRxConsumerIndex;
			ConditionVariable	fRxCondition;

			mutex				fTxLock;
			uint32				fTxProducerIndex;
			ConditionVariable	fTxCondition;

			// fLinkLock protects the MDIO interface, UMAC_CMD and the
			// receive filters.
			mutex				fLinkLock;
			bool				fLinkUp;
			uint32				fSpeed;
			bool				fFullDuplex;
			sem_id				fLinkStateSemaphore;

			bool				fPromiscuous;
			uint32				fMulticastCount;
			uint8				fMulticast[kMaxMulticastAddresses][6];
};


struct genet_handle {
	GenetDevice*	device;
	bool			nonBlocking;
};


static bool
is_valid_mac_address(const uint8* address)
{
	// neither all zeroes, nor all ones, nor a multicast address
	static const uint8 kZero[6] = {};
	static const uint8 kOnes[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	return memcmp(address, kZero, 6) != 0 && memcmp(address, kOnes, 6) != 0
		&& (address[0] & 0x01) == 0;
}


//	#pragma mark - GenetDevice


GenetDevice::GenetDevice(device_node* node)
	:
	fNode(node),
	fRegistersArea(-1),
	fRegisters(NULL),
	fRegistersPhysical(0),
	fBuffersArea(-1),
	fRxBuffersArea(-1),
	fRxBuffers(NULL),
	fTxBuffers(NULL),
	fRxBuffersPhysical(0),
	fTxBuffersPhysical(0),
	fPhyMode(PHY_MODE_RGMII_RXID),
	fPhyAddress(kDefaultPhyAddress),
	fBroadcomPhy(false),
	fOpenCount(0),
	fRunning(false),
	fRxConsumerIndex(0),
	fTxProducerIndex(0),
	fLinkUp(false),
	fSpeed(0),
	fFullDuplex(false),
	fLinkStateSemaphore(-1),
	fPromiscuous(false),
	fMulticastCount(0)
{
	fIrq[0] = fIrq[1] = 0;
	fInterruptsInstalled[0] = fInterruptsInstalled[1] = false;
	memset(fMacAddress, 0, sizeof(fMacAddress));

	mutex_init(&fLock, "genet");
	mutex_init(&fRxLock, "genet rx");
	mutex_init(&fTxLock, "genet tx");
	mutex_init(&fLinkLock, "genet link");
	fRxCondition.Init(this, "genet rx");
	fTxCondition.Init(this, "genet tx");
}


GenetDevice::~GenetDevice()
{
	if (fRegisters != NULL) {
		_DisableInterrupts();
		_DisableDma();
	}

	if (fInterruptsInstalled[0])
		remove_io_interrupt_handler(fIrq[0], _HandleInterrupt0, this);
	if (fInterruptsInstalled[1])
		remove_io_interrupt_handler(fIrq[1], _HandleInterrupt1, this);

	FreeBuffers();
	if (fRegistersArea >= 0)
		delete_area(fRegistersArea);

	mutex_destroy(&fLock);
	mutex_destroy(&fRxLock);
	mutex_destroy(&fTxLock);
	mutex_destroy(&fLinkLock);
}


/*!	Maps the registers, finds the interrupts, the MAC address and the PHY
	mode, and silences whatever DMA and interrupts the boot firmware left
	behind. The controller stays idle until the device is opened.
*/
status_t
GenetDevice::InitDriver()
{
	device_node* parent = sDeviceManager->get_parent_node(fNode);
	fdt_device_module_info* fdt;
	fdt_device* device;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&device);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	int length;
	const char* nodeStatus = (const char*)fdt->get_prop(device, "status",
		&length);
	if (nodeStatus != NULL && strcmp(nodeStatus, "okay") != 0
		&& strcmp(nodeStatus, "ok") != 0) {
		return B_DEVICE_NOT_FOUND;
	}

	uint64 size;
	if (!fdt->get_reg(device, 0, &fRegistersPhysical, &size)
		|| size < GENET_REGISTERS_SIZE) {
		ERROR("no usable registers\n");
		return B_DEVICE_NOT_FOUND;
	}

	// Line 0 carries the INTRL2_0 sources, among them the completions of
	// ring 16; line 1 carries INTRL2_1, the priority rings.
	for (uint32 i = 0; i < 2; i++) {
		uint64 irq;
		if (!fdt->get_interrupt(device, i, NULL, &irq) || irq > 0xffff) {
			ERROR("no interrupt %" B_PRIu32 "\n", i);
			return B_DEVICE_NOT_FOUND;
		}
		fIrq[i] = irq;
	}

	fRegistersArea = map_physical_memory("genet registers",
		fRegistersPhysical, GENET_REGISTERS_SIZE, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, (void**)&fRegisters);
	if (fRegistersArea < 0) {
		ERROR("cannot map registers\n");
		fRegisters = NULL;
		return fRegistersArea;
	}

	// GENET v5 reports major revision 6 (or 7 on some steppings); see
	// Linux bcmgenet_set_hw_params().
	uint32 revision = _Read(SYS_REV_CTRL);
	uint32 major = SYS_REV_MAJOR(revision);
	if (major != 6 && major != 7) {
		ERROR("unsupported GENET revision %#" B_PRIx32 "\n", revision);
		return B_DEVICE_NOT_FOUND;
	}

	status = _ReadPhyMode();
	if (status != B_OK)
		return status;

	// Stop whatever the firmware left running before anything else: a
	// receive DMA still active would write into memory the kernel now owns.
	_DisableInterrupts();
	_DisableDma();

	// before the UniMAC reset, which clears UMAC_MAC0/1
	_ReadMacAddress();

	for (uint32 i = 0; i < 2; i++) {
		status = install_io_interrupt_handler(fIrq[i],
			i == 0 ? _HandleInterrupt0 : _HandleInterrupt1, this, 0);
		if (status != B_OK) {
			ERROR("cannot install interrupt handler %" B_PRIu32 "\n", fIrq[i]);
			return status;
		}
		fInterruptsInstalled[i] = true;
	}

	TRACE_ALWAYS("GENET v5 (revision %#" B_PRIx32 ") at %#" B_PRIx64
		", interrupts %" B_PRIu32 "/%" B_PRIu32 ", MAC address "
		"%02x:%02x:%02x:%02x:%02x:%02x\n", revision, fRegistersPhysical,
		fIrq[0], fIrq[1], fMacAddress[0], fMacAddress[1], fMacAddress[2],
		fMacAddress[3], fMacAddress[4], fMacAddress[5]);
	return B_OK;
}


/*!	Allocates the buffers of the two rings, each ring's in one physically
	contiguous area.

	The GENET does not snoop the CPU caches. The transmit buffers are mapped
	non-cacheable (write-combining): the CPU only writes them, and nothing
	cached can then hide a frame from the device. The receive buffers stay
	cacheable, because reading frames out of non-cacheable memory limited
	receiving to about a quarter of the link: the CPU never writes them, so
	they never hold a dirty line, and Receive() invalidates a frame's lines
	after the device wrote it, before copying it out.
*/
status_t
GenetDevice::AllocateBuffers()
{
	if (fBuffersArea >= 0)
		return B_OK;

	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = kDmaHighAddress;
	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;

	void* rxAddress;
	fRxBuffersArea = create_area_etc(B_SYSTEM_TEAM, "genet receive buffers",
		kRxBufferCount * kBufferSize, B_CONTIGUOUS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0, &virtualRestrictions,
		&physicalRestrictions, &rxAddress);
	void* txAddress;
	fBuffersArea = create_area_etc(B_SYSTEM_TEAM, "genet transmit buffers",
		kTxBufferCount * kBufferSize, B_CONTIGUOUS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0, &virtualRestrictions,
		&physicalRestrictions, &txAddress);
	if (fRxBuffersArea < 0 || fBuffersArea < 0) {
		ERROR("cannot allocate the buffers\n");
		FreeBuffers();
		return B_NO_MEMORY;
	}

	physical_entry rxEntry, txEntry;
	status_t status = get_memory_map(rxAddress, kRxBufferCount * kBufferSize,
		&rxEntry, 1);
	if (status == B_OK) {
		status = get_memory_map(txAddress, kTxBufferCount * kBufferSize,
			&txEntry, 1);
	}
	if (status == B_OK) {
		// No cached copy may be written back over what the device writes
		arm64_dcache_flush_range((addr_t)rxAddress,
			kRxBufferCount * kBufferSize);
		arm64_dcache_flush_range((addr_t)txAddress,
			kTxBufferCount * kBufferSize);
		status = vm_set_area_memory_type(fBuffersArea, txEntry.address,
			B_WRITE_COMBINING_MEMORY);
	}
	if (status != B_OK) {
		ERROR("cannot set up the buffers\n");
		FreeBuffers();
		return status;
	}

	fRxBuffers = (uint8*)rxAddress;
	fTxBuffers = (uint8*)txAddress;
	fRxBuffersPhysical = rxEntry.address;
	fTxBuffersPhysical = txEntry.address;
	return B_OK;
}


void
GenetDevice::FreeBuffers()
{
	if (fRxBuffersArea >= 0)
		delete_area(fRxBuffersArea);
	if (fBuffersArea >= 0)
		delete_area(fBuffersArea);
	fRxBuffersArea = fBuffersArea = -1;
	fRxBuffers = fTxBuffers = NULL;
}


status_t
GenetDevice::Open()
{
	MutexLocker locker(fLock);

	if (fOpenCount == 0) {
		status_t status = _Start();
		if (status != B_OK)
			return status;
	}
	fOpenCount++;
	return B_OK;
}


void
GenetDevice::Close()
{
	MutexLocker locker(fLock);

	if (--fOpenCount == 0)
		_Stop();
}


/*!	Returns the next good frame from the receive ring, blocking until one
	arrives unless \a nonBlocking is set. Frames with errors and frames that
	do not fit into \a buffer are dropped.
*/
status_t
GenetDevice::Receive(void* buffer, size_t* _length, bool nonBlocking)
{
	while (true) {
		MutexLocker locker(fRxLock);
		if (!fRunning)
			return B_FILE_ERROR;

		uint32 producerIndex = _Read(RDMA_RING_REG(RDMA_PROD_INDEX))
			& DMA_INDEX_MASK;
		if (producerIndex != fRxConsumerIndex) {
			// The frame data and the descriptor status were written before
			// the producer index moved; read them only after it.
			arm64_dmb(ld);

			uint32 index = fRxConsumerIndex % kRxBufferCount;
			uint32 lengthStatus = _Read(RDMA_DESC(index)
				+ DMA_DESC_LENGTH_STATUS);
			uint32 length = (lengthStatus >> DMA_BUFLENGTH_SHIFT)
				& DMA_BUFLENGTH_MASK;

			status_t status = B_OK;
			size_t frameLength = 0;
			if ((lengthStatus & (DMA_SOP | DMA_EOP)) != (DMA_SOP | DMA_EOP)
				|| (lengthStatus & DMA_RX_ERROR_MASK) != 0
				|| length <= RX_ALIGN_PAD + kHeaderSize
				|| length > kBufferSize) {
				TRACE("dropping frame, status %#" B_PRIx32 "\n", lengthStatus);
				status = B_BAD_DATA;
			} else {
				frameLength = length - RX_ALIGN_PAD;
				if (frameLength > *_length)
					status = B_BUFFER_OVERFLOW;
				else {
					// Drop whatever the CPU cached of this buffer before the
					// device wrote it (lines are never dirty: see
					// AllocateBuffers())
					uint8* frame = fRxBuffers + index * kBufferSize;
					arm64_dcache_flush_range((addr_t)frame, length);
					status = user_memcpy(buffer, frame + RX_ALIGN_PAD,
						frameLength);
				}
			}

			// The copy must be complete before the buffer goes back to the
			// device.
			arm64_dsb(sy);
			fRxConsumerIndex = (fRxConsumerIndex + 1) & DMA_INDEX_MASK;
			_Write(RDMA_RING_REG(RDMA_CONS_INDEX), fRxConsumerIndex);

			if (status == B_OK) {
				*_length = frameLength;
				return B_OK;
			}
			if (status == B_BAD_ADDRESS) {
				*_length = 0;
				return status;
			}
			continue;
		}

		if (nonBlocking) {
			*_length = 0;
			return B_WOULD_BLOCK;
		}

		// The ring is empty: arm the interrupt and sleep. Waiting is
		// registered and the interrupt unmasked before the ring is checked
		// once more, so a frame that arrives in between is not missed.
		ConditionVariableEntry entry;
		fRxCondition.Add(&entry);
		_Write(GENET_INTRL2_0 + INTRL2_CPU_CLEAR, UMAC_IRQ_RXDMA_MBDONE);
		_Write(GENET_INTRL2_0 + INTRL2_CPU_MASK_CLEAR, UMAC_IRQ_RXDMA_MBDONE);
		if ((_Read(RDMA_RING_REG(RDMA_PROD_INDEX)) & DMA_INDEX_MASK)
				!= fRxConsumerIndex) {
			continue;
		}
		locker.Unlock();

		status_t status = entry.Wait(B_RELATIVE_TIMEOUT | B_CAN_INTERRUPT,
			kRxPollInterval);
		if (status == B_INTERRUPTED) {
			*_length = 0;
			return status;
		}
		// B_TIMED_OUT is the poll backstop, and a wakeup from _Stop() is
		// handled by the fRunning check: just look again.
	}
}


/*!	Queues one frame for transmission. Returns as soon as the frame is in a
	transmit buffer; only a full ring makes it wait.
*/
status_t
GenetDevice::Send(const void* buffer, size_t length, bool nonBlocking)
{
	if (length < kHeaderSize || length > kMaxFrameSize)
		return B_BAD_VALUE;

	MutexLocker locker(fTxLock);

	bigtime_t stallStart = 0;
	uint32 stalledConsumerIndex = 0;
	while (true) {
		if (!fRunning)
			return B_FILE_ERROR;
		if (!fLinkUp) {
			// the MAC stays in reset until the first link up
			return ENETDOWN;
		}

		// The descriptor and buffer for fTxProducerIndex were last used by
		// the frame kTxBufferCount frames ago. Keeping one descriptor in
		// reserve leaves no doubt that frame has been consumed.
		uint32 consumerIndex = _Read(TDMA_RING_REG(TDMA_CONS_INDEX))
			& DMA_INDEX_MASK;
		uint32 inFlight = (fTxProducerIndex - consumerIndex) & DMA_INDEX_MASK;
		if (inFlight < kTxBufferCount - 1)
			break;

		if (nonBlocking)
			return B_WOULD_BLOCK;

		// Only give up when the hardware made no progress at all for a while.
		if (stallStart == 0 || consumerIndex != stalledConsumerIndex) {
			stallStart = system_time();
			stalledConsumerIndex = consumerIndex;
		} else if (system_time() - stallStart > kTxTimeout) {
			ERROR("transmit ring stalled\n");
			return B_TIMED_OUT;
		}

		ConditionVariableEntry entry;
		fTxCondition.Add(&entry);
		_Write(GENET_INTRL2_0 + INTRL2_CPU_CLEAR, UMAC_IRQ_TXDMA_MBDONE);
		_Write(GENET_INTRL2_0 + INTRL2_CPU_MASK_CLEAR, UMAC_IRQ_TXDMA_MBDONE);
		consumerIndex = _Read(TDMA_RING_REG(TDMA_CONS_INDEX)) & DMA_INDEX_MASK;
		if (((fTxProducerIndex - consumerIndex) & DMA_INDEX_MASK)
				< kTxBufferCount - 1) {
			continue;
		}

		locker.Unlock();
		status_t status = entry.Wait(B_RELATIVE_TIMEOUT | B_CAN_INTERRUPT,
			kTxPollInterval);
		locker.Lock();
		if (status == B_INTERRUPTED)
			return status;
	}

	uint32 index = fTxProducerIndex % kTxBufferCount;
	uint8* slot = fTxBuffers + index * kBufferSize;
	if (user_memcpy(slot, buffer, length) != B_OK)
		return B_BAD_ADDRESS;
	if (length < kMinFrameSize) {
		memset(slot + length, 0, kMinFrameSize - length);
		length = kMinFrameSize;
	}

	// The frame must be in memory before the device can see the descriptor
	// (the buffer is normal memory, the registers are device memory, and
	// the two are not ordered against each other).
	arm64_dsb(sy);

	phys_addr_t address = fTxBuffersPhysical + index * kBufferSize;
	_Write(TDMA_DESC(index) + DMA_DESC_ADDRESS_LO, (uint32)address);
	_Write(TDMA_DESC(index) + DMA_DESC_ADDRESS_HI, (uint32)(address >> 32));
	_Write(TDMA_DESC(index) + DMA_DESC_LENGTH_STATUS,
		(length << DMA_BUFLENGTH_SHIFT) | DMA_SOP | DMA_EOP | DMA_TX_APPEND_CRC
			| (DMA_TX_QTAG_MASK << DMA_TX_QTAG_SHIFT));

	fTxProducerIndex = (fTxProducerIndex + 1) & DMA_INDEX_MASK;
	_Write(TDMA_RING_REG(TDMA_PROD_INDEX), fTxProducerIndex);
	return B_OK;
}


/*!	Reads the link state from the PHY and, when it changed, reprograms the
	MAC for it. Called by the ethernet module when the interface comes up and
	once a second afterwards, which makes a separate link poller unnecessary.
*/
status_t
GenetDevice::GetLinkState(ether_link_state* state)
{
	MutexLocker locker(fLinkLock);

	if (fRunning) {
		uint32 speed = 0;
		bool fullDuplex = false;
		bool up = _ReadLink(&speed, &fullDuplex);
		if (up != fLinkUp || (up && (speed != fSpeed
				|| fullDuplex != fFullDuplex))) {
			fLinkUp = up;
			fSpeed = speed;
			fFullDuplex = fullDuplex;
			_ConfigureMacForLink();

			if (up) {
				TRACE_ALWAYS("link up, %" B_PRIu32 " Mbit/s, %s duplex\n",
					speed, fullDuplex ? "full" : "half");
			} else
				TRACE_ALWAYS("link down\n");

			if (fLinkStateSemaphore >= 0)
				release_sem_etc(fLinkStateSemaphore, 1, B_DO_NOT_RESCHEDULE);
		}
	}

	state->media = IFM_ETHER;
	state->speed = 0;
	state->quality = 0;
	if (fRunning && fLinkUp) {
		state->media |= IFM_ACTIVE
			| (fFullDuplex ? IFM_FULL_DUPLEX : IFM_HALF_DUPLEX);
		switch (fSpeed) {
			case 1000:
				state->media |= IFM_1000_T;
				break;
			case 100:
				state->media |= IFM_100_TX;
				break;
			default:
				state->media |= IFM_10_T;
				break;
		}
		state->speed = (uint64)fSpeed * 1000000;
		state->quality = 1000;
	}
	return B_OK;
}


status_t
GenetDevice::SetPromiscuous(bool promiscuous)
{
	MutexLocker locker(fLinkLock);
	fPromiscuous = promiscuous;
	if (fRunning)
		_SetRxFilter();
	return B_OK;
}


status_t
GenetDevice::AddMulticast(const uint8* address)
{
	MutexLocker locker(fLinkLock);

	for (uint32 i = 0; i < fMulticastCount; i++) {
		if (memcmp(fMulticast[i], address, 6) == 0)
			return B_OK;
	}
	if (fMulticastCount == kMaxMulticastAddresses)
		return B_NO_MEMORY;

	memcpy(fMulticast[fMulticastCount++], address, 6);
	if (fRunning)
		_SetRxFilter();
	return B_OK;
}


status_t
GenetDevice::RemoveMulticast(const uint8* address)
{
	MutexLocker locker(fLinkLock);

	for (uint32 i = 0; i < fMulticastCount; i++) {
		if (memcmp(fMulticast[i], address, 6) != 0)
			continue;

		memmove(fMulticast[i], fMulticast[i + 1],
			(fMulticastCount - i - 1) * sizeof(fMulticast[0]));
		fMulticastCount--;
		if (fRunning)
			_SetRxFilter();
		return B_OK;
	}
	return B_BAD_VALUE;
}


//	#pragma mark - interrupts


/*static*/ int32
GenetDevice::_HandleInterrupt0(void* data)
{
	return ((GenetDevice*)data)->_Interrupt0();
}


/*static*/ int32
GenetDevice::_HandleInterrupt1(void* data)
{
	return ((GenetDevice*)data)->_Interrupt1();
}


int32
GenetDevice::_Interrupt0()
{
	uint32 status = _Read(GENET_INTRL2_0 + INTRL2_CPU_STAT)
		& ~_Read(GENET_INTRL2_0 + INTRL2_CPU_MASK_STATUS);
	if (status == 0)
		return B_UNHANDLED_INTERRUPT;

	// Mask what fired; the woken thread unmasks it again once it has run
	// out of work.
	_Write(GENET_INTRL2_0 + INTRL2_CPU_MASK_SET, status);
	_Write(GENET_INTRL2_0 + INTRL2_CPU_CLEAR, status);

	if ((status & UMAC_IRQ_RXDMA_MBDONE) != 0)
		fRxCondition.NotifyAll();
	if ((status & UMAC_IRQ_TXDMA_MBDONE) != 0)
		fTxCondition.NotifyAll();

	return B_INVOKE_SCHEDULER;
}


/*!	INTRL2_1 belongs to the priority rings, which are not used, so all its
	sources stay masked. Anything that still shows up is acknowledged and
	masked so that the level-triggered line cannot keep firing.
*/
int32
GenetDevice::_Interrupt1()
{
	uint32 status = _Read(GENET_INTRL2_1 + INTRL2_CPU_STAT)
		& ~_Read(GENET_INTRL2_1 + INTRL2_CPU_MASK_STATUS);
	if (status == 0)
		return B_UNHANDLED_INTERRUPT;

	_Write(GENET_INTRL2_1 + INTRL2_CPU_MASK_SET, status);
	_Write(GENET_INTRL2_1 + INTRL2_CPU_CLEAR, status);
	return B_HANDLED_INTERRUPT;
}


//	#pragma mark - controller setup


inline uint32
GenetDevice::_Read(uint32 offset) const
{
	return *(volatile uint32*)(fRegisters + offset);
}


inline void
GenetDevice::_Write(uint32 offset, uint32 value)
{
	*(volatile uint32*)(fRegisters + offset) = value;
}


/*!	The MAC address, in order of preference: from the device tree (the
	firmware fills in "local-mac-address"), from the UniMAC address
	registers (where a network boot loader leaves it), from the VideoCore
	firmware, and as a last resort a random locally administered one.
*/
void
GenetDevice::_ReadMacAddress()
{
	device_node* parent = sDeviceManager->get_parent_node(fNode);
	fdt_device_module_info* fdt;
	fdt_device* device;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&device);
	sDeviceManager->put_node(parent);

	static const char* const kProperties[] = {
		"local-mac-address",
		"mac-address",
	};
	for (size_t i = 0; status == B_OK && i < B_COUNT_OF(kProperties); i++) {
		int length;
		const uint8* address = (const uint8*)fdt->get_prop(device,
			kProperties[i], &length);
		if (address != NULL && length == 6 && is_valid_mac_address(address)) {
			memcpy(fMacAddress, address, 6);
			return;
		}
	}

	// see Linux bcmgenet_get_hw_addr()
	uint32 high = _Read(UMAC_MAC0);
	uint32 low = _Read(UMAC_MAC1);
	uint8 address[6] = {
		(uint8)(high >> 24), (uint8)(high >> 16), (uint8)(high >> 8),
		(uint8)high, (uint8)(low >> 8), (uint8)low
	};
	if (is_valid_mac_address(address)) {
		memcpy(fMacAddress, address, 6);
		return;
	}

	bcm2835_mailbox_module_info* mailbox;
	if (get_module(BCM2835_MAILBOX_MODULE_NAME, (module_info**)&mailbox)
			== B_OK) {
		// The response is the 6 address bytes in network order, padded to
		// a whole number of words.
		uint8 values[8] = {};
		status = mailbox->property_tag(kTagGetBoardMacAddress, values,
			sizeof(values), 0, NULL);
		put_module(BCM2835_MAILBOX_MODULE_NAME);
		if (status == B_OK && is_valid_mac_address(values)) {
			memcpy(fMacAddress, values, 6);
			return;
		}
	}

	ERROR("no MAC address found, using a random one\n");
	uint64 seed = system_time() ^ fRegistersPhysical;
	for (int i = 0; i < 6; i++) {
		seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
		fMacAddress[i] = seed >> 56;
	}
	fMacAddress[0] = (fMacAddress[0] & 0xfe) | 0x02;
		// unicast, locally administered
}


void
GenetDevice::_WriteMacAddress()
{
	// see Linux bcmgenet_set_hw_addr()
	_Write(UMAC_MAC0, ((uint32)fMacAddress[0] << 24)
		| ((uint32)fMacAddress[1] << 16) | ((uint32)fMacAddress[2] << 8)
		| fMacAddress[3]);
	_Write(UMAC_MAC1, ((uint32)fMacAddress[4] << 8) | fMacAddress[5]);
}


status_t
GenetDevice::_ReadPhyMode()
{
	device_node* parent = sDeviceManager->get_parent_node(fNode);
	fdt_device_module_info* fdt;
	fdt_device* device;
	status_t status = sDeviceManager->get_driver(parent,
		(driver_module_info**)&fdt, (void**)&device);
	sDeviceManager->put_node(parent);
	if (status != B_OK)
		return status;

	int length;
	const char* mode = (const char*)fdt->get_prop(device, "phy-mode", &length);
	if (mode == NULL) {
		mode = (const char*)fdt->get_prop(device, "phy-connection-type",
			&length);
	}
	if (mode == NULL || strcmp(mode, "rgmii-rxid") == 0)
		fPhyMode = PHY_MODE_RGMII_RXID;
	else if (strcmp(mode, "rgmii-txid") == 0)
		fPhyMode = PHY_MODE_RGMII_TXID;
	else if (strcmp(mode, "rgmii") == 0)
		fPhyMode = PHY_MODE_RGMII;
	else {
		ERROR("unsupported phy-mode \"%s\"\n", mode);
		return B_NOT_SUPPORTED;
	}
	return B_OK;
}


/*!	Brings the controller up; the sequence follows Linux bcmgenet_open():
	UniMAC reset and setup, DMA rings, then PHY. The MAC itself is left in
	reset until _ConfigureMacForLink() sees the first link up.
*/
status_t
GenetDevice::_Start()
{
	if (fRxBuffers == NULL)
		return B_NO_INIT;

	_DisableInterrupts();
	_DisableDma();
	_ResetUmac();
	_InitUmac();
	_WriteMacAddress();

	// Flush the transmit and the receive data path, as Linux
	// bcmgenet_init_dma() does.
	_Write(UMAC_TX_FLUSH, 1);
	spin(10);
	_Write(UMAC_TX_FLUSH, 0);
	uint32 flush = _Read(SYS_RBUF_FLUSH_CTRL);
	_Write(SYS_RBUF_FLUSH_CTRL, flush | RBUF_FLUSH_DMA);
	spin(10);
	_Write(SYS_RBUF_FLUSH_CTRL, flush);
	spin(10);

	_InitRxRing();
	_InitTxRing();
	_Write(RDMA_REG(DMA_CTRL), _Read(RDMA_REG(DMA_CTRL)) | DMA_EN);
	_Write(TDMA_REG(DMA_CTRL), _Read(TDMA_REG(DMA_CTRL)) | DMA_EN);

	MutexLocker linkLocker(fLinkLock);
	fLinkUp = false;
	fSpeed = 0;
	fFullDuplex = false;
	_SetRxFilter();
	_ConfigureRgmii();
	status_t status = _InitPhy();
	if (status != B_OK) {
		linkLocker.Unlock();
		_DisableDma();
		return status;
	}

	{
		MutexLocker rxLocker(fRxLock);
		MutexLocker txLocker(fTxLock);
		fRunning = true;
	}
	return B_OK;
}


/*!	Stops the controller in the order of Linux bcmgenet_netif_stop(), and
	wakes up the readers and writers, which then find fRunning cleared.
*/
void
GenetDevice::_Stop()
{
	{
		MutexLocker rxLocker(fRxLock);
		MutexLocker txLocker(fTxLock);
		fRunning = false;
	}
	fRxCondition.NotifyAll(B_FILE_ERROR);
	fTxCondition.NotifyAll(B_FILE_ERROR);

	// Taking the locks waits for a Receive() or Send() still at work.
	MutexLocker linkLocker(fLinkLock);
	MutexLocker rxLocker(fRxLock);
	MutexLocker txLocker(fTxLock);

	_DisableInterrupts();

	uint32 command = _Read(UMAC_CMD);
	if ((command & CMD_SW_RESET) == 0) {
		_Write(UMAC_CMD, command & ~CMD_RX_EN);
		// the MAC stops at a frame boundary: allow for a full-size frame
		snooze(2000);
	}
	_DisableDma();
	command = _Read(UMAC_CMD);
	if ((command & CMD_SW_RESET) == 0)
		_Write(UMAC_CMD, command & ~CMD_TX_EN);

	_Write(EXT_RGMII_OOB_CTRL, _Read(EXT_RGMII_OOB_CTRL) & ~RGMII_LINK);
	fLinkUp = false;
}


void
GenetDevice::_DisableInterrupts()
{
	// Linux bcmgenet_intr_disable()
	_Write(GENET_INTRL2_0 + INTRL2_CPU_MASK_SET, 0xffffffff);
	_Write(GENET_INTRL2_0 + INTRL2_CPU_CLEAR, 0xffffffff);
	_Write(GENET_INTRL2_1 + INTRL2_CPU_MASK_SET, 0xffffffff);
	_Write(GENET_INTRL2_1 + INTRL2_CPU_CLEAR, 0xffffffff);
}


/*!	Disables both DMA engines and all 17 rings, then waits for the status
	register to report them stopped (Linux bcmgenet_tdma_disable() and
	bcmgenet_rdma_disable(), here for every ring, since the firmware may have
	used others than ring 16).
*/
void
GenetDevice::_DisableDma()
{
	const uint32 mask = (((1 << (DMA_DEFAULT_RING + 1)) - 1)
		<< DMA_RING_BUF_EN_SHIFT) | DMA_EN;
	const uint32 controls[] = { TDMA_REG(DMA_CTRL), RDMA_REG(DMA_CTRL) };
	const uint32 statuses[] = { TDMA_REG(DMA_STATUS), RDMA_REG(DMA_STATUS) };

	for (int i = 0; i < 2; i++) {
		_Write(controls[i], _Read(controls[i]) & ~mask);

		bigtime_t timeout = system_time() + kDmaDisableTimeout;
		while ((_Read(statuses[i]) & mask) != mask) {
			if (system_time() > timeout) {
				ERROR("%s DMA did not stop\n", i == 0 ? "transmit" : "receive");
				break;
			}
			spin(1);
		}
	}
}


void
GenetDevice::_ResetUmac()
{
	// Linux bcmgenet_umac_reset()
	uint32 flush = _Read(SYS_RBUF_FLUSH_CTRL);
	_Write(SYS_RBUF_FLUSH_CTRL, flush | RBUF_FLUSH_UMAC);
	spin(10);
	_Write(SYS_RBUF_FLUSH_CTRL, flush & ~RBUF_FLUSH_UMAC);
	spin(10);

	// Linux reset_umac(): the MAC stays in software reset (receiver and
	// transmitter off) until the link is up.
	_Write(SYS_RBUF_FLUSH_CTRL, 0);
	spin(10);
	_Write(UMAC_CMD, CMD_SW_RESET);
	spin(2);
}


/*!	Linux init_umac(), with two deliberate differences: no 64-byte status
	blocks in either direction (no checksum offload is used, and the
	Phoenix-RTOS driver traced receive losses at gigabit speed to the receive
	status block), and the receive checksum checker left off with them.
*/
void
GenetDevice::_InitUmac()
{
	_Write(UMAC_MIB_CTRL, MIB_RESET_RX | MIB_RESET_TX | MIB_RESET_RUNT);
	_Write(UMAC_MIB_CTRL, 0);

	// The reset clears it; with 0 every frame is dropped.
	_Write(UMAC_MAX_FRAME_LEN, GENET_MAX_MTU_SIZE);

	_Write(TBUF_CTRL, _Read(TBUF_CTRL) & ~TBUF_64B_EN);
	_Write(RBUF_CTRL, (_Read(RBUF_CTRL) & ~RBUF_64B_EN) | RBUF_ALIGN_2B);
	_Write(RBUF_CHK_CTRL, 0);
	_Write(RBUF_TBUF_SIZE_CTRL, 1);

	// No Energy Efficient Ethernet on the MAC side either (see
	// _ConfigureBroadcomPhy()): its power saving can lose the first frame
	// after an idle period.
	_Write(RBUF_ENERGY_CTRL,
		_Read(RBUF_ENERGY_CTRL) & ~(RBUF_EEE_EN | RBUF_PM_EN));
	_Write(UMAC_EEE_CTRL, _Read(UMAC_EEE_CTRL) & ~EEE_EN);

	// No hardware filters: every frame goes to ring 16 (Linux
	// bcmgenet_hfb_clear(), without its rule that steers to ring 0).
	_Write(HFB_CTRL, 0);
	_Write(HFB_FLT_ENABLE, 0);
	_Write(HFB_FLT_ENABLE + 4, 0);
	for (uint32 i = 0; i < 8; i++)
		_Write(RDMA_REG(DMA_INDEX2RING(i)), 0);
}


/*!	Ring 16 over all 256 descriptors; each descriptor gets its own buffer
	for good. (Sharing buffers between descriptors defeats the ring's flow
	control: the Phoenix-RTOS driver once aliased 256 descriptors onto 16
	buffers, and the hardware overwrote frames not yet read.)
	The register values follow Linux bcmgenet_init_rx_ring() and
	bcmgenet_set_rx_coalesce().
*/
void
GenetDevice::_InitRxRing()
{
	for (uint32 i = 0; i < kRxBufferCount; i++) {
		phys_addr_t address = fRxBuffersPhysical + i * kBufferSize;
		_Write(RDMA_DESC(i) + DMA_DESC_ADDRESS_LO, (uint32)address);
		_Write(RDMA_DESC(i) + DMA_DESC_ADDRESS_HI, (uint32)(address >> 32));
		_Write(RDMA_DESC(i) + DMA_DESC_LENGTH_STATUS, 0);
	}

	_Write(RDMA_REG(DMA_SCB_BURST_SIZE), DMA_BURST_LENGTH_BCM2711);

	_Write(RDMA_RING_REG(RDMA_PROD_INDEX), 0);
	_Write(RDMA_RING_REG(RDMA_CONS_INDEX), 0);
	_Write(RDMA_RING_REG(DMA_RING_BUF_SIZE),
		(kRxBufferCount << DMA_RING_SIZE_SHIFT) | kBufferSize);
	_Write(RDMA_RING_REG(RDMA_XON_XOFF_THRESH),
		(DMA_FC_THRESH_LO << DMA_XOFF_THRESHOLD_SHIFT) | DMA_FC_THRESH_HI);
	_Write(RDMA_RING_REG(DMA_START_ADDR), 0);
	_Write(RDMA_RING_REG(RDMA_READ_PTR), 0);
	_Write(RDMA_RING_REG(RDMA_WRITE_PTR), 0);
	_Write(RDMA_RING_REG(DMA_END_ADDR), kRxBufferCount * DMA_DESC_WORDS - 1);

	// An interrupt per frame, and a hardware timeout that raises it again
	// for a frame left in the ring.
	_Write(RDMA_RING_REG(DMA_MBUF_DONE_THRESH), 1);
	_Write(RDMA_REG(DMA_RING16_TIMEOUT),
		(_Read(RDMA_REG(DMA_RING16_TIMEOUT)) & ~0xffff)
			| DMA_RX_TIMEOUT_TICKS);

	_Write(RDMA_REG(DMA_RING_CFG),
		_Read(RDMA_REG(DMA_RING_CFG)) | (1 << DMA_DEFAULT_RING));
	_Write(RDMA_REG(DMA_CTRL), _Read(RDMA_REG(DMA_CTRL))
		| (1 << (DMA_DEFAULT_RING + DMA_RING_BUF_EN_SHIFT)));

	fRxConsumerIndex = 0;
}


//! Linux bcmgenet_init_tx_ring(), for ring 16 over all 256 descriptors.
void
GenetDevice::_InitTxRing()
{
	_Write(TDMA_REG(DMA_SCB_BURST_SIZE), DMA_BURST_LENGTH_BCM2711);

	_Write(TDMA_RING_REG(TDMA_PROD_INDEX), 0);
	_Write(TDMA_RING_REG(TDMA_CONS_INDEX), 0);
	// The transmit interrupt is only unmasked while a writer waits for a
	// free descriptor; it should come as soon as one is.
	_Write(TDMA_RING_REG(DMA_MBUF_DONE_THRESH), 1);
	_Write(TDMA_RING_REG(TDMA_FLOW_PERIOD), 0);
	_Write(TDMA_RING_REG(DMA_RING_BUF_SIZE),
		(kTxBufferCount << DMA_RING_SIZE_SHIFT) | kBufferSize);
	_Write(TDMA_RING_REG(DMA_START_ADDR), 0);
	_Write(TDMA_RING_REG(TDMA_READ_PTR), 0);
	_Write(TDMA_RING_REG(TDMA_WRITE_PTR), 0);
	_Write(TDMA_RING_REG(DMA_END_ADDR), kTxBufferCount * DMA_DESC_WORDS - 1);

	_Write(TDMA_REG(DMA_RING_CFG),
		_Read(TDMA_REG(DMA_RING_CFG)) | (1 << DMA_DEFAULT_RING));
	_Write(TDMA_REG(DMA_CTRL), _Read(TDMA_REG(DMA_CTRL))
		| (1 << (DMA_DEFAULT_RING + DMA_RING_BUF_EN_SHIFT)));

	fTxProducerIndex = 0;
}


/*!	Programs the MAC destination filters with the broadcast address, our
	own address and the multicast addresses, or switches to promiscuous mode
	when asked to or when they do not fit (Linux bcmgenet_set_rx_mode()).
	Must be called with fLinkLock held.
*/
void
GenetDevice::_SetRxFilter()
{
	static const uint8 kBroadcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

	uint32 count = 2 + fMulticastCount;
	bool promiscuous = fPromiscuous || count > MDF_FILTER_COUNT;

	// UMAC_CMD is only written when it changes: rewriting it can drop a
	// frame that is being received (observed by the Phoenix-RTOS driver).
	uint32 command = _Read(UMAC_CMD);
	uint32 newCommand = promiscuous
		? command | CMD_PROMISC : command & ~CMD_PROMISC;
	if (newCommand != command)
		_Write(UMAC_CMD, newCommand);

	if (promiscuous) {
		_Write(UMAC_MDF_CTRL, 0);
		return;
	}

	_WriteFilterAddress(0, kBroadcast);
	_WriteFilterAddress(1, fMacAddress);
	for (uint32 i = 0; i < fMulticastCount; i++)
		_WriteFilterAddress(2 + i, fMulticast[i]);

	// filter n is enabled by bit 16 - n
	uint32 enable = 0;
	for (uint32 i = 0; i < count; i++)
		enable |= 1 << (MDF_FILTER_COUNT - 1 - i);
	_Write(UMAC_MDF_CTRL, enable);
}


void
GenetDevice::_WriteFilterAddress(uint32 index, const uint8* address)
{
	_Write(UMAC_MDF_ADDR + index * 8,
		((uint32)address[0] << 8) | address[1]);
	_Write(UMAC_MDF_ADDR + index * 8 + 4,
		((uint32)address[2] << 24) | ((uint32)address[3] << 16)
			| ((uint32)address[4] << 8) | address[5]);
}


//	#pragma mark - MDIO and PHY


status_t
GenetDevice::_MdioWait(uint32* _command)
{
	bigtime_t timeout = system_time() + kMdioTimeout;
	while (true) {
		uint32 command = _Read(UMAC_MDIO_CMD);
		if ((command & MDIO_START_BUSY) == 0) {
			*_command = command;
			return B_OK;
		}
		if (system_time() > timeout)
			return B_TIMED_OUT;
		spin(5);
	}
}


/*!	The UniMAC MDIO interface (Linux mdio-bcm-unimac.c): the command is
	written first, then again with the start bit.
*/
status_t
GenetDevice::_MdioRead(uint32 reg, uint16* _value)
{
	uint32 command = MDIO_RD | (fPhyAddress << MDIO_PMD_SHIFT)
		| (reg << MDIO_REG_SHIFT);
	_Write(UMAC_MDIO_CMD, command);
	_Write(UMAC_MDIO_CMD, command | MDIO_START_BUSY);

	status_t status = _MdioWait(&command);
	if (status != B_OK)
		return status;
	if ((command & MDIO_READ_FAIL) != 0)
		return B_IO_ERROR;

	*_value = command & MDIO_DATA_MASK;
	return B_OK;
}


status_t
GenetDevice::_MdioWrite(uint32 reg, uint16 value)
{
	uint32 command = MDIO_WR | (fPhyAddress << MDIO_PMD_SHIFT)
		| (reg << MDIO_REG_SHIFT) | value;
	_Write(UMAC_MDIO_CMD, command);
	_Write(UMAC_MDIO_CMD, command | MDIO_START_BUSY);

	return _MdioWait(&command);
}


uint16
GenetDevice::_PhyRead(uint32 reg)
{
	uint16 value;
	if (_MdioRead(reg, &value) != B_OK)
		return 0xffff;
	return value;
}


void
GenetDevice::_PhyWrite(uint32 reg, uint16 value)
{
	if (_MdioWrite(reg, value) != B_OK)
		ERROR("MDIO write to register %#" B_PRIx32 " failed\n", reg);
}


//! Clause 45 register access through clause 22 (IEEE 802.3 annex 22D).
uint16
GenetDevice::_PhyReadMmd(uint32 device, uint32 reg)
{
	_PhyWrite(MII_MMD_CTRL, device);
	_PhyWrite(MII_MMD_DATA, reg);
	_PhyWrite(MII_MMD_CTRL, MMD_CTRL_DATA_NO_INC | device);
	return _PhyRead(MII_MMD_DATA);
}


void
GenetDevice::_PhyWriteMmd(uint32 device, uint32 reg, uint16 value)
{
	_PhyWrite(MII_MMD_CTRL, device);
	_PhyWrite(MII_MMD_DATA, reg);
	_PhyWrite(MII_MMD_CTRL, MMD_CTRL_DATA_NO_INC | device);
	_PhyWrite(MII_MMD_DATA, value);
}


uint32
GenetDevice::_ReadPhyId()
{
	uint16 id1, id2;
	if (_MdioRead(MII_PHYSID1, &id1) != B_OK
		|| _MdioRead(MII_PHYSID2, &id2) != B_OK) {
		return 0;
	}
	return ((uint32)id1 << 16) | id2;
}


/*!	Finds the PHY, configures it and restarts autonegotiation. Like Linux,
	which has no soft reset for this PHY, the PHY is not reset: the settings
	below are all that matter, and a reset would undo the boot firmware's
	setup only to redo it.
	Must be called with fLinkLock held.
*/
status_t
GenetDevice::_InitPhy()
{
	fPhyAddress = kDefaultPhyAddress;
	uint32 id = _ReadPhyId();
	if (id == 0 || id == 0xffffffff) {
		for (fPhyAddress = 0; fPhyAddress < 32; fPhyAddress++) {
			id = _ReadPhyId();
			if (id != 0 && id != 0xffffffff)
				break;
		}
		if (fPhyAddress == 32) {
			ERROR("no PHY found\n");
			fPhyAddress = kDefaultPhyAddress;
			return B_DEVICE_NOT_FOUND;
		}
	}

	// The mask also covers the BCM54210PE of the Compute Module 4.
	fBroadcomPhy = (id & BCM54213PE_PHY_ID_MASK) == BCM54213PE_PHY_ID;
	TRACE_ALWAYS("PHY %#" B_PRIx32 " at address %" B_PRIu32 "\n", id,
		fPhyAddress);
	if (fBroadcomPhy)
		_ConfigureBroadcomPhy();
	else
		ERROR("unknown PHY, using it without its RGMII delay setup\n");

	// Advertise 10/100 half and full duplex and 1000 full duplex; no pause
	// frames, so that the MAC can ignore them. GENET is never paired with
	// a gigabit half-duplex link partner in practice.
	_PhyWrite(MII_ADVERTISE, ADVERTISE_CSMA | ADVERTISE_10HALF
		| ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL);
	_PhyWrite(MII_CTRL1000, (_PhyRead(MII_CTRL1000) & ~0x0300)
		| ADVERTISE_1000FULL);

	uint16 control = _PhyRead(MII_BMCR);
	control &= ~(BMCR_ISOLATE | BMCR_PDOWN);
	_PhyWrite(MII_BMCR, control | BMCR_ANENABLE | BMCR_ANRESTART);
	return B_OK;
}


/*!	Configures the BCM54213PE for the GENET:
	- AutogrEEEn off (Linux bcm54xx_config_init()), as well as Energy
	  Efficient Ethernet and auto power down. In these low-power modes the
	  PHY loses a frame that arrives right after an idle period, before it
	  reaches the MAC; the Phoenix-RTOS driver traced lost replies at gigabit
	  speed to that. Linux enables auto power down for the
	  "brcm,powerdown-enable" property of the Raspberry Pi 4 device tree;
	  this driver deliberately does not.
	- The RGMII clock delays for the phy-mode (Linux
	  bcm54xx_config_clock_delay()).
*/
void
GenetDevice::_ConfigureBroadcomPhy()
{
	_PhyWrite(BCM54XX_EXP_SEL, BCM54XX_TOP_MISC_MII_BUF_CNTL0);
	_PhyWrite(BCM54XX_EXP_DATA,
		_PhyRead(BCM54XX_EXP_DATA) & ~BCM54XX_AUTOGREEEN_EN);
	_PhyWrite(BCM54XX_EXP_SEL, 0);

	_PhyWriteMmd(MDIO_MMD_AN, BCM_CL45VEN_EEE_CONTROL,
		_PhyReadMmd(MDIO_MMD_AN, BCM_CL45VEN_EEE_CONTROL)
			& ~(LPI_FEATURE_EN | LPI_FEATURE_EN_DIG1000X));
	_PhyWriteMmd(MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0);

	_PhyWrite(BCM54XX_SHD, SHD_VAL(BCM54XX_SHD_APD));
	uint16 value = SHD_DATA(_PhyRead(BCM54XX_SHD)) & ~SHD_APD_EN;
	_PhyWrite(BCM54XX_SHD, SHD_WRITE | SHD_VAL(BCM54XX_SHD_APD)
		| SHD_DATA(value));

	/*	Linux passes the PHY driver a mode that differs from the device tree
		("we have not been correctly interpreting the phy_interface values",
		bcmgenet_mii_probe()): "rgmii" becomes "rgmii-id", "rgmii-txid"
		becomes "rgmii-rxid". Together with the MAC side (_ConfigureRgmii())
		that yields, per device tree mode:
			rgmii:		PHY delays both clocks, MAC none
			rgmii-txid,
			rgmii-rxid:	PHY delays the receive clock, MAC the transmit clock
	*/
	bool transmitDelay = fPhyMode == PHY_MODE_RGMII;

	// the receive clock: RXC-RXD skew in the miscellaneous control shadow
	_PhyWrite(BCM54XX_AUX_CTL, AUXCTL_SHDWSEL_MASK
		| (AUXCTL_SHDWSEL_MISC << AUXCTL_SHDWSEL_READ_SHIFT));
	value = _PhyRead(BCM54XX_AUX_CTL) | AUXCTL_MISC_WREN
		| AUXCTL_MISC_RGMII_SKEW_EN;
	_PhyWrite(BCM54XX_AUX_CTL, AUXCTL_SHDWSEL_MISC | value);

	// the transmit clock: GTXCLK delay in the clock alignment shadow
	_PhyWrite(BCM54XX_SHD, SHD_VAL(BCM54XX_SHD_CLK_CTL));
	value = SHD_DATA(_PhyRead(BCM54XX_SHD));
	if (transmitDelay)
		value |= SHD_CLK_CTL_GTXCLK_EN;
	else
		value &= ~SHD_CLK_CTL_GTXCLK_EN;
	_PhyWrite(BCM54XX_SHD, SHD_WRITE | SHD_VAL(BCM54XX_SHD_CLK_CTL)
		| SHD_DATA(value));
}


/*!	The MAC side of the RGMII interface (Linux bcmgenet_mii_config()):
	external gigabit PHY port, RGMII on, link state taken from the RGMII
	in-band status rather than the out-of-band pins, and the MAC's own
	transmit clock delay ("internal delay mode") unless the PHY provides it.
*/
void
GenetDevice::_ConfigureRgmii()
{
	_Write(SYS_PORT_CTRL, PORT_MODE_EXT_GPHY);

	uint32 control = _Read(EXT_RGMII_OOB_CTRL);
	control &= ~(OOB_DISABLE | ID_MODE_DIS);
	if (fPhyMode == PHY_MODE_RGMII)
		control |= ID_MODE_DIS;
	control |= RGMII_MODE_EN;
	_Write(EXT_RGMII_OOB_CTRL, control);
}


/*!	Returns whether the link is up, and its speed and duplex mode.
	Must be called with fLinkLock held.
*/
bool
GenetDevice::_ReadLink(uint32* _speed, bool* _fullDuplex)
{
	// The link status bit latches a link loss; the second read is current.
	_PhyRead(MII_BMSR);
	uint16 status = _PhyRead(MII_BMSR);
	if (status == 0xffff || (status & BMSR_LSTATUS) == 0
		|| (status & BMSR_ANEGCOMPLETE) == 0) {
		return false;
	}

	if (fBroadcomPhy) {
		// The auxiliary status summary has the negotiated mode (the
		// "highest common denominator") ready made.
		switch (AUX_STATUS_HCD(_PhyRead(BCM54XX_AUX_STATUS))) {
			case 1:
				*_speed = 10;
				*_fullDuplex = false;
				return true;
			case 2:
				*_speed = 10;
				*_fullDuplex = true;
				return true;
			case 3:
				*_speed = 100;
				*_fullDuplex = false;
				return true;
			case 5:
				*_speed = 100;
				*_fullDuplex = true;
				return true;
			case 6:
				*_speed = 1000;
				*_fullDuplex = false;
				return true;
			case 7:
				*_speed = 1000;
				*_fullDuplex = true;
				return true;
			default:
				return false;
		}
	}

	// IEEE 802.3 resolution: the best mode both sides advertised.
	uint16 local1000 = _PhyRead(MII_CTRL1000);
	uint16 partner1000 = _PhyRead(MII_STAT1000);
	if ((local1000 & ADVERTISE_1000FULL) != 0
		&& (partner1000 & LPA_1000FULL) != 0) {
		*_speed = 1000;
		*_fullDuplex = true;
		return true;
	}

	uint16 common = _PhyRead(MII_ADVERTISE) & _PhyRead(MII_LPA);
	if ((common & ADVERTISE_100FULL) != 0) {
		*_speed = 100;
		*_fullDuplex = true;
	} else if ((common & ADVERTISE_100HALF) != 0) {
		*_speed = 100;
		*_fullDuplex = false;
	} else if ((common & ADVERTISE_10FULL) != 0) {
		*_speed = 10;
		*_fullDuplex = true;
	} else {
		*_speed = 10;
		*_fullDuplex = false;
	}
	return true;
}


/*!	Programs the MAC for the current link (Linux bcmgenet_mac_config() and
	bcmgenet_mii_setup()). The first link up takes the MAC out of software
	reset and turns on its receiver and transmitter. Pause frames are always
	ignored, since none are advertised.
	Must be called with fLinkLock held.
*/
void
GenetDevice::_ConfigureMacForLink()
{
	uint32 control = _Read(EXT_RGMII_OOB_CTRL);
	if (!fLinkUp) {
		_Write(EXT_RGMII_OOB_CTRL, control & ~RGMII_LINK);
		return;
	}
	_Write(EXT_RGMII_OOB_CTRL, control | RGMII_LINK);

	uint32 command = _Read(UMAC_CMD);
	command &= ~(CMD_SPEED_MASK | CMD_HD_EN);
	switch (fSpeed) {
		case 1000:
			command |= CMD_SPEED_1000;
			break;
		case 100:
			command |= CMD_SPEED_100;
			break;
		default:
			command |= CMD_SPEED_10;
			break;
	}
	if (!fFullDuplex)
		command |= CMD_HD_EN;
	command |= CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE;

	if ((command & CMD_SW_RESET) != 0) {
		command &= ~CMD_SW_RESET;
		_Write(UMAC_CMD, command);
		spin(2);
		command |= CMD_TX_EN | CMD_RX_EN;
	}
	_Write(UMAC_CMD, command);
}


//	#pragma mark - device module API


static status_t
genet_init_device(void* driverCookie, void** _deviceCookie)
{
	GenetDevice* device = (GenetDevice*)driverCookie;

	status_t status = device->AllocateBuffers();
	if (status != B_OK)
		return status;

	*_deviceCookie = device;
	return B_OK;
}


static void
genet_uninit_device(void* deviceCookie)
{
	GenetDevice* device = (GenetDevice*)deviceCookie;
	device->FreeBuffers();
}


static status_t
genet_open(void* deviceCookie, const char* path, int openMode, void** _cookie)
{
	GenetDevice* device = (GenetDevice*)deviceCookie;

	genet_handle* handle = new(std::nothrow) genet_handle;
	if (handle == NULL)
		return B_NO_MEMORY;

	handle->device = device;
	handle->nonBlocking = (openMode & O_NONBLOCK) != 0;

	status_t status = device->Open();
	if (status != B_OK) {
		delete handle;
		return status;
	}

	*_cookie = handle;
	return B_OK;
}


static status_t
genet_close(void* cookie)
{
	genet_handle* handle = (genet_handle*)cookie;
	handle->device->Close();
	return B_OK;
}


static status_t
genet_free(void* cookie)
{
	delete (genet_handle*)cookie;
	return B_OK;
}


static status_t
genet_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	genet_handle* handle = (genet_handle*)cookie;
	return handle->device->Receive(buffer, _length, handle->nonBlocking);
}


static status_t
genet_write(void* cookie, off_t position, const void* buffer, size_t* _length)
{
	genet_handle* handle = (genet_handle*)cookie;
	status_t status = handle->device->Send(buffer, *_length,
		handle->nonBlocking);
	if (status != B_OK)
		*_length = 0;
	return status;
}


static status_t
genet_control(void* cookie, uint32 op, void* buffer, size_t length)
{
	genet_handle* handle = (genet_handle*)cookie;
	GenetDevice* device = handle->device;

	switch (op) {
		case ETHER_INIT:
			return B_OK;

		case ETHER_GETADDR:
			return user_memcpy(buffer, device->MacAddress(), 6);

		case ETHER_GETFRAMESIZE:
		{
			const uint32 frameSize = kMaxFrameSize;
			if (length < sizeof(frameSize))
				return B_BAD_VALUE;
			return user_memcpy(buffer, &frameSize, sizeof(frameSize));
		}

		case ETHER_NONBLOCK:
		{
			int32 value;
			if (length < sizeof(value))
				return B_BAD_VALUE;
			if (user_memcpy(&value, buffer, sizeof(value)) != B_OK)
				return B_BAD_ADDRESS;
			handle->nonBlocking = value != 0;
			return B_OK;
		}

		case ETHER_SETPROMISC:
		{
			int32 value;
			if (length < sizeof(value))
				return B_BAD_VALUE;
			if (user_memcpy(&value, buffer, sizeof(value)) != B_OK)
				return B_BAD_ADDRESS;
			return device->SetPromiscuous(value != 0);
		}

		case ETHER_ADDMULTI:
		case ETHER_REMMULTI:
		{
			uint8 address[6];
			if (length < sizeof(address))
				return B_BAD_VALUE;
			if (user_memcpy(address, buffer, sizeof(address)) != B_OK)
				return B_BAD_ADDRESS;
			if (op == ETHER_ADDMULTI)
				return device->AddMulticast(address);
			return device->RemoveMulticast(address);
		}

		case ETHER_SET_LINK_STATE_SEM:
		{
			sem_id semaphore;
			if (length < sizeof(semaphore))
				return B_BAD_VALUE;
			if (user_memcpy(&semaphore, buffer, sizeof(semaphore)) != B_OK)
				return B_BAD_ADDRESS;
			device->SetLinkStateSemaphore(semaphore);
			return B_OK;
		}

		case ETHER_GET_LINK_STATE:
		{
			ether_link_state state;
			if (length < sizeof(state))
				return B_BAD_VALUE;
			status_t status = device->GetLinkState(&state);
			if (status != B_OK)
				return status;
			return user_memcpy(buffer, &state, sizeof(state));
		}
	}

	return B_DEV_INVALID_IOCTL;
}


//	#pragma mark - driver module API


static float
genet_supports_device(device_node* parent)
{
	const char* bus;
	if (sDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
			!= B_OK || strcmp(bus, "fdt") != 0) {
		return 0.0f;
	}

	const char* compatible;
	if (sDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible,
			false) != B_OK || strcmp(compatible, kCompatible) != 0) {
		return 0.0f;
	}

	return 0.8f;
}


static status_t
genet_register_device(device_node* parent)
{
	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Broadcom GENET Ethernet" } },
		{ NULL }
	};

	return sDeviceManager->register_node(parent, GENET_DRIVER_MODULE_NAME,
		attrs, NULL, NULL);
}


static status_t
genet_init_driver(device_node* node, void** _driverCookie)
{
	GenetDevice* device = new(std::nothrow) GenetDevice(node);
	if (device == NULL)
		return B_NO_MEMORY;

	status_t status = device->InitDriver();
	if (status != B_OK) {
		delete device;
		return status;
	}

	*_driverCookie = device;
	return B_OK;
}


static void
genet_uninit_driver(void* driverCookie)
{
	delete (GenetDevice*)driverCookie;
}


static status_t
genet_register_child_devices(void* driverCookie)
{
	GenetDevice* device = (GenetDevice*)driverCookie;

	int32 id = sDeviceManager->create_id(GENET_DEVICE_ID_GENERATOR);
	if (id < 0)
		return id;

	char name[64];
	snprintf(name, sizeof(name), "net/genet/%" B_PRId32, id);

	status_t status = sDeviceManager->publish_device(device->Node(), name,
		GENET_DEVICE_MODULE_NAME);
	if (status != B_OK)
		sDeviceManager->free_id(GENET_DEVICE_ID_GENERATOR, id);
	return status;
}


//	#pragma mark -


module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&sDeviceManager },
	{}
};


static struct device_module_info sGenetDevice = {
	{
		GENET_DEVICE_MODULE_NAME,
		0,
		NULL
	},

	genet_init_device,
	genet_uninit_device,
	NULL,	// remove

	genet_open,
	genet_close,
	genet_free,
	genet_read,
	genet_write,
	NULL,	// io
	genet_control,

	NULL,	// select
	NULL,	// deselect
};


static struct driver_module_info sGenetDriver = {
	{
		GENET_DRIVER_MODULE_NAME,
		0,
		NULL
	},

	genet_supports_device,
	genet_register_device,
	genet_init_driver,
	genet_uninit_driver,
	genet_register_child_devices,
	NULL,	// rescan
	NULL,	// removed
};


module_info* modules[] = {
	(module_info*)&sGenetDriver,
	(module_info*)&sGenetDevice,
	NULL
};
