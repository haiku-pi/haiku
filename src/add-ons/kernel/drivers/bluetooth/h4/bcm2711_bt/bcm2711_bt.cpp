/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The Raspberry Pi 4's on-board Bluetooth controller, the Bluetooth half of
	its BCM43455 combo chip, as an H4 (UART) transport of the Bluetooth stack.

	The controller's UART is wired to GPIO 30 to 33. The firmware gives the
	PL011 to the serial console ("disable-bt" overlay), so this driver routes
	the AUX mini-UART (uart1) to those pins, powers the controller through the
	firmware's GPIO expander line BT_ON and talks HCI to it in H4 framing at
	115200 baud: a packet indicator byte (command, ACL data, SCO data, event)
	followed by the packet.

	Each open brings the controller up from power-on: HCI_Reset, then the patch
	RAM file from the boot volume (firmware/brcm/BCM4345C0.hcd in the data
	directories, a list of vendor commands replayed one by one after
	Download_Minidriver; its last one, Launch_RAM, restarts the controller on
	the patch), then HCI_Reset again. Only then is the controller registered
	with the HCI module; events and ACL data go up once the bluetooth_server has
	sent BT_UP, as with the USB transport (h2generic).

	Reception is interrupt driven: the mini-UART has an 8 byte receive FIFO,
	which the handler of the (shared) AUX interrupt drains into a ring buffer.
	A thread splits the ring into H4 packets and posts them to the HCI module,
	which takes whole packets only, and may block. Should the interrupt not
	arrive, the thread polls the FIFO instead.

	The mini-UART's baud rate derives from the VPU core clock, which the
	firmware lowers (500 -> 200 MHz) when the ARM leaves turbo: every open asks
	the firmware for the core clock's maximum, as the V3D driver does, and
	derives the divisor from the rate it then measures.
*/


#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <Drivers.h>
#include <FindDirectory.h>
#include <KernelExport.h>
#include <kernel.h>
#include <lock.h>
#include <util/AutoLock.h>

#include <bcm2835_mailbox.h>
#include <net_buffer.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/HCI/btHCI.h>
#include <bluetooth/HCI/btHCI_acl.h>
#include <bluetooth/HCI/btHCI_command.h>
#include <bluetooth/HCI/btHCI_event.h>
#include <bluetooth/HCI/btHCI_transport.h>

extern "C" {
#include <libfdt.h>
}


//#define TRACE_BCM2711_BT
#ifdef TRACE_BCM2711_BT
#	define TRACE(x...)		dprintf("bcm2711_bt: " x)
#else
#	define TRACE(x...)
#endif
#define TRACE_ALWAYS(x...)	dprintf("bcm2711_bt: " x)
#define ERROR(x...)			dprintf("bcm2711_bt: " x)


// the bluetooth_server watches bluetooth/h4/ for driver directories
#define DEVICE_NAME "bluetooth/h4/bcm2711_bt/0"


extern void* gFDT;


int32 api_version = B_CUR_DRIVER_API_VERSION;


// AUX block registers (BCM2711 ARM Peripherals, chapter 2), as word indices
enum {
	AUX_IRQ			= 0x00 / 4,
	AUX_ENABLES		= 0x04 / 4,
	AUX_MU_IO		= 0x40 / 4,
	AUX_MU_IER		= 0x44 / 4,
	AUX_MU_IIR		= 0x48 / 4,
	AUX_MU_LCR		= 0x4c / 4,
	AUX_MU_MCR		= 0x50 / 4,
	AUX_MU_LSR		= 0x54 / 4,
	AUX_MU_CNTL		= 0x60 / 4,
	AUX_MU_BAUD		= 0x68 / 4,
};
static const uint32 kAuxMiniUart = 1 << 0;
	// AUX_IRQ (interrupt pending) and AUX_ENABLES
// AUX_MU_IER: bit 0 enables the receive interrupt, as in a 16550; bits 3:2
// are documented as unused, but have to be set for interrupts to arrive
static const uint32 kIerReceive = 0x0d;
static const uint32 kIirClearFifos = 0xc6;
static const uint32 kLcr8Bits = 3;
	// 8 data bits; bit 1 is needed too, despite the documentation
static const uint32 kMcrRts = 1 << 1;
	// drives RTS low, which tells the controller to send
static const uint32 kLsrDataReady = 1 << 0;
static const uint32 kLsrOverrun = 1 << 1;
static const uint32 kLsrTransmitterEmpty = 1 << 5;
	// the transmit FIFO can take at least one byte
static const uint32 kCntlReceive = 1 << 0;
static const uint32 kCntlTransmit = 1 << 1;
static const uint32 kFifoSize = 8;

// GPIO (BCM2711): function select, 10 pins of 3 bits per register, and the
// pull-up/down control, 16 pins of 2 bits per register; as word indices
static const uint32 kGpioFunctionSelect0 = 0x00 / 4;
static const uint32 kGpioPullControl0 = 0xe4 / 4;
static const uint32 kGpioAlt5 = 2;
static const uint32 kPullNone = 0;
static const uint32 kPullUp = 1;

// The controller's UART on the mini-UART's ALT5 pins, pulled as the
// device tree's "uart1_bt_pins" group has them
static const struct {
	uint32	pin;
	uint32	pull;
} kPins[] = {
	{ 30, kPullUp },	// CTS1
	{ 31, kPullNone },	// RTS1
	{ 32, kPullNone },	// TXD1
	{ 33, kPullUp },	// RXD1
};

// BT_ON, the controller's power, is line 0 of the firmware's GPIO expander
static const uint32 kPowerLine = BCM2835_FIRMWARE_GPIO_BASE + 0;
static const bigtime_t kPowerOffTime = 50000;
static const bigtime_t kPowerOnTime = 200000;
	// the controller's start, and the UART's settling before the first command

static const uint32 kBaudRate = 115200;
static const bigtime_t kByteTime = 1000000 * 10 / kBaudRate;
static const bigtime_t kTransmitTimeout = 100000;
	// per byte; there is no flow control on transmit, so only a stuck UART
static const bigtime_t kCommandTimeout = 2000000;
static const uint32 kResetAttempts = 3;
static const bigtime_t kResetSettleTime = 100000;
static const bigtime_t kMinidriverSettleTime = 50000;
static const bigtime_t kLaunchSettleTime = 250000;
static const uint32 kPatchProgressRecords = 100;

static const char* kPatchFile = "firmware/brcm/BCM4345C0.hcd";
static const off_t kMaxPatchSize = 512 * 1024;

// H4 packet indicators
static const uint8 kH4Command = 0x01;
static const uint8 kH4Acl = 0x02;
static const uint8 kH4Sco = 0x03;
static const uint8 kH4Event = 0x04;

// Commands of the bring-up
static const uint16 kOpReset
	= PACK_OPCODE(OGF_CONTROL_BASEBAND, OCF_RESET);
static const uint16 kOpReadLocalVersion
	= PACK_OPCODE(OGF_INFORMATIONAL_PARAM, OCF_READ_LOCAL_VERSION);
static const uint16 kOpReadBdAddr
	= PACK_OPCODE(OGF_INFORMATIONAL_PARAM, OCF_READ_BD_ADDR);
static const uint16 kOpDownloadMinidriver = 0xfc2e;
	// Broadcom vendor command

static const uint32 kRingSize = 4096;
static_assert((kRingSize & (kRingSize - 1)) == 0, "the ring is a power of 2");
static const uint32 kMaxPacketSize = HCI_ACL_HDR_SIZE + HCI_MAX_ACL_SIZE;
static const uint32 kMaxCommandSize = sizeof(hci_command_header) + 255;
static const bigtime_t kIdleInterval = 20000;
static const bigtime_t kPollInterval = 500;
	// without an interrupt; 8 bytes at 115200 baud are 694 us
static const uint32 kMaxEmptyInterrupts = 1000;
static const uint32 kLoggedCommands = 8;
	// the first commands of the server are logged


struct bt_uart {
	mutex				lock;
		// open and free
	bool				open;

	area_id				aux_area;
	area_id				gpio_area;
	volatile uint32*	aux;
	volatile uint32*	gpio;
	int32				interrupt;
	bool				interrupt_installed;
	uint32				core_clock;

	// the HCI module's device, and whether packets go up to it
	bluetooth_device*	ndev;
	hci_id				hci;
	int32				forwarding;
	bool				bringing_up;

	// receive ring, filled by the interrupt handler (or the thread)
	spinlock			ring_lock;
	uint8				ring[kRingSize];
	uint32				ring_head;
	uint32				ring_tail;
	uint32				ring_overflows;
	uint32				uart_overruns;
	uint32				interrupts;
	uint32				empty_interrupts;
		// in a row, without a received byte
	uint64				bytes_received;
	bool				interrupt_storm;
		// the handler disabled the interrupt, which fired without data

	sem_id				rx_sem;
	thread_id			rx_thread;
	volatile bool		rx_quit;
	bool				polling;
	int32				parser_reset;

	// H4 parser, of the receive thread
	uint8				packet_type;
	uint32				packet_length;
	uint32				packet_expected;
	uint32				packet_header;
	uint8				packet[kMaxPacketSize];
	uint32				sync_errors;

	// the bring-up's commands: the reply to the pending opcode
	mutex				reply_lock;
	sem_id				reply_sem;
	bool				waiting;
	uint16				pending_opcode;
	uint8				reply[HCI_MAX_EVENT_SIZE];
	size_t				reply_length;
	uint8				hci_status;

	mutex				tx_lock;
	uint8				tx_buffer[1 + kMaxPacketSize];
	uint32				commands_logged;

	bt_hci_statistics	stats;
};

static bt_uart sDevice;

static bcm2835_mailbox_module_info* sMailbox;
static bt_hci_module_info* sHci;
static net_buffer_module_info* sBufferModule;


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


/*!	Translates \a address from the child to the parent side of the "ranges"
	of \a bus.
*/
static bool
apply_ranges(int bus, uint64& address)
{
	int length;
	const uint32* ranges = (const uint32*)fdt_getprop(gFDT, bus, "ranges",
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
		if (address >= child && address - child < size) {
			address = address - child + parent;
			return true;
		}
	}
	return false;
}


/*!	The CPU's physical address of the first register block of \a node. */
static status_t
node_registers(int node, phys_addr_t& physical)
{
	int bus = fdt_parent_offset(gFDT, node);
	const uint32* reg = (const uint32*)fdt_getprop(gFDT, node, "reg", NULL);
	if (bus < 0 || reg == NULL)
		return B_BAD_DATA;

	uint64 address = read_cells(reg, address_cells(bus));
	for (int parent = bus; fdt_parent_offset(gFDT, parent) >= 0;
			parent = fdt_parent_offset(gFDT, parent)) {
		if (!apply_ranges(parent, address))
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


/*!	Maps the page holding the registers of \a node. */
static status_t
map_registers(int node, const char* name, area_id& area,
	volatile uint32*& registers)
{
	phys_addr_t physical;
	if (node < 0 || node_registers(node, physical) != B_OK)
		return B_DEVICE_NOT_FOUND;

	void* address;
	area = map_physical_memory(name, physical & ~(phys_addr_t)(B_PAGE_SIZE - 1),
		B_PAGE_SIZE, B_ANY_KERNEL_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &address);
	if (area < 0)
		return area;

	registers = (volatile uint32*)((addr_t)address
		+ (physical & (B_PAGE_SIZE - 1)));
	return B_OK;
}


/*!	The interrupt vector of \a node's first GIC interrupt. */
static status_t
find_interrupt(int node, int32& interrupt)
{
	// GIC: <type number flags>, type 0 = SPI (from 32), 1 = PPI (from 16)
	int length;
	const uint32* interrupts = (const uint32*)fdt_getprop(gFDT, node,
		"interrupts", &length);
	if (interrupts == NULL || length < (int)(3 * sizeof(uint32)))
		return B_BAD_DATA;

	interrupt = fdt32_to_cpu(interrupts[1])
		+ (fdt32_to_cpu(interrupts[0]) == 0 ? 32 : 16);
	return B_OK;
}


//	#pragma mark - firmware


static status_t
set_power(bool on)
{
	uint32 values[2] = { kPowerLine, on ? 1u : 0u };
	return sMailbox->property_tag(BCM2835_TAG_SET_GPIO_STATE, values,
		sizeof(values), sizeof(values), NULL);
}


static int32
power_state()
{
	uint32 values[2] = { kPowerLine, 0 };
	if (sMailbox->property_tag(BCM2835_TAG_GET_GPIO_STATE, values,
			sizeof(values), sizeof(uint32), NULL) != B_OK) {
		return -1;
	}
	return values[1];
}


static uint32
clock_rate(uint32 tag, uint32 clock)
{
	uint32 values[2] = { clock, 0 };
	if (sMailbox->property_tag(tag, values, sizeof(values), sizeof(uint32),
			NULL) != B_OK || values[0] != clock) {
		return 0;
	}
	return values[1];
}


/*!	Asks the firmware to run the core clock at its maximum; returns that
	maximum, or 0.
*/
static uint32
set_core_clock_max()
{
	uint32 maxRate = clock_rate(BCM2835_TAG_GET_MAX_CLOCK_RATE,
		BCM2835_CLOCK_CORE);
	if (maxRate == 0)
		return 0;

	// clock ID, rate, skip setting turbo
	uint32 values[3] = { BCM2835_CLOCK_CORE, maxRate, 0 };
	if (sMailbox->property_tag(BCM2835_TAG_SET_CLOCK_RATE, values,
			sizeof(values), sizeof(values), NULL) != B_OK) {
		return 0;
	}
	return maxRate;
}


//	#pragma mark - UART


/*!	Moves what the receive FIFO holds into the ring. The ring lock must be
	held, with interrupts disabled.
*/
static uint32
drain_fifo_locked()
{
	volatile uint32* aux = sDevice.aux;
	uint32 count = 0;

	// bounded, in case the block reads back nonsense
	while (count < 4 * kFifoSize) {
		uint32 status = aux[AUX_MU_LSR];
		if ((status & kLsrOverrun) != 0)
			sDevice.uart_overruns++;
		if ((status & kLsrDataReady) == 0)
			break;

		uint8 byte = aux[AUX_MU_IO];
		uint32 next = (sDevice.ring_head + 1) & (kRingSize - 1);
		if (next == sDevice.ring_tail)
			sDevice.ring_overflows++;
		else {
			sDevice.ring[sDevice.ring_head] = byte;
			sDevice.ring_head = next;
		}
		count++;
	}
	sDevice.bytes_received += count;
	return count;
}


static int32
aux_interrupt(void* data)
{
	volatile uint32* aux = sDevice.aux;
	if ((aux[AUX_IRQ] & kAuxMiniUart) == 0)
		return B_UNHANDLED_INTERRUPT;
			// the AUX SPI controllers share the line

	SpinLocker locker(sDevice.ring_lock);
	sDevice.interrupts++;
	uint32 received = drain_fifo_locked();

	if (received > 0)
		sDevice.empty_interrupts = 0;
	else if (++sDevice.empty_interrupts > kMaxEmptyInterrupts) {
		// Only the receive interrupt is meant to be on: should the bits of
		// AUX_MU_IER mean something else (the transmit interrupt), this would
		// fire for good. The thread polls from now on.
		aux[AUX_MU_IER] = 0;
		sDevice.interrupt_storm = true;
	}
	locker.Unlock();

	if (received == 0)
		return B_HANDLED_INTERRUPT;

	release_sem_etc(sDevice.rx_sem, 1, B_DO_NOT_RESCHEDULE);
	return B_INVOKE_SCHEDULER;
}


/*!	Routes the mini-UART to the controller's pins and starts it, 8N1 at
	kBaudRate, with RTS asserted and no automatic flow control.
*/
static status_t
start_uart()
{
	const uint32 maxRate = set_core_clock_max();
	sDevice.core_clock = clock_rate(BCM2835_TAG_GET_CLOCK_MEASURED,
		BCM2835_CLOCK_CORE);
	if (sDevice.core_clock == 0)
		sDevice.core_clock = maxRate;
	if (sDevice.core_clock < 8 * kBaudRate) {
		ERROR("no core clock rate from the firmware\n");
		return B_ERROR;
	}

	// baud rate = core clock / (8 * (divisor + 1)), rounded
	uint32 divisor = (sDevice.core_clock + 4 * kBaudRate) / (8 * kBaudRate)
		- 1;
	if (divisor > 0xffff) {
		ERROR("core clock %" B_PRIu32 " Hz is too fast for %" B_PRIu32
			" baud\n", sDevice.core_clock, kBaudRate);
		return B_ERROR;
	}

	volatile uint32* gpio = sDevice.gpio;
	for (size_t i = 0; i < B_COUNT_OF(kPins); i++) {
		const uint32 pin = kPins[i].pin;
		uint32 index = kGpioFunctionSelect0 + pin / 10;
		uint32 shift = (pin % 10) * 3;
		gpio[index] = (gpio[index] & ~(7u << shift)) | (kGpioAlt5 << shift);

		index = kGpioPullControl0 + pin / 16;
		shift = (pin % 16) * 2;
		gpio[index] = (gpio[index] & ~(3u << shift))
			| (kPins[i].pull << shift);
	}

	volatile uint32* aux = sDevice.aux;
	aux[AUX_ENABLES] |= kAuxMiniUart;
	aux[AUX_MU_CNTL] = 0;
	aux[AUX_MU_LCR] = kLcr8Bits;
	aux[AUX_MU_IER] = 0;
	aux[AUX_MU_IIR] = kIirClearFifos;
	aux[AUX_MU_BAUD] = divisor;
	aux[AUX_MU_MCR] = kMcrRts;
	aux[AUX_MU_CNTL] = kCntlReceive | kCntlTransmit;
	if (sDevice.interrupt_installed)
		aux[AUX_MU_IER] = kIerReceive;
	else
		sDevice.polling = true;

	TRACE_ALWAYS("mini-UART on GPIO 30-33 (ALT5), core clock %" B_PRIu32
		" Hz (asked for %" B_PRIu32 " Hz), divisor %" B_PRIu32 ": %" B_PRIu32
		" baud\n", sDevice.core_clock, maxRate, divisor,
		sDevice.core_clock / (8 * (divisor + 1)));
	return B_OK;
}


static void
stop_uart()
{
	volatile uint32* aux = sDevice.aux;
	aux[AUX_MU_IER] = 0;
	aux[AUX_MU_CNTL] = 0;
	aux[AUX_MU_MCR] = 0;
	aux[AUX_ENABLES] &= ~kAuxMiniUart;
}


static status_t
uart_write(const uint8* data, size_t length)
{
	volatile uint32* aux = sDevice.aux;
	for (size_t i = 0; i < length; i++) {
		bigtime_t deadline = system_time() + kTransmitTimeout;
		while ((aux[AUX_MU_LSR] & kLsrTransmitterEmpty) == 0) {
			if (system_time() > deadline)
				return B_TIMED_OUT;
			snooze(kByteTime);
		}
		aux[AUX_MU_IO] = data[i];
	}
	sDevice.stats.bytesTX += length;
	return B_OK;
}


/*!	Sends one H4 packet: its indicator, then \a length bytes of \a data. The
	transmit lock must be held.
*/
static status_t
send_packet_locked(uint8 indicator, const void* data, size_t length)
{
	status_t status = uart_write(&indicator, 1);
	if (status == B_OK)
		status = uart_write((const uint8*)data, length);

	if (status == B_OK) {
		sDevice.stats.acceptedTX++;
		sDevice.stats.successfulTX++;
		if (indicator == kH4Command)
			sDevice.stats.commandTX++;
		else if (indicator == kH4Acl)
			sDevice.stats.aclTX++;
	} else {
		sDevice.stats.errorTX++;
		ERROR("transmit failed: %s\n", strerror(status));
	}
	return status;
}


static status_t
send_packet(uint8 indicator, const void* data, size_t length)
{
	MutexLocker locker(sDevice.tx_lock);
	return send_packet_locked(indicator, data, length);
}


//	#pragma mark - receive


/*!	Hands an event to the bring-up, when it is the reply to the command it
	waits for.
*/
static bool
take_reply(const uint8* event, size_t length)
{
	// Command Complete: code, length, Num_HCI_Command_Packets, opcode,
	// return parameters (status first); Command Status: code, length,
	// status, Num_HCI_Command_Packets, opcode
	uint16 opcode;
	const uint8* parameters;
	size_t parametersLength;
	if (event[0] == HCI_EVENT_CMD_COMPLETE && length >= 5) {
		opcode = event[3] | (event[4] << 8);
		parameters = event + 5;
		parametersLength = length - 5;
	} else if (event[0] == HCI_EVENT_CMD_STATUS && length >= 6) {
		opcode = event[4] | (event[5] << 8);
		parameters = event + 2;
		parametersLength = 1;
	} else
		return false;

	MutexLocker locker(sDevice.reply_lock);
	if (!sDevice.waiting || opcode != sDevice.pending_opcode)
		return false;

	sDevice.reply_length = min_c(parametersLength, sizeof(sDevice.reply));
	memcpy(sDevice.reply, parameters, sDevice.reply_length);
	sDevice.waiting = false;
	release_sem(sDevice.reply_sem);
	return true;
}


static void
deliver_packet(uint8 indicator, const uint8* data, size_t length)
{
	sDevice.stats.acceptedRX++;
	sDevice.stats.bytesRX += length;

	if (indicator == kH4Event) {
		sDevice.stats.eventRX++;
		if (take_reply(data, length)) {
			sDevice.stats.successfulRX++;
			return;
		}
	} else if (indicator == kH4Acl)
		sDevice.stats.aclRX++;
	else
		sDevice.stats.scoRX++;

	if (atomic_get(&sDevice.forwarding) == 0) {
		if (sDevice.bringing_up) {
			TRACE_ALWAYS("ignored packet %#x (%" B_PRIuSIZE " bytes, first %#x)"
				"\n", indicator, length, data[0]);
		}
		sDevice.stats.rejectedRX++;
		return;
	}

	// SCO goes over the PCM pins; the HCI module does not assemble it either
	if (indicator == kH4Sco) {
		sDevice.stats.rejectedRX++;
		return;
	}

	status_t status = sHci->PostTransportPacket(sDevice.hci,
		indicator == kH4Event ? BT_EVENT : BT_ACL, (void*)data, length);
	if (status == B_OK)
		sDevice.stats.successfulRX++;
	else
		sDevice.stats.errorRX++;
}


/*!	The H4 parser: one received byte. */
static void
parse_byte(uint8 byte)
{
	if (sDevice.packet_type == 0) {
		switch (byte) {
			case kH4Event:
				sDevice.packet_header = sizeof(hci_event_header);
				break;
			case kH4Acl:
				sDevice.packet_header = sizeof(hci_acl_header);
				break;
			case kH4Sco:
				sDevice.packet_header = 3;
				break;
			default:
				// not at a packet's start: skip until one
				sDevice.sync_errors++;
				return;
		}
		sDevice.packet_type = byte;
		sDevice.packet_length = 0;
		sDevice.packet_expected = 0;
		return;
	}

	sDevice.packet[sDevice.packet_length++] = byte;

	if (sDevice.packet_expected == 0
		&& sDevice.packet_length == sDevice.packet_header) {
		const uint8* header = sDevice.packet;
		uint32 payload;
		switch (sDevice.packet_type) {
			case kH4Event:
				payload = header[1];
				break;
			case kH4Acl:
				payload = header[2] | (header[3] << 8);
				break;
			default:
				payload = header[2];
				break;
		}
		if (payload > kMaxPacketSize - sDevice.packet_header) {
			sDevice.sync_errors++;
			sDevice.packet_type = 0;
			return;
		}
		sDevice.packet_expected = sDevice.packet_header + payload;
	}

	if (sDevice.packet_expected != 0
		&& sDevice.packet_length == sDevice.packet_expected) {
		deliver_packet(sDevice.packet_type, sDevice.packet,
			sDevice.packet_length);
		sDevice.packet_type = 0;
	}
}


static status_t
receive_thread(void*)
{
	while (!sDevice.rx_quit) {
		acquire_sem_etc(sDevice.rx_sem, 1, B_RELATIVE_TIMEOUT,
			sDevice.polling ? kPollInterval : kIdleInterval);
		if (sDevice.rx_quit)
			break;

		bool startPolling = false;
		{
			// whatever the interrupt left (or did not take) in the FIFO
			InterruptsSpinLocker locker(sDevice.ring_lock);
			uint32 found = drain_fifo_locked();
			if (!sDevice.polling && (sDevice.interrupt_storm
					|| (found > 0 && sDevice.interrupts == 0))) {
				sDevice.polling = startPolling = true;
			}
		}
		if (startPolling) {
			ERROR("no receive interrupt (%s), polling the FIFO\n",
				sDevice.interrupt_storm
					? "it fired without data" : "none came");
		}

		if (atomic_get_and_set(&sDevice.parser_reset, 0) != 0) {
			sDevice.packet_type = 0;
			InterruptsSpinLocker locker(sDevice.ring_lock);
			sDevice.ring_tail = sDevice.ring_head;
		}

		while (true) {
			uint8 chunk[256];
			uint32 count = 0;
			{
				InterruptsSpinLocker locker(sDevice.ring_lock);
				while (count < sizeof(chunk)
					&& sDevice.ring_tail != sDevice.ring_head) {
					chunk[count++] = sDevice.ring[sDevice.ring_tail];
					sDevice.ring_tail = (sDevice.ring_tail + 1)
						& (kRingSize - 1);
				}
			}
			if (count == 0)
				break;

			for (uint32 i = 0; i < count; i++)
				parse_byte(chunk[i]);
		}
	}
	return B_OK;
}


/*!	Forgets what was received so far, and a packet the parser has begun. */
static void
flush_receive()
{
	atomic_set(&sDevice.parser_reset, 1);
	release_sem(sDevice.rx_sem);
	snooze(10000);
}


//	#pragma mark - bring-up


/*!	Sends a command and waits for its Command Complete (or Command Status).
	Its return parameters, from the status on, go to \a reply. Fails with
	B_DEV_INVALID_IOCTL when the controller returned an error status, which is
	in sDevice.hci_status then.
*/
static status_t
hci_command(uint16 opcode, const void* parameters, uint8 length,
	void* reply = NULL, size_t replySize = 0)
{
	uint8 command[kMaxCommandSize];
	command[0] = opcode & 0xff;
	command[1] = opcode >> 8;
	command[2] = length;
	if (length > 0)
		memcpy(command + 3, parameters, length);

	// a late reply to an earlier command must not count for this one
	while (acquire_sem_etc(sDevice.reply_sem, 1, B_RELATIVE_TIMEOUT, 0)
			== B_OK) {
	}
	{
		MutexLocker locker(sDevice.reply_lock);
		sDevice.pending_opcode = opcode;
		sDevice.waiting = true;
	}

	status_t status = send_packet(kH4Command, command, 3 + length);
	if (status == B_OK) {
		status = acquire_sem_etc(sDevice.reply_sem, 1, B_RELATIVE_TIMEOUT,
			kCommandTimeout);
	}

	MutexLocker locker(sDevice.reply_lock);
	sDevice.waiting = false;
	if (status != B_OK)
		return status;
	if (sDevice.reply_length == 0)
		return B_BAD_DATA;

	sDevice.hci_status = sDevice.reply[0];
	if (sDevice.hci_status != 0)
		return B_DEV_INVALID_IOCTL;
	if (reply != NULL) {
		memset(reply, 0, replySize);
		memcpy(reply, sDevice.reply, min_c(replySize, sDevice.reply_length));
	}
	return B_OK;
}


static const char*
command_error(status_t status)
{
	static char buffer[32];
	if (status != B_DEV_INVALID_IOCTL)
		return strerror(status);
	snprintf(buffer, sizeof(buffer), "HCI status %#x", sDevice.hci_status);
	return buffer;
}


static void
power_cycle()
{
	status_t offStatus = set_power(false);
	snooze(kPowerOffTime);
	status_t onStatus = set_power(true);
	snooze(kPowerOnTime);
	TRACE_ALWAYS("BT_ON power cycled (%s, %s), line now %" B_PRId32 "\n",
		strerror(offStatus), strerror(onStatus), power_state());
	flush_receive();
}


static status_t
reset_controller()
{
	status_t status = B_ERROR;
	for (uint32 attempt = 1; attempt <= kResetAttempts; attempt++) {
		status = hci_command(kOpReset, NULL, 0);
		TRACE_ALWAYS("HCI_Reset: %s (attempt %" B_PRIu32 ")\n",
			command_error(status), attempt);
		if (status == B_OK)
			break;
		flush_receive();
	}
	snooze(kResetSettleTime);
	return status;
}


static void
log_local_version(const char* when)
{
	hci_rp_read_loc_version version;
	status_t status = hci_command(kOpReadLocalVersion, NULL, 0, &version,
		sizeof(version));
	if (status != B_OK) {
		ERROR("READ_LOCAL_VERSION (%s): %s\n", when, command_error(status));
		return;
	}
	TRACE_ALWAYS("local version (%s): HCI %#x revision %#x, LMP %#x "
		"subversion %#x, manufacturer %#x\n", when, version.hci_version,
		B_LENDIAN_TO_HOST_INT16(version.hci_revision), version.lmp_version,
		B_LENDIAN_TO_HOST_INT16(version.lmp_subversion),
		B_LENDIAN_TO_HOST_INT16(version.manufacturer));
}


static status_t
load_patch(uint8*& _data, size_t& _size)
{
	const directory_which directories[] = {
		B_SYSTEM_NONPACKAGED_DATA_DIRECTORY,
		B_SYSTEM_DATA_DIRECTORY,
	};

	char path[B_PATH_NAME_LENGTH];
	int fd = -1;
	for (size_t i = 0; i < B_COUNT_OF(directories) && fd < 0; i++) {
		if (find_directory(directories[i], -1, false, path, sizeof(path))
				!= B_OK) {
			continue;
		}
		strlcat(path, "/", sizeof(path));
		strlcat(path, kPatchFile, sizeof(path));
		fd = open(path, O_RDONLY);
	}
	if (fd < 0) {
		ERROR("no patch file %s in the data directories\n", kPatchFile);
		return B_ENTRY_NOT_FOUND;
	}

	status_t status = B_OK;
	uint8* data = NULL;
	off_t size = lseek(fd, 0, SEEK_END);
	if (size <= 0 || size > kMaxPatchSize)
		status = B_BAD_DATA;
	else if (lseek(fd, 0, SEEK_SET) != 0)
		status = B_IO_ERROR;
	else {
		data = (uint8*)malloc(size);
		if (data == NULL)
			status = B_NO_MEMORY;
		else if (read(fd, data, size) != size)
			status = B_IO_ERROR;
	}
	close(fd);

	if (status != B_OK) {
		ERROR("cannot read %s: %s\n", path, strerror(status));
		free(data);
		return status;
	}

	TRACE_ALWAYS("patch file %s, %" B_PRIdOFF " bytes\n", path, size);
	_data = data;
	_size = size;
	return B_OK;
}


/*!	Replays the patch file's records ([opcode le16] [length] [parameters],
	the last one Launch_RAM) after Download_Minidriver.
*/
static status_t
download_patch(const uint8* data, size_t size)
{
	bigtime_t start = system_time();
	status_t status = hci_command(kOpDownloadMinidriver, NULL, 0);
	if (status != B_OK) {
		ERROR("Download_Minidriver: %s\n", command_error(status));
		return status;
	}
	snooze(kMinidriverSettleTime);

	uint32 sent = 0;
	uint32 acknowledged = 0;
	uint16 opcode = 0;
	size_t offset = 0;
	while (offset < size) {
		if (size - offset < 3 || size - offset - 3 < data[offset + 2]) {
			status = B_BAD_DATA;
			ERROR("patch record %" B_PRIu32 " at %" B_PRIuSIZE
				" is truncated\n", sent, offset);
			break;
		}
		opcode = data[offset] | (data[offset + 1] << 8);
		const uint8 length = data[offset + 2];

		sent++;
		status = hci_command(opcode, data + offset + 3, length);
		if (status != B_OK) {
			ERROR("patch record %" B_PRIu32 " (opcode %#x, %u bytes): %s\n",
				sent, opcode, length, command_error(status));
			break;
		}
		acknowledged++;
		offset += 3 + length;

		if (acknowledged % kPatchProgressRecords == 0) {
			TRACE_ALWAYS("patch: %" B_PRIu32 " records acknowledged\n",
				acknowledged);
		}
	}

	TRACE_ALWAYS("patch: %" B_PRIu32 " records sent, %" B_PRIu32
		" acknowledged, last opcode %#x, %" B_PRIdBIGTIME " ms\n", sent,
		acknowledged, opcode, (system_time() - start) / 1000);
	return status;
}


static void
log_address()
{
	hci_rp_read_bd_addr reply;
	status_t status = hci_command(kOpReadBdAddr, NULL, 0, &reply,
		sizeof(reply));
	if (status != B_OK) {
		ERROR("READ_BD_ADDR: %s\n", command_error(status));
		return;
	}
	// sent least significant byte first
	const uint8* b = reply.bdaddr.b;
	TRACE_ALWAYS("BD_ADDR %02x:%02x:%02x:%02x:%02x:%02x\n", b[5], b[4], b[3],
		b[2], b[1], b[0]);
}


/*!	Powers the controller up from reset, patches it and resets it again. A
	controller that does not answer at all fails; one that cannot be patched
	runs on its ROM.
*/
static status_t
bring_up()
{
	sDevice.bringing_up = true;

	status_t status = start_uart();
	if (status != B_OK)
		return status;
	power_cycle();

	status = reset_controller();
	if (status != B_OK) {
		ERROR("the controller does not answer (%" B_PRIu64 " bytes received, %"
			B_PRIu32 " interrupts, %" B_PRIu32 " sync errors)\n",
			sDevice.bytes_received, sDevice.interrupts, sDevice.sync_errors);
		return B_DEVICE_NOT_FOUND;
	}

	uint8* patch = NULL;
	size_t patchSize = 0;
	if (load_patch(patch, patchSize) == B_OK) {
		const bool patched = download_patch(patch, patchSize) == B_OK;
		free(patch);

		if (patched) {
			// Launch_RAM restarts the controller on the patch
			snooze(kLaunchSettleTime);
			flush_receive();
		} else {
			ERROR("patch failed, continuing with the ROM firmware\n");
			power_cycle();
		}
		if (reset_controller() != B_OK)
			return B_DEVICE_NOT_FOUND;
		log_local_version(patched ? "patched" : "ROM");
	}

	log_address();
	TRACE_ALWAYS("receive: %" B_PRIu64 " bytes, %" B_PRIu32 " interrupts%s, %"
		B_PRIu32 " FIFO overruns, %" B_PRIu32 " ring overflows, %" B_PRIu32
		" sync errors\n", sDevice.bytes_received, sDevice.interrupts,
		sDevice.polling ? " (polling)" : "", sDevice.uart_overruns,
		sDevice.ring_overflows, sDevice.sync_errors);

	sDevice.bringing_up = false;
	return B_OK;
}


//	#pragma mark - HCI module hooks


static status_t
send_buffer(uint8 indicator, net_buffer* buffer)
{
	if (buffer == NULL || buffer->size > kMaxPacketSize)
		return B_BAD_VALUE;

	// sent from a copy, under the transmit lock as the other packets are
	MutexLocker locker(sDevice.tx_lock);
	status_t status = sBufferModule->read(buffer, 0, sDevice.tx_buffer,
		buffer->size);
	if (status == B_OK) {
		status = send_packet_locked(indicator, sDevice.tx_buffer,
			buffer->size);
	}
	locker.Unlock();

	// the buffer is the transport's once sent; on failure, it stays the
	// caller's, as with the USB transport
	if (status == B_OK)
		sBufferModule->free(buffer);
	return status;
}


static status_t
hook_send_command(hci_id id, net_buffer* buffer)
{
	return send_buffer(kH4Command, buffer);
}


static status_t
hook_send_acl(hci_id id, net_buffer* buffer)
{
	return send_buffer(kH4Acl, buffer);
}


static status_t
hook_send_sco(hci_id id, net_buffer* buffer)
{
	// the controller's SCO is routed to its PCM pins
	return B_NOT_SUPPORTED;
}


static bt_hci_transport_hooks sTransportHooks = {
	hook_send_command,
	hook_send_acl,
	hook_send_sco,
	NULL,
	NULL,
	H4
};


//	#pragma mark - device


static void
stop_device()
{
	atomic_set(&sDevice.forwarding, 0);
	if (sDevice.ndev != NULL && sHci != NULL) {
		sHci->UnregisterDriver(sDevice.hci);
		sDevice.ndev = NULL;
	}

	if (sDevice.rx_thread >= 0) {
		sDevice.rx_quit = true;
		release_sem(sDevice.rx_sem);
		status_t result;
		wait_for_thread(sDevice.rx_thread, &result);
		sDevice.rx_thread = -1;
	}

	stop_uart();
	if (sDevice.interrupt_installed) {
		remove_io_interrupt_handler(sDevice.interrupt, &aux_interrupt, NULL);
		sDevice.interrupt_installed = false;
	}
	set_power(false);

	delete_sem(sDevice.rx_sem);
	delete_sem(sDevice.reply_sem);
	sDevice.rx_sem = sDevice.reply_sem = -1;

	if (sBufferModule != NULL)
		put_module(NET_BUFFER_MODULE_NAME);
	if (sHci != NULL)
		put_module(BT_HCI_MODULE_NAME);
	sBufferModule = NULL;
	sHci = NULL;
}


static status_t
start_device()
{
	// everything but the hardware and the locks
	sDevice.ndev = NULL;
	sDevice.hci = -1;
	sDevice.forwarding = 0;
	sDevice.ring_head = sDevice.ring_tail = 0;
	sDevice.ring_overflows = sDevice.uart_overruns = sDevice.interrupts = 0;
	sDevice.empty_interrupts = 0;
	sDevice.bytes_received = 0;
	sDevice.interrupt_storm = false;
	sDevice.rx_quit = false;
	sDevice.polling = false;
	sDevice.parser_reset = 0;
	sDevice.packet_type = 0;
	sDevice.sync_errors = 0;
	sDevice.waiting = false;
	sDevice.commands_logged = 0;
	memset(&sDevice.stats, 0, sizeof(sDevice.stats));

	// the stack is loaded only for an open device, not whenever devfs scans
	// the drivers
	status_t status = get_module(BT_HCI_MODULE_NAME, (module_info**)&sHci);
	if (status != B_OK) {
		sHci = NULL;
		ERROR("no HCI module: %s\n", strerror(status));
		return status;
	}
	status = get_module(NET_BUFFER_MODULE_NAME, (module_info**)&sBufferModule);
	if (status != B_OK) {
		sBufferModule = NULL;
		return status;
	}

	sDevice.rx_sem = create_sem(0, "bcm2711_bt receive");
	sDevice.reply_sem = create_sem(0, "bcm2711_bt reply");
	if (sDevice.rx_sem < 0 || sDevice.reply_sem < 0)
		return B_NO_MORE_SEMS;

	sDevice.rx_thread = spawn_kernel_thread(&receive_thread, "bcm2711_bt rx",
		B_REAL_TIME_DISPLAY_PRIORITY, NULL);
	if (sDevice.rx_thread < 0)
		return sDevice.rx_thread;
	resume_thread(sDevice.rx_thread);

	// the receive interrupt is enabled by start_uart(), once this is in place
	status = install_io_interrupt_handler(sDevice.interrupt,
		&aux_interrupt, NULL, 0);
	if (status == B_OK)
		sDevice.interrupt_installed = true;
	else {
		ERROR("cannot install the handler of interrupt %" B_PRId32 ": %s\n",
			sDevice.interrupt, strerror(status));
	}

	status = bring_up();
	if (status != B_OK)
		return status;

	bluetooth_device* ndev;
	status = sHci->RegisterDriver(&sTransportHooks, &ndev);
	if (status != B_OK) {
		ERROR("cannot register with the HCI module: %s\n", strerror(status));
		return status;
	}
	sDevice.ndev = ndev;
	sDevice.hci = ndev->index;
	TRACE_ALWAYS("ready, HCI device %#" B_PRIx32 "\n", sDevice.hci);
	return B_OK;
}


static status_t
bcm2711_bt_open(const char* name, uint32 flags, void** _cookie)
{
	MutexLocker locker(sDevice.lock);
	if (sDevice.open)
		return B_BUSY;

	status_t status = start_device();
	if (status != B_OK) {
		ERROR("bring-up failed: %s\n", strerror(status));
		stop_device();
		return status;
	}

	sDevice.open = true;
	*_cookie = &sDevice;
	return B_OK;
}


static status_t
bcm2711_bt_close(void* cookie)
{
	return B_OK;
}


static status_t
bcm2711_bt_free(void* cookie)
{
	MutexLocker locker(sDevice.lock);
	TRACE_ALWAYS("closed: %" B_PRIu64 " bytes received, %" B_PRIu32
		" interrupts%s, %" B_PRIu32 " FIFO overruns, %" B_PRIu32
		" ring overflows, %" B_PRIu32 " sync errors\n", sDevice.bytes_received,
		sDevice.interrupts, sDevice.polling ? " (polling)" : "",
		sDevice.uart_overruns, sDevice.ring_overflows, sDevice.sync_errors);
	stop_device();
	sDevice.open = false;
	return B_OK;
}


static status_t
bcm2711_bt_control(void* cookie, uint32 op, void* buffer, size_t length)
{
	if (buffer == NULL || !IS_USER_ADDRESS(buffer))
		return B_BAD_ADDRESS;

	switch (op) {
		case ISSUE_BT_COMMAND:
		{
			uint8 command[kMaxCommandSize];
			if (length < sizeof(hci_command_header) || length > sizeof(command))
				return B_BAD_VALUE;
			if (user_memcpy(command, buffer, length) != B_OK)
				return B_BAD_ADDRESS;

			if (sDevice.commands_logged < kLoggedCommands) {
				sDevice.commands_logged++;
				TRACE_ALWAYS("command %#x from the server\n",
					command[0] | (command[1] << 8));
			}
			return send_packet(kH4Command, command, length);
		}

		case BT_UP:
			atomic_set(&sDevice.forwarding, 1);
			TRACE_ALWAYS("up, events and data go to the stack\n");
			return B_OK;

		case GET_STATS:
			return user_memcpy(buffer, &sDevice.stats,
				sizeof(bt_hci_statistics));

		case GET_HCI_ID:
			return user_memcpy(buffer, &sDevice.hci, sizeof(hci_id));
	}
	return B_DEV_INVALID_IOCTL;
}


static status_t
bcm2711_bt_read(void* cookie, off_t position, void* buffer, size_t* _length)
{
	*_length = 0;
	return B_OK;
}


static status_t
bcm2711_bt_write(void* cookie, off_t position, const void* buffer,
	size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static device_hooks sDeviceHooks = {
	bcm2711_bt_open,
	bcm2711_bt_close,
	bcm2711_bt_free,
	bcm2711_bt_control,
	bcm2711_bt_read,
	bcm2711_bt_write,
};

static const char* sDeviceNames[] = {
	DEVICE_NAME,
	NULL
};


//	#pragma mark - driver


static void
free_resources()
{
	delete_area(sDevice.gpio_area);
	delete_area(sDevice.aux_area);
	if (sMailbox != NULL)
		put_module(BCM2835_MAILBOX_MODULE_NAME);
	sMailbox = NULL;
	mutex_destroy(&sDevice.tx_lock);
	mutex_destroy(&sDevice.reply_lock);
	mutex_destroy(&sDevice.lock);
}


extern "C" status_t
init_hardware(void)
{
	if (gFDT == NULL || fdt_node_check_compatible(gFDT, 0, "brcm,bcm2711") != 0
		|| find_compatible("brcm,bcm2835-aux") < 0
		|| find_compatible("brcm,bcm2835-aux-uart") < 0) {
		return B_DEVICE_NOT_FOUND;
	}
	return B_OK;
}


extern "C" status_t
init_driver(void)
{
	memset(&sDevice, 0, sizeof(sDevice));
	sDevice.aux_area = sDevice.gpio_area = -1;
	sDevice.rx_sem = sDevice.reply_sem = -1;
	sDevice.rx_thread = -1;
	sDevice.hci = -1;
	B_INITIALIZE_SPINLOCK(&sDevice.ring_lock);
	mutex_init(&sDevice.lock, "bcm2711_bt");
	mutex_init(&sDevice.reply_lock, "bcm2711_bt reply");
	mutex_init(&sDevice.tx_lock, "bcm2711_bt transmit");

	status_t status = get_module(BCM2835_MAILBOX_MODULE_NAME,
		(module_info**)&sMailbox);
	if (status == B_OK) {
		status = map_registers(find_compatible("brcm,bcm2835-aux"),
			"bcm2711_bt aux", sDevice.aux_area, sDevice.aux);
	}
	if (status == B_OK) {
		status = map_registers(find_compatible("brcm,bcm2711-gpio"),
			"bcm2711_bt gpio", sDevice.gpio_area, sDevice.gpio);
	}
	if (status == B_OK) {
		status = find_interrupt(find_compatible("brcm,bcm2835-aux-uart"),
			sDevice.interrupt);
	}
	if (status != B_OK) {
		ERROR("init failed: %s\n", strerror(status));
		free_resources();
		return status;
	}

	TRACE_ALWAYS("AUX mini-UART, interrupt %" B_PRId32 "\n",
		sDevice.interrupt);
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
