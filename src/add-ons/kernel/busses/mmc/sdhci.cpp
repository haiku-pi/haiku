/*
 * Copyright 2018-2025 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *		B Krishnan Iyer, krishnaniyer97@gmail.com
 *		Adrien Destugues, pulkomandy@pulkomandy.tk
 *		Ron Ben Aroya, sed4906birdie@gmail.com
 */


#include <algorithm>
#include <new>
#include <stdio.h>
#include <string.h>

#include <bus/PCI.h>
#include <ACPI.h>
#include "acpi.h"

#include <util/AutoLock.h>
#include <vm/vm.h>

#if defined(__aarch64__)
#	include <arch/cpu.h>
#	include <arch_kernel.h>
#endif

#include <KernelExport.h>

#include "IOSchedulerSimple.h"
#include "mmc.h"
#include "sdhci.h"


//#define TRACE_SDHCI
#ifdef TRACE_SDHCI
#	define TRACE(x...) dprintf("\33[33msdhci:\33[0m " x)
#else
#	define TRACE(x...) ;
#endif
#define TRACE_ALWAYS(x...)	dprintf("\33[33msdhci:\33[0m " x)
#define ERROR(x...)			dprintf("\33[33msdhci:\33[0m " x)
#define CALLED(x...)		TRACE("CALLED %s\n", __PRETTY_FUNCTION__)


#define SDHCI_DEVICE_MODULE_NAME "busses/mmc/sdhci/driver_v1"


device_manager_info* gDeviceManager;
device_module_info* gMMCBusController;


static int32
sdhci_generic_interrupt(void* data)
{
	SdhciBus* bus = (SdhciBus*)data;
	return bus->HandleInterrupt();
}


SdhciBus::SdhciBus(struct registers* registers, uint8_t irq, bool poll,
	const sdhci_quirks* quirks)
	:
	fRegisters(registers),
	fIrq(irq),
	fCardType(CARD_TYPE_UNKNOWN),
	fBaseClockKHz(quirks != NULL ? quirks->base_clock_khz : 0),
	fNonRemovable(quirks != NULL && quirks->non_removable),
	fNoHighSpeedBit(quirks != NULL && quirks->no_high_speed_bit),
	fAssume3v3(quirks != NULL && quirks->assume_3v3),
	fDmaBusOffset(0),
	fDmaHighAddress(0x100000000LL),
	fDmaCoherent(true),
	fUseDma(true),
	fUseAdma(false),
	fAdmaArea(-1),
	fAdmaTable(NULL),
	fAdmaTablePhysical(0)
{
	if (irq == 0 || irq == 0xff) {
		ERROR("IRQ not assigned\n");
		fStatus = B_BAD_DATA;
		return;
	}

	fInterruptNotifier.Init(this, "SDHCI interrupts");
	fCardInterruptSemaphore = -1;
	B_INITIALIZE_SPINLOCK(&fCardInterruptLock);

	DisableInterrupts();

	fStatus = install_io_interrupt_handler(fIrq,
		sdhci_generic_interrupt, this, 0);

	if (fStatus != B_OK) {
		ERROR("can't install interrupt handler\n");
		return;
	}

	// First of all, we have to make sure we are in a sane state. The easiest
	// way is to reset everything.
	Reset();

	TRACE("Controller spec version: %d, vendor version: %#02x\n",
		fRegisters->host_controller_version.SpecVersion(),
		fRegisters->host_controller_version.VendorVersion());

	TRACE("Capabilities: %s%s%s%s%s%s%s%s%s%s%s%s%s%s\n"
		"    Clock multiplier: %" PRIx8 "\n"
		"    Retuning modes: %" PRIx8 "\n"
		"    Retuning timer count: %" PRIx8 "\n"
		"    Slot type: %" PRIx8 "\n"
		"    Supported voltages: %" PRIx8 "\n"
		"    Max block length: %" PRIx8 "\n"
		"    Base clock frequency: %" PRId8 " MHz\n"
		"    Timeout clock: %" PRIu32 " kHz\n",
		fRegisters->capabilities.UseTuningForSDR50() ? "SDR50 needs retuning, " : "",
		fRegisters->capabilities.TypeDSupport() ? "Type-D, " : "",
		fRegisters->capabilities.TypeCSupport() ? "Type-C, " : "",
		fRegisters->capabilities.TypeASupport() ? "Type-A, " : "",
		fRegisters->capabilities.DDR50Support() ? "DDR50, " : "",
		fRegisters->capabilities.SDR104Support() ? "SDR104, " : "",
		fRegisters->capabilities.SDR50Support() ? "SDR50, " : "",
		fRegisters->capabilities.AsynchronousInterrupts() ? "Asynchronous interrupts, " : "",
		fRegisters->capabilities.SystemBus64Bits() ? "64-bit system bus, " : "",
		fRegisters->capabilities.SuspendResume() ? "Suspend/Resume, " : "",
		fRegisters->capabilities.SimpleDMA() ? "Simple DMA, " : "",
		fRegisters->capabilities.HighSpeed() ? "High speed, " : "",
		fRegisters->capabilities.AdvancedDMA() ? "Advanced DMA, " : "",
		fRegisters->capabilities.Embedded8Bit() ? "8-bit Embedded mode, " : "",
		fRegisters->capabilities.ClockMultiplier(),
		fRegisters->capabilities.RetuningModes(),
		fRegisters->capabilities.RetuningTimerCount(),
		fRegisters->capabilities.SlotType(),
		fRegisters->capabilities.SupportedVoltages(),
		fRegisters->capabilities.MaxBlockLength(),
		fRegisters->capabilities.BaseClockFrequency(),
		fRegisters->capabilities.TimeoutClockFrequency());
	fUseDma = fRegisters->capabilities.SimpleDMA()
		&& (quirks == NULL || !quirks->force_pio);
	if (!fUseDma)
		TRACE_ALWAYS("no Simple DMA, transferring data by PIO\n");

	TRACE("Initial host control: %x\n", fRegisters->host_control.Bits());
	TRACE("Initial host control 2: %x\n", (uint16)fRegisters->host_control_2);

	if (fRegisters->host_controller_version.SpecVersion() > 3) {
		// TODO proper class for manipulating host_control_2
		fRegisters->host_control_2 &= ~(1<<12);
		TRACE("Host control 2 after disabling v4 DMA mode: %x\n",
			(uint16)fRegisters->host_control_2);
	}

	// Turn on the power supply to the card, if there is a card inserted
	if (PowerOn()) {
		// Then we configure the clock to the frequency needed for
		// initialization
		SetClock(400, false);
	}

	// Without a timeout clock in the capabilities, assume it is the base clock
	// (the longest timeout that may give)
	uint32 timeoutClockKHz = fRegisters->capabilities.TimeoutClockFrequency();
	if (timeoutClockKHz == 0)
		timeoutClockKHz = fBaseClockKHz;
	fRegisters->timeout_control.SetDivider(timeoutClockKHz, 500);

	// Finally, configure some useful interrupts
	EnableInterrupts(SDHCI_INT_CMD_MASK | SDHCI_INT_CARD_REM
		| SDHCI_INT_TRANSFER_MASK);

	// We want to see the other bits in the status register, but not have an
	// interrupt trigger on them (we get a "command complete" interrupt on
	// errors already). The SDIO card interrupt is a level that stays set until
	// its function driver services the card: it is not used yet.
	fRegisters->interrupt_status_enable |= SDHCI_INT_ERROR_MASK
		| (SDHCI_INT_NORMAL_MASK & ~SDHCI_INT_CARD_STATUS);

	if (poll) {
		// Spawn a polling thread, as the interrupts won't currently work on ACPI.
		fWorkerThread = spawn_kernel_thread(_WorkerThread, "SD bus poller",
			B_NORMAL_PRIORITY, this);
		resume_thread(fWorkerThread);
	}
}


SdhciBus::~SdhciBus()
{
	TerminateBus();

	if (fIrq != 0)
		remove_io_interrupt_handler(fIrq, sdhci_generic_interrupt, this);

	area_id regs_area = area_for(fRegisters);
	delete_area(regs_area);
	if (fAdmaArea >= 0)
		delete_area(fAdmaArea);

	fStatus = B_SHUTTING_DOWN;

	status_t result;
	if (fWorkerThread != 0)
		wait_for_thread(fWorkerThread, &result);
}


void
SdhciBus::EnableInterrupts(uint32_t mask)
{
	fRegisters->interrupt_status_enable |= mask;
	fRegisters->interrupt_signal_enable |= mask;
}


void
SdhciBus::DisableInterrupts()
{
	fRegisters->interrupt_status_enable = 0;
	fRegisters->interrupt_signal_enable = 0;
}


// #pragma mark -
/*
PartA2, SD Host Controller Simplified Specification, Version 4.20
§3.7.1.1 The sequence to issue an SD Command
*/
status_t
SdhciBus::ExecuteCommand(uint8_t command, uint32_t argument, uint32_t* response)
{
	TRACE("ExecuteCommand(%d, %x)\n", command, argument);

	uint32_t replyType;
	uint16 transferMode = 0;

	switch (command) {
		// Basic reply types
		case GO_IDLE_STATE:
			replyType = Command::kNoReplyType;
			break;
		case SD_APP_CMD:
		case SD_ERASE_WR_BLK_START:
		case SD_ERASE_WR_BLK_END:
			replyType = Command::kR1Type;
			break;
		case SELECT_DESELECT_CARD:
		case SD_ERASE:
			replyType = Command::kR1bType;
			break;
		case ALL_SEND_CID:
		case SEND_CSD:
			replyType = Command::kR2Type;
			break;
		case MMC_SEND_OP_COND:
		case SD_SEND_OP_COND: // SD Application command
		case SDIO_SEND_OP_COND:
			// R4 for SDIO has the shape of R3: no CRC, no index
			replyType = Command::kR3Type;
			break;
		case SDIO_RW_DIRECT:
			// R5 has the shape of R1
			replyType = Command::kR1Type;
			break;

		// Commands defined with different reply types in SD and MMC specifications
		case SD_SET_BUS_WIDTH: // SD application command. Also MMC_SWITCH, which is not.
			if (fCardType == CARD_TYPE_MMC)
				replyType = Command::kR1bType;
			else
				replyType = Command::kR1Type;
			break;
		case SD_SEND_RELATIVE_ADDR: // also MMC_SET_RELATIVE_ADDR
			if (fCardType == CARD_TYPE_MMC)
				replyType = Command::kR1Type;
			else
				replyType = Command::kR6Type;
			break;
		case SD_SEND_IF_COND: // also MMC_SEND_EXT_CSD
			if (fCardType == CARD_TYPE_MMC)
				replyType = Command::kR1Type;
			else
				replyType = Command::kR7Type;
			break;

		// Commands with data transfer replies, also set transferMode
		case SD_READ_SINGLE_BLOCK:
			transferMode = TransferMode::kRead | TransferMode::kDmaEnable;
			replyType = Command::kR1Type | Command::kDataPresent;
			break;
		case SD_READ_MULTIPLE_BLOCKS:
			transferMode = TransferMode::kRead | TransferMode::kMulti
				| TransferMode::kAutoCmd12Enable | TransferMode::kBlockCountEnable
				| TransferMode::kDmaEnable;
			replyType = Command::kR1Type | Command::kDataPresent;
			break;
		case SD_WRITE_SINGLE_BLOCK:
			transferMode = TransferMode::kWrite | TransferMode::kDmaEnable;
			replyType = Command::kR1Type | Command::kDataPresent;
			break;
		case SD_WRITE_MULTIPLE_BLOCKS:
			transferMode = TransferMode::kWrite | TransferMode::kMulti
				| TransferMode::kAutoCmd12Enable | TransferMode::kBlockCountEnable
				| TransferMode::kDmaEnable;
			replyType = Command::kR1Type | Command::kDataPresent;
			break;
		default:
			ERROR("Unknown command %x\n", command);
			return B_BAD_DATA;
	}

	return _ExecuteCommand(command, argument, response, replyType,
		transferMode);
}


status_t
SdhciBus::_ExecuteCommand(uint8_t command, uint32_t argument,
	uint32_t* response, uint32_t replyType, uint16 transferMode)
{
	// First of all clear the result
	fCommandResult = 0;

	// Check if it's possible to send a command right now.
	// It is not possible to send a command as long as the command line is busy.
	// The spec says we should wait, but we can't do that on kernel side, since
	// it leaves no chance for the upper layers to handle the problem. So we
	// just say we're busy and the caller can retry later.
	// Note that this should normally never happen: the command line is busy
	// only during command execution, and we don't leave this function with a
	// command running.
	if (fRegisters->present_state.CommandInhibit()) {
		TRACE_ALWAYS("Command execution impossible, command inhibit\n");
		return B_BUSY;
	}
	if (fRegisters->present_state.DataInhibit() && command != SD_IO_ABORT) {
		TRACE_ALWAYS("Command execution unwise, data inhibit\n");
		return B_BUSY;
	}

	// Get ready to accet interrupts that will occur during the command
	ConditionVariableEntry waiter;
	fInterruptNotifier.Add(&waiter);

	// Check if DATA line is available (if needed)
	if ((replyType & Command::k32BitResponseCheckBusy) != 0
		&& command != SD_STOP_TRANSMISSION && command != SD_IO_ABORT) {
		if (fRegisters->present_state.DataInhibit()) {
			ERROR("Execution aborted, data inhibit\n");
			return B_BUSY;
		}
	}

	if (fRegisters->present_state.CommandInhibit()) {
		ERROR("Command line busy at start of execute command\n");
		fRegisters->software_reset.ResetCommandLine();
		return B_BUSY;
	}

	fRegisters->argument = argument;

	if (!fUseDma)
		transferMode &= ~TransferMode::kDmaEnable;
	fRegisters->command.SendCommand(command, replyType, transferMode);

	// Wait for command response to be available ("command complete" interrupt)
	TRACE("Wait for command complete...");

	// A command completes within microseconds, much less than it takes to
	// wake a thread up: wait a moment for the interrupt handler before
	// sleeping (it matters for SDIO, with several commands per packet).
	const bigtime_t spinEnd = system_time() + 50;
	while (*(volatile uint32_t*)&fCommandResult == 0
		&& system_time() < spinEnd) {
		cpu_pause();
	}

	while (*(volatile uint32_t*)&fCommandResult == 0) {
		status_t result = waiter.Wait(B_RELATIVE_TIMEOUT, 1000000);
		if (result == B_TIMED_OUT) {
			ERROR("Command %d: no completion interrupt for a while, status %x, "
				"present state %x\n", command, fRegisters->interrupt_status,
				fRegisters->present_state.Bits());
		} else if (result != B_OK)
			panic("sdhci: Failed to wait for command complete: %s", strerror(result));

		fInterruptNotifier.Add(&waiter);
		TRACE("Command status: %x\n", fCommandResult);
		TRACE("real status = %x command line busy: %d\n",
			fRegisters->interrupt_status,
			fRegisters->present_state.CommandInhibit());
	}

	TRACE("Command response available\n");

	if (fCommandResult & SDHCI_INT_ERROR) {
		// TODO is it a good idea to clear interrupts here from outside the interrupt handler?
		fRegisters->interrupt_status = fCommandResult;
		if (fCommandResult & SDHCI_INT_COMMAND_TIMEOUT) {
			// a probe (CMD5, CMD8) times out on cards that do not know it
			TRACE("Command execution timed out\n");
			// At this point, the "command inhibit" bit is not set yet, it will be set only after
			// another command is sent while the controller is in the timeout state.
			// But resetting the controller state pre-emptively will allow to send another command.
			//
			// Clear the data line at the same time if it is busy
			fRegisters->software_reset.ResetCommandAndDataLines();
			return B_TIMED_OUT;
		}
		if (fCommandResult & SDHCI_INT_COMMAND_CRC) {
			ERROR("CRC error\n");
			return B_BAD_VALUE;
		}
		ERROR("Command execution failed %x\n", fCommandResult);
		fRegisters->software_reset.ResetCommandAndDataLines();
		// TODO look at errors in interrupt_status register for more details
		// and return a more appropriate error code
		return (fCommandResult & SDHCI_INT_DATA_ERROR_MASK) != 0
			? B_IO_ERROR : B_ERROR;
	}

	// The command inhibit bit may clear a moment after the interrupt
	const bigtime_t inhibitEnd = system_time() + 1000;
	while (fRegisters->present_state.CommandInhibit()
		&& system_time() < inhibitEnd) {
		cpu_pause();
	}
	if (fRegisters->present_state.CommandInhibit()) {
		TRACE("Command execution failed, card stalled\n");
		// Clear the stall
		fRegisters->software_reset.ResetCommandLine();
		return B_ERROR;
	}

	switch (replyType & Command::kReplySizeMask) {
		case Command::k32BitResponse:
			*response = fRegisters->response[0];
			break;
		case Command::k128BitResponse:
			response[0] = fRegisters->response[0];
			response[1] = fRegisters->response[1];
			response[2] = fRegisters->response[2];
			response[3] = fRegisters->response[3];
			break;

		default:
			// No response
			break;
	}

	if ((replyType == Command::kR1bType)
			&& (fCommandResult & SDHCI_INT_TRANSFER_MASK) == 0) {
		// R1b commands may use the data line so we must wait for the
		// "transfer complete" interrupt here.
		TRACE("Waiting for data line...\n");
		fInterruptNotifier.Add(&waiter);
		while (fRegisters->present_state.DataInhibit()) {
			status_t result = waiter.Wait();
			if (result != B_OK)
				panic("sdhci: Failed to wait for data line release: %s", strerror(result));
			fInterruptNotifier.Add(&waiter);
		}
		TRACE("Dataline is released.\n");
	}

	TRACE("Command execution %d complete\n", command);
	return B_OK;
}


/*!	Executes a command whose data phase reads \a length bytes (at most a
	block) into \a buffer, by PIO.
*/
status_t
SdhciBus::ExecuteDataCommand(uint8_t command, uint32_t argument, void* buffer,
	size_t length)
{
	if (length == 0 || length > 512 || (length % 4) != 0)
		return B_BAD_VALUE;

	fRegisters->block_size.ConfigureTransfer(length, BlockSize::kDmaBoundary512K,
		1);

	uint32_t response;
	status_t status = _ExecuteCommand(command, argument, &response,
		Command::kR1Type | Command::kDataPresent, TransferMode::kRead);
	if (status != B_OK)
		return status;

	const bigtime_t timeout = system_time() + 1000000;
	uint32_t interrupts;
	while (((interrupts = fRegisters->interrupt_status)
			& SDHCI_INT_BUF_READ_READY) == 0) {
		if ((interrupts & SDHCI_INT_ERROR) != 0 || system_time() > timeout) {
			fRegisters->software_reset.ResetDataLine();
			return B_IO_ERROR;
		}
		spin(1);
	}
	fRegisters->interrupt_status = SDHCI_INT_BUF_READ_READY;
	for (size_t i = 0; i < length / 4; i++)
		((uint32_t*)buffer)[i] = fRegisters->buffer_data_port;

	// the transfer complete interrupt ends the command
	while ((fCommandResult & SDHCI_INT_TRANSFER_MASK) == 0) {
		if (system_time() > timeout) {
			fRegisters->software_reset.ResetDataLine();
			return B_TIMED_OUT;
		}
		snooze(100);
	}
	return (fCommandResult & SDHCI_INT_DATA_ERROR_MASK) != 0 ? B_IO_ERROR : B_OK;
}


/*!	Executes a command whose data phase moves \a blockCount blocks of
	\a blockSize bytes (at most 2048, not necessarily a multiple of 4) between
	the card and the kernel buffer \a buffer, through the buffer data port:
	SDIO's IO_RW_EXTENDED. The buffer is ready when the Present State
	register says so; its latched interrupt status bit can be missed.
*/
status_t
SdhciBus::ExecuteIoCommand(uint8_t command, uint32_t argument, void* buffer,
	size_t blockSize, uint32_t blockCount, bool isWrite, uint32_t* response)
{
	if (blockSize == 0 || blockSize > 2048 || blockCount == 0
		|| blockCount > 0xffff) {
		return B_BAD_VALUE;
	}

	fRegisters->block_size.ConfigureTransfer(blockSize,
		BlockSize::kDmaBoundary512K, blockCount);

	uint16 transferMode = isWrite ? TransferMode::kWrite : TransferMode::kRead;
	if (blockCount > 1)
		transferMode |= TransferMode::kMulti | TransferMode::kBlockCountEnable;

	status_t status = _ExecuteCommand(command, argument, response,
		Command::kR1Type | Command::kDataPresent, transferMode);
	if (status != B_OK)
		return status;

	const uint32_t ready = isWrite
		? SDHCI_INT_BUF_WRITE_READY : SDHCI_INT_BUF_READ_READY;
	uint8_t* data = (uint8_t*)buffer;
	for (uint32_t block = 0; block < blockCount; block++) {
		const bigtime_t timeout = system_time() + 1000000;
		while (!(isWrite ? fRegisters->present_state.BufferWriteEnable()
				: fRegisters->present_state.BufferReadEnable())) {
			if ((fCommandResult & SDHCI_INT_DATA_ERROR_MASK) != 0
				|| system_time() > timeout) {
				ERROR("CMD%d %#" B_PRIx32 ": block %" B_PRIu32 " not ready, "
					"response %#" B_PRIx32 ", status %" B_PRIx32 ", present state %"
					B_PRIx32 ", host control %x, block %#" B_PRIx32 "\n", command,
					argument, block, *response,
					fCommandResult | fRegisters->interrupt_status,
					fRegisters->present_state.Bits(),
					fRegisters->host_control.Bits(),
					fRegisters->block_size.Bits());
				fRegisters->software_reset.ResetCommandAndDataLines();
				return B_IO_ERROR;
			}
			spin(1);
		}
		fRegisters->interrupt_status = ready;

		for (size_t done = 0; done < blockSize; done += 4) {
			const size_t chunk = min_c(blockSize - done, 4);
			uint32_t word = 0;
			if (isWrite) {
				memcpy(&word, data + done, chunk);
				fRegisters->buffer_data_port = word;
			} else {
				word = fRegisters->buffer_data_port;
				memcpy(data + done, &word, chunk);
			}
		}
		data += blockSize;
	}

	// the transfer complete interrupt ends the command
	const bigtime_t timeout = system_time() + 1000000;
	while ((fCommandResult & SDHCI_INT_TRANSFER_MASK) == 0) {
		if (system_time() > timeout) {
			ERROR("CMD%d: no transfer complete\n", command);
			fRegisters->software_reset.ResetCommandAndDataLines();
			return B_TIMED_OUT;
		}
		spin(10);
	}
	if ((fCommandResult & SDHCI_INT_DATA_ERROR_MASK) != 0) {
		ERROR("CMD%d: data error %" B_PRIx32 "\n", command, fCommandResult);
		fRegisters->software_reset.ResetCommandAndDataLines();
		return B_IO_ERROR;
	}
	return B_OK;
}


status_t
SdhciBus::InitCheck()
{
	return fStatus;
}


/*!	Configures how the controller reaches memory: \a busOffset is added to
	physical addresses to obtain the controller's DMA addresses, it can reach
	physical addresses below \a highAddress, and \a coherent tells whether it
	snoops the CPU caches.
*/
void
SdhciBus::SetDmaWindow(phys_addr_t busOffset, phys_addr_t highAddress,
	bool coherent)
{
	fDmaBusOffset = busOffset;
	fDmaHighAddress = highAddress;
	fDmaCoherent = coherent;

	if (fUseDma)
		fUseAdma = _InitAdma();
}


/*!	Uses ADMA2 with 32-bit descriptors if the controller supports it: one
	descriptor table, within the controller's reach.
*/
bool
SdhciBus::_InitAdma()
{
	if (!fRegisters->capabilities.AdvancedDMA())
		return false;

	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = min_c(fDmaHighAddress, 0x100000000LL);
	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	fAdmaArea = create_area_etc(B_SYSTEM_TEAM, "sdhci adma2 descriptors",
		B_PAGE_SIZE, B_CONTIGUOUS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0,
		&virtualRestrictions, &physicalRestrictions, (void**)&fAdmaTable);
	if (fAdmaArea < 0) {
		ERROR("no memory for the ADMA2 descriptors, using SDMA\n");
		return false;
	}

	physical_entry entry;
	get_memory_map(fAdmaTable, B_PAGE_SIZE, &entry, 1);
	fAdmaTablePhysical = entry.address;

	fRegisters->host_control.SetDMAMode(HostControl::kAdma32);
	TRACE_ALWAYS("using ADMA2\n");
	return true;
}


/*!	Describes [address, address + length) to the ADMA2 engine. Chunks stay
	below 64 KiB, so that no length has to be encoded as 0.
*/
void
SdhciBus::_SetupAdma(phys_addr_t address, size_t length)
{
	static const size_t kMaxChunk = 32 * 1024;
	static const size_t kMaxDescriptors
		= B_PAGE_SIZE / sizeof(struct adma2_descriptor);

	size_t count = 0;
	for (size_t done = 0; done < length; count++) {
		ASSERT(count < kMaxDescriptors);
		const size_t chunk = min_c(length - done, kMaxChunk);
		fAdmaTable[count].attributes = ADMA2_VALID | ADMA2_TRANSFER;
		fAdmaTable[count].length = chunk;
		fAdmaTable[count].address = address + done + fDmaBusOffset;
		done += chunk;
	}
	fAdmaTable[count - 1].attributes |= ADMA2_END;

	_SyncForDevice(fAdmaTablePhysical,
		count * sizeof(struct adma2_descriptor));

	// The DMA mode is lost when the controller is reset.
	fRegisters->host_control.SetDMAMode(HostControl::kAdma32);
	fRegisters->adma_system_address = fAdmaTablePhysical + fDmaBusOffset;
	fRegisters->adma_system_address_high = 0;
}


/*!	Moves the data of a transfer through the buffer data port, one block at
	a time, for controllers that cannot do DMA.
*/
status_t
SdhciBus::_TransferPio(phys_addr_t address, size_t length, bool isWrite)
{
	static const size_t kBlockSize = 512;
	const uint32_t ready = isWrite
		? SDHCI_INT_BUF_WRITE_READY : SDHCI_INT_BUF_READ_READY;
	uint32_t block[kBlockSize / sizeof(uint32_t)];

	for (size_t done = 0; done < length; done += kBlockSize) {
		const bigtime_t timeout = system_time() + 1000000;
		uint32_t status;
		while (((status = fRegisters->interrupt_status) & ready) == 0) {
			if ((status & SDHCI_INT_ERROR) != 0)
				return B_IO_ERROR;
			if (system_time() > timeout)
				return B_TIMED_OUT;
			spin(1);
		}
		fRegisters->interrupt_status = ready;

		if (isWrite) {
			vm_memcpy_from_physical(block, address + done, kBlockSize, false);
			for (size_t i = 0; i < B_COUNT_OF(block); i++)
				fRegisters->buffer_data_port = block[i];
		} else {
			for (size_t i = 0; i < B_COUNT_OF(block); i++)
				block[i] = fRegisters->buffer_data_port;
			vm_memcpy_to_physical(address + done, block, kBlockSize, false);
		}
	}
	return B_OK;
}


void
SdhciBus::_SyncForDevice(phys_addr_t address, size_t length)
{
	if (fDmaCoherent)
		return;

#if defined(__aarch64__)
	arm64_dcache_flush_range(KERNEL_PMAP_BASE + address, length);
#else
	panic("sdhci: non-coherent DMA is not supported on this architecture");
#endif
}


void
SdhciBus::Reset()
{
	if (!fRegisters->software_reset.ResetAll())
		ERROR("SdhciBus::Reset: SoftwareReset timeout\n");
}


void
SdhciBus::SetClock(int kilohertz, bool allowAuto)
{
	// The preset values are derived from the controller's own idea of its base
	// clock, so they are not used when the platform had to provide it.
	if (allowAuto && fBaseClockKHz == 0
		&& fRegisters->host_controller_version.SpecVersion() > 2) {
		TRACE("Ignoring set_clock, controller support presets\n");
		fRegisters->host_control_2 |= (1<<15);
		TRACE("Host control 2 after enabling preset mode: %x\n",
			(uint16)fRegisters->host_control_2);
		return;
	}

	int base_clock = fBaseClockKHz != 0
		? fBaseClockKHz : fRegisters->capabilities.BaseClockFrequency() * 1000;
	// Try to get as close to the requested frequency as possible, but not faster
	int divider = (base_clock + kilohertz - 1) / kilohertz;

	if (fRegisters->host_controller_version.SpecVersion() <= 1) {
		// Old controller only support power of two dividers up to 256,
		// round to next power of two up to 256
		if (divider > 256)
			divider = 256;

		divider--;
		divider |= divider >> 1;
		divider |= divider >> 2;
		divider |= divider >> 4;
		divider++;
	}

	divider = fRegisters->clock_control.SetDivider(divider);

	// Log the value after possible rounding by SetDivider (only even values
	// are allowed).
	TRACE("SDCLK frequency: requested %dkHz, effective %dkHz / %d = %dkHz\n", kilohertz,
		base_clock, divider, base_clock / divider);

	fRegisters->host_control.SetHighSpeed(kilohertz > 25000 && !fNoHighSpeedBit);

	// We have set the divider, now we can enable the internal clock.
	fRegisters->clock_control.EnableInternal();

	// wait until internal clock is stabilized
	while (!(fRegisters->clock_control.InternalStable()));

	fRegisters->clock_control.EnablePLL();
	while (!(fRegisters->clock_control.InternalStable()));

	// Finally, route the clock to the SD card
	fRegisters->clock_control.EnableSD();
}


status_t
SdhciBus::DoIO(uint8_t command, IOOperation* operation, bool offsetAsSectors)
{
	bool isWrite = operation->IsWrite();

	static const uint32 kBlockSize = 512;
	off_t offset = operation->Offset();
	generic_size_t length = operation->Length();

	TRACE("%s %" B_PRIuGENADDR " bytes at %" B_PRIdOFF "\n",
		isWrite ? "Write" : "Read", length, offset);

	// Check that the IO scheduler did its job in following our DMA restrictions
	// We can start a read only at a sector boundary
	ASSERT(offset % kBlockSize == 0);
	// We can only read complete sectors
	ASSERT(length % kBlockSize == 0);

	const generic_io_vec* vecs = operation->Vecs();
	generic_size_t vecOffset = 0;

	status_t result = B_OK;

	while (length > 0) {
		size_t toCopy = std::min((generic_size_t)length,
			vecs->length - vecOffset);

		// If the current vec is empty, we can move to the next
		if (toCopy == 0) {
			vecs++;
			vecOffset = 0;
			continue;
		}

		// Follow steps from SD Host Controller Simplified Specification Version 4.20
		// section 3.7.2.2.

		// With SDMA we can only transfer multiples of 1 sector
		ASSERT(toCopy % kBlockSize == 0);

		// Step 1: set system address (or the ADMA2 descriptor table)
		const phys_addr_t address = vecs->base + vecOffset;
		if (fUseAdma) {
			_SetupAdma(address, toCopy);
			_SyncForDevice(address, toCopy);
		} else if (fUseDma) {
			fRegisters->system_address = address + fDmaBusOffset;
			_SyncForDevice(address, toCopy);
		}

		// Step 2: Set block size
		// For simplicity we use a transfer size equal to the sector size. We could
		// go up to 2K here if the length to read in each individual vec is a
		// multiple of 2K, but we have no easy way to know this (we would need to
		// iterate through the IOOperation vecs and check the size of each of them).
		// We could also do smaller transfers, but it is not possible to start a
		// transfer anywhere else than the start of a sector, so it's a lot simpler
		// to always work in complete sectors. We set the B_DMA_ALIGNMENT device
		// node property accordingly, making sure that we don't get asked to do
		// transfers that are not aligned with sectors.
		//
		// Additionnally, set SDMA buffer boundary aligment to 512K. This is the
		// largest possible size. We also set the B_DMA_BOUNDARY property on the
		// published device node, so that the DMA resource manager knows that it
		// must respect this boundary. As a result, we will never be asked to
		// do a transfer that crosses this boundary, and we don't need to handle
		// the DMA boundary interrupt (the transfer will be split in two at an
		// upper layer).
		// Step 3: set block count (with the block size)
		fRegisters->block_size.ConfigureTransfer(kBlockSize,
			BlockSize::kDmaBoundary512K, toCopy / kBlockSize);

		// Steps done in ExecuteCommand:
		// Steps 4, 5 and 6: set argument register, transfer_mode and command register
		// Step 7, 8, 9: wait for command complete interrupt, clear interrupt, read response
		ConditionVariableEntry waiter;
		fInterruptNotifier.Add(&waiter);

		uint32_t response;
		result = ExecuteCommand(command,
			offset / (offsetAsSectors ? kBlockSize : 1), &response);
		if (result != B_OK)
			break;

		if (!fUseDma) {
			result = _TransferPio(address, toCopy, isWrite);
			if (result != B_OK) {
				ERROR("PIO transfer failed: %s\n", strerror(result));
				fRegisters->software_reset.ResetDataLine();
				break;
			}
		}

		// Step 10: Wait for DMA transfer to complete
		// In theory we could go on and send other commands as long as they
		// don't need the DAT lines, but it's overcomplicating things.
		TRACE("Wait for transfer complete...");
		while ((fCommandResult & SDHCI_INT_TRANSFER_MASK) == 0) {
			status_t result = waiter.Wait(B_RELATIVE_TIMEOUT, 1000000);
			if (result == B_TIMED_OUT) {
				ERROR("No transfer complete interrupt for a while, status %x, "
					"present state %x, result %x\n", fRegisters->interrupt_status,
					fRegisters->present_state.Bits(), fCommandResult);
			} else if (result != B_OK)
				panic("sdhci: Failed to wait for end of DMA transfer: %s", strerror(result));
			fInterruptNotifier.Add(&waiter);
		}

		if (fCommandResult & SDHCI_INT_DATA_TIMEOUT) {
			TRACE_ALWAYS("Request timed out!\n");
			fRegisters->software_reset.ResetDataLine();
			return B_TIMED_OUT;
		}
		if ((fCommandResult & SDHCI_INT_DATA_ERROR_MASK) != 0) {
			ERROR("data transfer error %x (ADMA error status %x)\n",
				fCommandResult, fRegisters->adma_error_status);
			fRegisters->software_reset.ResetDataLine();
			return B_IO_ERROR;
		}

		if (fUseDma && !isWrite)
			_SyncForDevice(address, toCopy);
				// drop lines the CPU fetched while the device wrote memory

		TRACE("transfer complete OK.\n");

		length -= toCopy;
		vecOffset += toCopy;
		offset += toCopy;
	}

	return result;
}


void
SdhciBus::SetScanSemaphore(sem_id sem)
{
	fScanSemaphore = sem;

	// If there is already a card in, start a scan immediately
	if (fNonRemovable || fRegisters->present_state.IsCardInserted())
		release_sem(fScanSemaphore);

	// We can now enable the card insertion interrupt for next time a card
	// is inserted. Drop an insertion latched before (for the card that is
	// already in), or that card would be scanned twice.
	fRegisters->interrupt_status = SDHCI_INT_CARD_INS | SDHCI_INT_CARD_REM;
	EnableInterrupts(SDHCI_INT_CARD_INS);
}


void
SdhciBus::SetBusWidth(int width)
{
	uint8_t widthBits;
	switch(width) {
		case 1:
			widthBits = HostControl::kDataTransfer1Bit;
			break;
		case 4:
			widthBits = HostControl::kDataTransfer4Bit;
			break;
		case 8:
			widthBits = HostControl::kDataTransfer8Bit;
			break;
		default:
			panic("Incorrect bitwidth value");
			return;
	}
	fRegisters->host_control.SetDataTransferWidth(widthBits);
}


void
SdhciBus::SetCardType(card_type type)
{
	fCardType = type;
}


bool
SdhciBus::PowerOn()
{
	if (!fNonRemovable && !fRegisters->present_state.IsCardInserted()) {
		TRACE("Card not inserted, not powering on for now\n");
		return false;
	}

	uint8_t supportedVoltages = fRegisters->capabilities.SupportedVoltages();
	if (fAssume3v3)
		supportedVoltages |= Capabilities::k3v3;
	if ((supportedVoltages & Capabilities::k3v3) != 0)
		fRegisters->power_control.SetVoltage(PowerControl::k3v3);
	else if ((supportedVoltages & Capabilities::k3v0) != 0)
		fRegisters->power_control.SetVoltage(PowerControl::k3v0);
	else if ((supportedVoltages & Capabilities::k1v8) != 0)
		fRegisters->power_control.SetVoltage(PowerControl::k1v8);
	else {
		fRegisters->power_control.PowerOff();
		ERROR("No voltage is supported\n");
		return false;
	}

	return true;
}


void
SdhciBus::PowerOff()
{
	fRegisters->power_control.PowerOff();
}


/*!	Releases \a sem whenever the SDIO card raises its interrupt (a negative
	\a sem stops that).
*/
void
SdhciBus::SetCardInterrupt(sem_id sem)
{
	InterruptsSpinLocker locker(fCardInterruptLock);
	fCardInterruptSemaphore = sem;
	if (sem >= 0) {
		fRegisters->interrupt_status_enable |= SDHCI_INT_CARD_STATUS;
		fRegisters->interrupt_signal_enable |= SDHCI_INT_CARD_STATUS;
	} else {
		fRegisters->interrupt_signal_enable &= ~SDHCI_INT_CARD_STATUS;
		fRegisters->interrupt_status_enable &= ~SDHCI_INT_CARD_STATUS;
	}
}


/*!	Unmasks the card interrupt again, once its driver serviced the card. */
void
SdhciBus::AcknowledgeCardInterrupt()
{
	InterruptsSpinLocker locker(fCardInterruptLock);
	if (fCardInterruptSemaphore < 0)
		return;
	fRegisters->interrupt_status_enable |= SDHCI_INT_CARD_STATUS;
	fRegisters->interrupt_signal_enable |= SDHCI_INT_CARD_STATUS;
}


void
SdhciBus::TerminateBus()
{
	CALLED();

	DisableInterrupts();
	fRegisters->clock_control.DisableSD();
	PowerOff();
	/*
	// Debugging.
	uint8_t powerBits = fRegisters->power_control.Bits();
	uint16_t clockBits = fRegisters->clock_control.Bits();
	if ((powerBits & 0x1) != 0 || (clockBits & (1 << 2)) != 0) {
		ERROR("TerminateBus: Not killed. "
			"(power=%#x, clock=%#x)\n", powerBits, clockBits);
	} else {
		TRACE("TerminateBus: killed. (power=%#x, "
			"clock=%#x)\n", powerBits, clockBits);
	}
	*/
}


void
SdhciBus::RecoverError()
{
	fRegisters->interrupt_signal_enable &= ~(SDHCI_INT_CMD_CMP
		| SDHCI_INT_TRANS_CMP | SDHCI_INT_CARD_INS | SDHCI_INT_CARD_REM);

	if (fRegisters->interrupt_status & 7)
		fRegisters->software_reset.ResetCommandLine();

	uint32_t error_status = fRegisters->interrupt_status;
	fRegisters->interrupt_status = error_status;
}


int32
SdhciBus::HandleInterrupt()
{
#if 0
	// We could use the slot register to quickly see for which slot the
	// interrupt is. But since we have an interrupt handler call for each slot
	// anyway, it's just as simple to let each of them scan its own interrupt
	// status register.
	if ( !(fRegisters->slot_interrupt_status & (1 << fSlot)) ) {
		TRACE("interrupt not for me.\n");
		return B_UNHANDLED_INTERRUPT;
	}
#endif
	
	uint32_t intmask = fRegisters->interrupt_status;

	// Shortcut: exit early if there is no interrupt or if the register is
	// clearly invalid. Only signalled bits are ours to claim: the line may be
	// shared with another controller (the Raspberry Pi 4's two SDHCIs share
	// one), and the first handler that claims a level-triggered interrupt
	// ends its dispatch.
	if ((intmask == 0) || (intmask == 0xffffffff)
		|| (intmask & fRegisters->interrupt_signal_enable) == 0) {
		return B_UNHANDLED_INTERRUPT;
	}

	TRACE("interrupt function called %x\n", intmask);

	// handling card presence interrupts
	if ((intmask & SDHCI_INT_CARD_REM) != 0) {
		// We can get spurious interrupts as the card is inserted or removed,
		// so check the actual state before acting
		if (!fRegisters->present_state.IsCardInserted())
			fRegisters->power_control.PowerOff();
		else
			TRACE("Card removed interrupt, but card is inserted\n");

		fRegisters->interrupt_status = SDHCI_INT_CARD_REM;
		TRACE("Card removal interrupt handled\n");
	}

	if ((intmask & SDHCI_INT_CARD_INS) != 0) {
		// We can get spurious interrupts as the card is inserted or removed,
		// so check the actual state before acting
		if (fRegisters->present_state.IsCardInserted()) {
			if (PowerOn())
				SetClock(400, false);
			release_sem_etc(fScanSemaphore, 1, B_DO_NOT_RESCHEDULE);
		} else
			TRACE("Card insertion interrupt, but card is removed\n");

		fRegisters->interrupt_status = SDHCI_INT_CARD_INS;
		TRACE("Card presence interrupt handled\n");
	}

	// handling command interrupt
	if (intmask & SDHCI_INT_CMD_MASK) {
		fCommandResult |= intmask;
			// Save the status before clearing so the thread can handle it

		fRegisters->interrupt_status = intmask & SDHCI_INT_CMD_MASK;

		// Notify the thread
		fInterruptNotifier.NotifyAll();
		TRACE("Command complete interrupt handled\n");
	}

	if (intmask & SDHCI_INT_TRANSFER_MASK) {
		fCommandResult |= intmask;
		fRegisters->interrupt_status = intmask & SDHCI_INT_TRANSFER_MASK;
		fInterruptNotifier.NotifyAll();
		TRACE("Transfer complete interrupt handled\n");
	}

	// SDIO card interrupt: a level that only the function driver can clear
	// by servicing the card, so it stays masked until it acknowledges it
	if ((intmask & SDHCI_INT_CARD_STATUS) != 0) {
		SpinLocker locker(fCardInterruptLock);
		fRegisters->interrupt_signal_enable &= ~SDHCI_INT_CARD_STATUS;
		fRegisters->interrupt_status_enable &= ~SDHCI_INT_CARD_STATUS;
		if (fCardInterruptSemaphore >= 0) {
			release_sem_etc(fCardInterruptSemaphore, 1,
				B_DO_NOT_RESCHEDULE);
		}
	}

	// handling bus power interrupt
	if (intmask & SDHCI_INT_BUS_POWER) {
		fRegisters->interrupt_status = SDHCI_INT_BUS_POWER;
		TRACE("card is consuming too much power\n");
	}

	// Check that all interrupts have been cleared (we check all the ones we
	// enabled, so that should always be the case)
	// (the buffer ready bits are only used, and cleared, by PIO transfers)
	intmask = fRegisters->interrupt_status
		& ~(SDHCI_INT_BUF_READ_READY | SDHCI_INT_BUF_WRITE_READY);
	if (intmask != 0) {
		// (a command completing meanwhile shows up here, and is handled by
		// the interrupt it raises)
		TRACE("Remaining interrupts at end of handler: %x\n", intmask);
	}

	return B_HANDLED_INTERRUPT;
}


status_t
SdhciBus::_WorkerThread(void* cookie) {
	SdhciBus* bus = (SdhciBus*)cookie;
	while (bus->fStatus != B_SHUTTING_DOWN) {
		uint32_t intmask = bus->fRegisters->interrupt_status;
		if (intmask & SDHCI_INT_CMD_CMP) {
			bus->fCommandResult = intmask;
			bus->fRegisters->interrupt_status = intmask & SDHCI_INT_CMD_MASK;
			bus->fInterruptNotifier.NotifyAll();
		}
		if (intmask & SDHCI_INT_TRANS_CMP) {
			bus->fCommandResult = intmask;
			bus->fRegisters->interrupt_status = SDHCI_INT_TRANS_CMP;
			bus->fInterruptNotifier.NotifyAll();
		}
		snooze(100);
	}
	TRACE("poller thread terminating");
	return B_OK;
}


// #pragma mark -


void
uninit_bus(void* bus_cookie)
{
	SdhciBus* bus = (SdhciBus*)bus_cookie;
	delete bus;

	// FIXME do we need to put() the PCI module here?
}


void
bus_removed(void* bus_cookie)
{
	return;
}


static status_t
register_child_devices(void* cookie)
{
	CALLED();
	SdhciDevice* context = (SdhciDevice*)cookie;
	status_t status = B_OK;
	const char* bus;
	device_node* parent = gDeviceManager->get_parent_node(context->fNode);
	status = gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false);
	if (status != B_OK) {
		TRACE("Could not find required attribute device/bus\n");
		return status;
	}

	if (strcmp(bus, "pci") == 0)
		status = register_child_devices_pci(cookie);
	else if (strcmp(bus, "acpi") == 0)
		status = register_child_devices_acpi(cookie);
	else if (strcmp(bus, "fdt") == 0)
		status = register_child_devices_fdt(cookie);
	else
		status = B_BAD_VALUE;

	return status;
}


static status_t
init_device(device_node* node, void** device_cookie)
{
	CALLED();

	SdhciDevice* context = new(std::nothrow)SdhciDevice;
	if (context == NULL)
		return B_NO_MEMORY;
	context->fNode = node;
	*device_cookie = context;

	status_t status = B_OK;
	const char* bus;
	device_node* parent = gDeviceManager->get_parent_node(node);
	status = gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false);
	if (status != B_OK) {
		TRACE("Could not find required attribute device/bus\n");
		return status;
	}

	if (strcmp(bus, "pci") == 0)
		return init_device_pci(node, context);

	return B_OK;
}


static void
uninit_device(void* device_cookie)
{
	SdhciDevice* context = (SdhciDevice*)device_cookie;
	device_node* parent = gDeviceManager->get_parent_node(context->fNode);

	const char* bus;
	if (gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false) != B_OK) {
		TRACE("Could not find required attribute device/bus\n");
	}

	if (strcmp(bus, "pci") == 0)
		uninit_device_pci(context, parent);

	gDeviceManager->put_node(parent);

	delete context;
}


static status_t
register_device(device_node* parent)
{
	device_attr attrs[] = {
		{B_DEVICE_PRETTY_NAME, B_STRING_TYPE, {.string = "SD Host Controller"}},
		{}
	};

	return gDeviceManager->register_node(parent, SDHCI_DEVICE_MODULE_NAME,
		attrs, NULL, NULL);
}


static float
supports_device(device_node* parent)
{
	const char* bus;

	// make sure parent is an ACPI, FDT or PCI SDHCI device node
	if (gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
		!= B_OK) {
		TRACE("Could not find required attribute device/bus\n");
		return -1;
	}

	if (strcmp(bus, "pci") == 0)
		return supports_device_pci(parent);
	else if (strcmp(bus, "acpi") == 0)
		return supports_device_acpi(parent);
	else if (strcmp(bus, "fdt") == 0)
		return supports_device_fdt(parent);

	return 0.0f;
}


module_dependency module_dependencies[] = {
	{ MMC_BUS_MODULE_NAME, (module_info**)&gMMCBusController},
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&gDeviceManager },
	{}
};

status_t
set_clock(void* controller, uint32_t kilohertz)
{
	SdhciBus* bus = (SdhciBus*)controller;

	bus->SetClock(kilohertz, true);
	return B_OK;
}


status_t
execute_command(void* controller, uint8_t command, uint32_t argument,
	uint32_t* response)
{
	SdhciBus* bus = (SdhciBus*)controller;
	return bus->ExecuteCommand(command, argument, response);
}


status_t
do_io(void* controller, uint8_t command, IOOperation* operation,
	bool offsetAsSectors)
{
	SdhciBus* bus = (SdhciBus*)controller;
	return bus->DoIO(command, operation, offsetAsSectors);
}


void
set_scan_semaphore(void* controller, sem_id sem)
{
	SdhciBus* bus = (SdhciBus*)controller;
	return bus->SetScanSemaphore(sem);
}


void
set_bus_width(void* controller, int width)
{
	SdhciBus* bus = (SdhciBus*)controller;
	return bus->SetBusWidth(width);
}


void
set_card_type(void* controller, card_type type)
{
	SdhciBus* bus = (SdhciBus*)controller;
	bus->SetCardType(type);
}


void
terminate_bus(void* controller)
{
	SdhciBus* bus = (SdhciBus*)controller;
	bus->TerminateBus();
}


status_t
execute_data_command(void* controller, uint8_t command, uint32_t argument,
	void* buffer, size_t length)
{
	SdhciBus* bus = (SdhciBus*)controller;
	return bus->ExecuteDataCommand(command, argument, buffer, length);
}


void
set_card_interrupt(void* controller, sem_id sem)
{
	SdhciBus* bus = (SdhciBus*)controller;
	bus->SetCardInterrupt(sem);
}


void
acknowledge_card_interrupt(void* controller)
{
	SdhciBus* bus = (SdhciBus*)controller;
	bus->AcknowledgeCardInterrupt();
}


status_t
execute_io_command(void* controller, uint8_t command, uint32_t argument,
	void* buffer, size_t blockSize, uint32_t blockCount, bool isWrite,
	uint32_t* response)
{
	SdhciBus* bus = (SdhciBus*)controller;
	return bus->ExecuteIoCommand(command, argument, buffer, blockSize,
		blockCount, isWrite, response);
}


// Root device that binds to the ACPI, FDT or PCI bus. It will register an mmc_bus_interface
// node for each SD slot in the device.
static driver_module_info sSDHCIDevice = {
	{
		SDHCI_DEVICE_MODULE_NAME,
		0,
		NULL
	},
	supports_device,
	register_device,
	init_device,
	uninit_device,
	register_child_devices,
	NULL,	// rescan
	NULL,	// device removed
};


module_info* modules[] = {
	(module_info* )&sSDHCIDevice,
	(module_info* )&gSDHCIPCIDeviceModule,
	(module_info* )&gSDHCIACPIDeviceModule,
	(module_info* )&gSDHCIFDTDeviceModule,
	NULL
};
