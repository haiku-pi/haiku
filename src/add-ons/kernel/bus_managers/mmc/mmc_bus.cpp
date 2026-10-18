/*
 * Copyright 2018-2020 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *		B Krishnan Iyer, krishnaniyer97@gmail.com
 */
#include "mmc_bus.h"

#include <Errors.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


MMCBus::MMCBus(device_node* node)
	:
	fNode(node),
	fController(NULL),
	fCookie(NULL),
	fStatus(B_OK),
	fWorkerThread(-1),
	fActiveDevice(0),
	fCardType(CARD_TYPE_UNKNOWN),
	fSdioMultiBlock(false)
{
	CALLED();

	memset(fSdioBlockSize, 0, sizeof(fSdioBlockSize));

	// Get the parent info, it includes the API to send commands to the hardware
	device_node* parent = gDeviceManager->get_parent_node(node);
	fStatus = gDeviceManager->get_driver(parent,
		(driver_module_info**)&fController, &fCookie);
	gDeviceManager->put_node(parent);

	if (fStatus != B_OK) {
		ERROR("Not able to establish the bus %s\n",
			strerror(fStatus));
		return;
	}

	fScanSemaphore = create_sem(0, "MMC bus scan");
	fLockSemaphore = create_sem(1, "MMC bus lock");
	fWorkerThread = spawn_kernel_thread(_WorkerThread, "SD bus controller",
		B_NORMAL_PRIORITY, this);
	resume_thread(fWorkerThread);

	fController->set_scan_semaphore(fCookie, fScanSemaphore);
}


MMCBus::~MMCBus()
{
	CALLED();

	// Tell the worker thread we want to stop
	fStatus = B_SHUTTING_DOWN;

	// Delete the semaphores (this will unlock the worker thread if it was
	// waiting on them)
	delete_sem(fScanSemaphore);
	delete_sem(fLockSemaphore);

	// Wait for the worker thread to terminate
	status_t result;
	if (fWorkerThread != 0)
		wait_for_thread(fWorkerThread, &result);
}


status_t
MMCBus::InitCheck()
{
	return fStatus;
}


void
MMCBus::Rescan()
{
	// Just wake up the thread for a scan
	release_sem(fScanSemaphore);
}


status_t
MMCBus::ExecuteCommand(uint16_t rca, uint8_t command, uint32_t argument,
	uint32_t* response)
{
	status_t status = _ActivateDevice(rca);
	if (status != B_OK)
		return status;
	return fController->execute_command(fCookie, command, argument, response);
}


status_t
MMCBus::DoIO(uint16_t rca, uint8_t command, IOOperation* operation,
	bool offsetAsSectors)
{
	status_t status = _ActivateDevice(rca);
	if (status != B_OK)
		return status;
	return fController->do_io(fCookie, command, operation, offsetAsSectors);
}


status_t
MMCBus::ExecuteDataCommand(uint16_t rca, uint8_t command, uint32_t argument,
	void* buffer, size_t length)
{
	if (fController->execute_data_command == NULL)
		return B_NOT_SUPPORTED;
	status_t status = _ActivateDevice(rca);
	if (status != B_OK)
		return status;
	return fController->execute_data_command(fCookie, command, argument,
		buffer, length);
}


void
MMCBus::SetClock(int frequency)
{
	fController->set_clock(fCookie, frequency);
}


void
MMCBus::SetBusWidth(int width)
{
	fController->set_bus_width(fCookie, width);
}


void
MMCBus::SetCardType(card_type type)
{
	fCardType = type;
	fController->set_card_type(fCookie, type);
}


status_t
MMCBus::_ActivateDevice(uint16_t rca)
{
	// Do nothing if the device is already activated
	if (fActiveDevice == rca)
		return B_OK;

	uint32_t response;
	status_t result;
	result = fController->execute_command(fCookie, SELECT_DESELECT_CARD, ((uint32)rca) << 16,
		&response);

	if (result == B_OK)
		fActiveDevice = rca;

	return result;
}


// SDIO register addresses (SDIO simplified specification 3.00, 6.9 and 6.10)
enum {
	kCccrRevision			= 0x00,
	kCccrIoEnable			= 0x02,
	kCccrIoReady			= 0x03,
	kCccrBusInterface		= 0x07,
	kCccrCardCapability		= 0x08,
	kCccrCisPointer			= 0x09,
	kCccrBlockSize			= 0x10,
	kCccrBusSpeed			= 0x13,
	kFbrBlockSize			= 0x10,
		// in the FBR of function n, at n * 0x100

	kBusWidth4Bit			= 0x02,
	kBusWidthMask			= 0x03,
	kCapabilityMultiBlock	= 0x02,
	kCapabilityLowSpeed		= 0x40,
	kCapabilityLowSpeed4Bit	= 0x80,
	kBusSpeedSupportsHigh	= 0x01,
	kBusSpeedEnableHigh		= 0x02,
};


/*!	Executes CMD52 (IO_RW_DIRECT) and returns the data byte of its R5
	response.
*/
status_t
MMCBus::_ExecuteDirect(uint16_t rca, uint32_t argument, uint8_t* value)
{
	uint32_t response;
	status_t status = ExecuteCommand(rca, SDIO_RW_DIRECT, argument, &response);
	if (status != B_OK)
		return status;

	// R5: response flags in bits 15:8 (error bits 7, 6, 3, 1, 0 of them)
	if ((response & 0xcb00) != 0) {
		ERROR("CMD52 %#" B_PRIx32 ": response flags %#" B_PRIx32 "\n",
			argument, (response >> 8) & 0xff);
		return B_IO_ERROR;
	}
	if (value != NULL)
		*value = response & 0xff;
	return B_OK;
}


/*!	Reads one register of an SDIO function. */
status_t
MMCBus::SdioReadDirect(uint16_t rca, uint8_t function, uint32_t address,
	uint8_t* value)
{
	return _ExecuteDirect(rca,
		((uint32_t)function << 28) | ((address & 0x1ffff) << 9), value);
}


/*!	Writes one register of an SDIO function. */
status_t
MMCBus::SdioWriteDirect(uint16_t rca, uint8_t function, uint32_t address,
	uint8_t value)
{
	return _ExecuteDirect(rca, (1u << 31) | ((uint32_t)function << 28)
		| ((address & 0x1ffff) << 9) | value, NULL);
}


/*!	Moves \a length bytes between \a buffer and a function's address space
	with CMD53 (IO_RW_EXTENDED): as many whole blocks as possible in block
	mode, the rest in byte mode, so that the card never transfers more than
	asked for. With \a incrementAddress false, all bytes go to (or come from)
	the same address, a FIFO.
*/
status_t
MMCBus::SdioReadWriteExtended(uint16_t rca, uint8_t function,
	uint32_t address, bool incrementAddress, bool isWrite, void* buffer,
	size_t length)
{
	if (fController->execute_io_command == NULL)
		return B_NOT_SUPPORTED;
	if (function > 7)
		return B_BAD_VALUE;

	status_t status = _ActivateDevice(rca);
	if (status != B_OK)
		return status;

	const uint32_t blockSize = fSdioBlockSize[function];
	const uint32_t byteModeLimit = blockSize != 0 ? min_c(blockSize, 512) : 512;
	uint8_t* data = (uint8_t*)buffer;
	while (length > 0) {
		uint32_t argument = (isWrite ? 1u << 31 : 0)
			| ((uint32_t)function << 28) | (incrementAddress ? 1u << 26 : 0)
			| ((address & 0x1ffff) << 9);
		size_t chunk;
		uint32_t response;
		if (fSdioMultiBlock && blockSize != 0 && length >= blockSize) {
			// the count is 9 bits, and 0 would mean "until aborted"
			const uint32_t blocks = min_c(length / blockSize, 511);
			chunk = blocks * blockSize;
			status = fController->execute_io_command(fCookie, SDIO_RW_EXTENDED,
				argument | (1u << 27) | blocks, data, blockSize, blocks,
				isWrite, &response);
		} else {
			// in byte mode, a count of 0 means 512
			chunk = min_c(length, byteModeLimit);
			status = fController->execute_io_command(fCookie, SDIO_RW_EXTENDED,
				argument | (chunk & 0x1ff), data, chunk, 1, isWrite, &response);
		}
		if (status != B_OK)
			return status;
		if ((response & 0xcb00) != 0) {
			ERROR("CMD53 %#" B_PRIx32 ": response flags %#" B_PRIx32 "\n",
				argument, (response >> 8) & 0xff);
			return B_IO_ERROR;
		}

		data += chunk;
		length -= chunk;
		if (incrementAddress)
			address += chunk;
	}
	return B_OK;
}


/*!	Sets the block size of a function's block mode transfers. */
status_t
MMCBus::SdioSetBlockSize(uint16_t rca, uint8_t function, uint16_t blockSize)
{
	if (function > 7 || blockSize == 0 || blockSize > 2048)
		return B_BAD_VALUE;

	const uint32_t address = function == 0
		? kCccrBlockSize : function * 0x100 + kFbrBlockSize;
	status_t status = SdioWriteDirect(rca, 0, address, blockSize & 0xff);
	if (status == B_OK)
		status = SdioWriteDirect(rca, 0, address + 1, blockSize >> 8);
	if (status == B_OK)
		fSdioBlockSize[function] = blockSize;
	return status;
}


/*!	Enables an I/O function and waits (up to a second) until it is ready, or
	disables it.
*/
status_t
MMCBus::SdioEnableFunction(uint16_t rca, uint8_t function, bool enable)
{
	if (function == 0 || function > 7)
		return B_BAD_VALUE;

	uint8_t enabled;
	status_t status = SdioReadDirect(rca, 0, kCccrIoEnable, &enabled);
	if (status != B_OK)
		return status;
	if (enable)
		enabled |= 1 << function;
	else
		enabled &= ~(1 << function);
	status = SdioWriteDirect(rca, 0, kCccrIoEnable, enabled);
	if (status != B_OK || !enable)
		return status;

	for (int32 i = 0; i < 1000; i++) {
		uint8_t ready;
		status = SdioReadDirect(rca, 0, kCccrIoReady, &ready);
		if (status != B_OK)
			return status;
		if ((ready & (1 << function)) != 0)
			return B_OK;
		snooze(1000);
	}
	return B_TIMED_OUT;
}


/*!	Brings an SDIO card that answered CMD5 out of its initialization state,
	gives it an address, reads its identification from the CIS and publishes
	it (SDIO physical layer spec 3.1, sections 3 and 6).
*/
bool
MMCBus::_ScanSdioCard(uint32_t ocr)
{
	const uint32_t functionCount = (ocr >> 28) & 7;

	// Ask for the card's voltage window until it is ready
	uint32_t response = 0;
	status_t status;
	for (int32 i = 0; ; i++) {
		status = ExecuteCommand(0, SDIO_SEND_OP_COND, ocr & 0x00ffff00,
			&response);
		if (status == B_OK && (response & (1u << 31)) != 0)
			break;
		if (i == 100) {
			ERROR("SDIO card does not become ready: %s, OCR %#" B_PRIx32 "\n",
				strerror(status), response);
			return false;
		}
		snooze(10000);
	}

	status = ExecuteCommand(0, SD_SEND_RELATIVE_ADDR, 0, &response);
	if (status != B_OK) {
		ERROR("SDIO card gets no address: %s\n", strerror(status));
		return false;
	}
	const uint16_t rca = response >> 16;

	// CCCR revision, and the common CIS (CCCR 0x09-0x0b); CMD52 selects the
	// card (CMD7) first
	uint8_t cccrRevision, cisPointer[3];
	if (SdioReadDirect(rca, 0, 0x00, &cccrRevision) != B_OK
		|| SdioReadDirect(rca, 0, 0x09, &cisPointer[0]) != B_OK
		|| SdioReadDirect(rca, 0, 0x0a, &cisPointer[1]) != B_OK
		|| SdioReadDirect(rca, 0, 0x0b, &cisPointer[2]) != B_OK) {
		ERROR("SDIO card: cannot read the CCCR\n");
		return false;
	}

	// Walk the CIS for CISTPL_MANFID (0x20): manufacturer and card ID
	uint32_t cis = cisPointer[0] | (cisPointer[1] << 8) | (cisPointer[2] << 16);
	uint16_t vendor = 0, device = 0;
	for (int32 tuples = 0; tuples < 64; tuples++) {
		uint8_t code, link;
		if (SdioReadDirect(rca, 0, cis, &code) != B_OK || code == 0xff)
			break;
		if (SdioReadDirect(rca, 0, cis + 1, &link) != B_OK)
			break;
		if (code == 0x20 && link >= 4) {
			uint8_t id[4];
			for (int32 i = 0; i < 4; i++)
				SdioReadDirect(rca, 0, cis + 2 + i, &id[i]);
			vendor = id[0] | (id[1] << 8);
			device = id[2] | (id[3] << 8);
			break;
		}
		if (link == 0xff)
			break;
		cis += 2 + link;
	}

	// Full speed cards and low speed cards that say so do 4-bit transfers
	uint8_t capability, busInterface;
	if (SdioReadDirect(rca, 0, kCccrCardCapability, &capability) != B_OK
		|| SdioReadDirect(rca, 0, kCccrBusInterface, &busInterface) != B_OK) {
		ERROR("SDIO card: cannot read its capabilities\n");
		return false;
	}
	fSdioMultiBlock = (capability & kCapabilityMultiBlock) != 0;
	int busWidth = 1;
	if ((capability & kCapabilityLowSpeed) == 0
		|| (capability & kCapabilityLowSpeed4Bit) != 0) {
		busInterface = (busInterface & ~kBusWidthMask) | kBusWidth4Bit;
		if (SdioWriteDirect(rca, 0, kCccrBusInterface, busInterface) == B_OK) {
			SetBusWidth(4);
			busWidth = 4;
		}
	}
	// High speed timing (up to 50 MHz): the card outputs data on the rising
	// clock edge
	uint8_t busSpeed;
	bool highSpeed = false;
	if (SdioReadDirect(rca, 0, kCccrBusSpeed, &busSpeed) == B_OK
		&& (busSpeed & kBusSpeedSupportsHigh) != 0) {
		highSpeed = SdioWriteDirect(rca, 0, kCccrBusSpeed,
			busSpeed | kBusSpeedEnableHigh) == B_OK;
	}
	SetClock(highSpeed ? 50000 : 25000);

	const uint8_t cardType = CARD_TYPE_SDIO;
	SetCardType((card_type)cardType);

	TRACE_ALWAYS("SDIO card %04x:%04x, %" B_PRIu32 " functions, CCCR revision "
		"%#x, RCA %#x, %d-bit%s%s\n", vendor, device, functionCount,
		cccrRevision, rca, busWidth, highSpeed ? ", high speed" : "",
		fSdioMultiBlock ? ", multi-block" : "");

	if (vendor == 0x02d0)
		_ProbeBroadcomChip(rca);
	device_attr attrs[] = {
		{ B_DEVICE_BUS, B_STRING_TYPE, {.string = "mmc" }},
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE, {.string = "SDIO device" }},
		{ B_DEVICE_VENDOR_ID, B_UINT16_TYPE, {.ui16 = vendor}},
		{ B_DEVICE_ID, B_UINT16_TYPE, {.ui16 = device}},
		{ kSdioFunctionsAttribute, B_UINT8_TYPE, {.ui8 = (uint8)functionCount}},
		{ kMmcRcaAttribute, B_UINT16_TYPE, {.ui16 = rca}},
		{ kMmcTypeAttribute, B_UINT8_TYPE, {.ui8 = cardType}},
		{}
	};
	gDeviceManager->register_node(fNode, MMC_BUS_MODULE_NAME, attrs, NULL,
		NULL);
	return true;
}


/*!	Bring-up check of the SDIO data path (TODO: remove once the WiFi driver
	does this itself): read the chip ID of a Broadcom WLAN chip through its
	backplane window, with byte mode and block mode transfers.
*/
void
MMCBus::_ProbeBroadcomChip(uint16_t rca)
{
	enum {
		kChipClockCsr			= 0x1000e,
		kSleepCsr				= 0x1001f,
		kKeepSdioOn				= 0x01,
		kAlpAvailableRequest	= 0x08,
		kForceAlp				= 0x01,
		kForceHardwareClockOff	= 0x20,
		kAlpAvailable			= 0x40,
		kChipCommon				= 0x18000000,
		kWindowAccess32Bit		= 0x8000,
	};

	status_t status = SdioEnableFunction(rca, 1, true);
	if (status == B_OK)
		status = SdioSetBlockSize(rca, 1, 64);
	uint8_t sleep = 0;
	if (status == B_OK)
		status = SdioReadDirect(rca, 1, kSleepCsr, &sleep);
	if (status == B_OK)
		status = SdioWriteDirect(rca, 1, kSleepCsr, sleep | kKeepSdioOn);
	if (status == B_OK) {
		status = SdioWriteDirect(rca, 1, kChipClockCsr,
			kForceHardwareClockOff | kAlpAvailableRequest);
	}
	uint8_t clock = 0;
	for (int32 i = 0; status == B_OK && i < 100; i++) {
		status = SdioReadDirect(rca, 1, kChipClockCsr, &clock);
		if ((clock & kAlpAvailable) != 0)
			break;
		snooze(1000);
	}
	// with the hardware clock requests off, the backplane only gets the ALP
	// clock when it is forced
	if (status == B_OK) {
		status = SdioWriteDirect(rca, 1, kChipClockCsr,
			kForceHardwareClockOff | kForceAlp);
		snooze(65);
	}
	if (status == B_OK)
		status = _SetBroadcomWindow(rca, kChipCommon);

	uint32_t id = 0;
	if (status == B_OK) {
		status = SdioReadWriteExtended(rca, 1,
			(kChipCommon & 0x7fff) | kWindowAccess32Bit, true, false, &id, 4);
	}

	// Write a pattern to the chip's RAM and read it back, in block mode
	static const uint32_t kRam = 0x198000;
	static const size_t kWords = 1024;
	uint32_t* pattern = (uint32_t*)malloc(2 * kWords * sizeof(uint32_t));
	if (pattern == NULL)
		return;
	uint32_t* readBack = pattern + kWords;
	for (size_t i = 0; i < kWords; i++)
		pattern[i] = (uint32_t)i * 0x9e3779b9 ^ 0x5a5aa5a5;
	memset(readBack, 0, kWords * sizeof(uint32_t));

	status_t written = B_NO_INIT, read = B_NO_INIT;
	if (status == B_OK)
		status = _SetBroadcomWindow(rca, kRam);
	if (status == B_OK) {
		written = SdioReadWriteExtended(rca, 1,
			(kRam & 0x7fff) | kWindowAccess32Bit, true, true, pattern,
			kWords * sizeof(uint32_t));
		read = SdioReadWriteExtended(rca, 1,
			(kRam & 0x7fff) | kWindowAccess32Bit, true, false, readBack,
			kWords * sizeof(uint32_t));
	}
	size_t firstDifference = kWords;
	for (size_t i = 0; i < kWords; i++) {
		if (pattern[i] != readBack[i]) {
			firstDifference = i;
			break;
		}
	}
	TRACE_ALWAYS("SDIO Broadcom probe: %s, chip %#" B_PRIx32 " (BCM%04" B_PRIx32
		" rev %" B_PRIu32 "), RAM write %s, read %s, %s (word %d)\n",
		strerror(status), id, id & 0xffff, (id >> 16) & 0xf, strerror(written),
		strerror(read), firstDifference == kWords ? "matches" : "DIFFERS",
		(int)firstDifference);
	free(pattern);
}


/*!	Points the 32 KB backplane window of a Broadcom chip's function 1 at
	\a address (TODO: belongs to the WiFi driver).
*/
status_t
MMCBus::_SetBroadcomWindow(uint16_t rca, uint32_t address)
{
	static const uint32_t kBackplaneWindow = 0x1000a;
	status_t status = SdioWriteDirect(rca, 1, kBackplaneWindow,
		(address >> 8) & 0x80);
	if (status == B_OK)
		status = SdioWriteDirect(rca, 1, kBackplaneWindow + 1, address >> 16);
	if (status == B_OK)
		status = SdioWriteDirect(rca, 1, kBackplaneWindow + 2, address >> 24);
	return status;
}


/*!	Releases \a sem when the SDIO card raises its interrupt; the interrupt
	stays masked until SdioAcknowledgeInterrupt().
*/
status_t
MMCBus::SdioSetInterrupt(sem_id sem)
{
	if (fController->set_card_interrupt == NULL)
		return B_NOT_SUPPORTED;
	fController->set_card_interrupt(fCookie, sem);
	return B_OK;
}


void
MMCBus::SdioAcknowledgeInterrupt()
{
	if (fController->acknowledge_card_interrupt != NULL)
		fController->acknowledge_card_interrupt(fCookie);
}


void MMCBus::_AcquireScanSemaphore()
{
	ReleaseBus();
	acquire_sem(fScanSemaphore);
	AcquireBus();
}


void
MMCBus::_TerminateBus()
{
	fController->terminate_bus(fCookie);
}


status_t
MMCBus::_WorkerThread(void* cookie)
{
	MMCBus* bus = (MMCBus*)cookie;
	uint32_t response;

	bus->AcquireBus();

	// We assume the bus defaults to 400kHz clock and has already powered on
	// cards.

	// Reset all cards on the bus
	// This does not work if the bus has not been powered on yet (the command
	// will timeout), in that case we wait until asked to scan again when a
	// card has been inserted and powered on.
	status_t result;
	do {
		bus->_AcquireScanSemaphore();

		// Check if we need to exit early (possible if the parent device did
		// not manage initialize itself correctly)
		if (bus->fStatus == B_SHUTTING_DOWN) {
			bus->ReleaseBus();
			return B_OK;
		}

		TRACE("Reset the bus...\n");
		result = bus->ExecuteCommand(0, GO_IDLE_STATE, 0, NULL);
		TRACE("CMD0 result: %s\n", strerror(result));
	} while (result != B_OK);

	// Need to wait at least 8 clock cycles after CMD0 before sending the next
	// command. With the default 400kHz clock that would be 20 microseconds,
	// but we need to wait at least 20ms here, otherwise the next command times
	// out
	snooze(30000);

	while (bus->fStatus != B_SHUTTING_DOWN) {
		TRACE("Scanning the bus\n");

		// Use the low speed clock and 1bit bus width for scanning
		bus->SetClock(400);
		bus->SetBusWidth(1);

		// SDIO cards answer CMD5, memory cards ignore it (as Linux, probe
		// it first)
		uint32_t ioOcr;
		if (bus->ExecuteCommand(0, SDIO_SEND_OP_COND, 0, &ioOcr) == B_OK) {
			bus->_ScanSdioCard(ioOcr);
			bus->_AcquireScanSemaphore();
			continue;
		}

		// A memory card flags the CMD5 it did not know in its next status;
		// reset it again so that its first R1 response is clean
		bus->ExecuteCommand(0, GO_IDLE_STATE, 0, NULL);
		snooze(30000);

		// Probe the voltage range
		enum {
			// Table 4-40 in physical layer specification v8.00
			// All other values are currently reserved
			HOST_27_36V = 1, // Host supplied voltage 2.7-3.6V
		};

		// An arbitrary value, we just need to check that the response
		// containts the same.
		static const uint8 kVoltageCheckPattern = 0xAA;
		uint8_t cardType = CARD_TYPE_SD;
		// FIXME MMC cards will not reply to this! They expect CMD1 instead
		// SD v1 cards will also not reply, but we can proceed to ACMD41
		// If ACMD41 also does not work, it may be an SDIO card, too
		uint32_t probe = (HOST_27_36V << 8) | kVoltageCheckPattern;
		uint32_t hcs = 1 << 30;
		uint32_t ocr;
		status_t status = bus->ExecuteCommand(0, SD_SEND_IF_COND, probe, &response);
		if (status != B_OK) {
			TRACE("Card does not implement CMD8, may be a V1 SD or MMC card\n");
			// Do not check for SDHC support in this case
			hcs = 0;

			TRACE("Trying MMC CMD1 initialization...\n");
			do {
				status = bus->ExecuteCommand(0, MMC_SEND_OP_COND, 0xFF8000, &ocr);
				// full voltage window, byte addressable, should look into this.
				if (status != B_OK) {
					TRACE("MMC CMD1 failed\n");
					break;
				}
				if ((ocr & (1 << 31)) == 0) {
					TRACE("MMC card is busy\n");
					snooze(100000);
				}
			} while ((ocr & (1 << 31)) == 0);

			if (status == B_OK && (ocr & (1 << 31)) != 0) {
				TRACE("Detected MMC card after CMD1\n");
				if ((ocr & (1 << 30)) != 0)
					cardType = CARD_TYPE_MMC_EXTENDED_CAPACITY;
				else
					cardType = CARD_TYPE_MMC;
			}
		} else if (response != probe) {
			ERROR("Card does not support voltage range (expected %x, "
				"reply %x)\n", probe, response);
			bus->_TerminateBus();
			bus->ReleaseBus();
			return B_ERROR;
		}

		// Probe OCR, waiting for card to become ready
		// We keep repeating ACMD41 until the card replies that it is
		// initialized. For MMC we already probed using CMD1 above.
		if ((cardType != CARD_TYPE_MMC) && (cardType != CARD_TYPE_MMC_EXTENDED_CAPACITY)) {
			do {
				uint32_t cardStatus;
				while (bus->ExecuteCommand(0, SD_APP_CMD, 0, &cardStatus) == B_BUSY) {
					ERROR("Card locked after CMD8...\n");
					snooze(1000000);
				}
				if ((cardStatus & 0xFFFF8000) != 0)
					ERROR("SD card reports error %x\n", cardStatus);
				if ((cardStatus & (1 << 5)) == 0)
					ERROR("Card did not enter ACMD mode\n");

				bus->ExecuteCommand(0, SD_SEND_OP_COND, hcs | 0xFF8000, &ocr);

				if ((ocr & (1 << 31)) == 0) {
					TRACE("Card is busy\n");
					snooze(100000);
				}
			} while ((ocr & (1 << 31)) == 0);
		}

		// FIXME this should be asked to each card, when there are multiple
		// ones. So ACMD41 should be moved inside the probing loop below?
		if (cardType == CARD_TYPE_SD) {
			if ((ocr & hcs) != 0)
				cardType = CARD_TYPE_SDHC;
			if ((ocr & (1 << 29)) != 0)
				cardType = CARD_TYPE_UHS2;
			if ((ocr & (1 << 24)) != 0)
				TRACE("Card supports 1.8v");
		}
		TRACE("Voltage range: %x\n", ocr & 0xFFFFFF);

		// Set the card type so the next commands have the correct reply types
		// (MMC and SD commands with the same identifier sometime expect different responses)
		bus->SetCardType((card_type)cardType);

		// TODO send CMD11 to switch to low voltage mode if card supports it?

		// We use CMD2 (ALL_SEND_CID) and CMD3 (SEND_RELATIVE_ADDR) to assign
		// an RCA to all cards. Initially all cards have an RCA of 0 and will
		// all receive CMD2. But only one of them will reply (they do collision
		// detection while sending the CID in reply). We assign a new RCA to
		// that first card, and repeat the process with the remaining ones
		// until no one answers to CMD2. Then we know all cards have an RCA
		// (and a matching published device on our side).
		uint32_t cid[4];
		uint32_t vendor;
		char name[7];
		uint32_t serial;
		uint16_t revision;
		uint8_t month;
		uint16_t year;
		uint16_t rca;
		bool cardFound = false;
		// This being an if statement as opposed to a while statement restricts
		// it to one device per bus.
		if ((cardType == CARD_TYPE_MMC) || (cardType == CARD_TYPE_MMC_EXTENDED_CAPACITY)) {
			if (bus->ExecuteCommand(0, ALL_SEND_CID, 0, cid) == B_OK) {
				// We currently support only a single card, so use a fixed RCA.
				rca = 1;
				status
					= bus->ExecuteCommand(0, MMC_SET_RELATIVE_ADDR, ((uint32)rca) << 16, &response);
				TRACE("MMC RCA: %x Status: %x\n", rca, response & 0xFFFF);
				if (status != B_OK) {
					TRACE("Failed to set RCA for MMC card\n");
				} else {
					MMCCid mmcCid(cid);
					vendor = mmcCid.VendorID();
					mmcCid.ProductName(name);
					serial = mmcCid.ProductSerial();
					revision = mmcCid.ProductRevision();
					month = mmcCid.ManufactureMonth();
					year = mmcCid.ManufactureYear(true);
					TRACE("MMC CID: MID=%" B_PRIu32 ", name=\"%s\", PSN=%" B_PRIu32
						  ", PRV=%u, MDT=%u/%u\n",
						vendor, name, serial, revision, month, year);
					cardFound = true;
				}
			}
		} else if (bus->ExecuteCommand(0, ALL_SEND_CID, 0, cid) == B_OK) {
			bus->ExecuteCommand(0, SD_SEND_RELATIVE_ADDR, 0, &response);

			TRACE("RCA: %x Status: %x\n", response >> 16, response & 0xFFFF);

			if ((response & 0xFF00) != 0x500) {
				TRACE("Card did not enter data state\n");
				// This probably means there are no more cards to scan on the
				// bus, so exit the loop.
				break;
			}

			// The card now has an RCA and it entered the data phase, which
			// means our initializing job is over, we can pass it on to the
			// mmc_disk driver.
			rca = response >> 16;
			SDCid sdCid(cid);
			vendor = sdCid.VendorID();
			sdCid.ProductName(name);
			serial = sdCid.ProductSerial();
			revision = sdCid.ProductRevision();
			month = sdCid.ManufactureMonth();
			year = sdCid.ManufactureYear();
			cardFound = true;
		}

		if (cardFound) {
			device_attr attrs[] = {
				{ B_DEVICE_BUS, B_STRING_TYPE, {.string = "mmc" }},
				{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE, {.string = "mmc device" }},
				{ B_DEVICE_VENDOR_ID, B_UINT32_TYPE, {.ui32 = vendor}},
				{ B_DEVICE_ID, B_STRING_TYPE, {.string = name}},
				{ B_DEVICE_UNIQUE_ID, B_UINT32_TYPE, {.ui32 = serial}},
				{ "mmc/revision", B_UINT16_TYPE, {.ui16 = revision}},
				{ "mmc/month", B_UINT8_TYPE, {.ui8 = month}},
				{ "mmc/year", B_UINT16_TYPE, {.ui16 = year}},
				{ kMmcRcaAttribute, B_UINT16_TYPE, {.ui16 = rca}},
				{ kMmcTypeAttribute, B_UINT8_TYPE, {.ui8 = cardType}},
				{}
			};

			// publish child device for the card
			gDeviceManager->register_node(bus->fNode, MMC_BUS_MODULE_NAME,
				attrs, NULL, NULL);
		}

		// TODO if there is a single card active, check if it supports CMD6
		// (spec version 1.10 or later in SCR). If it does, check if CMD6 can
		// enable high speed mode, use that to go to 50MHz instead of 25.
		bus->SetClock(25000);

		// FIXME we also need to unpublish devices that are gone. Probably need
		// to "ping" all RCAs somehow? Or is there an interrupt we can look for
		// to detect added/removed cards?

		// Wait for the next scan request
		// The thread will spend most of its time waiting here
		bus->_AcquireScanSemaphore();
	}

	bus->ReleaseBus();

	TRACE("poller thread terminating");
	return B_OK;
}
