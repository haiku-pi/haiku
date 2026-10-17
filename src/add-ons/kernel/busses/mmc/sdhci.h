/*
 * Copyright 2018-2025 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *		B Krishnan Iyer, krishnaniyer97@gmail.com
 *		Adrien Destugues, pulkomandy@pulkomandy.tk
 *		Ron Ben Aroya, sed4906birdie@gmail.com
 */
#ifndef _SDHCI_H
#define _SDHCI_H


#include <condition_variable.h>
#include <device_manager.h>

#include <KernelExport.h>

#include "mmc.h"


class SdhciBus {
	public:
								SdhciBus(struct registers* registers, uint8_t irq,
									bool poll, uint32 baseClockKHz = 0);
								~SdhciBus();

			void				EnableInterrupts(uint32_t mask);
			void				DisableInterrupts();
			status_t			ExecuteCommand(uint8_t command, uint32_t argument,
									uint32_t* response);
			int32				HandleInterrupt();
			status_t			InitCheck();
			void				Reset();
			void				SetClock(int kilohertz, bool allowAuto);
			status_t			DoIO(uint8_t command, IOOperation* operation,
									bool offsetAsSectors);
			void				SetScanSemaphore(sem_id sem);
			void				SetBusWidth(int width);
			void				SetCardType(card_type type);
			void				SetDmaWindow(phys_addr_t busOffset, bool coherent);
			void				TerminateBus();

	private:
			bool				PowerOn();
			void				PowerOff();
			void				RecoverError();
			void				_SyncForDevice(phys_addr_t address, size_t length);
	static	status_t			_WorkerThread(void*);

	private:
			struct registers*	fRegisters;
			uint32				fCommandResult;
			uint8				fIrq;
			ConditionVariable	fInterruptNotifier;
			sem_id				fScanSemaphore;
			status_t			fStatus;
			thread_id			fWorkerThread;
			card_type			fCardType;
			uint32				fBaseClockKHz;
				// 0: from the capabilities register
			phys_addr_t			fDmaBusOffset;
			bool				fDmaCoherent;
};


class SdhciDevice {
	public:
		device_node*	fNode;
		uint8_t			fRicohOriginalMode;
};

/*!	A register narrower than 32 bits, accessed through the aligned 32-bit word
	that contains it. Some controllers (the Raspberry Pi 4's "brcm,bcm2711-emmc2")
	ignore 8 and 16-bit accesses, and every SDHCI register may be accessed as
	part of a 32-bit word, so this is used for all of them. The object lives at
	the register's address in the mapped register block; its storage only
	reserves the register's bytes in struct registers.
	Read-modify-write is harmless for all registers that share a word with
	narrower ones, except the transfer mode, which shares one with the command
	register (see Command).
*/
template<typename T>
class NarrowRegister {
	public:
		operator T() const
		{
			return (T)(*_Word() >> _Shift());
		}

		NarrowRegister& operator=(T value)
		{
			const uint32_t mask = (uint32_t)(T)~0 << _Shift();
			*_Word() = (*_Word() & ~mask) | ((uint32_t)value << _Shift());
			return *this;
		}

		NarrowRegister& operator|=(T value) { return *this = (T)(*this | value); }
		NarrowRegister& operator&=(T value) { return *this = (T)(*this & value); }

	private:
		volatile uint32_t* _Word() const
			{ return (volatile uint32_t*)((addr_t)this & ~(addr_t)3); }
		int _Shift() const
			{ return ((addr_t)this & 3) * 8; }

		T fStorage;
} __attribute__((packed));


// Bits of the transfer mode register, which is written together with the
// command register (Command::SendCommand()).
class TransferMode {
	public:
		// TODO response interrupt
		// TODO response check

		static const uint8_t kR1 = 0 << 6;
		static const uint8_t kR5 = 1 << 6;

		static const uint8_t kMulti = 1 << 5;
		static const uint8_t kSingle = 0 << 5;

		static const uint8_t kRead = 1 << 4;
		static const uint8_t kWrite = 0 << 4;

		static const uint8_t kAutoCmdDisabled = 0 << 2;
		static const uint8_t kAutoCmd12Enable = 1 << 2;
		static const uint8_t kAutoCmd23Enable = 2 << 2;
		static const uint8_t kAutoCmdAutoSelect
			= kAutoCmd23Enable | kAutoCmd12Enable;

		static const uint8_t kBlockCountEnable = 1 << 1;

		static const uint8_t kDmaEnable = 1;
		static const uint8_t kNoDmaOrNoData = 0;
} __attribute__((packed));


// The transfer mode and command registers. They are written with a single
// 32-bit store: writing the command register issues the command, so writing
// the transfer mode on its own (or read-modify-writing it as part of the word)
// would issue a command as well.
class Command {
	public:
		void SendCommand(uint8_t command, uint8_t type, uint16_t transferMode)
		{
			fBits = ((uint32_t)((command << 8) | type) << 16) | transferMode;
		}

		static const uint8_t kDataPresent = 0x20;
		static const uint8_t kCheckIndex = 0x10;
		static const uint8_t kCRCEnable = 0x8;
		static const uint8_t kSubCommand = 0x4;
		static const uint8_t kReplySizeMask = 0x3;
		static const uint8_t k32BitResponse = 0x2;
		static const uint8_t k128BitResponse = 0x1;
		static const uint8_t k32BitResponseCheckBusy = 0x3;

		// For simplicity pre-define the standard response types from the SD
		// card specification
		static const uint8_t kNoReplyType = 0;
		static const uint8_t kR1Type = kCheckIndex | kCRCEnable
			| k32BitResponse;
		static const uint8_t kR1bType = (kCheckIndex | kCRCEnable 
			| k32BitResponseCheckBusy) & (~ kDataPresent);
 		static const uint8_t kR2Type = kCRCEnable | k128BitResponse;
		static const uint8_t kR3Type = k32BitResponse;
		static const uint8_t kR6Type = kCheckIndex | k32BitResponse;
		static const uint8_t kR7Type = kCheckIndex | kCRCEnable
			| k32BitResponse;

	private:
		volatile uint32_t fBits;
} __attribute__((packed));


class PresentState {
	public:
		uint32_t Bits() { return fBits; }

		bool IsCardInserted() { return fBits & (1 << 16); }
		bool CommandInhibit() { return fBits & (1 << 0); }
		bool DataInhibit() { return fBits & (1 << 1); }

	private:
		volatile uint32_t fBits;
} __attribute__((packed));


class PowerControl {
	public:
		uint8_t Bits() { return fBits; }

		void SetVoltage(int voltage) {
			fBits |= voltage | kBusPowerOn;
		}
		void PowerOff() { fBits &= ~kBusPowerOn; }

		static const uint8_t k3v3 = 7 << 1;
		static const uint8_t k3v0 = 6 << 1;
		static const uint8_t k1v8 = 5 << 1;
	private:
		NarrowRegister<uint8_t> fBits;

		static const uint8_t kBusPowerOn = 1;
} __attribute__((packed));


class ClockControl
{
	public:
		uint16_t Bits() { return fBits; }

		uint16_t SetDivider(uint16_t divider) {
			// The register holds half the divider; round up, so that the
			// card clock never exceeds the requested frequency.
			if (divider <= 1)
				divider = 0;
			else
				divider = (divider + 1) / 2;
			uint16_t bits = fBits & ~0xffc0;
			bits |= divider << 8;
			bits |= (divider >> 8) & 0xc0;
			fBits = bits;

			return divider == 0 ? 1 : divider * 2;
		}

		void EnableInternal() { fBits |= 1 << 0; }
		bool InternalStable() { return fBits & (1 << 1); }
		void EnableSD() { fBits |= 1 << 2; }
		void DisableSD() { fBits &= ~(1 << 2); }
		void EnablePLL() { fBits |= 1 << 3; }
	private:
		NarrowRegister<uint16_t> fBits;
} __attribute__((packed));


class TimeoutControl {
	public:
		void SetDivider(uint32_t base_khz, uint32_t delay_ms) {
			// We need a value i so that:
			//
			// base (Hz)        1
			// --------- < -----------
			//  2 ^ i       delay (s)
			//
			// This will divide the timeout clock to a period longer than the delay needed by the
			// SD card. Dividing each side of the equation by 1000 allows to use our input units
			// directly (kHz on one side and ms on the other), and be less worried about overflows.
			// In the code the equation is rearranged to not have to deal with divisions and lose
			// precision.
			//
			// The allowed range for i is 13 to 27, which is then converted to a value between 0
			// and 14 to write into the register.
			//
			// If we can't reach a sufficiently large delay, the loop will default to the maximum
			// allowed value of 2^27.
			uint32_t i;
			for (i = 13; i < 27; i++) {
				if ((base_khz * delay_ms) <= (1u << i))
					break;
			}

			fBits = i - 13;
		}
	private:
		NarrowRegister<uint8_t> fBits;
} __attribute__((packed));


class SoftwareReset {
	public:
		uint8_t Bits() { return fBits; }

		bool ResetAll() {
			fBits = 1;
			int i = 0;
			// wait up to 100ms
			while ((fBits & 1) != 0 && i++ < 10)
				snooze(10000);
			return i < 10;
		}

		void ResetCommandAndDataLines() {
			fBits |= 6;
			while(fBits & 6);
		}

		void ResetCommandLine() {
			fBits |= 2;
			while(fBits & 2);
		}

		void ResetDataLine() {
			fBits |= 4;
			while(fBits & 4);
		}

	private:
		NarrowRegister<uint8_t> fBits;
} __attribute__((packed));


// #pragma mark Interrupt registers
// The interrupt status register is "write 1 to clear": acknowledge interrupts
// by writing exactly their bits, never with a read-modify-write (|=), which
// would also clear the ones that arrived meanwhile.
#define SDHCI_INT_CMD_CMP			0x00000001	// command complete enable
#define SDHCI_INT_TRANS_CMP			0x00000002	// transfer complete enable
#define SDHCI_INT_BLOCK_GAP			0x00000004
#define SDHCI_INT_DMA				0x00000008
#define SDHCI_INT_BUF_WRITE_READY	0x00000010
#define SDHCI_INT_BUF_READ_READY	0x00000020	// buffer read ready enable
#define SDHCI_INT_CARD_INS			0x00000040	// card insertion enable
#define SDHCI_INT_CARD_REM			0x00000080	// card removal enable
#define SDHCI_INT_CARD_STATUS		0x00000100
#define SDHCI_INT_A					0x00000200
#define SDHCI_INT_B					0x00000400
#define SDHCI_INT_C					0x00000800
#define SDHCI_INT_RETUNING			0x00001000
#define SDHCI_INT_ERROR				0x00008000	// error
#define SDHCI_INT_COMMAND_TIMEOUT	0x00010000	// Timeout error
#define SDHCI_INT_COMMAND_CRC		0x00020000	// CRC error
#define SDHCI_INT_COMMAND_END_BIT	0x00040000	// end bit error
#define SDHCI_INT_COMMAND_INDEX 	0x00080000	// index error
#define SDHCI_INT_DATA_TIMEOUT	 	0x00100000
#define SDHCI_INT_DATA_CRC		 	0x00200000
#define SDHCI_INT_DATA_END		 	0x00400000
#define SDHCI_INT_BUS_POWER			0x00800000	// power fail
#define SDHCI_INT_AUTO_CMD_ERROR	0x01000000
#define SDHCI_INT_ADMA_ERROR		0x02000000
#define SDHCI_INT_TUNING_ERROR		0x04000000
#define SDHCI_INT_VENDOR_ERRORS		0xF0000000

#define	 SDHCI_INT_CMD_ERROR_MASK	(SDHCI_INT_COMMAND_TIMEOUT \
		| SDHCI_INT_COMMAND_CRC | SDHCI_INT_COMMAND_END_BIT | SDHCI_INT_COMMAND_INDEX)

#define SDHCI_INT_CMD_MASK 			(SDHCI_INT_CMD_CMP | SDHCI_INT_CMD_ERROR_MASK)
#define SDHCI_INT_TRANSFER_MASK 	(SDHCI_INT_TRANS_CMP | SDHCI_INT_DATA_TIMEOUT)

#define SDHCI_INT_ERROR_MASK		(SDHCI_INT_VENDOR_ERRORS | SDHCI_INT_TUNING_ERROR \
	| SDHCI_INT_ADMA_ERROR | SDHCI_INT_AUTO_CMD_ERROR | SDHCI_INT_BUS_POWER | SDHCI_INT_DATA_END \
	| SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_CMD_ERROR_MASK)
#define SDHCI_INT_NORMAL_MASK 		(SDHCI_INT_CMD_CMP | SDHCI_INT_TRANS_CMP \
	| SDHCI_INT_BLOCK_GAP | SDHCI_INT_DMA | SDHCI_INT_BUF_WRITE_READY | SDHCI_INT_BUF_READ_READY \
	| SDHCI_INT_CARD_INS | SDHCI_INT_CARD_REM | SDHCI_INT_CARD_STATUS | SDHCI_INT_A | SDHCI_INT_B \
	| SDHCI_INT_C | SDHCI_INT_RETUNING)


// #pragma mark -
class Capabilities
{
	public:
		uint8_t ClockMultiplier() { return (Bits() >> 48) & 0xFF; }
		uint8_t RetuningModes() { return (Bits() >> 46) & 3; }
		bool UseTuningForSDR50() { return (Bits() >> 45) & 1; }
		uint8_t RetuningTimerCount() { return (Bits() >> 40) & 7; }
		bool TypeDSupport() { return (Bits() >> 38) & 1; }
		bool TypeCSupport() { return (Bits() >> 37) & 1; }
		bool TypeASupport() { return (Bits() >> 36) & 1; }
		bool DDR50Support() { return (Bits() >> 34) & 1; }
		bool SDR104Support() { return (Bits() >> 33) & 1; }
		bool SDR50Support() { return (Bits() >> 32) & 1; }
		uint8_t SlotType() { return (Bits() >> 30) & 3; }
		bool AsynchronousInterrupts() { return (Bits() >> 29) & 1; }
		bool SystemBus64Bits() { return (Bits() >> 28) & 1; }
		uint8_t SupportedVoltages() { return (Bits() >> 24) & 7; }
		bool SuspendResume() { return (Bits() >> 23) & 1; }
		bool SimpleDMA() { return (Bits() >> 22) & 1; }
		bool HighSpeed() { return (Bits() >> 21) & 1; }
		bool AdvancedDMA() { return (Bits() >> 19) & 1; }
		bool Embedded8Bit() { return (Bits() >> 18) & 1; }
		uint8_t MaxBlockLength() { return (Bits() >> 16) & 3; }
		uint8_t BaseClockFrequency() { return (Bits() >> 8) & 0xFF; }
		uint32_t TimeoutClockFrequency()
		{
			bool megahertz = (Bits() >> 7) & 1;
			uint32_t frequency = (Bits() >> 0) & 0x3F;
			if (megahertz)
				frequency *= 1000;
			return frequency;
		}

		static const uint8_t k3v3 = 1;
		static const uint8_t k3v0 = 2;
		static const uint8_t k1v8 = 4;

	private:
		// Two 32-bit registers, read separately
		uint64_t Bits() { return fLow | ((uint64_t)fHigh << 32); }

		volatile uint32_t fLow;
		volatile uint32_t fHigh;
} __attribute__((packed));


class HostControllerVersion {
	public:
		uint8_t SpecVersion() { return fBits & 0xff; }
		uint8_t VendorVersion() { return fBits >> 8; }

	private:
		NarrowRegister<uint16_t> fBits;
} __attribute__((packed));


class HostControl {
	public:
		void SetDMAMode(uint8_t dmaMode)
		{
			value = (value & ~kDmaMask) | dmaMode;
		}

		void SetDataTransferWidth(uint8_t width)
		{
			value = (value & ~kDataTransferWidthMask) | width;
		}

		// TODO add other bits for LED control, etc.

		uint8 Bits() { return value; }

		static const uint8_t kDmaMask = 3 << 3;
		static const uint8_t kSdma = 0 << 3;
		static const uint8_t kAdma32 = 2 << 3;
		static const uint8_t kAdma64 = 3 << 3;

		// It's convenient to think of this as a single "bit width" setting,
		// but the bits for 4-bit and 8-bit modes were introduced at different
		// times and are not next to each other in the register.
		static const uint8_t kDataTransfer1Bit = 0;
		static const uint8_t kDataTransfer4Bit = 1 << 1;
		static const uint8_t kDataTransfer8Bit = 1 << 5;

		static const uint8_t kDataTransferWidthMask
			= kDataTransfer4Bit | kDataTransfer8Bit;
	private:
		NarrowRegister<uint8_t> value;
} __attribute__((packed));


class BlockSize {
	public:
		void ConfigureTransfer(uint16_t transferBlockSize,
			uint16_t dmaBoundary)
		{
			value = transferBlockSize | dmaBoundary << 12;
		}

		static const uint16_t kDmaBoundary4K = 0;
		static const uint16_t kDmaBoundary8K = 1;
		static const uint16_t kDmaBoundary16K = 2;
		static const uint16_t kDmaBoundary32K = 3;
		static const uint16_t kDmaBoundary64K = 4;
		static const uint16_t kDmaBoundary128K = 5;
		static const uint16_t kDmaBoundary256K = 6;
		static const uint16_t kDmaBoundary512K = 7;

	private:
		NarrowRegister<uint16_t> value;
} __attribute__((packed));


// #pragma mark -
struct registers {
	// SD command generation
	volatile uint32_t system_address;
	BlockSize block_size;
	NarrowRegister<uint16_t> block_count;
	volatile uint32_t argument;
	Command command;
		// transfer mode and command

	// Response
	volatile uint32_t response[4];

	// Buffer Data Port
	volatile uint32_t buffer_data_port;

	// Host control 1
	PresentState		present_state;
	HostControl			host_control;
	PowerControl		power_control;
	NarrowRegister<uint8_t>	block_gap_control;
	NarrowRegister<uint8_t>	wakeup_control;
	ClockControl		clock_control;
	TimeoutControl		timeout_control;
	SoftwareReset		software_reset;

	// Interrupt control
	volatile uint32_t interrupt_status;
	volatile uint32_t interrupt_status_enable;
	volatile uint32_t interrupt_signal_enable;
	NarrowRegister<uint16_t> auto_cmd12_error_status;

	// Host control 2
	NarrowRegister<uint16_t> host_control_2;

	// Capabilities
	Capabilities capabilities;
	volatile uint64_t max_current_capabilities;

	// Force event
	volatile uint16_t force_event_acmd_status;
	volatile uint16_t force_event_error_status;

	// ADMA2
	volatile uint8_t adma_error_status;
	volatile uint8_t padding[3];
	volatile uint64_t adma_system_address;

	// Preset values
	volatile uint64_t preset_value[2];
	volatile uint32_t padding1 :32;
	volatile uint16_t uhs2_preset_value;
	volatile uint16_t padding2 :16;

	// ADMA3
	volatile uint64_t adma3_id_address;

	// UHS-II
	volatile uint16_t uhs2_block_size;
	volatile uint16_t padding3 :16;
	volatile uint32_t uhs2_block_count;
	volatile uint8_t uhs2_command_packet[20];
	volatile uint16_t uhs2_transfer_mode;
	volatile uint16_t uhs2_command;
	volatile uint8_t uhs2_response[20];
	volatile uint8_t uhs2_msg_select;
	volatile uint8_t padding4[3];
	volatile uint32_t uhs2_msg;
	volatile uint16_t uhs2_device_interrupt_status;
	volatile uint8_t uhs2_device_select;
	volatile uint8_t uhs2_device_int_code;
	volatile uint16_t uhs2_software_reset;
	volatile uint16_t uhs2_timer_control;
	volatile uint32_t uhs2_error_interrupt_status;
	volatile uint32_t uhs2_error_interrupt_status_enable;
	volatile uint32_t uhs2_error_interrupt_signal_enable;
	volatile uint8_t padding5[16];

	// Pointers
	volatile uint16_t uhs2_settings_pointer;
	volatile uint16_t uhs2_host_capabilities_pointer;
	volatile uint16_t uhs2_test_pointer;
	volatile uint16_t embedded_control_pointer;
	volatile uint16_t vendor_specific_pointer;
	volatile uint16_t reserved_specific_pointer;
	volatile uint8_t padding6[16];

	// Common area
	NarrowRegister<uint16_t> slot_interrupt_status;
	HostControllerVersion host_controller_version;
} __attribute__((packed));

typedef void* sdhci_mmc_bus;

struct sdhci_crs {
	uint8	irq;
//	uint8	irq_triggering;
//	uint8	irq_polarity;
//	uint8	irq_shareable;

	uint32	addr_bas;
	uint32	addr_len;
};

extern float supports_device_acpi(device_node* parent);
extern float supports_device_pci(device_node* parent);

extern status_t register_child_devices_acpi(void* cookie);
extern status_t register_child_devices_pci(void* cookie);

extern status_t init_device_pci(device_node* node, SdhciDevice* context);
extern void uninit_device_pci(SdhciDevice* context, device_node* pciParent);

extern status_t init_bus_acpi(device_node* node, void** bus_cookie);
extern status_t init_bus_pci(device_node* node, void** bus_cookie);

extern void uninit_bus(void* bus_cookie);
extern void bus_removed(void* bus_cookie);

status_t set_clock(void* controller, uint32_t kilohertz);
status_t execute_command(void* controller, uint8_t command,
	uint32_t argument, uint32_t* response);
status_t do_io(void* controller, uint8_t command,
	IOOperation* operation, bool offsetAsSectors);
void set_scan_semaphore(void* controller, sem_id sem);
void set_bus_width(void* controller, int width);
void set_card_type(void* controller, card_type type);
void terminate_bus(void* controller);

extern mmc_bus_interface gSDHCIACPIDeviceModule;
extern mmc_bus_interface gSDHCIPCIDeviceModule;

extern device_manager_info* gDeviceManager;

#endif /*_SDHCI_H*/
