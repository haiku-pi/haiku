/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


#include "BrcmPCIController.h"

#include <new>
#include <string.h>

#include <AutoDeleterDrivers.h>
#include <util/AutoLock.h>
#include <ByteOrder.h>
#include <KernelExport.h>
#include <bcm2835_mailbox.h>


//#define TRACE_BRCM_PCI
#ifdef TRACE_BRCM_PCI
#	define TRACE(x...) dprintf("brcmstb_pci: " x)
#else
#	define TRACE(x...)
#endif
#define INFO(x...) dprintf("brcmstb_pci: " x)


#define CHECK_RET(err) {status_t _err = (err); if (_err < B_OK) return _err;}


// Registers, relative to the start of the controller's register block (Linux
// drivers/pci/controller/pcie-brcmstb.c). The first 4 KiB are the root port's
// own configuration space.
enum {
	kVendorSpecificReg1		= 0x0188,
	kPriv1IdVal3			= 0x043c,
	kPriv1LinkCapability	= 0x04dc,
	kMiscCtrl				= 0x4008,
	kMemWin0Lo				= 0x400c,
	kMemWin0Hi				= 0x4010,
	kRcBar1ConfigLo			= 0x402c,
	kRcBar2ConfigLo			= 0x4034,
	kRcBar2ConfigHi			= 0x4038,
	kRcBar3ConfigLo			= 0x403c,
	kPcieStatus				= 0x4068,
	kRevision				= 0x406c,
	kMemWin0BaseLimit		= 0x4070,
	kMemWin0BaseHi			= 0x4080,
	kMemWin0LimitHi			= 0x4084,
	kHardDebug				= 0x4204,
	kExtCfgData				= 0x8000,
	kExtCfgIndex			= 0x9000,
	kRgr1SwInit1			= 0x9210,
};

enum {
	kVendorSpecificEndianMask	= 0x0000000c,
	kPriv1ClassMask				= 0x00ffffff,
	kPriv1LinkCapAspmMask		= 0x00000c00,

	kMiscCtrlRcb64B				= 1 << 7,
	kMiscCtrlRcbMps				= 1 << 10,
	kMiscCtrlScbAccess			= 1 << 12,
	kMiscCtrlCfgReadUr			= 1 << 13,
	kMiscCtrlMaxBurstMask		= 3 << 20,
	kMiscCtrlScb0SizeShift		= 27,
	kMiscCtrlScb0SizeMask		= 0x1f << 27,

	kStatusRcMode				= 1 << 7,
	kStatusDlActive				= 1 << 5,
	kStatusPhyLinkUp			= 1 << 4,

	kHardDebugSerdesIddq		= 1 << 27,
	kHardDebugClkreqMask		= 0x00310002,

	kSwInitPerst				= 1 << 0,
	kSwInitBridge				= 1 << 1,
};

// The VL805 behind the root port; its firmware version lives in config 0x50
static const uint8 kVl805Bus = 1;
static const uint16 kVl805VendorId = 0x1106;
static const uint16 kVl805FirmwareVersion = 0x50;

static const bigtime_t kPerstHoldTime = 100000;
static const bigtime_t kLinkUpTimeout = 100000;


static uint64
read_cells(const uint32* cells, int count)
{
	uint64 value = 0;
	for (int i = 0; i < count; i++)
		value = (value << 32) | B_BENDIAN_TO_HOST_INT32(cells[i]);
	return value;
}


/*!	The CPU physical address range the controller's bus masters can reach,
	from the controller node's own "dma-ranges" (PCI address = CPU address on
	the BCM2711, below 3 GiB).
*/
static phys_addr_t
dma_high_address(fdt_device_module_info* fdt, fdt_device* device)
{
	int length;
	const uint32* ranges = (const uint32*)fdt->get_prop(device, "dma-ranges",
		&length);
	// 3 cells PCI address, 2 cells CPU address, 2 cells size
	if (ranges == NULL || length < 7 * 4)
		return 0x100000000LL;
	return read_cells(ranges + 3, 2) + read_cells(ranges + 5, 2);
}


//	#pragma mark - driver


float
BrcmPCIController::SupportsDevice(device_node* parent)
{
	const char* bus;
	if (gDeviceManager->get_attr_string(parent, B_DEVICE_BUS, &bus, false)
			!= B_OK || strcmp(bus, "fdt") != 0) {
		return 0.0f;
	}

	const char* compatible;
	if (gDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible,
			false) != B_OK) {
		return 0.0f;
	}

	return strcmp(compatible, "brcm,bcm2711-pcie") == 0 ? 1.0f : 0.0f;
}


status_t
BrcmPCIController::RegisterDevice(device_node* parent)
{
	fdt_device_module_info* fdt;
	fdt_device* device;
	CHECK_RET(gDeviceManager->get_driver(parent, (driver_module_info**)&fdt,
		(void**)&device));

	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE,
			{ .string = "Broadcom STB PCIe Host Controller" } },
		{ B_DEVICE_FIXED_CHILD, B_STRING_TYPE,
			{ .string = "bus_managers/pci/root/driver_v1" } },
		// inherited by the PCI devices behind us
		{ B_DMA_HIGH_ADDRESS, B_UINT64_TYPE,
			{ .ui64 = dma_high_address(fdt, device) } },
		{}
	};

	return gDeviceManager->register_node(parent, BRCMSTB_PCI_DRIVER_MODULE_NAME,
		attrs, NULL, NULL);
}


status_t
BrcmPCIController::InitDriver(device_node* node, BrcmPCIController*& outDriver)
{
	ObjectDeleter<BrcmPCIController> driver(
		new(std::nothrow) BrcmPCIController());
	if (!driver.IsSet())
		return B_NO_MEMORY;

	CHECK_RET(driver->_InitDriverInt(node));
	outDriver = driver.Detach();
	return B_OK;
}


void
BrcmPCIController::UninitDriver()
{
	delete this;
}


status_t
BrcmPCIController::_InitDriverInt(device_node* node)
{
	fNode = node;
	CHECK_RET(_ReadResourceInfo());

	// Until the bridge is out of reset only RGR1_SW_INIT_1 and HARD_DEBUG may
	// be accessed; anything else ends in an SError.
	status_t status = _ResetAndStartLink();
	if (status != B_OK) {
		// Without a link there are no devices; keep the (empty) domain.
		INFO("link down\n");
		return B_OK;
	}

	_ConfigureRootPort();
	_StartVl805();
	return B_OK;
}


status_t
BrcmPCIController::_ReadResourceInfo()
{
	DeviceNodePutter<&gDeviceManager> fdtNode(
		gDeviceManager->get_parent_node(fNode));

	fdt_device_module_info* fdt;
	fdt_device* device;
	CHECK_RET(gDeviceManager->get_driver(fdtNode.Get(),
		(driver_module_info**)&fdt, (void**)&device));

	uint64 regs, regsLength;
	if (!fdt->get_reg(device, 0, &regs, &regsLength))
		return B_ERROR;

	fRegsArea.SetTo(map_physical_memory("brcmstb pci registers", regs,
		(regsLength + B_PAGE_SIZE - 1) & ~(uint64)(B_PAGE_SIZE - 1),
		B_ANY_KERNEL_ADDRESS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
		(void**)&fRegs));
	CHECK_RET(fRegsArea.Get());

	// "ranges": 3 cells PCI address (flags, high, low), 2 cells CPU address,
	// 2 cells size
	int length;
	const uint32* ranges = (const uint32*)fdt->get_prop(device, "ranges",
		&length);
	if (ranges == NULL)
		return B_ERROR;

	for (int i = 0; (i + 1) * 7 * 4 <= length; i++) {
		const uint32* entry = ranges + i * 7;
		uint32 flags = B_BENDIAN_TO_HOST_INT32(entry[0]);

		pci_resource_range range = {};
		range.pci_address = read_cells(entry + 1, 2);
		range.host_address = read_cells(entry + 3, 2);
		range.size = read_cells(entry + 5, 2);
		if ((flags & 0x03000000) != 0x02000000) {
			// only 32-bit memory space can be routed through the windows
			continue;
		}
		range.type = B_IO_MEMORY;
		range.address_type = PCI_address_type_32;
		if ((flags & 0x40000000) != 0)
			range.address_type |= PCI_address_prefetchable;

		INFO("window: PCI %#" B_PRIx64 " <- CPU %#" B_PRIx64 ", %#" B_PRIx64
			" bytes\n", range.pci_address, range.host_address, range.size);
		fResourceRanges.Add(range);
	}

	if (fResourceRanges.Count() == 0)
		return B_ERROR;
	return B_OK;
}


/*!	Resets the controller, programs its windows and trains the link. U-Boot
	shuts the controller down when it starts the OS, so this is always needed.
	Follows Linux's brcm_pcie_setup() and brcm_pcie_start_link().
*/
status_t
BrcmPCIController::_ResetAndStartLink()
{
	// Hold the bridge in reset and assert PERST#; then take the bridge out of
	// reset and power up the SerDes. Nothing else can be touched before.
	_Modify32(kRgr1SwInit1, 0, kSwInitBridge);
	_Modify32(kRgr1SwInit1, 0, kSwInitPerst);
	const bigtime_t perstAsserted = system_time();
	snooze(200);
	_Modify32(kRgr1SwInit1, kSwInitBridge, 0);
	_Modify32(kHardDebug, kHardDebugSerdesIddq, 0);
	snooze(200);

	// SCB access, unsupported requests read as all ones, 128-byte bursts (the
	// limit on the BCM2711), and a 4 GiB memory controller window.
	_Modify32(kMiscCtrl, kMiscCtrlMaxBurstMask | kMiscCtrlScb0SizeMask,
		kMiscCtrlScbAccess | kMiscCtrlCfgReadUr | kMiscCtrlRcbMps
			| kMiscCtrlRcb64B | (17 << kMiscCtrlScb0SizeShift));

	_ProgramInboundWindow();

	if ((_Read32(kPcieStatus) & kStatusRcMode) == 0) {
		INFO("controller is not in root complex mode\n");
		return B_ERROR;
	}

	// A PCI-to-PCI bridge, without ASPM
	_Modify32(kPriv1IdVal3, kPriv1ClassMask, 0x060400);
	_Modify32(kPriv1LinkCapability, kPriv1LinkCapAspmMask, 0);

	const pci_resource_range& window = fResourceRanges[0];
	_ProgramOutboundWindow(window.host_address, window.pci_address,
		window.size);

	// little-endian inbound accesses; CLKREQ# not used
	_Modify32(kVendorSpecificReg1, kVendorSpecificEndianMask, 0);
	_Modify32(kHardDebug, kHardDebugClkreqMask, 0);

	// PERST# must be held for at least 100 ms
	bigtime_t held = system_time() - perstAsserted;
	if (held < kPerstHoldTime)
		snooze(kPerstHoldTime - held);
	_Modify32(kRgr1SwInit1, kSwInitPerst, 0);
	snooze(100000);

	for (bigtime_t waited = 0; !_LinkUp(); waited += 5000) {
		if (waited >= kLinkUpTimeout)
			return B_TIMED_OUT;
		snooze(5000);
	}

	fLinkUp = true;
	INFO("revision %#" B_PRIx32 "\n", _Read32(kRevision));
	uint16 linkStatus = *(vuint16*)(fRegs + 0xac + 0x12);
	INFO("link up: gen %u, x%u\n", linkStatus & 0xf, (linkStatus >> 4) & 0x3f);
	return B_OK;
}


/*!	Lets bus masters reach the first 4 GiB of the memory controller, at the
	same PCI address (the device tree's "dma-ranges" limit the RAM really
	reachable to 3 GiB, see RegisterDevice()).
*/
void
BrcmPCIController::_ProgramInboundWindow()
{
	_Write32(kRcBar1ConfigLo, 0);
	_Write32(kRcBar2ConfigLo, 0 | (32 - 15));
		// PCI offset 0, size code log2(4 GiB) - 15
	_Write32(kRcBar2ConfigHi, 0);
	_Write32(kRcBar3ConfigLo, 0);
}


void
BrcmPCIController::_ProgramOutboundWindow(phys_addr_t cpuAddress,
	phys_addr_t pciAddress, uint64 size)
{
	_Write32(kMemWin0Lo, (uint32)pciAddress);
	_Write32(kMemWin0Hi, (uint32)(pciAddress >> 32));

	// CPU base and limit in MiB: the low 12 bits here, the rest in the HI
	// registers
	const uint64 baseMb = cpuAddress >> 20;
	const uint64 limitMb = (cpuAddress + size - 1) >> 20;
	_Write32(kMemWin0BaseLimit, (uint32)((limitMb & 0xfff) << 20)
		| (uint32)((baseMb & 0xfff) << 4));
	_Modify32(kMemWin0BaseHi, 0xff, (uint32)(baseMb >> 12));
	_Modify32(kMemWin0LimitHi, 0xff, (uint32)(limitMb >> 12));
}


/*!	The root port is a PCI-to-PCI bridge in front of bus 1. Haiku's PCI bus
	manager does not program bridges on this platform, so do it here: bus
	numbers, the memory window (all of the outbound window), no I/O or
	prefetchable windows, then memory decoding and bus mastering.
*/
void
BrcmPCIController::_ConfigureRootPort()
{
	const pci_resource_range& window = fResourceRanges[0];
	const uint32 base = (uint32)window.pci_address;
	const uint32 limit = (uint32)(window.pci_address + window.size - 1);

	WriteConfig(0, 0, 0, PCI_primary_bus, 1, 0);
	WriteConfig(0, 0, 0, PCI_secondary_bus, 1, kVl805Bus);
	WriteConfig(0, 0, 0, PCI_subordinate_bus, 1, kVl805Bus);
	WriteConfig(0, 0, 0, PCI_memory_base, 2, (base >> 16) & 0xfff0);
	WriteConfig(0, 0, 0, PCI_memory_limit, 2, (limit >> 16) & 0xfff0);
	WriteConfig(0, 0, 0, PCI_io_base, 1, 0xf0);
	WriteConfig(0, 0, 0, PCI_io_limit, 1, 0);
	WriteConfig(0, 0, 0, PCI_prefetchable_memory_base, 2, 0xfff0);
	WriteConfig(0, 0, 0, PCI_prefetchable_memory_limit, 2, 0);
	WriteConfig(0, 0, 0, PCI_prefetchable_memory_base_upper32, 4, 0);
	WriteConfig(0, 0, 0, PCI_prefetchable_memory_limit_upper32, 4, 0);

	uint32 command;
	ReadConfig(0, 0, 0, PCI_command, 2, command);
	WriteConfig(0, 0, 0, PCI_command, 2,
		command | PCI_command_memory | PCI_command_master);
}


/*!	Boards without an EEPROM for the VL805 get its firmware from the
	VideoCore, which has to be told after every reset of the device.
*/
void
BrcmPCIController::_StartVl805()
{
	uint32 vendor, version;
	ReadConfig(kVl805Bus, 0, 0, PCI_vendor_id, 2, vendor);
	ReadConfig(kVl805Bus, 0, 0, kVl805FirmwareVersion, 4, version);
	INFO("device %#" B_PRIx32 " behind the root port, firmware %#" B_PRIx32
		"\n", vendor, version);
	if (vendor == kVl805VendorId && version != 0)
		return;

	bcm2835_mailbox_module_info* mailbox;
	if (get_module(BCM2835_MAILBOX_MODULE_NAME, (module_info**)&mailbox)
			!= B_OK) {
		INFO("no mailbox, cannot load the VL805 firmware\n");
		return;
	}

	uint32 address = (uint32)kVl805Bus << 20;
		// device 0, function 0
	status_t status = mailbox->property_tag(BCM2835_TAG_NOTIFY_XHCI_RESET,
		&address, sizeof(address), sizeof(address), NULL);
	put_module(BCM2835_MAILBOX_MODULE_NAME);
	if (status != B_OK) {
		INFO("firmware load request failed: %s\n", strerror(status));
		return;
	}

	for (int i = 0; i < 60; i++) {
		snooze(5000);
		ReadConfig(kVl805Bus, 0, 0, PCI_vendor_id, 2, vendor);
		if (vendor == kVl805VendorId)
			break;
	}
	ReadConfig(kVl805Bus, 0, 0, kVl805FirmwareVersion, 4, version);
	INFO("VL805 firmware %#" B_PRIx32 " loaded\n", version);

	// The firmware's handling of the request can disturb the inbound window.
	_ProgramInboundWindow();
}


bool
BrcmPCIController::_LinkUp()
{
	uint32 status = _Read32(kPcieStatus);
	return (status & kStatusDlActive) != 0 && (status & kStatusPhyLinkUp) != 0;
}


//	#pragma mark - PCI controller


/*!	Returns the address of a configuration register, or 0 where no function
	can be. The caller holds fLock: the index register and the data window
	are shared.
*/
addr_t
BrcmPCIController::_ConfigAddress(uint8 bus, uint8 device, uint8 function,
	uint16 offset)
{
	if (offset > 0xfff)
		return 0;

	if (bus == 0) {
		// The root port's header is the start of the register block; it
		// ignores the device number, so anything but 0:0.0 would alias it.
		if (device != 0 || function != 0)
			return 0;
		return fRegs + offset;
	}

	// a single-device link; and config cycles without a link abort the CPU
	if (device != 0 || !fLinkUp || !_LinkUp())
		return 0;

	_Write32(kExtCfgIndex, ((uint32)bus << 20) | ((uint32)device << 15)
		| ((uint32)function << 12));
	return fRegs + kExtCfgData + offset;
}


status_t
BrcmPCIController::ReadConfig(uint8 bus, uint8 device, uint8 function,
	uint16 offset, uint8 size, uint32& value)
{
	if (size != 1 && size != 2 && size != 4)
		return B_BAD_VALUE;

	InterruptsSpinLocker locker(fLock);

	addr_t address = _ConfigAddress(bus, device, function, offset);
	if (address == 0) {
		// like a master abort
		value = size == 4 ? 0xffffffff : (1u << (size * 8)) - 1;
		return B_OK;
	}

	switch (size) {
		case 1: value = *(vuint8*)address; break;
		case 2: value = *(vuint16*)address; break;
		case 4: value = *(vuint32*)address; break;
	}
	return B_OK;
}


status_t
BrcmPCIController::WriteConfig(uint8 bus, uint8 device, uint8 function,
	uint16 offset, uint8 size, uint32 value)
{
	if (size != 1 && size != 2 && size != 4)
		return B_BAD_VALUE;

	InterruptsSpinLocker locker(fLock);

	addr_t address = _ConfigAddress(bus, device, function, offset);
	if (address == 0)
		return B_OK;

	// Native widths: a widened write of PCI_command would also write
	// PCI_status, clearing its write-1-to-clear bits.
	switch (size) {
		case 1: *(vuint8*)address = (uint8)value; break;
		case 2: *(vuint16*)address = (uint16)value; break;
		case 4: *(vuint32*)address = value; break;
	}
	return B_OK;
}


status_t
BrcmPCIController::GetMaxBusDevices(int32& count)
{
	count = 32;
	return B_OK;
}


status_t
BrcmPCIController::ReadIrq(uint8 bus, uint8 device, uint8 function, uint8 pin,
	uint8& irq)
{
	return B_UNSUPPORTED;
}


status_t
BrcmPCIController::WriteIrq(uint8 bus, uint8 device, uint8 function, uint8 pin,
	uint8 irq)
{
	return B_UNSUPPORTED;
}


status_t
BrcmPCIController::GetRange(uint32 index, pci_resource_range* range)
{
	if (index >= (uint32)fResourceRanges.Count())
		return B_BAD_INDEX;

	*range = fResourceRanges[index];
	return B_OK;
}


/*!	Routes the legacy interrupts (INTA-D) of the devices behind us through
	the controller node's "interrupt-map".
*/
status_t
BrcmPCIController::Finalize()
{
	DeviceNodePutter<&gDeviceManager> parent(
		gDeviceManager->get_parent_node(fNode));

	fdt_device_module_info* fdt;
	fdt_device* device;
	CHECK_RET(gDeviceManager->get_driver(parent.Get(),
		(driver_module_info**)&fdt, (void**)&device));

	struct fdt_interrupt_map* interruptMap = fdt->get_interrupt_map(device);
	if (interruptMap == NULL)
		return B_OK;

	uint32 vendor;
	ReadConfig(kVl805Bus, 0, 0, PCI_vendor_id, 2, vendor);
	if (vendor != 0xffff)
		_FinalizeInterrupts(fdt, interruptMap, kVl805Bus, 0, 0);
	return B_OK;
}


void
BrcmPCIController::_FinalizeInterrupts(fdt_device_module_info* fdt,
	struct fdt_interrupt_map* interruptMap, uint8 bus, uint8 device,
	uint8 function)
{
	uint32 pin;
	ReadConfig(bus, device, function, PCI_interrupt_pin, 1, pin);
	if (pin == 0 || pin == 0xff)
		return;

	uint32 childAddress = ((uint32)bus << 16) | ((uint32)device << 11)
		| ((uint32)function << 8);
	uint32 irq = fdt->lookup_interrupt_map(interruptMap, childAddress, pin);
	if (irq == 0xffffffff) {
		INFO("no interrupt for %u:%u.%u, pin %" B_PRIu32 "\n", bus, device,
			function, pin);
		return;
	}

	INFO("%u:%u.%u: INT%c -> interrupt %" B_PRIu32 "\n", bus, device, function,
		'A' + (int)pin - 1, irq);
	gPCI->update_interrupt_line(bus, device, function, irq);
}
