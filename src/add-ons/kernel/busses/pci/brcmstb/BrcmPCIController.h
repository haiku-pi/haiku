/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _BRCM_PCI_CONTROLLER_H_
#define _BRCM_PCI_CONTROLLER_H_


#include <bus/FDT.h>
#include <bus/PCI.h>

#include <AutoDeleterOS.h>
#include <lock.h>
#include <util/Vector.h>


#define BRCMSTB_PCI_DRIVER_MODULE_NAME "busses/pci/brcmstb/driver_v1"


/*!	The PCIe root complex of the Broadcom set-top box family, as found in the
	BCM2711 of the Raspberry Pi 4 ("brcm,bcm2711-pcie"), whose single lane
	connects the VIA VL805 USB 3 controller.
*/
class BrcmPCIController {
public:
	static	float				SupportsDevice(device_node* parent);
	static	status_t			RegisterDevice(device_node* parent);
	static	status_t			InitDriver(device_node* node,
									BrcmPCIController*& outDriver);
			void				UninitDriver();

			status_t			ReadConfig(uint8 bus, uint8 device, uint8 function,
									uint16 offset, uint8 size, uint32& value);
			status_t			WriteConfig(uint8 bus, uint8 device,
									uint8 function, uint16 offset, uint8 size,
									uint32 value);
			status_t			GetMaxBusDevices(int32& count);
			status_t			ReadIrq(uint8 bus, uint8 device, uint8 function,
									uint8 pin, uint8& irq);
			status_t			WriteIrq(uint8 bus, uint8 device, uint8 function,
									uint8 pin, uint8 irq);
			status_t			GetRange(uint32 index, pci_resource_range* range);
			status_t			Finalize();

private:
			status_t			_InitDriverInt(device_node* node);
			status_t			_ReadResourceInfo();
			status_t			_ResetAndStartLink();
			void				_ProgramInboundWindow();
			void				_ProgramOutboundWindow(phys_addr_t cpuAddress,
									phys_addr_t pciAddress, uint64 size);
			void				_ConfigureRootPort();
			void				_StartVl805();
			bool				_LinkUp();
			addr_t				_ConfigAddress(uint8 bus, uint8 device,
									uint8 function, uint16 offset);
			void				_FinalizeInterrupts(fdt_device_module_info* fdt,
									struct fdt_interrupt_map* interruptMap,
									uint8 bus, uint8 device, uint8 function);

			uint32				_Read32(uint32 offset)
									{ return *(vuint32*)(fRegs + offset); }
			void				_Write32(uint32 offset, uint32 value)
									{ *(vuint32*)(fRegs + offset) = value; }
			void				_Modify32(uint32 offset, uint32 clear,
									uint32 set)
									{ _Write32(offset,
										(_Read32(offset) & ~clear) | set); }

private:
			spinlock			fLock = B_SPINLOCK_INITIALIZER;
				// guards the index register and the data window
			device_node*		fNode = NULL;
			AreaDeleter			fRegsArea;
			addr_t				fRegs = 0;
			Vector<pci_resource_range> fResourceRanges;
			bool				fLinkUp = false;
};


extern device_manager_info* gDeviceManager;
extern pci_module_info* gPCI;


#endif	// _BRCM_PCI_CONTROLLER_H_
