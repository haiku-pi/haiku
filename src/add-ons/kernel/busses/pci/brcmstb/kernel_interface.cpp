/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


#include "BrcmPCIController.h"


device_manager_info* gDeviceManager;
pci_module_info* gPCI;


pci_controller_module_info gPciControllerDriver = {
	.info = {
		.info = {
			.name = BRCMSTB_PCI_DRIVER_MODULE_NAME,
		},
		.supports_device = BrcmPCIController::SupportsDevice,
		.register_device = BrcmPCIController::RegisterDevice,
		.init_driver = [](device_node* node, void** driverCookie) {
			return BrcmPCIController::InitDriver(node,
				*(BrcmPCIController**)driverCookie);
		},
		.uninit_driver = [](void* driverCookie) {
			return static_cast<BrcmPCIController*>(driverCookie)->UninitDriver();
		},
	},
	.read_pci_config = [](void* cookie,
		uint8 bus, uint8 device, uint8 function,
		uint16 offset, uint8 size, uint32* value) {
		return static_cast<BrcmPCIController*>(cookie)
			->ReadConfig(bus, device, function, offset, size, *value);
	},
	.write_pci_config = [](void* cookie,
		uint8 bus, uint8 device, uint8 function,
		uint16 offset, uint8 size, uint32 value) {
		return static_cast<BrcmPCIController*>(cookie)
			->WriteConfig(bus, device, function, offset, size, value);
	},
	.get_max_bus_devices = [](void* cookie, int32* count) {
		return static_cast<BrcmPCIController*>(cookie)->GetMaxBusDevices(*count);
	},
	.read_pci_irq = [](void* cookie,
		uint8 bus, uint8 device, uint8 function,
		uint8 pin, uint8 *irq) {
		return static_cast<BrcmPCIController*>(cookie)
			->ReadIrq(bus, device, function, pin, *irq);
	},
	.write_pci_irq = [](void* cookie,
		uint8 bus, uint8 device, uint8 function,
		uint8 pin, uint8 irq) {
		return static_cast<BrcmPCIController*>(cookie)
			->WriteIrq(bus, device, function, pin, irq);
	},
	.get_range = [](void *cookie, uint32 index, pci_resource_range* range) {
		return static_cast<BrcmPCIController*>(cookie)->GetRange(index, range);
	},
	.finalize = [](void *cookie) {
		return static_cast<BrcmPCIController*>(cookie)->Finalize();
	}
};


_EXPORT module_dependency module_dependencies[] = {
	{ B_DEVICE_MANAGER_MODULE_NAME, (module_info**)&gDeviceManager },
	{ B_PCI_MODULE_NAME, (module_info**)&gPCI },
	{}
};

_EXPORT module_info *modules[] = {
	(module_info *)&gPciControllerDriver,
	NULL
};
