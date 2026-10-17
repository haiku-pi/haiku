/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


//!	SD host controllers described by a flattened device tree.


#include <new>
#include <stdio.h>
#include <string.h>

#include <ByteOrder.h>
#include <KernelExport.h>
#include <bus/FDT.h>

#ifdef __aarch64__
#	include <bcm2835_mailbox.h>
#endif

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


#define SDHCI_FDT_MMC_BUS_MODULE_NAME "busses/mmc/sdhci/fdt/device/v1"


static const char* const kCompatible[] = {
	// The Raspberry Pi 4's SD card slot. Its registers only accept 32-bit
	// accesses (which struct registers always uses), its base clock comes
	// from the VideoCore firmware, and it does not snoop the CPU caches.
	"brcm,bcm2711-emmc2",
};


static fdt_device*
get_fdt_device(device_node* node, fdt_device_module_info** _module)
{
	// node is the mmc bus node; its parent the SDHCI device, whose parent is
	// the FDT node.
	device_node* parent = gDeviceManager->get_parent_node(node);
	device_node* fdtNode = gDeviceManager->get_parent_node(parent);
	fdt_device* device;
	gDeviceManager->get_driver(fdtNode, (driver_module_info**)_module,
		(void**)&device);
	gDeviceManager->put_node(fdtNode);
	gDeviceManager->put_node(parent);
	return device;
}


/*!	The base clock of a controller whose node has no "clock-frequency": on
	the Raspberry Pi, the VideoCore firmware knows it.
*/
static uint32
firmware_base_clock_khz()
{
#ifdef __aarch64__
	const uint32 clockID = BCM2835_CLOCK_EMMC2;
	bcm2835_mailbox_module_info* mailbox;
	if (get_module(BCM2835_MAILBOX_MODULE_NAME, (module_info**)&mailbox) != B_OK)
		return 0;

	uint32 values[2] = { clockID, 0 };
	status_t status = mailbox->property_tag(BCM2835_TAG_GET_CLOCK_RATE, values,
		sizeof(values), sizeof(uint32), NULL);
	put_module(BCM2835_MAILBOX_MODULE_NAME);

	if (status != B_OK || values[0] != clockID)
		return 0;
	return values[1] / 1000;
#else
	return 0;
#endif
}


status_t
init_bus_fdt(device_node* node, void** bus_cookie)
{
	CALLED();

	fdt_device_module_info* fdt;
	fdt_device* device = get_fdt_device(node, &fdt);

	uint64 registers, size;
	if (!fdt->get_reg(device, 0, &registers, &size)) {
		ERROR("no registers\n");
		return B_DEVICE_NOT_FOUND;
	}

	uint64 interrupt;
	if (!fdt->get_interrupt(device, 0, NULL, &interrupt) || interrupt > 0xff) {
		ERROR("no usable interrupt\n");
		return B_DEVICE_NOT_FOUND;
	}

	uint32 baseClockKHz;
	const uint32* clock = (const uint32*)fdt->get_prop(device, "clock-frequency",
		NULL);
	if (clock != NULL)
		baseClockKHz = B_BENDIAN_TO_HOST_INT32(*clock) / 1000;
	else {
		baseClockKHz = firmware_base_clock_khz();
		if (baseClockKHz == 0) {
			ERROR("cannot get the base clock from the firmware\n");
			return B_ERROR;
		}
	}

	struct registers* regs;
	area_id regsArea = map_physical_memory("sdhc_regs_map", registers,
		(size + B_PAGE_SIZE - 1) & ~(uint64)(B_PAGE_SIZE - 1), B_ANY_KERNEL_BLOCK_ADDRESS,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, (void**)&regs);
	if (regsArea < B_OK) {
		ERROR("could not map registers\n");
		return regsArea;
	}

	TRACE_ALWAYS("%s at %#" B_PRIx64 ", interrupt %" B_PRIu64 ", base clock %"
		B_PRIu32 " kHz\n", fdt->get_name(device), registers, interrupt,
		baseClockKHz);

	SdhciBus* bus = new(std::nothrow) SdhciBus(regs, interrupt, false,
		baseClockKHz);
	status_t status = bus != NULL ? bus->InitCheck() : B_NO_MEMORY;
	if (status != B_OK) {
		if (bus != NULL)
			delete bus;
		else
			delete_area(regsArea);
		return status;
	}

	int64 busOffset;
	phys_addr_t physicalEnd;
	if (!fdt->get_dma_window(device, &busOffset, &physicalEnd))
		busOffset = 0;
	bus->SetDmaWindow(busOffset,
		fdt->get_prop(device, "dma-coherent", NULL) != NULL);

	*bus_cookie = bus;
	return B_OK;
}


status_t
register_child_devices_fdt(void* cookie)
{
	CALLED();
	SdhciDevice* context = (SdhciDevice*)cookie;

	device_node* parent = gDeviceManager->get_parent_node(context->fNode);
	fdt_device_module_info* fdt;
	fdt_device* device;
	gDeviceManager->get_driver(parent, (driver_module_info**)&fdt,
		(void**)&device);
	gDeviceManager->put_node(parent);

	// Only memory the controller can reach may be handed to it.
	int64 busOffset;
	phys_addr_t dmaHighAddress = 0x100000000LL;
	phys_addr_t physicalEnd;
	if (fdt->get_dma_window(device, &busOffset, &physicalEnd)
		&& physicalEnd < dmaHighAddress) {
		dmaHighAddress = physicalEnd;
	}

	device_attr attrs[] = {
		{ B_DEVICE_PRETTY_NAME, B_STRING_TYPE, { .string = "SDHC bus" } },
		{ B_DEVICE_FIXED_CHILD, B_STRING_TYPE,
			{ .string = MMC_BUS_MODULE_NAME } },
		{ B_DEVICE_BUS, B_STRING_TYPE, { .string = "mmc" } },

		// DMA properties, see register_child_devices_pci()
		{ B_DMA_ALIGNMENT, B_UINT32_TYPE, { .ui32 = 511 } },
		{ B_DMA_HIGH_ADDRESS, B_UINT64_TYPE, { .ui64 = dmaHighAddress } },
		{ B_DMA_BOUNDARY, B_UINT32_TYPE, { .ui32 = (1 << 19) - 1 } },
		{ B_DMA_MAX_SEGMENT_COUNT, B_UINT32_TYPE, { .ui32 = 1 } },
		{ B_DMA_MAX_SEGMENT_BLOCKS, B_UINT32_TYPE, { .ui32 = (1 << 10) - 1 } },
		{ NULL }
	};

	return gDeviceManager->register_node(context->fNode,
		SDHCI_FDT_MMC_BUS_MODULE_NAME, attrs, NULL, NULL);
}


float
supports_device_fdt(device_node* parent)
{
	// TODO: there is an "fdt/compatible" attribute for each compatible string;
	// only the first (most specific) one is looked at.
	const char* compatible;
	if (gDeviceManager->get_attr_string(parent, "fdt/compatible", &compatible,
			false) != B_OK) {
		return 0.0f;
	}

	for (size_t i = 0; i < B_COUNT_OF(kCompatible); i++) {
		if (strcmp(compatible, kCompatible[i]) == 0) {
			TRACE("SDHCI device found: %s\n", compatible);
			return 0.8f;
		}
	}
	return 0.0f;
}


// Device node registered for the SD slot. It implements the MMC operations so
// the bus manager can use it to communicate with SD cards.
mmc_bus_interface gSDHCIFDTDeviceModule = {
	.info = {
		.info = {
			.name = SDHCI_FDT_MMC_BUS_MODULE_NAME,
		},

		.init_driver = init_bus_fdt,
		.uninit_driver = uninit_bus,
		.device_removed = bus_removed,
	},

	.set_clock = set_clock,
	.execute_command = execute_command,
	.do_io = do_io,
	.set_scan_semaphore = set_scan_semaphore,
	.set_bus_width = set_bus_width,
	.terminate_bus = terminate_bus,
	.set_card_type = set_card_type,
};
