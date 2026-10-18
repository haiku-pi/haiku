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


struct controller_info {
	const char*	compatible;
	uint32		firmware_clock;
		// the VideoCore's ID of the base clock, if "clock-frequency" is missing
	bool		no_high_speed_bit;
	bool		force_pio;
	bool		assume_3v3;
	int32		power_line;
		// the firmware GPIO expander line that powers the card, or -1
};

static const controller_info kControllers[] = {
	// The Raspberry Pi 4's SD card slot. Its registers only accept 32-bit
	// accesses (which struct registers always uses), its base clock comes
	// from the VideoCore firmware (EMMC2), and it does not snoop the CPU
	// caches.
	{ "brcm,bcm2711-emmc2", 12, false, false, false, -1 },
	// The Raspberry Pi's original controller (Arasan); on the Pi 4 it is
	// wired to the WiFi chip's SDIO. Linux ignores its capabilities and does
	// not use its DMA or the High Speed Enable bit, and runs it at 3.3 V; base
	// clock: EMMC. The WiFi chip's WL_REG_ON is line 1 ("WL_ON") of the
	// firmware's GPIO expander.
	{ "brcm,bcm2835-mmc", 1, true, true, true, 1 },
};


static const controller_info*
find_controller(const char* compatible)
{
	for (size_t i = 0; i < B_COUNT_OF(kControllers); i++) {
		if (strcmp(compatible, kControllers[i].compatible) == 0)
			return &kControllers[i];
	}
	return NULL;
}


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
firmware_base_clock_khz(uint32 clockID)
{
#ifdef __aarch64__
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


/*!	Power-cycles a card whose power is switched by a line of the Raspberry Pi
	firmware's GPIO expander, so that it starts from reset whatever the boot
	loader or a previous system left it in.
*/
static void
firmware_power_cycle(uint32 line)
{
#ifdef __aarch64__
	bcm2835_mailbox_module_info* mailbox;
	if (get_module(BCM2835_MAILBOX_MODULE_NAME, (module_info**)&mailbox) != B_OK)
		return;

	const uint32 gpio = BCM2835_FIRMWARE_GPIO_BASE + line;
	uint32 values[2] = { gpio, 0 };
	mailbox->property_tag(BCM2835_TAG_GET_GPIO_STATE, values, sizeof(values),
		sizeof(uint32), NULL);
	const uint32 before = values[1];

	// The line state read back afterwards tells whether this worked
	status_t status[2];
	for (uint32 state = 0; state < 2; state++) {
		values[0] = gpio;
		values[1] = state;
		status[state] = mailbox->property_tag(BCM2835_TAG_SET_GPIO_STATE,
			values, sizeof(values), sizeof(values), NULL);
		snooze(state == 0 ? 50000 : 150000);
	}

	values[0] = gpio;
	values[1] = 0;
	mailbox->property_tag(BCM2835_TAG_GET_GPIO_STATE, values, sizeof(values),
		sizeof(uint32), NULL);
	put_module(BCM2835_MAILBOX_MODULE_NAME);

	TRACE_ALWAYS("power line %" B_PRIu32 " cycled (%s, %s): %" B_PRIu32 " -> %"
		B_PRIu32 "\n", line, strerror(status[0]), strerror(status[1]), before,
		values[1]);
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

	const controller_info* info = find_controller(
		(const char*)fdt->get_prop(device, "compatible", NULL));
	if (info == NULL)
		return B_DEVICE_NOT_FOUND;

	sdhci_quirks quirks = {};
	quirks.non_removable = fdt->get_prop(device, "non-removable", NULL) != NULL;
	quirks.no_high_speed_bit = info->no_high_speed_bit;
	quirks.force_pio = info->force_pio;
	quirks.assume_3v3 = info->assume_3v3;

	uint32 baseClockKHz;
	const uint32* clock = (const uint32*)fdt->get_prop(device, "clock-frequency",
		NULL);
	if (clock != NULL)
		baseClockKHz = B_BENDIAN_TO_HOST_INT32(*clock) / 1000;
	else {
		baseClockKHz = firmware_base_clock_khz(info->firmware_clock);
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

	if (info->power_line >= 0)
		firmware_power_cycle(info->power_line);

	quirks.base_clock_khz = baseClockKHz;
	SdhciBus* bus = new(std::nothrow) SdhciBus(regs, interrupt, false, &quirks);
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
	if (!fdt->get_dma_window(device, &busOffset, &physicalEnd)) {
		busOffset = 0;
		physicalEnd = 0x100000000LL;
	}
	bus->SetDmaWindow(busOffset, physicalEnd,
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

	if (find_controller(compatible) == NULL)
		return 0.0f;

	// The Raspberry Pi's device tree describes its legacy controller twice,
	// once disabled.
	fdt_device_module_info* fdt;
	fdt_device* device;
	if (gDeviceManager->get_driver(parent, (driver_module_info**)&fdt,
			(void**)&device) != B_OK) {
		return 0.0f;
	}
	const char* status = (const char*)fdt->get_prop(device, "status", NULL);
	if (status != NULL && strcmp(status, "okay") != 0
		&& strcmp(status, "ok") != 0) {
		return 0.0f;
	}

	TRACE("SDHCI device found: %s\n", compatible);
	return 0.8f;
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
	.execute_data_command = execute_data_command,
	.execute_io_command = execute_io_command,
	.set_card_interrupt = set_card_interrupt,
	.acknowledge_card_interrupt = acknowledge_card_interrupt,
};
