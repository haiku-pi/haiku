/*
 * Copyright 2020-2021, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _DRIVERS_BUS_FDT_H
#define _DRIVERS_BUS_FDT_H


#include <device_manager.h>


struct fdt_bus;
struct fdt_device;
struct fdt_interrupt_map;

typedef struct fdt_bus_module_info {
	driver_module_info info;
	device_node* (*node_by_phandle)(struct fdt_bus* bus, int phandle);
} fdt_bus_module_info;

typedef struct fdt_device_module_info {
	driver_module_info info;
	device_node* (*get_bus)(struct fdt_device* dev);
	const char* (*get_name)(struct fdt_device* dev);
	const void* (*get_prop)(struct fdt_device* dev, const char* name, int* len);
	bool (*get_reg)(struct fdt_device* dev, uint32 ord, uint64* regs, uint64* len);
	bool (*get_interrupt)(struct fdt_device* dev, uint32 ord,
		device_node** interruptController, uint64* interrupt);
	struct fdt_interrupt_map* (*get_interrupt_map)(struct fdt_device* dev);
	void (*print_interrupt_map)(struct fdt_interrupt_map* interruptMap);
	uint32 (*lookup_interrupt_map)(struct fdt_interrupt_map* interruptMap, uint32 childAddr, uint32 childIrq);

	/* How the device reaches memory: its DMA address for a physical address
	   is the physical address plus *busOffset, for physical addresses below
	   *physicalEnd. From the "dma-ranges" of the nearest bus that has one (its
	   largest range, the one for RAM); false if none does (DMA addresses are
	   physical addresses). */
	bool (*get_dma_window)(struct fdt_device* dev, int64* busOffset,
		phys_addr_t* physicalEnd);
} fdt_device_module_info;


#endif // _DRIVERS_BUS_FDT_H
