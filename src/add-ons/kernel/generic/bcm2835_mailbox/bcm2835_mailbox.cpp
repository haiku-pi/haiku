/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The property interface of the Raspberry Pi's VideoCore firmware.

	A request is a buffer in memory that the VideoCore reads and overwrites
	with the response; its bus address is posted to the ARM-to-VideoCore
	mailbox (channel 8) and the VideoCore posts it back when done. The
	VideoCore does not snoop the ARM caches.
*/


#include <bcm2835_mailbox.h>

#include <string.h>

#include <KernelExport.h>
#include <arch/cpu.h>
#include <lock.h>
#include <util/AutoLock.h>
#include <vm/vm.h>

extern "C" {
#include <libfdt.h>
}


//#define TRACE_MAILBOX
#ifdef TRACE_MAILBOX
#	define TRACE(x...) dprintf("bcm2835_mailbox: " x)
#else
#	define TRACE(x...)
#endif
#define ERROR(x...) dprintf("bcm2835_mailbox: " x)


extern void* gFDT;


// Mailbox registers: mailbox 0 carries VideoCore-to-ARM messages, mailbox 1
// ARM-to-VideoCore ones.
enum {
	MAILBOX0_READ		= 0x00 / 4,
	MAILBOX0_STATUS		= 0x18 / 4,
	MAILBOX1_WRITE		= 0x20 / 4,
	MAILBOX1_STATUS		= 0x38 / 4,
};

#define MAILBOX_STATUS_FULL		(1u << 31)
#define MAILBOX_STATUS_EMPTY	(1u << 30)
#define MAILBOX_CHANNEL_MASK	0xf
#define MAILBOX_CHANNEL_PROPERTY_ARM_TO_VC	8

#define PROPERTY_REQUEST			0x00000000
#define PROPERTY_RESPONSE_SUCCESS	0x80000000
#define TAG_RESPONSE				0x80000000

// The whole request: size, code, one tag (id, buffer size, code, values), end tag.
#define REQUEST_HEADER_WORDS	5
#define MAX_VALUES_SIZE			(B_PAGE_SIZE - (REQUEST_HEADER_WORDS + 1) * 4)

static const bigtime_t kResponseTimeout = 1000000;


static mutex sLock = MUTEX_INITIALIZER("bcm2835 mailbox");
static volatile uint32* sRegisters;
static area_id sRegistersArea = -1;
static uint32* sBuffer;
static area_id sBufferArea = -1;
static uint32 sBufferBusAddress;


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
	const uint32* prop = (const uint32*)fdt_getprop(gFDT, node, "#address-cells",
		NULL);
	return prop != NULL ? fdt32_to_cpu(*prop) : 2;
}


static uint32
size_cells(int node)
{
	const uint32* prop = (const uint32*)fdt_getprop(gFDT, node, "#size-cells",
		NULL);
	return prop != NULL ? fdt32_to_cpu(*prop) : 1;
}


/*!	Applies the first matching entry of the "ranges" (or "dma-ranges") of
	\a bus to \a address, in the direction given by \a toParent.
*/
static bool
apply_ranges(int bus, const char* property, bool toParent, uint64& address)
{
	int length;
	const uint32* ranges = (const uint32*)fdt_getprop(gFDT, bus, property,
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
		uint64 from = toParent ? child : parent;
		uint64 to = toParent ? parent : child;
		if (address >= from && address - from < size) {
			address = address - from + to;
			return true;
		}
	}
	return false;
}


static status_t
find_mailbox(phys_addr_t& registers, int& bus)
{
	if (gFDT == NULL)
		return B_DEV_NOT_READY;

	int node = fdt_node_offset_by_compatible(gFDT, -1, "brcm,bcm2835-mbox");
	if (node < 0)
		return B_DEVICE_NOT_FOUND;

	bus = fdt_parent_offset(gFDT, node);
	int length;
	const uint32* reg = (const uint32*)fdt_getprop(gFDT, node, "reg", &length);
	if (reg == NULL || bus < 0)
		return B_BAD_DATA;

	uint64 address = read_cells(reg, address_cells(bus));
	for (int parent = bus; fdt_parent_offset(gFDT, parent) >= 0;
			parent = fdt_parent_offset(gFDT, parent)) {
		if (!apply_ranges(parent, "ranges", true, address))
			return B_BAD_DATA;
	}

	registers = address;
	return B_OK;
}


static status_t
init_mailbox()
{
	phys_addr_t registers;
	int bus;
	status_t status = find_mailbox(registers, bus);
	if (status != B_OK) {
		ERROR("no mailbox in the device tree: %s\n", strerror(status));
		return status;
	}

	void* address;
	sRegistersArea = map_physical_memory("bcm2835 mailbox registers",
		registers & ~(phys_addr_t)(B_PAGE_SIZE - 1), B_PAGE_SIZE,
		B_ANY_KERNEL_ADDRESS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, &address);
	if (sRegistersArea < 0)
		return sRegistersArea;
	sRegisters = (volatile uint32*)((addr_t)address
		+ (registers & (B_PAGE_SIZE - 1)));

	// The request buffer must be visible to the VideoCore: below the end of
	// the bus's DMA window ("dma-ranges"), at the window's bus address.
	uint64 windowEnd = 0x40000000;
	{
		int length;
		const uint32* ranges = (const uint32*)fdt_getprop(gFDT, bus,
			"dma-ranges", &length);
		if (ranges != NULL && length > 0) {
			uint32 childCells = address_cells(bus);
			uint32 parentCells = address_cells(fdt_parent_offset(gFDT, bus));
			windowEnd = read_cells(ranges + childCells, parentCells)
				+ read_cells(ranges + childCells + parentCells, size_cells(bus));
		}
	}

	physical_address_restrictions physicalRestrictions = {};
	physicalRestrictions.high_address = windowEnd;
	virtual_address_restrictions virtualRestrictions = {};
	virtualRestrictions.address_specification = B_ANY_KERNEL_ADDRESS;
	sBufferArea = create_area_etc(B_SYSTEM_TEAM, "bcm2835 mailbox buffer",
		B_PAGE_SIZE, B_CONTIGUOUS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, 0, 0,
		&virtualRestrictions, &physicalRestrictions, (void**)&sBuffer);
	if (sBufferArea < 0) {
		delete_area(sRegistersArea);
		return sBufferArea;
	}

	physical_entry entry;
	get_memory_map(sBuffer, B_PAGE_SIZE, &entry, 1);
	uint64 busAddress = entry.address;
	if (!apply_ranges(bus, "dma-ranges", false, busAddress)
		|| busAddress > UINT32_MAX) {
		ERROR("buffer at %#" B_PRIxPHYSADDR " has no bus address\n",
			entry.address);
		delete_area(sBufferArea);
		delete_area(sRegistersArea);
		return B_ERROR;
	}
	sBufferBusAddress = busAddress;

	TRACE("registers at %#" B_PRIxPHYSADDR ", buffer at %#" B_PRIxPHYSADDR
		" (bus %#" B_PRIx32 ")\n", registers, entry.address, sBufferBusAddress);
	return B_OK;
}


static void
uninit_mailbox()
{
	delete_area(sBufferArea);
	delete_area(sRegistersArea);
}


static status_t
wait_for_status(uint32 statusRegister, uint32 notSet)
{
	bigtime_t timeout = system_time() + kResponseTimeout;
	while ((sRegisters[statusRegister] & notSet) != 0) {
		if (system_time() > timeout)
			return B_TIMED_OUT;
		spin(10);
	}
	return B_OK;
}


static status_t
property_tag(uint32 tag, void* values, uint32 bufferSize, uint32 requestSize,
	uint32* _responseSize)
{
	if (bufferSize > MAX_VALUES_SIZE || requestSize > bufferSize
		|| (bufferSize % 4) != 0) {
		return B_BAD_VALUE;
	}

	MutexLocker locker(sLock);

	uint32 words = REQUEST_HEADER_WORDS + bufferSize / 4 + 1;
	sBuffer[0] = words * 4;
	sBuffer[1] = PROPERTY_REQUEST;
	sBuffer[2] = tag;
	sBuffer[3] = bufferSize;
	sBuffer[4] = 0;
	memset(sBuffer + REQUEST_HEADER_WORDS, 0, bufferSize);
	memcpy(sBuffer + REQUEST_HEADER_WORDS, values, requestSize);
	sBuffer[words - 1] = 0;
		// end tag

	arm64_dcache_flush_range((addr_t)sBuffer, words * 4);

	if (wait_for_status(MAILBOX1_STATUS, MAILBOX_STATUS_FULL) != B_OK) {
		ERROR("mailbox stays full\n");
		return B_TIMED_OUT;
	}
	sRegisters[MAILBOX1_WRITE]
		= sBufferBusAddress | MAILBOX_CHANNEL_PROPERTY_ARM_TO_VC;

	while (true) {
		if (wait_for_status(MAILBOX0_STATUS, MAILBOX_STATUS_EMPTY) != B_OK) {
			ERROR("no response to tag %#" B_PRIx32 "\n", tag);
			return B_TIMED_OUT;
		}
		uint32 message = sRegisters[MAILBOX0_READ];
		if (message == (sBufferBusAddress | MAILBOX_CHANNEL_PROPERTY_ARM_TO_VC))
			break;
		TRACE("ignoring message %#" B_PRIx32 "\n", message);
	}

	arm64_dcache_flush_range((addr_t)sBuffer, words * 4);

	// Like Linux, only the buffer's status counts: the firmware does not mark
	// the response of every tag (SET_GPIO_STATE, for one)
	if (sBuffer[1] != PROPERTY_RESPONSE_SUCCESS) {
		ERROR("tag %#" B_PRIx32 " failed: %#" B_PRIx32 ", %#" B_PRIx32 "\n",
			tag, sBuffer[1], sBuffer[4]);
		return B_ERROR;
	}

	uint32 responseSize = sBuffer[4] & ~TAG_RESPONSE;
	memcpy(values, sBuffer + REQUEST_HEADER_WORDS,
		min_c(responseSize, bufferSize));
	if (_responseSize != NULL)
		*_responseSize = responseSize;
	return B_OK;
}


static status_t
std_ops(int32 op, ...)
{
	switch (op) {
		case B_MODULE_INIT:
			return init_mailbox();
		case B_MODULE_UNINIT:
			uninit_mailbox();
			return B_OK;
		default:
			return B_ERROR;
	}
}


static bcm2835_mailbox_module_info sMailboxModule = {
	{
		BCM2835_MAILBOX_MODULE_NAME,
		0,
		std_ops
	},
	property_tag
};


module_info* modules[] = {
	(module_info*)&sMailboxModule,
	NULL
};
