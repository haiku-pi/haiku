/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef VCHIQ_HAIKU_H
#define VCHIQ_HAIKU_H


/*!	The interface between the driver (vchiq_haiku.cpp, C++) and the ported
	VCHIQ core (C). Plain types only: the core's headers are C.
*/


#include <KernelExport.h>


#ifdef __cplusplus
extern "C" {
#endif


/* Provided by the driver */

/*!	Contiguous memory the VideoCore reaches (below the end of its DMA window),
	cleared and mapped non-cacheable. Returns its kernel address, physical
	address and area. */
status_t vchiq_haiku_dma_alloc(const char* name, size_t size,
	void** _address, phys_addr_t* _physical, area_id* _area);
void vchiq_haiku_dma_free(area_id area);

/*!	The VideoCore's address of physical memory \a physical (through the
	bus's DMA window), and whether bulk page lists hold physical addresses
	(BCM2711: 36-bit "long" bulk addresses) instead. */
uint32 vchiq_haiku_bus_address(phys_addr_t physical);
bool vchiq_haiku_long_bulk_addresses(void);
uint32 vchiq_haiku_cache_line_size(void);

/*!	The firmware's VCHIQ_INIT property call with the slots' bus address. */
status_t vchiq_haiku_firmware_init(uint32 slotsBusAddress);

/*!	Rings the VideoCore's doorbell (BELL2). */
void vchiq_haiku_ring_doorbell(void);

/*!	The memory of a bulk transfer, locked and resolved to physical runs. */
typedef struct vchiq_haiku_buffer {
	team_id			team;
	void*			address;
	size_t			size;
	bool			device_writes;
	uint32			run_count;
	physical_entry	runs[];
} vchiq_haiku_buffer;

status_t vchiq_haiku_buffer_lock(void* address, size_t size,
	bool deviceWrites, vchiq_haiku_buffer** _buffer);
void vchiq_haiku_buffer_unlock(vchiq_haiku_buffer* buffer);

/*!	Data cache maintenance on \a size bytes from \a offset into the buffer,
	through the physical map (any thread may call it). */
void vchiq_haiku_buffer_clean(vchiq_haiku_buffer* buffer, size_t offset,
	size_t size);
void vchiq_haiku_buffer_flush(vchiq_haiku_buffer* buffer, size_t offset,
	size_t size);
void vchiq_haiku_buffer_write(vchiq_haiku_buffer* buffer, size_t offset,
	const void* data, size_t size);


/* Provided by the core's Haiku glue (vchiq_arm.c, vchiq_2835_arm.c) */

/*!	Allocates the slots, starts the core's threads and hands the slots to
	the firmware. Once only: the firmware keeps using the memory. */
status_t vchiq_haiku_init(void);

/*!	From the doorbell interrupt: wakes whoever waits for the VideoCore. */
void vchiq_haiku_doorbell(void);

/*!	The /dev/vchiq file interface. The ioctl runs on a kernel copy of the
	argument and returns Linux's values: a count, or a negative error. */
status_t vchiq_haiku_open(team_id team, void** _instance);
void vchiq_haiku_free(void* instance);
int vchiq_haiku_ioctl(void* instance, unsigned long command, void* argument);

/*!	The core's state dump, as text; returns its length. */
size_t vchiq_haiku_dump_state(char* buffer, size_t size);

/*!	Sends \a command to the firmware's gencmd service ("GCMD") and returns
	its reply text (NUL-terminated) and the firmware's result code. */
status_t vchiq_haiku_gencmd(const char* command, char* reply, size_t size,
	int32* _result);


#ifdef __cplusplus
}
#endif


#endif	/* VCHIQ_HAIKU_H */
