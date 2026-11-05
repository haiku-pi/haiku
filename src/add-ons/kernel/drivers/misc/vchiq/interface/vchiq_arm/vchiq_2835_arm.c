/**
 * Copyright (c) 2010-2012 Broadcom. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Haiku port: the VideoCore platform layer on Haiku's memory primitives
 * (vchiq_haiku.cpp), with the doorbell signalling of FreeBSD's vchiq_kmod.c.
 */

#include <interface/compat/vchi_haiku.h>

#include "vchiq_arm.h"
#include "vchiq_2835.h"
#include "vchiq_connected.h"
#include "vchiq_killable.h"

#include "vchiq_haiku.h"

#define TOTAL_SLOTS (VCHIQ_SLOT_ZERO_SLOTS + 2 * 32)

#define MAX_FRAGMENTS (VCHIQ_NUM_CURRENT_BULKS * 2)

static int g_cache_line_size = 64;
static int g_fragment_size;

typedef struct vchiq_2835_state_struct {
   int inited;
   VCHIQ_ARM_STATE_T arm_state;
} VCHIQ_2835_ARM_STATE_T;

static char *g_slot_mem;
static int g_slot_mem_size;
static phys_addr_t g_slot_phys;
static area_id g_slot_area;

static char *g_fragments_base;
static char *g_free_fragments;
static struct semaphore g_free_fragments_sema;
static struct semaphore g_free_fragments_mutex;

typedef struct bulkinfo_struct {
	PAGELIST_T		*pagelist;
	phys_addr_t		pagelist_phys;
	area_id			pagelist_area;
	vchiq_haiku_buffer	*buffer;
} BULKINFO_T;

extern VCHIQ_STATE_T *vchiq_haiku_state;
#define g_state (*vchiq_haiku_state)
int vchiq_init(void);

static int
create_pagelist(char __user *buf, size_t count, unsigned short type,
	BULKINFO_T *bi);

static void
free_pagelist(BULKINFO_T *bi, int actual);

status_t
vchiq_haiku_init(void)
{
	vchiq_core_initialize();
	return vchiq_init() == 0 ? B_OK : B_ERROR;
}

void
vchiq_haiku_doorbell(void)
{
	remote_event_pollall(&g_state);
}

void
remote_event_signal(REMOTE_EVENT_T *event)
{
	wmb();

	event->fired = 1;

	/* The test on the next line also ensures the write on the previous
	   line has completed */
	dsb(sy);
	if (event->armed) {
		/* trigger vc interrupt */
		dsb(sy);
		vchiq_haiku_ring_doorbell();
	}
}

int __init
vchiq_platform_init(VCHIQ_STATE_T *state)
{
	VCHIQ_SLOT_ZERO_T *vchiq_slot_zero;
	int frag_mem_size;
	status_t err;
	int i;

	g_cache_line_size = vchiq_haiku_cache_line_size();

	/* Allocate space for the channels in coherent memory */
	g_slot_mem_size = PAGE_ALIGN(TOTAL_SLOTS * VCHIQ_SLOT_SIZE);
	g_fragment_size = 2*g_cache_line_size;
	frag_mem_size = PAGE_ALIGN(g_fragment_size * MAX_FRAGMENTS);

	err = vchiq_haiku_dma_alloc("vchiq slots",
		g_slot_mem_size + frag_mem_size, (void **)&g_slot_mem,
		&g_slot_phys, &g_slot_area);
	if (err != B_OK) {
		vchiq_log_error(vchiq_core_log_level,
			"Unable to allocate channel memory");
		return ENOMEM;
	}

	vchiq_slot_zero = vchiq_init_slots(g_slot_mem, g_slot_mem_size);
	if (!vchiq_slot_zero) {
		err = EINVAL;
		goto failed_init_slots;
	}

	vchiq_slot_zero->platform_data[VCHIQ_PLATFORM_FRAGMENTS_OFFSET_IDX] =
		(int)vchiq_haiku_bus_address(g_slot_phys + g_slot_mem_size);
	vchiq_slot_zero->platform_data[VCHIQ_PLATFORM_FRAGMENTS_COUNT_IDX] =
		MAX_FRAGMENTS;

	g_fragments_base = (char *)(g_slot_mem + g_slot_mem_size);
	g_slot_mem_size += frag_mem_size;

	g_free_fragments = g_fragments_base;
	for (i = 0; i < (MAX_FRAGMENTS - 1); i++) {
		*(char **)&g_fragments_base[i*g_fragment_size] =
			&g_fragments_base[(i + 1)*g_fragment_size];
	}
	*(char **)&g_fragments_base[i*g_fragment_size] = NULL;
	_sema_init(&g_free_fragments_sema, MAX_FRAGMENTS);
	_sema_init(&g_free_fragments_mutex, 1);

	if (vchiq_init_state(state, vchiq_slot_zero, 0/*slave*/) !=
		VCHIQ_SUCCESS)
		return EINVAL;
			/* the core's threads run: the memory stays */

	err = vchiq_haiku_firmware_init(vchiq_haiku_bus_address(g_slot_phys));
	if (err != B_OK)
		return err;
			/* the firmware may have seen the slots: they stay */

	vchiq_log_info(vchiq_arm_log_level,
		"vchiq_init - done (slots %zx, phys %" B_PRIxPHYSADDR ")",
		(size_t)vchiq_slot_zero, g_slot_phys);

	vchiq_call_connected_callbacks();

	return 0;

failed_init_slots:
	vchiq_haiku_dma_free(g_slot_area);

	return err;
}

void __exit
vchiq_platform_exit(VCHIQ_STATE_T *state)
{
	/* The firmware keeps using the slots: they are never freed. */
}

VCHIQ_STATUS_T
vchiq_platform_init_state(VCHIQ_STATE_T *state)
{
   VCHIQ_STATUS_T status = VCHIQ_SUCCESS;
   state->platform_state = kzalloc(sizeof(VCHIQ_2835_ARM_STATE_T), GFP_KERNEL);
   ((VCHIQ_2835_ARM_STATE_T*)state->platform_state)->inited = 1;
   status = vchiq_arm_init_state(state, &((VCHIQ_2835_ARM_STATE_T*)state->platform_state)->arm_state);
   if(status != VCHIQ_SUCCESS)
   {
      ((VCHIQ_2835_ARM_STATE_T*)state->platform_state)->inited = 0;
   }
   return status;
}

VCHIQ_ARM_STATE_T*
vchiq_platform_get_arm_state(VCHIQ_STATE_T *state)
{
   if(!((VCHIQ_2835_ARM_STATE_T*)state->platform_state)->inited)
   {
      BUG();
   }
   return &((VCHIQ_2835_ARM_STATE_T*)state->platform_state)->arm_state;
}

VCHIQ_STATUS_T
vchiq_prepare_bulk_data(VCHIQ_BULK_T *bulk, VCHI_MEM_HANDLE_T memhandle,
	void *offset, int size, int dir)
{
	BULKINFO_T *bi;
	int ret;

	WARN_ON(memhandle != VCHI_MEM_HANDLE_INVALID);
	bi = kzalloc(sizeof(*bi), GFP_KERNEL);
	if (bi == NULL)
		return VCHIQ_ERROR;

	ret = create_pagelist((char __user *)offset, size,
			(dir == VCHIQ_BULK_RECEIVE)
			? PAGELIST_READ
			: PAGELIST_WRITE,
			bi);
	if (ret != 0) {
		kfree(bi);
		return VCHIQ_ERROR;
	}

	bulk->handle = memhandle;
	bulk->data = (void *)(addr_t)vchiq_haiku_bus_address(bi->pagelist_phys);

	/* Store the pagelist address in remote_data, which isn't used by the
	   slave. */
	bulk->remote_data = bi;

	return VCHIQ_SUCCESS;
}

void
vchiq_complete_bulk(VCHIQ_BULK_T *bulk)
{
	/* Also without data (aborted, or empty): the buffer is unlocked. */
	if (bulk && bulk->remote_data)
		free_pagelist((BULKINFO_T *)bulk->remote_data, bulk->actual);
}

void
vchiq_transfer_bulk(VCHIQ_BULK_T *bulk)
{
	/*
	 * This should only be called on the master (VideoCore) side, but
	 * provide an implementation to avoid the need for ifdefery.
	 */
	BUG();
}

void
vchiq_dump_platform_state(void *dump_context)
{
	char buf[80];
	int len;
	len = snprintf(buf, sizeof(buf),
		"  Platform: 2835 (VC master)");
	vchiq_dump(dump_context, buf, len + 1);
}

VCHIQ_STATUS_T
vchiq_platform_suspend(VCHIQ_STATE_T *state)
{
   return VCHIQ_ERROR;
}

VCHIQ_STATUS_T
vchiq_platform_resume(VCHIQ_STATE_T *state)
{
   return VCHIQ_SUCCESS;
}

void
vchiq_platform_paused(VCHIQ_STATE_T *state)
{
}

void
vchiq_platform_resumed(VCHIQ_STATE_T *state)
{
}

int
vchiq_platform_videocore_wanted(VCHIQ_STATE_T* state)
{
   return 1; // autosuspend not supported - videocore always wanted
}

int
vchiq_platform_use_suspend_timer(void)
{
   return 0;
}
void
vchiq_dump_platform_use_state(VCHIQ_STATE_T *state)
{
	vchiq_log_info(vchiq_arm_log_level, "Suspend timer not in use");
}
void
vchiq_platform_handle_timeout(VCHIQ_STATE_T *state)
{
	(void)state;
}
/*
 * Local functions
 */

/* There is a potential problem with partial cache lines (pages?)
** at the ends of the block when reading. If the CPU accessed anything in
** the same line (page?) then it may have pulled old data into the cache,
** obscuring the new data underneath. We can solve this by transferring the
** partial cache lines separately, and allowing the ARM to copy into the
** cached area.
*/

static int
create_pagelist(char __user *buf, size_t count, unsigned short type,
	BULKINFO_T *bi)
{
	PAGELIST_T *pagelist;
	vchiq_haiku_buffer *buffer;
	uint32_t *addrs;
	unsigned int num_pages, i;
	size_t offset;
	int pagelist_size;
	int addridx;
	status_t err;

	offset = (addr_t)buf & (PAGE_SIZE - 1);
	num_pages = (count + offset + PAGE_SIZE - 1) / PAGE_SIZE;

	bi->pagelist = NULL;

	err = vchiq_haiku_buffer_lock(buf, count, type == PAGELIST_READ,
		&buffer);
	if (err != B_OK) {
		vchiq_log_error(vchiq_core_log_level,
			"cannot lock the bulk buffer %p (%zu bytes): %s", buf, count,
			strerror(err));
		return ENOMEM;
	}

	/* The page list itself, which the VideoCore reads */
	pagelist_size = sizeof(PAGELIST_T) + (num_pages * sizeof(uint32_t));

	err = vchiq_haiku_dma_alloc("vchiq pagelist", pagelist_size,
		(void **)&pagelist, &bi->pagelist_phys, &bi->pagelist_area);
	if (err != B_OK) {
		vchiq_log_error(vchiq_core_log_level,
			"Unable to allocate pagelist memory");
		vchiq_haiku_buffer_unlock(buffer);
		return ENOMEM;
	}

	vchiq_log_trace(vchiq_arm_log_level,
		"create_pagelist - %zx (%zu bytes @%p)", (size_t)pagelist, count, buf);

	addrs = pagelist->addrs;

	pagelist->length = count;
	pagelist->type = type;
	pagelist->offset = offset;

	/* Group the pages into runs of contiguous pages: the buffer's
	   physical runs, split where a run is longer than an entry can say.
	   BCM2711 ("long bulk space"): the physical address >> 4, the run
	   length - 1 in the low 8 bits; others: the bus address, the run
	   length - 1 in the low 12 bits. */

	size_t run_ceil = vchiq_haiku_long_bulk_addresses() ? 0x100 : PAGE_SIZE;
	unsigned int pg_addr_rshift = vchiq_haiku_long_bulk_addresses() ? 4 : 0;
	addridx = 0;
	for (i = 0; i < buffer->run_count; i++) {
		phys_addr_t page = buffer->runs[i].address & ~(phys_addr_t)(PAGE_SIZE - 1);
		phys_addr_t end = buffer->runs[i].address + buffer->runs[i].size;
		size_t pages = (end - page + PAGE_SIZE - 1) / PAGE_SIZE;

		while (pages > 0) {
			size_t run = pages < run_ceil ? pages : run_ceil;
			uint64_t base = vchiq_haiku_long_bulk_addresses()
				? (uint64_t)page : vchiq_haiku_bus_address(page);

			addrs[addridx++] = (uint32_t)(((base >> pg_addr_rshift)
				& ~(uint64_t)(run_ceil - 1)) + (run - 1));
			page += run * PAGE_SIZE;
			pages -= run;
		}
	}

	/* Partial cache lines (fragments) require special measures */
	if ((type == PAGELIST_READ) &&
		((pagelist->offset & (g_cache_line_size - 1)) ||
		((pagelist->offset + pagelist->length) &
		(g_cache_line_size - 1)))) {
		char *fragments;

		if (down_interruptible(&g_free_fragments_sema) != 0) {
			vchiq_haiku_dma_free(bi->pagelist_area);
			vchiq_haiku_buffer_unlock(buffer);
			return EINTR;
		}

		WARN_ON(g_free_fragments == NULL);

		down(&g_free_fragments_mutex);
		fragments = g_free_fragments;
		WARN_ON(fragments == NULL);
		g_free_fragments = *(char **) g_free_fragments;
		up(&g_free_fragments_mutex);
		pagelist->type = PAGELIST_READ_WITH_FRAGMENTS
		     + (fragments - g_fragments_base)/g_fragment_size;
	}

	/* The buffer's lines: written back before the VideoCore reads it, and
	   also dropped before it writes it (no dirty line may be evicted over
	   its data) */
	if (type == PAGELIST_READ)
		vchiq_haiku_buffer_flush(buffer, 0, count);
	else
		vchiq_haiku_buffer_clean(buffer, 0, count);

	bi->pagelist = pagelist;
	bi->buffer = buffer;

	return 0;
}

static void
free_pagelist(BULKINFO_T *bi, int actual)
{
	PAGELIST_T *pagelist;
	vchiq_haiku_buffer *buffer;

	pagelist = bi->pagelist;
	buffer = bi->buffer;

	vchiq_log_trace(vchiq_arm_log_level,
	    "free_pagelist - %zx, %d (%lu bytes @%p)",
	    (size_t)pagelist, (int)actual, (unsigned long)pagelist->length,
	    buffer->address);

	/* An aborted transfer has no data */
	if (actual < 0 || (size_t)actual > pagelist->length)
		actual = 0;

	/*
	 * Even if the user keeps their end of the bargain -- do NOT touch the
	 * buffers sent to VC -- the ARM may speculatively load cache lines of
	 * the buffer while the VC writes it, so they are dropped again.
	 */
	if (pagelist->type >= PAGELIST_READ && actual > 0)
		vchiq_haiku_buffer_flush(buffer, 0, actual);

	/* Deal with any partial cache lines (fragments) */
	if (pagelist->type >= PAGELIST_READ_WITH_FRAGMENTS) {
		char *fragments = g_fragments_base +
			(pagelist->type - PAGELIST_READ_WITH_FRAGMENTS)*g_fragment_size;
		int head_bytes, tail_bytes;
		head_bytes = (g_cache_line_size - pagelist->offset) &
			(g_cache_line_size - 1);
		tail_bytes = (pagelist->offset + actual) &
			(g_cache_line_size - 1);

		if ((actual >= 0) && (head_bytes != 0)) {
			if (head_bytes > actual)
				head_bytes = actual;

			vchiq_haiku_buffer_write(buffer, 0, fragments, head_bytes);
		}

		if ((actual >= 0) && (head_bytes < actual) &&
			(tail_bytes != 0)) {
			vchiq_haiku_buffer_write(buffer, actual - tail_bytes,
				fragments + g_cache_line_size, tail_bytes);
		}

		down(&g_free_fragments_mutex);
		*(char **) fragments = g_free_fragments;
		g_free_fragments = fragments;
		up(&g_free_fragments_mutex);
		up(&g_free_fragments_sema);
	}

	vchiq_haiku_buffer_unlock(buffer);
	vchiq_haiku_dma_free(bi->pagelist_area);
	kfree(bi);
}
