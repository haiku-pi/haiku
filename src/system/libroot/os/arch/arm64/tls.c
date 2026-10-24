/*
 * Copyright 2019-2022, Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */

#include <arch_elf.h>
#include <runtime_loader/runtime_loader.h>

#include <support/TLS.h>
#include <tls.h>


struct tls_index {
	unsigned long ti_module;
	unsigned long ti_offset;
};


static int32 gNextSlot = TLS_FIRST_FREE_SLOT;

void* __tls_get_addr(struct tls_index* ti);


static inline void**
get_tls()
{
	void **tls;
	asm volatile("MRS %0, tpidr_el0" : "=r" (tls));
	return tls;
}


int32
tls_allocate(void)
{
	int32 next = atomic_add(&gNextSlot, 1);
	if (next >= TLS_MAX_KEYS)
		return B_NO_MEMORY;

	return next;
}


void *
tls_get(int32 index)
{
	return get_tls()[index];
}


void **
tls_address(int32 index)
{
	return get_tls() + index;
}


void
tls_set(int32 index, void *value)
{
	get_tls()[index] = value;
}


void*
__tls_get_addr(struct tls_index* ti)
{
	// Fast path, without calling into the runtime loader: this thread's
	// dynamic thread vector is up to date (no TLS image unloaded since it was
	// last checked) and already holds the block. The layout is that of
	// DynamicThreadVector in runtime_loader/elf_tls.cpp: entry 0 points to
	// the vector's generation counter and size, entry dso + 1 to the block.
	void** vector = (void**)get_tls()[TLS_DYNAMIC_THREAD_VECTOR];
	const unsigned* generation = __gRuntimeLoader->tls_generation;
	if (vector != NULL && generation != NULL) {
		const unsigned* vectorGeneration = (const unsigned*)vector[0];
		if (vectorGeneration[0] == *generation
			&& ti->ti_module < vectorGeneration[1]) {
			char* block = (char*)vector[ti->ti_module + 1];
			if (block != NULL)
				return block + TLS_DTV_OFFSET + ti->ti_offset;
		}
	}

	return __gRuntimeLoader->get_tls_address(ti->ti_module, ti->ti_offset);
}
