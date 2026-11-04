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
	unsigned long module = ti->ti_module;

	if ((module & TLS_STATIC_MODULE) != 0) {
		// An image loaded at program start: its TLS is in this thread's
		// static block, at the offset the runtime loader gave as the module.
		// Such a block is never stale.
		char* block = (char*)get_tls()[TLS_STATIC_TLS_SLOT];
		if (__builtin_expect(block != NULL, 1))
			return block + (module & ~TLS_STATIC_MODULE) + ti->ti_offset;
	} else {
		// Without calling into the runtime loader if this thread's dynamic
		// thread vector is up to date (no TLS image unloaded since it was
		// last checked) and already holds the block. The layout is that of
		// DynamicThreadVector in runtime_loader/elf_tls.cpp: entry 0 points
		// to the vector's generation counter and size, entry dso + 1 to the
		// block.
		void** vector = (void**)get_tls()[TLS_DYNAMIC_THREAD_VECTOR];
		const unsigned* generation = __gRuntimeLoader->tls_generation;
		if (vector != NULL && generation != NULL) {
			const unsigned* vectorGeneration = (const unsigned*)vector[0];
			if (vectorGeneration[0] == *generation
				&& module < vectorGeneration[1]) {
				char* block = (char*)vector[module + 1];
				if (block != NULL)
					return block + TLS_DTV_OFFSET + ti->ti_offset;
			}
		}
	}

	// the first access of this thread, or a stale vector
	return __gRuntimeLoader->get_tls_address(module, ti->ti_offset);
}
