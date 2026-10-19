/*
 * Copyright 2018, Jaroslaw Pelczar <jarek@jpelczar.com>
 * Copyright 2026, Haiku, Inc.
 * Distributed under the terms of the MIT License.
 */
#ifndef _KERNEL_ARCH_ARM64_ARCH_CPU_H_
#define _KERNEL_ARCH_ARM64_ARCH_CPU_H_


#define CPU_MAX_CACHE_LEVEL 	8
#define CACHE_LINE_SIZE 		64

// TODO: These will require a real implementation when PAN is enabled
#define arch_cpu_enable_user_access()
#define arch_cpu_disable_user_access()

#include <kernel/arch/arm64/arm_registers.h>

#ifndef _ASSEMBLER

#include <arch/arm64/arch_thread_types.h>
#include <kernel.h>

// Options specifying barrier limitations.
// As it has to be specified for dsb (data synchronization barrier) and dmb (data memory barrier)
// these macros act as a whitelist.
// isb (instruction synchronization barrier) supports only sy, and it's the default.
// See for details:
// https://support.arm.com/documentation/dui0801/l/A64-General-Instructions/DSB--A64-
#define ARM64_BARRIER_OPT_sy	"sy" // full system, read-write (used for memory_full_barrier())
#define ARM64_BARRIER_OPT_st	"st" // full system, write only
#define ARM64_BARRIER_OPT_ld	"ld" // full system, read in group A, read-write in group B
#define ARM64_BARRIER_OPT_ish	"ish" // inner sharable, read-write
#define ARM64_BARRIER_OPT_ishst	"ishst" // inner sharable, write only (used for memory_write_barrier())
#define ARM64_BARRIER_OPT_ishld	"ishld" // inner sharable, read in group A, read-write in group B (used for memory_read_barrier())
#define ARM64_BARRIER_OPT_nsh	"nsh" // non-sharable, read-write
#define ARM64_BARRIER_OPT_nshst	"nshst" // non-sharable, write only
#define ARM64_BARRIER_OPT_nshld	"nshld" // non-sharable, read in group A, read-write in group B
#define ARM64_BARRIER_OPT_osh	"osh" // outer sharable, read-write
#define ARM64_BARRIER_OPT_oshst	"oshst" // outer sharable, write only
#define ARM64_BARRIER_OPT_oshld	"oshld" // outer sharable, read in group A, read-write in group B

// Barriers
#define arm64_dsb(limit)	__asm__ __volatile__("dsb " ARM64_BARRIER_OPT_##limit : : : "memory")
#define arm64_dmb(limit)	__asm__ __volatile__("dmb " ARM64_BARRIER_OPT_##limit : : : "memory")
#define arm64_isb()			__asm__ __volatile__("isb" : : : "memory")

#define arm64_sev()		__asm__ __volatile__("sev" : : : "memory")
#define arm64_wfe()		__asm__ __volatile__("wfe" : : : "memory")
#define arm64_nop()		__asm__ __volatile__("nop" : : : "memory")
#define arm64_wfi()		__asm__ __volatile__("wfi" : : : "memory")
#define arm64_yield()	__asm__ __volatile__("yield" : : : "memory")

/* Extract CPU affinity levels 0-3 */
#define	CPU_AFF0(mpidr)	(u_int)(((mpidr) >> 0) & 0xff)
#define	CPU_AFF1(mpidr)	(u_int)(((mpidr) >> 8) & 0xff)
#define	CPU_AFF2(mpidr)	(u_int)(((mpidr) >> 16) & 0xff)
#define	CPU_AFF3(mpidr)	(u_int)(((mpidr) >> 32) & 0xff)
#define	CPU_AFF0_MASK	0xffUL
#define	CPU_AFF1_MASK	0xff00UL
#define	CPU_AFF2_MASK	0xff0000UL
#define	CPU_AFF3_MASK	0xff00000000UL
#define	CPU_AFF_MASK	(CPU_AFF0_MASK | CPU_AFF1_MASK | \
    CPU_AFF2_MASK| CPU_AFF3_MASK)	/* Mask affinity fields in MPIDR_EL1 */

static inline uint64 arm64_get_cyclecount(void)
{
	return READ_SPECIALREG(cntvct_el0);
}

#define	ADDRESS_TRANSLATE_FUNC(stage)					\
static inline uint64									\
arm64_address_translate_ ##stage (uint64 addr)		\
{														\
	uint64 ret;											\
														\
	__asm __volatile(									\
	    "at " __ARMREG_STRING(stage) ", %1 \n"					\
	    "mrs %0, par_el1" : "=r"(ret) : "r"(addr));		\
														\
	return (ret);										\
}


ADDRESS_TRANSLATE_FUNC(s1e0r)
ADDRESS_TRANSLATE_FUNC(s1e0w)
ADDRESS_TRANSLATE_FUNC(s1e1r)
ADDRESS_TRANSLATE_FUNC(s1e1w)


/*	Data cache maintenance for DMA by devices that do not snoop the CPU caches
	(on the Raspberry Pi 4: everything on the VideoCore bus, like the SD
	controller and the mailbox). The ranges are virtual addresses; a physical
	range can be reached through the physical map, KERNEL_PMAP_BASE + address.
*/
static inline size_t
arm64_dcache_line_size(void)
{
	return 4 << ((READ_SPECIALREG(ctr_el0) >> 16) & 0xf);
}


#define DCACHE_RANGE_FUNC(name, op)										\
static inline void														\
name(addr_t start, size_t size)											\
{																		\
	const size_t lineSize = arm64_dcache_line_size();					\
	for (addr_t address = start & ~(addr_t)(lineSize - 1);				\
			address < start + size; address += lineSize) {				\
		__asm __volatile("dc " #op ", %0" : : "r" (address) : "memory");	\
	}																	\
	__asm __volatile("dsb sy" : : : "memory");							\
}

// Write dirty lines back to memory: before a device reads the range.
DCACHE_RANGE_FUNC(arm64_dcache_clean_range, cvac)
// Write back and discard the lines: before a device writes the range (so no
// dirty line is evicted over the new data), and again after it did (to drop
// lines fetched speculatively in the meantime).
DCACHE_RANGE_FUNC(arm64_dcache_flush_range, civac)


#ifdef __cplusplus
namespace BKernel {
	struct Thread;
}  // namespace BKernel


typedef struct arch_cpu_info {
	uint64						mpidr;	// multiprocessor affinity register
	uint64						midr;	// main ID register
	BKernel::Thread*			last_vfp_user;
} arch_cpu_info;
#endif


#ifdef __cplusplus
extern "C" {
#endif


static inline void arch_cpu_pause(void)
{
	arm64_yield();
}


/*!	Waits while *variable != test, for cpu_wait(): sleeps until another CPU
	writes the variable's cache line (the exclusive monitor's event, as Linux's
	__cmpwait()), or the generic timer's event stream wakes it (every ~150 us,
	arm64_timer_per_cpu_init()). Unlike a polling loop, this leaves the line
	alone: the polling loads of a waiter could make another CPU's
	load/store-exclusive pair on the same line (atomics without LSE, as on the
	Cortex-A72) fail for as long as they kept coming, with both CPUs' interrupts
	disabled.
*/
static inline void arch_cpu_wait(int32* variable, int32 test)
{
	uint32 value;
	__asm__ __volatile__(
		"	sevl\n"
		"	wfe\n"
		"	ldxr	%w0, %1\n"
		"	cmp		%w0, %w2\n"
		"	b.eq	1f\n"
		"	wfe\n"
		"1:\n"
		: "=&r" (value) : "Q" (*variable), "r" (test) : "memory", "cc");
}


static inline void arch_cpu_idle(void)
{
	// Complete the outstanding writes first (as Linux's cpu_do_idle): the
	// end-of-interrupt write to the GIC can otherwise wait in a buffer while
	// the CPU sleeps, and the GIC delivers it nothing else until then
	arm64_dsb(sy);
	arm64_wfi();
}


extern addr_t arm64_get_fp(void);


#ifdef __cplusplus
}
#endif

#endif


#endif /* _KERNEL_ARCH_ARM64_ARCH_CPU_H_ */
