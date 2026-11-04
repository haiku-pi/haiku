/*
 * Copyright 2019 Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */


#include <KernelExport.h>

#include <arch/cpu.h>
#include <arch/user_debugger.h>
#include <boot/kernel_args.h>
#include <commpage.h>
#include <elf.h>


extern "C" void _exception_vectors(void);


static uint32 sPsciConduit = 0;
static arm64_shutdown_hook sShutdownHook = NULL;


status_t
arch_cpu_preboot_init_percpu(kernel_args *args, int curr_cpu)
{
	WRITE_SPECIALREG(VBAR_EL1, _exception_vectors);
	return B_OK;
}


status_t
arch_cpu_init_percpu(kernel_args *args, int curr_cpu)
{
	uint64_t tcr = READ_SPECIALREG(TCR_EL1);
	uint64_t mmfr1 = READ_SPECIALREG(ID_AA64MMFR1_EL1);

	uint64_t hafdbs = ID_AA64MMFR1_HAFDBS(mmfr1);
	if (hafdbs == ID_AA64MMFR1_HAFDBS_AF) {
		tcr |= (1UL << 39);
	}
	if (hafdbs == ID_AA64MMFR1_HAFDBS_AF_DBS) {
		tcr |= (1UL << 40) | (1UL << 39);
	}

	WRITE_SPECIALREG(TCR_EL1, tcr);

	gCPU[curr_cpu].arch.mpidr = READ_SPECIALREG(MPIDR_EL1);
	gCPU[curr_cpu].arch.midr = READ_SPECIALREG(MIDR_EL1);

	arm64_init_user_debug_percpu(curr_cpu);

	return 0;
}


status_t
arch_cpu_init(kernel_args *args)
{
	sPsciConduit = args->arch_args.psci_conduit;

	for (uint32 i = 0; i < args->num_cpus; i++) {
		cpu_ent* cpu = &gCPU[i];

		cpu->topology_id[CPU_TOPOLOGY_PACKAGE] = 0;
		cpu->topology_id[CPU_TOPOLOGY_CORE] = i;
		cpu->topology_id[CPU_TOPOLOGY_SMT] = 0;
	}
	return B_OK;
}


status_t
arch_cpu_init_post_vm(kernel_args *args)
{
	return B_OK;
}


status_t
arch_cpu_init_post_modules(kernel_args *args)
{
	return B_OK;
}


void
arm64_set_shutdown_hook(arm64_shutdown_hook hook)
{
	sShutdownHook = hook;
}


status_t
arch_cpu_shutdown(bool reboot)
{
	// PSCI 0.2+: SYSTEM_RESET / SYSTEM_OFF. These don't return on success;
	// if one fails, the firmware may have changed x1-x17 (SMCCC 1.0).
	uint64 function = reboot ? 0x84000009UL : 0x84000008UL;
	register uint64 x0 asm("x0") = function;
	switch (sPsciConduit) {
		case 1:
			asm volatile("smc #0" : "+r" (x0) : : "x1", "x2", "x3", "x4",
				"x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
				"x14", "x15", "x16", "x17", "memory");
			break;
		case 2:
			asm volatile("hvc #0" : "+r" (x0) : : "x1", "x2", "x3", "x4",
				"x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
				"x14", "x15", "x16", "x17", "memory");
			break;
		default:
			break;
	}

	// No PSCI (the Raspberry Pi 4 under U-Boot: its device tree has no "psci"
	// node, the firmware starts the other CPUs from a spin table), or it
	// failed: a platform driver may know how.
	arm64_shutdown_hook hook = sShutdownHook;
	if (hook != NULL)
		return hook(reboot);

	return B_ERROR;
}


void
arch_cpu_sync_icache(void *address, size_t len)
{
	uint64_t ctr_el0 = 0;
	asm volatile ("mrs\t%0, ctr_el0":"=r" (ctr_el0));

	// CTR_EL0: IminLine (bits 3:0) and DminLine (bits 19:16) are log2 of the
	// smallest line size in words; IDC (bit 28) and DIC (bit 29) say that the
	// data cache clean and the instruction cache invalidation are not needed.
	uint64_t icache_line_size = 4 << (ctr_el0 & 0xF);
	uint64_t dcache_line_size = 4 << ((ctr_el0 >> 16) & 0xF);
	uint64_t addr = (uint64_t)address;
	uint64_t end = addr + len;

	if ((ctr_el0 & (1 << 28)) == 0) {
		for (uint64_t address_dcache = ROUNDDOWN(addr, dcache_line_size);
			address_dcache < end; address_dcache += dcache_line_size) {
			asm volatile ("dc cvau, %0" : : "r"(address_dcache) : "memory");
		}
	}

	asm("dsb ish");

	if ((ctr_el0 & (1 << 29)) == 0) {
		for (uint64_t address_icache = ROUNDDOWN(addr, icache_line_size);
			address_icache < end; address_icache += icache_line_size) {
			asm volatile ("ic ivau, %0" : : "r"(address_icache) : "memory");
		}
		asm("dsb ish");
	}
	asm("isb");
}


void
arch_cpu_invalidate_tlb_range(intptr_t, addr_t start, addr_t end)
{
	arch_cpu_global_tlb_invalidate();
}


void
arch_cpu_invalidate_tlb_list(intptr_t, addr_t pages[], int num_pages)
{
	arch_cpu_global_tlb_invalidate();
}


void
arch_cpu_global_tlb_invalidate()
{
	asm(
		"dsb ishst\n"
		"tlbi vmalle1\n"
		"dsb ish\n"
		"isb\n"
	);
}


void
arch_cpu_user_tlb_invalidate(intptr_t)
{
	arch_cpu_global_tlb_invalidate();
}
