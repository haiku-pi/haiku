/*
 * Copyright 2019-2026 Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */
#include <OS.h>

#include <arch/arm64/arch_cpu_type.h>
#include <arch/system_info.h>
#include <arch_cpu.h>
#include <boot/kernel_args.h>

#include "cpu.h"
#include <smp.h>


static cpu_vendor sCPUVendor = B_CPU_VENDOR_UNKNOWN;


void
arch_fill_topology_node(cpu_topology_node_info* node, int32 cpu)
{
	switch (node->type) {
		case B_TOPOLOGY_ROOT:
			node->data.root.platform = B_CPU_ARM_64;
			break;
		case B_TOPOLOGY_PACKAGE:
			node->data.package.vendor = sCPUVendor;
			node->data.package.cache_line_size = CACHE_LINE_SIZE;
			break;
		case B_TOPOLOGY_CORE:
			node->data.core.model = static_cast<uint32>(gCPU[cpu].arch.midr);
			// TODO: SMBIOS Type 4 or the device tree have the nominal clock on some
			// devices; until then the measured one (arch_get_frequency())
			arch_get_frequency(&node->data.core.default_frequency, cpu);
			break;
		default:
			break;
	}
}


status_t
arch_system_info_init(struct kernel_args *args)
{
	cpu_ent* cpu = get_cpu_struct();

	// TODO: It *is* possible for the package vendors to be heterogeneous,
	//       (Tegra X2 has NVIDIA and ARM-designed cores on the same package)
	//       but as vendor field is set for the whole package currently,
	//       it would have to be reworked for it to be supported.
	switch (CPU_IMPL(cpu->arch.midr)) {
		case CPU_IMPL_ARM:
			sCPUVendor = B_CPU_VENDOR_ARM;
			break;
		case CPU_IMPL_BROADCOM:
			sCPUVendor = B_CPU_VENDOR_BROADCOM;
			break;
		case CPU_IMPL_CAVIUM:
			sCPUVendor = B_CPU_VENDOR_CAVIUM;
			break;
		case CPU_IMPL_DEC:
			sCPUVendor = B_CPU_VENDOR_DEC;
			break;
		case CPU_IMPL_FUJITSU:
			sCPUVendor = B_CPU_VENDOR_FUJITSU;
			break;
		case CPU_IMPL_HISILICON:
			sCPUVendor = B_CPU_VENDOR_HISILICON;
			break;
		case CPU_IMPL_INFINEON:
			sCPUVendor = B_CPU_VENDOR_INFINEON;
			break;
		case CPU_IMPL_FREESCALE:
			sCPUVendor = B_CPU_VENDOR_FREESCALE;
			break;
		case CPU_IMPL_NVIDIA:
			sCPUVendor = B_CPU_VENDOR_NVIDIA;
			break;
		case CPU_IMPL_APM:
			sCPUVendor = B_CPU_VENDOR_APM;
			break;
		case CPU_IMPL_QUALCOMM:
			sCPUVendor = B_CPU_VENDOR_QUALCOMM;
			break;
		case CPU_IMPL_MARVELL:
			sCPUVendor = B_CPU_VENDOR_MARVELL;
			break;
		case CPU_IMPL_APPLE:
			sCPUVendor = B_CPU_VENDOR_APPLE;
			break;
		case CPU_IMPL_INTEL:
			sCPUVendor = B_CPU_VENDOR_INTEL;
			break;
		case CPU_IMPL_AMPERE:
			sCPUVendor = B_CPU_VENDOR_AMPERE;
			break;
		case CPU_IMPL_MICROSOFT:
			sCPUVendor = B_CPU_VENDOR_MICROSOFT;
			break;
		default:
			break;
	}

	return B_OK;
}


/*!	The current CPU's clock, measured: its PMU cycle counter against the
	generic timer over 100 us (the cycle counter is architectural; under QEMU
	it runs at a fixed rate). The result is kept for a second, as monitors
	ask often.
*/
static void
measure_frequency(void* _frequency, int cpu)
{
	static uint64 sFrequency[SMP_MAX_CPUS];
	static bigtime_t sMeasured[SMP_MAX_CPUS];
	uint64* frequency = (uint64*)_frequency;

	const bigtime_t now = system_time();
	if (sMeasured[cpu] != 0 && now - sMeasured[cpu] < 1000000) {
		*frequency = sFrequency[cpu];
		return;
	}

	// cycle counter on, counting every cycle (PMCR_EL0.E, D clear)
	const uint64 pmcr = READ_SPECIALREG(PMCR_EL0);
	WRITE_SPECIALREG(PMCR_EL0, (pmcr | 1) & ~(uint64)(1 << 3));
	WRITE_SPECIALREG(PMCNTENSET_EL0, 1ul << 31);
	arm64_isb();

	const uint64 timerFrequency = READ_SPECIALREG(CNTFRQ_EL0);
	const cpu_status state = disable_interrupts();
	const uint64 start = READ_SPECIALREG(CNTVCT_EL0);
	const uint64 startCycles = READ_SPECIALREG(PMCCNTR_EL0);
	const uint64 end = start + timerFrequency / 10000;
	uint64 ticks;
	while ((ticks = READ_SPECIALREG(CNTVCT_EL0)) < end)
		;
	const uint64 cycles = READ_SPECIALREG(PMCCNTR_EL0) - startCycles;
	restore_interrupts(state);

	sFrequency[cpu] = cycles * timerFrequency / (ticks - start);
	sMeasured[cpu] = now;
	*frequency = sFrequency[cpu];
}


status_t
arch_get_frequency(uint64 *frequency, int32 cpu)
{
	// The options vary wildly in availability: ARMv8.4 activity monitors
	// (AMEVCNTR0), the PMU cycle counter, ACPI CPPC feedback counters, Apple's
	// p-state registers. The PMU cycle counter is there on every core.
	*frequency = 0;
	if (cpu == smp_get_current_cpu())
		measure_frequency(frequency, cpu);
	else
		call_single_cpu_sync(cpu, &measure_frequency, frequency);
	return B_OK;
}
