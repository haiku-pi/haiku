/*
 * Copyright 2021-2026 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

#include <debug.h>
#include <interrupts.h>
#include <interrupt_controller.h>
#include <kernel.h>
#include <vm/vm.h>
#include <smp.h>
#include <thread.h>
#include <arch_cpu.h>
#include <KernelExport.h>

#include "arch_int_gicv2.h"
#include "gicv2_regs.h"


#define ICI_IRQ 0

#define GICC_REG_RPR	5
#define GICC_REG_HPPIR	6


static volatile uint32_t* sGicdRegs;
static volatile uint32_t* sGiccRegs;


// TODO: bring-up diagnostic for the Raspberry Pi 4's boot freeze (all CPUs
// idle, no timer firing, until any interrupt arrives): CPUs whose bit is set
// print their state at their next interrupt, before acknowledging it.
int32 gGICv2FreezeProbe;
extern bigtime_t timer_debug_next_event(int32 cpu);


static void
freeze_probe(int32 cpu)
{
	const uint32 rpr = sGiccRegs[GICC_REG_RPR];
	const uint32 hppir = sGiccRegs[GICC_REG_HPPIR] & 0x3ff;
	const uint32 pending = sGicdRegs[GICD_REG_ISPENDR];
	const uint32 active = sGicdRegs[GICD_REG_ISACTIVER];
	const bigtime_t now = system_time();
	const bigtime_t next = timer_debug_next_event(cpu);
#ifdef __aarch64__
	const uint64 control = READ_SPECIALREG(CNTV_CTL_EL0);
	const int64 ticksLeft = (int64)(READ_SPECIALREG(CNTV_CVAL_EL0)
		- READ_SPECIALREG(CNTVCT_EL0));
#else
	const uint64 control = 0;
	const int64 ticksLeft = 0;
#endif
	Thread* thread = thread_get_current_thread();
	dprintf("freeze probe: CPU %" B_PRId32 " at %" B_PRId64 " us, thread %"
		B_PRId32 " %s, CNTV_CTL %#" B_PRIx64 " fires in %" B_PRId64
		" ticks, next timer event in %" B_PRId64 " us, GIC SGI/PPI pending %#"
		B_PRIx32 " active %#" B_PRIx32 " running priority %#" B_PRIx32
		" highest pending %" B_PRIu32 "\n", cpu, now,
		thread != NULL ? thread->id : -1, thread != NULL ? thread->name : "-",
		control, ticksLeft, next >= 0 ? next - now : -1, pending, active, rpr,
		hppir);
}


/*!	The current CPU's view of the GIC: its banked SGI/PPI state, the
	priority it runs at (0xff = none active) and its highest pending
	interrupt.
*/
static int
dump_gic(int argc, char** argv)
{
	kprintf("CPU %" B_PRId32 ": SGI/PPI enabled %#" B_PRIx32 " pending %#" B_PRIx32
		" active %#" B_PRIx32 ", running priority %#" B_PRIx32
		", highest pending %" B_PRIu32 ", CPU interface control %#" B_PRIx32 "\n",
		smp_get_current_cpu(), sGicdRegs[GICD_REG_ISENABLER],
		sGicdRegs[GICD_REG_ISPENDR], sGicdRegs[GICD_REG_ISACTIVER],
		sGiccRegs[GICC_REG_RPR], sGiccRegs[GICC_REG_HPPIR] & 0x3ff,
		sGiccRegs[GICC_REG_CTLR]);
	kprintf("distributor control %#" B_PRIx32 ", SPI pending:", sGicdRegs[GICD_REG_CTLR]);
	for (int32 i = 1; i < 8; i++)
		kprintf(" %#" B_PRIx32, sGicdRegs[GICD_REG_ISPENDR + i]);
	kprintf(", SPI active:");
	for (int32 i = 1; i < 8; i++)
		kprintf(" %#" B_PRIx32, sGicdRegs[GICD_REG_ISACTIVER + i]);
	kprintf("\n");
	return 0;
}


GICv2InterruptController::GICv2InterruptController(phys_addr_t gicd_addr, phys_addr_t gicc_addr)
: InterruptController()
{
	// map memory area for distributor (GICD) and CPU interface (GICC)

	area_id gicd_area = vm_map_physical_memory(B_SYSTEM_TEAM, "intc-gicv2-gicd",
		(void**)&fGicdRegs, B_ANY_KERNEL_ADDRESS, GICD_REG_SIZE,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
		gicd_addr ? gicd_addr : GICD_REG_START, false);
	if (gicd_area < 0) {
		panic("not able to map the memory area for gicd\n");
	}

	area_id gicc_area = vm_map_physical_memory(B_SYSTEM_TEAM, "intc-gicv2-gicc",
		(void**)&fGiccRegs, B_ANY_KERNEL_ADDRESS, GICC_REG_SIZE,
		B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA,
		gicc_addr ? gicc_addr : GICC_REG_START, false);
	if (gicc_area < 0) {
		panic("not able to map the memory area for gicc\n");
	}

	// Configure the GIC. Refer to chapter 4, Programmers' Model,
	// in ARM GIC Architecture Spec v2.0
	// https://support.arm.com/documentation/ihi0048/latest/

	sGicdRegs = fGicdRegs;
	sGiccRegs = fGiccRegs;
	add_debugger_command("gic", &dump_gic,
		"Dumps the current CPU's view of the GICv2 interrupt controller");

	// disable GICC and GICD before configuring
	fGiccRegs[GICC_REG_CTLR] = 0x00;
	fGicdRegs[GICD_REG_CTLR] = 0x00;

	// how many registers cover all IRQs?
	uint32 typer = fGicdRegs[GICD_REG_TYPER];
	uint32 lineRegs = (typer & 0x1f) + 1;
	reserve_io_interrupt_vectors(32 * lineRegs, 0, INTERRUPT_TYPE_IRQ);

	for (uint32 i = 1; i < lineRegs; i++) { // 1 bit per IRQ
		// mask and clear state before enabling
		fGicdRegs[GICD_REG_ICENABLER + i] = 0xffffffff;
		fGicdRegs[GICD_REG_ICPENDR + i] = 0xffffffff;
	}

	// route SPIs to the boot CPU
	// TODO: don't assume boot CPU is CPU 0
	for (uint32 i = 8; i < lineRegs * 8; i++) // 8 bits per IRQ
		fGicdRegs[GICD_REG_ITARGETSR + i] = 0x01010101;

	// all SPIs level-sensitive
	for (uint32 i = 2; i < lineRegs * 2; i++) // 2 bits per IRQ
		fGicdRegs[GICD_REG_ICFGR + i] = 0x0;

	// GICD configured, enable (GICC follows per-CPU)
	fGicdRegs[GICD_REG_CTLR] = 0x03;

	// until each CPU reads its own (PerCpuInit())
	for (int32 i = 0; i < SMP_MAX_CPUS; i++)
		fCpuInterfaceMask[i] = 1 << (i % 8);

	// PerCpuInit is not called separately for the boot CPU
	PerCpuInit();
}


status_t
GICv2InterruptController::PerCpuInit()
{
	// The banked target register of the SGIs reads as this CPU's own
	// interface bit: SGIs must be addressed with it (as Linux's gic_cpu_map)
	const int32 cpu = smp_get_current_cpu();
	const uint8 mask = ((volatile uint8*)&fGicdRegs[GICD_REG_ITARGETSR])[0];
	fCpuInterfaceMask[cpu] = mask;
	if (mask != (1 << cpu))
		dprintf("GICv2: CPU %" B_PRId32 " is GIC CPU interface mask %#x\n", cpu, mask);

	// configure IRQs 0-31 (SGIs/PPIs) which are banked per CPU interface
	fGicdRegs[GICD_REG_ICENABLER] = 0xffffffff;
	fGicdRegs[GICD_REG_ICPENDR] = 0xffffffff;

	// disable priority masking
	fGiccRegs[GICC_REG_PMR] = 0xff;
	// disable interrupt preemption
	fGiccRegs[GICC_REG_BPR] = 0x07;

	// enable GICC
	fGiccRegs[GICC_REG_CTLR] = 0x01;

	EnableInterrupt(ICI_IRQ);

	return B_OK;
}


void
GICv2InterruptController::EnableInterrupt(int32 irq)
{
	uint32_t ena_reg = GICD_REG_ISENABLER + irq / 32;
	uint32_t ena_val = 1 << (irq % 32);
	fGicdRegs[ena_reg] = ena_val;

	uint32_t prio_reg = GICD_REG_IPRIORITYR + irq / 4;
	uint32_t prio_val = fGicdRegs[prio_reg];
	prio_val |= 0x80 << (irq % 4 * 8);
	fGicdRegs[prio_reg] = prio_val;
}


void
GICv2InterruptController::DisableInterrupt(int32 irq)
{
	fGicdRegs[GICD_REG_ICENABLER + irq / 32] = 1 << (irq % 32);
}


void
GICv2InterruptController::RouteInterrupt(int32 irq, uint32 cpuMask)
{
	// ITARGETSR: one byte per interrupt, only meaningful for SPIs
	if (irq < 32)
		return;
	volatile uint8* targets = (volatile uint8*)&fGicdRegs[GICD_REG_ITARGETSR];
	targets[irq] = cpuMask & 0xff;
}


void GICv2InterruptController::HandleInterrupt()
{
	const int32 cpu = smp_get_current_cpu();
	if ((atomic_get(&gGICv2FreezeProbe) & (1 << cpu)) != 0) {
		atomic_and(&gGICv2FreezeProbe, ~(1 << cpu));
		freeze_probe(cpu);
	}

	uint32_t iar = fGiccRegs[GICC_REG_IAR];
	uint32_t irqnr = iar & 0x3FF;

	if ((irqnr == 1022) || (irqnr == 1023)) {
		dprintf("spurious interrupt (%d)\n", irqnr);
	} else if (irqnr == ICI_IRQ) {
		// The sender's message must be read after the acknowledge (pairs with
		// the write barrier in SendMulticastIci(); as Linux's gic_handle_irq())
		memory_read_barrier();
		smp_intercpu_interrupt_handler(smp_get_current_cpu());
	} else {
		io_interrupt_handler(irqnr, B_LEVEL_TRIGGERED);
	}

	fGiccRegs[GICC_REG_EOIR] = iar;
}


void GICv2InterruptController::SendMulticastIci(CPUSet& cpuSet)
{
	// The message for the target CPUs is in memory: make it visible before
	// the interrupt can reach them (a store to the GIC's device memory is not
	// ordered after stores to normal memory).
	memory_write_barrier();
	uint32 targets = 0;
	for (int32 cpu = 0; cpu < smp_get_num_cpus(); cpu++) {
		if (cpuSet.GetBit(cpu))
			targets |= fCpuInterfaceMask[cpu];
	}
	fGicdRegs[GICD_REG_SGIR] = (targets & 0xff) << 16;
}


void GICv2InterruptController::SendBroadcastIci()
{
	memory_write_barrier();
	fGicdRegs[GICD_REG_SGIR] = (0b01 << 24);
}
