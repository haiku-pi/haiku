/*
 * Copyright 2019-2026 Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */
#include <frame_buffer_console.h>
#include <smp.h>
#include <boot/stage2.h>
#include <debug.h>
#include <interrupts.h>

#include <timer.h>
#include <arch/timer.h>

#include "arch_timer.h"
#include "soc.h"

#include <smp.h>


static uint64 sTimerTicksUS;
static bigtime_t sTimerMaxInterval;

#define TIMER_DISABLED (0)
#define TIMER_ENABLE (1)
#define TIMER_IMASK (2)
#define TIMER_ISTATUS (4)

#define TIMER_IRQ 27

// CNTKCTL_EL1: the event stream (an event when bit EVNTI of the virtual counter
// goes from 0 to 1: at 54 MHz, bit 12 every 152 us), which bounds every wait
// for an event (arch_cpu_wait()), as Linux's arch timer does it
#define CNTKCTL_EVNTEN (1 << 2)
#define CNTKCTL_EVNTDIR (1 << 3)
#define CNTKCTL_EVNTI_SHIFT 4
#define CNTKCTL_EVNTI_MASK (0xf << CNTKCTL_EVNTI_SHIFT)
static const uint32 kEventStreamBit = 12;


static void
enable_event_stream()
{
	uint64 control = READ_SPECIALREG(CNTKCTL_EL1);
	control &= ~(CNTKCTL_EVNTI_MASK | CNTKCTL_EVNTDIR);
	control |= CNTKCTL_EVNTEN | (kEventStreamBit << CNTKCTL_EVNTI_SHIFT);
	WRITE_SPECIALREG(CNTKCTL_EL1, control);
	arm64_isb();
}


/*!	A write to the timer's registers is only certain to reach the timer at the
	next context synchronization (as Linux's arch_timer_reg_write_cp15()):
	without it, a timer armed outside an interrupt could stay disabled until the
	CPU happened to take another interrupt (seconds on an idle CPU).
*/
static inline void
write_timer_control(uint64 control)
{
	WRITE_SPECIALREG(CNTV_CTL_EL0, control);
	arm64_isb();
}


void
arch_timer_set_hardware_timer(bigtime_t timeout)
{
	if (timeout > sTimerMaxInterval)
		timeout = sTimerMaxInterval;

	WRITE_SPECIALREG(CNTV_TVAL_EL0, timeout * sTimerTicksUS);
	write_timer_control(TIMER_ENABLE);
}


void
arch_timer_clear_hardware_timer()
{
	write_timer_control(TIMER_DISABLED);
}


static uint32 sTickCount[SMP_MAX_CPUS];


int32
arch_timer_interrupt(void *data)
{
	write_timer_control(TIMER_DISABLED);
	debug_heartbeat(0, debug_heartbeat_hash(++sTickCount[smp_get_current_cpu()]));
	return timer_interrupt();
}


static int
dump_arch_timer(int argc, char** argv)
{
	kprintf("CPU %" B_PRId32 ": CNTV_CTL %#" B_PRIx64 " CVAL %#" B_PRIx64 " TVAL %"
		B_PRId32 " CNTVCT %#" B_PRIx64 " (%" B_PRIu64 " ticks/us)\n",
		smp_get_current_cpu(), READ_SPECIALREG(CNTV_CTL_EL0),
		READ_SPECIALREG(CNTV_CVAL_EL0), (int32)READ_SPECIALREG(CNTV_TVAL_EL0),
		READ_SPECIALREG(CNTVCT_EL0), sTimerTicksUS);
	return 0;
}


int
arch_init_timer(kernel_args *args)
{
	add_debugger_command("arm64_timer", &dump_arch_timer,
		"Dumps the current CPU's virtual timer registers");

	sTimerTicksUS = READ_SPECIALREG(CNTFRQ_EL0) / 1000000;
	sTimerMaxInterval = INT32_MAX / sTimerTicksUS;

	WRITE_SPECIALREG(CNTV_CTL_EL0, TIMER_DISABLED);
	enable_event_stream();
	install_io_interrupt_handler(TIMER_IRQ, &arch_timer_interrupt, NULL, 0);

	return B_OK;
}


status_t
arm64_timer_per_cpu_init()
{
	InterruptController* ic = InterruptController::Get();

	if (ic == NULL)
		return B_ERROR;

	WRITE_SPECIALREG(CNTV_CTL_EL0, TIMER_DISABLED);
	enable_event_stream();
	ic->EnableInterrupt(TIMER_IRQ);

	return B_OK;
}
