/*
 * Copyright 2019-2026 Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */
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


void
arch_timer_set_hardware_timer(bigtime_t timeout)
{
	if (timeout > sTimerMaxInterval)
		timeout = sTimerMaxInterval;

	WRITE_SPECIALREG(CNTV_TVAL_EL0, timeout * sTimerTicksUS);
	WRITE_SPECIALREG(CNTV_CTL_EL0, TIMER_ENABLE);
}


void
arch_timer_clear_hardware_timer()
{
	WRITE_SPECIALREG(CNTV_CTL_EL0, TIMER_DISABLED);
}


int32
arch_timer_interrupt(void *data)
{
	WRITE_SPECIALREG(CNTV_CTL_EL0, TIMER_DISABLED);
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
	ic->EnableInterrupt(TIMER_IRQ);

	return B_OK;
}
