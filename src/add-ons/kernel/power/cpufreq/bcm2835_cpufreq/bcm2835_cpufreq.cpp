/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The ARM clock of the Raspberry Pi 4 (BCM2711), which the VideoCore
	firmware owns. The firmware runs the ARM at its maximum only for its
	boot-time turbo period (about two minutes after power-on), then drops it
	to the minimum (1500 -> 600 MHz, and the core clock 500 -> 200 MHz) unless
	the operating system asks for a rate, as Linux's cpufreq driver does.

	This module asks for the maximum in the low latency scheduler mode and
	for the minimum in the power saving mode. The firmware keeps lowering the
	clocks on its own when the SoC gets too hot or the supply too low. The
	ARM clock is shared by all cores, and the scheduler's per-core
	performance requests come with interrupts disabled, where the mailbox
	cannot be used: they are not supported, so the scheduler does not track
	the load for them.
*/


#include <cpufreq.h>
#include <KernelExport.h>

#include <string.h>

#include <bcm2835_mailbox.h>


#define BCM2835_CPUFREQ_MODULE_NAME	CPUFREQ_MODULES_PREFIX "/bcm2835_cpufreq/v1"


static bcm2835_mailbox_module_info* sMailbox;
static uint32 sMinRate;
static uint32 sMaxRate;


static status_t
get_arm_rate(uint32 tag, uint32& rate)
{
	// [clock ID] -> [clock ID, rate]
	uint32 values[2] = { BCM2835_CLOCK_ARM, 0 };
	status_t status = sMailbox->property_tag(tag, values, sizeof(values),
		sizeof(uint32), NULL);
	if (status == B_OK)
		rate = values[1];
	return status;
}


static status_t
set_arm_rate(uint32 rate)
{
	// [clock ID, rate, skip setting turbo] -> [clock ID, rate]: with turbo,
	// the firmware also raises the core clock with the maximum ARM rate
	uint32 values[3] = { BCM2835_CLOCK_ARM, rate, 0 };
	status_t status = sMailbox->property_tag(BCM2835_TAG_SET_CLOCK_RATE,
		values, sizeof(values), sizeof(values), NULL);
	if (status != B_OK) {
		dprintf("bcm2835_cpufreq: setting the ARM clock to %" B_PRIu32
			" MHz failed: %s\n", rate / 1000000, strerror(status));
	}
	return status;
}


static void
bcm2835_set_scheduler_mode(scheduler_mode mode)
{
	set_arm_rate(mode == SCHEDULER_MODE_POWER_SAVING ? sMinRate : sMaxRate);
}


static status_t
bcm2835_increase_performance(int delta)
{
	return B_NOT_SUPPORTED;
}


static status_t
bcm2835_decrease_performance(int delta)
{
	return B_NOT_SUPPORTED;
}


static status_t
init_cpufreq()
{
	status_t status = get_module(BCM2835_MAILBOX_MODULE_NAME,
		(module_info**)&sMailbox);
	if (status != B_OK)
		return status;

	if (get_arm_rate(BCM2835_TAG_GET_MIN_CLOCK_RATE, sMinRate) != B_OK
		|| get_arm_rate(BCM2835_TAG_GET_MAX_CLOCK_RATE, sMaxRate) != B_OK
		|| sMinRate == 0 || sMaxRate < sMinRate) {
		put_module(BCM2835_MAILBOX_MODULE_NAME);
		return B_ERROR;
	}

	dprintf("bcm2835_cpufreq: ARM clock %" B_PRIu32 "-%" B_PRIu32 " MHz\n",
		sMinRate / 1000000, sMaxRate / 1000000);
	bcm2835_set_scheduler_mode(SCHEDULER_MODE_LOW_LATENCY);
	return B_OK;
}


static status_t
std_ops(int32 op, ...)
{
	switch (op) {
		case B_MODULE_INIT:
			return init_cpufreq();

		case B_MODULE_UNINIT:
			put_module(BCM2835_MAILBOX_MODULE_NAME);
			return B_OK;
	}

	return B_ERROR;
}


static cpufreq_module_info sBcm2835CPUFreq = {
	{
		BCM2835_CPUFREQ_MODULE_NAME,
		0,
		std_ops,
	},

	1.0f,

	bcm2835_set_scheduler_mode,

	bcm2835_increase_performance,
	bcm2835_decrease_performance,
};


module_info* modules[] = {
	(module_info*)&sBcm2835CPUFreq,
	NULL
};
