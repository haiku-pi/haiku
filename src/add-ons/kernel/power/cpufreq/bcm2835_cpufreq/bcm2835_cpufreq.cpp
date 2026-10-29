/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	The ARM clock of the Raspberry Pi 4 (BCM2711), which the VideoCore
	firmware owns. The firmware runs the ARM at its maximum only for its
	boot-time turbo period (about two minutes after power-on), then drops it
	to the minimum (1500 -> 600 MHz, and the core clock 500 -> 200 MHz) unless
	the operating system asks for a rate, as Linux's cpufreq driver does.

	The clock follows the load, as the scheduler reports it: its requests to
	raise or lower a CPU's performance (each reschedule, with interrupts
	disabled) only move that CPU's level between 0 and kCPUPerformanceScaleMax,
	as intel_pstates moves its P-states. The ARM clock is shared by all cores:
	a thread looks at the highest level every kUpdateInterval and asks the
	firmware, through the mailbox (which cannot be used with interrupts
	disabled), for the lowest of its rates that covers it, among the minimum,
	750, 1000, 1200 MHz and the maximum. It lowers the rate only after
	kLowerAfter looks that agree, so that a short pause does not cost the next
	burst its speed. In the power saving scheduler mode the clock stays at the
	minimum. The firmware keeps lowering the clocks on its own when the SoC gets
	too hot or the supply too low.

	Never with turbo (the requests' "skip setting turbo" word, and turbo
	switched off at the start): while turbo is on, the firmware answers every
	rate with the maximum. Turbo also kept the core clock (the VPU, the AXI
	side of the hardware decoders and the GPU) at its maximum; without it the
	core idles at 200 MHz, which cost HEVC decoding a fifth of its frame rate:
	the core clock is set to its maximum here, once (before turbo goes off),
	and stays there.
*/


#include <cpufreq.h>
#include <KernelExport.h>

#include <string.h>

#include <smp.h>

#include <bcm2835_mailbox.h>


#define BCM2835_CPUFREQ_MODULE_NAME	CPUFREQ_MODULES_PREFIX "/bcm2835_cpufreq/v1"


static const bigtime_t kUpdateInterval = 50000;
static const int32 kLowerAfter = 4;
static const uint32 kIntermediateRates[] = { 750000000, 1000000000, 1200000000 };

static bcm2835_mailbox_module_info* sMailbox;
static uint32 sMinRate;
static uint32 sMaxRate;
static uint32 sRates[2 + B_COUNT_OF(kIntermediateRates)];
static int32 sRateCount;

static int32 sLevel[SMP_MAX_CPUS];
static bool sPowerSaving;
static thread_id sThread = -1;
static bool sQuit;


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
	// [clock ID, rate, skip setting turbo] -> [clock ID, rate]: never turbo,
	// which holds the ARM at its maximum whatever is asked later (as Linux)
	uint32 values[3] = { BCM2835_CLOCK_ARM, rate, 1 };
	status_t status = sMailbox->property_tag(BCM2835_TAG_SET_CLOCK_RATE,
		values, sizeof(values), sizeof(values), NULL);
	if (status != B_OK) {
		dprintf("bcm2835_cpufreq: setting the ARM clock to %" B_PRIu32
			" MHz failed: %s\n", rate / 1000000, strerror(status));
	}
	return status;
}


/*!	The lowest rate that gives \a level (0 to kCPUPerformanceScaleMax) of
	the range between the minimum and the maximum.
*/
static uint32
rate_for_level(int32 level)
{
	const uint64 wanted = sMinRate
		+ (uint64)(sMaxRate - sMinRate) * level / kCPUPerformanceScaleMax;
	for (int32 i = 0; i < sRateCount; i++) {
		if (sRates[i] >= wanted)
			return sRates[i];
	}
	return sMaxRate;
}


static status_t
clock_thread(void*)
{
	uint32 current = sMaxRate;
	int32 lowerVotes = 0;
	while (!sQuit) {
		snooze(kUpdateInterval);

		int32 level = 0;
		for (int32 cpu = 0; cpu < smp_get_num_cpus(); cpu++)
			level = max_c(level, atomic_get(&sLevel[cpu]));
		const uint32 wanted = sPowerSaving ? sMinRate : rate_for_level(level);

		if (wanted > current) {
			lowerVotes = 0;
		} else if (wanted < current && ++lowerVotes < kLowerAfter) {
			continue;
		} else if (wanted == current) {
			lowerVotes = 0;
			continue;
		}
		lowerVotes = 0;
		if (set_arm_rate(wanted) == B_OK)
			current = wanted;
	}
	return B_OK;
}


static void
bcm2835_set_scheduler_mode(scheduler_mode mode)
{
	sPowerSaving = mode == SCHEDULER_MODE_POWER_SAVING;
}


static status_t
bcm2835_increase_performance(int delta)
{
	int32* level = &sLevel[smp_get_current_cpu()];
	const int32 current = atomic_get(level);
	atomic_set(level, current
		+ (kCPUPerformanceScaleMax - current) * delta / kCPUPerformanceScaleMax);
	return B_OK;
}


static status_t
bcm2835_decrease_performance(int delta)
{
	int32* level = &sLevel[smp_get_current_cpu()];
	const int32 current = atomic_get(level);
	atomic_set(level, current - current * delta / kCPUPerformanceScaleMax);
	return B_OK;
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

	sRateCount = 0;
	sRates[sRateCount++] = sMinRate;
	for (size_t i = 0; i < B_COUNT_OF(kIntermediateRates); i++) {
		if (kIntermediateRates[i] > sMinRate && kIntermediateRates[i] < sMaxRate)
			sRates[sRateCount++] = kIntermediateRates[i];
	}
	if (sMaxRate > sMinRate)
		sRates[sRateCount++] = sMaxRate;

	// The core clock to its maximum first, while the firmware's boot-time
	// turbo still holds it there: switching turbo off with the core clock
	// left to the firmware let it drop the core clock, which also clocks the
	// peripheral bus, while the CPUs used it, and on a Pi that had been off
	// for minutes that froze the bus (a CPU stopped inside a UART access, the
	// others behind it: the whole system, the kernel debugger too). In that
	// order 4 of 16 network boots after 4 minutes off stopped so; in this
	// one 0 of 15.
	uint32 coreMax[2] = { BCM2835_CLOCK_CORE, 0 };
	if (sMailbox->property_tag(BCM2835_TAG_GET_MAX_CLOCK_RATE, coreMax,
			sizeof(coreMax), sizeof(uint32), NULL) == B_OK && coreMax[1] != 0) {
		// [clock ID, rate, skip setting turbo] -> [clock ID, rate]
		uint32 values[3] = { BCM2835_CLOCK_CORE, coreMax[1], 1 };
		sMailbox->property_tag(BCM2835_TAG_SET_CLOCK_RATE, values,
			sizeof(values), sizeof(values), NULL);
	}

	// turbo off (the firmware's, or a previous system's), [ID, level] -> [ID,
	// level]; then start at the maximum: the boot is busy
	uint32 turbo[2] = { 0, 0 };
	sMailbox->property_tag(BCM2835_TAG_GET_TURBO, turbo, sizeof(turbo),
		sizeof(uint32), NULL);
	const uint32 turboBefore = turbo[1];
	turbo[0] = 0;
	turbo[1] = 0;
	sMailbox->property_tag(BCM2835_TAG_SET_TURBO, turbo, sizeof(turbo),
		sizeof(turbo), NULL);
	for (int32 cpu = 0; cpu < SMP_MAX_CPUS; cpu++)
		sLevel[cpu] = kCPUPerformanceScaleMax;
	set_arm_rate(sMaxRate);

	sQuit = false;
	sThread = spawn_kernel_thread(&clock_thread, "bcm2835 cpufreq",
		B_URGENT_DISPLAY_PRIORITY, NULL);
	if (sThread < 0) {
		put_module(BCM2835_MAILBOX_MODULE_NAME);
		return sThread;
	}
	resume_thread(sThread);

	dprintf("bcm2835_cpufreq: ARM clock %" B_PRIu32 "-%" B_PRIu32 " MHz, "
		"following the load in %" B_PRId32 " steps (turbo was %" B_PRIu32
		", now %" B_PRIu32 ")\n", sMinRate / 1000000, sMaxRate / 1000000,
		sRateCount, turboBefore, turbo[1]);
	return B_OK;
}


static void
uninit_cpufreq()
{
	sQuit = true;
	status_t result;
	wait_for_thread(sThread, &result);
	set_arm_rate(sMaxRate);
	put_module(BCM2835_MAILBOX_MODULE_NAME);
}


static status_t
std_ops(int32 op, ...)
{
	switch (op) {
		case B_MODULE_INIT:
			return init_cpufreq();

		case B_MODULE_UNINIT:
			uninit_cpufreq();
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
