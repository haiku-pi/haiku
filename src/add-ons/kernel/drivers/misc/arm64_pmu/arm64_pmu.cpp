/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	/dev/misc/pmu for the Arm PMUv3 (Cortex-A72, ...): the event counters and
	the cycle counter of every CPU, counted system-wide (see <pmu.h>).

	Each CPU's PMU is programmed and read on that CPU, by call_all_cpus_sync().
	The event counters are 32 bits wide (PMUv3 before v3.5): at up to a few
	events per cycle they wrap within a second, so while counting, a kernel
	thread samples every CPU every kSampleInterval and adds what each counter
	advanced to 64-bit totals.

	The cycle counter (PMCCNTR_EL0) is shared with arch_get_frequency(), which
	measures the clock with it: that code only turns the counter on and takes
	differences, and so does this driver. Neither resets it (PMCR_EL0.C) or
	turns it off; the event counters are this driver's alone.

	User mode access to the PMU (PMUSERENR_EL0) stays off.

	What counts when: a CPU idles in WFI. The cycle counter and CPU_CYCLES stop
	while the core's clock is gated there (Cortex-A72), so "cycles" are the
	cycles a CPU was busy; events stop as well, as nothing executes. Under QEMU
	the cycle counter runs at a fixed rate, idle or not, and most cache events
	are not modelled (they stay 0).
*/


#include <Drivers.h>
#include <KernelExport.h>
#include <OS.h>

#include <stdlib.h>
#include <string.h>

#include <arch_cpu.h>
#include <kernel.h>
#include <lock.h>
#include <smp.h>
#include <util/AutoLock.h>

#include <pmu.h>


//#define TRACE_PMU
#ifdef TRACE_PMU
#	define TRACE(x...) dprintf("arm64_pmu: " x)
#else
#	define TRACE(x...)
#endif


// PMCR_EL0
#define PMCR_ENABLE				(1 << 0)
#define PMCR_CLOCK_DIVIDER		(1 << 3)	// count every 64th cycle
#define PMCR_LONG_CYCLES		(1 << 6)	// the cycle counter overflows at 64 bits
#define PMCR_COUNTERS(pmcr)		(((pmcr) >> 11) & 0x1f)

// PMEVTYPER<n>_EL0
#define PMEVTYPER_EXCLUDE_EL1	(1u << 31)
#define PMEVTYPER_EXCLUDE_EL0	(1u << 30)

#define CYCLE_COUNTER_BIT		(1u << 31)

static const bigtime_t kSampleInterval = 200000;
	// a 32-bit counter wraps after ~1 s at 3 events per cycle at 1.5 GHz


struct cpu_counts {
	uint64			cycles;
	uint64			lastCycles;
	uint64			events[PMU_MAX_EVENTS];
	uint32			lastEvents[PMU_MAX_EVENTS];
};


int32 api_version = B_CUR_DRIVER_API_VERSION;

static const char* sDeviceNames[] = { PMU_DEVICE_NAME, NULL };

static int32 sOpen;
static mutex sLock = MUTEX_INITIALIZER("arm64 pmu");
static int32 sCPUCount;
static uint32 sCounterCount;
	// event counters the PMU gives the kernel (PMCR_EL0.N)
static uint32 sPMUVersion;
static cpu_counts* sCounts;

// the session, guarded by sLock
static pmu_config sConfig;
static bool sRunning;
static bigtime_t sStartTime;
static bigtime_t sElapsed;
static bigtime_t sLastSample;
static bigtime_t sMaxSampleGap;
static thread_id sSampler = -1;
static sem_id sSamplerSem = -1;


static inline void
select_counter(uint32 counter)
{
	WRITE_SPECIALREG(PMSELR_EL0, counter);
	arm64_isb();
}


static inline uint32
event_counter_mask(uint32 count)
{
	return count >= 32 ? ~0u : (1u << count) - 1;
}


static void
program_cpu(void*, int cpu)
{
	cpu_counts& counts = sCounts[cpu];

	WRITE_SPECIALREG(PMCNTENCLR_EL0, event_counter_mask(sCounterCount));
	WRITE_SPECIALREG(PMINTENCLR_EL1,
		event_counter_mask(sCounterCount) | CYCLE_COUNTER_BIT);
	WRITE_SPECIALREG(PMUSERENR_EL0, 0);
	// the cycle counter counts in user mode and in the kernel (its reset
	// value is unknown)
	WRITE_SPECIALREG(PMCCFILTR_EL0, 0);

	uint32 filter = 0;
	if ((sConfig.flags & PMU_EXCLUDE_USER) != 0)
		filter |= PMEVTYPER_EXCLUDE_EL0;
	if ((sConfig.flags & PMU_EXCLUDE_KERNEL) != 0)
		filter |= PMEVTYPER_EXCLUDE_EL1;

	for (uint32 i = 0; i < sConfig.event_count; i++) {
		select_counter(i);
		WRITE_SPECIALREG(PMXEVTYPER_EL0, filter | sConfig.events[i]);
		WRITE_SPECIALREG(PMXEVCNTR_EL0, 0);
		counts.events[i] = 0;
		counts.lastEvents[i] = 0;
	}
	const uint32 used = event_counter_mask(sConfig.event_count);
	WRITE_SPECIALREG(PMOVSCLR_EL0, used);

	// On, every cycle counted; never PMCR_EL0.P or .C (see above).
	const uint64 pmcr = READ_SPECIALREG(PMCR_EL0);
	WRITE_SPECIALREG(PMCR_EL0,
		(pmcr | PMCR_ENABLE | PMCR_LONG_CYCLES) & ~(uint64)PMCR_CLOCK_DIVIDER);
	arm64_isb();

	WRITE_SPECIALREG(PMCNTENSET_EL0, used | CYCLE_COUNTER_BIT);
	arm64_isb();
	counts.cycles = 0;
	counts.lastCycles = READ_SPECIALREG(PMCCNTR_EL0);
}


static void
sample_cpu(void*, int cpu)
{
	cpu_counts& counts = sCounts[cpu];

	const uint64 cycles = READ_SPECIALREG(PMCCNTR_EL0);
	counts.cycles += cycles - counts.lastCycles;
	counts.lastCycles = cycles;

	for (uint32 i = 0; i < sConfig.event_count; i++) {
		select_counter(i);
		const uint32 value = READ_SPECIALREG(PMXEVCNTR_EL0);
		counts.events[i] += (uint32)(value - counts.lastEvents[i]);
		counts.lastEvents[i] = value;
	}
}


static void
stop_cpu(void* cookie, int cpu)
{
	sample_cpu(cookie, cpu);
	// the event counters only: the cycle counter stays on
	WRITE_SPECIALREG(PMCNTENCLR_EL0, event_counter_mask(sCounterCount));
	arm64_isb();
}


/*!	Called with sLock held. */
static void
sample_all_cpus()
{
	const bigtime_t now = system_time();
	call_all_cpus_sync(&sample_cpu, NULL);
	if (now - sLastSample > sMaxSampleGap)
		sMaxSampleGap = now - sLastSample;
	sLastSample = now;
}


static status_t
sampler_thread(void* _sem)
{
	const sem_id sem = (sem_id)(addr_t)_sem;
	while (true) {
		status_t status = acquire_sem_etc(sem, 1, B_RELATIVE_TIMEOUT,
			kSampleInterval);
		if (status != B_TIMED_OUT)
			break;

		MutexLocker locker(sLock);
		if (!sRunning)
			break;
		sample_all_cpus();
	}
	return B_OK;
}


static void
stop_counting()
{
	MutexLocker locker(sLock);
	if (!sRunning)
		return;

	call_all_cpus_sync(&stop_cpu, NULL);
	sRunning = false;
	sElapsed = system_time() - sStartTime;

	const thread_id sampler = sSampler;
	const sem_id sem = sSamplerSem;
	sSampler = -1;
	sSamplerSem = -1;
	locker.Unlock();

	delete_sem(sem);
	status_t result;
	wait_for_thread(sampler, &result);
}


static status_t
start_counting(const pmu_config& config)
{
	if (config.event_count > sCounterCount)
		return B_BAD_VALUE;
	const uint32 maxEvent = sPMUVersion >= 4 ? 0xffff : 0x3ff;
	for (uint32 i = 0; i < config.event_count; i++) {
		if (config.events[i] > maxEvent)
			return B_BAD_VALUE;
	}
	if ((config.flags & ~(uint32)(PMU_EXCLUDE_USER | PMU_EXCLUDE_KERNEL))
			!= 0) {
		return B_BAD_VALUE;
	}

	stop_counting();

	MutexLocker locker(sLock);
	sConfig = config;

	sSamplerSem = create_sem(0, "arm64 pmu sampler");
	if (sSamplerSem < 0)
		return sSamplerSem;

	call_all_cpus_sync(&program_cpu, NULL);
	sRunning = true;
	sStartTime = sLastSample = system_time();
	sElapsed = 0;
	sMaxSampleGap = 0;

	sSampler = spawn_kernel_thread(&sampler_thread, "arm64 pmu sampler",
		B_URGENT_PRIORITY, (void*)(addr_t)sSamplerSem);
	if (sSampler < 0 || resume_thread(sSampler) != B_OK) {
		status_t status = sSampler < 0 ? sSampler : B_ERROR;
		call_all_cpus_sync(&stop_cpu, NULL);
		sRunning = false;
		delete_sem(sSamplerSem);
		sSamplerSem = -1;
		sSampler = -1;
		return status;
	}

	TRACE("counting %" B_PRIu32 " events\n", config.event_count);
	return B_OK;
}


static status_t
read_counts(pmu_read_args* userArgs)
{
	pmu_read_args args;
	if (!IS_USER_ADDRESS(userArgs)
		|| user_memcpy(&args, userArgs, sizeof(args)) != B_OK) {
		return B_BAD_ADDRESS;
	}
	if (args.cpus == NULL || !IS_USER_ADDRESS(args.cpus))
		return B_BAD_ADDRESS;

	MutexLocker locker(sLock);
	if (sRunning)
		sample_all_cpus();

	const uint32 cpuCount = min_c(args.cpu_count, (uint32)sCPUCount);
	for (uint32 cpu = 0; cpu < cpuCount; cpu++) {
		pmu_cpu_counts counts;
		memset(&counts, 0, sizeof(counts));
		counts.cycles = sCounts[cpu].cycles;
		memcpy(counts.events, sCounts[cpu].events,
			sConfig.event_count * sizeof(uint64));
		if (user_memcpy(&args.cpus[cpu], &counts, sizeof(counts)) != B_OK)
			return B_BAD_ADDRESS;
	}

	args.cpu_count = cpuCount;
	args.event_count = sConfig.event_count;
	args.flags = sConfig.flags;
	args.running = sRunning ? 1 : 0;
	args.elapsed = sRunning ? system_time() - sStartTime : sElapsed;
	args.max_sample_gap = sMaxSampleGap;
	if (user_memcpy(userArgs, &args, sizeof(args)) != B_OK)
		return B_BAD_ADDRESS;
	return B_OK;
}


static status_t
get_info(pmu_info* userInfo)
{
	pmu_info info;
	memset(&info, 0, sizeof(info));
	info.version = PMU_INTERFACE_VERSION;
	info.cpu_count = sCPUCount;
	info.event_counters = min_c(sCounterCount, (uint32)PMU_MAX_EVENTS);
	info.pmu_version = sPMUVersion;
	info.cpu_id = READ_SPECIALREG(MIDR_EL1);
	info.pmcr = READ_SPECIALREG(PMCR_EL0);
	info.common_events[0] = READ_SPECIALREG(PMCEID0_EL0);
	info.common_events[1] = READ_SPECIALREG(PMCEID1_EL0);

	if (!IS_USER_ADDRESS(userInfo)
		|| user_memcpy(userInfo, &info, sizeof(info)) != B_OK) {
		return B_BAD_ADDRESS;
	}
	return B_OK;
}


//	#pragma mark - device hooks


static status_t
pmu_open(const char*, uint32, void** _cookie)
{
	if (atomic_test_and_set(&sOpen, 1, 0) != 0)
		return B_BUSY;
	*_cookie = NULL;
	return B_OK;
}


static status_t
pmu_close(void*)
{
	return B_OK;
}


static status_t
pmu_free(void*)
{
	stop_counting();
	atomic_set(&sOpen, 0);
	return B_OK;
}


static status_t
pmu_control(void*, uint32 op, void* buffer, size_t length)
{
	switch (op) {
		case PMU_GET_INFO:
			return get_info((pmu_info*)buffer);

		case PMU_START:
		{
			pmu_config config;
			if (!IS_USER_ADDRESS(buffer)
				|| user_memcpy(&config, buffer, sizeof(config)) != B_OK) {
				return B_BAD_ADDRESS;
			}
			return start_counting(config);
		}

		case PMU_STOP:
			stop_counting();
			return B_OK;

		case PMU_READ:
			return read_counts((pmu_read_args*)buffer);
	}

	return B_DEV_INVALID_IOCTL;
}


static status_t
pmu_read(void*, off_t, void*, size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


static status_t
pmu_write(void*, off_t, const void*, size_t* _length)
{
	*_length = 0;
	return B_NOT_ALLOWED;
}


//	#pragma mark - driver


static uint32
pmu_version()
{
	return (READ_SPECIALREG(ID_AA64DFR0_EL1) >> 8) & 0xf;
}


status_t
init_hardware()
{
	// 0: no PMU; 0xf: an implementation defined one, not PMUv3
	const uint32 version = pmu_version();
	if (version == 0 || version == 0xf)
		return B_DEVICE_NOT_FOUND;
	return B_OK;
}


status_t
init_driver()
{
	sPMUVersion = pmu_version();
	if (sPMUVersion == 0 || sPMUVersion == 0xf)
		return B_DEVICE_NOT_FOUND;

	sCPUCount = smp_get_num_cpus();
	sCounterCount = min_c(PMCR_COUNTERS(READ_SPECIALREG(PMCR_EL0)),
		(uint32)PMU_MAX_EVENTS);
	sCounts = (cpu_counts*)calloc(sCPUCount, sizeof(cpu_counts));
	if (sCounts == NULL)
		return B_NO_MEMORY;

	dprintf("arm64_pmu: PMUv3 (version %" B_PRIu32 "), %" B_PRIu32
		" event counters, %" B_PRId32 " CPUs\n", sPMUVersion, sCounterCount,
		sCPUCount);
	return B_OK;
}


void
uninit_driver()
{
	stop_counting();
	free(sCounts);
	sCounts = NULL;
}


const char**
publish_devices()
{
	return sDeviceNames;
}


device_hooks*
find_device(const char*)
{
	static device_hooks hooks = {
		&pmu_open,
		&pmu_close,
		&pmu_free,
		&pmu_control,
		&pmu_read,
		&pmu_write,
	};
	return &hooks;
}
