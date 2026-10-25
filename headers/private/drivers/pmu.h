/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _PMU_H
#define _PMU_H


#include <Drivers.h>


/*!	The interface of /dev/misc/pmu: the CPUs' hardware performance counters,
	counted system-wide (every CPU, every team, user and kernel mode). The
	counters are read in the kernel; user mode access to them stays off.

	Use: PMU_GET_INFO, PMU_START with the events, PMU_READ any number of times
	(while running or after PMU_STOP), PMU_STOP. One opener at a time; closing
	the device stops the counters.
*/


#define PMU_DEVICE_NAME			"misc/pmu"
#define PMU_INTERFACE_VERSION	1

#define PMU_MAX_EVENTS			31
	// event counters (Arm PMUv3: PMCR_EL0.N, up to 31; Cortex-A72 has 6)


enum {
	PMU_GET_INFO = B_DEVICE_OP_CODES_END + 0x5000,	// pmu_info*
	PMU_START,		// const pmu_config*
	PMU_STOP,		// NULL
	PMU_READ,		// pmu_read_args*
};


typedef struct pmu_info {
	uint32		version;			// PMU_INTERFACE_VERSION
	uint32		cpu_count;
	uint32		event_counters;		// how many events can be counted at once
	uint32		pmu_version;		// ID_AA64DFR0_EL1.PMUVer
	uint64		cpu_id;				// MIDR_EL1
	uint64		pmcr;				// PMCR_EL0 (implementer, ID code, N)
	uint64		common_events[2];	// PMCEID0/1_EL0: bit n of [0] = event n,
									// of [1] = event 0x20 + n is implemented
									// (n < 32; events from 0x40 on are
									// implementation defined, not listed)
} pmu_info;


// pmu_config.flags
#define PMU_EXCLUDE_USER		0x01	// do not count in user mode (EL0)
#define PMU_EXCLUDE_KERNEL		0x02	// do not count in the kernel (EL1)

typedef struct pmu_config {
	uint32		flags;
	uint32		event_count;		// <= pmu_info.event_counters
	uint16		events[PMU_MAX_EVENTS];	// Arm event numbers (0x08 INST_RETIRED,
										// ...)
} pmu_config;


typedef struct pmu_cpu_counts {
	uint64		cycles;				// the cycle counter (not filtered: counts
									// user and kernel mode; stops in WFI)
	uint64		events[PMU_MAX_EVENTS];
} pmu_cpu_counts;

typedef struct pmu_read_args {
	uint32		cpu_count;			// in: entries in cpus; out: CPUs counted
	uint32		event_count;		// out
	uint32		flags;				// out: the pmu_config's flags
	uint32		running;			// out: 1 if still counting
	bigtime_t	elapsed;			// out: time counted, in microseconds
	bigtime_t	max_sample_gap;		// out: the longest time the driver did not
									// sample the 32-bit counters (> ~1 s:
									// counts may have been lost)
	pmu_cpu_counts*	cpus;			// out: counts since PMU_START, per CPU
} pmu_read_args;


#endif	// _PMU_H
