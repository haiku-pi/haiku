/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	pmustat: counts hardware events (cycles, instructions, cache refills, ...)
	on all CPUs while a command runs, or for a time, with /dev/misc/pmu; like
	"perf stat -a".
*/


#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/wait.h>
#include <unistd.h>

#include <OS.h>

#include <pmu.h>


typedef struct event_name {
	const char*	name;
	uint16		number;
	const char*	description;
} event_name;

// Arm PMUv3 common events, then the Cortex-A72's own (its TRM, C12.9)
static const event_name kEvents[] = {
	{"sw_incr", 0x00, "software increment"},
	{"l1i_cache_refill", 0x01, "L1 instruction cache refills"},
	{"l1i_tlb_refill", 0x02, "L1 instruction TLB refills"},
	{"l1d_cache_refill", 0x03, "L1 data cache refills"},
	{"l1d_cache", 0x04, "L1 data cache accesses"},
	{"l1d_tlb_refill", 0x05, "L1 data TLB refills"},
	{"ld_retired", 0x06, "loads retired"},
	{"st_retired", 0x07, "stores retired"},
	{"inst_retired", 0x08, "instructions retired"},
	{"exc_taken", 0x09, "exceptions taken"},
	{"exc_return", 0x0a, "exception returns"},
	{"cid_write_retired", 0x0b, "CONTEXTIDR writes"},
	{"pc_write_retired", 0x0c, "software changes of the PC"},
	{"br_immed_retired", 0x0d, "immediate branches retired"},
	{"br_return_retired", 0x0e, "procedure returns retired"},
	{"unaligned_ldst_retired", 0x0f, "unaligned loads/stores retired"},
	{"br_mis_pred", 0x10, "branches mispredicted or not predicted"},
	{"cpu_cycles", 0x11, "cycles (filtered like the events)"},
	{"br_pred", 0x12, "predictable branches speculatively executed"},
	{"mem_access", 0x13, "data memory accesses"},
	{"l1i_cache", 0x14, "L1 instruction cache accesses"},
	{"l1d_cache_wb", 0x15, "L1 data cache write-backs"},
	{"l2d_cache", 0x16, "L2 data cache accesses"},
	{"l2d_cache_refill", 0x17, "L2 data cache refills"},
	{"l2d_cache_wb", 0x18, "L2 data cache write-backs"},
	{"bus_access", 0x19, "bus accesses"},
	{"memory_error", 0x1a, "local memory errors"},
	{"inst_spec", 0x1b, "instructions speculatively executed"},
	{"ttbr_write_retired", 0x1c, "TTBR writes"},
	{"bus_cycles", 0x1d, "bus cycles"},
	{"l1d_cache_allocate", 0x1f, "L1 data cache allocations without refill"},
	{"l2d_cache_allocate", 0x20, "L2 data cache allocations without refill"},
	{"br_retired", 0x21, "branches retired"},
	{"br_mis_pred_retired", 0x22, "mispredicted branches retired"},
	{"stall_frontend", 0x23, "cycles without an instruction to dispatch"},
	{"stall_backend", 0x24, "cycles a dispatched instruction waited"},
	{"l1d_tlb", 0x25, "L1 data TLB accesses"},
	{"l1i_tlb", 0x26, "L1 instruction TLB accesses"},
	{"l3d_cache_allocate", 0x29, "L3 data cache allocations"},
	{"l3d_cache_refill", 0x2a, "L3 data cache refills"},
	{"l3d_cache", 0x2b, "L3 data cache accesses"},
	{"l2d_tlb_refill", 0x2d, "L2 TLB refills"},
	{"l2d_tlb", 0x2f, "L2 TLB accesses"},

	{"l1d_cache_rd", 0x40, "L1 data cache reads"},
	{"l1d_cache_wr", 0x41, "L1 data cache writes"},
	{"l1d_cache_refill_rd", 0x42, "L1 data cache refills by reads"},
	{"l1d_cache_refill_wr", 0x43, "L1 data cache refills by writes"},
	{"l1d_cache_wb_victim", 0x46, "L1 data cache write-backs of victims"},
	{"l1d_cache_wb_clean", 0x47, "L1 data cache write-backs by cleaning"},
	{"l1d_cache_inval", 0x48, "L1 data cache invalidations"},
	{"l1d_tlb_refill_rd", 0x4c, "L1 data TLB refills by reads"},
	{"l1d_tlb_refill_wr", 0x4d, "L1 data TLB refills by writes"},
	{"l2d_cache_rd", 0x50, "L2 data cache reads"},
	{"l2d_cache_wr", 0x51, "L2 data cache writes"},
	{"l2d_cache_refill_rd", 0x52, "L2 data cache refills by reads"},
	{"l2d_cache_refill_wr", 0x53, "L2 data cache refills by writes"},
	{"l2d_cache_wb_victim", 0x56, "L2 data cache write-backs of victims"},
	{"l2d_cache_wb_clean", 0x57, "L2 data cache write-backs by cleaning"},
	{"l2d_cache_inval", 0x58, "L2 data cache invalidations"},
	{"bus_access_rd", 0x60, "bus reads"},
	{"bus_access_wr", 0x61, "bus writes"},
	{"bus_access_shared", 0x62, "bus accesses, shared"},
	{"bus_access_not_shared", 0x63, "bus accesses, not shared"},
	{"bus_access_normal", 0x64, "bus accesses, normal memory"},
	{"bus_access_periph", 0x65, "bus accesses, peripherals"},
	{"mem_access_rd", 0x66, "data memory reads"},
	{"mem_access_wr", 0x67, "data memory writes"},
	{"unaligned_ld_spec", 0x68, "unaligned loads"},
	{"unaligned_st_spec", 0x69, "unaligned stores"},
	{"unaligned_ldst_spec", 0x6a, "unaligned loads/stores"},
	{"ldrex_spec", 0x6c, "exclusive loads"},
	{"strex_pass_spec", 0x6d, "exclusive stores that passed"},
	{"strex_fail_spec", 0x6e, "exclusive stores that failed"},
	{"ld_spec", 0x70, "loads speculatively executed"},
	{"st_spec", 0x71, "stores speculatively executed"},
	{"ldst_spec", 0x72, "loads/stores speculatively executed"},
	{"dp_spec", 0x73, "integer data processing"},
	{"ase_spec", 0x74, "Advanced SIMD instructions"},
	{"vfp_spec", 0x75, "floating point instructions"},
	{"pc_write_spec", 0x76, "software changes of the PC, speculative"},
	{"crypto_spec", 0x77, "crypto instructions"},
	{"br_immed_spec", 0x78, "immediate branches, speculative"},
	{"br_return_spec", 0x79, "procedure returns, speculative"},
	{"br_indirect_spec", 0x7a, "indirect branches, speculative"},
	{"isb_spec", 0x7c, "ISB barriers"},
	{"dsb_spec", 0x7d, "DSB barriers"},
	{"dmb_spec", 0x7e, "DMB barriers"},
	{"exc_undef", 0x81, "undefined instruction exceptions"},
	{"exc_svc", 0x82, "supervisor calls (system calls)"},
	{"exc_pabort", 0x83, "instruction aborts"},
	{"exc_dabort", 0x84, "data aborts (page faults)"},
	{"exc_irq", 0x86, "IRQ exceptions"},
	{"exc_fiq", 0x87, "FIQ exceptions"},
	{"exc_smc", 0x88, "secure monitor calls"},
	{"exc_hvc", 0x8a, "hypervisor calls"},
};

#define EVENT_NAME_COUNT (sizeof(kEvents) / sizeof(kEvents[0]))

// the default set: what the derived metrics need, 6 counters
static const uint16 kDefaultEvents[] = {
	0x08,	// inst_retired
	0x04,	// l1d_cache
	0x03,	// l1d_cache_refill
	0x16,	// l2d_cache
	0x17,	// l2d_cache_refill
	0x10,	// br_mis_pred
};


static const char* kUsage =
	"Usage: %s [options] COMMAND [ARGS...]\n"
	"       %s [options] -a SECONDS\n"
	"Counts hardware events on all CPUs while COMMAND runs (or for SECONDS)\n"
	"and prints the totals, the rates and derived metrics.\n"
	"\n"
	"  -e EVENTS   comma-separated event names or numbers (0x13), up to the\n"
	"              number of counters (default: inst_retired, l1d_cache,\n"
	"              l1d_cache_refill, l2d_cache, l2d_cache_refill,\n"
	"              br_mis_pred); the cycle counter is always read\n"
	"  -a SECONDS  count for SECONDS (a fraction is fine) instead of a command\n"
	"  -A          also print the counts of each CPU\n"
	"  -u          count the events in user mode only\n"
	"  -k          count the events in the kernel only\n"
	"  -l          list the events and whether this CPU implements them\n"
	"  -h          this help\n";


static const event_name*
find_event(uint16 number)
{
	for (size_t i = 0; i < EVENT_NAME_COUNT; i++) {
		if (kEvents[i].number == number)
			return &kEvents[i];
	}
	return NULL;
}


static const char*
event_label(uint16 number, char* buffer, size_t size)
{
	const event_name* event = find_event(number);
	if (event != NULL)
		return event->name;
	snprintf(buffer, size, "raw_0x%02x", number);
	return buffer;
}


/*!	1 if implemented, 0 if not, -1 if the PMU does not say (implementation
	defined events, 0x40 and up).
*/
static int
event_implemented(const pmu_info* info, uint16 number)
{
	if (number < 0x20)
		return (info->common_events[0] >> number) & 1;
	if (number < 0x40)
		return (info->common_events[1] >> (number - 0x20)) & 1;
	return -1;
}


static bool
parse_events(const char* list, pmu_config* config, uint32 maxEvents)
{
	char* copy = strdup(list);
	char* save;
	config->event_count = 0;
	for (char* token = strtok_r(copy, ",", &save); token != NULL;
			token = strtok_r(NULL, ",", &save)) {
		long number = -1;
		for (size_t i = 0; i < EVENT_NAME_COUNT; i++) {
			if (strcasecmp(token, kEvents[i].name) == 0) {
				number = kEvents[i].number;
				break;
			}
		}
		if (number < 0) {
			char* end;
			number = strtol(token, &end, 0);
			if (*end != '\0' || end == token || number < 0
				|| number > 0xffff) {
				fprintf(stderr, "pmustat: unknown event \"%s\" (pmustat -l "
					"lists them)\n", token);
				free(copy);
				return false;
			}
		}
		if (config->event_count >= maxEvents) {
			fprintf(stderr, "pmustat: this PMU counts %" B_PRIu32 " events "
				"at a time; measure the others in another run\n", maxEvents);
			free(copy);
			return false;
		}
		config->events[config->event_count++] = (uint16)number;
	}
	free(copy);
	return true;
}


static void
list_events(const pmu_info* info)
{
	printf("%" B_PRIu32 " event counters and the cycle counter, on %" B_PRIu32
		" CPUs (MIDR 0x%08" B_PRIx64 ", PMU version %" B_PRIu32 ")\n\n",
		info->event_counters, info->cpu_count, info->cpu_id, info->pmu_version);
	for (size_t i = 0; i < EVENT_NAME_COUNT; i++) {
		int implemented = event_implemented(info, kEvents[i].number);
		printf("  0x%02x  %-24s %-44s %s\n", kEvents[i].number,
			kEvents[i].name, kEvents[i].description,
			implemented > 0 ? "yes" : implemented == 0 ? "no"
				: "(Cortex-A72)");
	}
}


/*!	Formats a count with thousands separators. */
static const char*
format_count(uint64 value, char* buffer, size_t size)
{
	char digits[32];
	snprintf(digits, sizeof(digits), "%" B_PRIu64, value);
	size_t length = strlen(digits);
	size_t out = 0;
	for (size_t i = 0; i < length && out + 2 < size; i++) {
		if (i > 0 && (length - i) % 3 == 0)
			buffer[out++] = ',';
		buffer[out++] = digits[i];
	}
	buffer[out] = '\0';
	return buffer;
}


static const char*
format_rate(double perSecond, char* buffer, size_t size)
{
	if (perSecond >= 1e9)
		snprintf(buffer, size, "%8.3f G/s", perSecond / 1e9);
	else if (perSecond >= 1e6)
		snprintf(buffer, size, "%8.3f M/s", perSecond / 1e6);
	else if (perSecond >= 1e3)
		snprintf(buffer, size, "%8.3f K/s", perSecond / 1e3);
	else
		snprintf(buffer, size, "%8.3f /s ", perSecond);
	return buffer;
}


static int
event_index(const pmu_config* config, uint16 number)
{
	for (uint32 i = 0; i < config->event_count; i++) {
		if (config->events[i] == number)
			return (int)i;
	}
	return -1;
}


static void
print_results(const pmu_info* info, const pmu_config* config,
	const pmu_read_args* args, const pmu_cpu_counts* cpus,
	const cpu_info* activeBefore, const cpu_info* activeAfter,
	bool perCPU, const char* what)
{
	const double seconds = args->elapsed / 1e6;
	uint64 cycles = 0;
	uint64 totals[PMU_MAX_EVENTS] = {0};
	for (uint32 cpu = 0; cpu < args->cpu_count; cpu++) {
		cycles += cpus[cpu].cycles;
		for (uint32 i = 0; i < args->event_count; i++)
			totals[i] += cpus[cpu].events[i];
	}
	bigtime_t active = 0;
	for (uint32 cpu = 0; cpu < args->cpu_count; cpu++)
		active += activeAfter[cpu].active_time - activeBefore[cpu].active_time;

	fprintf(stderr, "\n PMU counts, all %" B_PRIu32 " CPUs, %s%s: %s\n\n",
		args->cpu_count,
		(args->flags & PMU_EXCLUDE_KERNEL) != 0 ? "user mode only, "
			: (args->flags & PMU_EXCLUDE_USER) != 0 ? "kernel only, " : "",
		"events counted system-wide", what);

	char count[40], rate[32], label[24];
	fprintf(stderr, " %22s  %-20s %s", format_count(cycles, count,
		sizeof(count)), "cycles", format_rate(cycles / seconds, rate,
		sizeof(rate)));
	if (active > 0) {
		fprintf(stderr, "   # %.1f MHz while busy, CPUs busy %.1f %%",
			cycles / (double)active,
			100.0 * active / (args->elapsed * (double)args->cpu_count));
	}
	fprintf(stderr, "\n");

	const int inst = event_index(config, 0x08);
	const int cpuCycles = event_index(config, 0x11);
	for (uint32 i = 0; i < args->event_count; i++) {
		const uint16 number = config->events[i];
		fprintf(stderr, " %22s  %-20s %s", format_count(totals[i], count,
				sizeof(count)), event_label(number, label, sizeof(label)),
			format_rate(totals[i] / seconds, rate, sizeof(rate)));

		const uint64 base = cpuCycles >= 0 ? totals[cpuCycles] : cycles;
		int ratioOf = -1;
		switch (number) {
			case 0x08:
				if (base > 0)
					fprintf(stderr, "   # IPC %.3f", totals[i] / (double)base);
				break;
			case 0x03:
				ratioOf = event_index(config, 0x04);
				break;
			case 0x01:
				ratioOf = event_index(config, 0x14);
				break;
			case 0x17:
				ratioOf = event_index(config, 0x16);
				break;
			case 0x05:
				ratioOf = event_index(config, 0x25);
				break;
			case 0x10:
			case 0x22:
				ratioOf = event_index(config, 0x12);
				if (ratioOf < 0)
					ratioOf = event_index(config, 0x21);
				break;
			case 0x23:
			case 0x24:
				if (base > 0) {
					fprintf(stderr, "   # %.1f %% of cycles",
						100.0 * totals[i] / base);
				}
				break;
		}
		if (ratioOf >= 0 && totals[ratioOf] > 0) {
			fprintf(stderr, "   # %.2f %% of %s",
				100.0 * totals[i] / totals[ratioOf],
				event_label(config->events[ratioOf], label, sizeof(label)));
		} else if (number != 0x08 && inst >= 0 && totals[inst] > 0
			&& number != 0x11 && number != 0x23 && number != 0x24) {
			fprintf(stderr, "   # %.2f per 1k instructions",
				1000.0 * totals[i] / totals[inst]);
		}

		if (totals[i] == 0) {
			int implemented = event_implemented(info, number);
			fprintf(stderr, "   (%s)", implemented == 0
				? "not implemented by this CPU" : "counted nothing");
		}
		fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n %22.6f  seconds elapsed\n", seconds);

	if (perCPU) {
		fprintf(stderr, "\n %-20s", "per CPU");
		for (uint32 cpu = 0; cpu < args->cpu_count; cpu++)
			fprintf(stderr, " %18s%-2" B_PRIu32, "cpu", cpu);
		fprintf(stderr, "\n %-20s", "cycles");
		for (uint32 cpu = 0; cpu < args->cpu_count; cpu++) {
			fprintf(stderr, " %20s", format_count(cpus[cpu].cycles, count,
				sizeof(count)));
		}
		fprintf(stderr, "\n %-20s", "busy (scheduler)");
		for (uint32 cpu = 0; cpu < args->cpu_count; cpu++) {
			bigtime_t busy = activeAfter[cpu].active_time
				- activeBefore[cpu].active_time;
			fprintf(stderr, " %18.1f %%", 100.0 * busy / args->elapsed);
		}
		for (uint32 i = 0; i < args->event_count; i++) {
			fprintf(stderr, "\n %-20s", event_label(config->events[i], label,
				sizeof(label)));
			for (uint32 cpu = 0; cpu < args->cpu_count; cpu++) {
				fprintf(stderr, " %20s", format_count(cpus[cpu].events[i],
					count, sizeof(count)));
			}
		}
		fprintf(stderr, "\n");
	}

	if (args->max_sample_gap > 900000) {
		fprintf(stderr, "\n pmustat: the driver could not sample the counters "
			"for %.2f s: event counts may be short by multiples of 2^32\n",
			args->max_sample_gap / 1e6);
	}
	fprintf(stderr, "\n");
}


int
main(int argc, char** argv)
{
	const char* events = NULL;
	double seconds = -1;
	bool perCPU = false;
	bool list = false;
	uint32 flags = 0;

	int option;
	while ((option = getopt(argc, argv, "+e:a:Auklh")) != -1) {
		switch (option) {
			case 'e':
				events = optarg;
				break;
			case 'a':
				seconds = strtod(optarg, NULL);
				if (seconds <= 0) {
					fprintf(stderr, "pmustat: -a needs a time in seconds\n");
					return 2;
				}
				break;
			case 'A':
				perCPU = true;
				break;
			case 'u':
				flags |= PMU_EXCLUDE_KERNEL;
				break;
			case 'k':
				flags |= PMU_EXCLUDE_USER;
				break;
			case 'l':
				list = true;
				break;
			case 'h':
				printf(kUsage, argv[0], argv[0]);
				return 0;
			default:
				fprintf(stderr, kUsage, argv[0], argv[0]);
				return 2;
		}
	}
	if (flags == (PMU_EXCLUDE_KERNEL | PMU_EXCLUDE_USER)) {
		fprintf(stderr, "pmustat: -u and -k exclude each other\n");
		return 2;
	}
	if (!list && (seconds > 0) == (optind < argc)) {
		fprintf(stderr, kUsage, argv[0], argv[0]);
		return 2;
	}

	int fd = open("/dev/" PMU_DEVICE_NAME, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "pmustat: /dev/%s: %s\n", PMU_DEVICE_NAME,
			strerror(errno));
		return 1;
	}

	pmu_info info;
	if (ioctl(fd, PMU_GET_INFO, &info, sizeof(info)) != 0) {
		fprintf(stderr, "pmustat: PMU_GET_INFO: %s\n", strerror(errno));
		return 1;
	}
	if (list) {
		list_events(&info);
		return 0;
	}

	pmu_config config;
	memset(&config, 0, sizeof(config));
	config.flags = flags;
	if (events != NULL) {
		if (!parse_events(events, &config, info.event_counters))
			return 2;
	} else {
		for (size_t i = 0; i < sizeof(kDefaultEvents) / sizeof(uint16)
				&& i < info.event_counters; i++) {
			config.events[config.event_count++] = kDefaultEvents[i];
		}
	}
	for (uint32 i = 0; i < config.event_count; i++) {
		char label[24];
		if (event_implemented(&info, config.events[i]) == 0) {
			fprintf(stderr, "pmustat: %s (0x%02x): not implemented by this "
				"CPU\n", event_label(config.events[i], label, sizeof(label)),
				config.events[i]);
		}
	}

	pmu_cpu_counts* cpus = calloc(info.cpu_count, sizeof(pmu_cpu_counts));
	cpu_info* activeBefore = calloc(info.cpu_count, sizeof(cpu_info));
	cpu_info* activeAfter = calloc(info.cpu_count, sizeof(cpu_info));
	if (cpus == NULL || activeBefore == NULL || activeAfter == NULL) {
		fprintf(stderr, "pmustat: out of memory\n");
		return 1;
	}

	// what to call the run
	char what[256] = "";
	if (seconds > 0)
		snprintf(what, sizeof(what), "for %g s", seconds);
	else {
		for (int i = optind; i < argc; i++) {
			strlcat(what, i > optind ? " " : "'", sizeof(what));
			strlcat(what, argv[i], sizeof(what));
		}
		strlcat(what, "'", sizeof(what));
	}

	get_cpu_info(0, info.cpu_count, activeBefore);
	if (ioctl(fd, PMU_START, &config, sizeof(config)) != 0) {
		fprintf(stderr, "pmustat: PMU_START: %s%s\n", strerror(errno),
			errno == B_BAD_VALUE ? " (event numbers of this PMU go up to "
				"0x3ff, or 0xffff from PMUv3.1 on)" : "");
		return 1;
	}

	int exitCode = 0;
	if (seconds > 0) {
		snooze((bigtime_t)(seconds * 1e6));
	} else {
		pid_t child = fork();
		if (child < 0) {
			fprintf(stderr, "pmustat: fork: %s\n", strerror(errno));
			return 1;
		}
		if (child == 0) {
			execvp(argv[optind], argv + optind);
			fprintf(stderr, "pmustat: %s: %s\n", argv[optind],
				strerror(errno));
			_exit(127);
		}

		// as "perf stat": a Ctrl-C ends the command, then the counts are
		// printed
		signal(SIGINT, SIG_IGN);
		signal(SIGQUIT, SIG_IGN);
		int status;
		while (waitpid(child, &status, 0) < 0 && errno == EINTR)
			;
		if (WIFEXITED(status))
			exitCode = WEXITSTATUS(status);
		else if (WIFSIGNALED(status))
			exitCode = 128 + WTERMSIG(status);
	}

	pmu_read_args args;
	memset(&args, 0, sizeof(args));
	args.cpu_count = info.cpu_count;
	args.cpus = cpus;
	if (ioctl(fd, PMU_STOP, NULL, 0) != 0
		|| ioctl(fd, PMU_READ, &args, sizeof(args)) != 0) {
		fprintf(stderr, "pmustat: reading the counters: %s\n",
			strerror(errno));
		return 1;
	}
	get_cpu_info(0, info.cpu_count, activeAfter);
	close(fd);

	print_results(&info, &config, &args, cpus, activeBefore, activeAfter,
		perCPU, what);
	return exitCode;
}
