/*
 * Copyright 2018, Jaroslaw Pelczar <jarek@jpelczar.com>
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _KERNEL_ARCH_ARM64_ARCH_USER_DEBUGGER_H_
#define _KERNEL_ARCH_ARM64_ARCH_USER_DEBUGGER_H_


// the most break- and watchpoint register pairs the architecture defines
#define ARM64_MAX_BREAKPOINTS	16
#define ARM64_MAX_WATCHPOINTS	16


struct arch_breakpoint {
	void*	address;	// as requested; NULL, if unused
	uint32	control;	// the DBGBCR<n>_EL1/DBGWCR<n>_EL1 value
};


struct arch_team_debug_info {
	struct arch_breakpoint	breakpoints[ARM64_MAX_BREAKPOINTS];
	struct arch_breakpoint	watchpoints[ARM64_MAX_WATCHPOINTS];

	// identifies this set of break- and watchpoints (a CPU that has it
	// loaded need not load it again); 0, if none is set
	uint64					generation;
};


// arch_thread_debug_info::flags
enum {
	// the debugger wants the thread to execute a single instruction
	ARM64_THREAD_DEBUG_SINGLE_STEP	= 0x01,
	// the thread is continuing from a debug event at stop_address
	ARM64_THREAD_DEBUG_STOPPED		= 0x02,
	// the thread executes the instruction at stop_address with the break-
	// and watchpoints disabled, so it does not hit the same one again
	ARM64_THREAD_DEBUG_STEP_OVER	= 0x04,
};


struct arch_thread_debug_info {
	int32	flags;
	addr_t	stop_address;
};


#ifdef __cplusplus
extern "C" {
#endif

struct iframe;

extern int32 gArm64BreakpointCount;
extern int32 gArm64WatchpointCount;
extern const uint8 kArm64SoftwareBreakpoint[4];

void arm64_init_user_debug_percpu(int cpu);
void arm64_init_user_debug_at_kernel_exit(struct iframe* frame);
bool arm64_handle_user_debug_exception(struct iframe* frame);

#ifdef __cplusplus
}
#endif


// Feature macros we're supposed to define.
#define DEBUG_MAX_BREAKPOINTS				gArm64BreakpointCount
#define DEBUG_MAX_WATCHPOINTS				gArm64WatchpointCount
#define DEBUG_SOFTWARE_BREAKPOINT			kArm64SoftwareBreakpoint
#define DEBUG_SOFTWARE_BREAKPOINT_SIZE		4


#endif /* _KERNEL_ARCH_ARM64_ARCH_USER_DEBUGGER_H_ */
