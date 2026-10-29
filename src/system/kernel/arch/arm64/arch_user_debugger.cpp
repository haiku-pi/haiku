/*
 * Copyright 2019-2026 Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */


#include <arch/user_debugger.h>

#include <string.h>

#include <cpu.h>
#include <debugger.h>
#include <interrupts.h>
#include <team.h>
#include <thread.h>
#include <util/AutoLock.h>

#include "arm_registers.h"


//#define TRACE_ARCH_USER_DEBUGGER
#ifdef TRACE_ARCH_USER_DEBUGGER
#	define TRACE(x) dprintf x
#else
#	define TRACE(x) ;
#endif

#define B_NO_MORE_BREAKPOINTS				B_BUSY
#define B_NO_MORE_WATCHPOINTS				B_BUSY
#define B_BAD_WATCHPOINT_ALIGNMENT			B_BAD_VALUE
#define B_WATCHPOINT_TYPE_NOT_SUPPORTED		B_NOT_SUPPORTED
#define B_WATCHPOINT_LENGTH_NOT_SUPPORTED	B_NOT_SUPPORTED
#define B_BREAKPOINT_NOT_FOUND				B_NAME_NOT_FOUND
#define B_WATCHPOINT_NOT_FOUND				B_NAME_NOT_FOUND
	// TODO: Make those real error codes.


/*	User debugging on AArch64.

	Software breakpoints are "brk #0" instructions (the kernel's breakpoint
	manager writes them); the exception leaves the PC at the instruction.
	Hardware breakpoints and watchpoints use the debug registers
	DBGB{V,C}R<n>_EL1 and DBGW{V,C}R<n>_EL1, set up to match at EL0 only, and
	single-stepping the software step of MDSCR_EL1.SS and SPSR_EL1.SS.
	MDSCR_EL1.KDE stays clear: the kernel never takes a breakpoint, watchpoint
	or step exception itself, so the registers need not be switched when
	entering the kernel. They are a team's (single-stepping a thread's)
	setting, and are loaded on the way back to userland, when the CPU has
	another one loaded (arm64_init_user_debug_at_kernel_exit()).

	A hardware breakpoint or watchpoint exception happens before the
	instruction executes. A thread that continues from it, or from any other
	debug event at the same PC, therefore executes that one instruction with
	its break- and watchpoints disabled, single-stepping, before they are
	enabled again (ARM64_THREAD_DEBUG_STEP_OVER) -- as the breakpoint manager
	expects of a hardware breakpoint it puts in place of a software one at the
	PC a thread continues from.
*/


// the software breakpoint instruction: brk #0, little endian
const uint8 kArm64SoftwareBreakpoint[4] = { 0x00, 0x00, 0x20, 0xd4 };
static const uint32 kSoftwareBreakpointInstruction = 0xd4200000;

int32 gArm64BreakpointCount = 0;
int32 gArm64WatchpointCount = 0;

static int64 sGeneration = 0;


#define DEBUG_REGISTER_CASE(n, valueRegister, controlRegister)	\
	case n:														\
		WRITE_SPECIALREG(valueRegister##n##_el1, value);		\
		WRITE_SPECIALREG(controlRegister##n##_el1, control);	\
		break;

#define DEBUG_REGISTER_CASES(valueRegister, controlRegister)	\
	DEBUG_REGISTER_CASE(0, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(1, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(2, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(3, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(4, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(5, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(6, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(7, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(8, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(9, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(10, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(11, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(12, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(13, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(14, valueRegister, controlRegister)		\
	DEBUG_REGISTER_CASE(15, valueRegister, controlRegister)


static void
write_breakpoint_registers(int32 index, uint64 value, uint64 control)
{
	switch (index) {
		DEBUG_REGISTER_CASES(dbgbvr, dbgbcr)
	}
}


static void
write_watchpoint_registers(int32 index, uint64 value, uint64 control)
{
	switch (index) {
		DEBUG_REGISTER_CASES(dbgwvr, dbgwcr)
	}
}


static inline uint64
new_generation()
{
	return atomic_add64(&sGeneration, 1) + 1;
}


static bool
has_breakpoints(const arch_team_debug_info& info)
{
	for (int32 i = 0; i < ARM64_MAX_BREAKPOINTS; i++) {
		if (info.breakpoints[i].control != 0)
			return true;
	}
	for (int32 i = 0; i < ARM64_MAX_WATCHPOINTS; i++) {
		if (info.watchpoints[i].control != 0)
			return true;
	}
	return false;
}


/*!	Loads the team's break- and watchpoints into the debug registers.
	Interrupts must be disabled and the team debug info lock be held.
*/
static void
install_breakpoints(const arch_team_debug_info& info)
{
	for (int32 i = 0; i < gArm64BreakpointCount; i++) {
		write_breakpoint_registers(i, (addr_t)info.breakpoints[i].address,
			info.breakpoints[i].control);
	}

	for (int32 i = 0; i < gArm64WatchpointCount; i++) {
		write_watchpoint_registers(i,
			ROUNDDOWN((addr_t)info.watchpoints[i].address, 8),
			info.watchpoints[i].control);
	}
}


static void
disable_breakpoints()
{
	for (int32 i = 0; i < gArm64BreakpointCount; i++)
		write_breakpoint_registers(i, 0, 0);
	for (int32 i = 0; i < gArm64WatchpointCount; i++)
		write_watchpoint_registers(i, 0, 0);
}


static iframe*
arm64_get_user_iframe(Thread* thread)
{
	iframe_stack* iframes = &thread->arch_info.iframes;

	// walk outwards from the innermost frame, skipping any kernel frames
	for (int32 i = iframes->index - 1; i >= 0; i--) {
		iframe* frame = iframes->frames[i];
		if ((frame->spsr & PSR_M_MASK) == PSR_M_EL0t)
			return frame;
	}

	return NULL;
}


static void
get_cpu_state(iframe* frame, debug_cpu_state* cpuState)
{
	// iframe stores x0-x28 in x[], with x29 (the frame pointer) separate
	memcpy(cpuState->x, frame->x, sizeof(frame->x));
	cpuState->x[29] = frame->fp;
	cpuState->lr = frame->lr;
	cpuState->sp = frame->sp;
	cpuState->elr = frame->elr;
	cpuState->spsr = frame->spsr;
}


/*!	Sets a break- or watchpoint (\a watchpoint) in the current team.
*/
static status_t
set_breakpoint(void* address, uint32 control, bool watchpoint)
{
	Thread* thread = thread_get_current_thread();

	InterruptsSpinLocker locker(thread->team->debug_info.lock);
	arch_team_debug_info& info = thread->team->debug_info.arch_info;

	arch_breakpoint* breakpoints
		= watchpoint ? info.watchpoints : info.breakpoints;
	int32 count = watchpoint ? gArm64WatchpointCount : gArm64BreakpointCount;

	int32 slot = -1;
	for (int32 i = 0; i < count; i++) {
		if (breakpoints[i].address == address && breakpoints[i].control != 0) {
			// already set: update it
			slot = i;
			break;
		}
		if (slot < 0 && breakpoints[i].control == 0)
			slot = i;
	}

	if (slot < 0)
		return watchpoint ? B_NO_MORE_WATCHPOINTS : B_NO_MORE_BREAKPOINTS;

	breakpoints[slot].address = address;
	breakpoints[slot].control = control;
	info.generation = new_generation();

	TRACE(("arm64 set_breakpoint(%p, %#" B_PRIx32 ", %d): slot %" B_PRId32
		"\n", address, control, watchpoint, slot));
	return B_OK;
}


static status_t
clear_breakpoint(void* address, bool watchpoint)
{
	Thread* thread = thread_get_current_thread();

	InterruptsSpinLocker locker(thread->team->debug_info.lock);
	arch_team_debug_info& info = thread->team->debug_info.arch_info;

	arch_breakpoint* breakpoints
		= watchpoint ? info.watchpoints : info.breakpoints;
	int32 count = watchpoint ? gArm64WatchpointCount : gArm64BreakpointCount;

	for (int32 i = 0; i < count; i++) {
		if (breakpoints[i].address == address && breakpoints[i].control != 0) {
			breakpoints[i].address = NULL;
			breakpoints[i].control = 0;
			info.generation = has_breakpoints(info) ? new_generation() : 0;
			return B_OK;
		}
	}

	return watchpoint ? B_WATCHPOINT_NOT_FOUND : B_BREAKPOINT_NOT_FOUND;
}


// #pragma mark - in-kernel public interface


void
arch_clear_team_debug_info(arch_team_debug_info* info)
{
	memset(info, 0, sizeof(*info));
}


void
arch_destroy_team_debug_info(arch_team_debug_info* info)
{
	arch_clear_team_debug_info(info);
}


void
arch_clear_thread_debug_info(arch_thread_debug_info* info)
{
	info->flags = 0;
	info->stop_address = 0;
}


void
arch_destroy_thread_debug_info(arch_thread_debug_info* info)
{
	arch_clear_thread_debug_info(info);
}


void
arch_update_thread_single_step()
{
	Thread* thread = thread_get_current_thread();
	iframe* frame = arm64_get_user_iframe(thread);
	if (frame == NULL)
		return;

	// Set or clear the step state the thread returns to userland with: with
	// SPSR_EL1.SS set it executes one instruction, then takes the software
	// step exception (MDSCR_EL1.SS is set on the way out).
	if ((thread->debug_info.flags & B_THREAD_DEBUG_SINGLE_STEP) != 0) {
		atomic_or(&thread->debug_info.arch_info.flags,
			ARM64_THREAD_DEBUG_SINGLE_STEP);
		frame->spsr |= PSR_SS;
	} else {
		atomic_and(&thread->debug_info.arch_info.flags,
			~ARM64_THREAD_DEBUG_SINGLE_STEP);
	}
}


void
arch_set_debug_cpu_state(const debug_cpu_state* cpuState)
{
	iframe* frame = arm64_get_user_iframe(thread_get_current_thread());
	if (frame == NULL)
		return;

	memcpy(frame->x, cpuState->x, sizeof(frame->x));
	frame->fp = cpuState->x[29];
	frame->lr = cpuState->lr;
	frame->sp = cpuState->sp;
	frame->elr = cpuState->elr;
	frame->spsr = (frame->spsr & ~(uint64)PSR_USER_SETTABLE)
		| (cpuState->spsr & PSR_USER_SETTABLE);
}


void
arch_get_debug_cpu_state(debug_cpu_state* cpuState)
{
	if (iframe* frame = arm64_get_user_iframe(thread_get_current_thread()))
		get_cpu_state(frame, cpuState);
}


status_t
arch_get_thread_debug_cpu_state(Thread* thread, debug_cpu_state* cpuState)
{
	iframe* frame = arm64_get_user_iframe(thread);
	if (frame == NULL)
		return B_BAD_VALUE;

	get_cpu_state(frame, cpuState);
	return B_OK;
}


status_t
arch_set_breakpoint(void* address)
{
	if (address == NULL || ((addr_t)address & 3) != 0)
		return B_BAD_VALUE;

	return set_breakpoint(address,
		DBGBCR_E | DBGBCR_PMC_EL0 | DBGBCR_BAS_A64, false);
}


status_t
arch_clear_breakpoint(void* address)
{
	return clear_breakpoint(address, false);
}


status_t
arch_set_watchpoint(void* address, uint32 type, int32 length)
{
	if (address == NULL)
		return B_BAD_VALUE;

	uint32 access;
	switch (type) {
		case B_DATA_READ_WATCHPOINT:
			access = DBGWCR_LSC_LOAD;
			break;
		case B_DATA_WRITE_WATCHPOINT:
			access = DBGWCR_LSC_STORE;
			break;
		case B_DATA_READ_WRITE_WATCHPOINT:
			access = DBGWCR_LSC_LOAD | DBGWCR_LSC_STORE;
			break;
		default:
			return B_WATCHPOINT_TYPE_NOT_SUPPORTED;
	}

	// The byte address select field covers the bytes of an aligned
	// doubleword: up to 8 bytes that do not cross one.
	if (length < 1 || length > 8)
		return B_WATCHPOINT_LENGTH_NOT_SUPPORTED;
	uint32 offset = (addr_t)address & 7;
	if (offset + length > 8)
		return B_BAD_WATCHPOINT_ALIGNMENT;

	uint32 byteSelect = ((1U << length) - 1) << offset;
	return set_breakpoint(address, DBGWCR_E | DBGWCR_PAC_EL0 | access
		| (byteSelect << DBGWCR_BAS_SHIFT), true);
}


status_t
arch_clear_watchpoint(void* address)
{
	return clear_breakpoint(address, true);
}


bool
arch_has_breakpoints(arch_team_debug_info* info)
{
	// Reading the generation is atomic, so we don't need to lock. The caller
	// has to ensure, that the info doesn't go away.
	return info->generation != 0;
}


// #pragma mark - arm64 implementation interface


/*!	Initializes the current CPU's debug state: unlocks the OS lock, which
	disables breakpoint, watchpoint and step exceptions, disables all break-
	and watchpoints (their reset state is unknown) and enables those
	exceptions for EL0 (MDSCR_EL1.MDE; not for EL1: KDE).
*/
void
arm64_init_user_debug_percpu(int cpu)
{
	uint64 features = READ_SPECIALREG(ID_AA64DFR0_EL1);
	int32 breakpoints = ID_AA64DFR0_BRPS(features);
	int32 watchpoints = ID_AA64DFR0_WRPS(features);

	if (cpu == 0) {
		gArm64BreakpointCount = breakpoints;
		gArm64WatchpointCount = watchpoints;
		dprintf("arm64: %" B_PRId32 " hardware breakpoints, %" B_PRId32
			" watchpoints\n", breakpoints, watchpoints);
	} else {
		// use those every CPU has
		if (breakpoints < gArm64BreakpointCount)
			gArm64BreakpointCount = breakpoints;
		if (watchpoints < gArm64WatchpointCount)
			gArm64WatchpointCount = watchpoints;
	}

	WRITE_SPECIALREG(OSLAR_EL1, 0);
	WRITE_SPECIALREG(OSDLR_EL1, 0);
	arm64_isb();

	for (int32 i = 0; i < breakpoints; i++)
		write_breakpoint_registers(i, 0, 0);
	for (int32 i = 0; i < watchpoints; i++)
		write_watchpoint_registers(i, 0, 0);

	WRITE_SPECIALREG(MDSCR_EL1, MDSCR_MDE);
	arm64_isb();

	gCPU[cpu].arch.user_debug_generation = 0;
	gCPU[cpu].arch.user_single_step = false;
}


/*!	Sets up the CPU's debug registers and MDSCR_EL1.SS for the current thread
	returning to userland with \a frame, if they are not set so already.
	Called on every return to userland, with interrupts disabled; the ERET
	that follows makes the register writes take effect.
*/
void
arm64_init_user_debug_at_kernel_exit(iframe* frame)
{
	Thread* thread = thread_get_current_thread();
	arch_cpu_info& cpu = thread->cpu->arch;
	arch_thread_debug_info& threadInfo = thread->debug_info.arch_info;

	bool breakpointsDefined
		= (thread->flags & THREAD_FLAGS_BREAKPOINTS_DEFINED) != 0;
	if (!breakpointsDefined && threadInfo.flags == 0
		&& cpu.user_debug_generation == 0 && !cpu.user_single_step) {
		// the common case: no debugging
		return;
	}

	int32 flags = atomic_get(&threadInfo.flags);
	if ((flags & ARM64_THREAD_DEBUG_STOPPED) != 0) {
		// The thread continues from a debug event. If it does so at the
		// instruction it stopped at, a break- or watchpoint might be on it:
		// execute it with them disabled.
		atomic_and(&threadInfo.flags, ~ARM64_THREAD_DEBUG_STOPPED);
		if (breakpointsDefined && frame->elr == threadInfo.stop_address) {
			flags = atomic_or(&threadInfo.flags, ARM64_THREAD_DEBUG_STEP_OVER)
				| ARM64_THREAD_DEBUG_STEP_OVER;
			frame->spsr |= PSR_SS;
		}
	}

	uint64 generation = 0;
	if (breakpointsDefined && (flags & ARM64_THREAD_DEBUG_STEP_OVER) == 0) {
		arch_team_debug_info& teamInfo = thread->team->debug_info.arch_info;
		generation = atomic_get64((int64*)&teamInfo.generation);
		if (generation != 0 && generation != cpu.user_debug_generation) {
			SpinLocker locker(thread->team->debug_info.lock);
			generation = teamInfo.generation;
			if (generation != 0) {
				install_breakpoints(teamInfo);
				cpu.user_debug_generation = generation;
			}
		}
	}

	if (generation == 0 && cpu.user_debug_generation != 0) {
		disable_breakpoints();
		cpu.user_debug_generation = 0;
	}

	bool singleStep = (flags
		& (ARM64_THREAD_DEBUG_SINGLE_STEP | ARM64_THREAD_DEBUG_STEP_OVER)) != 0;
	if (singleStep != cpu.user_single_step) {
		WRITE_SPECIALREG(MDSCR_EL1,
			MDSCR_MDE | (singleStep ? MDSCR_SS : 0));
		cpu.user_single_step = singleStep;
	}
}


/*!	Handles a debug exception from userland: a hardware breakpoint, a
	watchpoint, a software step, or a software breakpoint ("brk #0") while
	the team is being debugged. Returns \c false for other BRK instructions
	(left to the general exception handling, as before user debugging).
	Interrupts are disabled and will be enabled by the function, if it handles
	the exception.
*/
bool
arm64_handle_user_debug_exception(iframe* frame)
{
	Thread* thread = thread_get_current_thread();
	arch_thread_debug_info& threadInfo = thread->debug_info.arch_info;

	switch (ESR_ELx_EXCEPTION(frame->esr)) {
		case EXCP_BRK:
		{
			if (ISS_BRK_COMMENT(frame->esr) != 0
				|| (atomic_get(&thread->team->debug_info.flags)
					& B_TEAM_DEBUG_DEBUGGER_INSTALLED) == 0) {
				return false;
			}

			addr_t address = frame->elr;
			threadInfo.stop_address = address;
			atomic_and(&threadInfo.flags, ~ARM64_THREAD_DEBUG_STEP_OVER);
			atomic_or(&threadInfo.flags, ARM64_THREAD_DEBUG_STOPPED);

			enable_interrupts();
			user_debug_breakpoint_hit(true);

			// The breakpoint manager replaces its software breakpoint by a
			// hardware one at the PC the thread continues from. A brk #0
			// still there is the program's own: continue after it.
			uint32 instruction;
			if (frame->elr == address
				&& user_memcpy(&instruction, (void*)address,
					sizeof(instruction)) == B_OK
				&& instruction == kSoftwareBreakpointInstruction) {
				frame->elr += 4;
				atomic_and(&threadInfo.flags, ~ARM64_THREAD_DEBUG_STOPPED);
			}
			return true;
		}

		case EXCP_BRKPT_EL0:
		case EXCP_WATCHPT_EL0:
		{
			threadInfo.stop_address = frame->elr;
			atomic_and(&threadInfo.flags, ~ARM64_THREAD_DEBUG_STEP_OVER);
			atomic_or(&threadInfo.flags, ARM64_THREAD_DEBUG_STOPPED);

			enable_interrupts();
			if (ESR_ELx_EXCEPTION(frame->esr) == EXCP_WATCHPT_EL0)
				user_debug_watchpoint_hit();
			else
				user_debug_breakpoint_hit(false);
			return true;
		}

		case EXCP_SOFTSTP_EL0:
		{
			int32 flags = atomic_and(&threadInfo.flags,
				~ARM64_THREAD_DEBUG_STEP_OVER);
			if ((flags & ARM64_THREAD_DEBUG_SINGLE_STEP) == 0) {
				// the end of a step over a break- or watchpoint (or a step
				// nobody wants anymore): just continue
				return true;
			}

			threadInfo.stop_address = frame->elr;
			atomic_or(&threadInfo.flags, ARM64_THREAD_DEBUG_STOPPED);

			enable_interrupts();
			user_debug_single_stepped();
			return true;
		}
	}

	return false;
}
