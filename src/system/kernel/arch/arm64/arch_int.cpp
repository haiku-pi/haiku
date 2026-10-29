/*
 * Copyright 2019-2026 Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */
#include <interrupts.h>

#include <arch/smp.h>
#include <arch/user_debugger.h>
#include <boot/kernel_args.h>
#include <device_manager.h>
#include <kscheduler.h>
#include <ksyscalls.h>
#include <interrupt_controller.h>
#include <smp.h>
#include <thread.h>
#include <timer.h>
#include <util/AutoLock.h>
#include <util/DoublyLinkedList.h>
#include <util/kernel_cpp.h>
#include <vm/vm.h>
#include <vm/vm_priv.h>
#include <vm/VMAddressSpace.h>
#include "syscall_numbers.h"
#include "VMSAv8TranslationMap.h"
#include <string.h>

#include "arch_int_gicv2.h"
#include "arch_int_gicv3.h"
#include "soc.h"

#define TRACE_ARCH_INT
#ifdef TRACE_ARCH_INT
#	define TRACE(x) dprintf x
#else
#	define TRACE(x) ;
#endif

//#define TRACE_ARCH_INT_IFRAMES

// An iframe stack used in the early boot process when we don't have
// threads yet.
struct iframe_stack gBootFrameStack;


void
arch_int_enable_io_interrupt(int32 irq)
{
	InterruptController *ic = InterruptController::Get();
	if (ic != NULL)
		ic->EnableInterrupt(irq);
}


void
arch_int_disable_io_interrupt(int32 irq)
{
	InterruptController *ic = InterruptController::Get();
	if (ic != NULL)
		ic->DisableInterrupt(irq);
}


int32
arch_int_assign_to_cpu(int32 irq, int32 cpu)
{
	// Not yet supported.
	return 0;
}


static void
print_iframe(const char *event, struct iframe *frame)
{
	if (event)
		dprintf("Exception: %s\n", event);

	dprintf("ELR=%016lx SPSR=%016lx\n",
		frame->elr, frame->spsr);
	dprintf("LR=%016lx  SP  =%016lx\n",
		frame->lr, frame->sp);
}


status_t
arch_int_init(kernel_args *args)
{
	return B_OK;
}


status_t
arch_int_init_post_vm(kernel_args *args)
{
	InterruptController *ic = NULL;
	if (strcmp(args->arch_args.interrupt_controller.kind, INTC_KIND_GICV2) == 0) {
		ic = new(std::nothrow) GICv2InterruptController(
			args->arch_args.interrupt_controller.regs1.start,
			args->arch_args.interrupt_controller.regs2.start);
	}
	if (strcmp(args->arch_args.interrupt_controller.kind, INTC_KIND_GICV3) == 0) {
		ic = new(std::nothrow)
			GICv3InterruptController(args->arch_args.interrupt_controller.regs1.start,
				args->arch_args.interrupt_controller.regs2.start, args->num_cpus);
	}

	if (ic == NULL)
		return B_ERROR;

	return B_OK;
}


status_t
arch_int_init_io(kernel_args* args)
{
	return B_OK;
}


status_t
arch_int_init_post_device_manager(struct kernel_args *args)
{
	return B_ENTRY_NOT_FOUND;
}


// TODO: reuse things from VMSAv8TranslationMap


static int page_bits = 12;

static uint64_t*
TableFromPa(phys_addr_t pa)
{
	return reinterpret_cast<uint64_t*>(KERNEL_PMAP_BASE + pa);
}


static bool
fixup_entry(phys_addr_t ptPa, int level, addr_t va, bool wr)
{
	int tableBits = page_bits - 3;
	uint64_t tableMask = (1UL << tableBits) - 1;

	int shift = tableBits * (3 - level) + page_bits;
	uint64_t entrySize = 1UL << shift;
	uint64_t entryMask = entrySize - 1;

	int index = (va >> shift) & tableMask;

	uint64_t *pte = &TableFromPa(ptPa)[index];
	uint64_t oldPte = atomic_get64((int64*)pte);

	int type = oldPte & kPteTypeMask;
	uint64_t addr = oldPte & kPteAddrMask;

	if ((level == 3 && type == kPteTypeL3Page) || (level < 3 && type == kPteTypeL12Block)) {
		if (!wr && (oldPte & kAttrAF) == 0) {
			uint64_t newPte = oldPte | kAttrAF;
            if ((uint64_t)atomic_test_and_set64((int64*)pte, newPte, oldPte) != oldPte)
				return true; // If something changed, handle it by taking another fault
			asm("dsb ishst");
			asm("isb");
			return true;
		}
		if (wr && (oldPte & kAttrSWDBM) != 0 && (oldPte & kAttrAPReadOnly) != 0) {
			uint64_t newPte = oldPte & ~kAttrAPReadOnly;
            if ((uint64_t)atomic_test_and_set64((int64*)pte, newPte, oldPte) != oldPte)
				return true;

			uint64_t asid = READ_SPECIALREG(TTBR0_EL1) >> 48;
			flush_va_if_accessed(oldPte, va, asid);

			return true;
		}
	} else if (level < 3 && type == kPteTypeL012Table) {
		return fixup_entry(addr, level + 1, va, wr);
	}

	return false;
}


void
after_exception()
{
	Thread* thread = thread_get_current_thread();
	cpu_status state = disable_interrupts();
	if (thread->post_interrupt_callback != NULL) {
		void (*callback)(void*) = thread->post_interrupt_callback;
		void* data = thread->post_interrupt_data;

		thread->post_interrupt_callback = NULL;
		thread->post_interrupt_data = NULL;

		restore_interrupts(state);

		callback(data);
	} else if (thread->cpu->invoke_scheduler) {
		SpinLocker schedulerLocker(thread->scheduler_lock);
		scheduler_reschedule(B_THREAD_READY);
		schedulerLocker.Unlock();
		restore_interrupts(state);
	}
}


class InterruptScope {
public:
	InterruptScope(struct iframe *iframe) {
		fThread = thread_get_current_thread();
		if (fThread) {
			arm64_push_iframe(&fThread->arch_info.iframes, iframe);
			// From user mode: the time since the thread last left the kernel
			// was user time (the destructor's thread_at_kernel_exit() counts
			// the time since this as kernel time), for every exception, not
			// only system calls: else a thread interrupted by its timer had
			// all its user time counted as kernel time
			if ((iframe->spsr & PSR_M_MASK) == PSR_M_EL0t)
				thread_at_kernel_entry(system_time());
		} else
			arm64_push_iframe(&gBootFrameStack, iframe);
	}

	virtual ~InterruptScope() {
		if (fThread == NULL) {
			arm64_pop_iframe(&gBootFrameStack);
			return;
		}

		iframe *frame = fThread->arch_info.iframes.frames[fThread->arch_info.iframes.index - 1];
		bool isUser = (frame->spsr & PSR_M_MASK) == PSR_M_EL0t;
		if (isUser) {
			disable_interrupts();
			atomic_and(&thread_get_current_thread()->flags, ~THREAD_FLAGS_SYSCALL_RESTARTED);
			if ((thread_get_current_thread()->flags
				& (THREAD_FLAGS_SIGNALS_PENDING
				| THREAD_FLAGS_DEBUG_THREAD
				| THREAD_FLAGS_TRAP_FOR_CORE_DUMP)) != 0) {
				enable_interrupts();
				thread_at_kernel_exit();
			} else {
				thread_at_kernel_exit_no_signals();
			}
			if ((THREAD_FLAGS_RESTART_SYSCALL & thread_get_current_thread()->flags) != 0) {
				atomic_and(&thread_get_current_thread()->flags, ~THREAD_FLAGS_RESTART_SYSCALL);
				atomic_or(&thread_get_current_thread()->flags, THREAD_FLAGS_SYSCALL_RESTARTED);

				// Restore old syscall argument and go
				// back an instruction
				frame->x[0] = thread_get_current_thread()->arch_info.old_x0;
				frame->elr -= 4;
			}
			arm64_init_user_debug_at_kernel_exit(frame);
		}
		arm64_pop_iframe(&fThread->arch_info.iframes);
	}
private:
	Thread* fThread;
};


/*!	Emulates a user read (MRS) of a register of the ID space (op0 3, op1 0,
	CRn 0), so that programs can read the CPU's features as on Linux: MIDR_EL1
	and the AArch64 feature registers are passed on, MPIDR_EL1 reads as on a
	uniprocessor, every other register of the space as 0. An ARMv8.0 CPU makes
	such a read an undefined instruction; with FEAT_IDST it is trapped (EC
	0x18). Returns whether the instruction was emulated.
*/
static bool
emulate_user_id_register_read(iframe* frame)
{
	uint32 crm, op2, rt;
	if (ESR_ELx_EXCEPTION(frame->esr) == EXCP_MSR) {
		uint64 iss = frame->esr & ESR_ELx_ISS_MASK;
		bool read = (iss & 1) != 0;
		if (!read || ((iss >> 20) & 0x3) != 3 || ((iss >> 14) & 0x7) != 0
			|| ((iss >> 10) & 0xf) != 0) {
			return false;
		}
		op2 = (iss >> 17) & 0x7;
		rt = (iss >> 5) & 0x1f;
		crm = (iss >> 1) & 0xf;
	} else {
		// reading the instruction may fault, as user page faults do with
		// interrupts on (the user context had them on)
		enable_interrupts();
		uint32 instruction;
		if (!IS_USER_ADDRESS(frame->elr)
			|| user_memcpy(&instruction, (void*)frame->elr, sizeof(instruction)) != B_OK
			|| (instruction & 0xfffff000) != 0xd5380000) {
			// not MRS Xt, S3_0_C0_Cm_op2
			return false;
		}
		crm = (instruction >> 8) & 0xf;
		op2 = (instruction >> 5) & 0x7;
		rt = instruction & 0x1f;
	}

	// unallocated encodings of the space read as zero (RAZ), so all of these
	// can be read on any ARMv8 CPU
	uint64 value = 0;
	switch ((crm << 3) | op2) {
		case (0 << 3) | 0: value = READ_SPECIALREG(MIDR_EL1); break;
		case (0 << 3) | 5: value = 1UL << 31; break;
			// MPIDR_EL1: its RES1 bit, affinity 0
		case (4 << 3) | 0: value = READ_SPECIALREG(s3_0_c0_c4_0); break;
		case (4 << 3) | 1: value = READ_SPECIALREG(s3_0_c0_c4_1); break;
		case (4 << 3) | 4: value = READ_SPECIALREG(s3_0_c0_c4_4); break;
		case (5 << 3) | 0: value = READ_SPECIALREG(s3_0_c0_c5_0); break;
		case (5 << 3) | 1: value = READ_SPECIALREG(s3_0_c0_c5_1); break;
		case (6 << 3) | 0: value = READ_SPECIALREG(s3_0_c0_c6_0); break;
		case (6 << 3) | 1: value = READ_SPECIALREG(s3_0_c0_c6_1); break;
		case (6 << 3) | 2: value = READ_SPECIALREG(s3_0_c0_c6_2); break;
		case (7 << 3) | 0: value = READ_SPECIALREG(s3_0_c0_c7_0); break;
		case (7 << 3) | 1: value = READ_SPECIALREG(s3_0_c0_c7_1); break;
		case (7 << 3) | 2: value = READ_SPECIALREG(s3_0_c0_c7_2); break;
	}

	if (rt < 29)
		frame->x[rt] = value;
	else if (rt == 29)
		frame->fp = value;
	else if (rt == 30)
		frame->lr = value;
	// rt 31: xzr
	frame->elr += 4;
	return true;
}


extern "C" void
do_sync_handler(iframe * frame)
{
#ifdef TRACE_ARCH_INT_IFRAMES
	print_iframe("Sync abort", frame);
#endif

	InterruptScope scope(frame);
	// What a user thread gets for an exception that is not handled below
	debug_exception_type exceptionType = B_GENERAL_PROTECTION_FAULT;
	uint32 signalNumber = SIGILL;
	int32 signalCode = ILL_ILLOPC;
	uint64 signalAddress = frame->elr;

	bool isUser = (frame->spsr & PSR_M_MASK) == PSR_M_EL0t;
	bool isExec = false;
	switch (ESR_ELx_EXCEPTION(frame->esr)) {
		case EXCP_UNKNOWN:
			if (isUser && emulate_user_id_register_read(frame))
				return;
			exceptionType = B_INVALID_OPCODE_EXCEPTION;
			break;
		case EXCP_MSR:
			if (isUser && emulate_user_id_register_read(frame))
				return;
			exceptionType = B_INVALID_OPCODE_EXCEPTION;
			break;
		case EXCP_BRK:
		case EXCP_BRKPT_EL0:
		case EXCP_SOFTSTP_EL0:
		case EXCP_WATCHPT_EL0:
			if (isUser && arm64_handle_user_debug_exception(frame))
				return;
			break;
		case EXCP_PC_ALIGN:
			exceptionType = B_ALIGNMENT_EXCEPTION;
			signalNumber = SIGBUS;
			signalCode = BUS_ADRALN;
			signalAddress = frame->elr;
			break;
		case EXCP_INSN_ABORT_L:
		case EXCP_INSN_ABORT:
			isExec = true;
		case EXCP_DATA_ABORT_L:
		case EXCP_DATA_ABORT:
		{
			bool write = (frame->esr & ISS_DATA_WnR) != 0;
			bool known = false;

			// A user fault at address 0 reaches the program as a fault at 0 (SIGSEGV's si_addr): when FAR
			// does not hold the faulting address (ESR's FnV bit, an external abort), that is wrong, and
			// only ESR tells
			if (isUser && frame->far == 0) {
				dprintf("arm64: thread %" B_PRId32 ": user %s abort with FAR 0, ESR %#lx, ELR %#lx\n",
					thread_get_current_thread_id(), isExec ? "instruction" : "data", frame->esr,
					frame->elr);
			}

			int initialLevel = VMSAv8TranslationMap::CalcStartLevel(48, 12);
			phys_addr_t ptPa;
			bool addrType = (frame->far & (1UL << 63)) != 0;
			if (addrType)
				ptPa = READ_SPECIALREG(TTBR1_EL1);
			else
				ptPa = READ_SPECIALREG(TTBR0_EL1);
			ptPa &= kTtbrBasePhysAddrMask;

			switch (frame->esr & ISS_DATA_DFSC_MASK) {
				case ISS_DATA_DFSC_TF_L0:
				case ISS_DATA_DFSC_TF_L1:
				case ISS_DATA_DFSC_TF_L2:
				case ISS_DATA_DFSC_TF_L3:
					known = true;
				break;

				case ISS_DATA_DFSC_AFF_L1:
				case ISS_DATA_DFSC_AFF_L2:
				case ISS_DATA_DFSC_AFF_L3:
					known = true;
					if (fixup_entry(ptPa, initialLevel, frame->far, false))
						return;
				break;

				case ISS_DATA_DFSC_PF_L1:
				case ISS_DATA_DFSC_PF_L2:
				case ISS_DATA_DFSC_PF_L3:
					known = true;
					if (write && fixup_entry(ptPa, initialLevel, frame->far, true))
						return;
				break;
			}

			if (!known) {
				if ((frame->esr & ISS_DATA_DFSC_MASK) == ISS_DATA_DFSC_ALIGN) {
					exceptionType = B_ALIGNMENT_EXCEPTION;
					signalNumber = SIGBUS;
					signalCode = BUS_ADRALN;
				} else {
					exceptionType = B_SEGMENT_VIOLATION;
					signalNumber = SIGSEGV;
					signalCode = SEGV_ACCERR;
				}
				signalAddress = frame->far;
				break;
			}

			if (debug_debugger_running()) {
				Thread* thread = thread_get_current_thread();
				if (thread != NULL) {
					cpu_ent* cpu = &gCPU[smp_get_current_cpu()];
					if (cpu->fault_handler != 0) {
						debug_set_page_fault_info(frame->far, frame->elr,
							write ? DEBUG_PAGE_FAULT_WRITE : 0);
						frame->elr = cpu->fault_handler;
						frame->sp = cpu->fault_handler_stack_pointer;
						return;
					}
				}
			}

			Thread *thread = thread_get_current_thread();
			ASSERT(thread);

			if ((frame->spsr & PSR_I) != 0) {
				// interrupts disabled
				uintptr_t handler = reinterpret_cast<uintptr_t>(thread->fault_handler);
				if (thread->fault_handler != 0) {
					frame->elr = handler;
					return;
				}
			} else if (thread->page_faults_allowed != 0) {
				enable_interrupts();
				addr_t ret = 0;
				vm_page_fault(frame->far, frame->elr, write, isExec, isUser, &ret);
				if (ret != 0)
					frame->elr = ret;
				return;
			}

			panic("unhandled pagefault! FAR=%lx ELR=%lx ESR=%lx",
				frame->far, frame->elr, frame->esr);
			break;
		}

		case EXCP_SVC64:
		{
			uint32 syscall = (frame->esr & 0xffff);
			uint32 count = kExtendedSyscallInfos[syscall].parameter_count;

			uint64_t args[20];
			if (count > 20) {
				frame->x[0] = B_ERROR;
				return;
			}

			memset(args, 0, sizeof(args));
			memcpy(args, frame->x, (count < 8 ? count : 8) * 8);

			if (count > 8) {
				if (!IS_USER_ADDRESS(frame->sp)
					|| user_memcpy(&args[8], (void*)frame->sp, (count - 8) * 8) != B_OK) {
					frame->x[0] = B_BAD_ADDRESS;
					return;
				}
			}

			thread_get_current_thread()->arch_info.old_x0 = frame->x[0];

			enable_interrupts();
			syscall_dispatcher(syscall, (void*)args, &frame->x[0]);
			return;
		}
	}

	if (isUser) {
		struct sigaction action;
		Thread* thread = thread_get_current_thread();

		enable_interrupts();

		bool handled = sigaction(signalNumber, NULL, &action) == 0
			&& action.sa_handler != SIG_DFL && action.sa_handler != SIG_IGN;
		// only the exceptions that end the program: one that handles the
		// signal may take thousands (an x86 emulator's misaligned accesses,
		// Wine's write watches), and each line costs the serial line's time
		if (!handled) {
			dprintf("arm64: thread %" B_PRId32 " (%s): exception ESR %#lx (EC %#lx) "
				"FAR %#lx ELR %#lx -> signal %" B_PRIu32 "\n", thread->id, thread->name,
				frame->esr, (frame->esr >> 26) & 0x3f, frame->far, frame->elr,
				signalNumber);
		}

		if (handled || user_debug_exception_occurred(exceptionType, signalNumber)) {
			Signal signal(signalNumber, signalCode, B_ERROR,
				thread->team->id);
			signal.SetAddress((void*)signalAddress);
			send_fault_signal(thread, signal);
		}
	} else {
		panic("unhandled exception! FAR=%lx ELR=%lx ESR=%lx (EC=%lx)",
			frame->far, frame->elr, frame->esr, (frame->esr >> 26) & 0x3f);
	}
}


extern "C" void
do_error_handler(iframe * frame)
{
#ifdef TRACE_ARCH_INT_IFRAMES
	print_iframe("Error", frame);
#endif

	InterruptScope scope(frame);

	panic("unhandled error! FAR=%lx ELR=%lx ESR=%lx", frame->far, frame->elr, frame->esr);
}


extern "C" void
do_irq_handler(iframe * frame)
{
#ifdef TRACE_ARCH_INT_IFRAMES
	print_iframe("IRQ", frame);
#endif

	InterruptScope scope(frame);

	InterruptController *ic = InterruptController::Get();
	if (ic != NULL)
		ic->HandleInterrupt();

	after_exception();
}


extern "C" void
do_fiq_handler(iframe * frame)
{
#ifdef TRACE_ARCH_INT_IFRAMES
	print_iframe("FIQ", frame);
#endif

	InterruptScope scope(frame);

	panic("do_fiq_handler");
}
