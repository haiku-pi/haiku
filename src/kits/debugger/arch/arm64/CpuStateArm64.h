/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef CPU_STATE_ARM64_H
#define CPU_STATE_ARM64_H

#include <bitset>

#include <debugger.h>

#include "CpuState.h"


enum {
	ARM64_REGISTER_X0 = 0,
	// x1 - x28 follow
	ARM64_REGISTER_FP = 29,		// x29
	ARM64_REGISTER_LR = 30,		// x30
	ARM64_REGISTER_SP,
	ARM64_REGISTER_PC,
	ARM64_REGISTER_PSTATE,

	ARM64_REGISTER_COUNT
};


class CpuStateArm64 : public CpuState {
public:
								CpuStateArm64();
								CpuStateArm64(const arm64_debug_cpu_state& state);
	virtual						~CpuStateArm64();

	virtual	status_t			Clone(CpuState*& _clone) const;

	virtual	status_t			UpdateDebugState(void* state, size_t size)
									const;

	virtual	target_addr_t		InstructionPointer() const;
	virtual void				SetInstructionPointer(target_addr_t address);

	virtual	target_addr_t		StackFramePointer() const;
	virtual	target_addr_t		StackPointer() const;
	virtual	bool				GetRegisterValue(const Register* reg,
									BVariant& _value) const;
	virtual	bool				SetRegisterValue(const Register* reg,
									const BVariant& value);

			bool				IsRegisterSet(int32 index) const;

			uint64				IntRegisterValue(int32 index) const;
			void				SetIntRegister(int32 index, uint64 value);

private:
	typedef std::bitset<ARM64_REGISTER_COUNT> RegisterBitSet;

private:
			uint64				fRegisters[ARM64_REGISTER_COUNT];
			RegisterBitSet		fSetRegisters;
};


#endif	// CPU_STATE_ARM64_H
