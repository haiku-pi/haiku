/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

#include "CpuStateArm64.h"

#include <new>

#include <string.h>

#include "Register.h"


CpuStateArm64::CpuStateArm64()
	:
	fSetRegisters()
{
	memset(fRegisters, 0, sizeof(fRegisters));
}


CpuStateArm64::CpuStateArm64(const arm64_debug_cpu_state& state)
	:
	fSetRegisters()
{
	memset(fRegisters, 0, sizeof(fRegisters));

	for (int32 i = 0; i < 30; i++)
		SetIntRegister(ARM64_REGISTER_X0 + i, state.x[i]);
	SetIntRegister(ARM64_REGISTER_LR, state.lr);
	SetIntRegister(ARM64_REGISTER_SP, state.sp);
	SetIntRegister(ARM64_REGISTER_PC, state.elr);
	SetIntRegister(ARM64_REGISTER_PSTATE, state.spsr);
}


CpuStateArm64::~CpuStateArm64()
{
}


status_t
CpuStateArm64::Clone(CpuState*& _clone) const
{
	CpuStateArm64* newState = new(std::nothrow) CpuStateArm64();
	if (newState == NULL)
		return B_NO_MEMORY;

	memcpy(newState->fRegisters, fRegisters, sizeof(fRegisters));
	newState->fSetRegisters = fSetRegisters;

	_clone = newState;
	return B_OK;
}


status_t
CpuStateArm64::UpdateDebugState(void* state, size_t size) const
{
	if (size != sizeof(arm64_debug_cpu_state))
		return B_BAD_VALUE;

	arm64_debug_cpu_state* arm64State = (arm64_debug_cpu_state*)state;

	for (int32 i = 0; i < 30; i++)
		arm64State->x[i] = IntRegisterValue(ARM64_REGISTER_X0 + i);
	arm64State->lr = IntRegisterValue(ARM64_REGISTER_LR);
	arm64State->sp = IntRegisterValue(ARM64_REGISTER_SP);
	arm64State->elr = IntRegisterValue(ARM64_REGISTER_PC);
	arm64State->spsr = IntRegisterValue(ARM64_REGISTER_PSTATE);

	return B_OK;
}


target_addr_t
CpuStateArm64::InstructionPointer() const
{
	// A caller's state built by unwinding a frame has no pc: AArch64's DWARF
	// return address column is the link register, so the return address --
	// the caller's instruction pointer -- is in lr.
	if (IsRegisterSet(ARM64_REGISTER_PC))
		return IntRegisterValue(ARM64_REGISTER_PC);
	return IsRegisterSet(ARM64_REGISTER_LR)
		? IntRegisterValue(ARM64_REGISTER_LR) : 0;
}


void
CpuStateArm64::SetInstructionPointer(target_addr_t address)
{
	SetIntRegister(ARM64_REGISTER_PC, address);
}


target_addr_t
CpuStateArm64::StackFramePointer() const
{
	return IsRegisterSet(ARM64_REGISTER_FP)
		? IntRegisterValue(ARM64_REGISTER_FP) : 0;
}


target_addr_t
CpuStateArm64::StackPointer() const
{
	return IsRegisterSet(ARM64_REGISTER_SP)
		? IntRegisterValue(ARM64_REGISTER_SP) : 0;
}


bool
CpuStateArm64::GetRegisterValue(const Register* reg, BVariant& _value) const
{
	int32 index = reg->Index();
	if (!IsRegisterSet(index))
		return false;

	if (reg->BitSize() == 32)
		_value.SetTo((uint32)fRegisters[index]);
	else
		_value.SetTo(fRegisters[index]);

	return true;
}


bool
CpuStateArm64::SetRegisterValue(const Register* reg, const BVariant& value)
{
	int32 index = reg->Index();
	if (index < 0 || index >= ARM64_REGISTER_COUNT)
		return false;

	fRegisters[index] = value.ToUInt64();
	fSetRegisters[index] = 1;
	return true;
}


bool
CpuStateArm64::IsRegisterSet(int32 index) const
{
	return index >= 0 && index < ARM64_REGISTER_COUNT && fSetRegisters[index];
}


uint64
CpuStateArm64::IntRegisterValue(int32 index) const
{
	if (!IsRegisterSet(index))
		return 0;

	return fRegisters[index];
}


void
CpuStateArm64::SetIntRegister(int32 index, uint64 value)
{
	if (index < 0 || index >= ARM64_REGISTER_COUNT)
		return;

	fRegisters[index] = value;
	fSetRegisters[index] = 1;
}
