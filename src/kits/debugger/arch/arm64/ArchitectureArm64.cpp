/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


#include "ArchitectureArm64.h"

#include <new>

#include <String.h>

#include <AutoDeleter.h>

#include "Arm64AssemblyLanguage.h"
#include "CfaContext.h"
#include "CpuStateArm64.h"
#include "DisassembledCode.h"
#include "disasm/DisassemblerArm64.h"
#include "FunctionDebugInfo.h"
#include "InstructionInfo.h"
#include "NoOpStackFrameDebugInfo.h"
#include "RegisterMap.h"
#include "StackFrame.h"
#include "Statement.h"
#include "TeamMemory.h"
#include "ValueLocation.h"


// DWARF register numbers for AArch64: x0 - x30, then sp. The return address
// column is x30, the link register; the pc has no DWARF number.
static const int32 kFromDwarfRegisters[] = {
	ARM64_REGISTER_X0 + 0,
	ARM64_REGISTER_X0 + 1,
	ARM64_REGISTER_X0 + 2,
	ARM64_REGISTER_X0 + 3,
	ARM64_REGISTER_X0 + 4,
	ARM64_REGISTER_X0 + 5,
	ARM64_REGISTER_X0 + 6,
	ARM64_REGISTER_X0 + 7,
	ARM64_REGISTER_X0 + 8,
	ARM64_REGISTER_X0 + 9,
	ARM64_REGISTER_X0 + 10,
	ARM64_REGISTER_X0 + 11,
	ARM64_REGISTER_X0 + 12,
	ARM64_REGISTER_X0 + 13,
	ARM64_REGISTER_X0 + 14,
	ARM64_REGISTER_X0 + 15,
	ARM64_REGISTER_X0 + 16,
	ARM64_REGISTER_X0 + 17,
	ARM64_REGISTER_X0 + 18,
	ARM64_REGISTER_X0 + 19,
	ARM64_REGISTER_X0 + 20,
	ARM64_REGISTER_X0 + 21,
	ARM64_REGISTER_X0 + 22,
	ARM64_REGISTER_X0 + 23,
	ARM64_REGISTER_X0 + 24,
	ARM64_REGISTER_X0 + 25,
	ARM64_REGISTER_X0 + 26,
	ARM64_REGISTER_X0 + 27,
	ARM64_REGISTER_X0 + 28,
	ARM64_REGISTER_FP,
	ARM64_REGISTER_LR,
	ARM64_REGISTER_SP,
};

static const int32 kFromDwarfRegisterCount = sizeof(kFromDwarfRegisters) / 4;

static const uint32 kInstructionSize = 4;
// how far into a function to look for the frame record's set-up
static const uint32 kMaxPrologueInstructions = 16;


// #pragma mark - ToDwarfRegisterMap


struct ArchitectureArm64::ToDwarfRegisterMap : RegisterMap {
	ToDwarfRegisterMap()
	{
		// init the index array from the reverse map
		memset(fIndices, -1, sizeof(fIndices));
		for (int32 i = 0; i < kFromDwarfRegisterCount; i++) {
			if (kFromDwarfRegisters[i] >= 0)
				fIndices[kFromDwarfRegisters[i]] = i;
		}
	}

	virtual int32 CountRegisters() const
	{
		return ARM64_REGISTER_COUNT;
	}

	virtual int32 MapRegisterIndex(int32 index) const
	{
		return index >= 0 && index < ARM64_REGISTER_COUNT ? fIndices[index] : -1;
	}

private:
	int32	fIndices[ARM64_REGISTER_COUNT];
};


// #pragma mark - FromDwarfRegisterMap


struct ArchitectureArm64::FromDwarfRegisterMap : RegisterMap {
	virtual int32 CountRegisters() const
	{
		return kFromDwarfRegisterCount;
	}

	virtual int32 MapRegisterIndex(int32 index) const
	{
		return index >= 0 && index < kFromDwarfRegisterCount
			? kFromDwarfRegisters[index] : -1;
	}
};


// #pragma mark - ArchitectureArm64


ArchitectureArm64::ArchitectureArm64(TeamMemory* teamMemory)
	:
	Architecture(teamMemory, 8, sizeof(arm64_debug_cpu_state), false),
	fAssemblyLanguage(NULL),
	fToDwarfRegisterMap(NULL),
	fFromDwarfRegisterMap(NULL)
{
}


ArchitectureArm64::~ArchitectureArm64()
{
	if (fAssemblyLanguage != NULL)
		fAssemblyLanguage->ReleaseReference();
	if (fToDwarfRegisterMap != NULL)
		fToDwarfRegisterMap->ReleaseReference();
	if (fFromDwarfRegisterMap != NULL)
		fFromDwarfRegisterMap->ReleaseReference();
}


status_t
ArchitectureArm64::Init()
{
	fAssemblyLanguage = new(std::nothrow) Arm64AssemblyLanguage;
	if (fAssemblyLanguage == NULL)
		return B_NO_MEMORY;

	try {
		// in the order of their indices: the base class takes a register's
		// index for its position in Registers()
		// (a Register keeps the name's pointer)
		static const char* const kNames[] = {
			"x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10",
			"x11", "x12", "x13", "x14", "x15", "x16", "x17", "x18", "x19",
			"x20", "x21", "x22", "x23", "x24", "x25", "x26", "x27", "x28"
		};
		for (int32 i = 0; i <= 28; i++) {
			_AddIntegerRegister(ARM64_REGISTER_X0 + i, kNames[i],
				B_UINT64_TYPE, REGISTER_TYPE_GENERAL_PURPOSE,
				i >= 19);
		}
		_AddIntegerRegister(ARM64_REGISTER_FP, "fp", B_UINT64_TYPE,
			REGISTER_TYPE_GENERAL_PURPOSE, true);
		_AddIntegerRegister(ARM64_REGISTER_LR, "lr", B_UINT64_TYPE,
			REGISTER_TYPE_RETURN_ADDRESS, false);
		_AddIntegerRegister(ARM64_REGISTER_SP, "sp", B_UINT64_TYPE,
			REGISTER_TYPE_STACK_POINTER, true);
		_AddIntegerRegister(ARM64_REGISTER_PC, "pc", B_UINT64_TYPE,
			REGISTER_TYPE_INSTRUCTION_POINTER, false);
		_AddIntegerRegister(ARM64_REGISTER_PSTATE, "pstate", B_UINT32_TYPE,
			REGISTER_TYPE_SPECIAL_PURPOSE, false);
	} catch (std::bad_alloc&) {
		return B_NO_MEMORY;
	}

	fToDwarfRegisterMap = new(std::nothrow) ToDwarfRegisterMap;
	fFromDwarfRegisterMap = new(std::nothrow) FromDwarfRegisterMap;

	if (fToDwarfRegisterMap == NULL || fFromDwarfRegisterMap == NULL)
		return B_NO_MEMORY;

	return B_OK;
}


int32
ArchitectureArm64::StackGrowthDirection() const
{
	return STACK_GROWTH_DIRECTION_NEGATIVE;
}


int32
ArchitectureArm64::CountRegisters() const
{
	return fRegisters.Count();
}


const Register*
ArchitectureArm64::Registers() const
{
	return fRegisters.Elements();
}


status_t
ArchitectureArm64::GetDwarfRegisterMaps(RegisterMap** _toDwarf,
	RegisterMap** _fromDwarf) const
{
	if (_toDwarf != NULL) {
		*_toDwarf = fToDwarfRegisterMap;
		fToDwarfRegisterMap->AcquireReference();
	}

	if (_fromDwarf != NULL) {
		*_fromDwarf = fFromDwarfRegisterMap;
		fFromDwarfRegisterMap->AcquireReference();
	}

	return B_OK;
}


status_t
ArchitectureArm64::GetCpuFeatures(uint32& flags)
{
	flags = 0;
	return B_OK;
}


status_t
ArchitectureArm64::CreateCpuState(CpuState*& _state)
{
	CpuStateArm64* state = new(std::nothrow) CpuStateArm64;
	if (state == NULL)
		return B_NO_MEMORY;

	_state = state;
	return B_OK;
}


status_t
ArchitectureArm64::CreateCpuState(const void* cpuStateData, size_t size,
	CpuState*& _state)
{
	if (size != sizeof(arm64_debug_cpu_state))
		return B_BAD_VALUE;

	CpuStateArm64* state = new(std::nothrow) CpuStateArm64(
		*(const arm64_debug_cpu_state*)cpuStateData);
	if (state == NULL)
		return B_NO_MEMORY;

	_state = state;
	return B_OK;
}


/*!	Creates a frame without call frame information, from the frame records
	the frame pointer chains: a frame record is the pair (previous fp, return
	address) at [fp]. A top frame whose function has not (yet) set up its frame
	record -- a thread waiting in a syscall stub, a leaf function, a function
	still in its prologue -- has its return address in lr instead.
*/
status_t
ArchitectureArm64::CreateStackFrame(Image* image, FunctionDebugInfo* function,
	CpuState* _cpuState, bool isTopFrame, StackFrame*& _frame,
	CpuState*& _previousCpuState)
{
	CpuStateArm64* cpuState = dynamic_cast<CpuStateArm64*>(_cpuState);
	uint64 framePointer = cpuState->IntRegisterValue(ARM64_REGISTER_FP);
	uint64 pc = cpuState->InstructionPointer();

	stack_frame_type frameType = STACK_FRAME_TYPE_STANDARD;
	bool readFrameRecord = true;
	uint64 previousFramePointer = 0;
	uint64 returnAddress = 0;

	if (isTopFrame && cpuState->IsRegisterSet(ARM64_REGISTER_LR)) {
		uint32 previousInstruction;
		bool inSyscall = pc >= kInstructionSize
			&& fTeamMemory->ReadMemory(pc - kInstructionSize,
				&previousInstruction, kInstructionSize) == kInstructionSize
			&& previousInstruction == 0xd4000001;
				// svc #0
		if (inSyscall || _IsBeforeFrameRecord(function, pc)) {
			frameType = inSyscall
				? STACK_FRAME_TYPE_SYSCALL : STACK_FRAME_TYPE_FRAMELESS;
			returnAddress = cpuState->IntRegisterValue(ARM64_REGISTER_LR);
			previousFramePointer = framePointer;
			framePointer = 0;
			readFrameRecord = false;
		}
	}

	// create the stack frame
	StackFrameDebugInfo* stackFrameDebugInfo
		= new(std::nothrow) NoOpStackFrameDebugInfo;
	if (stackFrameDebugInfo == NULL)
		return B_NO_MEMORY;
	BReference<StackFrameDebugInfo> stackFrameDebugInfoReference(
		stackFrameDebugInfo, true);

	StackFrame* frame = new(std::nothrow) StackFrame(frameType, cpuState,
		framePointer, pc, stackFrameDebugInfo);
	if (frame == NULL)
		return B_NO_MEMORY;
	BReference<StackFrame> frameReference(frame, true);

	status_t error = frame->Init();
	if (error != B_OK)
		return error;

	// read the frame record
	if (readFrameRecord) {
		uint64 frameRecord[2];
		if (framePointer != 0
			&& fTeamMemory->ReadMemory(framePointer, frameRecord, 16) == 16) {
			previousFramePointer = frameRecord[0];
			returnAddress = frameRecord[1];
		}
	}

	// create the CPU state, if we have any info
	CpuStateArm64* previousCpuState = NULL;
	if (returnAddress != 0) {
		previousCpuState = new(std::nothrow) CpuStateArm64;
		if (previousCpuState == NULL)
			return B_NO_MEMORY;

		previousCpuState->SetIntRegister(ARM64_REGISTER_FP,
			previousFramePointer);
		previousCpuState->SetIntRegister(ARM64_REGISTER_PC, returnAddress);
		frame->SetPreviousCpuState(previousCpuState);
	}

	frame->SetReturnAddress(returnAddress);

	_frame = frameReference.Detach();
	_previousCpuState = previousCpuState;
	return B_OK;
}


void
ArchitectureArm64::UpdateStackFrameCpuState(const StackFrame* frame,
	Image* previousImage, FunctionDebugInfo* previousFunction,
	CpuState* previousCpuState)
{
	// This is not a top frame: its instruction pointer is a return address.
	// Make it point to the call (bl/blr) the frame returns from -- every
	// instruction is 4 bytes long.
	CpuStateArm64* cpuState = dynamic_cast<CpuStateArm64*>(previousCpuState);
	uint64 pc = cpuState->InstructionPointer();
	if (previousFunction == NULL || pc <= previousFunction->Address())
		return;

	cpuState->SetInstructionPointer(pc - kInstructionSize);
}


status_t
ArchitectureArm64::ReadValueFromMemory(target_addr_t address, uint32 valueType,
	BVariant& _value) const
{
	uint8 buffer[64];
	size_t size = BVariant::SizeOfType(valueType);
	if (size == 0 || size > sizeof(buffer))
		return B_BAD_VALUE;

	ssize_t bytesRead = fTeamMemory->ReadMemory(address, buffer, size);
	if (bytesRead < 0)
		return bytesRead;
	if ((size_t)bytesRead != size)
		return B_ERROR;

	// TODO: We need to swap endianess, if the host is big endian!

	switch (valueType) {
		case B_INT8_TYPE:
			_value.SetTo(*(int8*)buffer);
			return B_OK;
		case B_UINT8_TYPE:
			_value.SetTo(*(uint8*)buffer);
			return B_OK;
		case B_INT16_TYPE:
			_value.SetTo(*(int16*)buffer);
			return B_OK;
		case B_UINT16_TYPE:
			_value.SetTo(*(uint16*)buffer);
			return B_OK;
		case B_INT32_TYPE:
			_value.SetTo(*(int32*)buffer);
			return B_OK;
		case B_UINT32_TYPE:
			_value.SetTo(*(uint32*)buffer);
			return B_OK;
		case B_INT64_TYPE:
			_value.SetTo(*(int64*)buffer);
			return B_OK;
		case B_UINT64_TYPE:
			_value.SetTo(*(uint64*)buffer);
			return B_OK;
		case B_FLOAT_TYPE:
			_value.SetTo(*(float*)buffer);
				// TODO: float on the host might work differently!
			return B_OK;
		case B_DOUBLE_TYPE:
			_value.SetTo(*(double*)buffer);
				// TODO: double on the host might work differently!
			return B_OK;
		default:
			return B_BAD_VALUE;
	}
}


status_t
ArchitectureArm64::ReadValueFromMemory(target_addr_t addressSpace,
	target_addr_t address, uint32 valueType, BVariant& _value) const
{
	// n/a on this architecture
	return B_BAD_VALUE;
}


status_t
ArchitectureArm64::DisassembleCode(FunctionDebugInfo* function,
	const void* buffer, size_t bufferSize, DisassembledCode*& _sourceCode)
{
	DisassembledCode* source = new(std::nothrow) DisassembledCode(
		fAssemblyLanguage);
	if (source == NULL)
		return B_NO_MEMORY;
	BReference<DisassembledCode> sourceReference(source, true);

	DisassemblerArm64 disassembler;
	status_t error = disassembler.Init(function->Address(), buffer, bufferSize);
	if (error != B_OK)
		return error;

	// add a function name line
	BString functionName(function->PrettyName());
	if (!source->AddCommentLine((functionName << ':').String()))
		return B_NO_MEMORY;

	// disassemble the instructions
	BString line;
	target_addr_t instructionAddress;
	target_size_t instructionSize;
	bool breakpointAllowed;
	while (disassembler.GetNextInstruction(line, instructionAddress,
			instructionSize, breakpointAllowed) == B_OK) {
		if (!source->AddInstructionLine(line, instructionAddress,
				instructionSize)) {
			return B_NO_MEMORY;
		}
	}

	_sourceCode = sourceReference.Detach();
	return B_OK;
}


status_t
ArchitectureArm64::GetStatement(FunctionDebugInfo* function,
	target_addr_t address, Statement*& _statement)
{
	// every instruction is a statement of its own, 4 bytes long
	ContiguousStatement* statement = new(std::nothrow) ContiguousStatement(
		SourceLocation(-1), TargetAddressRange(
			address & ~(target_addr_t)(kInstructionSize - 1), kInstructionSize));
	if (statement == NULL)
		return B_NO_MEMORY;

	_statement = statement;
	return B_OK;
}


status_t
ArchitectureArm64::GetInstructionInfo(target_addr_t address,
	InstructionInfo& _info, CpuState* state)
{
	uint32 instruction;
	if (fTeamMemory->ReadMemory(address, &instruction, kInstructionSize)
			!= kInstructionSize) {
		return B_ERROR;
	}

	// tell calls (bl, blr) and jumps from the rest
	instruction_type type = INSTRUCTION_TYPE_OTHER;
	target_addr_t targetAddress = 0;
	if ((instruction & 0xfc000000) == 0x94000000) {
		// bl: a signed 26 bit word offset
		int64 offset = (int64)((int32)(instruction << 6) >> 6) * 4;
		type = INSTRUCTION_TYPE_SUBROUTINE_CALL;
		targetAddress = address + offset;
	} else if ((instruction & 0xfffffc1f) == 0xd63f0000) {
		// blr xN
		type = INSTRUCTION_TYPE_SUBROUTINE_CALL;
		CpuStateArm64* cpuState = dynamic_cast<CpuStateArm64*>(state);
		if (cpuState != NULL) {
			targetAddress = cpuState->IntRegisterValue(
				ARM64_REGISTER_X0 + ((instruction >> 5) & 0x1f));
		}
	} else if ((instruction & 0xfc000000) == 0x14000000) {
		// b: a signed 26 bit word offset
		int64 offset = (int64)((int32)(instruction << 6) >> 6) * 4;
		type = INSTRUCTION_TYPE_JUMP;
		targetAddress = address + offset;
	} else if ((instruction & 0xfffffc1f) == 0xd61f0000
		|| (instruction & 0xfffffc1f) == 0xd65f0000) {
		// br xN, ret xN
		type = INSTRUCTION_TYPE_JUMP;
		CpuStateArm64* cpuState = dynamic_cast<CpuStateArm64*>(state);
		if (cpuState != NULL) {
			targetAddress = cpuState->IntRegisterValue(
				ARM64_REGISTER_X0 + ((instruction >> 5) & 0x1f));
		}
	}

	BString line;
	status_t error = DisassemblerArm64::Disassemble(address, instruction, line);
	if (error != B_OK)
		return error;

	if (!_info.SetTo(address, targetAddress, kInstructionSize, type, true,
			line)) {
		return B_NO_MEMORY;
	}

	return B_OK;
}


status_t
ArchitectureArm64::ResolvePICFunctionAddress(target_addr_t instructionAddress,
	CpuState* state, target_addr_t& _targetAddress)
{
	// A PLT entry loads the function's address from its GOT slot:
	//   adrp x16, <page of the slot>
	//   ldr x17, [x16, #<offset of the slot in the page>]
	//   add x16, x16, #<offset>
	//   br x17
	uint32 code[2];
	if (fTeamMemory->ReadMemory(instructionAddress, code, sizeof(code))
			!= (ssize_t)sizeof(code)) {
		return B_BAD_VALUE;
	}

	const uint32 adrp = code[0];
	const uint32 ldr = code[1];
	if ((adrp & 0x9f00001f) != 0x90000010
		|| (ldr & 0xffc003ff) != 0xf9400211) {
		return B_BAD_VALUE;
	}

	// immhi:immlo: a signed 21 bit count of 4 KiB pages
	uint64 pages = (((uint64)(adrp >> 5) & 0x7ffff) << 2) | ((adrp >> 29) & 0x3);
	int64 pageOffset = (int64)(pages << 43) >> 31;
	target_addr_t slot = (instructionAddress & ~(target_addr_t)0xfff)
		+ pageOffset + ((ldr >> 10) & 0xfff) * 8;

	target_addr_t address;
	if (fTeamMemory->ReadMemory(slot, &address, sizeof(address))
			!= (ssize_t)sizeof(address)) {
		return B_BAD_VALUE;
	}

	_targetAddress = address;
	return B_OK;
}


status_t
ArchitectureArm64::GetWatchpointDebugCapabilities(int32& _maxRegisterCount,
	int32& _maxBytesPerRegister, uint8& _watchpointCapabilityFlags)
{
	// The Cortex-A53, -A72 and QEMU have 4 watchpoints (at least 2 are
	// architectural); one watches up to 8 bytes of an aligned doubleword
	// (the kernel checks that), loads, stores or both.
	_maxRegisterCount = 4;
	_maxBytesPerRegister = 8;
	_watchpointCapabilityFlags = WATCHPOINT_CAPABILITY_FLAG_READ
		| WATCHPOINT_CAPABILITY_FLAG_WRITE
		| WATCHPOINT_CAPABILITY_FLAG_READ_WRITE;
	return B_OK;
}


status_t
ArchitectureArm64::GetReturnAddressLocation(StackFrame* frame,
	target_size_t valueSize, ValueLocation*& _location)
{
	// AAPCS64: values of up to 8 bytes are returned in x0, up to 16 in x0 and
	// x1; larger ones in memory the caller provides (x8 at the call, which is
	// not preserved), which is not supported here
	if (valueSize > 16)
		return B_UNSUPPORTED;

	ValueLocation* location = new(std::nothrow) ValueLocation(IsBigEndian());
	if (location == NULL)
		return B_NO_MEMORY;
	BReference<ValueLocation> locationReference(location, true);

	ValuePieceLocation piece;
	piece.SetSize(valueSize > 8 ? 8 : valueSize);
	piece.SetToRegister(ARM64_REGISTER_X0);
	if (!location->AddPiece(piece))
		return B_NO_MEMORY;

	if (valueSize > 8) {
		ValuePieceLocation highPiece;
		highPiece.SetSize(valueSize - 8);
		highPiece.SetToRegister(ARM64_REGISTER_X0 + 1);
		if (!location->AddPiece(highPiece))
			return B_NO_MEMORY;
	}

	_location = locationReference.Detach();
	return B_OK;
}


void
ArchitectureArm64::_AddIntegerRegister(int32 index, const char* name,
	uint32 valueType, register_type type, bool calleePreserved)
{
	if (!fRegisters.Add(Register(index, name,
			8 * BVariant::SizeOfType(valueType), valueType, type,
			calleePreserved))) {
		throw std::bad_alloc();
	}
}


/*!	Whether the function's frame record is not set up at \a pc: the function
	sets none (a leaf function), or \a pc is before the instruction that
	points fp at it ("mov x29, sp" or "add x29, sp, #imm").
*/
bool
ArchitectureArm64::_IsBeforeFrameRecord(FunctionDebugInfo* function,
	target_addr_t pc) const
{
	if (function == NULL)
		return false;
		// nothing known about it: follow the frame pointer

	uint32 count = function->Size() / kInstructionSize;
	if (count > kMaxPrologueInstructions)
		count = kMaxPrologueInstructions;

	uint32 code[kMaxPrologueInstructions];
	if (count == 0 || fTeamMemory->ReadMemory(function->Address(), code,
			count * kInstructionSize) != (ssize_t)(count * kInstructionSize)) {
		return false;
	}

	for (uint32 i = 0; i < count; i++) {
		// add x29, sp, #imm (mov x29, sp is #0), unshifted 12 bit immediate
		if ((code[i] & 0xffc003ff) == 0x910003fd)
			return pc <= function->Address() + i * kInstructionSize;
	}

	// no frame record set up at the start: a leaf function
	return true;
}
