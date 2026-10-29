/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


#include "DisassemblerArm64.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <ByteOrder.h>

extern "C" {
#include "aarch64_disasm_compat.h"
#include "aarch64_disasm.h"
}


static const size_t kInstructionSize = 4;

// the line disasm_insn() prints into (it has no context argument)
static __thread BString* sOutput;


static void
print(const char* format, ...)
{
	char buffer[256];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	sOutput->Append(buffer);
}


static void
print_address(uintptr_t address)
{
	char buffer[24];
	snprintf(buffer, sizeof(buffer), "0x%" B_PRIx64, (uint64)address);
	sOutput->Append(buffer);
}


static const disasm_interface_t kInterface = {
	NULL,
	print_address,
	print
};


DisassemblerArm64::DisassemblerArm64()
	:
	fAddress(0),
	fCode(NULL),
	fCodeSize(0),
	fOffset(0)
{
}


DisassemblerArm64::~DisassemblerArm64()
{
}


status_t
DisassemblerArm64::Init(target_addr_t address, const void* code,
	size_t codeSize)
{
	fAddress = address;
	fCode = (const uint8*)code;
	fCodeSize = codeSize;
	fOffset = 0;
	return B_OK;
}


status_t
DisassemblerArm64::GetNextInstruction(BString& line, target_addr_t& _address,
	target_size_t& _size, bool& _breakpointAllowed)
{
	if (fOffset + kInstructionSize > fCodeSize)
		return B_ENTRY_NOT_FOUND;

	uint32 instruction;
	memcpy(&instruction, fCode + fOffset, sizeof(instruction));
	target_addr_t address = fAddress + fOffset;
	fOffset += kInstructionSize;

	status_t error = Disassemble(address,
		B_LENDIAN_TO_HOST_INT32(instruction), line);
	if (error != B_OK)
		return error;

	_address = address;
	_size = kInstructionSize;
	_breakpointAllowed = true;
	return B_OK;
}


/*static*/ status_t
DisassemblerArm64::Disassemble(target_addr_t address, uint32 instruction,
	BString& _line)
{
	BString text;
	sOutput = &text;
	disasm_insn(&kInterface, (uintptr_t)address, instruction);
	sOutput = NULL;

	// "mnemonic\toperands\n" -> "mnemonic operands"
	text.RemoveAll("\n");
	text.ReplaceAll('\t', ' ');
	if (text.IsEmpty())
		text.SetToFormat(".insn 0x%08" B_PRIx32, instruction);

	_line.SetToFormat("0x%016" B_PRIx64 ": %08" B_PRIx32 "  %s",
		(uint64)address, instruction, text.String());
	return _line.IsEmpty() ? B_NO_MEMORY : B_OK;
}
