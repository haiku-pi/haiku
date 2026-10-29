/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef DISASSEMBLER_ARM64_H
#define DISASSEMBLER_ARM64_H


#include <String.h>

#include "Types.h"


/*!	Disassembles AArch64 code with NetBSD's disassembler (aarch64_disasm.c):
	every instruction is 4 bytes long; an encoding it does not know is shown
	as ".insn 0x<hex>".
*/
class DisassemblerArm64 {
public:
								DisassemblerArm64();
								~DisassemblerArm64();

			status_t			Init(target_addr_t address, const void* code,
									size_t codeSize);

			status_t			GetNextInstruction(BString& line,
									target_addr_t& _address,
									target_size_t& _size,
									bool& _breakpointAllowed);

	static	status_t			Disassemble(target_addr_t address,
									uint32 instruction, BString& _line);

private:
			target_addr_t		fAddress;
			const uint8*		fCode;
			size_t				fCodeSize;
			size_t				fOffset;
};


#endif	// DISASSEMBLER_ARM64_H
