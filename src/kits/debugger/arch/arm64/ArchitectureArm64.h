/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef ARCHITECTURE_ARM64_H
#define ARCHITECTURE_ARM64_H


#include <Array.h>

#include "Architecture.h"
#include "Register.h"


class SourceLanguage;


class ArchitectureArm64 : public Architecture {
public:
								ArchitectureArm64(TeamMemory* teamMemory);
	virtual						~ArchitectureArm64();

	virtual	status_t			Init();

	virtual int32				StackGrowthDirection() const;

	virtual	int32				CountRegisters() const;
	virtual	const Register*		Registers() const;

	virtual	status_t			GetDwarfRegisterMaps(RegisterMap** _toDwarf,
									RegisterMap** _fromDwarf) const;

	virtual	status_t			GetCpuFeatures(uint32& flags);

	virtual	status_t			CreateCpuState(CpuState*& _state);
	virtual	status_t			CreateCpuState(const void* cpuStateData,
									size_t size, CpuState*& _state);
	virtual	status_t			CreateStackFrame(Image* image,
									FunctionDebugInfo* function,
									CpuState* cpuState, bool isTopFrame,
									StackFrame*& _previousFrame,
									CpuState*& _previousCpuState);
	virtual	void				UpdateStackFrameCpuState(
									const StackFrame* frame,
									Image* previousImage,
									FunctionDebugInfo* previousFunction,
									CpuState* previousCpuState);

	virtual	status_t			ReadValueFromMemory(target_addr_t address,
									uint32 valueType, BVariant& _value) const;
	virtual	status_t			ReadValueFromMemory(target_addr_t addressSpace,
									target_addr_t address, uint32 valueType,
									BVariant& _value) const;

	virtual	status_t			DisassembleCode(FunctionDebugInfo* function,
									const void* buffer, size_t bufferSize,
									DisassembledCode*& _sourceCode);
	virtual	status_t			GetStatement(FunctionDebugInfo* function,
									target_addr_t address,
									Statement*& _statement);
	virtual	status_t			GetInstructionInfo(target_addr_t address,
									InstructionInfo& _info, CpuState* state);
	virtual	status_t			ResolvePICFunctionAddress(target_addr_t
									instructionAddress,
									CpuState* state,
									target_addr_t& _targetAddress);

	virtual	status_t			GetWatchpointDebugCapabilities(
									int32& _maxRegisterCount,
									int32& _maxBytesPerRegister,
									uint8& _watchpointCapabilityFlags);

	virtual	status_t			GetReturnAddressLocation(
									StackFrame* frame, target_size_t valueSize,
									ValueLocation*& _location);

private:
			struct ToDwarfRegisterMap;
			struct FromDwarfRegisterMap;

private:
			void				_AddIntegerRegister(int32 index,
									const char* name, uint32 valueType,
									register_type type, bool calleePreserved);
			bool				_IsBeforeFrameRecord(
									FunctionDebugInfo* function,
									target_addr_t pc) const;

private:
			Array<Register>		fRegisters;
			SourceLanguage*		fAssemblyLanguage;
			ToDwarfRegisterMap*	fToDwarfRegisterMap;
			FromDwarfRegisterMap* fFromDwarfRegisterMap;
};


#endif	// ARCHITECTURE_ARM64_H
