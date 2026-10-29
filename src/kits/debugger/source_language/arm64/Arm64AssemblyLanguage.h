/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef ARM64_ASSEMBLY_LANGUAGE_H
#define ARM64_ASSEMBLY_LANGUAGE_H


#include "SourceLanguage.h"


class Arm64AssemblyLanguage : public SourceLanguage {
public:
								Arm64AssemblyLanguage();
	virtual						~Arm64AssemblyLanguage();

	virtual	const char*			Name() const;
};


#endif	// ARM64_ASSEMBLY_LANGUAGE_H
