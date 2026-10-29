/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef CLI_STEP_COMMAND_H
#define CLI_STEP_COMMAND_H


#include "CliCommand.h"


class CliStepCommand : public CliCommand {
public:
								CliStepCommand(uint32 action,
									const char* summary, const char* usage);
	virtual	void				Execute(int argc, const char* const* argv,
									CliContext& context);

private:
			uint32				fAction;
};


#endif	// CLI_STEP_COMMAND_H
