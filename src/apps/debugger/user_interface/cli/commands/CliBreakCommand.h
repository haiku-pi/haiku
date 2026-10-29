/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef CLI_BREAK_COMMAND_H
#define CLI_BREAK_COMMAND_H


#include "CliCommand.h"


class CliBreakCommand : public CliCommand {
public:
								CliBreakCommand();
	virtual	void				Execute(int argc, const char* const* argv,
									CliContext& context);
};


#endif	// CLI_BREAK_COMMAND_H
