/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


#include "CliStepCommand.h"

#include <stdio.h>

#include <AutoLocker.h>

#include "CliContext.h"
#include "Team.h"
#include "UserInterface.h"


CliStepCommand::CliStepCommand(uint32 action, const char* summary,
	const char* usage)
	:
	CliCommand(summary, usage),
	fAction(action)
{
}


void
CliStepCommand::Execute(int argc, const char* const* argv,
	CliContext& context)
{
	if (argc != 1) {
		PrintUsage(argv[0]);
		return;
	}

	AutoLocker<Team> teamLocker(context.GetTeam());
	Thread* thread = context.CurrentThread();
	if (thread == NULL) {
		printf("Error: No current thread.\n");
		return;
	}

	if (thread->State() != THREAD_STATE_STOPPED) {
		printf("Error: The current thread is not stopped.\n");
		return;
	}

	context.GetUserInterfaceListener()->ThreadActionRequested(thread->ID(),
		fAction);
}
