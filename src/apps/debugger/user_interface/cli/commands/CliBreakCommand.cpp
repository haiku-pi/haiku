/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


#include "CliBreakCommand.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <AutoLocker.h>

#include "CliContext.h"
#include "FunctionInstance.h"
#include "Image.h"
#include "ImageDebugInfo.h"
#include "Team.h"
#include "UserInterface.h"


/*!	Whether \a name is \a wanted, or \a wanted followed by the parameter
	list ("add3" for "add3(int)").
*/
static bool
name_matches(const BString& name, const char* wanted)
{
	if (name == wanted)
		return true;

	int32 length = strlen(wanted);
	return name.Compare(wanted, length) == 0 && name.ByteAt(length) == '(';
}


CliBreakCommand::CliBreakCommand()
	:
	CliCommand("set a breakpoint",
		"%s <function> | <address>\n"
		"Sets a breakpoint at the start of the function with the given name\n"
		"or at the given address.")
{
}


void
CliBreakCommand::Execute(int argc, const char* const* argv,
	CliContext& context)
{
	if (argc != 2) {
		PrintUsage(argv[0]);
		return;
	}

	target_addr_t address = 0;
	BString name;

	char* end;
	address = strtoull(argv[1], &end, 0);
	if (end == argv[1] || *end != '\0') {
		// not a number: look the function up in the images whose debug
		// information has been loaded
		address = 0;
		int32 imagesNotLoaded = 0;

		Team* team = context.GetTeam();
		AutoLocker<Team> teamLocker(team);
		for (ImageList::ConstIterator it = team->Images().GetIterator();
				Image* image = it.Next();) {
			ImageDebugInfo* debugInfo = image->GetImageDebugInfo();
			if (debugInfo == NULL) {
				if (image->ImageDebugInfoState() == IMAGE_DEBUG_INFO_NOT_LOADED
					|| image->ImageDebugInfoState()
						== IMAGE_DEBUG_INFO_LOADING) {
					imagesNotLoaded++;
					context.GetUserInterfaceListener()->ImageDebugInfoRequested(
						image);
				}
				continue;
			}

			for (int32 i = 0;
					FunctionInstance* function = debugInfo->FunctionAt(i);
					i++) {
				if (name_matches(function->Name(), argv[1])
					|| name_matches(function->PrettyName(), argv[1])) {
					address = function->Address();
					name = function->PrettyName();
					break;
				}
			}
			if (address != 0)
				break;
		}

		if (address == 0) {
			if (imagesNotLoaded > 0) {
				printf("Function \"%s\" not found; the debug information of %"
					B_PRId32 " images is being loaded, try again.\n", argv[1],
					imagesNotLoaded);
			} else
				printf("Function \"%s\" not found.\n", argv[1]);
			return;
		}
	}

	context.GetUserInterfaceListener()->SetBreakpointRequested(address, true);

	if (name.IsEmpty())
		printf("Breakpoint at %#" B_PRIx64 ".\n", address);
	else
		printf("Breakpoint at %#" B_PRIx64 " (%s).\n", address, name.String());
}
