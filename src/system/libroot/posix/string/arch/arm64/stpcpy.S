/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

/* stpcpy: from Arm Optimized Routines (see arm-optimized-routines/README). */

#define WANT_GNU_PROPERTY 0
#define BUILD_STPCPY 1
#define __stpcpy_aarch64 stpcpy

#include "arm-optimized-routines/strcpy.S"
