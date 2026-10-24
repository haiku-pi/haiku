/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

/* memrchr: from Arm Optimized Routines (see arm-optimized-routines/README). */

#define WANT_GNU_PROPERTY 0
#define __memrchr_aarch64 memrchr

#include "arm-optimized-routines/memrchr.S"
