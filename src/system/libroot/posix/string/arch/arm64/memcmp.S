/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

/* memcmp: from Arm Optimized Routines (see arm-optimized-routines/README). */

#define WANT_GNU_PROPERTY 0
#define __memcmp_aarch64 memcmp

#include "arm-optimized-routines/memcmp.S"
