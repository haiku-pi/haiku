/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

/* memcpy, memmove: from Arm Optimized Routines (see arm-optimized-routines/README). */

#define WANT_GNU_PROPERTY 0
#define __memcpy_aarch64 memcpy
#define __memmove_aarch64 memmove
#define __memcpy_aarch64_sc __memcpy_sc
#define __memmove_aarch64_sc __memmove_sc

#include "arm-optimized-routines/memcpy.S"
