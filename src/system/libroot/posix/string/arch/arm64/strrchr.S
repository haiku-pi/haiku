/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

/* strrchr: from Arm Optimized Routines (see arm-optimized-routines/README). */

#define WANT_GNU_PROPERTY 0
#define __strrchr_aarch64 strrchr

#include "arm-optimized-routines/strrchr.S"
