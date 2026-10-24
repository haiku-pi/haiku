/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */

/*
 * arch_user_memcpy(), the copy behind user_memcpy() (see
 * arch/generic/user_memory.h): libroot's memcpy from Arm Optimized Routines,
 * which uses general-purpose registers only. It makes unaligned accesses, so
 * it is not the kernel's memcpy(): those would fault on device memory. User
 * memory and the kernel buffers copied to and from it are normal memory.
 */

#define WANT_GNU_PROPERTY 0
#define __memcpy_aarch64 arch_user_memcpy
#define __memmove_aarch64 arch_user_memmove
#define __memcpy_aarch64_sc arch_user_memcpy_sc
#define __memmove_aarch64_sc arch_user_memmove_sc

#include "../../../libroot/posix/string/arch/arm64/arm-optimized-routines/memcpy.S"
