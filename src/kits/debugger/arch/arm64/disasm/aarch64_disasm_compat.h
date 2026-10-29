/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef AARCH64_DISASM_COMPAT_H
#define AARCH64_DISASM_COMPAT_H

/*!	What NetBSD's AArch64 disassembler (aarch64_disasm.c) takes from the
	NetBSD headers.
*/

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ByteOrder.h>


#ifndef __arraycount
#	define __arraycount(array)	(sizeof(array) / sizeof((array)[0]))
#endif
#ifndef __unused
#	define __unused				__attribute__((__unused__))
#endif
#ifndef __printflike
#	define __printflike(format, arguments) \
		__attribute__((__format__(__printf__, format, arguments)))
#endif

#ifndef le32toh
#	define le32toh(value)		B_LENDIAN_TO_HOST_INT32(value)
#endif


static inline int
fls64(uint64_t value)
{
	return value == 0 ? 0 : 64 - __builtin_clzll(value);
}


#endif	// AARCH64_DISASM_COMPAT_H
