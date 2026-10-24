/*
 * Copyright 2026, Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */


//! The ChaCha20 keystream, from OpenBSD's chacha_private.h (D. J. Bernstein,
//! public domain). In C: the header's 16-character constant strings fill
//! their arrays exactly, which C++ does not accept.


#include "chacha_stream.h"

#define KEYSTREAM_ONLY
#include "chacha_private.h"


_Static_assert(sizeof(chacha_stream) == sizeof(chacha_ctx),
	"chacha_stream must match chacha_ctx");


void
chacha_stream_init(chacha_stream* stream, const uint8* keyAndIV)
{
	chacha_keysetup((chacha_ctx*)stream, keyAndIV, CHACHA_KEY_SIZE * 8);
	chacha_ivsetup((chacha_ctx*)stream, keyAndIV + CHACHA_KEY_SIZE);
}


void
chacha_stream_generate(chacha_stream* stream, uint8* buffer, uint32 length)
{
	chacha_encrypt_bytes((chacha_ctx*)stream, buffer, buffer, length);
}
