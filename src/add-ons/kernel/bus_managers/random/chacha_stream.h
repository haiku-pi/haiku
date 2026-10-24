/*
 * Copyright 2026, Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _CHACHA_STREAM_H
#define _CHACHA_STREAM_H


#include <SupportDefs.h>


#define CHACHA_KEY_SIZE		32
#define CHACHA_IV_SIZE		8


typedef struct chacha_stream {
	uint32	input[16];
} chacha_stream;


#ifdef __cplusplus
extern "C" {
#endif

void chacha_stream_init(chacha_stream* stream, const uint8* keyAndIV);
void chacha_stream_generate(chacha_stream* stream, uint8* buffer,
	uint32 length);

#ifdef __cplusplus
}
#endif


#endif /* _CHACHA_STREAM_H */
