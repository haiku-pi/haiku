/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _FBSD_COMPAT_DEV_SDIO_HAIKU_SDIO_H_
#define _FBSD_COMPAT_DEV_SDIO_HAIKU_SDIO_H_


/*!	An SDIO card found by Haiku's MMC bus manager, as the device ivars of a
	driver probed on the "sdio" bus (HAIKU_FBSD_WLAN_SDIO_DRIVER_GLUE). Each
	call goes to the bus manager, which serializes access to the bus.
	Functions return 0 or a (negative) Haiku error code.
*/


#include <sys/types.h>


#ifdef __cplusplus
extern "C" {
#endif

struct haiku_sdio_device;

uint16_t	haiku_sdio_vendor(struct haiku_sdio_device* device);
uint16_t	haiku_sdio_product(struct haiku_sdio_device* device);
int			haiku_sdio_function_count(struct haiku_sdio_device* device);

int		haiku_sdio_read_direct(struct haiku_sdio_device* device, int function,
			uint32_t address, uint8_t* _value);
int		haiku_sdio_write_direct(struct haiku_sdio_device* device, int function,
			uint32_t address, uint8_t value);
int		haiku_sdio_read_write_extended(struct haiku_sdio_device* device,
			int function, uint32_t address, int incrementAddress, int isWrite,
			void* buffer, size_t length);
int		haiku_sdio_set_block_size(struct haiku_sdio_device* device,
			int function, uint16_t blockSize);
int		haiku_sdio_enable_function(struct haiku_sdio_device* device,
			int function, int enable);
int		haiku_sdio_set_interrupt(struct haiku_sdio_device* device,
			int32_t semaphore);
			/* the card interrupt releases the semaphore, then stays masked */
void	haiku_sdio_acknowledge_interrupt(struct haiku_sdio_device* device);

#ifdef __cplusplus
}
#endif


#endif	/* _FBSD_COMPAT_DEV_SDIO_HAIKU_SDIO_H_ */
