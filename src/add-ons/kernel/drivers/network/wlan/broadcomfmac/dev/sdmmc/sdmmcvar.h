/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef _BROADCOMFMAC_SDMMCVAR_H_
#define _BROADCOMFMAC_SDMMCVAR_H_


/*!	The subset of OpenBSD's sdmmc(4) interface that bwfm(4) uses, implemented
	on top of the SDIO card that Haiku's MMC bus manager found (see
	dev/sdio/haiku_sdio.h). This is not a copy of OpenBSD's sdmmcvar.h; only the
	members and functions if_bwfm_sdio.c touches exist.
*/


#include <sys/param.h>
#include <sys/queue.h>
#include <sys/rwlock.h>


#ifndef SIMPLEQ_HEAD
/* OpenBSD's simple queues are FreeBSD's singly-linked tail queues */
#define SIMPLEQ_HEAD			STAILQ_HEAD
#define SIMPLEQ_ENTRY			STAILQ_ENTRY
#define SIMPLEQ_INIT			STAILQ_INIT
#define SIMPLEQ_FIRST			STAILQ_FIRST
#define SIMPLEQ_FOREACH			STAILQ_FOREACH
#define SIMPLEQ_INSERT_TAIL		STAILQ_INSERT_TAIL
#define SIMPLEQ_REMOVE_HEAD		STAILQ_REMOVE_HEAD
#endif


struct haiku_sdio_device;
struct sdmmc_intr_handler;

struct sdmmc_cis {
	uint16_t	manufacturer;
	uint16_t	product;
};

struct sdmmc_function {
	struct sdmmc_softc*	sc;
	void*				cookie;
		/* FDT node of the function in OpenBSD; always NULL here */
	SIMPLEQ_ENTRY(sdmmc_function) sf_list;
	int					number;
	struct sdmmc_cis	cis;
	unsigned int		cur_blklen;
};

/* One SDIO card: function 0 (the CCCR) plus its I/O functions */
struct sdmmc_softc {
	struct rwlock		sc_lock;
		/* write-held around every card access, as in OpenBSD */
	int					sc_function_count;
	struct sdmmc_function* sc_fn0;
	SIMPLEQ_HEAD(, sdmmc_function) sf_head;

	struct haiku_sdio_device* sc_sdio;
	struct sdmmc_intr_handler* sc_ih;
	uint8_t				sc_intr_enabled;
		/* functions whose card interrupt is enabled */
};

struct sdmmc_attach_args {
	struct sdmmc_function* sf;
};


struct sdmmc_softc* sdmmc_haiku_create(struct haiku_sdio_device* sdio);
void	sdmmc_haiku_destroy(struct sdmmc_softc* sc);
struct sdmmc_function* sdmmc_haiku_function(struct sdmmc_softc* sc,
			int number);

uint8_t	sdmmc_io_read_1(struct sdmmc_function* sf, int reg);
void	sdmmc_io_write_1(struct sdmmc_function* sf, int reg, uint8_t data);
uint32_t sdmmc_io_read_4(struct sdmmc_function* sf, int reg);
void	sdmmc_io_write_4(struct sdmmc_function* sf, int reg, uint32_t data);
int		sdmmc_io_read_multi_1(struct sdmmc_function* sf, int reg,
			u_char* data, int datalen);
int		sdmmc_io_read_region_1(struct sdmmc_function* sf, int reg,
			u_char* data, int datalen);
int		sdmmc_io_write_region_1(struct sdmmc_function* sf, int reg,
			u_char* data, int datalen);
void	sdmmc_io_set_blocklen(struct sdmmc_function* sf, unsigned int blklen);
int		sdmmc_io_function_enable(struct sdmmc_function* sf);
void	sdmmc_io_function_disable(struct sdmmc_function* sf);

void*	sdmmc_intr_establish(struct sdmmc_softc* sc, int (*fun)(void*),
			void* arg, const char* name);
void	sdmmc_intr_disestablish(void* cookie);
void	sdmmc_intr_enable(struct sdmmc_function* sf);
void	sdmmc_intr_disable(struct sdmmc_function* sf);


#endif	/* _BROADCOMFMAC_SDMMCVAR_H_ */
