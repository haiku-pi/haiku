/*
 * Copyright 2026, Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */


/*!	OpenBSD's sdmmc(4) I/O interface (dev/sdmmc/sdmmcvar.h), as far as bwfm(4)
	needs it, on top of Haiku's MMC bus manager.

	The bus manager does not deliver SDIO card interrupts yet, so an
	established interrupt handler is polled from a kernel thread instead.
*/


#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/rwlock.h>

#include <KernelExport.h>

#include <dev/sdio/haiku_sdio.h>
#include <dev/sdmmc/sdmmcvar.h>


#define SD_IO_CCCR_INT_ENABLE	0x04
	/* bit 0 is the master enable, bit n enables function n */

#define SDMMC_INTR_POLL_INTERVAL	10000	/* us */


struct sdmmc_intr_handler {
	struct sdmmc_softc*	ih_softc;
	int					(*ih_fun)(void*);
	void*				ih_arg;
	thread_id			ih_thread;
	volatile int32		ih_quit;
};


struct sdmmc_softc*
sdmmc_haiku_create(struct haiku_sdio_device* sdio)
{
	struct sdmmc_softc* sc;
	struct sdmmc_function* sf;
	int i;

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);
	if (sc == NULL)
		return NULL;

	rw_init(&sc->sc_lock, "sdmmc");
	SIMPLEQ_INIT(&sc->sf_head);
	sc->sc_sdio = sdio;
	sc->sc_function_count = haiku_sdio_function_count(sdio);

	/* Function 0 (the CCCR) first, then the I/O functions */
	for (i = 0; i <= sc->sc_function_count; i++) {
		sf = malloc(sizeof(*sf), M_DEVBUF, M_WAITOK | M_ZERO);
		if (sf == NULL) {
			sdmmc_haiku_destroy(sc);
			return NULL;
		}

		sf->sc = sc;
		sf->number = i;
		sf->cis.manufacturer = haiku_sdio_vendor(sdio);
		sf->cis.product = haiku_sdio_product(sdio);
		SIMPLEQ_INSERT_TAIL(&sc->sf_head, sf, sf_list);
		if (i == 0)
			sc->sc_fn0 = sf;
	}

	return sc;
}


void
sdmmc_haiku_destroy(struct sdmmc_softc* sc)
{
	struct sdmmc_function* sf;

	if (sc == NULL)
		return;

	if (sc->sc_ih != NULL)
		sdmmc_intr_disestablish(sc->sc_ih);

	while ((sf = SIMPLEQ_FIRST(&sc->sf_head)) != NULL) {
		SIMPLEQ_REMOVE_HEAD(&sc->sf_head, sf_list);
		free(sf, M_DEVBUF, sizeof(*sf));
	}

	rw_lock_destroy(&sc->sc_lock.lock);
	free(sc, M_DEVBUF, sizeof(*sc));
}


struct sdmmc_function*
sdmmc_haiku_function(struct sdmmc_softc* sc, int number)
{
	struct sdmmc_function* sf;

	SIMPLEQ_FOREACH(sf, &sc->sf_head, sf_list) {
		if (sf->number == number)
			return sf;
	}
	return NULL;
}


/* #pragma mark - I/O (CMD52 / CMD53) */


uint8_t
sdmmc_io_read_1(struct sdmmc_function* sf, int reg)
{
	uint8_t data = 0;

	rw_assert_wrlock(&sf->sc->sc_lock);

	haiku_sdio_read_direct(sf->sc->sc_sdio, sf->number, reg, &data);
	return data;
}


void
sdmmc_io_write_1(struct sdmmc_function* sf, int reg, uint8_t data)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	haiku_sdio_write_direct(sf->sc->sc_sdio, sf->number, reg, data);
}


uint32_t
sdmmc_io_read_4(struct sdmmc_function* sf, int reg)
{
	uint32_t data = 0;

	rw_assert_wrlock(&sf->sc->sc_lock);

	haiku_sdio_read_write_extended(sf->sc->sc_sdio, sf->number, reg, 1, 0,
		&data, sizeof(data));
	return data;
}


void
sdmmc_io_write_4(struct sdmmc_function* sf, int reg, uint32_t data)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	haiku_sdio_read_write_extended(sf->sc->sc_sdio, sf->number, reg, 1, 1,
		&data, sizeof(data));
}


/*!	Reads \a datalen bytes from the FIFO at \a reg (fixed address). The bus
	manager splits the transfer into block and byte mode as needed.
*/
int
sdmmc_io_read_multi_1(struct sdmmc_function* sf, int reg, u_char* data,
	int datalen)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	return haiku_sdio_read_write_extended(sf->sc->sc_sdio, sf->number, reg,
		0, 0, data, datalen);
}


int
sdmmc_io_read_region_1(struct sdmmc_function* sf, int reg, u_char* data,
	int datalen)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	return haiku_sdio_read_write_extended(sf->sc->sc_sdio, sf->number, reg,
		1, 0, data, datalen);
}


int
sdmmc_io_write_region_1(struct sdmmc_function* sf, int reg, u_char* data,
	int datalen)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	return haiku_sdio_read_write_extended(sf->sc->sc_sdio, sf->number, reg,
		1, 1, data, datalen);
}


void
sdmmc_io_set_blocklen(struct sdmmc_function* sf, unsigned int blklen)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	if (blklen == 0)
		blklen = 512;

	if (haiku_sdio_set_block_size(sf->sc->sc_sdio, sf->number, blklen) == 0)
		sf->cur_blklen = blklen;
}


int
sdmmc_io_function_enable(struct sdmmc_function* sf)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	if (sf->number == 0)
		return 0;	/* FN0 is always enabled */

	return haiku_sdio_enable_function(sf->sc->sc_sdio, sf->number, 1);
}


void
sdmmc_io_function_disable(struct sdmmc_function* sf)
{
	rw_assert_wrlock(&sf->sc->sc_lock);

	if (sf->number == 0)
		return;		/* FN0 is always enabled */

	haiku_sdio_enable_function(sf->sc->sc_sdio, sf->number, 0);
}


/* #pragma mark - card interrupts */


static int32
sdmmc_intr_poll_thread(void* arg)
{
	struct sdmmc_intr_handler* ih = arg;

	while (!ih->ih_quit) {
		if (ih->ih_softc->sc_intr_enabled != 0) {
			/* Like the compat layer's interrupt threads (not MP safe) */
			mtx_lock(&Giant);
			ih->ih_fun(ih->ih_arg);
			mtx_unlock(&Giant);
		}
		snooze(SDMMC_INTR_POLL_INTERVAL);
	}
	return 0;
}


/*!	Establishes the card interrupt handler \a fun. As card interrupts do not
	reach us, it is called periodically while any function's interrupt is
	enabled (sdmmc_intr_enable()). Only one handler is supported.
*/
void*
sdmmc_intr_establish(struct sdmmc_softc* sc, int (*fun)(void*), void* arg,
	const char* name)
{
	struct sdmmc_intr_handler* ih;

	if (sc->sc_ih != NULL)
		return NULL;

	ih = malloc(sizeof(*ih), M_DEVBUF, M_WAITOK | M_ZERO);
	if (ih == NULL)
		return NULL;

	ih->ih_softc = sc;
	ih->ih_fun = fun;
	ih->ih_arg = arg;
	ih->ih_thread = spawn_kernel_thread(sdmmc_intr_poll_thread, name,
		B_REAL_TIME_DISPLAY_PRIORITY, ih);
	if (ih->ih_thread < 0) {
		free(ih, M_DEVBUF, sizeof(*ih));
		return NULL;
	}

	sc->sc_ih = ih;
	resume_thread(ih->ih_thread);
	return ih;
}


void
sdmmc_intr_disestablish(void* cookie)
{
	struct sdmmc_intr_handler* ih = cookie;
	status_t result;
	int giant;

	ih->ih_quit = 1;

	/* The poll thread may be waiting for Giant */
	giant = mtx_owned(&Giant);
	if (giant)
		mtx_unlock(&Giant);
	wait_for_thread(ih->ih_thread, &result);
	if (giant)
		mtx_lock(&Giant);

	ih->ih_softc->sc_ih = NULL;
	free(ih, M_DEVBUF, sizeof(*ih));
}


void
sdmmc_intr_enable(struct sdmmc_function* sf)
{
	struct sdmmc_softc* sc = sf->sc;
	uint8_t imask;

	rw_assert_wrlock(&sc->sc_lock);

	/* Set the master enable along with the function's bit */
	imask = sdmmc_io_read_1(sc->sc_fn0, SD_IO_CCCR_INT_ENABLE);
	imask |= 1 | (1 << sf->number);
	sdmmc_io_write_1(sc->sc_fn0, SD_IO_CCCR_INT_ENABLE, imask);

	sc->sc_intr_enabled |= 1 << sf->number;
}


void
sdmmc_intr_disable(struct sdmmc_function* sf)
{
	struct sdmmc_softc* sc = sf->sc;
	uint8_t imask;

	rw_assert_wrlock(&sc->sc_lock);

	imask = sdmmc_io_read_1(sc->sc_fn0, SD_IO_CCCR_INT_ENABLE);
	imask &= ~(1 << sf->number);
	sdmmc_io_write_1(sc->sc_fn0, SD_IO_CCCR_INT_ENABLE, imask);

	sc->sc_intr_enabled &= ~(1 << sf->number);
}
