/*
 * Copyright 2019 Haiku, Inc. All Rights Reserved.
 * Distributed under the terms of the MIT License.
 */
#include <arch/debug_console.h>
#include <arch/generic/debug_uart.h>
#include <arch/generic/debug_uart_8250.h>
// #include <arch/arm/arch_uart_8250_omap.h>
#include <arch/arm/arch_uart_pl011.h>
#include <arch/arm64/arch_uart_linflex.h>
#include <arch/arm64/arch_uart_samsung.h>
#include <boot/kernel_args.h>
#include <cpu.h>
#include <debug.h>
#include <kernel.h>
#include <safemode.h>
#include <smp.h>
#include "soc.h"
#include <util/AutoLock.h>
#include <vm/vm.h>
#include <string.h>


static DebugUART *sArchDebugUART = NULL;

// Once the UART's interrupt is installed, the kernel's output is queued here
// and sent by the transmit interrupt: a CPU that prints no longer waits for
// the line to go out (at 115200 baud, 87 us a character) with its interrupts
// disabled. The kernel debugger writes directly, after what is still queued.
// The kernel setting "serial_debug_buffered false" keeps all output direct
// (a hang then cannot hide the last lines in the buffer).
static const size_t kTxBufferSize = 16 * 1024;
static char sTxBuffer[kTxBufferSize];
static size_t sTxHead;
static size_t sTxCount;
static spinlock sTxLock = B_SPINLOCK_INITIALIZER;
static bool sTxBuffered;
static bool sTxInterruptEnabled;


//!	Moves what fits into the UART's FIFO. Called with sTxLock held.
static void
tx_fill_fifo()
{
	while (sTxCount > 0 && sArchDebugUART->CanPutChar()) {
		sArchDebugUART->PutChar(sTxBuffer[sTxHead]);
		sTxHead = (sTxHead + 1) % kTxBufferSize;
		sTxCount--;
	}

	// With characters left, the FIFO is full: it raises the interrupt when it
	// has drained to its trigger level.
	bool enable = sTxCount > 0;
	if (enable != sTxInterruptEnabled) {
		sArchDebugUART->EnableTransmitInterrupt(enable);
		sTxInterruptEnabled = enable;
	}
}


//!	Called with sTxLock held.
static void
tx_queue(char c)
{
	if (sTxCount == kTxBufferSize) {
		// full: send the oldest character, waiting for room in the FIFO
		sArchDebugUART->PutChar(sTxBuffer[sTxHead]);
		sTxHead = (sTxHead + 1) % kTxBufferSize;
		sTxCount--;
	}
	sTxBuffer[(sTxHead + sTxCount) % kTxBufferSize] = c;
	sTxCount++;
}


/*!	Sends what is queued, waiting for the UART, before direct output. In the
	kernel debugger the other CPUs are stopped, maybe holding the lock: it is
	then not waited for long.
*/
static void
tx_flush()
{
	if (sTxCount == 0)
		return;

	cpu_status state = disable_interrupts();
	bool locked = false;
	for (int32 i = 0; i < 1000000; i++) {
		if (try_acquire_spinlock(&sTxLock)) {
			locked = true;
			break;
		}
		cpu_pause();
	}

	while (sTxCount > 0) {
		sArchDebugUART->PutChar(sTxBuffer[sTxHead]);
		sTxHead = (sTxHead + 1) % kTxBufferSize;
		sTxCount--;
	}
	if (sTxInterruptEnabled) {
		sArchDebugUART->EnableTransmitInterrupt(false);
		sTxInterruptEnabled = false;
	}

	if (locked)
		release_spinlock(&sTxLock);
	restore_interrupts(state);
}


static inline bool
tx_buffered()
{
	return sTxBuffered && !debug_debugger_running();
}


static int32
debug_uart_interrupt(void* /*data*/)
{
	// (cleared first: input arriving meanwhile raises it again)
	sArchDebugUART->ClearReceiveInterrupt();
	int c;
	bool any = false;
	while ((c = sArchDebugUART->GetChar(false)) >= 0) {
		debug_serial_emergency_input((char)c);
		any = true;
	}

	if (sTxBuffered) {
		SpinLocker locker(sTxLock);
		if (sTxInterruptEnabled) {
			tx_fill_fifo();
			any = true;
		}
	}

	return any ? B_HANDLED_INTERRUPT : B_UNHANDLED_INTERRUPT;
}


/*!	Watches the debug UART's input from its interrupt, so that the serial
	emergency keys work even when no thread can run anymore.
*/
void
arm64_debug_console_init_interrupt(kernel_args* args)
{
	uint32 irq = args->arch_args.uart.irq;
	if (sArchDebugUART == NULL || irq == 0)
		return;
	if (install_io_interrupt_handler(irq, &debug_uart_interrupt, NULL, 0)
			!= B_OK) {
		return;
	}
	// Any CPU may serve it: the keys must work while one CPU is stuck
	if (InterruptController::Get() != NULL)
		InterruptController::Get()->RouteInterrupt(irq, 0xff);
	if (!sArchDebugUART->EnableReceiveInterrupt()) {
		remove_io_interrupt_handler(irq, &debug_uart_interrupt, NULL);
		return;
	}

	if (get_safemode_boolean("serial_debug_buffered", true))
		sTxBuffered = sArchDebugUART->EnableTransmitInterrupt(false);
}


void
arch_debug_remove_interrupt_handler(uint32 line)
{
}


int
arch_debug_blue_screen_try_getchar(void)
{
	// TODO: Implement correctly!
	return arch_debug_blue_screen_getchar();
}


char
arch_debug_blue_screen_getchar(void)
{
	return arch_debug_serial_getchar();
}


int
arch_debug_serial_try_getchar(void)
{
	if (sArchDebugUART == NULL)
		return -1;

	return sArchDebugUART->GetChar(false);
}


char
arch_debug_serial_getchar(void)
{
	if (sArchDebugUART == NULL)
		return '\0';

	return sArchDebugUART->GetChar(true);
}


void
arch_debug_serial_putchar(const char c)
{
	if (sArchDebugUART == NULL)
		return;

	if (tx_buffered()) {
		InterruptsSpinLocker locker(sTxLock);
		tx_queue(c);
		tx_fill_fifo();
		return;
	}

	tx_flush();
	sArchDebugUART->PutChar(c);
}


void
arch_debug_serial_puts(const char *s)
{
	if (sArchDebugUART == NULL)
		return;

	if (tx_buffered()) {
		InterruptsSpinLocker locker(sTxLock);
		for (; *s != '\0'; s++) {
			if (*s == '\n')
				tx_queue('\r');
			if (*s != '\r')
				tx_queue(*s);
		}
		tx_fill_fifo();
		return;
	}

	tx_flush();
	while (*s != '\0') {
		char ch = *s;
		if (ch == '\n') {
			sArchDebugUART->PutChar('\r');
			sArchDebugUART->PutChar('\n');
		} else if (ch != '\r')
			sArchDebugUART->PutChar(ch);
		s++;
	}
}


void
arch_debug_serial_early_boot_message(const char *string)
{
	// this function will only be called in fatal situations
	sTxBuffered = false;
	arch_debug_serial_puts(string);
}


status_t
arch_debug_console_init(kernel_args *args)
{
	if (strncmp(args->arch_args.uart.kind, UART_KIND_PL011,
		sizeof(args->arch_args.uart.kind)) == 0) {
		sArchDebugUART = arch_get_uart_pl011(args->arch_args.uart.regs.start,
			args->arch_args.uart.clock);
	} else if (strncmp(args->arch_args.uart.kind, UART_KIND_LINFLEX,
		sizeof(args->arch_args.uart.kind)) == 0) {
		sArchDebugUART = arch_get_uart_linflex(args->arch_args.uart.regs.start,
			args->arch_args.uart.clock);
	}/* else if (strncmp(args->arch_args.uart.kind, UART_KIND_8250_OMAP,
		sizeof(args->arch_args.uart.kind)) == 0) {
		sArchDebugUART = arch_get_uart_8250_omap(args->arch_args.uart.regs.start,
			args->arch_args.uart.clock);
	}*/ else if (strncmp(args->arch_args.uart.kind, UART_KIND_8250,
		sizeof(args->arch_args.uart.kind)) == 0) {
		sArchDebugUART = arch_get_uart_8250(args->arch_args.uart.regs.start,
			args->arch_args.uart.clock);
	} else if (strncmp(args->arch_args.uart.kind, UART_KIND_SAMSUNG,
		sizeof(args->arch_args.uart.kind)) == 0) {
		sArchDebugUART = arch_get_uart_samsung(args->arch_args.uart.regs.start,
			args->arch_args.uart.clock);
	}

	// Oh well.
	if (sArchDebugUART == NULL)
		return B_ERROR;

	sArchDebugUART->InitEarly();

	return B_OK;
}


status_t
arch_debug_console_init_settings(kernel_args *args)
{
	return B_OK;
}
