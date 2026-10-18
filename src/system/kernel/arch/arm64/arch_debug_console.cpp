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
#include <debug.h>
#include <kernel.h>
#include <vm/vm.h>
#include <string.h>


static DebugUART *sArchDebugUART = NULL;


static int32
debug_uart_interrupt(void* /*data*/)
{
	int c;
	bool any = false;
	while ((c = sArchDebugUART->GetChar(false)) >= 0) {
		debug_serial_emergency_input((char)c);
		any = true;
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
	if (!sArchDebugUART->EnableReceiveInterrupt())
		remove_io_interrupt_handler(irq, &debug_uart_interrupt, NULL);
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

	sArchDebugUART->PutChar(c);
}


void
arch_debug_serial_puts(const char *s)
{
	while (*s != '\0') {
		char ch = *s;
		if (ch == '\n') {
			arch_debug_serial_putchar('\r');
			arch_debug_serial_putchar('\n');
		} else if (ch != '\r')
			arch_debug_serial_putchar(ch);
		s++;
	}
}


void
arch_debug_serial_early_boot_message(const char *string)
{
	// this function will only be called in fatal situations
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
