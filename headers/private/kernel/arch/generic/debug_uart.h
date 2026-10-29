/*
 * Copyright 2012 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 *
 * Authors:
 *		François Revol, revol@free.fr
 */
#ifndef _KERNEL_ARCH_DEBUG_UART_H
#define _KERNEL_ARCH_DEBUG_UART_H


#include <sys/types.h>

#include <SupportDefs.h>


class DebugUART {
public:
							DebugUART(addr_t base, int64 clock)
								: fBase(base),
								fClock(clock),
								fEnabled(true) {};
							~DebugUART() {};

	virtual	void			InitEarly() {};
	virtual	void			Init() {};
	virtual	void			InitPort(uint32 baud) {};

	virtual	void			Enable() { fEnabled = true; }
	virtual	void			Disable() { fEnabled = false; }

	virtual	int				PutChar(char c) = 0;
	virtual	int				GetChar(bool wait) = 0;

	virtual	void			FlushTx() = 0;
	virtual	void			FlushRx() = 0;

	// Raise the UART's interrupt when input arrives (reading all of it
	// clears the interrupt). Returns false if not supported.
	virtual	bool			EnableReceiveInterrupt() { return false; }
	// Clear the receive interrupt before reading the input (where reading
	// the data alone does not clear it).
	virtual	void			ClearReceiveInterrupt() {}

	// Raise the UART's interrupt when its transmit FIFO has drained to its
	// trigger level (enable), or not. Returns false if not supported. While
	// it is used, CanPutChar() says whether PutChar() would not wait.
	virtual	bool			EnableTransmitInterrupt(bool enable)
								{ return false; }
	virtual	bool			CanPutChar() { return true; }

			void			SetBase(addr_t base) { fBase = base; }
			addr_t			Base() const { return fBase; }
			int64			Clock() const { return fClock; }
			bool			Enabled() const { return fEnabled; }

protected:
	virtual	void			Out8(int reg, uint8 value);
	virtual	uint8			In8(int reg);
	virtual	void			Barrier();

private:
			addr_t			fBase;
			int64			fClock;
			bool			fEnabled;
};


#endif	/* _KERNEL_ARCH_DEBUG_UART_H */
