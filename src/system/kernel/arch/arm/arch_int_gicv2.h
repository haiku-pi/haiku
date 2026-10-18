/*
 * Copyright 2021-2026 Haiku, Inc. All rights reserved.
 * Distributed under the terms of the MIT License.
 */
#ifndef ARCH_ARM_GICV2_H
#define ARCH_ARM_GICV2_H

#include <SupportDefs.h>

#include "soc.h"

class GICv2InterruptController : public InterruptController {
public:
	GICv2InterruptController(phys_addr_t gicd_regs = 0, phys_addr_t gicc_regs = 0);
	void EnableInterrupt(int32 irq);
	void DisableInterrupt(int32 irq);
	void HandleInterrupt();
	void SendMulticastIci(CPUSet& cpuSet);
	void SendBroadcastIci();
	status_t PerCpuInit();
	void RouteInterrupt(int32 irq, uint32 cpuMask);
private:
	volatile uint32_t *fGicdRegs;
	volatile uint32_t *fGiccRegs;
	uint8 fCpuInterfaceMask[SMP_MAX_CPUS];
		// the GIC's target bit of each CPU (not necessarily 1 << cpu)
};

#endif /* ARCH_ARM_GICV2_H */
