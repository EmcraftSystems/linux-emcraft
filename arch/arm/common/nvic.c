/*
 *  linux/arch/arm/common/nvic.c
 *
 *  Copyright (C) 2008 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Support for the Nested Vectored Interrupt Controller found on the
 * ARMv7-M CPUs (Cortex-M3)
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/smp.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>
#include <asm/hardware/nvic.h>

static DEFINE_SPINLOCK(irq_controller_lock);

/*
 * Routines to acknowledge, disable and enable interrupts
 *
 * Linux assumes that when we're done with an interrupt we need to
 * unmask it, in the same way we need to unmask an interrupt when
 * we first enable it.
 *
 * The NVIC has a separate notion of "end of interrupt" to re-enable
 * an interrupt after handling, in order to support hardware
 * prioritisation.
 *
 * We can make the NVIC behave in the way that Linux expects by making
 * our "acknowledge" routine disable the interrupt, then mark it as
 * complete.
 */
static void nvic_ack_irq(unsigned int irq)
{
	u32 mask = 1 << (irq % 32);

	spin_lock(&irq_controller_lock);
	writel(mask, NVIC_CLEAR_ENABLE + irq / 32 * 4);
	spin_unlock(&irq_controller_lock);
}

static void nvic_mask_irq(unsigned int irq)
{
	u32 mask = 1 << (irq % 32);

	spin_lock(&irq_controller_lock);
	writel(mask, NVIC_CLEAR_ENABLE + irq / 32 * 4);
	spin_unlock(&irq_controller_lock);
}

static void nvic_unmask_irq(unsigned int irq)
{
	u32 mask = 1 << (irq % 32);

	spin_lock(&irq_controller_lock);
	writel(mask, NVIC_SET_ENABLE + irq / 32 * 4);
	spin_unlock(&irq_controller_lock);
}

static struct irq_chip nvic_chip = {
	.name		= "NVIC",
	.ack		= nvic_ack_irq,
	.mask		= nvic_mask_irq,
	.unmask		= nvic_unmask_irq,
};

void __init nvic_init(void)
{
	unsigned int max_irq, i;

	max_irq = ((readl(NVIC_INTR_CTRL) & 0x1f) + 1) * 32;

	/*
	 * Disable all interrupts
	 */
	for (i = 0; i < max_irq / 32; i++)
		writel(~0, NVIC_CLEAR_ENABLE + i * 4);

	/*
	 * Set priority on all interrupts.
	 */
	for (i = 0; i < max_irq; i += 4)
		writel(0, NVIC_PRIORITY + i);

	/*
	 * Setup the Linux IRQ subsystem.
	 */
	for (i = 0; i < NR_IRQS; i++) {
		set_irq_chip(i, &nvic_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}
}
