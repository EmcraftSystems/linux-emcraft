/*
 * linux/arch/arm/mach-a2f/fpga.c
 * SmartFusion FPGA managent code. Includes:
 * - Generic set-up to allow DMA-style accessed to MSS memory by FPGA IP
 * - Demultiplexer for IRQs triggered by FPGA CoreInterrupt
 *
 * Copyright (C) 2011 Dmitry Cherukhin, Emcraft Systems
 * Copyright (C) 2012 Vladimir Khusainov, Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <mach/a2f.h>
#include <mach/fpga.h>

/*
 * Set up the MSS / FPGA interfaces to allow DMA-style accesses
 * to the MSS memory by an FPGA-based IP
 * @returns		0 (success) or -1 (failure)
 */
int a2f_fpga_dma_init(void)
{
	uint32_t v;
	int ret = -1;

#define PERIPHERAL_DMA_ENABLE	(1<<2)
#define FPGA_MASTER_ENABLE	(1<<0)
	v = readl(&A2F_SYSREG->ahb_matrix_cr) |
		PERIPHERAL_DMA_ENABLE |
		FPGA_MASTER_ENABLE;
	writel(v, &A2F_SYSREG->ahb_matrix_cr);

#define PROT_REGION_ENABLE	(1<<0)
	v = readl(&A2F_SYSREG->fab_prot_base_cr) &
		~PROT_REGION_ENABLE;
	writel(v, &A2F_SYSREG->fab_prot_base_cr);

	/*
	 * Here. means success
	 */
	ret = 0;

	return ret;
}

#if defined(CONFIG_A2F_FPGA_CINT)

/*
 * CoreInterrupt registers
 */
struct a2f_fpga_cint {
	uint8_t		FIQSoftInt;	/* FIQ soft interrupt register */
	uint8_t		unused1[3];
	uint8_t		FIQSoftIntClear;/* FIQ soft interrupt clear register */
	uint8_t		unused2[3];
	uint8_t		FIQEnable;	/* FIQ enable register */
	uint8_t		unused3[3];
	uint8_t		FIQEnableClear;	/* FIQ enable clear register */
	uint8_t		unused4[3];
	uint8_t		FIQRawStatus;	/* FIQ raw status register */
	uint8_t		unused5[3];
	uint8_t		FIQStatus;	/* FIQ status register */
	uint8_t		unused6[3];
	uint32_t	IRQSoftInt;	/* IRQ soft interrupt register */
	uint32_t	IRQSoftIntClear;/* IRQ soft interrupt clear register */
	uint32_t	IRQEnable;	/* IRQ enable register */
	uint32_t	IRQEnableClear; /* IRQ enable clear register */
	uint32_t	IRQRawStatus;	/* IRQ raw status register */
	uint32_t	IRQStatus;	/* IRQ status register */
};
#define A2F_FPGA_CINT	((struct a2f_fpga_cint *)(CONFIG_A2F_FPGA_CINT_BASE))

#if defined(CONFIG_A2F_FPGA_DEMUX)

struct a2f_fpga_demux {
	struct irq_chip		irq_chip;
	struct a2f_fpga_cint	*cint;
	uint32_t		irq_masked;
	spinlock_t		spinlock;
};

static void a2f_fpga_demux_irq_ack(unsigned int irq);
static void a2f_fpga_demux_irq_mask(unsigned int irq);
static void a2f_fpga_demux_irq_unmask(unsigned int irq);

/*
 * irq_chip structure and other variables for the demultiplexer
 */
static struct a2f_fpga_demux a2f_fpga_demux = {
	.irq_chip = {
		.name		= "CoreInterrupt IRQ demux",
		.ack		= a2f_fpga_demux_irq_ack,
		.mask		= a2f_fpga_demux_irq_mask,
		.unmask		= a2f_fpga_demux_irq_unmask,
	},
	.cint			= A2F_FPGA_CINT,
	.irq_masked		= -1u,
	.spinlock		= SPIN_LOCK_UNLOCKED,
};


/*
 * Acknowledge interrupt source in the demultiplexer; nothing to do
 * @param irq		MPU IRQ number
 */
static void a2f_fpga_demux_irq_ack(unsigned int irq)
{
}

/*
 * Mask interrupt source in the demultiplexer
 * @param irq		MPU IRQ number
 */
static void a2f_fpga_demux_irq_mask(unsigned int irq)
{
	unsigned long flags;
	uint32_t mask = 1 << (irq - A2F_FPGA_DEMUX_IRQ_BASE);

	spin_lock_irqsave(&a2f_fpga_demux.spinlock, flags);
	a2f_fpga_demux.irq_masked |= mask;
	writel(mask, &a2f_fpga_demux.cint->IRQEnableClear);
	spin_unlock_irqrestore(&a2f_fpga_demux.spinlock, flags);
}

/*
 * Unmask interrupt source in the demultiplexer
 * @param irq		MPU IRQ number
 */
static void a2f_fpga_demux_irq_unmask(unsigned int irq)
{
	unsigned long flags;
	uint32_t mask = 1 << (irq - A2F_FPGA_DEMUX_IRQ_BASE);

	spin_lock_irqsave(&a2f_fpga_demux.spinlock, flags);
	a2f_fpga_demux.irq_masked &= ~mask;
	writel(mask, &a2f_fpga_demux.cint->IRQEnable);
	spin_unlock_irqrestore(&a2f_fpga_demux.spinlock, flags);
}

/*
 * This function is called then IRQ from CoreInterrupt is triggered;
 * the function demultiplexes this interrupt and call separate
 * interrupt handlers for each active source within CoreInterrupt
 * @param irq		CoreInterrupt IRQ number (=A2F_FPGA_CINT_IRQ)
 * @param desc		IRQ descriptor, not used
 */
static void a2f_fpga_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	if (a2f_fpga_demux.irq_masked != -1u) {
		int i;
		uint32_t status = readl(&a2f_fpga_demux.cint->IRQStatus);

		/* generate software IRQ for each active FPGA source */
		for (i = 0; i < 32; i++) {
			if (status & (1 << i)) {
				generic_handle_irq(A2F_FPGA_DEMUX_IRQ_MAP(i));
			}
		}
	}
}

/*
 * Initialize the demultiplexer for IRQ generated by FPGA CoreInterrupt
 */
void a2f_fpga_demux_init(void)
{
	set_irq_chained_handler(A2F_FPGA_CINT_IRQ, a2f_fpga_demux_handler);
}

#endif /* !defined(CONFIG_A2F_FPGA_DEMUX) */

/*
 * Attach IRQ source to the demultiplexer; this function should be
 * called for each IRQ source within CoreInterrupt during init time.
 * If the demultiplexer is not used, then this function enables
 * IRQ source within CoreInterrupt.
 * @param irq		IRQ source within CoreInterrupt (0..31)
 * @returns		0 -> success, < 0 -> error code
 */
int a2f_fpga_demux_irq_source_enable(unsigned int irq)
{
	int ret;
	if (irq > 31) {
		printk(KERN_ERR "a2f_fpga_demux: attempt to attach "
				"an incorrect IRQ source %d\n", irq);
		ret = -EINVAL;
		goto Done_release_nothing;
	}
#if defined(CONFIG_A2F_FPGA_DEMUX)
	if ((ret = set_irq_chip(A2F_FPGA_DEMUX_IRQ_MAP(irq),
			&a2f_fpga_demux.irq_chip)) < 0) {
		printk(KERN_ERR "a2f_fpga_demux: error %d attaching "
				"IRQ source %d\n", ret, irq);
		goto Done_release_nothing;
	}
	set_irq_handler(A2F_FPGA_DEMUX_IRQ_MAP(irq), handle_simple_irq);
#else /* !defined(CONFIG_A2F_FPGA_DEMUX) */
	writel(1 << irq, &A2F_FPGA_CINT->IRQEnable);
#endif /* defined(CONFIG_A2F_FPGA_DEMUX) */
	ret = 0;
Done_release_nothing:
	return ret;
}

#endif /* defined(CONFIG_A2F_FPGA_CINT) */

