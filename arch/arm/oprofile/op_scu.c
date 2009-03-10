/**
 * @file op_scu.c
 * MPCORE Snoop Control Unit Event Monitor Driver
 * @remark Copyright 2004-7 ARM SMP Development Team
 * @remark Copyright 2000-2004 Deepak Saxena <dsaxena@mvista.com>
 * @remark Copyright 2000-2004 MontaVista Software Inc
 * @remark Copyright 2004 Dave Jiang <dave.jiang@intel.com>
 * @remark Copyright 2004 Intel Corporation
 * @remark Copyright 2004 Zwane Mwaikambo <zwane@arm.linux.org.uk>
 * @remark Copyright 2004 Oprofile Authors
 *
 * @remark Read the file COPYING
 *
 * @author Zwane Mwaikambo
 *
 */

/* #define DEBUG */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/smp.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/system.h>
#include <mach/hardware.h>

#include "op_counter.h"
#include "op_arm_model.h"
#include "op_scu.h"

struct eventmonitor {
	unsigned long PMCR;
	unsigned char MCEB[8];
	unsigned long MC[8];
};

#define PMCR_E	1

/*
 * Bitmask of used SCU counters
 */
static unsigned int scu_em_used;

/*
 * 2 helper fns take a counter number from 0-7 (not the userspace-visible counter number)
 */
static inline void scu_reset_counter(struct eventmonitor __iomem *emc, unsigned int n)
{
	writel(-(u32)counter_config[COUNTER_SCU_MN(n)].count, &emc->MC[n]);
}

static inline void scu_set_event(struct eventmonitor __iomem *emc, unsigned int n, u32 event)
{
	event &= 0xff;
	writeb(event, &emc->MCEB[n]);
}

/*
 * SCU counters' IRQ handler (one IRQ per counter => 2 IRQs per CPU)
 */
static irqreturn_t scu_em_interrupt(int irq, void *arg)
{
	struct eventmonitor __iomem *emc = SCU_EVENTMONITORS_VA_BASE;
	unsigned int cnt, tmp;

	cnt = irq - IRQ_PMU_SCU0;
	oprofile_add_sample(get_irq_regs(), COUNTER_SCU_MN(cnt));
	scu_reset_counter(emc, cnt);

	/* Clear overflow flag for this counter */
	tmp = readl(&emc->PMCR);
	tmp &= 0xff00ffff;	/* mask out any other overflow flags */
	tmp |= 1 << (cnt + 16);
	writel(tmp, &emc->PMCR);

	return IRQ_HANDLED;
}

/* Configure just the SCU counters that the user has requested */
void scu_setup(void)
{
	struct eventmonitor __iomem *emc = SCU_EVENTMONITORS_VA_BASE;
	unsigned int i;

	scu_em_used = 0;

	for (i = 0; i < NUM_SCU_COUNTERS; i++) {
		if (counter_config[COUNTER_SCU_MN(i)].enabled &&
		    counter_config[COUNTER_SCU_MN(i)].event) {
			scu_set_event(emc, i, 0); /* disable counter for now */
			scu_em_used |= 1 << i;
		}
	}
}

int scu_start(void)
{
	struct eventmonitor __iomem *emc = SCU_EVENTMONITORS_VA_BASE;
	unsigned int temp, i;
	unsigned long event;
	int ret = 0;

	/*
	 * request the SCU counter interrupts that we need
	 */
	for (i = 0; i < NUM_SCU_COUNTERS; i++) {
		if (scu_em_used & (1 << i)) {
			ret = request_irq(IRQ_PMU_SCU0 + i, scu_em_interrupt, IRQF_DISABLED,
					"SCU PMU", NULL);
			if (ret) {
				printk(KERN_ERR
					"oprofile: unable to request IRQ%u for SCU Event Monitor\n",
					IRQ_PMU_SCU0 + i);
				goto err_free_scu;
			}
		}
	}

	/*
	 * clear overflow and enable interrupt for all used counters
	 */
	temp = readl(&emc->PMCR);
	for (i = 0; i < NUM_SCU_COUNTERS; i++) {
		if (scu_em_used & (1 << i)) {
			scu_reset_counter(emc, i);
			event = counter_config[COUNTER_SCU_MN(i)].event;
			scu_set_event(emc, i, event);

			/* clear overflow/interrupt */
			temp |= 1 << (i + 16);
			/* enable interrupt*/
			temp |= 1 << (i + 8);
		}
	}

	/* Enable all 8 counters */
	temp |= PMCR_E;
	writel(temp, &emc->PMCR);

	return 0;

 err_free_scu:
	while (i--)
		free_irq(IRQ_PMU_SCU0 + i, NULL);
	return ret;
}

void scu_stop(void)
{
	struct eventmonitor __iomem *emc = SCU_EVENTMONITORS_VA_BASE;
	unsigned int temp, i;

	/* Disable counter interrupts */
	/* Don't disable all 8 counters (with the E bit) as they may be in use */
	temp = readl(&emc->PMCR);
	for (i = 0; i < NUM_SCU_COUNTERS; i++) {
		if (scu_em_used & (1 << i))
			temp &= ~(1 << (i + 8));
	}
	writel(temp, &emc->PMCR);

	/* Free counter interrupts and reset counters */
	for (i = 0; i < NUM_SCU_COUNTERS; i++) {
		if (scu_em_used & (1 << i)) {
			scu_reset_counter(emc, i);
			free_irq(IRQ_PMU_SCU0 + i, NULL);
		}
	}
}

