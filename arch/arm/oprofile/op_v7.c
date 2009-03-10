/**
 * @file op_v7.c
 * ARM V7 Performance Monitor Unit Driver
 *
 * @remark Copyright 2007 ARM SMP Development Team
 *
 * @remark Read the file COPYING
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/smp.h>

#include "op_counter.h"
#include "op_arm_model.h"
#include "op_v7.h"

/*
 * We assume each CPU has PMU_COUNTERS_PER_CPU counters, where the
 * last one is a cycle counter, the rest are event counters.
 * The oprofile event files in userland should ensure that we will not
 * access counters that aren't physically present, but we also check
 * the counter numbers here.
 */

#define PMCR_N_MASK	0xf800
#define PMCR_N_SHIFT	11

static unsigned event_counters_per_cpu;

/*
 * ARM V7 PMU support
 */
static inline void v7_write_pmcr(u32 val)
{
	asm volatile("mcr p15, 0, %0, c9, c12, 0" : : "r" (val));
}

static inline u32 v7_read_pmcr(void)
{
	u32 val;
	asm volatile("mrc p15, 0, %0, c9, c12, 0" : "=r" (val));
	return val;
}

static inline void v7_reset_counter(unsigned int cpu, unsigned int cnt)
{
	u32 val = -(u32)counter_config[COUNTER_CPUn_PMNm(cpu, cnt)].count;

	if (cnt == CCNT)
		/* Set cycle count in PMCCNTR */
		asm volatile("mcr p15, 0, %0, c9, c13, 0" : : "r" (val));
	else {
		/* Select correct counter using PMSELR */
		asm volatile("mcr p15, 0, %0, c9, c12, 5" : : "r" (cnt));
		/* Set the count value */
		asm volatile("mcr p15, 0, %0, c9, c13, 2" : : "r" (val));
	}
}

static inline void v7_set_event(unsigned int cnt, u32 val)
{
	/* Select correct counter using PMSELR */
	asm volatile("mcr p15, 0, %0, c9, c12, 5" : : "r" (cnt));
	/* Set event type in PMXEVTYPER*/
	asm volatile("mcr p15, 0, %0, c9, c13, 1" : : "r" (val));
}

static inline void v7_clear_overflow_status(u32 status)
{
	/* Clear overflow bits in PMOVSR */
	asm volatile("mcr p15, 0, %0, c9, c12, 3" : : "r" (status));
}

static inline u32 v7_read_overflow_status(void)
{
	u32 status;

	/* Read overflow bits in PMOVSR */
	asm volatile("mrc p15, 0, %0, c9, c12, 3" : "=r" (status));

	return status;
}

static inline void v7_enable_counter(unsigned int cnt)
{
	u32 val;

	if (cnt == CCNT)
		val = PMCNTEN_CCNT;
	else
		val = PMCNTEN_PMN0 << cnt;

	/* Set bit in PMCNTEN */
	asm volatile("mcr p15, 0, %0, c9, c12, 1" : : "r" (val));
}

static inline void v7_disable_counter(unsigned int cnt)
{
	u32 val;

	if (cnt == CCNT)
		val = PMCNTEN_CCNT;
	else
		val = PMCNTEN_PMN0 << cnt;

	/* Clear bit in PMCNTEN */
	asm volatile("mcr p15, 0, %0, c9, c12, 2" : : "r" (val));
}

static inline void v7_set_interrupts(u32 interrupts)
{
	/* Clear all interrupts in PMINTENCLR */
	asm volatile("mcr p15, 0, %0, c9, c14, 2" : : "r" (0xFFFFFFFFU));

	/* Set requested interrupts in PMINTENSET */
	asm volatile("mcr p15, 0, %0, c9, c14, 1" : : "r" (interrupts));
}

int v7_setup_pmu(void)
{
	unsigned int cnt, cpu;
	u32 pmcr, interrupts;

	/*
         * No need for get_cpu/put_cpu, because we were either called
         * from em_call_function(), which itself uses get_cpu/put_cpu,
         * or smp_call_function(), which means we are in IRQ context.
         */
	pmcr = v7_read_pmcr();
	cpu = smp_processor_id();

	if (pmcr & PMCR_E) {
		printk(KERN_ERR "oprofile: CPU%u PMU still enabled when setup new event counter.\n", cpu);
		return -EBUSY;
	}

	/* Discover the number of counters */
	event_counters_per_cpu = (pmcr & PMCR_N_MASK) >> PMCR_N_SHIFT;

	/* initialize PMNC: reset all counters, clear DP,X,D,E bits */
	v7_write_pmcr(PMCR_C | PMCR_P);
	for (cnt = 0; cnt < event_counters_per_cpu; cnt++)
		v7_disable_counter(cnt);
	v7_disable_counter(CCNT);

	interrupts = 0;

	/* Set up the event counters */
	for (cnt = 0; cnt < event_counters_per_cpu; cnt++) {
		unsigned long event;

		if (!counter_config[COUNTER_CPUn_PMNm(cpu, cnt)].enabled)
			continue;

		event = counter_config[COUNTER_CPUn_PMNm(cpu, cnt)].event & 255;

		v7_set_event(cnt, event);
		v7_reset_counter(cpu, cnt);
		v7_enable_counter(cnt);
		interrupts |= PMINTEN_PMN0 << cnt;
	}

	/* Now set up the cycle counter */
	if (counter_config[COUNTER_CPUn_CCNT(cpu)].enabled) {
		v7_reset_counter(cpu, CCNT);
		v7_enable_counter(CCNT);
		interrupts |= PMINTEN_CCNT;
	}

	/* Enable the required interrupts */
	v7_set_interrupts(interrupts);

	return 0;
}

int v7_start_pmu(void)
{
	v7_write_pmcr(v7_read_pmcr() | PMCR_E);
	return 0;
}

int v7_stop_pmu(void)
{
	v7_write_pmcr(v7_read_pmcr() & ~PMCR_E);
	return 0;
}

/*
 * CPU counters' IRQ handler (one IRQ per CPU)
 */
static irqreturn_t v7_pmu_interrupt(int irq, void *arg)
{
	struct pt_regs *regs = get_irq_regs();
	unsigned int cnt, cpu;
	u32 overflowed;

	overflowed = v7_read_overflow_status();
	cpu = smp_processor_id();

	/* Check each event counter */
	for (cnt = 0; cnt < CCNT; cnt++) {
		if (overflowed & (PMOVSR_PMN0 << cnt)) {
			v7_reset_counter(cpu, cnt);
			oprofile_add_sample(regs, COUNTER_CPUn_PMNm(cpu, cnt));
		}
	}

	/* Check the cycle counter */
	if (overflowed & PMOVSR_CCNT) {
		v7_reset_counter(cpu, CCNT);
		oprofile_add_sample(regs, COUNTER_CPUn_CCNT(cpu));
	}

	v7_clear_overflow_status(overflowed);
	return IRQ_HANDLED;
}

int v7_request_interrupts(int *irqs, int nr)
{
	unsigned int i;
	int ret = 0;

	for(i = 0; i < nr; i++) {
		ret = request_irq(irqs[i], v7_pmu_interrupt, IRQF_DISABLED, "CP15 PMU", NULL);
		if (ret != 0) {
			printk(KERN_ERR "oprofile: unable to request IRQ%u for CP15 PMU\n",
			       irqs[i]);
			break;
		}
	}

	if (i != nr)
		while (i-- != 0)
			free_irq(irqs[i], NULL);

	return ret;
}

void v7_release_interrupts(int *irqs, int nr)
{
	unsigned int i;

	for (i = 0; i < nr; i++)
		free_irq(irqs[i], NULL);
}
