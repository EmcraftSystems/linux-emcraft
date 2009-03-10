/**
 * @file op_arm11.c
 * ARM11 CP15 Performance Monitor Unit Driver
 *
 * @remark Copyright 2004-7 ARM SMP Development Team
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/smp.h>

#include "op_counter.h"
#include "op_arm_model.h"
#include "op_arm11.h"

static inline void arm11_write_pmnc(u32 val)
{
	/* upper 4bits and 7, 11 are write-as-0 */
	val &= 0x0ffff77f;
	asm volatile("mcr p15, 0, %0, c15, c12, 0" : : "r" (val));
}

static inline u32 arm11_read_pmnc(void)
{
	u32 val;
	asm volatile("mrc p15, 0, %0, c15, c12, 0" : "=r" (val));
	return val;
}

static void arm11_reset_counter(unsigned int cnt)
{
	u32 val = -(u32)counter_config[COUNTER_CPUn_PMNm(smp_processor_id(), cnt)].count;
	switch (cnt) {
	case CCNT:
		asm volatile("mcr p15, 0, %0, c15, c12, 1" : : "r" (val));
		break;

	case PMN0:
		asm volatile("mcr p15, 0, %0, c15, c12, 2" : : "r" (val));
		break;

	case PMN1:
		asm volatile("mcr p15, 0, %0, c15, c12, 3" : : "r" (val));
		break;
	}
}

int arm11_setup_pmu(void)
{
	unsigned long event;
	unsigned cpu;
	u32 pmnc;

	cpu = smp_processor_id();
	if (arm11_read_pmnc() & PMCR_E) {
		printk(KERN_ERR "oprofile: CPU%u PMU still enabled when setup new event counter.\n", cpu);
		return -EBUSY;
	}

	/* initialize PMNC, reset overflow, D bit, C bit and P bit. */
	arm11_write_pmnc(PMCR_OFL_PMN0 | PMCR_OFL_PMN1 | PMCR_OFL_CCNT |
			 PMCR_C | PMCR_P);

	pmnc = 0;

	if (counter_config[COUNTER_CPUn_PMNm(cpu, PMN0)].enabled) {
		event = counter_config[COUNTER_CPUn_PMNm(cpu, PMN0)].event & 255;
		pmnc |= event << 20;
		pmnc |= PMCR_IEN_PMN0;
		arm11_reset_counter(PMN0);
	}
	if (counter_config[COUNTER_CPUn_PMNm(cpu, PMN1)].enabled) {
		event = counter_config[COUNTER_CPUn_PMNm(cpu, PMN1)].event & 255;
		pmnc |= event << 12;
		pmnc |= PMCR_IEN_PMN1;
		arm11_reset_counter(PMN1);
	}
	if (counter_config[COUNTER_CPUn_CCNT(cpu)].enabled) {
		pmnc |= PMCR_IEN_CCNT;
		arm11_reset_counter(CCNT);
	}

	arm11_write_pmnc(pmnc);
	return 0;
}

int arm11_start_pmu(void)
{
	arm11_write_pmnc(arm11_read_pmnc() | PMCR_E);
	return 0;
}

int arm11_stop_pmu(void)
{
	unsigned int cnt;

	arm11_write_pmnc(arm11_read_pmnc() & ~PMCR_E);

	for (cnt = PMN0; cnt <= CCNT; cnt++)
		arm11_reset_counter(cnt);

	return 0;
}

/*
 * CPU counters' IRQ handler (one IRQ per CPU)
 */
static irqreturn_t arm11_pmu_interrupt(int irq, void *arg)
{
	struct pt_regs *regs = get_irq_regs();
	unsigned int cnt;
	u32 pmnc;

	pmnc = arm11_read_pmnc();

	/* First check if the two event counters have overflowed */
	for (cnt = PMN0; cnt <= PMN1; ++cnt) {
		if ((pmnc & (PMCR_OFL_PMN0 << cnt)) && (pmnc & (PMCR_IEN_PMN0 << cnt))) {
			arm11_reset_counter(cnt);
			oprofile_add_sample(regs, COUNTER_CPUn_PMNm(smp_processor_id(), cnt));
		}
	}

	/* Now check if the cycle counter has overflowed */
	if ((pmnc & PMCR_OFL_CCNT) && (pmnc & PMCR_IEN_CCNT)) {
		arm11_reset_counter(CCNT);
		oprofile_add_sample(regs, COUNTER_CPUn_CCNT(smp_processor_id()));
	}

	/* Clear counter flag(s) */
	arm11_write_pmnc(pmnc);
	return IRQ_HANDLED;
}

int arm11_request_interrupts(int *irqs, int nr)
{
	unsigned int i;
	int ret = 0;

	for(i = 0; i < nr; i++) {
		ret = request_irq(irqs[i], arm11_pmu_interrupt, IRQF_DISABLED, "CP15 PMU", NULL);
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

void arm11_release_interrupts(int *irqs, int nr)
{
	unsigned int i;

	for (i = 0; i < nr; i++)
		free_irq(irqs[i], NULL);
}
