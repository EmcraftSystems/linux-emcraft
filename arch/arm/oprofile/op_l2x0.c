/**
 * @file op_model_l2x0.c
 * ARM L220/L230 Level 2 Cache Controller Event Counter Driver
 * @remark Copyright 2004-7 ARM SMP Development Team
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/smp.h>

#include <asm/io.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <mach/irqs.h>

#include "op_counter.h"
#include "op_arm_model.h"
#include "op_l2x0.h"

static unsigned l2x0_base, l2x0_irq;

/*
 * Determine L2X0 base address and event counter IRQ
 */
void l2x0_ec_setup(void)
{
#if defined(CONFIG_MACH_REALVIEW_EB)
	l2x0_base = IO_ADDRESS(REALVIEW_EB11MP_L220_BASE);
	l2x0_irq = IRQ_EB11MP_L220_EVENT;
#elif defined(CONFIG_MACH_REALVIEW_PB11MP)
	l2x0_base = IO_ADDRESS(REALVIEW_TC11MP_L220_BASE);
	l2x0_irq = IRQ_TC11MP_L220_EVENT;
#elif defined(CONFIG_MACH_REALVIEW_PBX)
	l2x0_base = IO_ADDRESS(REALVIEW_PBX_TILE_L220_BASE);
	l2x0_irq = IRQ_PBX_L220_EVENT;
#else
#error l2x0_base and l2x0_irq not set!
#endif
}



/*
 * Read the configuration of an event counter
 */
static inline u32 l2x0_ec_read_config(unsigned int cnt)
{
	return readl(l2x0_base + L2X0_EVENT_CNT0_CFG - cnt * 4);
}

/*
 * Change the configuration of an event counter
 */
static inline void l2x0_ec_write_config(unsigned int cnt, u32 config)
{
	writel(config, l2x0_base + L2X0_EVENT_CNT0_CFG - cnt * 4);
}

/*
 * Reset a counter to its initial value
 */
static inline void l2x0_ec_reset(unsigned int cnt)
{
	u32 val, temp;

	/*
	 * We can only write to the counter value when the counter is disabled
	 */
	temp = l2x0_ec_read_config(cnt);
	l2x0_ec_write_config(cnt, L2X0_EVENT_CONFIG_DISABLED);

	/*
	 * Ok, set the counter value
	 */
	val = -(u32)counter_config[COUNTER_L2X0_EC(cnt)].count;
	writel(val, l2x0_base + L2X0_EVENT_CNT0_VAL - cnt * 4);

	/*
	 * Now put the counter config back to what it was before
	 */
	l2x0_ec_write_config(cnt, temp);
}

/*
 * Read the current value of an event counter
 */
static inline u32 l2x0_ec_read_value(unsigned int cnt)
{
	return readl(l2x0_base + L2X0_EVENT_CNT0_VAL - cnt * 4);
}

/*
 * Enable/disable L220/L230 event counting system
 * We assume the Event Monitoring Bus is already enabled
 * (that is, bit 20 is set in the L2X0 Aux control register)
 * because it can't be set while the L2X0 is enabled.
 */
static inline void l2x0_ec_system_setup(unsigned enable)
{
	u32 val;
	unsigned cnt;

	/*
	 * Enable/disable and Reset all the counters
	 */
	 val = L2X0_EVENT_CONTROL_RESET_ALL;
	 if (enable)
		val |= L2X0_EVENT_CONTROL_ENABLE;
	 writel(val, l2x0_base + L2X0_EVENT_CNT_CTRL);

	/*
	 * Set the individual counters to disabled (for now at least)
	 */
	for (cnt = 0; cnt < L2X0_NUM_COUNTERS; ++cnt)
		l2x0_ec_write_config(cnt, L2X0_EVENT_CONFIG_DISABLED);

	/*
	 * Clear any stray EC interrupt, and set the mask appropriately
	 */
	writel(L2X0_INTR_ECNTR, l2x0_base + L2X0_INTR_CLEAR);
	val = readl(l2x0_base + L2X0_INTR_MASK);
	if (enable)
		val |= L2X0_INTR_ECNTR;
	else
		val &= !L2X0_INTR_ECNTR;
	writel(val, l2x0_base + L2X0_INTR_MASK);
}

#ifdef CONFIG_SMP
/*
 * Rotate L220/L230 EC interrupts around all the online CPUs in an SMP system.
 * We do this because we can't know which CPU caused an L220/L230 event,
 * and this gives us a sensible statistical picture of what was running.
 * This function is always called in interrupt context.
 */
static inline void l2x0_ec_rotate_irq(int irq)
{
	static unsigned cpu = 0;
	cpumask_t mask;

	if (is_smp()) {
		cpu = next_cpu(cpu, cpu_online_map);
		if (cpu >= NR_CPUS)
			cpu = first_cpu(cpu_online_map);
		mask = cpumask_of_cpu(cpu);
		irq_set_affinity(irq, mask);
	}
}
#endif

/*
 * L220/L230 event counter IRQ handler (
 */
static irqreturn_t l2x0_ec_interrupt(int irq, void *arg)
{
	u32 interrupt_status;
	unsigned int cnt;

	/* If it's an L2X0 EC interrupt, process it */
	interrupt_status = readl(l2x0_base + L2X0_MASKED_INTR_STAT);

	if (interrupt_status & L2X0_INTR_ECNTR) {
		/*
		 * A counter that has overflowed reads 0xffffffff
		 * This is not actually documented anywhere...
		 */
		for (cnt = 0; cnt < L2X0_NUM_COUNTERS; ++cnt) {
			if (l2x0_ec_read_value(cnt) == 0xffffffff) {
				oprofile_add_sample(get_irq_regs(),
						    COUNTER_L2X0_EC(cnt));
				l2x0_ec_reset(cnt);
			}
		}
		/*
		 * Clear the interrupt, and move it onto the next CPU.
		 */
		writel(L2X0_INTR_ECNTR, l2x0_base + L2X0_INTR_CLEAR);
#ifdef CONFIG_SMP
		l2x0_ec_rotate_irq(irq);
#endif
		return IRQ_HANDLED;
	}
	else {
		return IRQ_NONE;
	}
}

int l2x0_ec_start(void)
{
	int ret = 0;
	unsigned cnt;
	u32 cfg;

	/*
	 * Install handler for the L220/L230 event counter interrupt
	 */
	ret = request_irq(l2x0_irq, l2x0_ec_interrupt, IRQF_DISABLED,
			 "L2X0 EC", NULL);
	if (ret) {
		printk(KERN_ERR "oprofile: unable to request IRQ%u "
			"for L2X0 Event Counter\n", l2x0_irq);
		return ret;
	}

	/*
	 * Enable the event counter system
	 */
	l2x0_ec_system_setup(1);

	/*
	 * Configure the events we're interested in, and reset the counters
	 */
	for (cnt = 0; cnt < L2X0_NUM_COUNTERS; ++cnt) {
		if (counter_config[COUNTER_L2X0_EC(cnt)].enabled) {
			cfg = counter_config[COUNTER_L2X0_EC(cnt)].event & 0xFF;
			cfg <<= 2;
			cfg |= L2X0_EVENT_INTERRUPT_ON_OVF;
			l2x0_ec_write_config(cnt, cfg);
			l2x0_ec_reset(cnt);
		}
		else
			l2x0_ec_write_config(cnt, L2X0_EVENT_CONFIG_DISABLED);
	}

	return 0;
}

void l2x0_ec_stop(void)
{
	unsigned cnt;

	/* Disable individual L220/L230 event counters */
	for (cnt = 0; cnt < L2X0_NUM_COUNTERS; ++cnt)
		l2x0_ec_write_config(cnt, L2X0_EVENT_CONFIG_DISABLED);

	/* Disable L220/L230 event counter system */
	l2x0_ec_system_setup(0);

	/* Remove L220/L230 event counter interrupt handler */
	free_irq(l2x0_irq, NULL);
}
