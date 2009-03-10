/**
 * @file op_model_v6-7.c
 * ARM V6 and V7 Performance Monitor models
 *
 * Based on op_model_xscale.c
 *
 * @remark Copyright 2007 ARM SMP Development Team
 * @remark Copyright 2000-2004 Deepak Saxena <dsaxena@mvista.com>
 * @remark Copyright 2000-2004 MontaVista Software Inc
 * @remark Copyright 2004 Dave Jiang <dave.jiang@intel.com>
 * @remark Copyright 2004 Intel Corporation
 * @remark Copyright 2004 Zwane Mwaikambo <zwane@arm.linux.org.uk>
 * @remark Copyright 2004 OProfile Authors
 *
 * @remark Read the file COPYING
 *
 * @author Tony Lindgren <tony@atomide.com>
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/system.h>

#include <linux/smp.h>

#include "op_counter.h"
#include "op_arm_model.h"
#include "op_arm11.h"
#include "op_v7.h"
#include "op_scu.h"
#include "op_l2x0.h"

static int arm11_irqs[] = {
	[0]	= IRQ_PMU_CPU0,
#ifdef CONFIG_SMP
	[1]	= IRQ_PMU_CPU1,
	[2]	= IRQ_PMU_CPU2,
	[3]	= IRQ_PMU_CPU3
#endif
};

static int v7_irqs[] = {
	[0]	= IRQ_PMU_CPU0,
#ifdef CONFIG_SMP
	[1]	= IRQ_PMU_CPU1,
	[2]	= IRQ_PMU_CPU2,
	[3]	= IRQ_PMU_CPU3
#endif
};


/*
 * Functions and struct to enable calling a function on all CPUs in an SMP
 * system. This works on a non-SMP system too (i.e. just calls the function!)
 */
struct em_function_data {
	int (*fn)(void);
	int ret;
};

static void em_func(void *data)
{
	struct em_function_data *d = data;
	int ret = d->fn();
	if (ret)
		d->ret = ret;
}

static int em_call_function(int (*fn)(void))
{
	struct em_function_data data;

	data.fn = fn;
	data.ret = 0;

		get_cpu();
	if (is_smp())
		smp_call_function(em_func, &data, 1);
	em_func(&data);
	put_cpu();

	return data.ret;
}


/*
 * Why isn't there a function to route an IRQ to a specific CPU in
 * genirq?
 */
#ifdef CONFIG_SMP
void em_route_irq(int irq, unsigned int cpu)
{
	irq_set_affinity(irq, *(get_cpu_mask(cpu)));
}
#endif

/*
 * ARM V6 Oprofile callbacks
 */
static void v6_stop(void)
{
	em_call_function(arm11_stop_pmu);
	arm11_release_interrupts(arm11_irqs, ARRAY_SIZE(arm11_irqs));
	if (is_smp())
		scu_stop();
	if (have_l2x0())
		l2x0_ec_stop();
}

static int v6_start(void)
{
	int ret;
#ifdef CONFIG_SMP
	unsigned i;

	if (is_smp()) {
		/*
		 * Send SCU and CP15 PMU interrupts to the "owner" CPU.
		 */
		for (i=0; i<CONFIG_NR_CPUS; ++i) {
			em_route_irq(IRQ_PMU_SCU0 + 2 * i, i);
			em_route_irq(IRQ_PMU_SCU1 + 2 * i, i);
			em_route_irq(IRQ_PMU_CPU0 + i, i);
		}
	}
#endif

	ret = arm11_request_interrupts(arm11_irqs, ARRAY_SIZE(arm11_irqs));
	if (ret == 0) {
		em_call_function(arm11_start_pmu);

		if (is_smp())
			ret = scu_start();

		if (!ret && have_l2x0())
			ret = l2x0_ec_start();

		if (ret)
			arm11_release_interrupts(arm11_irqs, ARRAY_SIZE(arm11_irqs));
	}
	return ret;
}

static int v6_init(void)
{
	return 0;
}

static int v6_setup_ctrs(void)
{
	int ret;
	ret = em_call_function(arm11_setup_pmu);

	if (ret == 0 && is_smp())
		scu_setup();

	if (ret == 0 && have_l2x0())
		l2x0_ec_setup();

	return ret;
}

/*
 * ARM V7 Oprofile callbacks
 */
static int v7_init(void)
{
	return 0;
}


static int v7_setup_ctrs(void)
{
	int ret;

	ret = em_call_function(v7_setup_pmu);

	if (ret == 0 && have_l2x0())
		l2x0_ec_setup();

	return ret;
}

static int v7_start(void)
{
	int ret;
#ifdef CONFIG_SMP
	unsigned i;

	if (is_smp()) {
		/*
		 * Send CP15 PMU interrupts to the owner CPU.
		 */
		for (i=0; i<CONFIG_NR_CPUS; ++i) {
			em_route_irq(IRQ_PMU_CPU0 + i, i);
		}
	}
#endif

	ret = v7_request_interrupts(v7_irqs, ARRAY_SIZE(v7_irqs));
	if (ret == 0) {
		em_call_function(v7_start_pmu);

		if (have_l2x0())
			ret = l2x0_ec_start();

		if (ret)
			v7_release_interrupts(v7_irqs, ARRAY_SIZE(v7_irqs));
	}
	return ret;
}

static void v7_stop(void)
{
	em_call_function(v7_stop_pmu);
	v7_release_interrupts(v7_irqs, ARRAY_SIZE(v7_irqs));
	if (have_l2x0())
		l2x0_ec_stop();
}


struct op_arm_model_spec op_armv6_spec = {
	.init		= v6_init,
	.num_counters	= NUM_COUNTERS,
	.setup_ctrs	= v6_setup_ctrs,
	.start		= v6_start,
	.stop		= v6_stop,
	.name		= "arm/v6",  /* This may get overwritten in common.c */
};

struct op_arm_model_spec op_armv7_spec = {
	.init		= v7_init,
	.num_counters	= NUM_COUNTERS,
	.setup_ctrs	= v7_setup_ctrs,
	.start		= v7_start,
	.stop		= v7_stop,
	.name		= "arm/v7",  /* This gets overwritten in common.c */
};
