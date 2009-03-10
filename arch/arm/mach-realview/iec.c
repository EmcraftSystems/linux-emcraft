/*
 * arch/arm/mach-realview/iec.c
 *
 * Copyright (C) 2009 ARM Limited
 * Written by Turhan Oz <turhan.oz@gmail.com>
 * Maintained by Catalin Marinas <catalin.marinas@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/amba/bus.h>

#include <asm/io.h>

#define	CPU_TRANSITION_LATENCY	100000

/*
 * IEC OFFSET
 */
#define	IEC_CFGCPUFREQ_OFFSET	0x020   /* max Freq (ro)*/
#define	IEC_DPCTGTPERF_OFFSET	0x008   /* target performance (wo)*/
#define	IEC_DPCCRNTPERF_OFFSET	0x00C   /* current performance (ro)*/

#define	IEC_ID0_OFFSET		0xFF0	/* identification register 0 */
#define	IEC_ID1_OFFSET		0xFF4	/* identification register 1 */
#define	IEC_ID2_OFFSET		0xFF8	/* identification register 2 */
#define	IEC_ID3_OFFSET		0xFFC	/* identification register 3 */

#define	IEC_PERIFID0_OFFSET	0xFE0	/* Peripheral id register 0 */
#define	IEC_PERIFID1_OFFSET	0xFE4	/* Peripheral id register 1 */
#define	IEC_PERIFID2_OFFSET	0xFE8	/* Peripheral id register 2 */
#define	IEC_PERIFID3_OFFSET	0xFEC	/* Peripheral id register 3 */
#define	IEC_PERIFID4_OFFSET	0xFD0	/* Peripheral id register 4 */
#define	IEC_PERIFID5_OFFSET	0xFD4	/* Peripheral id register 5 */
#define	IEC_PERIFID6_OFFSET	0xFD8	/* Peripheral id register 6 */
#define	IEC_PERIFID7_OFFSET	0xFDC	/* Peripheral id register 7 */

static void __iomem *iec_base;

/*
 * This function returns a performance (4 - 128) out of a frequency within
 * the boundaries (min/max).
 */
static u32 get_performance(struct cpufreq_policy *policy,
		        	unsigned int target_freq)
{
	unsigned int perf = 0;

	if(target_freq < policy->min)
		target_freq = (u32)policy->min;
	if(target_freq > policy->max)
		target_freq = (u32)policy->max;

	perf = (target_freq*0x80)/policy->cpuinfo.max_freq;
	if (perf < 4)
		perf = 4;

	return perf;
}

/*
 * This function set the IECDPCTGTPERF register out of the performance level
 * (4 - 128).
 */
static void set_performance(u32 perf)
{
	if(perf > 0x80)
		perf = 0x80;
	if(perf < 4)
		perf = 4;
	writel(perf, iec_base + IEC_DPCTGTPERF_OFFSET);
}

/*
 * Verify if the requested policy is within the limits.
 */
static int iec_verify_policy(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
		policy->cpuinfo.max_freq);
	return 0;
}

/*
 * Return the current IEC performance (4-128).
 */
static inline u32 iec_get_current_perf(void){
	return (unsigned int)readl(iec_base + IEC_DPCCRNTPERF_OFFSET);
}

/*
 * Return the current speed (i.e the current freq) of the CPU The register
 * DPCCRNTPERF returns the performance (4 - 128).
 */
static unsigned int iec_get_speed(unsigned int cpu)
{
	unsigned int freq = 0;
	unsigned int current_perf = 0;
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

	if (cpu)
		return 0;

	current_perf = iec_get_current_perf();
	pr_debug("iec: current performance value (from register): %u\n",
		 current_perf);

	freq = (current_perf*policy->cpuinfo.max_freq)/0x80;
	pr_debug("iec: current frequency (from register) : %u\n", freq);

	return freq;
}

/*
 * Here is where the Freq changes.  The CPUFreq driver must set the new
 * frequency when called here.
 */
static int iec_set_target(struct cpufreq_policy *policy,
                          unsigned int target_freq,
                          unsigned int relation)
{
	struct cpufreq_freqs freqs;
	unsigned int cur = iec_get_speed(0);
	unsigned int perf;

	/*
	 * Some governors do not respects CPU and policy lower limits which
	 * leads to bad things (division by zero etc), ensure that such things
	 * do not happen.
	 */
	if(target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;
	if(target_freq < policy->min)
		target_freq = policy->min;

	switch(relation){
	case CPUFREQ_RELATION_L:
		pr_debug("iec: try to select a freq higher than or equal to %u\n", 
			 target_freq);
		break;
	case CPUFREQ_RELATION_H:
		pr_debug("iec: try to select a freq lower than or equal to %u\n", 
			 target_freq);
		break;
	}

	perf =  get_performance(policy, target_freq);
	pr_debug("iec: new frequency %u (-> perf level %u)\n", target_freq,
		 perf);

	freqs.old = cur;
	freqs.new = target_freq;
	freqs.cpu = 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	set_performance(perf);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

/*
 * Whenever a new CPU is registered with the device model, or after the
 * cpufreq driver registers itself, the per-CPU initialization function
 * cpufreq_driver.init is called.
 */
static int __init iec_cpu_init(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->cpuinfo.max_freq = (unsigned int)readl(iec_base + IEC_CFGCPUFREQ_OFFSET);
	policy->cpuinfo.min_freq = policy->cpuinfo.max_freq/2; /* 50% of cpuinfo.max_freq */ 
	policy->cpuinfo.transition_latency = CPU_TRANSITION_LATENCY;

	policy->max = policy->cpuinfo.max_freq;
	policy->min = policy->cpuinfo.min_freq;
	policy->cur = (iec_get_current_perf()*policy->cpuinfo.max_freq)/0x80;

	pr_debug("iec: cpu_max : %u, cpu_min : %u, cpu_latency : %u \n",
		 policy->cpuinfo.max_freq, policy->cpuinfo.min_freq,
		 policy->cpuinfo.transition_latency);

	return 0;
}

/*
 * CPUFreq Driver Interface.
 */
static struct cpufreq_driver iec_cpufreq_driver = {
	.name		= "iec",
	.flags		= CPUFREQ_STICKY,
	.verify		= iec_verify_policy,
	.target		= iec_set_target,
	.get		= iec_get_speed,
	.init		= iec_cpu_init,
};

static int __init iec_probe(struct amba_device *dev, void *id)
{
	int ret;

	ret = amba_request_regions(dev, NULL);
	if (ret)
		goto out;

	iec_base = ioremap(dev->res.start, resource_size(&dev->res));
	if (!iec_base) {
		ret = -ENOMEM;
		goto release;
	}

	ret = cpufreq_register_driver(&iec_cpufreq_driver);
	if (ret == 0)
		goto out;

	iounmap(iec_base);
release:
	amba_release_regions(dev);
out:
	return ret;
}

static int iec_remove(struct amba_device *dev)
{
	cpufreq_unregister_driver(&iec_cpufreq_driver);
	iounmap(iec_base);
	amba_release_regions(dev);

	return 0;
}

static struct amba_id iec_ids[] __initdata = {
	{
		.id	= 0x08041750,
		.mask	= 0xffffffff
	},
	{ 0, 0 }
};

static struct amba_driver iec_driver = {
	.drv = {
		.name	= "iec",
	},
	.id_table	= iec_ids,
	.probe		= iec_probe,
	.remove		= iec_remove,
};

static int __init iec_init(void)
{
	return amba_driver_register(&iec_driver);
}

static void __exit iec_exit(void)
{
	amba_driver_unregister(&iec_driver);
}

module_init(iec_init);
module_exit(iec_exit);

MODULE_LICENSE("GPL");
