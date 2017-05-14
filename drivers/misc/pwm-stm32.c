/*
 * (C) Copyright 2016
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * PWM framework driver for STM32. Note, the driver implements the
 * minimal basic, and should be extended to support the full set of
 * PWM features and channels available in STM32.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/list.h>

#include <asm/io.h>
#include <mach/pwm.h>
#include <mach/clock.h>

/*
 * Register offsets
 */
#define TIM_CR1			0x00
#define TIM_CR2			0x04
#define TIM_EGR			0x14
#define TIM_CCMR1		0x18
#define TIM_CCER		0x20
#define TIM_CNT			0x24
#define TIM_PSC			0x28
#define TIM_ARR			0x2C
#define TIM_CCR1		0x34

/*
 * Register bits
 */
#define CR1_CEN			(1 << 0)
#define CR1_ARPE		(1 << 7)

#define CR2_MMS_MASK		(7 << 4)
#define CR2_MMS_OC_REF(x)	((3 + (x)) << 4)

#define EGR_UG			(1 << 0)

#define CCMR_OCxM_MASK		((1 << 16) | (7 << 4))
#define CCMR_OCxM_PWM		((0 << 16) | (6 << 4))
#define CCMR_OCxM_OCPE		(1 << 3)
#define CCMR_OCxM_ODD(v)	((v) << 0)
#define CCMR_OCxM_EVE(v)	((v) << 8)

#define CCER_CCxE(c)		((1 << 0) << ((c) << 2))
#define CCER_CCxP(c)		((1 << 1) << ((c) << 2))

/*
 * STM32 PWM descriptor
 */
struct stm_pwm_chip {
	struct list_head node;
	struct device	*dev;
	void __iomem	*regs;

	int		tmr;
	int		chan;
	int		bits;

	bool		high_on_init;
	bool		trigger_output;
};

static LIST_HEAD(pwm_list);

static int stm_pwm_config(struct stm_pwm_chip *pc, int duty_ns, int period_ns)
{
	unsigned long long c;
	unsigned long period_cycles, prescale, duty, period;

	c = stm32_clock_get(CLOCK_PTMR1);
	c = c * period_ns;
	do_div(c, 1000000000ULL);
	period_cycles = c;

	do_div(c, pc->bits == 16 ? 0xFFFFUL : 0xFFFFFFFFUL);
	prescale = c;
	if (prescale > 0xFFFFUL)
		return -EINVAL;

	period = period_cycles / (prescale + 1);

	c = period;
	c = c * duty_ns;
	do_div(c, period_ns);
	duty = c;

	dev_dbg(pc->dev, "period=%dns,duty=%dns -> "
		"prescale=%ld,period=%ld,duty=%ld\n",
		period_ns, duty_ns, prescale, period, duty);

	writel(prescale, pc->regs + TIM_PSC);
	writel(duty, pc->regs + TIM_CCR1 + (pc->chan << 2));
	writel(period, pc->regs + TIM_ARR);

	writel(EGR_UG, pc->regs + TIM_EGR);

	return 0;
}

/*
 * PWM API: enable PWM signal
 */
static int stm_pwm_enable(struct stm_pwm_chip *pc)
{
	unsigned int val;
	unsigned long f;

	/*
	 * Disable IRQs to get the min possible delay between CCER & CR1 regs
	 * update
	 */
	local_irq_save(f);

	if (pc->high_on_init) {
		pc->high_on_init = false;

		val = readl(pc->regs + TIM_CCER);
		val &= ~CCER_CCxP(pc->chan);
		writel(val, pc->regs + TIM_CCER);
	}

	val = readl(pc->regs + TIM_CR1);
	val |= CR1_CEN;
	writel(val, pc->regs + TIM_CR1);

	local_irq_restore(f);

	return 0;
}

/*
 * PWM API: disable PWM signal
 */
static int stm_pwm_disable(struct stm_pwm_chip *pc)
{
	unsigned int val;
	unsigned long f;

	local_irq_save(f);
	val = readl(pc->regs + TIM_CR1);
	val &= ~CR1_CEN;
	writel(val, pc->regs + TIM_CR1);
	local_irq_restore(f);

	return 0;
}

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct stm_pwm_chip *pc;

	list_for_each_entry(pc, &pwm_list, node)
		if (pc->tmr == pwm_id)
			return (struct pwm_device *)pc;

	return NULL;
}
EXPORT_SYMBOL(pwm_request);

int pwm_enable(struct pwm_device *pwm)
{
	return stm_pwm_enable((struct stm_pwm_chip *)pwm);
}
EXPORT_SYMBOL(pwm_enable);

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	return stm_pwm_config((struct stm_pwm_chip *)pwm, duty_ns, period_ns);
}
EXPORT_SYMBOL(pwm_config);

void pwm_disable(struct pwm_device *pwm)
{
	stm_pwm_disable((struct stm_pwm_chip *)pwm);
}
EXPORT_SYMBOL(pwm_disable);

static const struct of_device_id stm_pwm_of_match[] = {
	{ .compatible = "st,stm32-pwm", },
	{ }
};
MODULE_DEVICE_TABLE(of, pwm_of_match);

static int stm_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_pwm_data *pdata = dev->platform_data;
	struct stm_pwm_chip *pc = NULL;
	unsigned int val, ofs;
	struct resource *res;
	int rv, tmr, chan;

	if (!pdata) {
		dev_err(dev, "Error: No platfrom data\n");
		return -ENODEV;
	}

	tmr = pdata->tmr;
	chan = pdata->chan;

	/*
	 * Supported for now:
	 *  - TIM2-5 timers with 1-4 channels
	 *  - TIM9,12 timers with 1-2 channels
	 */
	if ((tmr < 2 || tmr > 5 || chan < 1 || chan > 4)
	    && ((tmr != 9 && tmr != 12) || (chan != 1 && chan != 2))) {
		dev_err(dev, "not supported timer/channel %d/%d\n",
			tmr, chan);
		rv = -EINVAL;
		goto out;
	}

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc) {
		rv = -ENOMEM;
		goto out;
	}
	pc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(pc->regs)) {
		dev_err(dev, "can't map timer registers\n");
		rv = PTR_ERR(pc->regs);
		goto out;
	}

	/*
	 * 'trigger-output' allows to generate internal TRGO
	 */
	pc->trigger_output = pdata->trigger_output;

	/*
	 * 'high-on-init' allows to initialize PWM with HIGH output until
	 * first PWM run.
	 */
	pc->high_on_init = pdata->high_on_init;

	/* Detect whether the timer is 16 or 32 bits */
	writel(~0U, pc->regs + TIM_ARR);
	pc->bits = readl(pc->regs + TIM_ARR) == ~0U ? 32 : 16;
	pc->tmr = tmr;
	pc->chan = chan - 1;

	if (pc->trigger_output) {
		/*
		 * Configure timer for triggering internal event
		 */
		val = readl(pc->regs + TIM_CR2);
		val &= ~CR2_MMS_MASK;
		val |= CR2_MMS_OC_REF(chan);
		writel(val, pc->regs + TIM_CR2);
	}

	/*
	 * Configure the specified timer channel
	 */
	ofs = TIM_CCMR1 + ((pc->chan / 2) << 2);
	val = readl(pc->regs + ofs);
	if (chan % 2) {
		val &= ~CCMR_OCxM_ODD(CCMR_OCxM_MASK);
		val |= CCMR_OCxM_ODD(CCMR_OCxM_PWM);
		val |= CCMR_OCxM_ODD(CCMR_OCxM_OCPE);
	} else {
		val &= ~CCMR_OCxM_EVE(CCMR_OCxM_MASK);
		val |= CCMR_OCxM_EVE(CCMR_OCxM_PWM);
		val |= CCMR_OCxM_EVE(CCMR_OCxM_OCPE);
	}
	writel(val, pc->regs + ofs);

	val = readl(pc->regs + TIM_CCER);
	val |= CCER_CCxE(pc->chan);
	if (pc->high_on_init)
		val |= CCER_CCxP(pc->chan);
	writel(val, pc->regs + TIM_CCER);

	val = readl(pc->regs + TIM_CR1);
	val |= CR1_ARPE;
	writel(val, pc->regs + TIM_CR1);

	dev_info(dev, "basing on TIM%d.%d(x%d)\n", tmr, chan, pc->bits);
	platform_set_drvdata(pdev, pc);

	list_add(&pc->node, &pwm_list);
	if (!pdata->duty_cycle) {
		rv = 0;
		goto out;
	}

	stm_pwm_config(pc, pdata->duty_cycle, pdata->period);
	stm_pwm_enable(pc);

	rv = 0;
out:
	if (rv) {
		if (pc)
			devm_kfree(dev, pc);
	}

	return rv;
}

static struct platform_driver stm_pwm_driver = {
	.driver		= {
		.name	= "stm32-pwm",
		.owner	= THIS_MODULE,
	},
	.probe		= stm_pwm_probe,
};

static int __init stm_pwm_init(void)
{
	int ret;
	ret = platform_driver_register(&stm_pwm_driver);
	return ret;
}
module_init(stm_pwm_init);

MODULE_DESCRIPTION("STM32 PWM driver");
MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_LICENSE("GPL");
