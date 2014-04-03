/*
 * Kinetis On-Chip Real Time Clock Driver
 *
 * (C) Copyright 2014
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/rtc.h>

/*
 * Debug output control. While debugging, have KINETIS_RTC_DEBUG defined.
 * In deployment, make sure that KINETIS_RTC_DEBUG is undefined
 * to avoid the performance and size overhead of debug messages.
 */
#define KINETIS_RTC_DEBUG
#if 0
#undef KINETIS_RTC_DEBUG
#endif

#if defined(KINETIS_RTC_DEBUG)

/*
 * Driver verbosity level: 0->silent; >0->verbose
 */
static int kinetis_rtc_debug = 0;

/*
 * User can change verbosity of the driver
 */
module_param(kinetis_rtc_debug, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(kinetis_rtc_debug, "Kinetis RTC clock");

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (kinetis_rtc_debug >= level) printk(KERN_INFO "%s: " fmt,	\
					__func__, ## args)
#if !defined(MODULE)

/*
 * Parser for the boot time driver verbosity parameter
 * @param str		user defintion of the parameter
 * @returns		1->success
 */
static int __init kinetis_rtc_debug_setup(char * str)
{
	get_option(&str, &kinetis_rtc_debug);
	return 1;
}
__setup("kinetis_rtc_debug=", kinetis_rtc_debug_setup);

#endif /* !defined(MODULE) */

#else

#define d_printk(level, fmt, args...)

#endif /* defined(KINETIS_RTC_DEBUG) */

/*
 * Description of the the Kinetis RTC hardware registers
 */
struct kinetis_rtc_regs {
	u32 rtc_tsr;
	u32 rtc_tpr;
	u32 rtc_tar;
	u32 rtc_tcr;
	u32 rtc_cr;
	u32 rtc_sr;
	u32 rtc_lr;
	u32 rtc_ier;
	u32 rtc_ttsr;
	u32 rtc_mer;
	u32 rtc_mlcr;
	u32 rtc_mchr;
	u32 rtc_ter;
	u32 rtc_tdr;
	u32 rtc_ttr;
	u32 rtc_tir;
	u32 rsrvd[496];
	u32 rtc_war;
	u32 rtc_rar;
};

/*
 * Access handle for the control registers
 */
#define KINETIS_RTC_REGS(regs)	((volatile struct kinetis_rtc_regs *)(regs))
#define KINETIS_RTC(c)		(KINETIS_RTC_REGS(c->regs))

/*
 * Private data structure for the RTC device
 */
struct kinetis_rtc {
	struct rtc_device *rtc_dev;
	spinlock_t lock;
	struct clk *clk;
	int irq;
	void * __iomem regs;
};

/*
 * Initialize the RTC hardware
 * @rtc			RTC private data structure
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_hw_init(struct kinetis_rtc *rtc)
{
	int ret = 0;

	/*
	 * On the Emcraft Kinetis SOM, U-Boot provides basic configuration
	 * for the RTC OSC. We therefore don't do it again.
	 * Other hardware designs may have to configure the OSC and
	 * the oscicallotor loads as appropriate for a specific design.
	 */

	/*
	 * Disable all interrupts so something that might have been set up
	 * by a previous software sessions doesn't trigger once we enable
	 * interrupts.
	 */
	writel(0x0, &KINETIS_RTC(rtc)->rtc_ier);

	/*
	 * Set the time compensation register
	 */
	writel(0x0, &KINETIS_RTC(rtc)->rtc_tcr);

	/*
	 * Check that Seconds Counter is not stuck
	 */
	if (readl(&KINETIS_RTC(rtc)->rtc_sr) & (1<<0)) {

		/*
		 * This condition means that time is invalid.
		 * This does happen on VBAT POR (eg. SOM is re-plugged).
		 * Disable Seconds Counter and clear the condition
		 * by writing a start counter to Seconds Counter.
		 */
		writel(0x0, &KINETIS_RTC(rtc)->rtc_sr);
		writel(0x1, &KINETIS_RTC(rtc)->rtc_tsr);
	}

	/*
	 * Time Counter is probably already running but if it is not
	 * we need to get is started
	 */
	writel(1<<4, &KINETIS_RTC(rtc)->rtc_sr);

	d_printk(1, "cr=%x,sr=%x,ier=%x,ret=%d\n",
		readl(&KINETIS_RTC(rtc)->rtc_cr),
		readl(&KINETIS_RTC(rtc)->rtc_sr),
		readl(&KINETIS_RTC(rtc)->rtc_ier), ret);
	return ret;
}

/*
 * Shutdown the RTC hardware
 * @rtc			RTC private data structure
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_hw_shutdown(struct kinetis_rtc *rtc)
{
	int ret = 0;

	d_printk(1, "ret=%d\n", ret);
	return ret;
}

/*
 * RTC interrupt handler
 * @rtc			RTC private data structure
 * @returns		IRQ handler return code
 */
static irqreturn_t kinetis_rtc_irq(int irq, void *dev_id)
{
	struct kinetis_rtc *rtc = (struct kinetis_rtc *) dev_id;
	long unsigned int secs;

	spin_lock(&rtc->lock);

	/*
	 * Disable alarm interrupts
	 */
	writel(readl(&KINETIS_RTC(rtc)->rtc_ier) & ~(1<<2),
		&KINETIS_RTC(rtc)->rtc_ier);

	/*
	 * Clear the alarm condition
	 */
	secs = 0;
	writel(&secs, &KINETIS_RTC(rtc)->rtc_tar);

	spin_unlock(&rtc->lock);

	d_printk(2, "RTC interrupt\n");
	return IRQ_HANDLED;
}

/*
 * Return the current time and date from the RTC
 * @dev			RTC device
 * @tm			Gregorian time and date
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct kinetis_rtc *rtc = dev_get_drvdata(dev);
	long unsigned int secs;
	int ret = 0;

	spin_lock_irq(&rtc->lock);
	secs = readl(&KINETIS_RTC(rtc)->rtc_tsr); 
	rtc_time_to_tm(secs, tm);
	spin_unlock_irq(&rtc->lock);

	d_printk(2, "secs=%ld,ret=%d\n", secs, ret);
	return ret;
}

/*
 * Write a time and date to the RTC
 * @dev			RTC device
 * @tm			Gregorian time and date
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct kinetis_rtc *rtc = dev_get_drvdata(dev);
	long unsigned int secs;
	int ret;

	spin_lock_irq(&rtc->lock);

	/*
	 * Validate the supplied time
	 */
	ret = rtc_valid_tm(tm);
	if (ret) {
		dev_err(dev, "invalid time supplied\n");
		goto Done;
	}

	/*
	 * Convert a Greorian date to seconds since epoch
	 */
	ret = rtc_tm_to_time(tm, &secs);
	if (ret) {
		dev_err(dev, "failed to convert tm to secs\n");
		goto Done;
	}

	/*
	 * Update the Time Seconds counter
	 */
	writel(readl(&KINETIS_RTC(rtc)->rtc_sr) & ~(1<<4),
		&KINETIS_RTC(rtc)->rtc_sr);
	writel(secs, &KINETIS_RTC(rtc)->rtc_tsr);
	writel(readl(&KINETIS_RTC(rtc)->rtc_sr) | (1<<4),
		&KINETIS_RTC(rtc)->rtc_sr);

Done:
	spin_unlock_irq(&rtc->lock);

	d_printk(2, "secs=%ld,ret=%d\n", secs, ret);
	return ret;
}
	
#if defined(CONFIG_RTC_DRV_KINETIS_HCTOSYS)

/*
 * Synchronize the system time with the RTC
 * @dev			RTC device
 * @verbos		Print an info message
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_sync_time(struct device *dev, int verbose)
{
	struct kinetis_rtc *rtc = dev_get_drvdata(dev);
	long unsigned int secs;
	struct timespec	time;
	int ret = 0;

	spin_lock_irq(&rtc->lock);
	secs = readl(&KINETIS_RTC(rtc)->rtc_tsr); 
	set_normalized_timespec(&time, secs, NSEC_PER_SEC >> 1);
	do_settimeofday(&time);
	spin_unlock_irq(&rtc->lock);

	if (verbose) {
		dev_info(dev,
			"System time set to RTC time (seconds): %ld\n",
			secs);
	}	

	d_printk(2, "secs=%ld,ret=%d\n", secs, ret);
	return ret;
}

#endif /* CONFIG_RTC_DRV_KINETIS_HCTOSYS */

/*
 * Return the current settings of the alarm
 * @dev			RTC device
 * @alrm		RTC wkalrm
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct kinetis_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	long unsigned int secs;
	unsigned int ret = 0;

	spin_lock_irq(&rtc->lock);

	secs = readl(&KINETIS_RTC(rtc)->rtc_tar); 
	rtc_time_to_tm(secs, tm);
	alrm->enabled = (readl(&KINETIS_RTC(rtc)->rtc_ier) & (1<<2)) ? 1 : 0;
	alrm->pending = (readl(&KINETIS_RTC(rtc)->rtc_sr) & (1<<3)) ? 1 : 0;

	spin_unlock_irq(&rtc->lock);

	d_printk(2, "secs=%ld,ret=%d\n", secs, ret);
	return ret;
}

/*
 * Set an RTC alarm
 * @dev			RTC device
 * @alrm		RTC wkalrm
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct kinetis_rtc *rtc = dev_get_drvdata(dev);
	long unsigned int secs;
	int ret = 0;

	spin_lock_irq(&rtc->lock);

	/*
	 * Set the Time Alarm
	 */
	rtc_tm_to_time(&alrm->time, &secs);
	writel(secs - 1, &KINETIS_RTC(rtc)->rtc_tar);

	/*
	 * Enable alarm interrupts
	 */
	writel(readl(&KINETIS_RTC(rtc)->rtc_ier) | (1<<2),
		&KINETIS_RTC(rtc)->rtc_ier);

	spin_unlock_irq(&rtc->lock);

	d_printk(2, "secs=%ld,delta=%ld,ret=%d\n",
		secs, secs - readl(&KINETIS_RTC(rtc)->rtc_tsr), ret);
	return ret;
}

/*
 * Supported RTC callbacks
 */
static struct rtc_class_ops kinetis_rtc_ops = {
	.read_time		= kinetis_rtc_read_time,
	.set_time		= kinetis_rtc_set_time,
	.read_alarm		= kinetis_rtc_read_alarm,
	.set_alarm		= kinetis_rtc_set_alarm,
};

/*
 * Instantiate the RTC device
 * @dev			RTC platform device
 * @returns		0->success, <0->error code
 */
static int __devinit kinetis_rtc_probe(struct platform_device *pdev)
{
	struct kinetis_rtc *rtc;
	struct device *dev = &pdev->dev;
	struct resource *regs;
	int irq;
	int ret;

	/*
	 * Allocate memory for the RTC private data structure
	 */
	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (! rtc) {
		dev_err(&pdev->dev, "kzalloc failure\n");
		ret = -ENOMEM;
		goto Done_kzalloc_failure;
	}

	/*
	 * Tie the private data structure to the platform device
	 */
	platform_set_drvdata(pdev, rtc);

	/*
	 * Get the IRQ number from the platform device
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq failure: %d\n", irq);
		ret = irq;
		goto Done_platform_get_irq_failure;
	}
	rtc->irq = irq;

	/*
	 * Get the register base from the platform device
	 */
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (! regs) {
		dev_err(&pdev->dev, "platform_get_resource failure\n");
		ret = -ENXIO;
		goto Done_platform_get_resource_failure;
	}

	/*
 	 * Map in the controller registers
 	 */
	rtc->regs = ioremap(regs->start, resource_size(regs));
	if (!rtc->regs) {
		dev_err(&pdev->dev, "ioremap failure: %x\n", regs->start);
		ret = -EINVAL;
		goto Done_ioremap_failure;
	}

	/*
	 * Get the clock
	 */
	rtc->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(rtc->clk)) {
		ret = PTR_ERR(rtc->clk);
		dev_err(&pdev->dev, "clk_get failure\n");
		goto Done_clk_get_failure;
	}

	/*
	 * Enable the clock to the RTC
	 */
	ret = clk_enable(rtc->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk_enable failure\n");
		goto Done_clk_enable_failure;
	}

	/*
	 * Initialize the RTC hardware
	 */
	ret = kinetis_rtc_hw_init(rtc);
	if (ret) {
		dev_err(&pdev->dev, "kinetis_rtc_hw_init failure\n");
		goto Done_kinetis_hw_init_failure;
	}

	/*
	 * RTC may wakeup
	 */
	device_init_wakeup(dev, 1);

	/* 
 	 * Init spin lock
 	 */
	spin_lock_init(&rtc->lock);

	/*
	 * Register our RTC with the RTC framework
	 */
	rtc->rtc_dev = rtc_device_register(
		pdev->name, dev, &kinetis_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc_dev)) {
		dev_err(&pdev->dev, "rtc_device_register failure\n");
		ret = PTR_ERR(rtc->rtc_dev);
		goto Done_rtc_device_register_failure;
	}

	/*
	 * Handle RTC interrupts 
	 */
	ret = request_irq(rtc->irq, kinetis_rtc_irq, 0, pdev->name, rtc);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failure: %d\n",
			rtc->irq);
		goto Done_request_irq_failure;
	}

	/*
	 * Here, means success
	 */
	ret = 0;
	dev_info(&pdev->dev, "Kinetis on-chip RTC at %p, irq %d\n",
		rtc->regs, rtc->irq);

#if defined(CONFIG_RTC_DRV_KINETIS_HCTOSYS)
	/*
	 * Sync up the system time with RTC
	 */
	kinetis_rtc_sync_time(&pdev->dev, 1);
#endif
	goto Done;

/*
 * Clean-up on failure
 */
Done_request_irq_failure:
	rtc_device_unregister(rtc->rtc_dev);
Done_rtc_device_register_failure:
	kinetis_rtc_hw_shutdown(rtc);
Done_kinetis_hw_init_failure:
	clk_disable(rtc->clk);
Done_clk_enable_failure:
	clk_put(rtc->clk);
Done_clk_get_failure:
	iounmap(rtc->regs);
Done_ioremap_failure:
Done_platform_get_resource_failure:
Done_platform_get_irq_failure:
	platform_set_drvdata(pdev, NULL);
	kfree(rtc);
Done_kzalloc_failure:

Done:
	d_printk(1, "dev=%s,ret=%d\n", dev_name(&pdev->dev), ret);
	return ret;
}

/*
 * Shutdown of the Kinetis on-chip RTC 
 * @dev			RTC platform device
 * @returns		0->success, <0->error code
 */
static int __devexit kinetis_rtc_remove(struct platform_device *pdev)
{
	struct kinetis_rtc *rtc = platform_get_drvdata(pdev);

	free_irq(rtc->irq, rtc);
	rtc_device_unregister(rtc->rtc_dev);
	kinetis_rtc_hw_shutdown(rtc);
	clk_disable(rtc->clk);
	clk_put(rtc->clk);
	iounmap(rtc->regs);
	platform_set_drvdata(pdev, NULL);
	kfree(rtc);

	d_printk(1, "dev=%s\n", dev_name(&pdev->dev));
	return 0;
}

#if defined(CONFIG_PM)

/*
 * Enter-low-power callback 
 * @dev			RTC platform device
 * @state		PM state
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_suspend(
	struct platform_device *pdev, pm_message_t state)
{
	d_printk(1, "dev=%s\n", dev_name(&pdev->dev));
	return 0;
}

/*
 * Exit-low-power callback 
 * @dev			RTC platform device
 * @returns		0->success, <0->error code
 */
static int kinetis_rtc_resume(struct platform_device *pdev)
{
#if defined(CONFIG_RTC_DRV_KINETIS_HCTOSYS)
	kinetis_rtc_sync_time(&pdev->dev, 0);
#endif

	d_printk(1, "dev=%s\n", dev_name(&pdev->dev));
	return 0;
}

#endif /* CONFIG_PM */

/*
 * Driver data structure
 */
static struct platform_driver kinetis_rtc_driver = {
	.driver		= {
		.name	= "rtc-kinetis",
		.owner	= THIS_MODULE,
	},
	.probe		= kinetis_rtc_probe,
	.remove		= __devexit_p(kinetis_rtc_remove),
#if defined(CONFIG_PM)
	.suspend	= kinetis_rtc_suspend,
	.resume		= kinetis_rtc_resume,
#endif
};

/*
 * Driver init
 * @returns		0->success, <0->error code
 */
static int __init kinetis_rtc_init(void)
{
	int ret;
	
	ret = platform_driver_register(&kinetis_rtc_driver);

	d_printk(1, "drv=%s,ret=%d\n", kinetis_rtc_driver.driver.name, ret);
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit kinetis_rtc_exit(void)
{
	d_printk(1, "drv=%s\n", kinetis_rtc_driver.driver.name);
	platform_driver_unregister(&kinetis_rtc_driver);
}

module_init(kinetis_rtc_init);
module_exit(kinetis_rtc_exit);

MODULE_DESCRIPTION("Kinetis On-Chip Real Time Clock Driver");
MODULE_AUTHOR("Vladimir Khusainov <vlad@emcraft.com>");
MODULE_LICENSE("GPL");
