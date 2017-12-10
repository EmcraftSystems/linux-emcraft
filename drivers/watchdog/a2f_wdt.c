/*
 * drivers/char/watchdog/a2f_wdt.c
 *
 * Watchdog driver for NXP LPC23xx/LPC24xx devices
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>,
 *
 * Copyright (C) 2011 NXP Semiconductors
 *
 * Port to LPC178X
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Sergei Poselenov <sposelenov@emcraft.com>
 *
 * Port to Microsemi SmartFusion
 * (C) Copyright 2017
 * Z-LASER Optoelektronik GmbH <www.z-laser.com>
 * Reinhard Mötzel <moetzel@z-laser.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>

#define MODULE_NAME "a2f-wdt"

/* Watchdog timer register set definition */
#define A2F_WDOGVALUE   0x00
#define A2F_WDOGLOAD    0x04
#define A2F_WDOGMVRP    0x08
#define A2F_WDOGREFRESH 0x0C
#define A2F_WDOGENABLE  0x10
#define A2F_WDOGCONTROL 0x14

/* WDOGREFRESH bit definitions */
#define A2F_WDOGREFRESH_REFRESH_KEY  0xAC15DE42

/* WDOGENABLE bit definitions */
#define A2F_WDOGENABLE_ENABLE        0x1
#define A2F_WDOGENBALE_DISABLE_KEY   0x4C6E55FA

/* WDOGCONTROL bit definitions */
#define A2F_WDOGCONTROL_MODE         0x4
#define A2F_WDOGCONTROL_WAKEUPINTEN  0x2
#define A2F_WDOGCONTROL_TIMEOUTINTEN 0x1


/* MSS Status Register */
#define A2F_MSS_SR                   0xE004201C

/* MSS_SR bit definitions  */
#define A2F_MSS_SR_WDOGTIMEOUTEVENT  0x2



/*
 * The A2F watchdog clock is always running at 100 MHz.
 */
#define A2F_WDOG_CLK    100000000 

#define MAX_HEARTBEAT   40 /* seconds */

static struct platform_device *a2f_wdt_dev;

struct a2f_wdt_dev {
	void __iomem		*base;
	struct device		*dev;
	unsigned long		num_users;
	struct resource		*mem, *res;
	struct miscdevice	miscdev;
	struct clk		*clk;
	long			heartbeat;
};

static void a2f_wdt_feed(void __iomem *base)
{
	unsigned long flags;

	/* WDT feeds must not be interrupted */
	local_irq_save(flags);
	__raw_writel(A2F_WDOGREFRESH_REFRESH_KEY, base + A2F_WDOGREFRESH);
	//printk(KERN_CRIT MODULE_NAME
	//        ":watchdog triggered...\n");
	local_irq_restore(flags);
}

static void a2f_wdt_show_warning(void)
{
	printk(KERN_CRIT MODULE_NAME
		": this driver, once enabled, cannot be stopped. The\n"
		"  watchdog must be continuously serviced or the system will\n"
		"  reset once the watchdog expires.\n");
}

static void a2f_wdt_set_rate(struct a2f_wdt_dev *wdtdev)
{
	long clkrate;

	clkrate = A2F_WDOG_CLK;

	/* Determine divider and set timeout */
	clkrate = clkrate * wdtdev->heartbeat;
	__raw_writel(clkrate, wdtdev->base + A2F_WDOGLOAD);
	a2f_wdt_feed(wdtdev->base);
}

static void a2f_wdt_enable(struct a2f_wdt_dev *wdtdev)
{
	/* Set clock rate */
	a2f_wdt_set_rate(wdtdev);

	/* Enable (1-way) watchdog */
	__raw_writel(0, wdtdev->base + A2F_WDOGCONTROL);

	/* Initial ping */
	a2f_wdt_feed(wdtdev->base);
}

static ssize_t a2f_wdt_write(struct file *file, const char *data,
	size_t len, loff_t *ppos)
{
	struct a2f_wdt_dev *wdtdev = platform_get_drvdata(a2f_wdt_dev);

	if (len)
		a2f_wdt_feed(wdtdev->base);

	return len;
}

static const struct watchdog_info a2f_ident = {
	.options = WDIOF_CARDRESET | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "A2F Watchdog",
};

static long a2f_wdt_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int ret = -ENOTTY;
	int boot_status, time;
	struct a2f_wdt_dev *wdtdev = platform_get_drvdata(a2f_wdt_dev);

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info *) arg,
			&a2f_ident,  sizeof(a2f_ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, (int *) arg);
		break;

	case WDIOC_GETBOOTSTATUS:
		//boot_status = (int) __raw_readl(wdtdev->base + LPC2K_WDMOD);
		//ret = put_user((boot_status >> 2) & 1, (int *) arg);
		boot_status = (int) ((__raw_readl(A2F_MSS_SR) & A2F_MSS_SR_WDOGTIMEOUTEVENT) >> 1);
		ret = put_user(boot_status, (int *) arg);
		break;

	case WDIOC_KEEPALIVE:
		a2f_wdt_feed(wdtdev->base);
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, (int *) arg);
		if (ret)
			break;

		if (time <= 0 || time > MAX_HEARTBEAT) {
			ret = -EINVAL;
			break;
		}

		wdtdev->heartbeat = time;
		a2f_wdt_enable(wdtdev);
		/* no break */
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(wdtdev->heartbeat, (int *) arg);
		break;

	case WDIOC_SETOPTIONS:
	    /* ignore this command (just suppress the warning during boot process) */
	    ret = 0;
	    break;
	}
	return ret;
}

static int a2f_wdt_open(struct inode *inode, struct file *file)
{
	int ret;
	struct a2f_wdt_dev *wdtdev = platform_get_drvdata(a2f_wdt_dev);

	/* Bit will never clear once set */
	if (test_and_set_bit(1, &wdtdev->num_users))
		return -EBUSY;

	ret = 0; /*success*/ /*clk_enable(wdtdev->clk);*/
	if (ret) {
		clear_bit(1, &wdtdev->num_users);
		return ret;
	}

	a2f_wdt_enable(wdtdev);

	return nonseekable_open(inode, file);
}

static int a2f_wdt_release(struct inode *inode, struct file *file)
{
	a2f_wdt_show_warning();

	return 0;
}

static const struct file_operations a2f_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = a2f_wdt_write,
	.unlocked_ioctl = a2f_wdt_ioctl,
	.open = a2f_wdt_open,
	.release = a2f_wdt_release,
};

static int __devinit a2f_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct a2f_wdt_dev *wdtdev;
	struct resource *mem, *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR MODULE_NAME
			"failed to get memory region resouce\n");
		return -ENOENT;
	}

	mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (mem == NULL) {
		printk(KERN_ERR MODULE_NAME "failed to get memory region\n");
		return -EBUSY;
	}

	/* we don't need the IRQ for this simple driver */

	wdtdev = kzalloc(sizeof(struct a2f_wdt_dev), GFP_KERNEL);
	if (!wdtdev) {
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	wdtdev->dev = &pdev->dev;
	wdtdev->mem = mem;
	wdtdev->res = res;
	platform_set_drvdata(pdev, wdtdev);

	wdtdev->base = ioremap(res->start, resource_size(res));
	if (!wdtdev->base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

#if 0
	/*
	 * Since this peripheral is always clocked, we'll get the clock
	 * here and get the rate on open
	 */
	wdtdev->clk = clk_get(&pdev->dev, NULL);
	if (!wdtdev->clk) {
		ret = -ENOMEM;
		goto err_clk;
	}
#endif

	wdtdev->miscdev.parent = &pdev->dev;
	wdtdev->miscdev.minor = WATCHDOG_MINOR;
	wdtdev->miscdev.name = "watchdog";
	wdtdev->miscdev.fops = &a2f_wdt_fops;

	/* Default reset timeout */
	wdtdev->heartbeat = MAX_HEARTBEAT;

	ret = misc_register(&wdtdev->miscdev);
	if (ret)
		goto err_miscreg;

	a2f_wdt_show_warning();

	a2f_wdt_dev = pdev;

	return 0;
err_miscreg:
#if 0
	clk_put(wdtdev->clk);
err_clk:
#endif
	iounmap(wdtdev->base);
err_ioremap:
	platform_set_drvdata(pdev, NULL);
	kfree(wdtdev);
err_kzalloc:
	release_mem_region(res->start, resource_size(res));

	return ret;
}

static int __devexit a2f_wdt_remove(struct platform_device *pdev)
{
	struct a2f_wdt_dev *wdtdev = platform_get_drvdata(pdev);

	misc_deregister(&wdtdev->miscdev);

	if (test_bit(1, &wdtdev->num_users)) {
		clk_disable(wdtdev->clk);
		a2f_wdt_show_warning();
	}

	clk_put(wdtdev->clk);

	iounmap(wdtdev->base);
	platform_set_drvdata(pdev, NULL);
	kfree(wdtdev);
	release_mem_region(wdtdev->res->start, resource_size(wdtdev->res));

	return 0;
}

static struct platform_driver a2f_wdt_driver = {
	.driver = {
		.name = MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.probe = a2f_wdt_probe,
	.remove = __devexit_p(a2f_wdt_remove),
};

static int __init a2f_wdt_init(void)
{
	return platform_driver_register(&a2f_wdt_driver);
}

static void __exit a2f_wdt_exit(void)
{
	platform_driver_unregister(&a2f_wdt_driver);
}

module_init(a2f_wdt_init);
module_exit(a2f_wdt_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("A2F Watchdog Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:a2f-wdt");
