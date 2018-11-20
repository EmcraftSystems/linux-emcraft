// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for STM32 Independent Watchdog
 *
 * Copyright (C) STMicroelectronics 2017
 * Author: Yannick Fertre <yannick.fertre@st.com> for STMicroelectronics.
 *
 * This driver is based on tegra_wdt.c
 *
 * (C) Copyright 2018
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Skvortsov, Emcraft Systems, <vskvortsov@emcraft.com>
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <mach/clock.h>

/* iwdg registers */
#define IWDG_KR		0x00 /* Key register */
#define IWDG_PR		0x04 /* Prescaler Register */
#define IWDG_RLR	0x08 /* ReLoad Register */
#define IWDG_SR		0x0C /* Status Register */
#define IWDG_WINR	0x10 /* Windows Register */

/* IWDG_KR register bit mask */
#define KR_KEY_RELOAD	0xAAAA /* reload counter enable */
#define KR_KEY_ENABLE	0xCCCC /* peripheral enable */
#define KR_KEY_EWA	0x5555 /* write access enable */
#define KR_KEY_DWA	0x0000 /* write access disable */

/* IWDG_PR register bit values */
#define PR_4		0x00 /* prescaler set to 4 */
#define PR_8		0x01 /* prescaler set to 8 */
#define PR_16		0x02 /* prescaler set to 16 */
#define PR_32		0x03 /* prescaler set to 32 */
#define PR_64		0x04 /* prescaler set to 64 */
#define PR_128		0x05 /* prescaler set to 128 */
#define PR_256		0x06 /* prescaler set to 256 */

/* IWDG_RLR register values */
#define RLR_MIN		0x07C /* min value supported by reload register */
#define RLR_MAX		0xFFF /* max value supported by reload register */

/* IWDG_SR register bit mask */
#define FLAG_PVU	BIT(0) /* Watchdog prescaler value update */
#define FLAG_RVU	BIT(1) /* Watchdog counter reload value update */

/* set timeout to 100000 us */
#define TIMEOUT_US	100000
#define SLEEP_US	1000

#define HAS_PCLK	true

struct stm32_iwdg {
	void __iomem		*regs;
	int			timeout;
	unsigned int		rate;
	unsigned long		users;
};

static struct platform_device *stm32_wdt_dev = NULL;

static inline u32 reg_read(void __iomem *base, u32 reg)
{
	return readl(base + reg);
}

static inline void reg_write(void __iomem *base, u32 reg, u32 val)
{
	writel(val, base + reg);
}

static int stm32_iwdg_start(void)
{
	struct stm32_iwdg *wdt = platform_get_drvdata(stm32_wdt_dev);
	u32 val = FLAG_PVU | FLAG_RVU;
	u32 reload;
	int ret;
	int tmout;

	dev_dbg(&stm32_wdt_dev->dev, "%s\n", __func__);

	/* prescaler fixed to 256 */
	reload = clamp_t(unsigned int, ((wdt->timeout * wdt->rate) / 256) - 1,
			 RLR_MIN, RLR_MAX);

	/* enable write access */
	reg_write(wdt->regs, IWDG_KR, KR_KEY_EWA);

	/* set prescaler & reload registers */
	reg_write(wdt->regs, IWDG_PR, PR_256); /* prescaler fix to 256 */
	reg_write(wdt->regs, IWDG_RLR, reload);
	reg_write(wdt->regs, IWDG_KR, KR_KEY_ENABLE);

	/* wait for the registers to be updated (max 100ms) */
	ret = 0;
	tmout = 10;
	val = readl(wdt->regs + IWDG_SR);
	while ( val & (FLAG_PVU | FLAG_RVU) ){
		if ( ! tmout -- ) {
			ret = -ETIMEDOUT;
			break;
		}
		msleep(10);
		val = readl(wdt->regs + IWDG_SR);
	}

	if (ret) {
		dev_err(&stm32_wdt_dev->dev,
			"Fail to set prescaler or reload registers\n");
		return ret;
	}

	/* reload watchdog */
	reg_write(wdt->regs, IWDG_KR, KR_KEY_RELOAD);

	return 0;
}

static int stm32_iwdg_ping(void)
{
	struct stm32_iwdg *wdt = platform_get_drvdata(stm32_wdt_dev);

	dev_dbg(&stm32_wdt_dev->dev, "%s\n", __func__);

	/* reload watchdog */
	reg_write(wdt->regs, IWDG_KR, KR_KEY_RELOAD);

	return 0;
}

static int stm32_iwdg_set_timeout(unsigned int timeout)
{
	struct stm32_iwdg *wdt = platform_get_drvdata(stm32_wdt_dev);

	dev_dbg(&stm32_wdt_dev->dev, "%s timeout: %d sec\n", __func__, timeout);

	wdt->timeout = timeout;

	return 0;
}

static int stm32_iwdg_get_timeout(void)
{
	struct stm32_iwdg *wdt = platform_get_drvdata(stm32_wdt_dev);

	return wdt->timeout;
}

/*
 * Watchdog device is opened, and watchdog starts running.
 */
static int stm32_iwdg_open(struct inode *inode, struct file *file)
{
	struct stm32_iwdg *wdt = platform_get_drvdata(stm32_wdt_dev);

	if (test_and_set_bit(1, &wdt->users))
		return -EBUSY;

	return 0;
}

/*
 * Close the watchdog device.
 */
static int stm32_iwdg_release(struct inode *inode, struct file *file)
{
	struct stm32_iwdg *wdt = platform_get_drvdata(stm32_wdt_dev);

	clear_bit(1, &wdt->users);
	return 0;
}

static ssize_t stm32_iwdg_write(struct file *file, const char __user *data,
				size_t len, loff_t *ppos)
{
	if (len) {
		stm32_iwdg_ping();
	}

	return 0;
}

static const struct watchdog_info stm32_iwdg_info = {
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_MAGICCLOSE |
			  WDIOF_KEEPALIVEPING,
	.identity	= "STM32 Independent Watchdog",
};

/*
 * Handle commands from user-space.
 */
static long stm32_iwdg_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int ret = -ENOTTY;
	int time;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user(argp, &stm32_iwdg_info,
				   sizeof(&stm32_iwdg_info)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		ret = put_user(0, p);
		break;

	case WDIOC_SETOPTIONS:
		ret = get_user(time, p);
		if (ret)
			break;
		if (time & WDIOS_ENABLECARD) {
			ret = stm32_iwdg_start();
		}
		break;
	case WDIOC_KEEPALIVE:
		ret = stm32_iwdg_ping();
		break;
	case WDIOC_SETTIMEOUT:
		ret = get_user(time, p);
		if (ret)
			break;

		stm32_iwdg_set_timeout(time);

		ret = stm32_iwdg_start();
		break;
	case WDIOC_GETTIMEOUT:
		time = stm32_iwdg_get_timeout();
		ret = put_user(time, p);
		break;

	}

	return ret;

}

static const struct file_operations stm32_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = stm32_iwdg_write,
	.unlocked_ioctl = stm32_iwdg_ioctl,
	.open = stm32_iwdg_open,
	.release = stm32_iwdg_release,
};

static struct miscdevice stm32_iwdg_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &stm32_wdt_fops,
};

static int stm32_iwdg_probe(struct platform_device *pdev)
{
	struct stm32_iwdg *wdt;
	struct resource *res;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	/* This is the timer base. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdt->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdt->regs)) {
		dev_err(&pdev->dev, "Could not get resource\n");
		return PTR_ERR(wdt->regs);
	}

	wdt->rate = stm32_clock_get(CLOCK_LSI);

	stm32_iwdg_miscdev.parent = &pdev->dev;
	ret = misc_register(&stm32_iwdg_miscdev);
	if (ret) {
		dev_err(&pdev->dev,
			"cannot register miscdev on minor=%d (err=%d)\n",
							WATCHDOG_MINOR, ret);
		return ret;
	}

	platform_set_drvdata(pdev, wdt);
	stm32_wdt_dev = pdev;

	return 0;
}

static int stm32_iwdg_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int __maybe_unused stm32_iwdg_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int __maybe_unused stm32_iwdg_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define stm32_iwdg_suspend NULL
#define stm32_iwdg_resume NULL
#endif

static struct platform_driver stm32_iwdg_driver = {
	.probe		= stm32_iwdg_probe,
	.remove		= stm32_iwdg_remove,
	.driver = {
		.name	= "stm32-iwdg",
	},
	.suspend = stm32_iwdg_suspend,
	.resume = stm32_iwdg_resume,
};

static int __init stm32_iwdg_init(void)
{
	int ret;
	ret = platform_driver_register(&stm32_iwdg_driver);
	return ret;
}
module_init(stm32_iwdg_init);

MODULE_AUTHOR("Yannick Fertre <yannick.fertre@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 Independent Watchdog Driver");
MODULE_LICENSE("GPL v2");
