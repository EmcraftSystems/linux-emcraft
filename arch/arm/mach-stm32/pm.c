/*
 * arch/arm/mach-stm32/pm.c
 *
 * STM32 Power Management code
 *
 * Copyright (C) 2016 Emcraft Systems
 * Yuri Tikhonov, Emcraft Systems, <yur@emcraft.com>
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

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/gpio.h>

#include <mach/platform.h>
#include <mach/stm32.h>
#include <mach/irqs.h>
#include <mach/fb.h>
#include <mach/iomux.h>

/*
 * Assembler-level imports (from SRAM)
 */
#define SRAM_TEXT	__attribute__((section (".sram.text"),__long_call__))
SRAM_TEXT void stm32_suspend_to_ram(void);
extern u32 stm32_suspend_moder[STM32_GPIO_PORTS];

/*
 * Wake-up GPIO
 */
#define STM32_WAKEUP_GPIO_PORT	9	/* PJ4/LCD_R5, also see iomux.c */
#define STM32_WAKEUP_GPIO_PIN	4
#define STM32_WAKEUP_GPIO_GPIO	STM32_GPIO_PORTPIN2NUM(STM32_WAKEUP_GPIO_PORT, \
						       STM32_WAKEUP_GPIO_PIN)

/*
 * Wake-up USB
 */
#define STM32_WAKEUP_UHS0_PORT	0	/* PA3/OTG_HS_ULPI_D0 */
#define STM32_WAKEUP_UHS0_PIN	3
#define STM32_WAKEUP_UHS0_GPIO	STM32_GPIO_PORTPIN2NUM(STM32_WAKEUP_UHS0_PORT, \
						       STM32_WAKEUP_UHS0_PIN)

#define STM32_WAKEUP_UHS1_PORT	1	/* PB0/OTG_HS_ULPI_D1 */
#define STM32_WAKEUP_UHS1_PIN	0
#define STM32_WAKEUP_UHS1_GPIO	STM32_GPIO_PORTPIN2NUM(STM32_WAKEUP_UHS1_PORT, \
						       STM32_WAKEUP_UHS1_PIN)

#define STM32_WAKEUP_UFSP_PORT	0	/* PA12/OTG_FS_DP */
#define STM32_WAKEUP_UFSP_PIN	12
#define STM32_WAKEUP_UFSP_GPIO	STM32_GPIO_PORTPIN2NUM(STM32_WAKEUP_UFSP_PORT, \
						       STM32_WAKEUP_UFSP_PIN)

#define STM32_USTP_PORT		2	/* PC0/OTG_HS_ULPI_STP */
#define STM32_USTP_PIN		0
#define STM32_USTP_GPIO		STM32_GPIO_PORTPIN2NUM(STM32_USTP_PORT, \
						       STM32_USTP_PIN)

#define STM32_UDIR_PORT		8	/* PI11/OTG_HS_ULPI_DIR */
#define STM32_UDIR_PIN		11
#define STM32_UDIR_GPIO		STM32_GPIO_PORTPIN2NUM(STM32_UDIR_PORT, \
						       STM32_UDIR_PIN)

/*
 * PHY switch GPIO
 */
#define STM32_PHY_PORT		6	/* PG6 */
#define STM32_PHY_PIN		6
#define STM32_PHY_GPIO		STM32_GPIO_PORTPIN2NUM(STM32_PHY_PORT, \
						       STM32_PHY_PIN)

/*
 * Cortex-M3 System Control Register
 */
#define CM3_SCR_BASE		0xE000ED10
#define CM3_SCR_SLEEPDEEP	(1 << 2)

/*
 * RCC registers bits and masks
 */
#define STM32_RCC_CR_HSE_BIT	16
#define STM32_RCC_CR_PLL_BIT	24
#define STM32_RCC_CR_I2S_BIT	26
#define STM32_RCC_CR_SAI_BIT	28

#define STM32_RCC_CFGR_SW_MSK	0x3

/*
 * PWR registers bits and masks
 */
#define STM32_PWR_CR_LPDS	(1 << 0)
#define STM32_PWR_CR_FPDS	(1 << 9)
#define STM32_PWR_CR_LPUDS	(1 << 10)
#define STM32_PWR_CR_ODEN	(1 << 16)
#define STM32_PWR_CR_ODSWEN	(1 << 17)
#define STM32_PWR_CR_UDEN	(0x3 << 18)

#define STM32_PWR_CSR_ODRDY	(1 << 16)

/*
 * Different data we saved on entering, and restore on exiting
 * from PM
 */
static struct {
	struct {
		u32	cr;
		u32	cfgr;
	} rcc;
	struct {
		u32	cr;
	} pwr;
	struct {
		u32	gcr;
	} ltdc;
} stm32_pm_bck;

/*
 * Device data structure
 */
static struct platform_driver stm32_pm_driver = {
	.driver = {
		.name = "stm32_pm",
	},
};

static irqreturn_t stm32_pm_wakeup_handler(int irq, void *dev)
{
	return IRQ_HANDLED;
}

/*
 * Validate suspend state
 * @state		State being entered
 * @returns		1->valid, 0->invalid
 */
static int stm32_pm_valid(suspend_state_t state)
{
	int ret;

	switch (state) {

	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

/*
 * Prepare system hardware to suspend. Some settings are reset on
 * exiting from Stop, so we save these here, and restore then
 */
static void stm32_pm_prepare_to_suspend(void)
{
	int	platform = stm32_platform_get();
#if defined(STM32_WAKEUP_GPIO_GPIO)
	/*
	 * Specify IRQF_TIMER to avoid disabling this IRQ during
	 * suspend_device_irqs() procedure
	 */
	if (request_irq(NVIC_IRQS + STM32_WAKEUP_GPIO_GPIO,
			stm32_pm_wakeup_handler,
			IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TIMER,
			"Wake-up GPIO", &stm32_pm_driver))
		printk(KERN_ERR "%s: gpio irq request failed\n", __func__);
#endif

#if defined(CONFIG_STM32_USB_OTG_HS_HOST)
	if (request_irq(NVIC_IRQS + STM32_WAKEUP_UHS0_GPIO,
			stm32_pm_wakeup_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TIMER,
			"Wake-up USB HS D0", &stm32_pm_driver))
		printk(KERN_ERR "%s: usb hs d0 irq request failed\n", __func__);

	if (request_irq(NVIC_IRQS + STM32_WAKEUP_UHS1_GPIO,
			stm32_pm_wakeup_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TIMER,
			"Wake-up USB HS D1", &stm32_pm_driver))
		printk(KERN_ERR "%s: usb hs d1 irq request failed\n", __func__);
#endif

#if defined(CONFIG_STM32_USB_OTG_FS_DEVICE)
	if (request_irq(NVIC_IRQS + STM32_WAKEUP_UFSP_GPIO,
			stm32_pm_wakeup_handler,
			IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TIMER,
			"Wake-up USB FS D+", &stm32_pm_driver))
		printk(KERN_ERR "%s: usb fs d+ irq request failed\n", __func__);
#endif

	/*
	 * Save RCC
	 */
	stm32_pm_bck.rcc.cr = STM32_RCC->cr;
	stm32_pm_bck.rcc.cfgr = STM32_RCC->cfgr & STM32_RCC_CFGR_SW_MSK;

	/*
	 * Save over-drive settings
	 */
	stm32_pm_bck.pwr.cr = STM32_PWR->cr;

	/*
	 * Switch off PHY power
	 */
	if (platform == PLATFORM_STM32_STM_STM32F7_SOM)
		gpio_direction_output(STM32_PHY_GPIO, 1);

	/*
	 * Set ULPI STOP to zero to avoid incidental wake-up
	 */
	if (platform == PLATFORM_STM32_STM_STM32F7_SOM ||
	    platform == PLATFORM_STM32_OLYMPUS_STM32F7) {
#if defined(CONFIG_STM32_USB_OTG_HS_HOST)
		gpio_direction_output(STM32_USTP_GPIO, 0);
#endif
	}

	/*
	 * FB driver may be off, so always stop LTDC here to avoid SDRAM access
	 */
	stm32_pm_bck.ltdc.gcr = readl(STM32F4_LTDC_BASE + LTDC_GCR);
	writel(stm32_pm_bck.ltdc.gcr & ~1, STM32F4_LTDC_BASE + LTDC_GCR);
}

/*
 * Prepare system hardware to resume
 */
static void stm32_pm_prepare_to_resume(void)
{
	u32 pll[] = { STM32_RCC_CR_HSE_BIT, STM32_RCC_CR_PLL_BIT,
		      STM32_RCC_CR_I2S_BIT, STM32_RCC_CR_SAI_BIT };
	int platform = stm32_platform_get();
	int i;

	/*
	 * Restore LTDC
	 */
	if (stm32_pm_bck.ltdc.gcr & 1)
		writel(stm32_pm_bck.ltdc.gcr, STM32F4_LTDC_BASE + LTDC_GCR);

	/*
	 * Restore PHY power
	 */
	if (platform == PLATFORM_STM32_STM_STM32F7_SOM)
		gpio_direction_input(STM32_PHY_GPIO);

	if (platform == PLATFORM_STM32_STM_STM32F7_SOM ||
	    platform == PLATFORM_STM32_OLYMPUS_STM32F7) {
#if defined(CONFIG_STM32_USB_OTG_HS_HOST)
		/*
		 * Bring USB HS ULPI PHY out of low-power:
		 * - set STOP high
		 * - wait for DIR low
		 * - set STOP low
		 * Maximum CLKOUT start-up time is several ms (e.g. 3.5ms
		 * in USB3300 PHY).
		 */
		gpio_direction_output(STM32_USTP_GPIO, 1);
		gpio_direction_input(STM32_UDIR_GPIO);
		for (i = 2000; i > 0; i--) {
			if (!gpio_get_value(STM32_UDIR_GPIO))
				break;
			udelay(10);
		}
		if (!i)
			printk(KERN_WARNING "USB HS ULPI recovering timeout\n");
		gpio_set_value(STM32_USTP_GPIO, 0);

		/*
		 * Recover GPIO AFs
		 */
		stm32_iomux_usb_hs_init();
#endif
	}

	/*
	 * Restore over-drive
	 */
	if (stm32_pm_bck.pwr.cr & STM32_PWR_CR_ODSWEN) {
		STM32_PWR->cr |= STM32_PWR_CR_ODEN;
		while (!(STM32_PWR->csr & STM32_PWR_CSR_ODRDY));
		STM32_PWR->cr |= STM32_PWR_CR_ODSWEN;
	}

	/*
	 * Restore RCC PLLs. Assume here that RDY bit is next after the
	 * appropriate ON bit in RCC CR register
	 */
	for (i = 0; i < ARRAY_SIZE(pll); i++) {
		if (!(stm32_pm_bck.rcc.cr & (1 << pll[i])))
			continue;
		STM32_RCC->cr |= 1 << pll[i];
		while (!(STM32_RCC->cr & (1 << (pll[i] + 1))));
	}
	STM32_RCC->cfgr &= ~STM32_RCC_CFGR_SW_MSK;
	STM32_RCC->cfgr |= stm32_pm_bck.rcc.cfgr;
	while ((STM32_RCC->cfgr & STM32_RCC_CFGR_SW_MSK) !=
		stm32_pm_bck.rcc.cfgr);

#if defined(CONFIG_STM32_USB_OTG_FS_DEVICE_DEVICE)
	free_irq(NVIC_IRQS + STM32_WAKEUP_UFSP_GPIO, &stm32_pm_driver);
#endif

#if defined(CONFIG_STM32_USB_OTG_HS_HOST)
	free_irq(NVIC_IRQS + STM32_WAKEUP_UHS1_GPIO, &stm32_pm_driver);
	free_irq(NVIC_IRQS + STM32_WAKEUP_UHS0_GPIO, &stm32_pm_driver);
#endif

#if defined(STM32_WAKEUP_GPIO_GPIO)
	free_irq(NVIC_IRQS + STM32_WAKEUP_GPIO_GPIO, &stm32_pm_driver);
#endif
}

/*
 * Enter suspend
 * @state		State being entered
 * @returns		0->success, <0->error code
 */
static int stm32_pm_enter(suspend_state_t state)
{
	volatile u32 *scr = (void *)CM3_SCR_BASE;

	/*
	 * Prepare the system hardware to suspend
	 */
	stm32_pm_prepare_to_suspend();

	/*
	 * Allow STOP mode. Enter SLEEP DEEP on WFI.
	 */
	*scr |= CM3_SCR_SLEEPDEEP;

	/*
	 * Jump to suspend code in SRAM
	 */
	stm32_suspend_to_ram();

	/*
	 * Switch to Normal mode. Disable SLEEP DEEP on WFI.
	 */
	*scr &= ~CM3_SCR_SLEEPDEEP;

	/*
	 * Prepare the system hardware to resume
	 */
	stm32_pm_prepare_to_resume();

	return 0;
}

/*
 * Power Management operations
 */
static struct platform_suspend_ops stm32_pm_ops = {
	.valid = stm32_pm_valid,
	.enter = stm32_pm_enter,
};

/*
 * Driver init
 * @returns		0->success, <0->error code
 */
static int __init stm32_pm_init(void)
{
	int i, ret;
	int platform = stm32_platform_get();

	/*
	 * Initialize GPIOx_MODER we'll use in suspend: to get the minimal
	 * consumption we'll configure all GPIOs as Analog Inputs
	 */
	for (i = 0; i < STM32_GPIO_PORTS; i++)
		stm32_suspend_moder[i] = 0xFFFFFFFF;

	/*
	 * Exceptions are GPIOs which are used for WakeUp/Phy controls
	 */
#if defined(STM32_WAKEUP_GPIO_GPIO)
	stm32_suspend_moder[STM32_WAKEUP_GPIO_PORT] &=
					~(3 << (STM32_WAKEUP_GPIO_PIN * 2));
#endif

	stm32_suspend_moder[STM32_PHY_PORT] &= ~(3 << (STM32_PHY_PIN * 2));
	stm32_suspend_moder[STM32_PHY_PORT] |= 1 << (STM32_PHY_PIN * 2);

#if defined(CONFIG_STM32_USB_OTG_HS_HOST)
	stm32_suspend_moder[STM32_USTP_PORT] &= ~(3 << (STM32_USTP_PIN *2));
	stm32_suspend_moder[STM32_USTP_PORT] |= 1 << (STM32_USTP_PIN * 2);

	stm32_suspend_moder[STM32_WAKEUP_UHS0_PORT] &=
					~(3 << (STM32_WAKEUP_UHS0_PIN * 2));
	stm32_suspend_moder[STM32_WAKEUP_UHS1_PORT] &=
					~(3 << (STM32_WAKEUP_UHS1_PIN * 2));
#endif

#if defined(CONFIG_STM32_USB_OTG_FS_DEVICE)
	stm32_suspend_moder[STM32_WAKEUP_UFSP_PORT] &=
					~(3 << (STM32_WAKEUP_UFSP_PIN * 2));
#endif

	/*
	 * Request PHY control GPIO
	 */
	if (platform == PLATFORM_STM32_STM_STM32F7_SOM) {
		ret = gpio_request(STM32_PHY_GPIO, "PHY");
		if (ret)
			printk(KERN_ERR "%s: phy gpio req failed\n", __func__);
	}

	if (platform == PLATFORM_STM32_STM_STM32F7_SOM ||
	    platform == PLATFORM_STM32_OLYMPUS_STM32F7) {
		ret = gpio_request(STM32_USTP_GPIO, "USTP");
		if (ret)
			printk(KERN_ERR "%s: ustp gpio req failed\n", __func__);

		ret = gpio_request(STM32_UDIR_GPIO, "UDIR");
		if (ret)
			printk(KERN_ERR "%s: udir gpio req failed\n", __func__);
	}

	/*
	 * Want the lowest consumption in Stop mode
	 */
	STM32_PWR->cr |= STM32_PWR_CR_UDEN | STM32_PWR_CR_LPUDS |
			 STM32_PWR_CR_FPDS | STM32_PWR_CR_LPDS;

	/*
	 * Register the PM driver
	 */
	if (platform_driver_register(&stm32_pm_driver) != 0) {
		printk(KERN_ERR "%s: register failed\n", __func__);
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Register PM operations
	 */
	suspend_set_ops(&stm32_pm_ops);

	/*
	 * Here, means success
	 */
	printk(KERN_INFO "Power Management for STM32\n");
	ret = 0;
out:
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit stm32_pm_cleanup(void)
{
	platform_driver_unregister(&stm32_pm_driver);
}

module_init(stm32_pm_init);
module_exit(stm32_pm_cleanup);

MODULE_AUTHOR("Yuri Tikhonov");
MODULE_DESCRIPTION("STM32 PM driver");
MODULE_LICENSE("GPL");

