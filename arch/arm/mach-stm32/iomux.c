/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * Add SDIO pin configuration for STM3220G-EVAL
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <mach/iomux.h>
#include <mach/platform.h>
#include <mach/stm32.h>

/*
 * GPIO registers bases
 */
#define STM32F2_GPIOA_BASE	(STM32_AHB1PERITH_BASE + 0x0000)
#define STM32F2_GPIOB_BASE	(STM32_AHB1PERITH_BASE + 0x0400)
#define STM32F2_GPIOC_BASE	(STM32_AHB1PERITH_BASE + 0x0800)
#define STM32F2_GPIOD_BASE	(STM32_AHB1PERITH_BASE + 0x0C00)
#define STM32F2_GPIOE_BASE	(STM32_AHB1PERITH_BASE + 0x1000)
#define STM32F2_GPIOF_BASE	(STM32_AHB1PERITH_BASE + 0x1400)
#define STM32F2_GPIOG_BASE	(STM32_AHB1PERITH_BASE + 0x1800)
#define STM32F2_GPIOH_BASE	(STM32_AHB1PERITH_BASE + 0x1C00)
#define STM32F2_GPIOI_BASE	(STM32_AHB1PERITH_BASE + 0x2000)

/*
 * GPIO configuration mode
 */
#define STM32F2_GPIO_MODE_IN	0x00
#define STM32F2_GPIO_MODE_OUT	0x01
#define STM32F2_GPIO_MODE_AF	0x02
#define STM32F2_GPIO_MODE_AN	0x03

/*
 * GPIO output type
 */
#define STM32F2_GPIO_OTYPE_PP	0x00
#define STM32F2_GPIO_OTYPE_OD	0x01

/*
 * GPIO output maximum frequency
 */
#define STM32F2_GPIO_SPEED_2M	0x00
#define STM32F2_GPIO_SPEED_25M	0x01
#define STM32F2_GPIO_SPEED_50M	0x02
#define STM32F2_GPIO_SPEED_100M	0x03

/*
 * GPIO pullup, pulldown configuration
 */
#define STM32F2_GPIO_PUPD_NO	0x00
#define STM32F2_GPIO_PUPD_UP	0x01
#define STM32F2_GPIO_PUPD_DOWN	0x02

/*
 * AF7 selection
 */
#define STM32F2_GPIO_AF_USART1	0x07
#define STM32F2_GPIO_AF_USART2	0x07
#define STM32F2_GPIO_AF_USART3	0x07

/*
 * AF8 selection
 */
#define STM32F2_GPIO_AF_USART4	0x08
#define STM32F2_GPIO_AF_USART5	0x08
#define STM32F2_GPIO_AF_USART6	0x08

/*
 * AF11 selection
 */
#define STM32F2_GPIO_AF_MAC	0x0B

/*
 * AF12 selection
 */
#define STM32F2_GPIO_AF_FSMC	0x0C
#define STM32F2_GPIO_AF_SDIO	0x0C

/*
 * GPIO register map
 */
struct stm32f2_gpio_regs {
	u32	moder;		/* GPIO port mode			      */
	u32	otyper;		/* GPIO port output type		      */
	u32	ospeedr;	/* GPIO port output speed		      */
	u32	pupdr;		/* GPIO port pull-up/pull-down		      */
	u32	idr;		/* GPIO port input data			      */
	u32	odr;		/* GPIO port output data		      */
	u16	bsrrl;		/* GPIO port bit set/reset low		      */
	u16	bsrrh;		/* GPIO port bit set/reset high		      */
	u32	lckr;		/* GPIO port configuration lock		      */
	u32	afr[2];		/* GPIO alternate function		      */
};

/*
 * GPIO roles (alternative functions); role determines by whom GPIO is used
 */
enum stm32f2_gpio_role {
	STM32F2_GPIO_ROLE_USART1,	/* USART1			      */
	STM32F2_GPIO_ROLE_USART2,	/* USART2			      */
	STM32F2_GPIO_ROLE_USART3,	/* USART3			      */
	STM32F2_GPIO_ROLE_USART4,	/* USART4			      */
	STM32F2_GPIO_ROLE_USART5,	/* USART5			      */
	STM32F2_GPIO_ROLE_USART6,	/* USART6			      */
	STM32F2_GPIO_ROLE_ETHERNET,	/* MAC				      */
	STM32F2_GPIO_ROLE_SDIO,		/* SDIO				      */
	STM32F2_GPIO_ROLE_MCO		/* MC external output clock	      */
};

/*
 * GPIO descriptor
 */
struct stm32f2_gpio_dsc {
	u32		port;		/* GPIO port			      */
	u32		pin;		/* GPIO pin			      */
};

/*
 * Register map bases
 */
static const unsigned long io_base[] = {
	STM32F2_GPIOA_BASE, STM32F2_GPIOB_BASE, STM32F2_GPIOC_BASE,
	STM32F2_GPIOD_BASE, STM32F2_GPIOE_BASE, STM32F2_GPIOF_BASE,
	STM32F2_GPIOG_BASE, STM32F2_GPIOH_BASE, STM32F2_GPIOI_BASE
};

/*
 * AF values (note, indexed by enum stm32f2_gpio_role)
 */
static const u32 af_val[] = {
	STM32F2_GPIO_AF_USART1, STM32F2_GPIO_AF_USART2, STM32F2_GPIO_AF_USART3,
	STM32F2_GPIO_AF_USART4, STM32F2_GPIO_AF_USART5, STM32F2_GPIO_AF_USART6,
	STM32F2_GPIO_AF_MAC, STM32F2_GPIO_AF_SDIO,
	0
};

/*
 * Configure the specified GPIO for the specified role
 */
#ifndef CONFIG_ARCH_STM32F1
static int stm32f2_gpio_config(struct stm32f2_gpio_dsc *dsc,
			       enum stm32f2_gpio_role role)
{
	volatile struct stm32f2_gpio_regs	*gpio_regs;

	u32	otype, ospeed, pupd, i;
	int	rv;

	/*
	 * Check params
	 */
	if (!dsc || dsc->port > 8 || dsc->pin > 15) {
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Depending on the role, select the appropriate io params
	 */
	switch (role) {
	case STM32F2_GPIO_ROLE_USART1:
	case STM32F2_GPIO_ROLE_USART2:
	case STM32F2_GPIO_ROLE_USART3:
	case STM32F2_GPIO_ROLE_USART4:
	case STM32F2_GPIO_ROLE_USART5:
	case STM32F2_GPIO_ROLE_USART6:
		otype  = STM32F2_GPIO_OTYPE_PP;
		ospeed = STM32F2_GPIO_SPEED_50M;
		pupd   = STM32F2_GPIO_PUPD_UP;
		break;
	case STM32F2_GPIO_ROLE_ETHERNET:
	case STM32F2_GPIO_ROLE_MCO:
		otype  = STM32F2_GPIO_OTYPE_PP;
		ospeed = STM32F2_GPIO_SPEED_100M;
		pupd   = STM32F2_GPIO_PUPD_NO;
		break;
	case STM32F2_GPIO_ROLE_SDIO:
		otype  = STM32F2_GPIO_OTYPE_PP;
		ospeed = STM32F2_GPIO_SPEED_50M;
		pupd   = STM32F2_GPIO_PUPD_NO;
		break;
	default:
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Get reg base
	 */
	gpio_regs = (struct stm32f2_gpio_regs *)io_base[dsc->port];

	/*
	 * Enable GPIO clocks
	 */
	STM32_RCC->ahb1enr |= 1 << dsc->port;

	if (role != STM32F2_GPIO_ROLE_MCO) {
		/*
		 * Connect PXy to the specified controller (role)
		 */
		i = (dsc->pin & 0x07) * 4;
		gpio_regs->afr[dsc->pin >> 3] &= ~(0xF << i);
		gpio_regs->afr[dsc->pin >> 3] |= af_val[role] << i;
	}

	i = dsc->pin * 2;

	/*
	 * Set Alternative function mode
	 */
	gpio_regs->moder &= ~(0x3 << i);
	gpio_regs->moder |= STM32F2_GPIO_MODE_AF << i;

	/*
	 * Output mode configuration
	 */
	gpio_regs->otyper &= ~(0x3 << i);
	gpio_regs->otyper |= otype << i;

	/*
	 * Speed mode configuration
	 */
	gpio_regs->ospeedr &= ~(0x3 << i);
	gpio_regs->ospeedr |= ospeed << i;

	/*
	 * Pull-up, pull-down resistor configuration
	 */
	gpio_regs->pupdr &= ~(0x3 << i);
	gpio_regs->pupdr |= pupd << i;

	rv = 0;
out:
	return rv;
}
#endif /* !CONFIG_ARCH_STM32F1 */

/*
 * Initialize the GPIO Alternative Functions of the STM32.
 */
void __init stm32_iomux_init(void)
{
	struct stm32f2_gpio_dsc		gpio_dsc;
	int				platform;

	/*
	 * Configure IOs depending on the board we're running on, and
	 * the configuration options we're using.
	 * Let's control platform strictly: if some of it does not need to
	 * play with iomux, it must be present in switch below (otherwise,
	 * the warning message will be printed-out)
	 */
	platform = stm32_platform_get();
	switch (platform) {
#ifndef CONFIG_ARCH_STM32F1
	/* STM32F2-based platforms */
	case PLATFORM_STM32_STM3220G_EVAL:
	case PLATFORM_STM32_STM3240G_EVAL:
#if defined(CONFIG_STM32_USART1)
		gpio_dsc.port = 0;
		gpio_dsc.pin  = 9;
		stm32f2_gpio_config(&gpio_dsc, STM32F2_GPIO_ROLE_USART1);

		gpio_dsc.port = 0;
		gpio_dsc.pin  = 10;
		stm32f2_gpio_config(&gpio_dsc, STM32F2_GPIO_ROLE_USART1);
#endif
#if defined(CONFIG_STM32_USART3)
		gpio_dsc.port = 2;
		gpio_dsc.pin  = 10;
		stm32f2_gpio_config(&gpio_dsc, STM32F2_GPIO_ROLE_USART3);

		gpio_dsc.port = 2;
		gpio_dsc.pin  = 11;
		stm32f2_gpio_config(&gpio_dsc, STM32F2_GPIO_ROLE_USART3);
#endif
#if defined(CONFIG_STM32_MAC)
		do {
			static struct stm32f2_gpio_dsc mac_gpio[] = {
				{0,  1}, {0,  2}, {0,  7},
				{1,  5}, {1,  8},
				{2,  1}, {2,  2}, {2,  3}, {2,  4}, {2,  5},
				{6, 11}, {6, 13}, {6, 14},
				{7,  2}, {7,  3}, {7,  6}, {7,  7},
				{8, 10}
			};
			int	i;

			for (i = 0; i < ARRAY_SIZE(mac_gpio); i++) {
				stm32f2_gpio_config(&mac_gpio[i],
						    STM32F2_GPIO_ROLE_ETHERNET);
			}
		} while (0);
#endif
#if defined(CONFIG_MMC_ARMMMCI) || defined(CONFIG_MMC_ARMMMCI_MODULE)
		do {
			static struct stm32f2_gpio_dsc sdcard_gpio[] = {
				{2,  8}, {2,  9}, {2, 10}, {2, 11},
				{2, 12}, {3,  2}
			};
			int	i;

			for (i = 0; i < ARRAY_SIZE(sdcard_gpio); i++) {
				stm32f2_gpio_config(&sdcard_gpio[i],
						    STM32F2_GPIO_ROLE_SDIO);
			}
		} while (0);
#endif /* defined(CONFIG_MMC_ARMMMCI) || defined(CONFIG_MMC_ARMMMCI_MODULE) */
		break;
#else
	/* STM32F1-based platforms */
	case PLATFORM_STM32_SWISSEMBEDDED_COMM:
		/*
		 * Rely on the IOMUX configuration initialized by the bootloader
		 */
		break;
#endif
	default:
		printk(KERN_WARNING "%s: unsupported platform %d\n", __func__,
			platform);
		/*
		 * Just to avoid compilation warns in case of configuration
		 * which doesn't need iomux, 'use' gpio_dsc var
		 */
		gpio_dsc.port = gpio_dsc.pin = 0;
		break;
	}
}
