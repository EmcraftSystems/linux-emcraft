/*
 * (C) Copyright 2011-2013
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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

#ifndef _MACH_STM32_IOMUX_H_
#define _MACH_STM32_IOMUX_H_

#include <linux/init.h>
#include <linux/kernel.h>
#include <mach/stm32.h>

/*
 * GPIO registers base addresses
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
 * STM32F2/F4 GPIO control and status registers
 */
struct stm32f2_gpio_regs {
	u32	moder;		/* GPIO port mode		*/
	u32	otyper;		/* GPIO port output type	*/
	u32	ospeedr;	/* GPIO port output speed	*/
	u32	pupdr;		/* GPIO port pull-up/pull-down	*/
	u32	idr;		/* GPIO port input data		*/
	u32	odr;		/* GPIO port output data	*/
	u16	bsrrl;		/* GPIO port bit set/reset low	*/
	u16	bsrrh;		/* GPIO port bit set/reset high	*/
	u32	lckr;		/* GPIO port configuration lock	*/
	u32	afr[2];		/* GPIO alternate function	*/
};

/*
 * Initialize the GPIO subsystem
 */
void __init stm32_iomux_init(void);

#endif	/*_MACH_STM32_IOMUX_H_ */
