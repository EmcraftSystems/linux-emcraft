/*
 * (C) Copyright 2012-2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
 * Anton Protopopov <antonp@emcraft.com>
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <mach/lpc18xx.h>
#include <mach/platform.h>

/*
 * Bits and bit groups inside the SCU_SFS registers
 */
/* Selects pin function */
#define LPC18XX_IOMUX_CONFIG_FUNC_BITS	0
/* Enable pull-down resistor at pad */
#define LPC18XX_IOMUX_CONFIG_EPD_BIT	3
/* Disable pull-up resistor at pad */
#define LPC18XX_IOMUX_CONFIG_EPUN_BIT	4
/* Select Slew rate */
#define LPC18XX_IOMUX_CONFIG_EHS_BIT	5
/* Input buffer enable */
#define LPC18XX_IOMUX_CONFIG_EZI_BIT	6
/* Input glitch filter */
#define LPC18XX_IOMUX_CONFIG_ZIF_BIT	7

/*
 * Normal drive pins
 */
#define LPC18XX_IOMUX_CONFIG(func,epd,epun,ehs,ezi,zif) \
	((func << LPC18XX_IOMUX_CONFIG_FUNC_BITS) | \
	(epd   << LPC18XX_IOMUX_CONFIG_EPD_BIT) | \
	(epun  << LPC18XX_IOMUX_CONFIG_EPUN_BIT) | \
	(ehs   << LPC18XX_IOMUX_CONFIG_EHS_BIT) | \
	(ezi   << LPC18XX_IOMUX_CONFIG_EZI_BIT) | \
	(zif   << LPC18XX_IOMUX_CONFIG_ZIF_BIT))

/*
 * Pin settings for a clock signal
 */
#define LPC18XX_IOMUX_CONFIG_CLOCK(func)			\
	(LPC18XX_IOMUX_CONFIG(func, 0, 0, 1, 0, 1))

/*
 * Pin settings for an input signal
 */
#define LPC18XX_IOMUX_CONFIG_IN(func)				\
	(LPC18XX_IOMUX_CONFIG(func, 0, 0, 1, 1, 1))

/*
 * Pin settings for an output signal
 */
#define LPC18XX_IOMUX_CONFIG_OUT(func)				\
	(LPC18XX_IOMUX_CONFIG(func, 0, 0, 1, 0, 1))

/*
 * GPIO ports registers map
 */
struct lpc18xx_gpio_regs {
	u8 pbyte[256];		/* GPIO port byte pin registers */
	u32 rsv0[960];
	u32 pword[256];		/* GPIO port word pin registers */
	u32 rsv1[768];
	u32 dir[8];		/* GPIO port direction registers */
	u32 rsv2[24];
	u32 mask[8];		/* GPIO port mask registers */
	u32 rsv3[24];
	u32 pin[8];		/* GPIO port pin registers */
	u32 rsv4[24];
	u32 mpin[8];		/* GPIO masked port pin registers */
	u32 rsv5[24];
	u32 set[8];		/* GPIO port set registers */
	u32 rsv6[24];
	u32 clr[8];		/* GPIO port clear registers */
	u32 rsv7[24];
	u32 not[8];		/* GPIO port toggle registers */
};

/*
 * GPIO registers access handlers
 */
#define LPC18XX_GPIO_BASE	0x400F4000
#define LPC18XX_GPIO		((volatile struct lpc18xx_gpio_regs *) \
					LPC18XX_GPIO_BASE)

/*
 * 16 pin groups. Number of pins in each group is limited to 32.
 */
/* Number of IOMUX pin groups */
#define LPC18XX_IOMUX_GROUPS		16
/* Maximum number of pins in each group */
#define LPC18XX_IOMUX_GROUP_PINS	32

/*
 * Pins CLK0..CLK3 with imaginary numbers 0x18.0-0x18.3
 */
/* Index of the the imaginary group of pins */
#define LPC18XX_IOMUX_CLK_GROUP		24
/* Number of CLK0..CLK3 pins */
#define LPC18XX_IOMUX_CLK_PINS		4

/*
 * System Control Unit (SCU) registers base
 */
#define LPC18XX_SCU_BASE	(LPC18XX_APB0PERIPH_BASE + 0x00006000)
/*
 * Address of the SCU_SFS register for the given pin
 */
#define LPC18XX_PIN_REG_ADDR(group,pin) \
	(LPC18XX_SCU_BASE + (group) * 0x80 + (pin) * 4)
/*
 * Reference to the SCU_SFS register for the given pin
 */
#define LPC18XX_PIN(group,pin) \
	(*(volatile u32 *)LPC18XX_PIN_REG_ADDR(group,pin))

/*
 * I2C0 configuration register
 */
#if defined(CONFIG_LPC18XX_I2C0)
#define LPC18XX_SFSI2C0 ((u32 volatile *) (LPC18XX_SCU_BASE + 0xC84))
#endif

/*
 * Check that the given (pin group, pin) pair is a valid LPC18xx pin.
 * Returns 0 on success, -EINVAL otherwise.
 */
static inline int lpc18xx_validate_pin(int group, int pin)
{
	int rv = 0;

	if (((group >= LPC18XX_IOMUX_GROUPS ||
		pin >= LPC18XX_IOMUX_GROUP_PINS) &&
		(group != LPC18XX_IOMUX_CLK_GROUP ||
		pin >= LPC18XX_IOMUX_CLK_PINS))) {

		printk("IOMUX: incorrect params %d.%d.\n", group, pin);
		rv = -EINVAL;
	}

	return rv;
}

/*
 * Configure the specified MCU pin.
 * Returns 0 on success, -EINVAL otherwise.
 */
static int lpc18xx_pin_config(int group, int pin, u32 regval)
{
	int rv;

	rv = lpc18xx_validate_pin(group, pin);
	if (! rv) {

		/*
		 * Clear the register, in case some bits were set by firmware,
		 * and then write the new value
		 */
		writel(0, &LPC18XX_PIN(group, pin));
		writel(regval, &LPC18XX_PIN(group, pin));
	}

	return rv;
}

/*
 * Set up direction of a GPIO: 1-> out; 0-> in
 */
void lpc18xx_gpio_dir(int group, int pin, int dir)
{
	unsigned int v = LPC18XX_GPIO->dir[group];

	if (dir) {
		writel(v | (1 << pin), &LPC18XX_GPIO->dir[group]);
		LPC18XX_GPIO->dir[group] = v | (1 << pin);
	} else {
		writel(v & ~(1 << pin), &LPC18XX_GPIO->dir[group]);
	}
}
EXPORT_SYMBOL(lpc18xx_gpio_dir);

/*
 * Define the value of a general-purpose output
 */
void lpc18xx_gpio_out(int group, int pin, int c)
{
	unsigned int v;

	if (c) {
		v = readl(&LPC18XX_GPIO->set[group]);
		writel(v | (1 << pin), &LPC18XX_GPIO->set[group]);
	}
	else {
		v = readl(&LPC18XX_GPIO->clr[group]);
		writel(v | (1 << pin), &LPC18XX_GPIO->clr[group]);
	}
}
EXPORT_SYMBOL(lpc18xx_gpio_out);

/*
 * Set up IOMUX configuration of the various processor chips
 */
void __init lpc18xx_iomux_init(void)
{
	int	p = lpc18xx_platform_get();

	if (p == PLATFORM_LPC18XX_HITEX_LPC4350_EVAL) {

#if defined(CONFIG_LPC18XX_SPI0)

		/*
		 * Tie the SPI Flash of Hitex EVAL LPC4350 to SSP0.
		 * Note that CS is defined as a software-driven GPIO.
		 */
		lpc18xx_pin_config(0x3, 3, LPC18XX_IOMUX_CONFIG_CLOCK(2));
		lpc18xx_pin_config(0x3, 6, LPC18XX_IOMUX_CONFIG_IN(5));
		lpc18xx_pin_config(0x3, 7, LPC18XX_IOMUX_CONFIG_OUT(5));
		lpc18xx_pin_config(0x3, 8, LPC18XX_IOMUX_CONFIG_OUT(4));
#endif

#if defined(CONFIG_LPC18XX_I2C0)
		/*
		 * The I2C0 pins are configured using special SFSI2C0 register
		 * (see section 15.4.5 of User manual). SFSI2C0 register
		 * contains eight configuration bits: four for I2C0_SCL, and
		 * four for I2C0_SDA.  To configure I2C0 interface, we should
		 * set up the following values (also see Remark from the
		 * section 15.2 of UM):
		 *    bit | name | val | meaning
		 *    0/8 | EFP  |  0  | Glitch filter time: 0=50ns, 1=3ns
		 *    2/A | EHD  |  0  | I2C mode: 0=Standard/Fast, 1=Fast-mode
		 *    3/B | EZI  |  1  | 1=Enable the input receiver
		 *    7/F | ZIF  |  1  | Glitch filter: 0=Enable, 1=Disable
		 * The above configuration corresponds to the 0x8888 hex value.
		 */
		writel(0x8888, LPC18XX_SFSI2C0);
#endif

#if defined(CONFIG_LPC18XX_I2C1)
		/*
		 * Configure I2C1 pins I2C1_SDA and I2C1_SCL: setup EHS, EZI,
		 * ZIF bits (refer to section 15.4.1 of UM)
		 */
		lpc18xx_pin_config(0xE, 13, LPC18XX_IOMUX_CONFIG(2, 0, 0, 1, 1, 1));
		lpc18xx_pin_config(0xE, 15, LPC18XX_IOMUX_CONFIG(2, 0, 0, 1, 1, 1));
#endif
	}
}

