/*
 * (C) Copyright 2012-2015
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
 * Anton Protopopov <antonp@emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
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
#include <mach/clock.h>
#include <mach/gpio.h>

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
 * I2C0 configuration register
 */
#if defined(CONFIG_LPC18XX_I2C0)
#define LPC18XX_SFSI2C0 ((u32 volatile *) (LPC18XX_SCU_BASE + 0xC84))
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
#define LPC18XX_SFSI2C0_CONFIG	0x8888
#endif


#if defined(CONFIG_FB_ARMCLCD)
struct iomux_pin_config {
	int group, pin;
	u32 mask;
};
#endif /* CONFIG_FB_ARMCLCD */

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
int lpc18xx_pin_config(int group, int pin, u32 regval)
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
EXPORT_SYMBOL(lpc18xx_pin_config);

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
		writel(LPC18XX_SFSI2C0_CONFIG, LPC18XX_SFSI2C0);
#endif

#if defined(CONFIG_LPC18XX_I2C1)
		/*
		 * Configure I2C1 pins I2C1_SDA and I2C1_SCL: setup EHS, EZI,
		 * ZIF bits (refer to section 15.4.1 of UM)
		 */
		lpc18xx_pin_config(0xE, 13, LPC18XX_IOMUX_CONFIG(2, 0, 0, 1, 1, 1));
		lpc18xx_pin_config(0xE, 15, LPC18XX_IOMUX_CONFIG(2, 0, 0, 1, 1, 1));
#endif

#if defined(CONFIG_LPC18XX_MMC)
/* LPC18XX_IOMUX_CONFIG(func,epulldown,nepullup,ehighslew,einput,glitchfilter) */
		/* PC_0 - SDIO_CLK */
		lpc18xx_pin_config(0xC, 0, LPC18XX_IOMUX_CONFIG(7, 0, 1, 1, 1, 1));
		/* PC_1 - SDIO_VOLT0 */
		//lpc18xx_pin_config(0xC, 1, LPC18XX_IOMUX_CONFIG(7, 0, 0, 1, 0, 1));
		/* PC_2 - SDIO_RST (?) */
		lpc18xx_pin_config(0xC, 2, LPC18XX_IOMUX_CONFIG(7, 0, 1, 1, 0, 1));
		/* PC_3 - SDIO_VOLT1 */
		//lpc18xx_pin_config(0xC, 3, LPC18XX_IOMUX_CONFIG(7, 0, 0, 1, 0, 1));
		/* PC_4 - SDIO_D0 */
		lpc18xx_pin_config(0xC, 4, LPC18XX_IOMUX_CONFIG(7, 0, 1, 1, 1, 1));
		/* PC_5 - SDIO_D1 */
		lpc18xx_pin_config(0xC, 5, LPC18XX_IOMUX_CONFIG(7, 0, 1, 1, 1, 1));
		/* PC_6 - SDIO_D2 */
		lpc18xx_pin_config(0xC, 6, LPC18XX_IOMUX_CONFIG(7, 0, 1, 1, 1, 1));
		/* PC_7 - SDIO_D3 */
		lpc18xx_pin_config(0xC, 7, LPC18XX_IOMUX_CONFIG(7, 0, 1, 1, 1, 1));
		/* PC_8 - SDIO_CD */
		lpc18xx_pin_config(0xC, 8, LPC18XX_IOMUX_CONFIG(7, 0, 1, 0, 1, 0));
		/* PC_10 - SDIO_CMD */
		lpc18xx_pin_config(0xC, 10, LPC18XX_IOMUX_CONFIG(7, 0, 1, 1, 1, 1));
		/* PD_1 - SDIO_POW */
		lpc18xx_pin_config(0xD, 1, LPC18XX_IOMUX_CONFIG(5, 0, 1, 0, 0, 0));
#endif /* CONFIG_LPC18XX_MMC */
	}

	if (p == PLATFORM_LPC18XX_EA_LPC4357_EVAL) {

#if defined (CONFIG_FB_ARMCLCD) || defined(CONFIG_MTD_M25P80_SPIFI)
		int i;
#endif

#if defined (CONFIG_FB_ARMCLCD)

		static struct iomux_pin_config arm_clcd_iomux[] = {

		/* RED0->4 */
		{0x4, 2, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0x8, 7, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},
		{0x8, 6, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},
		{0x8, 5, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},
		{0x8, 4, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},

		/* GREEN0->5 */
		{0x4, 10, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0x4, 9, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0x8, 3, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},
		{0xB, 6, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0xB, 5, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0xB, 4, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},

		/* BLUE0->4 */
		{0x7, 1, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},
		{0xB, 3, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0xB, 2, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0xB, 1, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		{0xB, 0, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},

		/* LCD_FP */
		{0x4, 5, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		/* LCD_ENAB */
		{0x4, 6, LPC18XX_IOMUX_CONFIG(2, 0, 1, 1, 1, 0)},
		/* LCD_DCLK */
		{0x4, 7, LPC18XX_IOMUX_CONFIG(0, 0, 1, 1, 1, 0)},
		/* LCD_LE */
		{0x7, 0, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},
		/* LCD_LP */
		{0x7, 6, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},
		/* LCD_PWR */
		{0x7, 7, LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 0)},

		};

		for (i = 0; i < ARRAY_SIZE(arm_clcd_iomux); i++) {
			struct iomux_pin_config *p = &arm_clcd_iomux[i];
			lpc18xx_pin_config(p->group, p->pin, p->mask);
		}
#endif /* CONFIG_FB_ARMCLCD */

#if defined(CONFIG_LPC18XX_I2C0)
		writel(LPC18XX_SFSI2C0_CONFIG, LPC18XX_SFSI2C0);
#endif

#if defined(CONFIG_LPC18XX_I2C1)
		/*
		 * Configure I2C1 pins I2C1_SDA and I2C1_SCL: setup EHS, EZI,
		 * ZIF bits (refer to section 15.4.1 of UM)
		 */
		lpc18xx_pin_config(0x2, 3,
				LPC18XX_IOMUX_CONFIG(1, 0, 0, 1, 1, 1));
		lpc18xx_pin_config(0x2, 4,
				LPC18XX_IOMUX_CONFIG(1, 0, 0, 1, 1, 1));
#endif

#if defined(CONFIG_MTD_M25P80_SPIFI)
		/* Setup SPIFI pins */
		for (i = 3; i <= 7; ++i) {
			lpc18xx_pin_config(0x3, i,
				LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 1, 1));
		}
		/* SSEL is output only */
		lpc18xx_pin_config(0x3, 8,
				LPC18XX_IOMUX_CONFIG(3, 0, 1, 1, 0, 0));
#endif
	}
}
