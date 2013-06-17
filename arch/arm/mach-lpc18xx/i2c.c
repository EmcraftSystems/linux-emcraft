/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
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
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <mach/i2c.h>
#include <mach/platform.h>

/*
 * I2C interfaces' registers bases
 */
#define LPC18XX_I2C0_BASE	(0x400A1000)
#define LPC18XX_I2C1_BASE	(0x400E0000)

#define LPC18XX_I2C0_IRQ	18
#define LPC18XX_I2C1_IRQ	19

/*
 * I2C platform devices and resources they use
 */
#define I2C_PLAT_DEVICE(uid)						\
static struct resource lpc18xx_i2c## uid ##_resources[] = {		\
        {								\
                .start	= LPC18XX_I2C## uid ##_BASE,			\
                .end	= LPC18XX_I2C## uid ##_BASE + SZ_4K - 1,	\
                .flags	= IORESOURCE_MEM,				\
        },								\
	{								\
                .start	= LPC18XX_I2C## uid ##_IRQ,			\
                .flags	= IORESOURCE_IRQ,				\
        },								\
};									\
struct platform_device lpc18xx_i2c## uid ##_device = {			\
	.name           = "lpc2k-i2c",					\
	.id             = uid,						\
	.num_resources  = ARRAY_SIZE(lpc18xx_i2c## uid ##_resources),	\
	.resource       = lpc18xx_i2c## uid ##_resources,		\
}

/*
 * Declare 3 platform devices
 */
I2C_PLAT_DEVICE(0);
I2C_PLAT_DEVICE(1);

/*
 * Declare the eeprom device supported by the EEPROM_AT24 driver
 */
#if defined(CONFIG_EEPROM_AT24)
static struct i2c_board_info i2c0_eeprom = { I2C_BOARD_INFO("24c02", 0x50) };
#endif

void __init lpc18xx_i2c_init(void)
{
	/*
	 * Register platform devices
	 */
	platform_device_register(&lpc18xx_i2c0_device);
	platform_device_register(&lpc18xx_i2c1_device);

#if defined(CONFIG_EEPROM_AT24)
	i2c_register_board_info(0, &i2c0_eeprom, 1);
#endif
}
