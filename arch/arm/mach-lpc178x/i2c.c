/*
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <mach/i2c.h>
#include <mach/power.h>
#include <mach/platform.h>

/*
 * I2C0 interface register and "Interrupt ID"
 */
#if defined(CONFIG_LPC178X_I2C0)
#define LPC178X_I2C0_BASE	(LPC178X_APB_PERIPH_BASE + 0x0001C000)
#define LPC178X_I2C0_IRQ	10
#endif

/*
 * I2C1 interface register and "Interrupt ID"
 */
#if defined(CONFIG_LPC178X_I2C1)
#define LPC178X_I2C1_BASE	(LPC178X_APB_PERIPH_BASE + 0x0005C000)
#define LPC178X_I2C1_IRQ	11
#endif

/*
 * I2C2 interface register and "Interrupt ID"
 */
#if defined(CONFIG_LPC178X_I2C2)
#define LPC178X_I2C2_BASE	(LPC178X_APB_PERIPH_BASE + 0x000A0000)
#define LPC178X_I2C2_IRQ	12
#endif

/*
 * I2C platform devices and resources they use
 */
#define I2C_PLAT_DEVICE(uid)						\
static struct resource lpc178x_i2c## uid ##_resources[] = {		\
        {								\
                .start	= LPC178X_I2C## uid ##_BASE,			\
                .end	= LPC178X_I2C## uid ##_BASE + SZ_4K - 1,	\
                .flags	= IORESOURCE_MEM,				\
        },								\
	{								\
                .start	= LPC178X_I2C## uid ##_IRQ,			\
                .flags	= IORESOURCE_IRQ,				\
        },								\
};									\
struct platform_device lpc178x_i2c## uid ##_device = {			\
	.name           = "lpc2k-i2c",					\
	.id             = uid,						\
	.num_resources  = ARRAY_SIZE(lpc178x_i2c## uid ##_resources),	\
	.resource       = lpc178x_i2c## uid ##_resources,		\
}

#if defined(CONFIG_LPC178X_I2C0)
I2C_PLAT_DEVICE(0);
#endif

#if defined(CONFIG_LPC178X_I2C1)
I2C_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_LPC178X_I2C2)
I2C_PLAT_DEVICE(2);
#endif

void __init lpc178x_i2c_init(void)
{
	/*
	 * Register platform devices
	 */

#if defined(CONFIG_LPC178X_I2C0)
	platform_device_register(&lpc178x_i2c0_device);
#endif

#if defined(CONFIG_LPC178X_I2C1)
	platform_device_register(&lpc178x_i2c1_device);
#endif

#if defined(CONFIG_LPC178X_I2C2)
	platform_device_register(&lpc178x_i2c2_device);
#endif
}
