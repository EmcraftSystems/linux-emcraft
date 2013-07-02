/*
 * linux/arch/arm/mach-m2s/i2c.c
 *
 * Copyright (C) 2012 Vladimir Khusainov, Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <mach/m2s.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/i2c.h>

/*
 * MSS I2C_0
 */
#if defined(CONFIG_M2S_MSS_I2C0)

#define I2C_M2S_DEV0_IRQ	4
#define I2C_M2S_DEV0_REGS	0x40002000
#define I2C0_RST_CLR		(1 << 11)

static struct resource i2c_m2s_dev0_resources[] = {
	{
		.start	= I2C_M2S_DEV0_IRQ,
		.end	= I2C_M2S_DEV0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= I2C_M2S_DEV0_REGS,
		.end	= I2C_M2S_DEV0_REGS + 0xFFF,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device i2c_m2s_dev0 = {
	.name           = "i2c_a2f",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(i2c_m2s_dev0_resources),
	.resource       = i2c_m2s_dev0_resources,
};

static struct i2c_a2f_data i2c_m2s_data_dev0 = {
	.i2c_clk	= 100000,
};

#endif	/* CONFIG_M2S_MSS_I2C0 */

/*
 * MSS I2C_1
 */
#if defined(CONFIG_M2S_MSS_I2C1)

#define I2C_M2S_DEV1_IRQ	7
#define I2C_M2S_DEV1_REGS	0x40012000
#define I2C1_RST_CLR		(1 << 12)

static struct resource i2c_m2s_dev1_resources[] = {
	{
		.start	= I2C_M2S_DEV1_IRQ,
		.end	= I2C_M2S_DEV1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= I2C_M2S_DEV1_REGS,
		.end	= I2C_M2S_DEV1_REGS + 0xFFF,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device i2c_m2s_dev1 = {
	.name           = "i2c_a2f",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(i2c_m2s_dev1_resources),
	.resource       = i2c_m2s_dev1_resources,
};

static struct i2c_a2f_data i2c_m2s_data_dev1 = {
	.i2c_clk	= 400000,
};

#endif	/* CONFIG_M2S_MSS_I2C1 */

/*
 * Register the M2S specific I2C devices with the kernel.
 */
void __init m2s_i2c_init(void)
{
	int p = m2s_platform_get();

#if defined(CONFIG_M2S_MSS_I2C0)

	/*
 	 * Reset the I2C controller and then bring it out of reset
 	 */
	writel(readl(&M2S_SYSREG->soft_reset_cr) | I2C0_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);
	writel(readl(&M2S_SYSREG->soft_reset_cr) & ~I2C0_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);

	/*
 	 * Pass the device parameters to the driver
 	 */
	i2c_m2s_data_dev0.ref_clk = m2s_clock_get(CLCK_PCLK0);
	platform_set_drvdata(&i2c_m2s_dev0, &i2c_m2s_data_dev0);

	/*
	 * Register a platform device for this interface
	 */
	platform_device_register(&i2c_m2s_dev0);
#endif

#if defined(CONFIG_M2S_MSS_I2C1)

	/*
 	 * Reset the I2C controller and then bring it out of reset
 	 */
	writel(readl(&M2S_SYSREG->soft_reset_cr) | I2C1_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);
	writel(readl(&M2S_SYSREG->soft_reset_cr) & ~I2C1_RST_CLR,
		&M2S_SYSREG->soft_reset_cr);

	/*
 	 * Pass the device parameters to the driver
 	 */
	i2c_m2s_data_dev1.ref_clk = m2s_clock_get(CLCK_PCLK1);
	platform_set_drvdata(&i2c_m2s_dev1, &i2c_m2s_data_dev1);

	/*
	 * Register a platform device for this interface
	 */
	platform_device_register(&i2c_m2s_dev1);
#endif

	/*
 	 * Perform board-specific I2C device registration
 	 */
	if (p == PLATFORM_M2S_SOM || p == PLATFORM_M2S_FG484_SOM) {
#if defined(CONFIG_M2S_MSS_I2C1)

		/*
		 * This assumes that a compatible I2C Flash is
		 * wired to I2C_1 in the baseboard area.
		 */
#if defined(CONFIG_EEPROM_AT24)
		static struct i2c_board_info i2c_eeprom__m2s_som= {
			I2C_BOARD_INFO("24c512", 0x51)
		};
#endif

#if defined(CONFIG_EEPROM_AT24)
		i2c_register_board_info(1, &i2c_eeprom__m2s_som, 1);
#endif

#endif
	}
}

