/*
 * linux/arch/arm/mach-stm32/i2c.c
 *
 * Copyright (C) 2013 Vladimir Khusainov, Emcraft Systems
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
#include <linux/i2c.h>
#include <mach/stm32.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/i2c.h>

/* 
 * Size of the I2C controller register area
 */
#define I2C_STM32_REGS_SIZE	0x3FF

/*
 * I2C_1
 */
#if defined(CONFIG_STM32_I2C1)

#define I2C_STM32_DEV1_IRQ	31
#define I2C_STM32_DEV1_REGS	0x40005400

static struct resource i2c_stm32_dev1_resources[] = {
	{
		.start	= I2C_STM32_DEV1_IRQ,
		.end	= I2C_STM32_DEV1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= I2C_STM32_DEV1_REGS,
		.end	= I2C_STM32_DEV1_REGS + I2C_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device i2c_stm32_dev1 = {
	.name           = "i2c_stm32",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(i2c_stm32_dev1_resources),
	.resource       = i2c_stm32_dev1_resources,
};

static struct i2c_stm32_data i2c_stm32_data_dev1 = {
	.i2c_clk	= 100000,
};

#endif	/* CONFIG_STM32_I2C1 */

/*
 * I2C_2
 */
#if defined(CONFIG_STM32_I2C2)

#define I2C_STM32_DEV2_IRQ	33
#define I2C_STM32_DEV2_REGS	0x40005800

static struct resource i2c_stm32_dev2_resources[] = {
	{
		.start	= I2C_STM32_DEV2_IRQ,
		.end	= I2C_STM32_DEV2_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= I2C_STM32_DEV2_REGS,
		.end	= I2C_STM32_DEV2_REGS + I2C_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device i2c_stm32_dev2 = {
	.name           = "i2c_stm32",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(i2c_stm32_dev2_resources),
	.resource       = i2c_stm32_dev2_resources,
};

static struct i2c_stm32_data i2c_stm32_data_dev2 = {
	.i2c_clk	= 100000,
};

#endif	/* CONFIG_STM32_I2C2 */

/*
 * I2C_3
 */
#if defined(CONFIG_STM32_I2C3)

#define I2C_STM32_DEV3_IRQ	72
#define I2C_STM32_DEV3_REGS	0x40005C00

static struct resource i2c_stm32_dev3_resources[] = {
	{
		.start	= I2C_STM32_DEV3_IRQ,
		.end	= I2C_STM32_DEV3_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= I2C_STM32_DEV3_REGS,
		.end	= I2C_STM32_DEV3_REGS + I2C_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device i2c_stm32_dev3 = {
	.name           = "i2c_stm32",
	.id             = 2,
	.num_resources  = ARRAY_SIZE(i2c_stm32_dev3_resources),
	.resource       = i2c_stm32_dev3_resources,
};

static struct i2c_stm32_data i2c_stm32_data_dev3 = {
	.i2c_clk	= 100000,
};

#endif	/* CONFIG_STM32_I2C3 */

/*
 * Register the STM32 specific I2C devices with the kernel.
 */
void __init stm32_i2c_init(void)
{
	int p = stm32_platform_get();

#if defined(CONFIG_STM32_I2C1)
	/*
 	 * Pass the device parameters to the driver
 	 */
	i2c_stm32_data_dev1.ref_clk = stm32_clock_get(CLOCK_PCLK1);
	platform_set_drvdata(&i2c_stm32_dev1, &i2c_stm32_data_dev1);

	/*
	 * Register a platform device for this interface
	 */
	platform_device_register(&i2c_stm32_dev1);		
#endif

#if defined(CONFIG_STM32_I2C2)
	/*
 	 * Pass the device parameters to the driver
 	 */
	i2c_stm32_data_dev2.ref_clk = stm32_clock_get(CLOCK_PCLK1);
	platform_set_drvdata(&i2c_stm32_dev2, &i2c_stm32_data_dev2);

	/*
	 * Register a platform device for this interface
	 */
	platform_device_register(&i2c_stm32_dev2);		
#endif

#if defined(CONFIG_STM32_I2C3)
	/*
 	 * Pass the device parameters to the driver
 	 */
	i2c_stm32_data_dev3.ref_clk = stm32_clock_get(CLOCK_PCLK1);
	platform_set_drvdata(&i2c_stm32_dev3, &i2c_stm32_data_dev3);

	/*
	 * Register a platform device for this interface
	 */
	platform_device_register(&i2c_stm32_dev3);		
#endif

	/*
 	 * Perform board-specific I2C device registration
 	 */
	if (p == PLATFORM_STM32_STM_SOM) {
#if defined(CONFIG_STM32_I2C1)

		/*
		 * This assumes that a compatible I2C EEPROM is
		 * wired to I2C_1 in the baseboard area.
		 */
#if defined(CONFIG_EEPROM_AT24)
		static struct i2c_board_info i2c_eeprom__stm32_som= {
			I2C_BOARD_INFO("24c512", 0x57)
		};
#endif

#if defined(CONFIG_EEPROM_AT24)
		i2c_register_board_info(0, &i2c_eeprom__stm32_som, 1);
#endif

#endif
	}
}

