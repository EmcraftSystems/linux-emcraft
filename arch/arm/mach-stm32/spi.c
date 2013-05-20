/*
 * linux/arch/arm/mach-stm32/spi.c
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
#include <linux/gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_stm32.h>
#include <linux/spi/flash.h>
#include <mach/stm32.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/spi.h>

/* 
 * Size of the SPI controller register area
 */
#define SPI_STM32_REGS_SIZE	0x3FF

/*
 * SPI_1
 */
#if defined(CONFIG_STM32_SPI1)

#define SPI_STM32_DEV1_IRQ	35
#define SPI_STM32_DEV1_REGS	0x40013000

static struct resource spi_stm32_dev1_resources[] = {
	{
		.start	= SPI_STM32_DEV1_IRQ,
		.end	= SPI_STM32_DEV1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_STM32_DEV1_REGS,
		.end	= SPI_STM32_DEV1_REGS + SPI_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_stm32_dev1 = {
	.name           = "spi_stm32",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(spi_stm32_dev1_resources),
	.resource       = spi_stm32_dev1_resources,
};

#endif

/*
 * SPI_2
 */
#if defined(CONFIG_STM32_SPI2)

#define SPI_STM32_DEV2_IRQ	36
#define SPI_STM32_DEV2_REGS	0x40003800

static struct resource spi_stm32_dev2_resources[] = {
	{
		.start	= SPI_STM32_DEV2_IRQ,
		.end	= SPI_STM32_DEV2_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_STM32_DEV2_REGS,
		.end	= SPI_STM32_DEV2_REGS + SPI_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_stm32_dev2 = {
	.name           = "spi_stm32",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(spi_stm32_dev2_resources),
	.resource       = spi_stm32_dev2_resources,
};

#endif	/* CONFIG_STM32_SPI2 */

/*
 * SPI_3
 */
#if defined(CONFIG_STM32_SPI3)

#define SPI_STM32_DEV3_IRQ	51
#define SPI_STM32_DEV3_REGS	0x40003C00

static struct resource spi_stm32_dev3_resources[] = {
	{
		.start	= SPI_STM32_DEV3_IRQ,
		.end	= SPI_STM32_DEV3_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_STM32_DEV3_REGS,
		.end	= SPI_STM32_DEV3_REGS + SPI_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_stm32_dev3 = {
	.name           = "spi_stm32",
	.id             = 2,
	.num_resources  = ARRAY_SIZE(spi_stm32_dev3_resources),
	.resource       = spi_stm32_dev3_resources,
};

#endif

/*
 * SPI_4
 */
#if defined(CONFIG_STM32_SPI4)

#define SPI_STM32_DEV4_IRQ	84
#define SPI_STM32_DEV4_REGS	0x40013400

static struct resource spi_stm32_dev4_resources[] = {
	{
		.start	= SPI_STM32_DEV4_IRQ,
		.end	= SPI_STM32_DEV4_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_STM32_DEV4_REGS,
		.end	= SPI_STM32_DEV4_REGS + SPI_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_stm32_dev4 = {
	.name           = "spi_stm32",
	.id             = 3,
	.num_resources  = ARRAY_SIZE(spi_stm32_dev4_resources),
	.resource       = spi_stm32_dev4_resources,
};

#endif

/*
 * SPI_5
 */
#if defined(CONFIG_STM32_SPI5)

#define SPI_STM32_DEV5_IRQ	85
#define SPI_STM32_DEV5_REGS	0x40015000

static struct resource spi_stm32_dev5_resources[] = {
	{
		.start	= SPI_STM32_DEV5_IRQ,
		.end	= SPI_STM32_DEV5_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_STM32_DEV5_REGS,
		.end	= SPI_STM32_DEV5_REGS + SPI_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_stm32_dev5 = {
	.name           = "spi_stm32",
	.id             = 4,
	.num_resources  = ARRAY_SIZE(spi_stm32_dev5_resources),
	.resource       = spi_stm32_dev5_resources,
};

#endif	/* CONFIG_STM32_SPI5 */

/*
 * SPI_6
 */
#if defined(CONFIG_STM32_SPI6)

#define SPI_STM32_DEV6_IRQ	86
#define SPI_STM32_DEV6_REGS	0x40015400

static struct resource spi_stm32_dev6_resources[] = {
	{
		.start	= SPI_STM32_DEV6_IRQ,
		.end	= SPI_STM32_DEV6_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_STM32_DEV6_REGS,
		.end	= SPI_STM32_DEV6_REGS + SPI_STM32_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_stm32_dev6 = {
	.name           = "spi_stm32",
	.id             = 5,
	.num_resources  = ARRAY_SIZE(spi_stm32_dev6_resources),
	.resource       = spi_stm32_dev6_resources,
};

#endif

/*
 * Register the STM32 specific SPI controllers and devices with the kernel.
 */
void __init stm32_spi_init(void)
{
	int	p = stm32_platform_get();

#if defined(CONFIG_STM32_SPI1)
	platform_set_drvdata(&spi_stm32_dev1, 
		(void *) stm32_clock_get(CLOCK_PCLK2));
	platform_device_register(&spi_stm32_dev1);
#endif
#if defined(CONFIG_STM32_SPI2)
	platform_set_drvdata(&spi_stm32_dev2, 
		(void *) stm32_clock_get(CLOCK_PCLK1));
	platform_device_register(&spi_stm32_dev2);
#endif
#if defined(CONFIG_STM32_SPI3)
	platform_set_drvdata(&spi_stm32_dev3, 
		(void *) stm32_clock_get(CLOCK_PCLK1));
	platform_device_register(&spi_stm32_dev3);
#endif
#if defined(CONFIG_STM32_SPI4)
	platform_set_drvdata(&spi_stm32_dev4, 
		(void *) stm32_clock_get(CLOCK_PCLK2));
	platform_device_register(&spi_stm32_dev4);
#endif
#if defined(CONFIG_STM32_SPI5)
	platform_set_drvdata(&spi_stm32_dev5, 
		(void *) stm32_clock_get(CLOCK_PCLK2));
	platform_device_register(&spi_stm32_dev5);
#endif
#if defined(CONFIG_STM32_SPI6)
	platform_set_drvdata(&spi_stm32_dev6, 
		(void *) stm32_clock_get(CLOCK_PCLK2));
	platform_device_register(&spi_stm32_dev6);
#endif

	if (p == PLATFORM_STM32_STM_SOM) {

		/* This assumes that there is an SPI Flash device
		 * handwired to SPI5 on the breadboard area of SOM-BSB-EXT.
		 * SPI Flash can be accessed either via SPIDEV interface
		 * and this takes precedence if SPIDEV is enabled in the kernel.
		 * If SPIDEV is disabled, then SPI Flash can be
		 * accessed via the Flash MTD interface
		 */
#if defined(CONFIG_STM32_SPI5) && \
	(defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV))

		/* 
		 * Flash MTD partitioning.
		 * This is used only if CONFIG_SPIDEV is undefined
		 */
#if !defined(CONFIG_SPI_SPIDEV)

		/*
		 * SPI Flash partitioning:
		 */
#		define FLASH_JFFS2_OFFSET__STM32F4_SOM	(1024 * 1024 * 1)
#		define FLASH_SIZE__STM32F4_SOM		(1024 * 1024 * 4)
		static struct mtd_partition 
			spi_stm32_flash_partitions__stm32f4_som[] = {
			{
				.name = "spi_flash_part0",
				.size = FLASH_JFFS2_OFFSET__STM32F4_SOM,
				.offset = 0,
			},
			{
				.name = "spi_flash_part1",
				.size = FLASH_SIZE__STM32F4_SOM - 
					FLASH_JFFS2_OFFSET__STM32F4_SOM,
				.offset = FLASH_JFFS2_OFFSET__STM32F4_SOM,
			},
		};

		/*
		 * SPI Flash
 		 */
		static struct flash_platform_data 
			spi_stm32_flash_data__stm32f4_som = {
			.name = "m25p32",
			.parts =  spi_stm32_flash_partitions__stm32f4_som,
			.nr_parts = 
			ARRAY_SIZE(spi_stm32_flash_partitions__stm32f4_som),
			.type = "m25p32",
		};
#endif

#define SPI_FLASH_CS_PORT__STM32F4_SOM		7
#define SPI_FLASH_CS_PIN__STM32F4_SOM		5
#define SPI_FLASH_CS_GPIO__STM32F4_SOM		\
	STM32_GPIO_PORTPIN2NUM(			\
		SPI_FLASH_CS_PORT__STM32F4_SOM, \
		SPI_FLASH_CS_PIN__STM32F4_SOM)

		/*
 		 * SPI slave
 		 */
		static struct spi_stm32_slv 
			spi_stm32_flash_slv__stm32f4_som  = {
			.cs_gpio = SPI_FLASH_CS_GPIO__STM32F4_SOM,
			.timeout = 3,
		};
		static struct spi_board_info 
			spi_stm32_flash_info__stm32f4_som = {
			/*
			 * SPIDEV has precedence over Flash MTD
			 */
#if defined(CONFIG_SPI_SPIDEV)
			.modalias = "spidev",
#else
			.modalias = "m25p32",
			.platform_data = &spi_stm32_flash_data__stm32f4_som,
#endif
			.max_speed_hz = 25000000,
			.bus_num = 4,
			.chip_select = 0,
			.controller_data = &spi_stm32_flash_slv__stm32f4_som,
			.mode = SPI_MODE_3,
		};

		/*
		 * Set up the Chip Select GPIO for the SPI Flash
		 */
		gpio_direction_output(SPI_FLASH_CS_GPIO__STM32F4_SOM, 1);

		/*
		 * Register SPI slaves
		 */
		spi_register_board_info(&spi_stm32_flash_info__stm32f4_som,
			sizeof(spi_stm32_flash_info__stm32f4_som) / 
			sizeof(struct spi_board_info));
#endif
	}
}
