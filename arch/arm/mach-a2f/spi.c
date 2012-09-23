/*
 * linux/arch/arm/mach-a2f/spi.c
 *
 * Copyright (C) 2011,2012 Vladimir Khusainov, Emcraft Systems
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
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <mach/a2f.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/spi.h>

/*
 * The MSS subsystem of SmartFusion contains two SPI ports
 */
#define SPI_A2F_REGS_SIZE	0x28

/*
 * MSS SPI_0
 */
#if defined(CONFIG_A2F_MSS_SPI0)

#define SPI_A2F_DEV0_IRQ	12
#define SPI_A2F_DEV0_REGS	0x40001000

static struct resource spi_a2f_dev0_resources[] = {
	{
		.start	= SPI_A2F_DEV0_IRQ,
		.end	= SPI_A2F_DEV0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_A2F_DEV0_REGS,
		.end	= SPI_A2F_DEV0_REGS + SPI_A2F_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_a2f_dev0 = {
	.name           = "spi_a2f",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(spi_a2f_dev0_resources),
	.resource       = spi_a2f_dev0_resources,
};

#endif	/* CONFIG_A2F_MSS_SPI0 */

/*
 * MSS SPI_1
 */
#if defined(CONFIG_A2F_MSS_SPI1)

#define SPI_A2F_DEV1_IRQ	13
#define SPI_A2F_DEV1_REGS	0x40011000

static struct resource spi_a2f_dev1_resources[] = {
	{
		.start	= SPI_A2F_DEV1_IRQ,
		.end	= SPI_A2F_DEV1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= SPI_A2F_DEV1_REGS,
		.end	= SPI_A2F_DEV1_REGS + SPI_A2F_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device spi_a2f_dev1 = {
	.name           = "spi_a2f",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(spi_a2f_dev1_resources),
	.resource       = spi_a2f_dev1_resources,
};

#endif	/* CONFIG_A2F_MSS_SPI1 */

/*
 * Register the A2F specific SPI controllers and devices with the kernel.
 */
void __init a2f_spi_init(void)
{
	int p = a2f_platform_get();

#if defined(CONFIG_A2F_MSS_SPI0)
	/*
 	 * Pass the reference clock to the driver
 	 */
	platform_set_drvdata(&spi_a2f_dev0, 
		(void *) (a2f_clock_get(CLCK_PCLK0)));

	/*
	 * Register a platform device for this interface
	 */
	platform_device_register(&spi_a2f_dev0);		
#endif
#if defined(CONFIG_A2F_MSS_SPI1)
	/*
 	 * Pass the reference clock to the driver
 	 */
	platform_set_drvdata(&spi_a2f_dev1, 
		(void *) (a2f_clock_get(CLCK_PCLK1)));

	/*
	 * Register a platform device for this interface
	 */
	platform_device_register(&spi_a2f_dev1);		
#endif

	/*
	 * Perform board-specific SPI device registration
	 */
	if (p == PLATFORM_A2F_LNX_EVB) {
	}
	else if (p == PLATFORM_A2F_ACTEL_DEV_BRD) {
	}
	else if (p == PLATFORM_A2F_HOERMANN_BRD) {
#if defined(CONFIG_A2F_MSS_SPI1)

#if defined(CONFIG_MTD_M25P80)

		/*
		 * SPI Flash partitioning
		 */
#		define FLASH_JFFS2_OFFSET__A2F_HOERMANN_BRD	(1024*1024*2)
#		define FLASH_SIZE__A2F_HOERMANN_BRD		(1024*1024*8)
		static struct mtd_partition
			spi_a2f_flash_partitions__a2f_hoermann_brd[] = {
			{
				.name = "spi_flash_part0",
				.size = FLASH_JFFS2_OFFSET__A2F_HOERMANN_BRD,
				.offset = 0,
			},
			{
				.name = "spi_flash_part1",
				.size = FLASH_SIZE__A2F_HOERMANN_BRD -
					FLASH_JFFS2_OFFSET__A2F_HOERMANN_BRD,
				.offset = FLASH_JFFS2_OFFSET__A2F_HOERMANN_BRD,
			},
		};

		/*
		 * SPI Flash
		 */
		static struct flash_platform_data
			spi_a2f_flash_data__a2f_hoermann_brd = {
			.name = "at25df641",
			.parts =  spi_a2f_flash_partitions__a2f_hoermann_brd,
			.nr_parts =
		ARRAY_SIZE(spi_a2f_flash_partitions__a2f_hoermann_brd),
			.type = "at25df641",
		};

		/*
		 * SPI slave device
		 */
		static struct spi_board_info
			spi_a2f_eeprom__a2f_hoermann_brd = {
			.modalias = "m25p32",
			.max_speed_hz = 10000000,
			.bus_num = 1,
			.chip_select = 0,
			.platform_data = &spi_a2f_flash_data__a2f_hoermann_brd,
			.mode = SPI_MODE_3,
		};
#endif

#if defined(CONFIG_MTD_M25P80)
		spi_register_board_info(&spi_a2f_eeprom__a2f_hoermann_brd, 1);
#endif

#endif
	}
}
