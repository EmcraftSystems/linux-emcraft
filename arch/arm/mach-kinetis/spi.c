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
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <mach/platform.h>
#include <mach/kinetis.h>
#include <mach/spi.h>
#include <mach/spi-mvf.h>

#define SPI_PLAT_DEVICE(uid)						\
static struct resource kinetis_spi ## uid ##_resources[] = {		\
        {								\
                .start	= KINETIS_SPI ## uid ## _BASE,			\
                .end	= KINETIS_SPI ## uid ## _BASE + SZ_4K - 1,	\
                .flags	= IORESOURCE_MEM,				\
        },								\
	{								\
                .start	= KINETIS_SPI ## uid ## _IRQ,			\
                .flags	= IORESOURCE_IRQ,				\
        },								\
};									\
struct platform_device kinetis_spi ## uid ##_device = {			\
	.name           = "kinetis-dspi",				\
	.id             = uid,						\
	.num_resources  = ARRAY_SIZE(kinetis_spi ## uid ## _resources),	\
	.resource       = kinetis_spi ## uid ## _resources,		\
};									\
static const struct spi_mvf_master kinetis_spi ## uid ## _pdata = {	\
        .bus_num = uid,							\
        .chipselect = NULL,						\
        .num_chipselect = 1,						\
        .cs_control = NULL,						\
}

#ifdef CONFIG_KINETIS_SPI0
#define KINETIS_SPI0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x0002C000)
#define KINETIS_SPI0_IRQ	26
SPI_PLAT_DEVICE(0);
#endif

#ifdef CONFIG_KINETIS_SPI1
#define KINETIS_SPI1_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x0002D000)
#define KINETIS_SPI1_IRQ	27
SPI_PLAT_DEVICE(1);
#endif

#ifdef CONFIG_KINETIS_SPI2
#define KINETIS_SPI2_BASE	(KINETIS_AIPS1PERIPH_BASE + 0x0002C000)
#define KINETIS_SPI2_IRQ	28
SPI_PLAT_DEVICE(2);
#endif

void __init kinetis_spi_init(void)
{
	int p = kinetis_platform_get();

#ifdef CONFIG_KINETIS_SPI0
	platform_device_add_data(&kinetis_spi0_device,
		&kinetis_spi0_pdata, sizeof(kinetis_spi0_pdata));
	platform_device_register(&kinetis_spi0_device);
#endif
#ifdef CONFIG_KINETIS_SPI1
	platform_device_add_data(&kinetis_spi1_device,
		&kinetis_spi1_pdata, sizeof(kinetis_spi1_pdata));
	platform_device_register(&kinetis_spi1_device);
#endif
#ifdef CONFIG_KINETIS_SPI2
	platform_device_add_data(&kinetis_spi2_device,
		&kinetis_spi2_pdata, sizeof(kinetis_spi2_pdata));
	platform_device_register(&kinetis_spi2_device);
#endif

	if (p == PLATFORM_KINETIS_K70_SOM ||
	    p == PLATFORM_KINETIS_K61_SOM) {

		/* This assumes that there is an SPI Flash device
		 * handwired to SPI1 on the breadboard area of TWR-SOM-BSB.
		 * SPI Flash can be accessed either via SPIDEV interface
		 * and this takes precedence if SPIDEV is enabled in the kernel.
		 * If SPIDEV is disabled, then SPI Flash can be
		 * accessed via the Flash MTD interface
		 */
#if defined(CONFIG_KINETIS_SPI1) && \
	(defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV))

		/*
		 * Flash MTD partitioning.
		 * This is used only if CONFIG_SPIDEV is undefined
		 */
#if !defined(CONFIG_SPI_SPIDEV)

		/*
		 * SPI Flash partitioning:
		 */
#		define FLASH_JFFS2_OFFSET__DONGLE	(1024 * 1024 * 2)
#		define FLASH_SIZE__DONGLE		(1024 * 1024 * 4)
		static struct mtd_partition
			spi_kinetis_flash_partitions__dongle[] = {
			{
				.name = "spi_flash_part0",
				.size = FLASH_JFFS2_OFFSET__DONGLE,
				.offset = 0,
			},
			{
				.name = "spi_flash_part1",
				.size = FLASH_SIZE__DONGLE -
					FLASH_JFFS2_OFFSET__DONGLE,
				.offset = FLASH_JFFS2_OFFSET__DONGLE,
			},
		};

		/*
		 * SPI Flash
		 */
		static struct flash_platform_data
			spi_kinetis_flash_data__dongle = {
			.name = "m25p32",
			.parts =  spi_kinetis_flash_partitions__dongle,
			.nr_parts =
			ARRAY_SIZE(spi_kinetis_flash_partitions__dongle),
			.type = "m25p32",
		};
#endif

		/*
		 * SPI slave
		 */
		static struct spi_mvf_chip
			spi_kinetis_flash_slv__dongle = {
			.mode = SPI_MODE_3,
			.bits_per_word = 8,
		};
		static struct spi_board_info
			spi_kinetis_flash_info__dongle = {

			/*
			 * SPIDEV has precedence over Flash MTD
			 */
#if defined(CONFIG_SPI_SPIDEV)
			.modalias = "spidev",
#else
			.modalias = "m25p32",
			.platform_data = &spi_kinetis_flash_data__dongle,
#endif
			.max_speed_hz = 25000000,
			.chip_select = 0,
			.bus_num = 1,
			.controller_data = &spi_kinetis_flash_slv__dongle,
			.mode = SPI_MODE_3,
		};

		/*
		 * Register SPI slaves
		 */
		spi_register_board_info(&spi_kinetis_flash_info__dongle,
			sizeof(spi_kinetis_flash_info__dongle) /
			sizeof(struct spi_board_info));
#endif
	}
}
