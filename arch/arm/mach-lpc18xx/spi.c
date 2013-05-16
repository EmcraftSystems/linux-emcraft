/*
 * linux/arch/arm/mach-lpc18xx/spi.c
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
#include <linux/irq.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl022.h>
#include <linux/spi/flash.h>
#include <mach/lpc18xx.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/iomux.h>
#include <mach/spi.h>

/* 
 * Size of the SSP/SPI controller register area
 */
#define SPI_LPC18XX_REGS_SIZE	0xFFF

/*
 * SSP/SPI_0
 */
#if defined(CONFIG_LPC18XX_SPI0)

#define SPI_LPC18XX_DEV0_IRQ	22
#define SPI_LPC18XX_DEV0_REGS	0x40083000

static struct pl022_ssp_controller lpc18xx_spi0_data = {
	.bus_id                 = 0,
	.num_chipselect         = 8,
	.enable_dma             = 0,
};

static struct amba_device lpc18xx_spi0_dev = {
	.dev                            = {
 		.coherent_dma_mask      = ~0,
		.init_name              = "dev:ssp0",
		.platform_data          = &lpc18xx_spi0_data,
	},
	.res                            = {
 		.start                  = SPI_LPC18XX_DEV0_REGS,
		.end                    = SPI_LPC18XX_DEV0_REGS + 
						SPI_LPC18XX_REGS_SIZE,
		.flags                  = IORESOURCE_MEM,
 	},
	.dma_mask                       = ~0,
	.irq                            = {SPI_LPC18XX_DEV0_IRQ, NO_IRQ},
};

#endif

/*
 * SSP/SPI_1
 */
#if defined(CONFIG_LPC18XX_SPI1)

#define SPI_LPC18XX_DEV1_IRQ	23
#define SPI_LPC18XX_DEV1_REGS	0x400C5000

static struct pl022_ssp_controller lpc18xx_spi1_data = {
	.bus_id                 = 1,
	.num_chipselect         = 8,
	.enable_dma             = 0,
};

static struct amba_device lpc18xx_spi1_dev = {
	.dev                            = {
 		.coherent_dma_mask      = ~0,
		.init_name              = "dev:ssp1",
		.platform_data          = &lpc18xx_spi1_data,
	},
	.res                            = {
 		.start                  = SPI_LPC18XX_DEV1_REGS,
		.end                    = SPI_LPC18XX_DEV1_REGS + 
						SPI_LPC18XX_REGS_SIZE,
		.flags                  = IORESOURCE_MEM,
 	},
	.dma_mask                       = ~0,
	.irq                            = {SPI_LPC18XX_DEV1_IRQ, NO_IRQ},
};

#endif

#if defined(CONFIG_LPC18XX_SPI0) && \
	(defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV))

#define SPI_FLASH_CS_GRP__LPC18XX_EVAL	5
#define SPI_FLASH_CS_PIN__LPC18XX_EVAL	11

/*
 * Chip Select control for the SPI Flash on SPI0 of Hitex LPC4350 eval board
 */
static void spi_lpc18xx_flash_cs__lpc18xx_eval(u32 control)
{
	lpc18xx_gpio_out(SPI_FLASH_CS_GRP__LPC18XX_EVAL, 
				SPI_FLASH_CS_PIN__LPC18XX_EVAL, control);
}

#endif

/*
 * Register the LPC18XX specific SPI controllers and devices with the kernel.
 */
void __init lpc18xx_spi_init(void)
{
	int	p = lpc18xx_platform_get();

#if defined(CONFIG_LPC18XX_SPI0)
	amba_device_register(&lpc18xx_spi0_dev, &iomem_resource);
#endif

#if defined(CONFIG_LPC18XX_SPI1)
	amba_device_register(&lpc18xx_spi1_dev, &iomem_resource);
#endif

	if (p == PLATFORM_LPC18XX_HITEX_LPC4350_EVAL) {

		/* This assumes that the code is running on
		 * the Hitex LPC4350 eval board, which
		 * has an SPI Flash on SSP/SPI0.
		 * SPI Flash can be accessed either via the SPIDEV interface
		 * and this takes precedence if SPIDEV is enabled in the kernel.
		 * If SPIDEV is disabled, then SPI Flash can be
		 * accessed via the Flash MTD interface.
		 */
#if defined(CONFIG_LPC18XX_SPI0) && \
	(defined(CONFIG_MTD_M25P80) || defined(CONFIG_SPI_SPIDEV))

		/* 
		 * Flash MTD partitioning.
		 * This is used only if CONFIG_SPIDEV is undefined
		 */
#if !defined(CONFIG_SPI_SPIDEV)

		/*
		 * SPI Flash partitioning:
		 */
#		define FLASH_IMAGE_OFFSET__LPC18XX_EVAL	0x40000
#		define FLASH_JFFS2_OFFSET__LPC18XX_EVAL	(6*1024*1024)
		static struct mtd_partition 
			spi_lpc18xx_flash_partitions__lpc18xx_eval[] = {
			{
				.name	= "flash_uboot",
				.offset = 0,
				.size	= FLASH_IMAGE_OFFSET__LPC18XX_EVAL,
			},
			{
				.name	= "flash_linux_image",
				.offset = FLASH_IMAGE_OFFSET__LPC18XX_EVAL,
				.size	= (FLASH_JFFS2_OFFSET__LPC18XX_EVAL
					 - FLASH_IMAGE_OFFSET__LPC18XX_EVAL),
			},
			{
				.name	= "flash_jffs2",
				.offset = FLASH_JFFS2_OFFSET__LPC18XX_EVAL,
			},
		};

		/*
		 * SPI Flash
 		 */
		static struct flash_platform_data 
			spi_lpc18xx_flash_data__lpc18xx_eval = {
			.name = "s25sl064a",
			.parts =  spi_lpc18xx_flash_partitions__lpc18xx_eval,
			.nr_parts = 
			ARRAY_SIZE(spi_lpc18xx_flash_partitions__lpc18xx_eval),
			.type = "s25sl064a",
		};
#endif

		/*
 		 * SPI slave
 		 */
		static struct pl022_config_chip 
			spi_lpc18xx_flash_slv__lpc18xx_eval = {
 			.com_mode = INTERRUPT_TRANSFER,
			.iface = SSP_INTERFACE_MOTOROLA_SPI,
			.hierarchy = SSP_MASTER,
			.slave_tx_disable = 0,
			.rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
			.tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
			.ctrl_len = SSP_BITS_8,
			.data_size = SSP_DATA_BITS_8,
			.wait_state = SSP_MWIRE_WAIT_ZERO,
			.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
			.cs_control = spi_lpc18xx_flash_cs__lpc18xx_eval,
		};

		static struct spi_board_info 
			spi_lpc18xx_flash_info__lpc18xx_eval = {
			/*
			 * SPIDEV has precedence over Flash MTD
			 */
#if defined(CONFIG_SPI_SPIDEV)
			.modalias = "spidev",
#else
			.modalias = "m25p32",
			.platform_data = &spi_lpc18xx_flash_data__lpc18xx_eval,
#endif
			.max_speed_hz = 25000000,
			.bus_num = 0,
			.chip_select = 0,
			.controller_data = &spi_lpc18xx_flash_slv__lpc18xx_eval,
			.mode = SPI_MODE_0,
		};

		/*
 		 * Set up the Chip Select GPIO for the SPI Flash
 		 */
		lpc18xx_gpio_dir(SPI_FLASH_CS_GRP__LPC18XX_EVAL, 
				SPI_FLASH_CS_PIN__LPC18XX_EVAL, 1);
		lpc18xx_gpio_out(SPI_FLASH_CS_GRP__LPC18XX_EVAL, 
				SPI_FLASH_CS_PIN__LPC18XX_EVAL, 1);

		/*
		 * Register SPI slaves
		 */
		spi_register_board_info(&spi_lpc18xx_flash_info__lpc18xx_eval,
			sizeof(spi_lpc18xx_flash_info__lpc18xx_eval) / 
			sizeof(struct spi_board_info));
#endif
	}
}
