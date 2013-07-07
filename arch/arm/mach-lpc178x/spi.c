/*
 * linux/arch/arm/mach-lpc178x/spi.c
 *
 * Copyright (C) 2013 Vladimir Khusainov, Emcraft Systems
 * Copyright (C) 2013 Andreas Haas, ah114088@gmx.de
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
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>
#include <mach/lpc178x.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/iomux.h>
#include <mach/spi.h>

/* 
 * Size of the SSP/SPI controller register area 
 */
#define SPI_LPC178X_REGS_SIZE	0xFFF

/*
 * SSP/SPI_0
 */
#if defined(CONFIG_LPC178X_SPI0)

#define SPI_LPC178X_DEV0_IRQ	14
#define SPI_LPC178X_DEV0_REGS	0x40088000

static struct pl022_ssp_controller lpc178x_spi0_data = {
	.bus_id                 = 0,
	.num_chipselect         = 8,
	.enable_dma             = 0,
};

static struct amba_device lpc178x_spi0_dev = {
	.dev                            = {
 		.coherent_dma_mask      = ~0,
		.init_name              = "dev:ssp0",
		.platform_data          = &lpc178x_spi0_data,
	},
	.res                            = {
 		.start                  = SPI_LPC178X_DEV0_REGS,
		.end                    = SPI_LPC178X_DEV0_REGS + 
						SPI_LPC178X_REGS_SIZE,
		.flags                  = IORESOURCE_MEM,
 	},
	.dma_mask                       = ~0,
	.irq                            = {SPI_LPC178X_DEV0_IRQ, NO_IRQ},
};

#endif

#if defined(CONFIG_LPC178X_SPI0) && defined(CONFIG_MMC_SPI)

/*
 * Chip Select control for the SPI Flash on SPI0 of LPC-LNX eval board
 */
#define SPI_FLASH_CS_GRP__LPC178X_EVAL	0
#define SPI_FLASH_CS_PIN__LPC178X_EVAL	16

static void spi_lpc178x_flash_cs__lpc178x_eval(u32 control)
{
	lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL, 
				SPI_FLASH_CS_PIN__LPC178X_EVAL, control); 
}

#endif

/*
 * Register the LPC178X specific SPI controllers and devices with the kernel.
 */
void __init lpc178x_spi_init(void)
{
	int p = lpc178x_platform_get();

#if defined(CONFIG_LPC178X_SPI0)
	amba_device_register(&lpc178x_spi0_dev, &iomem_resource);
#endif

	if (p == PLATFORM_LPC178X_LNX_EVB) {

		/* This assumes that the code is running on
		 * the Emcraft LPC-LNX-EVB board, which
		 * has an SD Card hand-wired on SSP/SPI0.
		 */
#if defined(CONFIG_LPC178X_SPI0) && defined(CONFIG_MMC_SPI)

		/*
 		 * SPI slave
 		 */
		static struct pl022_config_chip spi0_slave = {
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
			.cs_control = spi_lpc178x_flash_cs__lpc178x_eval,
		};

		static struct mmc_spi_platform_data mmc_pdata = {
				.detect_delay = 100,
				.powerup_msecs = 100,
				.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
		}; 

		static struct spi_board_info spi0_board_info = {
			.modalias = "mmc_spi",
			.max_speed_hz = 10000000,
			.bus_num = 0,
			.chip_select = 0,
			.controller_data = &spi0_slave,
			.mode = SPI_MODE_0,
			.platform_data = &mmc_pdata, 
		};

		/*
		 * Set up the Chip Select GPIO for the SPI Flash
		 */
		lpc178x_gpio_dir(SPI_FLASH_CS_GRP__LPC178X_EVAL, 
			SPI_FLASH_CS_PIN__LPC178X_EVAL, 1);
		lpc178x_gpio_out(SPI_FLASH_CS_GRP__LPC178X_EVAL, 
				SPI_FLASH_CS_PIN__LPC178X_EVAL, 1);

		/*
		 * Register SPI slaves
		 */
		spi_register_board_info(&spi0_board_info,
			sizeof(spi0_board_info) / 
			sizeof(struct spi_board_info));
#endif
	}
}
