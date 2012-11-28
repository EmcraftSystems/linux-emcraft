/*
 * linux/arch/arm/mach-m2s/spi.c
 *
 * Copyright (C) 2012 Yuri Tikhonov, Emcraft Systems
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
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <mach/m2s.h>
#include <mach/platform.h>
#include <mach/clock.h>
#include <mach/spi.h>

/*
 * MSS PDMA is common for both SPI ports. So, actually, both SPI ports
 * at once couldn't work (PDMA resource will be busy). Should implement
 * a separate PDMA driver to fix this.
 */
#define PDMA_M2S_REGS		0x40003000
#define PDMA_M2S_REGS_SIZE	0x120

/*
 * The MSS subsystem of SmartFusion contains two SPI ports
 */
#define SPI_M2S_REGS_SIZE	0x50

#if defined(CONFIG_M2S_MSS_SPI0)
# define SPI_M2S_ID		0
# define SPI_M2S_REGS		0x40001000
# define SPI_M2S_CLK		CLCK_PCLK0
# define SPI_M2S_RX_DMA		0
# define SPI_M2S_TX_DMA		1
#elif defined(CONFIG_M2S_MSS_SPI1)
# define SPI_M2S_ID		1
# define SPI_M2S_REGS		0x40011000
# define SPI_M2S_CLK		CLCK_PCLK1
# define SPI_M2S_RX_DMA		2
# define SPI_M2S_TX_DMA		3
#endif

#if defined(CONFIG_M2S_MSS_SPI0) || defined(CONFIG_M2S_MSS_SPI1)

static struct resource spi_m2s_dev_resources[] = {
	{
		.start	= SPI_M2S_REGS,
		.end	= SPI_M2S_REGS + SPI_M2S_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= PDMA_M2S_REGS,
		.end	= PDMA_M2S_REGS + PDMA_M2S_REGS_SIZE,
		.flags	= IORESOURCE_MEM,
	}
};

static struct spi_m2s_platform_data spi_m2s_data_dev = {
	.dma_rx		= SPI_M2S_RX_DMA,
	.dma_tx		= SPI_M2S_TX_DMA,
};

static struct platform_device spi_m2s_dev = {
	.name           = "spi_m2s",
	.id             = SPI_M2S_ID,
	.num_resources  = ARRAY_SIZE(spi_m2s_dev_resources),
	.resource       = spi_m2s_dev_resources,
};

#endif	/* CONFIG_M2S_MSS_SPIx */

/*
 * Register the M2S specific SPI controllers and devices with the kernel.
 */
void __init m2s_spi_init(void)
{
	int	p = m2s_platform_get();

#if defined(CONFIG_M2S_MSS_SPI0) || defined(CONFIG_M2S_MSS_SPI1)
	spi_m2s_data_dev.ref_clk = m2s_clock_get(SPI_M2S_CLK);

	/*
	 * Pass additional params to the driver, and register device
	 */
	platform_set_drvdata(&spi_m2s_dev, &spi_m2s_data_dev);
	platform_device_register(&spi_m2s_dev);
#endif

	if (p == PLATFORM_M2S_SOM) {
#if defined(CONFIG_M2S_MSS_SPI0) && defined(CONFIG_MTD_M25P80)
		/*
		 * SPI Flash partitioning:
		 */
#		define M2S_SOM_SF_MTD_OFFSET		0x010000 /* 64 KB */
#		define M2S_SOM_SF_MTD_SIZE0		0x400000 /*  4 MB */
#		define M2S_SOM_SF_MTD_SIZE1		0xBF0000 /*~12 MB */
		static struct mtd_partition m2s_som_sf_mtd[] = {
			   {
				.name = "spi_flash_part0",
				.offset = M2S_SOM_SF_MTD_OFFSET,
				.size = M2S_SOM_SF_MTD_SIZE0,
			}, {
				.name = "spi_flash_part1",
				.offset = M2S_SOM_SF_MTD_OFFSET +
					  M2S_SOM_SF_MTD_SIZE0,
				.size = M2S_SOM_SF_MTD_SIZE1,
			},
		};

		static struct flash_platform_data m2s_som_sf_data = {
			.name = "s25fl129p1",
			.parts = m2s_som_sf_mtd,
			.nr_parts = ARRAY_SIZE(m2s_som_sf_mtd),
			.type = "s25fl129p1",
		};

		static struct spi_board_info m2s_som_sf_inf = {
			.modalias = "m25p32",
			.max_speed_hz = 20000000,
			.bus_num = 0,
			.chip_select = 0,
			.platform_data = &m2s_som_sf_data,
			.mode = SPI_MODE_3,
		};

		spi_register_board_info(&m2s_som_sf_inf, 1);
#endif
	}
}
