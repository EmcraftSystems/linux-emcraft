/*
 * (C) Copyright 2014
 * Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
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
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/mmc/dw_mmc.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <mach/lpc18xx.h>
#include <mach/mmc.h>
#include <mach/platform.h>

#define LPC18XX_SDMMC_BASE	(0x40004000)
#define LPC18XX_SDMMC_IRQ	6

int lpc18xx_dw_mci_init(u32 slot_id, irq_handler_t handler, void *data)
{
	return 0;
}

struct dw_mci_board lpc18xx_mmc_data = {
	.init = lpc18xx_dw_mci_init,
	.quirks = DW_MCI_QUIRK_IDMAC_DTO | DW_MCI_QUIRK_RETRY_DELAY,
};

struct resource lpc18xx_mmc_resources[] = {
	{
		.start			= LPC18XX_SDMMC_BASE,
		.end			= LPC18XX_SDMMC_BASE + SZ_4K - 1,
		.flags			= IORESOURCE_MEM
	},
	{
		.start			= LPC18XX_SDMMC_IRQ,
		.flags			= IORESOURCE_IRQ,
	},
};

struct platform_device lpc18xx_mmc_device = {
	.name				= "dw_mmc",
	.dev				= {
		.coherent_dma_mask = ~0UL,
		.platform_data = &lpc18xx_mmc_data
	},
	.num_resources			= ARRAY_SIZE(lpc18xx_mmc_resources),
	.resource			= lpc18xx_mmc_resources,
};

void __init lpc18xx_mmc_init(void)
{
	struct clk *sdio_clk;
	sdio_clk = clk_get(NULL, "sdio");
	if (sdio_clk == NULL) {
		printk(KERN_ERR "can't find sdio_clk\n");
		return;
	}
	lpc18xx_mmc_data.bus_hz = clk_get_rate(sdio_clk);
	platform_device_register(&lpc18xx_mmc_device);
}
