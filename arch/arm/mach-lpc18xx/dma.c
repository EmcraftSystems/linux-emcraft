/*
 * arch/arm/mach-lpc18xx/dma.c
 *
 *  (was named linux/arch/arm/mach-lpc32xx/ma-lpc32xx.c)
 *  Copyright (C) 2008 NXP Semiconductors
 *  (Based on parts of the PNX4008 DMA driver)
 *
 * Customized for LPC178x/7x by:
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * Customized for LPC43XX:
 * (C) Copyright 2014, Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <asm/dma-mapping.h>
#include <asm/io.h>

#include <mach/dmac.h>
#include <mach/lpc18xx.h>

#include <linux/amba/bus.h>
#include <linux/amba/pl08x.h>

atomic_t dmamux10_used;

#define LPC18XX_CREG_DMAMUXPER9		(1 << 18)
#define LPC18XX_CREG_DMAMUXPER10	(1 << 20)

static int pl08x_get_xfer_signal(const struct pl08x_channel_data *cd)
{
	if (cd->min_signal == 10) {
		if (!atomic_dec_and_test(&dmamux10_used))
			return -EBUSY;
		LPC18XX_CREG->dmamux |= cd->muxval;
	}
	return cd->min_signal;
}

static void pl08x_put_xfer_signal(const struct pl08x_channel_data *cd, int ch)
{
	if (cd->min_signal == 10) {
		atomic_inc(&dmamux10_used);
		LPC18XX_CREG->dmamux &= ~cd->muxval;
	}
}

static struct pl08x_channel_data lpc4357_dma_info[] = {
	{
		.bus_id = "spi0_rx",
		.min_signal = 9,
		.max_signal = 9,
		.periph_buses = PL08X_AHB2,
		.single = true,
	},
	{
		.bus_id = "spi0_tx",
		.min_signal = 10,
		.max_signal = 10,
		.periph_buses = PL08X_AHB2,
		.single = true,
	},
	{
		.bus_id = "i2s0_tx",
		.min_signal = 10,
		.max_signal = 10,
		.periph_buses = PL08X_AHB2,
		.muxval = LPC18XX_CREG_DMAMUXPER10,
	}
};

static struct pl08x_platform_data  lpc18xx_pl08x_dma_data = {
	.slave_channels = &lpc4357_dma_info[0],
	.num_slave_channels = ARRAY_SIZE(lpc4357_dma_info),
	.memcpy_channel = {},
	.get_signal = pl08x_get_xfer_signal,
	.put_signal = pl08x_put_xfer_signal,
	.mem_buses = PL08X_AHB1,
};

static struct amba_device lpc18xx_pl08x_dma = {
	.dev                            = {
		.coherent_dma_mask      = ~0,
		.init_name              = "pl08xdmac",
		.platform_data          = &lpc18xx_pl08x_dma_data,
	},
	.res                            = {
		.start                  = LPC18XX_DMA_BASE,
		.end                    = LPC18XX_DMA_BASE + 0x1000 - 1,
		.flags                  = IORESOURCE_MEM,
	},
	.dma_mask                       = ~0,
	.irq                            = {LPC18XX_DMA_IRQ, NO_IRQ},
};

void __init lpc18xx_dma_init(void)
{
	atomic_set(&dmamux10_used, 1);
	amba_device_register(&lpc18xx_pl08x_dma, &iomem_resource);
}
