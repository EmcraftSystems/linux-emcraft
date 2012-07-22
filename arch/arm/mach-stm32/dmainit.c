/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
#include <linux/kernel.h>
#include <linux/errno.h>

#include <mach/stm32.h>
#include <mach/dmainit.h>
#include <mach/dmac.h>
#include <mach/dmaregs.h>

struct stm32f2_dma_ch_config {
	int ch;		/* DMA channel ("stream" in terms on STM32) */
	u8 source;	/* DMA request source ("channel" in terms of STM32) */
};

static int dma_ch_config(const struct stm32f2_dma_ch_config *cfg)
{
	int rv;
	int dma_ctrl;
	volatile struct stm32_dma_regs *dma_regs;

	/*
	 * Verify the function argument
	 */
	if (cfg->ch < 0 || cfg->ch >= STM32F2_DMA_CH_NUM) {
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Enable clock for the necessary DMA controller (DMA1 or DMA2)
	 */
	dma_ctrl = cfg->ch < STM32F2_DMA_CH_NUM_DMA1 ? 0 : 1;
	STM32_RCC->ahb1enr |= dma_ctrl ?
		STM32_RCC_AHB1ENR_DMA2_MSK : STM32_RCC_AHB1ENR_DMA1_MSK;

	/*
	 * Set DMA request source for the DMA channel
	 */
	dma_regs = dma_ctrl ? STM32_DMA2 : STM32_DMA1;
	dma_regs->s[cfg->ch % STM32F2_DMA_CH_NUM_DMA1].cr =
		cfg->source << STM32_DMA_CR_CHSEL_BIT;

	rv = 0;
out:
	return rv;
}

static int dma_ch_config_table(const struct stm32f2_dma_ch_config *table,
			       unsigned int len)
{
	unsigned int i;
	int rv;

	for (i = 0; i < len; i ++) {
		rv = dma_ch_config(&table[i]);
		if (rv != 0)
			goto out;
	}

	rv = 0;
out:
	return rv;
}

static const struct stm32f2_dma_ch_config stm32f2_dmamux_table[] = {
	/*
	 * DMA2 request mapping (channels 7-15)
	 */
#if defined(CONFIG_MMC_ARMMMCI)
	/* SDIO Tx/Rx: {DMA2-stream3, channel4} or {DMA2-stream6, channel4} */
	{STM32F2_DMACH_SDIO, 4},
#endif /* CONFIG_MMC_ARMMMCI */
};

/*
 * Initialize the IOMUX Alternative Functions of the STM32F2 MCU
 */
void __init stm32_dma_init(void)
{
#ifndef CONFIG_ARCH_STM32F1
	/* Configure the DMA request multiplexing */
	dma_ch_config_table(
		stm32f2_dmamux_table, ARRAY_SIZE(stm32f2_dmamux_table));
#endif

	/* Initialize the DMA controller driver API */
	stm32_dmac_init();
}
