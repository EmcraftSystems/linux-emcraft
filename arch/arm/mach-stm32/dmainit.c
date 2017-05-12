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

/* DMA source codes */
enum stm32_dma_map_codes {
	I2C1_RX,
	I2C2_RX,
	I2C3_RX,
	I2C4_RX,
	I2C1_TX,
	I2C2_TX,
	I2C3_TX,
	I2C4_TX,
	SDMMC,
	UART1_RX,
	UART2_RX,
	UART3_RX,
	UART4_RX,
	UART5_RX,
	UART6_RX,
	UART7_RX,
	UART8_RX,
	UART1_TX,
	UART2_TX,
	UART3_TX,
	UART4_TX,
	UART5_TX,
	UART6_TX,
	UART7_TX,
	UART8_TX,
	DAC,
	NOTSUP,		/* not supported by software */
	NOTDEF,		/* not defined by hardware */
};

struct stm32f2_dma_ch_config {
	int ch;		/* DMA channel ("stream" in terms on STM32) */
        enum stm32_dma_map_codes code;	/* Functional code of the DMA request source */
};

/* DMA request mapping */
static enum stm32_dma_map_codes request_map[16][8] =
{
	/* DMA1-stream 0 */
	{ NOTSUP, I2C1_RX, NOTSUP,  NOTDEF, UART5_RX, UART8_TX, NOTSUP, NOTSUP },
	/* DMA1-stream 1 */
	{ NOTSUP, I2C3_RX, NOTDEF, NOTSUP, UART3_RX, UART7_TX, NOTSUP, NOTSUP },
	/* DMA1-stream 2 */
	{ NOTSUP, NOTSUP, I2C4_RX, I2C3_RX, UART4_RX, NOTSUP, NOTSUP, I2C2_RX },
	/* DMA1-stream 3 */
	{ NOTSUP, NOTDEF, NOTSUP, NOTDEF, UART3_TX, UART7_RX, NOTSUP, I2C2_RX },
	/* DMA1-stream 4 */
	{ NOTSUP, NOTSUP, NOTDEF, I2C3_TX, UART4_TX, NOTSUP, NOTSUP, UART3_TX },
	/* DMA1-stream 5 */
	{ NOTSUP, I2C1_RX, I2C4_TX, NOTSUP, UART2_RX, NOTSUP, NOTDEF, DAC },
	/* DMA1-stream 6 */
	{ NOTSUP, I2C1_TX, NOTSUP, NOTSUP, UART2_TX, UART8_RX, NOTSUP, DAC },
	/* DMA1-stream 7 */
	{ NOTSUP, I2C1_TX, NOTSUP, NOTSUP, UART5_TX, NOTSUP, NOTDEF, I2C2_TX },
	/* DMA2-stream 0 */
	{ NOTSUP, NOTDEF, NOTSUP, NOTSUP, NOTSUP, NOTDEF, NOTSUP, NOTDEF },
	/* DMA2-stream 1 */
	{ NOTSUP, NOTSUP, NOTSUP, NOTDEF, NOTSUP, UART6_RX, NOTSUP, NOTSUP },
	/* DMA2-stream 2 */
	{ NOTSUP, NOTSUP, NOTDEF, NOTSUP, UART1_RX, UART6_RX, NOTSUP, NOTSUP },
	/* DMA2-stream 3 */
	{ NOTSUP, NOTSUP, NOTSUP, NOTSUP, SDMMC, NOTSUP, NOTSUP, NOTSUP },
	/* DMA2-stream 4 */
	{ NOTSUP, NOTSUP, NOTSUP, NOTSUP, NOTDEF, NOTSUP, NOTSUP, NOTSUP },
	/* DMA2-stream 5 */
	{ NOTSUP, NOTSUP, NOTSUP, NOTSUP, UART1_RX, NOTDEF, NOTSUP, NOTSUP },
	/* DMA2-stream 6 */
	{ NOTSUP, NOTSUP, NOTSUP, NOTSUP, SDMMC, UART6_TX, NOTSUP, NOTSUP },
	/* DMA2-stream 7 */
	{ NOTSUP, NOTSUP, NOTSUP, NOTSUP, UART1_TX, UART6_TX, NOTDEF, NOTSUP },
};

static int dma_ch_config(const struct stm32f2_dma_ch_config *cfg)
{
	int rv;
	int dma_ctrl;
	volatile struct stm32_dma_regs *dma_regs;
	int i;
	u8 source;	/* DMA request source ("channel" in terms of STM32) */

	/*
	 * Verify the function argument
	 */
	if (cfg->ch < 0 || cfg->ch >= STM32F2_DMA_CH_NUM) {
		rv = -EINVAL;
		goto out;
	}

	/* Detect channel number */
	for (i = 0; i < 8; i++) {
		if (request_map[cfg->ch][i] == cfg->code) {
			source = i;
			break;
		}
	}

	if (i == 8) {
		/* Channel not found */
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
		source << STM32_DMA_CR_CHSEL_BIT;

	printk("configure stream %d chan %d\n", cfg->ch, source);

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
#if defined(CONFIG_I2C_STM32F7)
#if defined(CONFIG_STM32_I2C1)
	/* I2C1 Rx: {DMA1-stream0, channel1} or {DMA1-stream5, channel1} */
	{STM32F7_DMACH_I2C1_RX, I2C1_RX},
	/* I2C1 Tx: {DMA1-stream6, channel1} or {DMA1-stream7, channel1} */
	{STM32F7_DMACH_I2C1_TX, I2C1_TX},
#endif /* CONFIG_STM32_I2C1 */
#if defined(CONFIG_STM32_I2C2)
	/* I2C2 Rx: {DMA1-stream2, channel7} or {DMA1-stream3, channel7} */
	{STM32F7_DMACH_I2C2_RX, I2C2_RX},
	/* I2C2 Tx: {DMA1-stream7, channel7} */
	{STM32F7_DMACH_I2C2_TX, I2C2_TX},
#endif /* CONFIG_STM32_I2C2 */
#if defined(CONFIG_STM32_I2C3)
	/* I2C1 Rx: {DMA1-stream1, channel1} or {DMA1-stream2, channel3} */
	{STM32F7_DMACH_I2C3_RX, I2C3_RX},
	/* I2C1 Tx: {DMA1-stream4, channel3} */
	{STM32F7_DMACH_I2C3_TX, I2C3_TX},
#endif /* CONFIG_STM32_I2C3 */
#if defined(CONFIG_STM32_I2C4)
	/* I2C1 Rx: {DMA1-stream2, channel2} */
	{STM32F7_DMACH_I2C4_RX, I2C4_RX},
	/* I2C1 Tx: {DMA1-stream5, channel2} */
	{STM32F7_DMACH_I2C4_TX, I2C4_TX},
#endif /* CONFIG_STM32_I2C4 */
#endif /* CONFIG_I2C_STM32F7 */

#if defined(CONFIG_STM32_DAC1_DMA)
	{STM32F7_DMACH_DAC1, DAC },
#endif /* CONFIG_STM32_DAC1_DMA */
#if defined(CONFIG_STM32_DAC2_DMA)
	{STM32F7_DMACH_DAC2, DAC },
#endif /* CONFIG_STM32_DAC2_DMA */

	/*
	 * DMA2 request mapping (channels 7-15)
	 */
#if defined(CONFIG_MMC_ARMMMCI)
	/* SDIO Tx/Rx: {DMA2-stream3, channel4} or {DMA2-stream6, channel4} */
	{STM32F2_DMACH_SDIO, SDMMC},
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
