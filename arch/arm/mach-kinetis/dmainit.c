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

#include <mach/kinetis.h>
#include <mach/power.h>
#include <mach/dmainit.h>
#include <mach/dmac.h>

/* Number of DMA channels associated with DMAMUX0 */
#define KINETIS_DMA_CH_NUM_MUX0		16

struct kinetis_dma_ch_config {
	int ch;		/* DMA channel */
	u8 source;	/* DMA request source ID */
};

/*
 * DMA request multiplexer (DMAMUX) register map
 */
struct kinetis_dmamux_regs {
	u8 chcfg[16];
};

/*
 * DMAMUX register map bases
 */
#define KINETIS_DMAMUX0_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00021000)
#define KINETIS_DMAMUX0		((volatile struct kinetis_dmamux_regs *) \
				KINETIS_DMAMUX0_BASE)
#define KINETIS_DMAMUX1_BASE	(KINETIS_AIPS0PERIPH_BASE + 0x00022000)
#define KINETIS_DMAMUX1		((volatile struct kinetis_dmamux_regs *) \
				KINETIS_DMAMUX1_BASE)

/*
 * DMAMUXx_CHCFGn bit fields
 */
/* DMA Channel Enable */
#define KINETIS_DMAMUX_CHCFG_ENBL_MSK		(1 << 7)
/* DMA Channel Trigger Enable */
#define KINETIS_DMAMUX_CHCFG_TRIG_MSK		(1 << 6)
/* DMA Channel Source (slot) */
#define KINETIS_DMAMUX_CHCFG_SOURCE_BITS	0

static int dma_ch_config(const struct kinetis_dma_ch_config *cfg)
{
	int rv;
	int dmamux;
	volatile struct kinetis_dmamux_regs *dmamux_regs;

	/*
	 * Verify the function argument
	 */
	if (cfg->ch < 0 || cfg->ch >= KINETIS_DMA_CH_NUM) {
		rv = -EINVAL;
		goto out;
	}

	/*
	 * Enable DMA controller clock
	 */
	rv = kinetis_periph_enable(KINETIS_CG_DMA, 1);
	if (rv != 0)
		goto out;

	/*
	 * Enable clock for the necessary DMA request multiplexer
	 */
	dmamux = cfg->ch < KINETIS_DMA_CH_NUM_MUX0 ? 0 : 1;
	rv = kinetis_periph_enable(
		dmamux ? KINETIS_CG_DMAMUX1 : KINETIS_CG_DMAMUX0, 1);
	if (rv != 0)
		goto out;

	/*
	 * Set DMA request source and enable this DMA request
	 * in the respective multiplexer.
	 */
	dmamux_regs = dmamux ? KINETIS_DMAMUX1 : KINETIS_DMAMUX0;
	dmamux_regs->chcfg[cfg->ch % KINETIS_DMA_CH_NUM_MUX0] =
		KINETIS_DMAMUX_CHCFG_ENBL_MSK |
		(cfg->source << KINETIS_DMAMUX_CHCFG_SOURCE_BITS);

	rv = 0;
out:
	return rv;
}

static int dma_ch_config_table(const struct kinetis_dma_ch_config *table,
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

static const struct kinetis_dma_ch_config k70_dmamux_table[] = {
	/*
	 * DMAMUX0, channels 0-15
	 */
#if defined(CONFIG_KINETIS_UART0)
	{KINETIS_DMACH_UART0_RX, 2},
#endif /* CONFIG_KINETIS_UART0 */

#if defined(CONFIG_KINETIS_UART1)
	{KINETIS_DMACH_UART1_RX, 4},
#endif /* CONFIG_KINETIS_UART1 */

#if defined(CONFIG_KINETIS_UART2)
	{KINETIS_DMACH_UART2_RX, 6},
#endif /* CONFIG_KINETIS_UART2 */

#if defined(CONFIG_KINETIS_UART3)
	{KINETIS_DMACH_UART3_RX, 8},
#endif /* CONFIG_KINETIS_UART3 */

#if defined(CONFIG_KINETIS_UART4)
	{KINETIS_DMACH_UART4_RX, 10},
#endif /* CONFIG_KINETIS_UART4 */

#if defined(CONFIG_KINETIS_UART5)
	{KINETIS_DMACH_UART5_RX, 12},
#endif /* CONFIG_KINETIS_UART5 */

	{KINETIS_DMACH_ESDHC, 54},
};

/*
 * Initialize the IOMUX Alternative Functions of the Freescale Kinetis MCU
 */
void __init kinetis_dma_init(void)
{
	/* Configure the DMA request multiplexers */
	dma_ch_config_table(k70_dmamux_table, ARRAY_SIZE(k70_dmamux_table));

	/* Initialize the DMA controller driver API */
	kinetis_dmac_init();
}
