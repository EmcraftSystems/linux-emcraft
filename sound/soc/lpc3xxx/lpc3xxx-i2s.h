/*
 * sound/soc/lpc3xxx/lpc3xxx-i2s.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
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

#ifndef __SOUND_SOC_LPC3XXX_I2S_H
#define __SOUND_SOC_LPC3XXX_I2S_H

#include <linux/types.h>
#include <mach/clkdev.h>
#include <mach/i2s.h>

#ifdef CONFIG_ARCH_LPC32XX
#define NUM_I2S_DEVICES		2
#else /* LPC178x/7x */
#define NUM_I2S_DEVICES		1
#endif

#define I2S_DMA_XMIT		0x1
#define I2S_DMA_RECV		0x2

struct lpc3xxx_i2s_info {
	char *name;
	spinlock_t lock;
	void *iomem;
	unsigned short initialized;
	u32 dma_flags;
	char *clkname;
	struct clk *clk;
	u32 clkrate;
	u32 baseio;
	int freq;
	unsigned short daifmt;
	int clkdiv;
	u32 dao_save, dai_save, irq_save;
};

extern struct snd_soc_dai lpc3xxx_i2s_dai[];

#endif
