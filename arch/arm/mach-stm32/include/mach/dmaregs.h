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

#ifndef _MACH_STM32_DMAREGS_H_
#define _MACH_STM32_DMAREGS_H_

#include <linux/types.h>

#include <mach/stm32.h>

/* Number of DMA channels (0..15) */
#define STM32F2_DMA_CH_NUM		16
/* Number of DMA channels associated with the DMA1 controller (0..7) */
#define STM32F2_DMA_CH_NUM_DMA1		8

/*
 * DMA CR bits
 */
#ifndef CONFIG_ARCH_STM32F1
/*
 * Streams and Double Buffer Mode are not supported on STM32F1
 */
/* Channel selection */
#define STM32_DMA_CR_CHSEL_BIT		25
/* Current target */
#define STM32_DMA_CR_CT			(1 << 19)
/* Double buffer mode */
#define STM32_DMA_CR_DBM_BIT		18
#define STM32_DMA_CR_DBM		(1 << STM32_DMA_CR_DBM_BIT)

/* Memory burst transfer configuration */
#define STM32_DMA_CR_MBURST_BITS	23
#define STM32_DMA_CR_MBURST_MSK		(3 << STM32_DMA_CR_MBURST_BITS)
/* Peripheral burst transfer configuration */
#define STM32_DMA_CR_PBURST_BITS	21
#define STM32_DMA_CR_PBURST_MSK		(3 << STM32_DMA_CR_PBURST_BITS)

/* Peripheral increment offset size */
#define STM32_DMA_CR_PINCOS_MSK		(1 << 15)

/* Memory data size */
#define STM32_DMA_CR_MSIZE_BITS		13
#define STM32_DMA_CR_MSIZE_MSK		(3 << STM32_DMA_CR_MSIZE_BITS)
/* Peripheral data size */
#define STM32_DMA_CR_PSIZE_BITS		11
#define STM32_DMA_CR_PSIZE_MSK		(3 << STM32_DMA_CR_PSIZE_BITS)
#endif

/* Priority level */
#ifdef CONFIG_ARCH_STM32F1
#define STM32_DMA_CR_PL_BIT		12		/* STM32F1 */
#else
#define STM32_DMA_CR_PL_BIT		16		/* STM32F2 */
#endif
#define STM32_DMA_CR_PL_MSK		(3 << STM32_DMA_CR_PL_BIT)
#define STM32_DMA_CR_PL_HIGH		0x2

#ifdef CONFIG_ARCH_STM32F1
/* STM32F1 */
/* Memory increment mode */
#define STM32_DMA_CR_MINC		(1 << 7)
/* Circular mode */
#define STM32_DMA_CR_CIRC_BIT		5
#define STM32_DMA_CR_CIRC		(1 << STM32_DMA_CR_CIRC_BIT)
/* Transfer complete irq ena */
#define STM32_DMA_CR_TCIE		(1 << 1)
/* Half transfer irq ena */
#define STM32_DMA_CR_HTIE		(1 << 2)
#else
/* STM32F2 */
/* Memory increment mode */
#define STM32_DMA_CR_MINC_BIT		10
#define STM32_DMA_CR_MINC		(1 << STM32_DMA_CR_MINC_BIT)
/* Peripheral increment mode */
#define STM32_DMA_CR_PINC_BIT		9
#define STM32_DMA_CR_PINC		(1 << STM32_DMA_CR_PINC_BIT)
/* Circular mode */
#define STM32_DMA_CR_CIRC_BIT		8
#define STM32_DMA_CR_CIRC		(1 << STM32_DMA_CR_CIRC_BIT)
/* Data transfer direction */
#define STM32_DMA_CR_DIR_BITS		6
#define STM32_DMA_CR_DIR_MSK		(3 << STM32_DMA_CR_DIR_BITS)
/* Peripheral flow controller */
#define STM32_DMA_CR_PFCTRL_BIT		5
#define STM32_DMA_CR_PFCTRL_MSK		(1 << STM32_DMA_CR_PFCTRL_BIT)
/* Transfer complete irq ena */
#define STM32_DMA_CR_TCIE		(1 << 4)
/* Half transfer irq ena */
#define STM32_DMA_CR_HTIE		(1 << 3)
#endif

/* Stream enable */
#define STM32_DMA_CR_EN			(1 << 0)

/*
 * DMA NDTR bits
 */
/* Num of data items to xfer */
#define STM32_DMA_NDTR_NDT_BIT		0
#define STM32_DMA_NDTR_NDT_MSK		0xFFFF

/*
 * DMA channel register map. Part of the DMA controller register map.
 */
struct stm32_dma_ch_regs {
	u32		cr;	/* configuration */
	u32		ndtr;	/* number of data */
	volatile void	*par;	/* peripheral address */
	volatile void	*m0ar;	/* memory 0 address */
#ifdef CONFIG_ARCH_STM32F1
	u32		rsv0;
#else
	volatile void	*m1ar;	/* memory 1 address */
	u32		fcr;	/* FIFO control */
#endif
};

/*
 * DMA register map
 */
struct stm32_dma_regs {
	u32	lisr;			/* low interrupt status		      */
#ifndef CONFIG_ARCH_STM32F1
	u32	hisr;			/* high interrupt status	      */
#endif
	u32	lifcr;			/* low interrupt flag clear	      */
#ifndef CONFIG_ARCH_STM32F1
	u32	hifcr;			/* high interrupt flag clear	      */
#endif

	struct stm32_dma_ch_regs s[8];
};

/*
 * STM32 DMA bases
 */
#ifdef CONFIG_ARCH_STM32F1
/* STM32F1 */
#define STM32_DMA1_BASE		(STM32_AHB1PERITH_BASE + 0x0000)
#define STM32_DMA2_BASE		(STM32_AHB1PERITH_BASE + 0x0400)
#else
/* STM32F2 */
#define STM32_DMA1_BASE		(STM32_AHB1PERITH_BASE + 0x6000)
#define STM32_DMA2_BASE		(STM32_AHB1PERITH_BASE + 0x6400)
#endif

#define STM32_DMA1		((volatile struct stm32_dma_regs *) \
				STM32_DMA1_BASE)
#define STM32_DMA2		((volatile struct stm32_dma_regs *) \
				STM32_DMA2_BASE)

/*
 * Masks for the DMA{1,2}_EN bits in the RCC_AHB1ENR register
 */
#ifdef CONFIG_ARCH_STM32F1
/* STM32F1 */
#define STM32_RCC_AHB1ENR_DMA1_MSK	(1 << 0)
#define STM32_RCC_AHB1ENR_DMA2_MSK	(1 << 1)
#else
/* STM32F2 */
#define STM32_RCC_AHB1ENR_DMA1_MSK	(1 << 21)
#define STM32_RCC_AHB1ENR_DMA2_MSK	(1 << 22)
#endif

#endif	/*_MACH_STM32_DMAREGS_H_ */
