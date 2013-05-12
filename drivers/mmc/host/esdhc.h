/*
 * Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *  Author: Chenghu Wu <b16972@freescale.com>
 *	     Xiaobo Xie <X.Xie@freescale.com>
 *
 * Based on mpc837x/driver/mmc/host/esdhc.c done by Xiaobo Xie
 * Ported to Coldfire platform by Chenghu Wu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#ifndef ESDHC_H
#define ESDHC_H

#define MMC_ERR_NONE    0
#define MMC_ERR_TIMEOUT 1
#define MMC_ERR_BADCRC  2
#define MMC_ERR_FIFO    3
#define MMC_ERR_FAILED  4
#define MMC_ERR_INVALID 5

//#define MCF_CLOCK_PLL_DR        (*(volatile unsigned long *)(0xFC0C0004))
#define MCF_ESDHC_HOSTCAPBLT    (*(volatile unsigned long *)(0x400B1040))
//#define MCF_ESDHC_ADMAESR       (*(volatile unsigned long *)(0xFC0CC054))
//#define MCF_ESDHC_ADMASAR       (*(volatile unsigned long *)(0xFC0CC058))
#define MCF_ESDHC_VSR           (*(volatile unsigned long *)(0x400B10C0))
//#define MCF_ESDHC_HOSTVER       (*(volatile unsigned long *)(0xFC0CC0FC))
/*
 * Controller registers (Big Endian)
 */

#if 0
#define MCF_GPIO_PAR_SDHC_DATA3   0x20
#define MCF_GPIO_PAR_SDHC_DATA2   0x10
#define MCF_GPIO_PAR_SDHC_DATA1   0x08
#define MCF_GPIO_PAR_SDHC_DATA0   0x04
#define MCF_GPIO_PAR_SDHC_CMD     0x02
#define MCF_GPIO_PAR_SDHC_CLK     0x01


#define MCF_GPIO_SRCR_SDHC_LOWEST    0x00
#define MCF_GPIO_SRCR_SDHC_LOWE      0x01
#define MCF_GPIO_SRCR_SDHC_HIGH      0x02
#define MCF_GPIO_SRCR_SDHC_HIGHEST   0x03


#define MCF_GPIO_PCRL_SDHC_DATA3   0x80
#define MCF_GPIO_PCRL_SDHC_DATA2   0x40
#define MCF_GPIO_PCRL_SDHC_DATA1   0x20
#define MCF_GPIO_PCRL_SDHC_DATA0   0x10
#define MCF_GPIO_PCRL_SDHC_CMD     0x08
#define MCF_GPIO_PCRL_SDHC_CLK     0x04
#endif

#define MCF_INT_SDHC               63
/* DMA System Address Register */
#define ESDHC_DMA_ADDRESS	0x00

/* Block Attributes Register */
#define ESDHC_BLOCK_ATTR	0x04
#define ESDHC_BLOCK_SIZE_MASK	0x00000fff
#define ESDHC_BLCOK_CNT_MASK	0xffff0000
#define ESDHC_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 12) | (blksz & 0xFFF))

/* Command Argument */
#define ESDHC_ARGUMENT		0x08

/* Transfer Type Register */
#define ESDHC_COMMAND		0x0C

#define ESDHC_TRNS_DMA		0x00000001
#define ESDHC_TRNS_BLK_CNT_EN	0x00000002
#define ESDHC_TRNS_ACMD12	0x00000004
#define ESDHC_TRNS_READ		0x00000010
#define ESDHC_TRNS_MULTI	0x00000020

#define ESDHC_CMD_RESP_MASK	0x00030000
#define ESDHC_CMD_CRC_EN	0x00080000
#define ESDHC_CMD_INDEX_EN	0x00100000
#define ESDHC_CMD_DATA		0x00200000
#define ESDHC_CMD_TYPE_MASK	0x00c00000
#define ESDHC_CMD_INDEX		0x3f000000

#define ESDHC_CMD_RESP_NONE	0x00000000
#define ESDHC_CMD_RESP_LONG	0x00010000
#define ESDHC_CMD_RESP_SHORT	0x00020000
#define ESDHC_CMD_RESP_SHORT_BUSY 0x00030000

#define ESDHC_MAKE_CMD(c, f) (((c & 0xff) << 24) | (f & 0xfb0037))

/* Response Register */
#define ESDHC_RESPONSE		0x10

/* Buffer Data Port Register */
#define ESDHC_BUFFER		0x20

/* Present State Register */
#define ESDHC_PRESENT_STATE	0x24
#define  ESDHC_CMD_INHIBIT	0x00000001
#define  ESDHC_DATA_INHIBIT	0x00000002
#define  ESDHC_DATA_DLA		0x00000004
#define  ESDHC_SDSTB		0x00000008
#define  ESDHC_DOING_WRITE	0x00000100
#define  ESDHC_DOING_READ	0x00000200
#define  ESDHC_SPACE_AVAILABLE	0x00000400
#define  ESDHC_DATA_AVAILABLE	0x00000800
#define  ESDHC_CARD_PRESENT	0x00010000
#define  ESDHC_WRITE_PROTECT	0x00080000

/* Protocol control Register */
#define ESDHC_PROTOCOL_CONTROL	0x28

#define ESDHC_CTRL_BUS_MASK	0x00000006
#define ESDHC_CTRL_4BITBUS	0x00000002
#define ESDHC_CTRL_D3_DETEC	0x00000008
#define ESDHC_CTRL_DTCT_EN	0x00000080
#define ESDHC_CTRL_DTCT_STATUS	0x00000040
#define ESDHC_CTRL_DMAS_SDMA	0x00000000
#define ESDHC_CTRL_DMAS_ADMA1	0x00000100
#define ESDHC_CTRL_DMAS_ADMA2	0x00000200
#define ESDHC_CTRL_SABGREQ	0x00010000
#define ESDHC_CTRL_WU_CRM	0x04000000
#define ESDHC_CTRL_WU_CINS	0x02000000
#define ESDHC_CTRL_WU_CINT	0x01000000

/* System Control Register */
#define ESDHC_SYSTEM_CONTROL	0x2C

#define ESDHC_CLOCK_MASK	0x0000fff0
#define ESDHC_CLOCK_DEFAULT	0x00008000
#define ESDHC_PREDIV_SHIFT	8
#define ESDHC_DIVIDER_SHIFT	4
#define ESDHC_CLOCK_SDCLKEN	0x00000008
#define ESDHC_CLOCK_CARD_EN	0x00000004
#define ESDHC_CLOCK_INT_STABLE	0x00000002
#define ESDHC_CLOCK_INT_EN	0x00000001

#define ESDHC_TIMEOUT_MASK	0x000f0000
#define ESDHC_TIMEOUT_SHIFT	16

#define ESDHC_RESET_SHIFT	24
#define ESDHC_RESET_ALL		0x01
#define ESDHC_RESET_CMD		0x02
#define ESDHC_RESET_DATA	0x04
#define ESDHC_INIT_CARD		0x08

/* Interrupt Register */
#define ESDHC_INT_STATUS	0x30
#define ESDHC_INT_ENABLE	0x34
#define ESDHC_SIGNAL_ENABLE	0x38

#define ESDHC_INT_RESPONSE	0x00000001
#define ESDHC_INT_DATA_END	0x00000002
#define ESDHC_INT_DMA_END	0x00000008
#define ESDHC_INT_SPACE_AVAIL	0x00000010
#define ESDHC_INT_DATA_AVAIL	0x00000020
#define ESDHC_INT_CARD_INSERT	0x00000040
#define ESDHC_INT_CARD_REMOVE	0x00000080
#define ESDHC_INT_CARD_INT	0x00000100

#define ESDHC_INT_TIMEOUT	0x00010000
#define ESDHC_INT_CRC		0x00020000
#define ESDHC_INT_END_BIT	0x00040000
#define ESDHC_INT_INDEX		0x00080000
#define ESDHC_INT_DATA_TIMEOUT	0x00100000
#define ESDHC_INT_DATA_CRC	0x00200000
#define ESDHC_INT_DATA_END_BIT	0x00400000
#define ESDHC_INT_ACMD12ERR	0x01000000
#define ESDHC_INT_DMAERR	0x10000000

#define ESDHC_INT_NORMAL_MASK	0x00007FFF
#define ESDHC_INT_ERROR_MASK	0xFFFF8000

#define ESDHC_INT_CMD_MASK	(ESDHC_INT_RESPONSE | ESDHC_INT_TIMEOUT | \
		ESDHC_INT_CRC | ESDHC_INT_END_BIT | ESDHC_INT_INDEX)
#define ESDHC_INT_DATA_MASK	(ESDHC_INT_DATA_END | ESDHC_INT_DMA_END | \
		ESDHC_INT_DATA_AVAIL | ESDHC_INT_SPACE_AVAIL | \
		ESDHC_INT_DATA_TIMEOUT | ESDHC_INT_DATA_CRC | \
		ESDHC_INT_DATA_END_BIT)

#define ESDHC_INT_INSERT_MASK (ESDHC_INT_DATA_END_BIT | ESDHC_INT_DATA_CRC | \
		ESDHC_INT_DATA_TIMEOUT | ESDHC_INT_INDEX | \
		ESDHC_INT_END_BIT | ESDHC_INT_CRC | ESDHC_INT_TIMEOUT | \
		ESDHC_INT_DATA_AVAIL | ESDHC_INT_SPACE_AVAIL | \
		ESDHC_INT_DMA_END | ESDHC_INT_DATA_END | \
		ESDHC_INT_RESPONSE | ESDHC_INT_CARD_REMOVE)

#define ESDHC_INT_REMOVE_MASK (ESDHC_INT_DATA_END_BIT | ESDHC_INT_DATA_CRC | \
		ESDHC_INT_DATA_TIMEOUT | ESDHC_INT_INDEX | \
		ESDHC_INT_END_BIT | ESDHC_INT_CRC | ESDHC_INT_TIMEOUT | \
		ESDHC_INT_DATA_AVAIL | ESDHC_INT_SPACE_AVAIL | \
		ESDHC_INT_DMA_END | ESDHC_INT_DATA_END | \
		ESDHC_INT_RESPONSE | ESDHC_INT_CARD_INSERT)

/* Auto CMD12 Error Status Register */
#define ESDHC_ACMD12_ERR	0x3C

/* 3E-3F reserved */
/* Host Controller Capabilities */
#define ESDHC_CAPABILITIES	0x40

#define ESDHC_MAX_BLOCK_MASK	0x00070000
#define ESDHC_MAX_BLOCK_SHIFT	16
#define ESDHC_CAN_DO_HISPD	0x00200000
#define ESDHC_CAN_DO_DMA	0x00400000
#define ESDHC_CAN_DO_SUSPEND	0x00800000
#define ESDHC_CAN_VDD_330	0x01000000
#define ESDHC_CAN_VDD_300	0x02000000
#define ESDHC_CAN_VDD_180	0x04000000

/* Watermark Level Register */
#define ESDHC_WML		0x44
#define ESDHC_WML_MASK		0xff
#define ESDHC_WML_READ_SHIFT	0
#define ESDHC_WML_WRITE_SHIFT	16

/* 45-4F reserved for more caps and max curren*/

/* Force Event Register */
#define ESDHC_FORCE_EVENT	0x50

/* 54-FB reserved */

#define ESDHC_VENDOR		0xC0

/* Host Controller Version Register */
#define ESDHC_HOST_VERSION	0xFC

#define ESDHC_VENDOR_VER_MASK	0xFF00
#define ESDHC_VENDOR_VER_SHIFT	8
#define ESDHC_SPEC_VER_MASK	0x00FF
#define ESDHC_SPEC_VER_SHIFT	0

#define ESDHC_DMA_SYSCTL	0x40C
#define ESDHC_DMA_SNOOP		0x00000040

#define ESDHC_SLOTS_NUMBER	1

/* The SCCR[SDHCCM] Register */
#define MPC837X_SCCR_OFFS	0xA08
#define MPC837X_SDHCCM_MASK	0x0c000000
#define MPC837X_SDHCCM_SHIFT	26

#define ESDHC_ADMA_ACT_NOP	(0 << 4)
#define ESDHC_ADMA_ACT_RSV	(1 << 4)
#define ESDHC_ADMA_ACT_TRAN	(2 << 4)
#define ESDHC_ADMA_ACT_LINK	(3 << 4)

#define ESDHC_ADMA_INT		(1 << 2)
#define ESDHC_ADMA_END		(1 << 1)
#define ESDHC_ADMA_VALID	(1 << 0)

#define esdhc_readl(addr) \
	({ unsigned int __v = (*(volatile unsigned int *) (addr)); __v; })

#define esdhc_writel(b, addr) (void)((*(volatile unsigned int *) (addr)) = (b))

static inline u32 _fsl_readl(const char *who, unsigned __iomem *addr)
{
	u32 val;
	/*val = inl(addr);*/
	val = esdhc_readl(addr);
	return val;
}

static inline void _fsl_writel(const char *who, unsigned __iomem *addr, u32 val)
{
	/*outl(val, addr);*/
	esdhc_writel(val, addr);
}

#define fsl_readl(adr)		_fsl_readl(__func__, adr)
#define fsl_writel(adr, val)	_fsl_writel(__func__, adr, val)

#define setbits32(_addr, _v) outl((_addr), inl(_addr) |  (_v))
#define clrbits32(_addr, _v) outl((_addr), inl(_addr) & ~(_v))

struct esdhc_chip;

struct esdhc_host {
	struct esdhc_chip	*chip;
	struct mmc_host		*mmc;		/* MMC structure */

	spinlock_t		lock;		/* Mutex */

	int			flags;		/* Host attributes */
#define ESDHC_USE_DMA		(1<<0)

	unsigned int		max_clk;	/* Max possible freq (MHz) */
	unsigned int		timeout_clk;	/* Timeout freq (KHz) */

	unsigned int		clock;		/* Current clock (MHz) */
	unsigned short		power;		/* Current voltage */
	unsigned short		bus_width;	/* current bus width */

	struct mmc_request	*mrq;		/* Current request */
	struct mmc_command	*cmd;		/* Current command */
	struct mmc_data		*data;		/* Current data request */

	struct scatterlist	*cur_sg;	/* We're working on this */
	int			num_sg;		/* Entries left */
	int			offset;		/* Offset into current sg */
	int			remain;		/* Bytes left in current */

	int			dma_run;

	char			slot_descr[20];	/* Name for reservations */

	int			card_insert;
	int			retries;

	int			irq;		/* Device IRQ */
	unsigned long		addr;		/* Bus address */
	unsigned int		size;		/* IO size */
	void __iomem		*ioaddr;	/* Mapped address */

	struct tasklet_struct	card_tasklet;	/* Tasklet structures */
	struct tasklet_struct	finish_tasklet;

	struct timer_list	timer;		/* Timer for timeouts */
	void			*dma_tx_buf;
	dma_addr_t		dma_tx_dmahandle;
	unsigned int		dma_tx_blocks;
};

struct esdhc_chip {
	struct platform_device	*pdev;

	unsigned long		quirks;

	int			num_slots;	/* Slots on controller */
	struct esdhc_host	*hosts[0];	/* Pointers to hosts */
};

struct adma_bd {
	unsigned short		attr;
	unsigned short		len;
	unsigned int		addr;
};

#endif
