/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Shaohui Xie <b21989@freescale.com>
 *	   Jason Jin <Jason.jin@freescale.com>
 *
 * Description:
 * MPC5125 Nand driver.
 * Jason ported to M54418TWR.
 *
 * Based on original driver mpc5121_nfc.c.
 *
 * (C) Copyright 2012
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
 * Add support for Freescale Kinetis, used by TWR-K70F120M
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>

#ifdef CONFIG_M68K
#include <asm/mcfsim.h>
#endif

#include <linux/mtd/fsl_nfc.h>
#include "fsl_nfc.h"

#define	DRV_NAME		"fsl_nfc"
#define	DRV_VERSION		"1.0"

/* Timeouts */
#define NFC_RESET_TIMEOUT	1000		/* 1 ms */
#define NFC_TIMEOUT		(HZ)


#define ECC_SRAM_ADDR	(0x840 >> 3)
#define ECC_STATUS_MASK	0x80
#define ECC_ERR_COUNT	0x3F

#define MIN(x, y)		((x < y) ? x : y)

#ifdef CONFIG_MTD_NAND_FSL_NFC_SWECC
static int hardware_ecc;
#else
static int hardware_ecc = 1;
#endif


struct fsl_nfc_prv {
	struct mtd_info		mtd;
	struct nand_chip	chip;
	int			irq;
	void __iomem		*regs;
	struct clk		*clk;
	wait_queue_head_t	irq_waitq;
	uint			column;
	int			spareonly;
	int			page;
};

static int get_status;
static int get_id;

static u8 bbt_pattern[] = {'B', 'b', 't', '0' };
static u8 mirror_pattern[] = {'1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	11,
	.len = 4,
	.veroffs = 15,
	.maxblocks = 4,
	.pattern = bbt_pattern,
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	11,
	.len = 4,
	.veroffs = 15,
	.maxblocks = 4,
	.pattern = mirror_pattern,
};


#ifdef CONFIG_MTD_PARTITIONS
static const char *fsl_nfc_pprobes[] = { "cmdlinepart", NULL };
#endif

static struct nand_ecclayout fsl_nfc_ecc45 = {
	.eccbytes = 45,
	.eccpos = {19, 20, 21, 22, 23,
		   24, 25, 26, 27, 28, 29, 30, 31,
		   32, 33, 34, 35, 36, 37, 38, 39,
		   40, 41, 42, 43, 44, 45, 46, 47,
		   48, 49, 50, 51, 52, 53, 54, 55,
		   56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = {
		{
			.offset = 8,
			/*
			 * 11 bytes are actually available for a client to
			 * place data into the out of band area (OOB.)
			 *
			 * We write "2" here though to make the JFFS2 code
			 * happy. See how the function `jffs2_check_oob_empty()`
			 * checks if a block is empty.
			 *
			 * TBD: Properly fix this JFFS2 problem.
			 */
			.length = 2,
		},
	},
};

static inline u32 nfc_read(struct mtd_info *mtd, uint reg)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	return __raw_readl(prv->regs + reg);
}

/* Write NFC register */
static inline void nfc_write(struct mtd_info *mtd, uint reg, u32 val)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	__raw_writel(val, prv->regs + reg);
}

/* Set bits in NFC register */
static inline void nfc_set(struct mtd_info *mtd, uint reg, u32 bits)
{
	nfc_write(mtd, reg, nfc_read(mtd, reg) | bits);
}

/* Clear bits in NFC register */
static inline void nfc_clear(struct mtd_info *mtd, uint reg, u32 bits)
{
	nfc_write(mtd, reg, nfc_read(mtd, reg) & ~bits);
}

static inline void
nfc_set_field(struct mtd_info *mtd, u32 reg, u32 mask, u32 shift, u32 val)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	__raw_writel((__raw_readl(prv->regs + reg) & (~mask)) | val << shift,
		prv->regs + reg);
}

static inline int
nfc_get_field(struct mtd_info *mtd, u32 reg, u32 field_mask)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	return __raw_readl(prv->regs + reg) & field_mask;
}

static inline u8 nfc_check_status(struct mtd_info *mtd)
{
	u8 fls_status = 0;
	fls_status = nfc_get_field(mtd, NFC_FLASH_STATUS2, STATUS_BYTE1_MASK);
	return fls_status;
}

/* clear cmd_done and cmd_idle falg for the coming command */
static void fsl_nfc_clear(struct mtd_info *mtd)
{
	nfc_write(mtd, NFC_IRQ_STATUS, 1 << CMD_DONE_CLEAR_SHIFT);
	nfc_write(mtd, NFC_IRQ_STATUS, 1 << IDLE_CLEAR_SHIFT);
}

/* Wait for operation complete */
static void fsl_nfc_done(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;
	int rv;

	nfc_set(mtd, NFC_IRQ_STATUS, IDLE_EN_MASK);

	nfc_set_field(mtd, NFC_FLASH_CMD2, START_MASK,
			START_SHIFT, 1);

	if (!nfc_get_field(mtd, NFC_IRQ_STATUS, IDLE_IRQ_MASK)) {
		rv = wait_event_timeout(prv->irq_waitq,
			nfc_get_field(mtd, NFC_IRQ_STATUS,
				IDLE_IRQ_MASK), NFC_TIMEOUT);
		if (!rv)
			printk(KERN_DEBUG DRV_NAME
				": Timeout while waiting for BUSY.\n");
	}
	fsl_nfc_clear(mtd);
}

static inline u8 fsl_nfc_get_id(struct mtd_info *mtd, int col)
{
	/*
	 * Get the (col+1)th byte from the Flash Status Register 1
	 */
	return (u8)(nfc_read(mtd, NFC_FLASH_STATUS1) >> ((3 - col) * 8));
}

static inline u8 fsl_nfc_get_status(struct mtd_info *mtd)
{
	/*
	 * Get the byte returned by the read status command
	 */
	return (u8)nfc_read(mtd, NFC_FLASH_STATUS2);
}

/* Invoke command cycle */
static inline void
fsl_nfc_send_cmd(struct mtd_info *mtd, u32 cmd_byte1,
		u32 cmd_byte2, u32 cmd_code)
{
	fsl_nfc_clear(mtd);
	nfc_set_field(mtd, NFC_FLASH_CMD2, CMD_BYTE1_MASK,
			CMD_BYTE1_SHIFT, cmd_byte1);

	nfc_set_field(mtd, NFC_FLASH_CMD1, CMD_BYTE2_MASK,
			CMD_BYTE2_SHIFT, cmd_byte2);

	nfc_set_field(mtd, NFC_FLASH_CMD2, BUFNO_MASK,
			BUFNO_SHIFT, 0);

	nfc_set_field(mtd, NFC_FLASH_CMD2, CMD_CODE_MASK,
			CMD_CODE_SHIFT, cmd_code);

	if (cmd_code == RANDOM_OUT_CMD_CODE)
		nfc_set_field(mtd, NFC_FLASH_CMD2, BUFNO_MASK,
			BUFNO_SHIFT, 1);
}

/* Receive ID and status from NAND flash */
static inline void
fsl_nfc_send_one_byte(struct mtd_info *mtd, u32 cmd_byte1, u32 cmd_code)
{
	fsl_nfc_clear(mtd);

	nfc_set_field(mtd, NFC_FLASH_CMD2, CMD_BYTE1_MASK,
			CMD_BYTE1_SHIFT, cmd_byte1);

	nfc_set_field(mtd, NFC_FLASH_CMD2, BUFNO_MASK,
			BUFNO_SHIFT, 0);

	nfc_set_field(mtd, NFC_FLASH_CMD2, CMD_CODE_MASK,
			CMD_CODE_SHIFT, cmd_code);
}

/* NFC interrupt handler */
static irqreturn_t
fsl_nfc_irq(int irq, void *data)
{
	struct mtd_info *mtd = data;
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	nfc_clear(mtd, NFC_IRQ_STATUS, IDLE_EN_MASK);
	wake_up(&prv->irq_waitq);

	return IRQ_HANDLED;
}

/* Do address cycle(s) */
static void
fsl_nfc_addr_cycle(struct mtd_info *mtd, int column, int page)
{

	if (column != -1) {
		nfc_set_field(mtd, NFC_COL_ADDR,
				COL_ADDR_MASK,
				COL_ADDR_SHIFT, column);
	}

	if (page != -1) {
		nfc_set_field(mtd, NFC_ROW_ADDR,
				ROW_ADDR_MASK,
				ROW_ADDR_SHIFT, page);
	}

	/* DMA Disable */
	nfc_clear(mtd, NFC_FLASH_CONFIG, CONFIG_DMA_REQ_MASK);

	/* PAGE_CNT = 1 */
	nfc_set_field(mtd, NFC_FLASH_CONFIG, CONFIG_PAGE_CNT_MASK,
			CONFIG_PAGE_CNT_SHIFT, 0x1);
}

/* Control chips select signal on m54418twr board */
static void
m54418twr_select_chip(struct mtd_info *mtd, int chip)
{
#ifdef CONFIG_M68K
	if (chip < 0) {
		MCF_GPIO_PAR_FBCTL &= (MCF_GPIO_PAR_FBCTL_ALE_MASK &
				   MCF_GPIO_PAR_FBCTL_TA_MASK);
		MCF_GPIO_PAR_FBCTL |= MCF_GPIO_PAR_FBCTL_ALE_FB_TS |
				   MCF_GPIO_PAR_FBCTL_TA_TA;

		MCF_GPIO_PAR_BE =
		    MCF_GPIO_PAR_BE_BE3_BE3 | MCF_GPIO_PAR_BE_BE2_BE2 |
		    MCF_GPIO_PAR_BE_BE1_BE1 | MCF_GPIO_PAR_BE_BE0_BE0;

		MCF_GPIO_PAR_CS &= ~MCF_GPIO_PAR_CS_CS1_NFC_CE;
		MCF_GPIO_PAR_CS = MCF_GPIO_PAR_CS_CS0_CS0;
		return;
	}

	MCF_GPIO_PAR_FBCTL &= (MCF_GPIO_PAR_FBCTL_ALE_MASK &
			MCF_GPIO_PAR_FBCTL_TA_MASK);
	MCF_GPIO_PAR_FBCTL |= MCF_GPIO_PAR_FBCTL_ALE_FB_ALE |
			MCF_GPIO_PAR_FBCTL_TA_NFC_RB;
	MCF_GPIO_PAR_BE = MCF_GPIO_PAR_BE_BE3_FB_A1 | MCF_GPIO_PAR_BE_BE2_FB_A0 |
		MCF_GPIO_PAR_BE_BE1_BE1 | MCF_GPIO_PAR_BE_BE0_BE0;

	MCF_GPIO_PAR_CS &= (MCF_GPIO_PAR_BE_BE3_MASK & MCF_GPIO_PAR_BE_BE2_MASK);
	MCF_GPIO_PAR_CS = MCF_GPIO_PAR_CS_CS1_NFC_CE;
	return;
#endif /* CONFIG_M68K */
}

/* Read NAND Ready/Busy signal */
static int
fsl_nfc_dev_ready(struct mtd_info *mtd)
{
	/*
	 * NFC handles ready/busy signal internally. Therefore, this function
	 * always returns status as ready.
	 */
	return 1;
}

/* Write command to NAND flash */
static void
fsl_nfc_command(struct mtd_info *mtd, unsigned command,
					int column, int page)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	prv->column = (column >= 0) ? column : 0;
	prv->spareonly = 0;
	get_id = 0;
	get_status = 0;

	if (page != -1) prv->page = page;

	nfc_set_field(mtd, NFC_FLASH_CONFIG,
		CONFIG_ECC_MODE_MASK,
		CONFIG_ECC_MODE_SHIFT, ECC_45_BYTE);

	if (!(page%0x40)) {
			nfc_set_field(mtd, NFC_FLASH_CONFIG,
				CONFIG_ECC_MODE_MASK,
				CONFIG_ECC_MODE_SHIFT, ECC_BYPASS);
	}

	switch (command) {
	case NAND_CMD_PAGEPROG:
		if (!(prv->page%0x40))
			nfc_set_field(mtd, NFC_FLASH_CONFIG,
				CONFIG_ECC_MODE_MASK,
				CONFIG_ECC_MODE_SHIFT, ECC_BYPASS);

		fsl_nfc_send_cmd(mtd,
				PROGRAM_PAGE_CMD_BYTE1,
				PROGRAM_PAGE_CMD_BYTE2,
				PROGRAM_PAGE_CMD_CODE);
		break;
	/*
	 * NFC does not support sub-page reads and writes,
	 * so emulate them using full page transfers.
	 */
	case NAND_CMD_READ0:
		column = 0;
		goto read0;
		break;

	case NAND_CMD_READ1:
		prv->column += 256;
		command = NAND_CMD_READ0;
		column = 0;
		goto read0;
		break;

	case NAND_CMD_READOOB:
		prv->spareonly = 1;
		command = NAND_CMD_READ0;
		column = 0;
read0:
		fsl_nfc_send_cmd(mtd,
				PAGE_READ_CMD_BYTE1,
				PAGE_READ_CMD_BYTE2,
				READ_PAGE_CMD_CODE);
		break;

	case NAND_CMD_SEQIN:
		fsl_nfc_command(mtd, NAND_CMD_READ0, column, page);
		column = 0;
		break;

	case NAND_CMD_ERASE1:
		fsl_nfc_send_cmd(mtd,
				ERASE_CMD_BYTE1,
				ERASE_CMD_BYTE2,
				ERASE_CMD_CODE);
		break;
	case NAND_CMD_ERASE2:
		return;
	case NAND_CMD_READID:
		get_id = 1;
		fsl_nfc_send_one_byte(mtd, command, READ_ID_CMD_CODE);
		break;
	case NAND_CMD_STATUS:
		get_status = 1;
		fsl_nfc_send_one_byte(mtd, command, STATUS_READ_CMD_CODE);
		break;
	case NAND_CMD_RNDOUT:
		fsl_nfc_send_cmd(mtd,
				RANDOM_OUT_CMD_BYTE1,
				RANDOM_OUT_CMD_BYTE2,
				RANDOM_OUT_CMD_CODE);
		break;
	case NAND_CMD_RESET:
		fsl_nfc_send_one_byte(mtd, command, RESET_CMD_CODE);
		break;
	default:
		return;
	}

	fsl_nfc_addr_cycle(mtd, column, page);

	fsl_nfc_done(mtd);
}

/* Copy data from/to NFC spare buffers. */
static void
fsl_nfc_copy_spare(struct mtd_info *mtd, uint offset,
			u8 *buffer, uint size, int wr)
{
	struct nand_chip *nand = mtd->priv;
	struct fsl_nfc_prv *prv = nand->priv;
	uint o, s, sbsize, blksize;

	/*
	 * NAND spare area is available through NFC spare buffers.
	 * The NFC divides spare area into (page_size / 512) chunks.
	 * Each chunk is placed into separate spare memory area, using
	 * first (spare_size / num_of_chunks) bytes of the buffer.
	 *
	 * For NAND device in which the spare area is not divided fully
	 * by the number of chunks, number of used bytes in each spare
	 * buffer is rounded down to the nearest even number of bytes,
	 * and all remaining bytes are added to the last used spare area.
	 *
	 * For more information read section 26.6.10 of MPC5121e
	 * Microcontroller Reference Manual, Rev. 3.
	 */

	/* Calculate number of valid bytes in each spare buffer */
/*	sbsize = (mtd->oobsize / (mtd->writesize / 512)) & ~1;*/
	sbsize = (mtd->oobsize / (mtd->writesize / 2048)) & ~1;


	while (size) {
		/* Calculate spare buffer number */
		s = offset / sbsize;
		if (s > NFC_SPARE_BUFFERS - 1)
			s = NFC_SPARE_BUFFERS - 1;

		/*
		 * Calculate offset to requested data block in selected spare
		 * buffer and its size.
		 */
		o = offset - (s * sbsize);
		blksize = min(sbsize - o, size);

		if (wr)
			memcpy_toio(prv->regs + NFC_SPARE_AREA(s) + o,
							buffer, blksize);
		else {
			memcpy_fromio(buffer,
				prv->regs + NFC_SPARE_AREA(s) + o, blksize);
		}

		buffer += blksize;
		offset += blksize;
		size -= blksize;
	};
}

/* Copy data from/to NFC main and spare buffers */
static void
fsl_nfc_buf_copy(struct mtd_info *mtd, u_char *buf, int len, int wr)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;
	uint c = prv->column;
	uint l;

	/* Handle spare area access */
	if (prv->spareonly || c >= mtd->writesize) {
		/* Calculate offset from beginning of spare area */
		if (c >= mtd->writesize)
			c -= mtd->writesize;

		prv->column += len;
		fsl_nfc_copy_spare(mtd, c, buf, len, wr);
		return;
	}

	/*
	 * Handle main area access - limit copy length to prevent
	 * crossing main/spare boundary.
	 */
	l = min((uint)len, mtd->writesize - c);
	prv->column += l;

	if (wr)
		memcpy_toio(prv->regs + NFC_MAIN_AREA(0) + c, buf, l);
	else {
		if (get_status) {
			get_status = 0;
			*buf = fsl_nfc_get_status(mtd);
		} else if (l == 1 && c <= 3 && get_id) {
			*buf = fsl_nfc_get_id(mtd, c);
		} else
			memcpy_fromio(buf, prv->regs + NFC_MAIN_AREA(0) + c, l);
	}

	/* Handle crossing main/spare boundary */
	if (l != len) {
		buf += l;
		len -= l;
		fsl_nfc_buf_copy(mtd, buf, len, wr);
	}
}

/* Read data from NFC buffers */
static void
fsl_nfc_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	fsl_nfc_buf_copy(mtd, buf, len, 0);
}

/* Write data to NFC buffers */
static void
fsl_nfc_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	fsl_nfc_buf_copy(mtd, (u_char *)buf, len, 1);
}

/* Compare buffer with NAND flash */
static int
fsl_nfc_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	u_char tmp[256];
	uint bsize;

	while (len) {
		bsize = min(len, 256);
		fsl_nfc_read_buf(mtd, tmp, bsize);

		if (memcmp(buf, tmp, bsize))
			return 1;

		buf += bsize;
		len -= bsize;
	}

	return 0;
}

/* Read byte from NFC buffers */
static u8
fsl_nfc_read_byte(struct mtd_info *mtd)
{
	u8 tmp;
	fsl_nfc_read_buf(mtd, &tmp, sizeof(tmp));
	return tmp;
}

/* Read word from NFC buffers */
static u16
fsl_nfc_read_word(struct mtd_info *mtd)
{
	u16 tmp;
	fsl_nfc_read_buf(mtd, (u_char *)&tmp, sizeof(tmp));
	return tmp;
}

static void
copy_from_to_spare(struct mtd_info *mtd, void *pbuf, int len, int wr)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;
	int i, copy_count, copy_size;

/*	copy_count = mtd->writesize / 512;*/
	copy_count = mtd->writesize / 2048;
	/*
	 * Each spare area has 16 bytes for 512, 2K and normal 4K nand.
	 * For 4K nand with large 218 byte spare size, the size is 26 bytes for
	 * the first 7 buffers and 36 for the last.
	 */
/*	copy_size = mtd->oobsize == 218 ? 26 : 16;*/
	copy_size = 64;

	/*
	 * Each spare area has 16 bytes for 512, 2K and normal 4K nand.
	 * For 4K nand with large 218 byte spare size, the size is 26
	 * bytes for the first 7 buffers and 36 for the last.
	 */
	for (i = 0; i < copy_count - 1 && len > 0; i++) {
		if (wr)
			memcpy_toio(prv->regs + NFC_SPARE_AREA(i),
					pbuf, MIN(len, copy_size));
		else
			memcpy_fromio(pbuf, prv->regs + NFC_SPARE_AREA(i),
					MIN(len, copy_size));
		pbuf += copy_size;
		len -= copy_size;
	}
	if (len > 0) {
		if (wr)
			memcpy_toio(prv->regs + NFC_SPARE_AREA(i),
				pbuf, len);
		else
			memcpy_fromio(pbuf,
				prv->regs + NFC_SPARE_AREA(i), len);
	}
}


static int fsl_nfc_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
				int page, int sndcmd)
{
	fsl_nfc_command(mtd, NAND_CMD_READ0, 0, page);

	copy_from_to_spare(mtd, chip->oob_poi, mtd->oobsize, 0);
	return 0;
}

static int fsl_nfc_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
					int page)
{
	fsl_nfc_command(mtd, NAND_CMD_READ0, 0, page);
	/* copy the oob data */
	copy_from_to_spare(mtd, chip->oob_poi, mtd->oobsize, 1);
	fsl_nfc_command(mtd, NAND_CMD_PAGEPROG, 0, page);
	return 0;
}

static int fsl_nfc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
					uint8_t *buf, int page)
{
	struct fsl_nfc_prv *prv = chip->priv;

	memcpy_fromio((void *)buf, prv->regs + NFC_MAIN_AREA(0),
			mtd->writesize);
	copy_from_to_spare(mtd, chip->oob_poi, mtd->oobsize, 0);
	return 0;
}

static void fsl_nfc_write_page(struct mtd_info *mtd,
		struct nand_chip *chip, const uint8_t *buf)
{
	struct fsl_nfc_prv *prv = chip->priv;
	memcpy_toio(prv->regs + NFC_MAIN_AREA(0), buf, mtd->writesize);
	copy_from_to_spare(mtd, chip->oob_poi, mtd->oobsize, 1);
}

static void fsl_nfc_enable_hwecc(struct mtd_info *mtd, int mode)
{
	return;
}

/* Free driver resources */
static void
fsl_nfc_free(struct  platform_device *dev, struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	kfree(prv);
}

static int __init
fsl_nfc_probe(struct platform_device *pdev)
{
	struct fsl_nfc_prv *prv;
	struct resource *res;
	struct mtd_info *mtd;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *parts;
	struct fsl_nfc_nand_platform_data *pdata = pdev->dev.platform_data;
#endif
	struct nand_chip *chip;
	unsigned long regs_paddr, regs_size;
	int retval = 0;

	prv = kzalloc(sizeof(*prv), GFP_KERNEL);
	if (!prv) {
		printk(KERN_ERR DRV_NAME ": Memory exhausted!\n");
		return -ENOMEM;
	}
	mtd = &prv->mtd;
	chip = &prv->chip;

	mtd->priv = chip;
	chip->priv = prv;

	prv->irq = platform_get_irq(pdev, 0);
	if (prv->irq <= 0) {
		printk(KERN_ERR DRV_NAME ": Error mapping IRQ!\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "%s platform_get_resource MEM  failed %x\n",
			__func__, (unsigned int)res);
		retval = -ENOMEM;
		goto error;
	}
	regs_paddr = res->start;
	regs_size = res->end - res->start + 1;

#if 0
	if (!request_mem_region(regs_paddr, regs_size, DRV_NAME)) {
		printk(KERN_ERR DRV_NAME ": Error requesting memory region!\n");
		return -EBUSY;
	}

	prv->regs = ioremap(regs_paddr, regs_size);
#endif
	prv->regs = (void __iomem *)regs_paddr;
	if (!prv->regs) {
		printk(KERN_ERR DRV_NAME ": Error mapping memory region!\n");
		return -ENOMEM;
	}

	mtd->name = "NAND";
	mtd->writesize = 2048;
	mtd->oobsize = 64;

	chip->dev_ready = fsl_nfc_dev_ready;
	chip->cmdfunc = fsl_nfc_command;
	chip->read_byte = fsl_nfc_read_byte;
	chip->read_word = fsl_nfc_read_word;
	chip->read_buf = fsl_nfc_read_buf;
	chip->write_buf = fsl_nfc_write_buf;
	chip->verify_buf = fsl_nfc_verify_buf;
	chip->options = NAND_NO_AUTOINCR | NAND_USE_FLASH_BBT | NAND_BUSWIDTH_16 | NAND_CACHEPRG;

	chip->select_chip = m54418twr_select_chip;

	if (hardware_ecc) {
		chip->ecc.read_page = fsl_nfc_read_page;
		chip->ecc.write_page = fsl_nfc_write_page;
		chip->ecc.read_oob = fsl_nfc_read_oob;
		chip->ecc.write_oob = fsl_nfc_write_oob;
		chip->ecc.layout = &fsl_nfc_ecc45;

		/* propagate ecc.layout to mtd_info */
		mtd->ecclayout = chip->ecc.layout;
		chip->ecc.calculate = NULL;
		chip->ecc.hwctl = fsl_nfc_enable_hwecc;
		chip->ecc.correct = NULL;
		chip->ecc.mode = NAND_ECC_HW;
		/* RS-ECC is applied for both MAIN+SPARE not MAIN alone */
		chip->ecc.steps = 1;
		chip->ecc.bytes = 45;
		chip->ecc.size = 0x800;

		/* set ECC mode = ECC_45_BYTE */
		nfc_set_field(mtd, NFC_FLASH_CONFIG,
				CONFIG_ECC_MODE_MASK,
				CONFIG_ECC_MODE_SHIFT, ECC_45_BYTE);
		/* set ECC_STATUS write position */
		nfc_set_field(mtd, NFC_FLASH_CONFIG,
				CONFIG_ECC_SRAM_ADDR_MASK,
				CONFIG_ECC_SRAM_ADDR_SHIFT, ECC_SRAM_ADDR);
		/* enable ECC_STATUS results write */
		nfc_set_field(mtd, NFC_FLASH_CONFIG,
				CONFIG_ECC_SRAM_REQ_MASK,
				CONFIG_ECC_SRAM_REQ_SHIFT, 1);
	} else {
		chip->ecc.mode = NAND_ECC_SOFT;
		/* set ECC BY_PASS */
		nfc_set_field(mtd, NFC_FLASH_CONFIG,
				CONFIG_ECC_MODE_MASK,
				CONFIG_ECC_MODE_SHIFT, ECC_BYPASS);
	}
	chip->bbt_td = &bbt_main_descr;
	chip->bbt_md = &bbt_mirror_descr;
	bbt_main_descr.pattern = bbt_pattern;
	bbt_mirror_descr.pattern = mirror_pattern;

	init_waitqueue_head(&prv->irq_waitq);
	retval = request_irq(prv->irq, fsl_nfc_irq, IRQF_DISABLED, DRV_NAME, mtd);
	if (retval) {
		printk(KERN_ERR DRV_NAME ": Error requesting IRQ!\n");
		goto error;
	}

	/* SET SECTOR SIZE */
	nfc_write(mtd, NFC_SECTOR_SIZE, PAGE_2K | PAGE_64);

	nfc_set_field(mtd, NFC_FLASH_CONFIG,
			CONFIG_ADDR_AUTO_INCR_MASK,
			CONFIG_ADDR_AUTO_INCR_SHIFT, 0);

	nfc_set_field(mtd, NFC_FLASH_CONFIG,
			CONFIG_BUFNO_AUTO_INCR_MASK,
			CONFIG_BUFNO_AUTO_INCR_SHIFT, 0);
	/* SET FAST_FLASH = 1 */
#if 0
	nfc_set_field(mtd, NFC_FLASH_CONFIG,
			CONFIG_FAST_FLASH_MASK,
			CONFIG_FAST_FLASH_SHIFT, 1);
#endif

	nfc_set_field(mtd, NFC_FLASH_CONFIG,
			CONFIG_16BIT_MASK,
			CONFIG_16BIT_SHIFT, 1);


	/* Detect NAND chips */
	if (nand_scan(mtd, 1)) {
		printk(KERN_ERR DRV_NAME ": NAND Flash not found !\n");
		free_irq(prv->irq, mtd);
		retval = -ENXIO;
		goto error;
	}

	platform_set_drvdata(pdev, mtd);

	/* Register device in MTD */
#ifdef CONFIG_MTD_PARTITIONS
	retval = parse_mtd_partitions(mtd, fsl_nfc_pprobes, &parts, 0);
	if (retval < 0) {
		printk(KERN_ERR DRV_NAME ": Error parsing MTD partitions!\n");
		free_irq(prv->irq, mtd);
		retval = -EINVAL;
		goto error;
	}

	if (retval > 0)
		retval = add_mtd_partitions(mtd, parts, retval);
	else if (pdata && pdata->nr_parts) {
		retval = add_mtd_partitions(
			mtd, pdata->parts, pdata->nr_parts);
	} else
#endif
		retval = add_mtd_device(mtd);

	if (retval) {
		printk(KERN_ERR DRV_NAME ": Error adding MTD device!\n");
		free_irq(prv->irq, mtd);
		goto error;
	}

	return 0;
error:
	fsl_nfc_free(pdev, mtd);
	return retval;
}

static int __exit
fsl_nfc_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct nand_chip *chip = mtd->priv;
	struct fsl_nfc_prv *prv = chip->priv;

	nand_release(mtd);
	free_irq(prv->irq, mtd);
	fsl_nfc_free(pdev, mtd);

	return 0;
}

static struct platform_driver fsl_nfc_driver = {
	.probe		= fsl_nfc_probe,
	.remove		= __exit_p(fsl_nfc_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init fsl_nfc_init(void)
{
	pr_info("FSL NFC MTD nand Driver %s\n", DRV_VERSION);
	if (platform_driver_register(&fsl_nfc_driver) != 0) {
		printk(KERN_ERR DRV_NAME ": Driver register failed!\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit fsl_nfc_cleanup(void)
{
	platform_driver_unregister(&fsl_nfc_driver);
}

module_init(fsl_nfc_init);
module_exit(fsl_nfc_cleanup);

MODULE_AUTHOR("Alexander Potashev <aspotashev@emcraft.com>");
MODULE_DESCRIPTION("FSL NFC NAND MTD driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
