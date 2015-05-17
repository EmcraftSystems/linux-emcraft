/*
 * MTD SPIFI driver for ST M25Pxx (and similar) driven over SPIFI LPC43XX block
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 *
 * Copyright (c) 2005, Intec Automation Inc.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/sched.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>

#include <mach/spifi.h>
#include <asm/io.h>

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_DOR		0x3b	/* Dual read */
#define	OPCODE_QOR		0x6b	/* Quad read */
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */

/* Used for SST flashes only. */
#define	OPCODE_BP		0x02	/* Byte program */
#define	OPCODE_WRDI		0x04	/* Write disable */
#define	OPCODE_AAI_WP		0xad	/* Auto address increment word program */

/* Used for Spansion flashes only. */
#define OPCODE_BRWR             0x17    /* Bank register write */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(40 * HZ)	/* M25P16 specs 40s max chip erase */
#define	MAX_CMD_SIZE		4

/****************************************************************************/

struct m25p_spifi {
	struct platform_device	*pdev;
	struct mutex		lock;
	spinlock_t		spin;
	struct mtd_info		mtd;
	unsigned		partitioned:1;
	u16			page_size;
	u16			addr_width;
	u32			read_cmd;
	u32			pp_cmd;

	u8			erase_opcode;

	u8			spifi_frameform;
	u8			spifi_flags;
#define	SPIFI_WEN_EACH		(1 << 0)

	void	__iomem		*regs;
	void	__iomem		*mem;
};

static inline struct m25p_spifi *mtd_to_m25p(struct mtd_info *mtd)
{
	return container_of(mtd, struct m25p_spifi, mtd);
}

/****************************************************************************/

#define WAIT_UNTIL(condition, timeout) do {		\
	unsigned long deadline;				\
							\
	ret = -ETIME;					\
	deadline = jiffies + timeout;			\
							\
	do {						\
		if (!(condition)) {			\
			ret = 0;			\
			break;				\
		}					\
	} while (!time_after_eq(jiffies, deadline));	\
} while(0);


static int spifi_wait_cmd(struct m25p_spifi *flash)
{
	int ret;
#define SPIFI_CMD_STAT	(1 << LPC18XX_SPIFI_STAT_CMD_OFF)

	WAIT_UNTIL(
		readl(flash->regs + LPC18XX_SPIFI_STAT_OFFSET) & SPIFI_CMD_STAT,
		HZ);

	return ret;
}

/* This is generally bus-specific commands */
static int spifi_opcode_read(struct m25p_spifi *flash, u8 opcode, u8 *res, size_t len)
{
	spin_lock(&flash->spin);

	writel(SPIFI_CMD_OPCODE(opcode) |
		SPIFI_CMD_DATALEN(len) |
		SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
		SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP),

		flash->regs + LPC18XX_SPIFI_CMD_OFFSET);

	while (res && len) {
		*res = readb(flash->regs + LPC18XX_SPIFI_DATA_OFFSET);
		++res;
		--len;
	}

	spifi_wait_cmd(flash);

	spin_unlock(&flash->spin);

	return 0;
}

static inline int spifi_opcode(struct m25p_spifi *flash, u8 opcode)
{
	return spifi_opcode_read(flash, opcode, NULL, 0);
}

static int spifi_program(struct m25p_spifi *flash, loff_t to, const u_char *buf, size_t len)
{
	spin_lock(&flash->spin);

	writel(to, flash->regs + LPC18XX_SPIFI_ADDR_OFFSET);

	writel(flash->pp_cmd | SPIFI_CMD_DATALEN(len),
		flash->regs + LPC18XX_SPIFI_CMD_OFFSET);


	while (len) {
		writeb(*buf, flash->regs + LPC18XX_SPIFI_DATA_OFFSET);
		++buf;
		--len;
	}

	spifi_wait_cmd(flash);

	spin_unlock(&flash->spin);

	return 0;
}

static int spifi_opcode_write(struct m25p_spifi *flash, u8 opcode, u8 *val, size_t len)
{
	spin_lock(&flash->spin);

	writel(SPIFI_CMD_OPCODE(opcode) |
		SPIFI_CMD_DATALEN(len) |
		SPIFI_CMD_DOUT(1) |
		SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
		SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP),

		flash->regs + LPC18XX_SPIFI_CMD_OFFSET);

	while (len) {
		writeb(*val, flash->regs + LPC18XX_SPIFI_DATA_OFFSET);
		++val;
		--len;
	}

	spifi_wait_cmd(flash);

	spin_unlock(&flash->spin);

	return 0;
}


static int spifi_opcode_addr(struct m25p_spifi *flash, u8 opcode, unsigned long addr)
{
	spin_lock(&flash->spin);

	writel(addr, flash->regs + LPC18XX_SPIFI_ADDR_OFFSET);

	writel(SPIFI_CMD_OPCODE(opcode) |
		SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
		SPIFI_CMD_FRAMEFORM(flash->spifi_frameform),

		flash->regs + LPC18XX_SPIFI_CMD_OFFSET);

	spifi_wait_cmd(flash);

	spin_unlock(&flash->spin);

	return 0;
}

static int spifi_reset(struct m25p_spifi *flash)
{
	int ret;

	spin_lock(&flash->spin);

	writel(SPIFI_RESET, flash->regs + LPC18XX_SPIFI_STAT_OFFSET);

	WAIT_UNTIL(readl(flash->regs + LPC18XX_SPIFI_STAT_OFFSET) & SPIFI_RESET, HZ);

	spin_unlock(&flash->spin);

	return ret;
}

static int spifi_memory_mode_off(struct m25p_spifi *flash)
{
	unsigned long status;

	spin_lock(&flash->spin);

	status = readl(flash->regs + LPC18XX_SPIFI_STAT_OFFSET);
	/* Already NOT in memory mode */
	if (!(status & LPC18XX_SPIFI_STAT_MCINIT_MSK))
		return 0;

	spifi_reset(flash);

	spin_unlock(&flash->spin);

	return 0;
}

static int spifi_memory_mode(struct m25p_spifi *flash, void __iomem** mem)
{
	unsigned long status;
	int ret = 0;

	spin_lock(&flash->spin);

	status = readl(flash->regs + LPC18XX_SPIFI_STAT_OFFSET);

	/* Already in memory mode */
	if (status & LPC18XX_SPIFI_STAT_MCINIT_MSK) {
		goto pointer;
	}

	/* Switch to memory mode */
	writel(flash->read_cmd,
		flash->regs + LPC18XX_SPIFI_MCMD_OFFSET);

	/* Wait until MCINIT is set */
	WAIT_UNTIL(!(readl(flash->regs + LPC18XX_SPIFI_STAT_OFFSET) &
			LPC18XX_SPIFI_STAT_MCINIT_MSK), HZ);

pointer:
	if (mem)
		*mem = flash->mem;

	spin_unlock(&flash->spin);

	return ret;
}


/*
 * Internal helper functions
 */

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct m25p_spifi *flash)
{
	ssize_t retval;
	u8 code = OPCODE_RDSR;
	u8 val;

	retval = spifi_opcode_read(flash, code, &val, 1);

	if (retval < 0) {
		dev_err(&flash->pdev->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct m25p_spifi *flash, u8 val)
{
	return spifi_opcode_write(flash, OPCODE_WRSR, &val, 1);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct m25p_spifi *flash)
{
	return spifi_opcode(flash, OPCODE_WREN);
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct m25p_spifi *flash)
{
	return spifi_opcode(flash, OPCODE_WRDI);
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct m25p_spifi *flash)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((sr = read_sr(flash)) < 0)
			break;
		if (!(sr & SR_WIP))
			return 0;

		cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	return 1;
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct m25p_spifi *flash)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %lldKiB\n",
	      dev_name(&flash->pdev->dev), __func__,
	      (long long)(flash->mtd.size >> 10));

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	spifi_opcode(flash, OPCODE_CHIP_ERASE);

	return 0;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct m25p_spifi *flash, u32 offset)
{
	DEBUG(MTD_DEBUG_LEVEL3, "%s: %s %dKiB at 0x%08x\n",
			dev_name(&flash->pdev->dev), __func__,
			flash->mtd.erasesize / 1024, offset);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(flash);

	spifi_opcode_addr(flash, flash->erase_opcode, offset);

	return 0;
}

/****************************************************************************/

/*
 * MTD implementation
 */

/*
 * Erase an address range on the flash chip.  The address range may extend
 * one or more erase sectors.  Return an error is there is a problem erasing.
 */
static int m25p80_spifi_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct m25p_spifi *flash = mtd_to_m25p(mtd);
	u32 addr,len;
	uint32_t rem;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%llx, len %lld\n",
	      dev_name(&flash->pdev->dev), __func__, "at",
	      (long long)instr->addr, (long long)instr->len);

	/* sanity checks */
	if (instr->addr + instr->len > flash->mtd.size)
		return -EINVAL;
	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&flash->lock);

	/* whole-chip erase? */
	if (len == flash->mtd.size) {
		if (erase_chip(flash)) {
			instr->state = MTD_ERASE_FAILED;
			mutex_unlock(&flash->lock);
			return -EIO;
		}

	/* REVISIT in some cases we could speed up erasing large regions
	 * by using OPCODE_SE instead of OPCODE_BE_4K.  We may have set up
	 * to use "small sector erase", but that's not always optimal.
	 */

	/* "sector"-at-a-time erase */
	} else {
		while (len) {
			if (erase_sector(flash, addr)) {
				instr->state = MTD_ERASE_FAILED;
				mutex_unlock(&flash->lock);
				return -EIO;
			}

			addr += mtd->erasesize;
			len -= mtd->erasesize;
		}
	}

	mutex_unlock(&flash->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/*
 * Read an address range from the flash chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int m25p80_spifi_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct m25p_spifi *flash = mtd_to_m25p(mtd);
	int ret;
	void __iomem *mem;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->pdev->dev), __func__, "from",
			(u32)from, len);

	/* sanity checks */
	if (!len)
		return 0;

	if (from + len > flash->mtd.size)
		return -EINVAL;

	/* Byte count starts at zero. */
	if (retlen)
		*retlen = 0;

	mutex_lock(&flash->lock);

       /* Wait till previous write/erase is done. */
	if (wait_till_ready(flash)) {
		dev_err(&flash->pdev->dev, "Flash busy\n");
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = spifi_memory_mode(flash, &mem);
	if (ret) {
		dev_err(&flash->pdev->dev, "Unable to prepare SPIFI for memory mode: %d\n", ret);
		goto out_unlock;
	}

	memcpy_fromio(buf, mem + from, len);

	*retlen = len;

out_unlock:
	mutex_unlock(&flash->lock);

	return ret;
}

/*
 * Write an address range to the flash chip.  Data must be written in
 * FLASH_PAGESIZE chunks.  The address range may be any size provided
 * it is within the physical boundaries.
 */
static int m25p80_spifi_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	struct m25p_spifi *flash = mtd_to_m25p(mtd);
	u32 page_offset, page_size;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%08x, len %zd\n",
			dev_name(&flash->pdev->dev), __func__, "to",
			(u32)to, len);

	if (retlen)
		*retlen = 0;

	/* sanity checks */
	if (!len)
		return(0);

	if (to + len > flash->mtd.size)
		return -EINVAL;

	mutex_lock(&flash->lock);

	spifi_memory_mode_off(flash);

	/* Wait until finished previous write command. */
	if (wait_till_ready(flash)) {
		mutex_unlock(&flash->lock);
		return 1;
	}

	write_enable(flash);

	page_offset = to & (flash->page_size - 1);

	/* do all the bytes fit onto one page? */
	if (page_offset + len <= flash->page_size) {
		spifi_program(flash, to, buf, len);

		if (retlen)
			*retlen = len;
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		page_size = flash->page_size - page_offset;

		spifi_program(flash, to, buf, page_size);

		if (retlen)
			*retlen = page_size;

		/* write everything in flash->page_size chunks */
		for (i = page_size; i < len; i += page_size) {
			page_size = len - i;
			if (page_size > flash->page_size)
				page_size = flash->page_size;

			wait_till_ready(flash);

			if (flash->spifi_flags & SPIFI_WEN_EACH)
				write_enable(flash);

			spifi_program(flash, to + i, buf + i, page_size);

			if (retlen)
				*retlen += page_size;
		}
	}

	mutex_unlock(&flash->lock);

	return 0;
}

static int sst_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	BUG_ON(1);
	return 0;
}

/****************************************************************************/

/*
 * SPI device driver setup and teardown
 */

struct flash_info {
	/* JEDEC id zero means "no ID" (most older chips); otherwise it has
	 * a high byte of zero plus three data bytes: the manufacturer id,
	 * then a two byte device id.
	 */
	u32		jedec_id;
	u16             ext_id;

	/* The size listed here is what works with OPCODE_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned	sector_size;
	u16		n_sectors;

	u16		page_size;
	u16		addr_width;

	u16		flags;
#define	SECT_4K		0x01		/* OPCODE_BE_4K works uniformly */
#define	M25P_NO_ERASE	0x02		/* No erase command needed */
#define	M25P_QUAD	0x04		/* Can do QuadSPI using 0x6B command */
#define	M25P_WEN_EACH	0x08		/* Write enable after each */
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.jedec_id = (_jedec_id),				\
		.ext_id = (_ext_id),					\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.addr_width = 3,					\
		.flags = (_flags),					\
	})

#define CAT25_INFO(_sector_size, _n_sectors, _page_size, _addr_width)	\
	((kernel_ulong_t)&(struct flash_info) {				\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = (_page_size),				\
		.addr_width = (_addr_width),				\
		.flags = M25P_NO_ERASE,					\
	})

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static struct platform_device_id m25p_spifi_ids[] = {
	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{ "at25fs010",  INFO(0x1f6601, 0, 32 * 1024,   4, SECT_4K) },
	{ "at25fs040",  INFO(0x1f6604, 0, 64 * 1024,   8, SECT_4K) },

	{ "at25df041a", INFO(0x1f4401, 0, 64 * 1024,   8, SECT_4K) },
	{ "at25df641",  INFO(0x1f4800, 0, 64 * 1024, 128, SECT_4K) },

	{ "at26f004",   INFO(0x1f0400, 0, 64 * 1024,  8, SECT_4K) },
	{ "at26df081a", INFO(0x1f4501, 0, 64 * 1024, 16, SECT_4K) },
	{ "at26df161a", INFO(0x1f4601, 0, 64 * 1024, 32, SECT_4K) },
	{ "at26df321",  INFO(0x1f4701, 0, 64 * 1024, 64, SECT_4K) },

	/* Macronix */
	{ "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8, SECT_4K) },
	{ "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64, 0) },
	{ "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128, 0) },
	{ "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256, 0) },
	{ "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256, 0) },

	/* Spansion -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{ "s25sl004a",  INFO(0x010212,      0,  64 * 1024,   8, 0) },
	{ "s25sl008a",  INFO(0x010213,      0,  64 * 1024,  16, 0) },
	{ "s25sl016a",  INFO(0x010214,      0,  64 * 1024,  32, 0) },
	{ "s25sl032a",  INFO(0x010215,      0,  64 * 1024,  64, 0) },
	{ "s25sl064a",  INFO(0x010216,      0,  64 * 1024, 128, 0) },
	{ "s25sl12800", INFO(0x012018, 0x0300, 256 * 1024,  64, 0) },
	{ "s25sl12801", INFO(0x012018, 0x0301,  64 * 1024, 256, 0) },
	{ "s25fl129p0", INFO(0x012018, 0x4d00, 256 * 1024,  64, 0) },
	{ "s25fl129p1", INFO(0x012018, 0x4d01,  64 * 1024, 256, 0) },
	{ "s25fl256s1", INFO(0x010219, 0x4d01,  64 * 1024, 512, M25P_QUAD | M25P_WEN_EACH) },
	{ "s25fl008k",  INFO(0xef4014,      0,  64 * 1024,  16, SECT_4K) },
	{ "s25fl016k",  INFO(0xef4015,      0,  64 * 1024,  32, SECT_4K) },
	{ "s25fl064k",  INFO(0xef4017,      0,  64 * 1024, 128, SECT_4K) },
	{ "s25fl132k",  INFO(0x014016,      0,  64 * 1024,  64, 0) },

	/* SST -- large erase sizes are "overlays", "sectors" are 4K */
	{ "sst25vf040b", INFO(0xbf258d, 0, 64 * 1024,  8, SECT_4K) },
	{ "sst25vf080b", INFO(0xbf258e, 0, 64 * 1024, 16, SECT_4K) },
	{ "sst25vf016b", INFO(0xbf2541, 0, 64 * 1024, 32, SECT_4K) },
	{ "sst25vf032b", INFO(0xbf254a, 0, 64 * 1024, 64, SECT_4K) },
	{ "sst25wf512",  INFO(0xbf2501, 0, 64 * 1024,  1, SECT_4K) },
	{ "sst25wf010",  INFO(0xbf2502, 0, 64 * 1024,  2, SECT_4K) },
	{ "sst25wf020",  INFO(0xbf2503, 0, 64 * 1024,  4, SECT_4K) },
	{ "sst25wf040",  INFO(0xbf2504, 0, 64 * 1024,  8, SECT_4K) },

	/* ST Microelectronics -- newer production may have feature updates */
	{ "m25p05",  INFO(0x202010,  0,  32 * 1024,   2, 0) },
	{ "m25p10",  INFO(0x202011,  0,  32 * 1024,   4, 0) },
	{ "m25p20",  INFO(0x202012,  0,  64 * 1024,   4, 0) },
	{ "m25p40",  INFO(0x202013,  0,  64 * 1024,   8, 0) },
	{ "m25p80",  INFO(0x202014,  0,  64 * 1024,  16, 0) },
	{ "m25p16",  INFO(0x202015,  0,  64 * 1024,  32, 0) },
	{ "m25p32",  INFO(0x202016,  0,  64 * 1024,  64, 0) },
	{ "m25p64",  INFO(0x202017,  0,  64 * 1024, 128, 0) },
	{ "m25p128", INFO(0x202018,  0, 256 * 1024,  64, 0) },

	{ "m45pe10", INFO(0x204011,  0, 64 * 1024,    2, 0) },
	{ "m45pe80", INFO(0x204014,  0, 64 * 1024,   16, 0) },
	{ "m45pe16", INFO(0x204015,  0, 64 * 1024,   32, 0) },

	{ "m25pe80", INFO(0x208014,  0, 64 * 1024, 16,       0) },
	{ "m25pe16", INFO(0x208015,  0, 64 * 1024, 32, SECT_4K) },

	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ "w25x10", INFO(0xef3011, 0, 64 * 1024,  2,  SECT_4K) },
	{ "w25x20", INFO(0xef3012, 0, 64 * 1024,  4,  SECT_4K) },
	{ "w25x40", INFO(0xef3013, 0, 64 * 1024,  8,  SECT_4K) },
	{ "w25x80", INFO(0xef3014, 0, 64 * 1024,  16, SECT_4K) },
	{ "w25x16", INFO(0xef3015, 0, 64 * 1024,  32, SECT_4K) },
	{ "w25x32", INFO(0xef3016, 0, 64 * 1024,  64, SECT_4K) },
	{ "w25x64", INFO(0xef3017, 0, 64 * 1024, 128, SECT_4K) },

	/* Catalyst / On Semiconductor -- non-JEDEC */
	{ "cat25c11", CAT25_INFO(  16, 8, 16, 1) },
	{ "cat25c03", CAT25_INFO(  32, 8, 16, 2) },
	{ "cat25c09", CAT25_INFO( 128, 8, 32, 2) },
	{ "cat25c17", CAT25_INFO( 256, 8, 32, 2) },
	{ "cat25128", CAT25_INFO(2048, 8, 64, 2) },
	{ },
};

void msleep(unsigned int msecs);

static const struct platform_device_id *jedec_probe(struct m25p_spifi *flash)
{
	int			tmp;
	u8			id[5];
	u32			jedec;
	u16                     ext_jedec;
	struct flash_info	*info;

	/* JEDEC also defines an optional "extended device information"
	 * string for after vendor-specific data, after the three bytes
	 * we use here.  Supporting some chips might require using it.
	 */
	tmp = spifi_opcode_read(flash, OPCODE_RDID, id, 5);
	if (tmp < 0) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: error %d reading JEDEC ID\n",
			dev_name(&flash->pdev->dev), tmp);
		return NULL;
	}
	jedec = id[0];
	jedec = jedec << 8;
	jedec |= id[1];
	jedec = jedec << 8;
	jedec |= id[2];

	/*
	 * Some chips (like Numonyx M25P80) have JEDEC and non-JEDEC variants,
	 * which depend on technology process. Officially RDID command doesn't
	 * exist for non-JEDEC chips, but for compatibility they return ID 0.
	 */
	if (jedec == 0)
		return NULL;

	ext_jedec = id[3] << 8 | id[4];

	//printk("jedec = %x, ext_jedec = %x\n", jedec, ext_jedec);

	for (tmp = 0; tmp < ARRAY_SIZE(m25p_spifi_ids) - 1; tmp++) {
		info = (void *)m25p_spifi_ids[tmp].driver_data;
		if (info->jedec_id == jedec) {
			if (info->ext_id != 0 && info->ext_id != ext_jedec)
				continue;
			return &m25p_spifi_ids[tmp];
		}
	}
	return NULL;
}

/*
 * Enable/disable QUAD mode
 */
static inline int set_quad(struct m25p_spifi *flash, int enable)
{
	u8	v[2];

	flash->read_cmd = SPIFI_CMD_OPCODE(OPCODE_QOR) |
		SPIFI_CMD_INTLEN(1) |
		SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_DUAL_DATA) |
		SPIFI_CMD_FRAMEFORM(flash->spifi_frameform);

	write_enable(flash);

	wait_till_ready(flash);

	v[0] = read_sr(flash);
	v[1] = (!!enable) << 1;
	spifi_opcode_write(flash, OPCODE_WRSR, v, 2);

	wait_till_ready(flash);

	return 0;
}

/*
 * Enable/disable 4-byte addressing mode.
 */
static inline int set_4byte(struct m25p_spifi *flash, u32 jedec_id, u8 enable)
{
	switch (jedec_id >> 16) {
	case 0x20: /* Micron, actually */
	case 0xC2:
	case 0xEF /* winbond */:
		dev_err(&flash->pdev->dev, "Unknown jedec_id: %x\n", jedec_id);
		return -ENOSYS;
	default:
		/* Spansion style */
		enable = (!!enable) << 7;
		return spifi_opcode_write(flash, OPCODE_BRWR, &enable, 1);
	}
}


/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int __devinit m25p_spifi_probe(struct platform_device *pdev)
{
	struct flash_platform_data	*data;
	struct m25p_spifi		*flash;
	struct flash_info		*info = NULL;
	unsigned			i;
	const struct platform_device_id	*id = NULL;
	struct resource			*res;
	void __iomem *mem;

	flash = kzalloc(sizeof *flash, GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "no memory base for SPIFI controller\n");
		kfree(flash);
		return -ENXIO;
	}

	flash->mem = (void __iomem*)res->start;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no register base for SPIFI controller\n");
		kfree(flash);
		return -ENXIO;
	}

	flash->regs = ioremap(res->start, resource_size(res));

	spifi_reset(flash);

	/* Platform data helps sort out which chip type we have, as
	 * well as how this board partitions it.  If we don't have
	 * a chip ID, try the JEDEC id commands; they'll work for most
	 * newer chips, even if we don't recognize the particular chip.
	 */
	data = pdev->dev.platform_data;
	if (data && data->type) {
		const struct platform_device_id *plat_id;

		for (i = 0; i < ARRAY_SIZE(m25p_spifi_ids) - 1; i++) {
			plat_id = &m25p_spifi_ids[i];
			if (strcmp(data->type, plat_id->name))
				continue;
			break;
		}

		if (plat_id)
			id = plat_id;
		else
			dev_warn(&pdev->dev, "unrecognized id %s\n", data->type);
	}

	flash->pdev = pdev;
	mutex_init(&flash->lock);
	spin_lock_init(&flash->spin);
	dev_set_drvdata(&pdev->dev, flash);

	writel(0, flash->regs + LPC18XX_SPIFI_CLIMIT_OFFSET);

	spifi_memory_mode_off(flash);

	/* Default read_cmd */
	flash->read_cmd = SPIFI_CMD_OPCODE(OPCODE_NORM_READ) |
		SPIFI_CMD_INTLEN(0) |
		SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
		SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS);

	/* Reset cache. Somehow this is required */
	if (spifi_memory_mode(flash, &mem)) {
		kfree(flash);
		return -ENXIO;
	}
	readb(mem); readb(mem + 1); readb(mem + 2);

	if (id)
		info = (void *)id->driver_data;

	if (!id || info->jedec_id) {
		const struct platform_device_id *jid;

		jid = jedec_probe(flash);
		if (!jid) {
			dev_info(&pdev->dev, "non-JEDEC variant of %s\n",
				 id->name);
		} else if (jid != id) {
			/*
			 * JEDEC knows better, so overwrite platform ID. We
			 * can't trust partitions any longer, but we'll let
			 * mtd apply them anyway, since some partitions may be
			 * marked read-only, and we don't want to lose that
			 * information, even if it's not 100% accurate.
			 */
			dev_warn(&pdev->dev, "found %s, expected %s\n",
				 jid->name, id->name);
			id = jid;
			info = (void *)jid->driver_data;
		}
	}

	/*
	 * Atmel and SST serial flash tend to power
	 * up with the software protection bits set
	 */

	if (info->jedec_id >> 16 == 0x1f ||
	    info->jedec_id >> 16 == 0xbf) {
		write_enable(flash);
		write_sr(flash, 0);
	}

	if (data && data->name)
		flash->mtd.name = data->name;
	else
		flash->mtd.name = dev_name(&pdev->dev);

	flash->mtd.type = MTD_NORFLASH;
	flash->mtd.writesize = 1;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd.flags = MTD_CAP_NORFLASH;
	flash->mtd.size = info->sector_size * info->n_sectors;
	flash->mtd.erase = m25p80_spifi_erase;
	flash->mtd.read = m25p80_spifi_read;

	flash->spifi_frameform = SPIFI_FRAMEFORM_OP_3ADDRESS;

	if (info->flags & M25P_WEN_EACH)
		flash->spifi_flags = SPIFI_WEN_EACH;

	/* 4 byte required */
	if (flash->mtd.size > 0x1000000) {
		flash->addr_width = 4;
		flash->spifi_frameform = SPIFI_FRAMEFORM_OP_4ADDRESS;
		set_4byte(flash, info->jedec_id, 1);
	}

	flash->read_cmd = SPIFI_CMD_OPCODE(OPCODE_FAST_READ) |
		SPIFI_CMD_INTLEN(1) |
		SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
		SPIFI_CMD_FRAMEFORM(flash->spifi_frameform);

	flash->pp_cmd = SPIFI_CMD_OPCODE(OPCODE_PP) |
		SPIFI_CMD_DOUT(1) |
		SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
		SPIFI_CMD_FRAMEFORM(flash->spifi_frameform);

	/* sst flash chips use AAI word program */
	if (info->jedec_id >> 16 == 0xbf)
		flash->mtd.write = sst_write;
	else
		flash->mtd.write = m25p80_spifi_write;

	/* prefer "small sector" erase if possible */
	if (info->flags & SECT_4K) {
		flash->erase_opcode = OPCODE_BE_4K;
		flash->mtd.erasesize = 4096;
	} else {
		flash->erase_opcode = OPCODE_SE;
		flash->mtd.erasesize = info->sector_size;
	}

	if (info->flags & M25P_QUAD) {
		set_quad(flash, 1);
	}

	if (info->flags & M25P_NO_ERASE)
		flash->mtd.flags |= MTD_NO_ERASE;

	flash->mtd.dev.parent = &pdev->dev;
	flash->page_size = info->page_size;
	flash->addr_width = info->addr_width;

	dev_info(&pdev->dev, "%s (%lld Kbytes)\n", id->name,
			(long long)flash->mtd.size >> 10);

	DEBUG(MTD_DEBUG_LEVEL2,
		"mtd .name = %s, .size = 0x%llx (%lldMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		flash->mtd.name,
		(long long)flash->mtd.size, (long long)(flash->mtd.size >> 20),
		flash->mtd.erasesize, flash->mtd.erasesize / 1024,
		flash->mtd.numeraseregions);

	if (flash->mtd.numeraseregions)
		for (i = 0; i < flash->mtd.numeraseregions; i++)
			DEBUG(MTD_DEBUG_LEVEL2,
				"mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)flash->mtd.eraseregions[i].offset,
				flash->mtd.eraseregions[i].erasesize,
				flash->mtd.eraseregions[i].erasesize / 1024,
				flash->mtd.eraseregions[i].numblocks);


	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	if (mtd_has_partitions()) {
		struct mtd_partition	*parts = NULL;
		int			nr_parts = 0;

		if (mtd_has_cmdlinepart()) {
			static const char *part_probes[]
					= { "cmdlinepart", NULL, };

			nr_parts = parse_mtd_partitions(&flash->mtd,
					part_probes, &parts, 0);
		}

		if (nr_parts <= 0 && data && data->parts) {
			parts = data->parts;
			nr_parts = data->nr_parts;
		}

		if (nr_parts > 0) {
			for (i = 0; i < nr_parts; i++) {
				DEBUG(MTD_DEBUG_LEVEL2, "partitions[%d] = "
					"{.name = %s, .offset = 0x%llx, "
						".size = 0x%llx (%lldKiB) }\n",
					i, parts[i].name,
					(long long)parts[i].offset,
					(long long)parts[i].size,
					(long long)(parts[i].size >> 10));
			}
			flash->partitioned = 1;
			return add_mtd_partitions(&flash->mtd, parts, nr_parts);
		}
	} else if (data && data->nr_parts)
		dev_warn(&pdev->dev, "ignoring %d default partitions on %s\n",
				data->nr_parts, data->name);

	return add_mtd_device(&flash->mtd) == 1 ? -ENODEV : 0;
}


static int __devexit m25p_spifi_remove(struct platform_device *pdev)
{
	struct m25p_spifi	*flash = dev_get_drvdata(&pdev->dev);
	int		status;

	/* Clean up MTD stuff. */
	if (mtd_has_partitions() && flash->partitioned)
		status = del_mtd_partitions(&flash->mtd);
	else
		status = del_mtd_device(&flash->mtd);
	if (status == 0) {
		kfree(flash);
	}
	return 0;
}


static struct platform_driver m25p80_spifi_driver = {
	.driver = {
		.name	= "m25p80_spifi",
		.owner	= THIS_MODULE,
	},
	//.id_table	= m25p_spifi_ids,
	.probe	= m25p_spifi_probe,
	.remove	= __devexit_p(m25p_spifi_remove),

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};


static int __init m25p80_spifi_init(void)
{
	return platform_driver_register(&m25p80_spifi_driver);
}


static void __exit m25p80_spifi_exit(void)
{
	platform_driver_unregister(&m25p80_spifi_driver);
}


module_init(m25p80_spifi_init);
module_exit(m25p80_spifi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pavel Boldin");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips over LPC43XX SPIFI");
