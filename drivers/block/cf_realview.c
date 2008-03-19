/*
 * drivers/block/cf_realview.c
 *
 * Copyright (C) 2008 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/arch/platform.h>

#define DRV_NAME "realview_cf"
#define DRV_VERSION "v0.1"

/*
 * Undef this to change it back to 8-bit mode.
 * RevA boards work in 8-bit, RevB boards in 16-bit mode.
 */
#define REALVIEW_CF_16BIT_MODE

#define DEBUG	0
#if DEBUG >= 1
#define dbg(fmt, args...) 					\
	printk(KERN_EMERG "%s: " fmt "\n", 			\
	       __FUNCTION__ , ## args)
#else
#define dbg(msg, args...)
#endif				/* USE_DEBUG >= 1 */

/* ATA Command Set */
#define ATA_CMD_CHECK_POWER_MODE		0xE5
#define ATA_CMD_EXECUTE_DRIVE_DIAG		0x90
#define ATA_CMD_ERASE_SECTORS			0xC0
#define ATA_CMD_IDENTIFY_DRIVE			0xEC
#define ATA_CMD_IDLE				0xE3
#define ATA_CMD_IDLE_IMMEDIATE			0xE1
#define ATA_CMD_INIT_DRIVE_PARAMS		0x91
#define ATA_CMD_READ_BUFFER			0xE4
#define ATA_CMD_READ_MULTIPLE			0xC4
#define ATA_CMD_READ_LONG_SECTOR		0x22
#define ATA_CMD_READ_SECTOR			0x20
#define ATA_CMD_READ_VERIFY_SECTORS		0x40
#define ATA_CMD_RECALIBRATE			0x10
#define ATA_CMD_REQUEST_SENSE			0x03
#define ATA_CMD_SEEK				0x70
#define ATA_CMD_SET_FEATURES			0xEF
#define ATA_CMD_SET_MULTIPLE_MODE		0xC6
#define ATA_CMD_SET_SLEEP_MODE			0xE6
#define ATA_CMD_STAND_BY			0xE2
#define ATA_CMD_STAND_BY_IMM			0xE0
#define ATA_CMD_TRANSLATE_SECTOR		0x87
#define ATA_CMD_WEAR_LEVEL			0xF5
#define ATA_CMD_WRITE_BUFFER			0xE8
#define ATA_CMD_WRITE_LONG_SECTOR		0x32
#define ATA_CMD_WRITE_MULTIPLE			0xC5
#define ATA_CMD_WRITE_MULTI_NO_ERASE		0xCD
#define ATA_CMD_WRITE_SECTOR			0x30
#define ATA_CMD_WRITE_SECTORS_NO_ERASE		0x38
#define ATA_CMD_WRITE_VERIFY_SECTORS		0x3C

/* Control registers region */
#define IDE_REALVIEW_CTL_OFFSET			0x1C

#define ATA_REG_DATA				0x00
#define ATA_REG_ERR				0x01
#define ATA_REG_NSECT				0x02
#define ATA_REG_LBAL				0x03
#define ATA_REG_LBAM				0x04
#define ATA_REG_LBAH				0x05
#define ATA_REG_DEVICE				0x06
#define ATA_REG_STATUS				0x07

#define ATA_REG_FEATURE				ATA_REG_ERR
#define ATA_REG_CMD				ATA_REG_STATUS
#define ATA_REG_BYTEL				ATA_REG_LBAM
#define ATA_REG_BYTEH				ATA_REG_LBAH
#define ATA_REG_DEVSEL				ATA_REG_DEVICE
#define ATA_REG_IRQ				ATA_REG_NSECT

struct ata_ioports {
	void __iomem *cmd_addr;
	void __iomem *data_addr;
	void __iomem *error_addr;
	void __iomem *feature_addr;
	void __iomem *nsect_addr;
	void __iomem *lbal_addr;
	void __iomem *lbam_addr;
	void __iomem *lbah_addr;
	void __iomem *device_addr;
	void __iomem *status_addr;
	void __iomem *command_addr;
	void __iomem *altstatus_addr;
	void __iomem *ctl_addr;
};

/* Enough information to form disk geometry */
struct realview_cf_drive_info {
	u16 cur_cyc_cnt;
	u16 cur_head_cnt;
	u16 def_cyc_cnt;
	u16 def_head_cnt;
	u32 cur_sect_cap;
	u32 def_sect_cap;
	u32 sect_size;
	/* 1 extra for '\0', just playing safe */
	char serial[21];
	char model[41];
	char firmware[9];
};

struct realview_cf_device {
	/* ATA and platform device */
	struct ata_ioports port;
	int irq;

	/* Block device support */
	int size;
	int major;
	short users;
	short media_change;
	spinlock_t lock;
	spinlock_t reglock;
	struct request_queue *queue;
	struct gendisk *gd;

	/* A subset of low-level ata disk drive details, (from id page) */
	struct realview_cf_drive_info *info;
};

/* The compact flash device */
static struct realview_cf_device *cf_dev;

static int realview_cf_dev_open(struct inode *inode, struct file *fp)
{
	struct realview_cf_device *cf_dev;
	cf_dev = inode->i_bdev->bd_disk->private_data;
	fp->private_data = cf_dev;
	spin_lock(&cf_dev->lock);
	if (!cf_dev->users)
		check_disk_change(inode->i_bdev);
	cf_dev->users++;
	spin_unlock(&cf_dev->lock);

	return 0;
}

static int realview_cf_dev_release(struct inode *inode, struct file *fp)
{
	struct realview_cf_device *cf_dev;

	cf_dev = inode->i_bdev->bd_disk->private_data;
	spin_lock(&cf_dev->lock);
	cf_dev->users--;
	spin_unlock(&cf_dev->lock);

	return 0;
}

static int
realview_cf_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct realview_cf_device *cf_dev = bdev->bd_disk->private_data;

	geo->cylinders = cf_dev->info->cur_cyc_cnt;
	geo->heads = cf_dev->info->cur_head_cnt;
	geo->sectors = cf_dev->info->cur_sect_cap;
	return 0;
}

static int realview_cf_dev_ioctl(struct inode *inode, struct file *fp,
				 unsigned int cmd, unsigned long arg)
{
	struct hd_geometry geo;
	struct realview_cf_device *cf_dev;

	/* This does happen. */
	if (fp)
		if (fp->private_data)
			cf_dev = fp->private_data;
		else
			return -EINVAL;
	else
		return -EINVAL;

	dbg("cmd: %d", cmd);
	switch (cmd) {
		case HDIO_GETGEO:
			geo.cylinders = cf_dev->info->cur_cyc_cnt;
			geo.heads = cf_dev->info->cur_head_cnt;
			geo.sectors = cf_dev->info->cur_sect_cap;
			geo.start = 0;
			if (copy_to_user((void __user *)arg, &geo,
					 sizeof(geo)))
				return -EFAULT;
		return 0;
	}
	return -ENOTTY;	/* Unknown ioctl command */
}

static int realview_cf_dev_media_changed(struct gendisk *gd)
{
	return 0;
}

static int realview_cf_dev_revalidate_disk(struct gendisk *gd)
{
	return 0;
}

struct block_device_operations realview_cf_dev_ops = {
	.owner		= THIS_MODULE,
	.open		= realview_cf_dev_open,
	.release	= realview_cf_dev_release,
	.ioctl 		= realview_cf_dev_ioctl,
	.getgeo		= realview_cf_getgeo,
	.media_changed	= realview_cf_dev_media_changed,
	.revalidate_disk= realview_cf_dev_revalidate_disk
};

static void realview_cf_init_port_offsets(struct ata_ioports *ioaddr)
{
#if defined(REALVIEW_CF_16BIT_MODE)
	u32 mode = 2;
#else
	u32 mode = 1;
#endif
	ioaddr->data_addr = ioaddr->cmd_addr + ATA_REG_DATA*mode;
	ioaddr->error_addr = ioaddr->cmd_addr + ATA_REG_ERR*mode;
	ioaddr->feature_addr = ioaddr->cmd_addr + ATA_REG_FEATURE*mode;
	ioaddr->nsect_addr = ioaddr->cmd_addr + ATA_REG_NSECT*mode;
	ioaddr->lbal_addr = ioaddr->cmd_addr + ATA_REG_LBAL*mode;
	ioaddr->lbam_addr = ioaddr->cmd_addr + ATA_REG_LBAM*mode;
	ioaddr->lbah_addr = ioaddr->cmd_addr + ATA_REG_LBAH*mode;
	ioaddr->device_addr = ioaddr->cmd_addr + ATA_REG_DEVICE*mode;
	ioaddr->status_addr = ioaddr->cmd_addr + ATA_REG_STATUS*mode;
	ioaddr->command_addr = ioaddr->cmd_addr + ATA_REG_CMD*mode;
	ioaddr->ctl_addr = ioaddr->cmd_addr + IDE_REALVIEW_CTL_OFFSET*mode;
}

#define RVCF_TIMEOUT			1000
#define REALVIEW_CF_READY_STAT		0x40
static int realview_cf_dev_ready_timeout(struct realview_cf_device *cf_dev, int mul)
{
	int cnt = 33;

	if (mul)
		cnt *= mul;
	do {
		cnt--;
		udelay(30);
	} while(cnt && (!readb(cf_dev->port.status_addr)
			& REALVIEW_CF_READY_STAT) == REALVIEW_CF_READY_STAT);
	return cnt;
}

#define REALVIEW_CF_BUSY_STAT		0x80
static int realview_cf_dev_busy_timeout(struct realview_cf_device *cf_dev, int mul)
{
	int cnt = 33;

	if (mul)
		cnt *= mul;
	do {
		cnt--;
		udelay(30);
	} while(cnt && (readb(cf_dev->port.status_addr)
			& REALVIEW_CF_BUSY_STAT) == REALVIEW_CF_BUSY_STAT);
	return cnt;
}

#define REALVIEW_CF_ERROR_STAT		0x01
#define REALVIEW_CF_DRQ_STAT		0x08
static int realview_cf_dev_data_timeout(struct realview_cf_device *cf_dev, int mul)
{
	u32 badflags = REALVIEW_CF_BUSY_STAT | REALVIEW_CF_ERROR_STAT;
	int cnt = 33;
	u16 stat;

	if (mul)
		cnt *= mul;
	do {
		stat = readb(cf_dev->port.status_addr);
		cnt--;
		udelay(30);
	} while(cnt &&
		((stat & badflags) || (!(stat & REALVIEW_CF_DRQ_STAT))));
	return cnt;
}

#define	REALVIEW_CF_RESET		0x0E
#define REALVIEW_CF_NRESET		0x0A
static int realview_cf_reset_ata_device(struct realview_cf_device *cf_dev)
{
	writew(REALVIEW_CF_RESET, cf_dev->port.ctl_addr);
	writew(REALVIEW_CF_NRESET, cf_dev->port.ctl_addr);
	if (unlikely(!realview_cf_dev_busy_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Drive reset timed out.\n");
		return -EBUSY;
	}
	return 0;
}

typedef	int (*sector_io_op_t)(struct realview_cf_device *, int, void *);

static inline int realview_cf_sector_op_retry(int retry,
					      sector_io_op_t sector_io_op,
					      struct realview_cf_device *cf_dev,
					      int sector, void __iomem *buf)
{
	int ret;
	int rtotal = retry;

	/* Try once */
	ret = sector_io_op(cf_dev, sector, buf);
	retry--;

	/* Reset and retry on failure */
	while ((retry > 0) && (ret < 0)) {
		printk(KERN_INFO "Sector operation failed, retry: %d\n",
		       rtotal - retry);
		realview_cf_reset_ata_device(cf_dev);
		ret = sector_io_op(cf_dev, sector, buf);
		retry--;
	}
	return ret;
}

static int realview_cf_write_sector(struct realview_cf_device *cf_dev,
				    int sector, void __iomem *buf)
{
	int i, err = 0;
	u16 __iomem *bufp = (u16 *)buf;

	/* Wait for ready:
	 * We wait up to 10 seconds, because hw can be buggy at times. */
	if (unlikely(!realview_cf_dev_ready_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Ready signal timed out.");
		return -EBUSY;
	}
	/* Issue command */
	writeb(0xE0 + ((sector & 0x0F000000) >> 24), cf_dev->port.device_addr);
	writeb(1, cf_dev->port.nsect_addr); /* Writing one sector */
	writeb((sector & 0xFF0000) >> 16, cf_dev->port.lbah_addr);
	writeb((sector & 0xFF00) >> 8, cf_dev->port.lbam_addr);
	writeb((sector & 0xFF), cf_dev->port.lbal_addr);
	writeb(ATA_CMD_WRITE_VERIFY_SECTORS, cf_dev->port.command_addr);
	/* Wait for data ready */
	if (unlikely(!realview_cf_dev_data_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Data ready signal timed out.");
		return -EBUSY;
	}
	/* Write data */
	for (i = 0; i < 256; i++) {
		writew(bufp[i], cf_dev->port.data_addr);
	}
	/* Wait for busy */
	if (unlikely(!realview_cf_dev_busy_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Busy signal timed out.");
		return -EBUSY;
	}
	/* Return errors */
	err = readb(cf_dev->port.status_addr);
	if ((err = readb(cf_dev->port.error_addr)))
		return -EIO;

	return 0;
}

static int realview_cf_read_sector(struct realview_cf_device *cf_dev,
				   int sector, void __iomem *buf)
{
	int i, err = 0;
	u16 __iomem *bufp = (u16 *)buf;

	/* We wait up to 10 seconds, hw can be buggy at times. */
	if (unlikely(!realview_cf_dev_ready_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Ready signal timed out.");
		return -EBUSY;
	}
	/* Issue command */
	writeb(0xE0 + ((sector & 0x0F000000) >> 24), cf_dev->port.device_addr);
	writeb(1, cf_dev->port.nsect_addr); /* Reading one sector */
	writeb((sector & 0xFF0000) >> 16, cf_dev->port.lbah_addr);
	writeb((sector & 0xFF00) >> 8, cf_dev->port.lbam_addr);
	writeb((sector & 0xFF), cf_dev->port.lbal_addr);
	writeb(ATA_CMD_READ_SECTOR, cf_dev->port.command_addr);

	if (unlikely(!realview_cf_dev_data_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Data ready signal timed out.");
		return -EBUSY;
	}
	/* Read back data */
	for (i = 0; i < 256; i++) {
		bufp[i] = readw(cf_dev->port.data_addr);
	}
	/* Wait for busy */
	if (unlikely(!realview_cf_dev_busy_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Busy signal timed out.");
		return -EBUSY;
	}
	/* Return errors */
	err = readb(cf_dev->port.status_addr);
	if ((err = readb(cf_dev->port.error_addr)))
		return -EIO;

	return 0;
}

#define KERNEL_SECTOR_SIZE		512
static int realview_cf_write_sectors(struct realview_cf_device *cf_dev,
				     unsigned long sect_begin,
				     unsigned long nsect, void *buf)
{
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&cf_dev->reglock, flags);
	for (i = sect_begin; i < sect_begin + nsect; i++) {
		if (unlikely((ret = realview_cf_sector_op_retry(3,
			realview_cf_write_sector, cf_dev, i, buf)) < 0)) {
			spin_unlock_irqrestore(&cf_dev->reglock, flags);
			dbg("Problem writing sector: 0x%x", i);
			return ret;
		}
		buf += KERNEL_SECTOR_SIZE;
	}
	spin_unlock_irqrestore(&cf_dev->reglock, flags);
	return 0;
}

static int
realview_cf_read_sectors(struct realview_cf_device *cf_dev,
			 unsigned long sect_begin,
			 unsigned long nsect, void *buf)
{
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&cf_dev->reglock, flags);
	for (i = sect_begin; i < sect_begin + nsect; i++) {
		if (unlikely((ret = realview_cf_sector_op_retry(3,
			realview_cf_read_sector, cf_dev, i, buf)) < 0)) {
			spin_unlock_irqrestore(&cf_dev->reglock, flags);
			dbg("Problem reading sector: 0x%x", i);
			return ret;
		}
		buf += KERNEL_SECTOR_SIZE;
	}
	spin_unlock_irqrestore(&cf_dev->reglock, flags);
	return 0;
}

static int realview_cf_transfer(struct realview_cf_device *cf_dev,
				unsigned long sect_begin, unsigned long nsect,
				void *buf, int wr)
{
	int err;

	if ((sect_begin + nsect) > cf_dev->info->cur_sect_cap) {
		printk("<2> %s: Beyond end write (%ld %ld), max: (%d)\n",
		       __FUNCTION__, sect_begin, nsect, cf_dev->info->cur_sect_cap);
		return -EINVAL;
	}
	if (wr) {
		err = realview_cf_write_sectors(cf_dev, sect_begin, nsect, buf);
		flush_icache_range((unsigned long)buf, (unsigned long)(buf + nsect*512));
		return err;
	} else {
		err = realview_cf_read_sectors(cf_dev, sect_begin, nsect, buf);
		flush_icache_range((unsigned long)buf, (unsigned long)(buf + nsect*512));
		return err;
	}
}

static int realview_cf_xfer_bio(struct realview_cf_device *cf_dev,
				struct bio *bio)
{
	int i, err;
	struct bio_vec *bvec;
	sector_t sector = bio->bi_sector;

	bio_for_each_segment(bvec, bio, i) {
		char *buf = __bio_kmap_atomic(bio, i, KM_USER0);
		if ((err = realview_cf_transfer(cf_dev, sector,
						bio_cur_sectors(bio), buf,
						bio_data_dir(bio) == WRITE))
						< 0) {
			goto err_out;
		}
		sector += bio_cur_sectors(bio);
		__bio_kunmap_atomic(bio, KM_USER0);
	}
	return 0;

err_out:
	__bio_kunmap_atomic(bio, KM_USER0);
	return err;
}

static int realview_cf_make_request(request_queue_t *q, struct bio *bio)
{
	struct realview_cf_device *cf_dev = q->queuedata;
	int status = realview_cf_xfer_bio(cf_dev, bio);
	bio_endio(bio, status);
	return 0;
}


#define ATA_IDENTIFY_PAGE_SIZE		512
static void printk_drive_info(struct realview_cf_drive_info *info)
{
	printk(KERN_INFO "%s: Cylinders: %d\n",
	       DRV_NAME, info->cur_cyc_cnt);
	printk(KERN_INFO "%s: Heads: %d\n",
	       DRV_NAME, info->cur_head_cnt);
	printk(KERN_INFO "%s: Capacity (bytes): %d\n",
	       DRV_NAME, info->cur_sect_cap * info->sect_size);
	printk(KERN_INFO "%s: Serial No: %s\n", DRV_NAME, info->serial);
	printk(KERN_INFO "%s: Model: %s\n", DRV_NAME, info->model);
	printk(KERN_INFO "%s: Firmware version: %s\n",
	       DRV_NAME, info->firmware);
}

static int realview_cf_identify_ata_device(struct realview_cf_device *cf_dev)
{
	int i, c;
	struct realview_cf_drive_info *info = cf_dev->info;
	u8 *id_buf = kzalloc(ATA_IDENTIFY_PAGE_SIZE, GFP_KERNEL);
	u16 *id_page = (u16 *)id_buf;

	if (unlikely(!id_buf))
		return -ENOMEM;

	if (unlikely(!realview_cf_dev_ready_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Device ready timed out.\n");
		return -EBUSY;
	}

	/* Issue DEVICE_IDENTIFY */
	writeb(0xE0, cf_dev->port.device_addr);
	writeb(0x01, cf_dev->port.nsect_addr);
	writeb(0x01, cf_dev->port.lbal_addr);
	writeb(0x00, cf_dev->port.lbam_addr);
	writeb(0x00, cf_dev->port.lbah_addr);
	writeb(ATA_CMD_IDENTIFY_DRIVE, cf_dev->port.command_addr);

	/* Collect results */
	if (unlikely(!realview_cf_dev_busy_timeout(cf_dev, RVCF_TIMEOUT))) {
		dbg("Device data timed out during identify.\n");
		return -EBUSY;
	}

	for (i = 0; i < 256; i++)
		id_page[i] = readw(cf_dev->port.data_addr);

	/* Interpret identify page fields */
	/* Default cylinder count */
	info->def_cyc_cnt = id_page[1];
	/* Current cylinder count */
	info->cur_cyc_cnt = id_page[54];

	/* Default head count */
	info->def_head_cnt = id_page[3];
	/* Current head count */
	info->cur_head_cnt = id_page[55];

	/* Default capacity */
	info->def_sect_cap = (u32)(id_page[7] << 16); 	/* MSW */
	info->def_sect_cap |= (u32)id_page[8];		/* LSW */

	/* Current capacity */
	info->cur_sect_cap = (u32)id_page[57]; /* LSW */
	info->cur_sect_cap |= (u32)(id_page[58] << 16); /* MSW */
	for (c = 0; c < 10; c++) {
		int val = *((u16 *)(&id_page[10] + c));
		info->serial[c*2] = (val >> 8);
		info->serial[c*2+1]= val & 0xFF;
	}
	for (c = 0; c < 4; c++) {
		int val = *((u16 *)(&id_page[23] + c));
		info->firmware[c*2] = (val >> 8);
		info->firmware[c*2+1]= val & 0xFF;
	}
	for (c = 0; c < 20; c++) {
		int val = *((u16 *)(&id_page[27] + c));
		info->model[c*2] = (val >> 8);
		info->model[c*2+1]= val & 0xFF;
	}
	/* Use most likely sector size for simplicity */
	info->sect_size = KERNEL_SECTOR_SIZE;
	cf_dev->size = info->cur_sect_cap * info->sect_size;
	printk_drive_info(cf_dev->info);
	kfree(id_buf);
	return 0;
}

/* Initialise the hardware, obtain device geometry parameters */
static int realview_cf_dev_ata_init(struct realview_cf_device *cf_dev)
{
	int err = 0;

	dbg("Resetting drive...");
	/* Reset drive */
	if (unlikely((err = realview_cf_reset_ata_device(cf_dev)) < 0))
		return err;

	dbg("Identifying drive.");
	/* Identify drive */
	if (unlikely((err = realview_cf_identify_ata_device(cf_dev)) < 0))
		return err;
	return 0;
}

/* Allocates and initialisas the basic properties of the device such as
 * its virtual address, irq, memory resources etc. */
static struct realview_cf_device *
realview_cf_dev_alloc_init(void __iomem *base, int irq)
{
	struct realview_cf_device *cf_dev
		= kzalloc(sizeof(struct realview_cf_device), GFP_KERNEL);

	if (unlikely(!cf_dev))
		return 0;
	cf_dev->info = kzalloc(sizeof(struct realview_cf_drive_info),
			       GFP_KERNEL);
	if (unlikely(!cf_dev->info)) {
		kfree(cf_dev);
		return 0;
	}

	cf_dev->port.cmd_addr = base;
	realview_cf_init_port_offsets(&cf_dev->port);
	cf_dev->irq = irq;
	return cf_dev;
}

#define REALVIEW_CF_DEV_MINORS		16
/* Initialises block device interface using disk geometry information
 * (e.g. sectors, capacity) obtained by ata initialisation method */
static int realview_cf_dev_blkdev_init(struct realview_cf_device *cf_dev)
{
	int err = 0;

	cf_dev->major = register_blkdev(0, DRV_NAME);
	spin_lock_init(&cf_dev->lock);
	spin_lock_init(&cf_dev->reglock);
	if (unlikely(!(cf_dev->queue = blk_alloc_queue(GFP_KERNEL)))) {
		err = -ENOMEM;
		goto out1;
	}
	/* Register request function for this queue */
	blk_queue_make_request(cf_dev->queue, realview_cf_make_request);

	blk_queue_hardsect_size(cf_dev->queue, cf_dev->info->sect_size);
	cf_dev->queue->queuedata = cf_dev;

	cf_dev->gd = alloc_disk(REALVIEW_CF_DEV_MINORS);
	if (!cf_dev->gd) {
		err = -ENOMEM;
		goto out2;
	}
	cf_dev->gd->major = cf_dev->major;
	/* Minor 0 is used for the device itself, e.g. /dev/hda, or
	 * /dev/hda0. The subsequent minors are used for the
	 * partitions, e.g. /dev/hda1 */
	cf_dev->gd->first_minor = 0;
	cf_dev->gd->minors = REALVIEW_CF_DEV_MINORS;
	cf_dev->gd->fops = &realview_cf_dev_ops;
	cf_dev->gd->queue = cf_dev->queue;
	cf_dev->gd->private_data = cf_dev;
	strcpy(cf_dev->gd->disk_name, DRV_NAME);
	set_capacity(cf_dev->gd, cf_dev->info->cur_sect_cap *
		     (cf_dev->info->sect_size/KERNEL_SECTOR_SIZE));
	add_disk(cf_dev->gd);
	return 0;
out2:
	/* FIXME: How to free block_alloc_queue()??? */
out1:
	unregister_blkdev(cf_dev->major, DRV_NAME);
	return err;
}

static void realview_cf_init_memory_controller(void __iomem *cookie)
{
	/* Set up the memory controller to 8-bit mode */
	writel(0xF, (cookie));
	writel(0xF, (cookie + 0x4));
	writel(0xF, (cookie + 0x8));
	writel(0x5, (cookie + 0xC));
	writel(0x5, (cookie + 0x10));
#if defined(REALVIEW_CF_16BIT_MODE)
	writel(0x303011, (cookie + 0x14));
#else
	writel(0x303001, (cookie + 0x14));
#endif
	writel(0x1F, (cookie + 0x1C));
}

static int realview_cf_probe(struct device *dev)
{
	int err = 0;
	void __iomem *vaddr, *vaddr_memres1;
	unsigned int memres_len, memres1_len;
	struct resource *memres, *memres1, *irqres;
	struct platform_device *pdev = to_platform_device(dev);

	if (strcmp(DRV_NAME, pdev->name))
		return -ENODEV;

	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	memres1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	irqres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	memres_len = memres->end - memres->start;
	memres1_len = memres1->end - memres1->start;

	if (!memres || !memres1)
		dbg("Platform device: %s, Failed to claim memory resource %d.",
		    pdev->name, (!memres) ? 0 : 1);
	if (!irqres)
		dbg("Platform device: %s, Failed to claim irq resource\n",
		    pdev->name);

	if (!request_mem_region(memres->start, memres_len, memres->name))
		return -EBUSY;
	if (!request_mem_region(memres1->start, memres1_len, memres1->name)) {
		err = -EBUSY;
		goto out1;
	}
	if (!(vaddr = ioremap_nocache(memres->start, memres_len))) {
		err = -ENOMEM;
		goto out2;
	}
	if (!(vaddr_memres1 = ioremap_nocache(memres1->start, memres1_len))) {
		err = -ENOMEM;
		goto out3;
	}
	realview_cf_init_memory_controller(vaddr_memres1);
	iounmap(vaddr_memres1);
	if (!(cf_dev = realview_cf_dev_alloc_init(vaddr, irqres->start))) {
		err = -ENOMEM;
		goto out3;
	}
	if ((err = realview_cf_dev_ata_init(cf_dev)) < 0) {
		dbg("ATA initialisation error.");
		goto out3;
	}

	return realview_cf_dev_blkdev_init(cf_dev);

out3:
	iounmap(vaddr);
out2:
	release_mem_region(memres1->start, memres1_len);
out1:
	release_mem_region(memres->start, memres_len);
	dbg("Failed with error: %d", err);
	return err;
}

static int realview_cf_remove(struct device *dev)
{
	unsigned int memres_len, irqres_len;
	struct resource *memres, *irqres;
	struct platform_device *pdev = to_platform_device(dev);

	/* Release platform device resources */
	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irqres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	memres_len = memres->end - memres->start;
	irqres_len = irqres->end - irqres->start;

	iounmap(cf_dev->port.cmd_addr);
	release_mem_region(memres->start, memres_len);
	release_mem_region(irqres->start, irqres_len);

	/* Release blkdev related resources */
	if (likely(cf_dev->gd)) {
		del_gendisk(cf_dev->gd);
		put_disk(cf_dev->gd);
	}
	if (likely(cf_dev->queue)) {
		blk_put_queue(cf_dev->queue);
	}
	unregister_blkdev(cf_dev->major, DRV_NAME);
	if (cf_dev->info)
		kfree(cf_dev->info);
	kfree(cf_dev);
	return 0;
}

static struct device_driver realview_cf_driver = {
	.name 		= DRV_NAME,
	.probe 		= realview_cf_probe,
	.remove		= realview_cf_remove,
	.bus		= &platform_bus_type
};

static int __init realview_cf_init(void)
{
	return driver_register(&realview_cf_driver);
}

static void __exit realview_cf_exit(void)
{
	driver_unregister(&realview_cf_driver);
}

MODULE_AUTHOR("Bahadir Balban");
MODULE_DESCRIPTION("Realview PB11MPCore CompactFlash driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(realview_cf_init);
module_exit(realview_cf_exit);
