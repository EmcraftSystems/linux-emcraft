/*
 * Copyright (C) 2014
 * Anton Protopopov, Emcraft Systems, antonp@emcraft.com
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

/*
 * Based on the code designed by Yuri Azanov <azanow@ya.ru>
 */

#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <asm/setup.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/iomux.h>
#include <asm/mach/arch.h>

static struct mtd_info *ea1788_mtd;
static void __iomem *ea1788_nand_base;

/*
 * hardware specific access to control-lines
 *
 *	NCE IO_ADDR_W
 *	ALE IO_ADDR_W+(1<<19)
 *	CLE IO_ADDR_W+(1<<20)
 *
 */

#define NAND_ALE_OFFS	(1UL << 19)
#define NAND_CLE_OFFS	(1UL << 20)

static void ea1788_nand_hwcontrol(struct mtd_info *mtd, int cmd,
						unsigned int ctrl)
{
	unsigned long addr = (unsigned long) ea1788_nand_base;

	if (ctrl & NAND_ALE)
		addr |= NAND_ALE_OFFS;
	else if (ctrl & NAND_CLE)
		addr |= NAND_CLE_OFFS;

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, (void __iomem *) addr);
}

#ifdef CONFIG_MTD_CMDLINE_PARTS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

static int ea1788_nand_probe(struct platform_device *dev)
{
	struct physmap_flash_data *nand_data;
	struct nand_chip *this;
	const char *part_type;
	unsigned long size;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *mtd_parts;
	int nbparts = 0;
#endif
	int ret = 0;

	nand_data = dev->dev.platform_data;
	if (nand_data == NULL)
		return -ENODEV;

	if (dev->num_resources != 1)
		return -ENODEV;

	/* Allocate memory for MTD device structure and private data */
	ea1788_mtd = kzalloc(sizeof(*ea1788_mtd) + sizeof(*this), GFP_KERNEL);
	if (!ea1788_mtd) {
		pr_warning("Unable to allocate NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* Map physical adress */
	size = dev->resource[0].end - dev->resource[0].start + 1;
	ea1788_nand_base = ioremap(dev->resource[0].start, size);
	if (!ea1788_nand_base) {
		printk("Ioremap NAND MTD chip fails.\n");
		ret = -EINVAL;
		goto err_ioremap;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)(&ea1788_mtd[1]);

	/* Link the private data with the MTD structure */
	ea1788_mtd->priv = this;
	ea1788_mtd->owner = THIS_MODULE;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = ea1788_nand_base;
	this->IO_ADDR_W = ea1788_nand_base;
	this->cmd_ctrl = ea1788_nand_hwcontrol;
	this->chip_delay = CONFIG_MTD_NAND_EA1788_CHIP_DELAY;
	this->ecc.mode = NAND_ECC_SOFT;
	this->options = NAND_USE_FLASH_BBT;

	/* Scan to find existance of the device */
	if (nand_scan(ea1788_mtd, 1)) {
		ret = -ENXIO;
		goto err_scan;
	}

#ifdef CONFIG_MTD_PARTITIONS

#ifdef CONFIG_MTD_CMDLINE_PARTS
	nbparts = parse_mtd_partitions(ea1788_mtd, part_probes, &mtd_parts, 0);
	if (nbparts > 0)
		part_type = "command line";
	else
		nbparts = 0;
#endif

	if (!nbparts) {
		mtd_parts = nand_data->parts;
		nbparts = nand_data->nr_parts;
		part_type = "static";
	}

	/* Register the partitions */
	pr_debug("Using %s partition definition\n", part_type);
	ret = add_mtd_partitions(ea1788_mtd, mtd_parts, nbparts);
	if (ret)
		goto err_scan;
#endif

	return 0;

err_scan:
	iounmap(ea1788_nand_base);
err_ioremap:
	kfree(ea1788_mtd);

	return ret;
}

static int ea1788_nand_remove(struct platform_device *dev)
{
	if (ea1788_mtd) {
		nand_release(ea1788_mtd);
		iounmap(ea1788_nand_base);
		kfree(ea1788_mtd);
	}
	return 0;
}

static struct platform_driver ea1788_nand_driver = {
	.probe		= ea1788_nand_probe,
	.remove		= ea1788_nand_remove,
	.driver		= {
		.name	= "lpc178x_nand",
		.owner	= THIS_MODULE,
	},
};

static int __init ea1788_init(void)
{
	return platform_driver_register(&ea1788_nand_driver);
}

static void __exit ea1788_exit(void)
{
	platform_driver_unregister(&ea1788_nand_driver);
}

module_init(ea1788_init);
module_exit(ea1788_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anton Protopopov <antonp@emcraft.com>");
MODULE_DESCRIPTION("NAND flash driver for LPC178x-based boards");
