#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/setup.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/iomux.h>
#include <asm/mach/arch.h>

#define NAND_BASE	0x90000000
#define NAND_SIZE	0x04000000

static struct mtd_info *ea1788_mtd;
static void __iomem *ea1788_nand_base;

#ifdef CONFIG_MTD_PARTITIONS

#define NAND_IMAGE_OFFSET	0x20000
#define NAND_JFFS2_OFFSET	(3*1024*1024)

static struct mtd_partition ea1788_nand_partition_info[] = {
	{
		.name	= "nand_env",
		.offset	= 0,
		.size	= NAND_IMAGE_OFFSET,
	},
	{
		.name	= "nand_kernel",
		.offset	= NAND_IMAGE_OFFSET,
		.size	= NAND_JFFS2_OFFSET - NAND_IMAGE_OFFSET,
	},
	{
		.name   = "nand_filesystem",
		.offset = NAND_JFFS2_OFFSET,
		.size   = MTDPART_SIZ_FULL,
	},
};

#define NUM_PARTITIONS (ARRAY_SIZE(ea1788_nand_partition_info))

#endif


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

static int __init ea1788_init(void)
{
	struct nand_chip *this;
	const char *part_type;
#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	static const char *part_probes[] = { "cmdlinepart", NULL };
#endif
	struct mtd_partition *mtd_parts;
	int nbparts = 0;
#endif
	int ret = 0;

	/* Allocate memory for MTD device structure and private data */
	ea1788_mtd = kzalloc(sizeof(*ea1788_mtd) + sizeof(*this), GFP_KERNEL);
	if (!ea1788_mtd) {
		printk("Unable to allocate NAND MTD device structure.\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Map physical adress */
	ea1788_nand_base = ioremap(NAND_BASE, NAND_SIZE);
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
	this->chip_delay = 25;
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
		mtd_parts = ea1788_nand_partition_info;
		nbparts = NUM_PARTITIONS;
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
out:
	return ret;
}

static void __exit ea1788_cleanup(void)
{
	if (ea1788_mtd) {
		nand_release(ea1788_mtd);
		iounmap(ea1788_nand_base);
		kfree(ea1788_mtd);
	}
}

module_init(ea1788_init);
module_exit(ea1788_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yuri Azanov <azanow@ya.ru>");
MODULE_DESCRIPTION("NAND flash driver for Embedded Artists LPC2478 OEM Board");
