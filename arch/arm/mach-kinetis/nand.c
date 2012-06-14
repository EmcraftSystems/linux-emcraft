/*
 * NAND Flash resource initialization for the Kinetis-based boards
 *
 * Copyright (C) 2011
 * Vladimir Khusainov, Emcraft Systems, vlad@emcraft.com
 * Sergei Poselenov, Emcraft Systems, sposelenov@emcraft.com
 *
 * Copyright (C) 2012
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/fsl_nfc.h>

#include <mach/kinetis.h>
#include <mach/platform.h>
#include <mach/nand.h>

/*
 * Provide support for the external Flash.
 * This is board specific:
 *    TWR-K70F120M board has a 256 MBytes NAND flash.
 *    K70-SOM board has a 128 MBytes NAND flash.
 */

/*
 * Base address of all NFC memory-mapped data structures
 * (buffers and registers.)
 */
#define KINETIS_NFC_BASE	(KINETIS_AIPS1PERIPH_BASE + 0x00028000)
/*
 * NAND Flash Controller (NFC) interrupt
 */
#define KINETIS_NFC_IRQ		95

#ifdef CONFIG_MTD_PARTITIONS
/*
 * Flash partitioning. Generally speaking, this is application
 * specific. However, the default U-boot does make some assumptions
 * about Flash partitioning, specifically:
 *
 * 0-fffff:			U-boot environment (4 blocks to work around bad
 *				blocks and a redundant copy of the same size)
 * 100000-end of Flash:		Linux bootable image + whatever else.
 *
 * Based on these assumptions, we define the following Flash partitions:
 *
 * 0-fffff:			U-boot environment
 * 100000-1ffffff:		Linux bootable image
 * 2000000-end of Flash:	JFFS2 filesystem
 */
#define FLASH_IMAGE_OFFSET	0x100000
#define FLASH_JFFS2_OFFSET	(32*1024*1024)
static struct mtd_partition flash_partitions[] = {
	{
		.name	= "flash_uboot_env",
		.offset = 0,
		.size	= FLASH_IMAGE_OFFSET,
	},
	{
		.name	= "flash_linux_image",
		.offset = FLASH_IMAGE_OFFSET,
		.size	= (FLASH_JFFS2_OFFSET - FLASH_IMAGE_OFFSET),
	},
	{
		.name	= "flash_jffs2",
		.offset = FLASH_JFFS2_OFFSET,
	},
};
#endif /* CONFIG_MTD_PARTITIONS */

/*
 * Platform resources for the NAND Flash Controller
 */
static struct resource nfc_resources[] = {
	[0] = {
		.name	= "nfc-config",
		.start	= KINETIS_NFC_BASE,
		.end	= KINETIS_NFC_BASE + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "nfc-int-level",
		.start  = KINETIS_NFC_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

/*
 * Platform-specific data for the FSL NAND Flash driver
 */
static struct fsl_nfc_nand_platform_data flash_data = {
#ifdef CONFIG_MTD_PARTITIONS
	.nr_parts	= ARRAY_SIZE(flash_partitions),
	.parts		= flash_partitions,
#endif /* CONFIG_MTD_PARTITIONS */
	.flags		= 0,
};

/*
 * Platform device for the NAND Flash Controller
 */
static struct platform_device nfc_device = {
	.name           = "fsl_nfc",
	.id             = -1,
	.resource       = nfc_resources,
	.num_resources  = ARRAY_SIZE(nfc_resources),
	.dev		= {
		.platform_data = &flash_data,
	},
};

/*
 * Register the Flash platform device with the kernel.
 */
void __init kinetis_nand_init(void)
{
	unsigned int size = 0;

	/*
	 * Calculate Flash and partition sizes at run time
	 */
	switch (kinetis_platform_get()) {
	case PLATFORM_KINETIS_TWR_K70F120M:
		size = 256*1024*1024;
		break;
	case PLATFORM_KINETIS_K70_SOM:
	case PLATFORM_KINETIS_K61_SOM:
		size = 128*1024*1024;
		/* The NAND flash chip is 8-bit */
		flash_data.flags |= FSL_NFC_NAND_FLAGS_BUSWIDTH_8;
		break;
	default:
		printk(KERN_ERR "%s: Unknown platform %#x, exit\n", __func__,
			kinetis_platform_get());
		goto xit;
	}

#ifdef CONFIG_MTD_PARTITIONS
	flash_partitions[2].size = size - FLASH_JFFS2_OFFSET;
#endif /* CONFIG_MTD_PARTITIONS */

	/*
	 * Register a platform device for the external Flash.
	 * If there is no external Flash in your design, or
	 * you don't want to use it in your application, just
	 * comment out the line below.
	 */
	platform_device_register(&nfc_device);
xit:
	return;
}
