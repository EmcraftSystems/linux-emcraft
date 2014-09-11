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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/serial_8250.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <mach/platform.h>
#include <mach/lpc178x.h>
#include <mach/nand.h>

static struct resource nand_resources[] = {
	{
		.flags	= IORESOURCE_MEM,
	},
};

#define NAND_IMAGE_OFFSET	0x20000
#define NAND_JFFS2_OFFSET	(3*1024*1024)
static struct mtd_partition nand_partitions[] = {
	{
		.name	= "nand_uboot_env",
		.offset = 0,
		.size	= NAND_IMAGE_OFFSET,
	},
	{
		.name	= "nand_linux_image",
		.offset = NAND_IMAGE_OFFSET,
		.size	= (NAND_JFFS2_OFFSET - NAND_IMAGE_OFFSET),
	},
	{
		.name	= "nand_jffs2",
		.offset = NAND_JFFS2_OFFSET,
	},
};

static struct physmap_flash_data nand_data = {
	.parts		= nand_partitions,
	.nr_parts	= ARRAY_SIZE(nand_partitions),
};

static struct platform_device nand_dev = {
	.name           = "lpc178x_nand",
	.id             = -1,
	.resource       = nand_resources,
	.num_resources  = ARRAY_SIZE(nand_resources),
	.dev		= { .platform_data = &nand_data },
};

void __init lpc178x_nand_init(void)
{
	unsigned int start, size;
	int plat = lpc178x_platform_get();

	/*
	 * Check that platform is supported
	 */
	switch (plat) {
	case PLATFORM_LPC178X_EA_LPC1788:
		start = 0x90000000;
		size = 0x04000000;
		break;
	default:
		pr_err("%s: Unknown platform %#x, exit\n", __func__, plat);
		return;
	}

	/*
	 * Setup platform-specific start and size of the NAND memory
	 */
	nand_resources[0].start = start;
	nand_resources[0].end = nand_resources[0].start + size - 1;

	/*
	 * Setup the size of the last partition
	 */
	nand_partitions[2].size = size - NAND_JFFS2_OFFSET;

	platform_device_register(&nand_dev);
}
