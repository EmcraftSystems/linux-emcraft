/*
 * linux/arch/arm/mach-lpc18xx/spifi.c
 *
 * Copyright (C) 2014-2015 Emcraft Systems
 *
 * Pavel Boldin <paboldin@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
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
#include <linux/list.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>
#include <linux/platform_device.h>

#include <mach/lpc18xx.h>
#include <mach/platform.h>

/*
 * LPC18XX SPIFI I/O resources
 */
#define LPC18XX_SPIFI_BASE	0x40003000

static struct resource flash_resources[] = {
	{
		.start	= LPC18XX_SPIFI_BASE,
		.end	= LPC18XX_SPIFI_BASE + 0x400,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 0x14000000,
		.end	= 0x14000000 + 2 * 1024 * 1024,
		.flags	= IORESOURCE_MEM,
	},
};

/*
 * SPIFI Flash partitioning information
 */
#define FLASH_IMAGE_OFFSET	0x1000
#define FLASH_JFFS2_OFFSET	(4*1024*1024)

static struct mtd_partition flash_partitions[] = {
	{
		.name	= "flash_uboot_env",
		.offset = 0,
		.size	= FLASH_IMAGE_OFFSET,
	},
#if 1
	/*
	 * On the EA LPC4357 Dev Kit, SPIFI is only 2MB
	 * A JFFS2 partition doesn't fit in
	 */
	{
		.name	= "flash_linux_image",
		.offset = FLASH_IMAGE_OFFSET,
	},
#else
	{
		.name	= "flash_jffs2",
		.offset = FLASH_IMAGE_OFFSET,
	},
#endif
};

static struct flash_platform_data flash_data = {
	.nr_parts	= ARRAY_SIZE(flash_partitions),
	.parts		= flash_partitions,
	.type		= "s25fl016k",
};

/*
 * SPIFI platform info
 */
static struct platform_device flash_dev = {
	.name           = "m25p80_spifi",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(flash_resources),
	.resource       = flash_resources,
	.dev		= {
		.platform_data = &flash_data,
	},
};

void __init lpc18xx_spifi_init(void)
{
	int p = lpc18xx_platform_get();

	if (p == PLATFORM_LPC18XX_EA_LPC4357_EVAL) {

		platform_device_register(&flash_dev);
	}
}
