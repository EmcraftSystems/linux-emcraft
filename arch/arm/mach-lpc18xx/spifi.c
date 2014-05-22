/*
 * linux/arch/arm/mach-lpc18xx/spifi.c
 *
 * Copyright (C) 2014 Emcraft Systems
 *
 * Pavel Boldin <paboldin@emcraft.com>
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

#define LPC18XX_SPIFI_BASE	0x40003000
static struct resource flash_resources[] = {
	{
		.start	= LPC18XX_SPIFI_BASE,
		.end	= LPC18XX_SPIFI_BASE + 0x400,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 0x14000000,
		.end	= 0x14000000 + 32 * 1024 * 1024,
		.flags	= IORESOURCE_MEM,
	},
};

#define FLASH_IMAGE_OFFSET	0x10000
#define FLASH_JFFS2_OFFSET	(16*1024*1024)
static struct mtd_partition flash_partitions[] = {
	{
		.name	= "flash_uboot",
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

static struct flash_platform_data flash_data = {
	.nr_parts	= ARRAY_SIZE(flash_partitions),
	.parts		= flash_partitions,
};

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
	platform_device_register(&flash_dev);
}
