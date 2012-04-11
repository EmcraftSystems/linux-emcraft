/*
 * Public header file for the FSL NAND Flash driver
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

#ifndef __LINUX_MTD_FSL_NFC__
#define __LINUX_MTD_FSL_NFC__

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

/*
 * Use this structure in order to pass the partition table and other options
 * from the platform-specific code to the FSL NAND Flash driver.
 */
struct fsl_nfc_nand_platform_data {
#ifdef CONFIG_MTD_PARTITIONS
	unsigned int		nr_parts;
	struct mtd_partition	*parts;
#endif /* CONFIG_MTD_PARTITIONS */
	unsigned int		flags;
};

/*
 * Bit descriptions of the `fec_platform_data::flags` bit field
 */
/* Whether the NAND flash device is 8-bit */
#define FSL_NFC_NAND_FLAGS_BUSWIDTH_8	(1 << 0)

#endif /* __LINUX_MTD_FSL_NFC__ */
