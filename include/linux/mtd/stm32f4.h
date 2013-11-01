/*
 * Based on phys-map code.
 *
 * Copyright (C) 2013
 * Pavel Boldin, Emcraft Systems, paboldin@emcraft.com
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

#ifndef __LINUX_MTD_STM32F4__
#define __LINUX_MTD_STM32F4__

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

struct map_info;

struct stm32f4_flash_data {
	unsigned int		width;
	void			(*set_vpp)(struct map_info *, int);
	unsigned int		nr_parts;
	unsigned int		pfow_base;
	struct mtd_partition	*parts;
};

#endif /* __LINUX_MTD_STM32F4__ */
