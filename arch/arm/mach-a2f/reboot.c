/*
 * linux/arch/arm/mach-a2f/reboot.c
 *
 * Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
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

#include <mach/a2f.h>
#include <mach/reboot.h>

/*
 * Perform the low-level reboot.
 */
void a2f_reboot(void)
{
	/*
	 * Perform reset but keep priority group unchanged.
	 */
	A2F_SCB->aircr  = (0x5FA << 16) |
                          (A2F_SCB->aircr & (7<<8)) |
                          (1<<2); 
}
