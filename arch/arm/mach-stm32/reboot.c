/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/init.h>

struct cm3_scb {
	unsigned int	cpuid;
	unsigned int	icsr;
	unsigned int	vtor;
	unsigned int	aircr;
};

#define CM3_SCB_BASE	0xE000ED00
#define CM3_SCB		((volatile struct cm3_scb *)(CM3_SCB_BASE))

/*
 * Perform the low-level reboot.
 */
void stm32_reboot(void)
{
	/*
	 * Perform reset but keep priority group unchanged.
	 */
	CM3_SCB->aircr  = (0x5FA << 16) |
                          (CM3_SCB->aircr & (7<<8)) |
                          (1<<2);
}
