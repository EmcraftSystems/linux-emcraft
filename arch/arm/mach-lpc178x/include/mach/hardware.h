/*
 * (C) Copyright 2009
 * ARM Ltd.
 *
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

#ifdef CONFIG_MMU
#error "ARM LPC178x/7x platform only support !MMU"
#endif

/*
 * These definitions are used by some drivers imported from
 * the NXP LPC32xx Linux port.
 */
#define _SBF(f, v)		((v) << (f))
#define _BIT(n)			_SBF(n, 1)
#define io_p2v(x)		((volatile void __iomem *)x)

#define IO_ADDRESS(x)		(x)
#define __io_address(n)		__io(IO_ADDRESS(n))

#endif
