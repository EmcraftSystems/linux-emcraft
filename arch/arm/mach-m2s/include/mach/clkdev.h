/*
 * arch/arm/mach-m2s/include/mach/clkdev.h
 *
 * Copyright (C) 2009 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ASM_MACH_CLKDEV_H
#define __ASM_MACH_CLKDEV_H

#define __clk_get(clk) ({ 1; })
#define __clk_put(clk) do { } while (0)

#endif
