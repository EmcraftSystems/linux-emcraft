/*
 *  linux/arch/arm/mach-m2s/include/mach/uart.h
 *
 *  Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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

#ifndef _MACH_M2S_UART_H_
#define _MACH_M2S_UART_H_

#define MSS_UART0_BASE  0x40000000
#define MSS_UART1_BASE  0x40010000

#ifndef __ASSEMBLY__

extern void __init m2s_uart_init(void);

#endif /* __ASSEMBLY__ */

#endif /* _MACH_M2S_UART_H_ */
