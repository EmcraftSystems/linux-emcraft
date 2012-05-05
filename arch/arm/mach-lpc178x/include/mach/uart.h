/*
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Sergei Poselenov <sposelenov@emcraft.com>
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

#ifndef _MACH_LPC178X_UART_H_
#define _MACH_LPC178X_UART_H_

/*
 * Base addresses of UART registers
 */
#define LPC178X_UART0_BASE	0x4000C000
#define LPC178X_UART1_BASE	0x40010000
#define LPC178X_UART2_BASE	0x40098000
#define LPC178X_UART3_BASE	0x4009C000
#define LPC178X_UART4_BASE	0x400A4000

#ifndef __ASSEMBLY__
void __init lpc178x_uart_init(void);

#endif /* __ASSEMBLY__ */

#endif	/*_MACH_LPC178X_UART_H_ */
