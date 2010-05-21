/*
 * arch/arm/mach-mps/include/mach/irqs.h
 *
 * Copyright (C) 2009 ARM Ltd.
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

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/*
 * MPS interrupt sources
 */
#define IRQ_MPS_WDOG		0	/* Watchdog timer */
#define IRQ_MPS_RTC		1	/* Real Time Clock */
#define IRQ_MPS_TIMER0_1	2	/* Timer 0 and 1 */
#define IRQ_MPS_TIMER2_3	3	/* Timer 2 and 3 */
#define IRQ_MPS_MMCIA		4	/* Multimedia Card A */
#define IRQ_MPS_MMCIB		5	/* Multimedia Card B */
#define IRQ_MPS_UART0		6	/* UART 0 on development chip */
#define IRQ_MPS_UART1		7	/* UART 1 on development chip */
#define IRQ_MPS_UART2		8	/* UART 2 on development chip */
					/* Reserved */
#define IRQ_MPS_AACI		10	/* Audio Codec */
#define IRQ_MPS_CLCD		11	/* CLCD controller */
#define IRQ_MPS_ETH		12	/* Ethernet controller */
#define IRQ_MPS_USB		13	/* USB controller */
#define IRQ_MPS_USB_HC		14	/* USB controller */
#define IRQ_MPS_CHARLCD		15	/* Character LCD */
					/* 16 - 29 reserved */
#define IRQ_MPS_UART3		30	/* UART 3 on development chip */
#define IRQ_MPS_SPI		31	/* Touchscreen */

#define NR_IRQS			32

#endif
