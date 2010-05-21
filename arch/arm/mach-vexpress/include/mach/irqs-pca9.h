/*
 * arch/arm/mach-vexpress/include/mach/irqs-pca9.h
 *
 * Copyright (C) 2009 ARM Limited
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

#ifndef __MACH_IRQS_VEXPRESS_H
#define __MACH_IRQS_VEXPRESS_H

/*
 * Irqs
 */
#define IRQ_ARM_VEXPRESS_GIC_START	32

/*
 * ARM_VEXPRESS gic irq sources
 */
#define IRQ_VEXPRESS_WATCHDOG	(IRQ_ARM_VEXPRESS_GIC_START + 0)	/* Watchdog timer */
#define IRQ_VEXPRESS_SOFT	(IRQ_ARM_VEXPRESS_GIC_START + 1)	/* Software interrupt */
#define IRQ_VEXPRESS_TIMER0_1	(IRQ_ARM_VEXPRESS_GIC_START + 2)	/* Timer 0/1 (default timer) */
#define IRQ_VEXPRESS_TIMER2_3	(IRQ_ARM_VEXPRESS_GIC_START + 3)	/* Timer 2/3 */
#define IRQ_VEXPRESS_RTC	(IRQ_ARM_VEXPRESS_GIC_START + 4)	/* Real Time Clock */
#define IRQ_VEXPRESS_UART0	(IRQ_ARM_VEXPRESS_GIC_START + 5)	/* UART 0 on development chip */
#define IRQ_VEXPRESS_UART1	(IRQ_ARM_VEXPRESS_GIC_START + 6)	/* UART 1 on development chip */
#define IRQ_VEXPRESS_UART2	(IRQ_ARM_VEXPRESS_GIC_START + 7)	/* UART 2 on development chip */
#define IRQ_VEXPRESS_UART3	(IRQ_ARM_VEXPRESS_GIC_START + 8)	/* UART 3 on development chip */
#define IRQ_VEXPRESS_MMCI0A	(IRQ_ARM_VEXPRESS_GIC_START + 9)	/* Multimedia Card 0A */
#define IRQ_VEXPRESS_MMCI0B	(IRQ_ARM_VEXPRESS_GIC_START + 10)	/* Multimedia Card 0B */
#define IRQ_VEXPRESS_AACI	(IRQ_ARM_VEXPRESS_GIC_START + 11)	/* Audio Codec */
#define IRQ_VEXPRESS_KMI0	(IRQ_ARM_VEXPRESS_GIC_START + 12)	/* Keyboard/Mouse port 0 */
#define IRQ_VEXPRESS_KMI1	(IRQ_ARM_VEXPRESS_GIC_START + 13)	/* Keyboard/Mouse port 1 */
#define IRQ_VEXPRESS_CLCD	(IRQ_ARM_VEXPRESS_GIC_START + 14)	/* Motherboard CLCD controller */
#define IRQ_VEXPRESS_ETH	(IRQ_ARM_VEXPRESS_GIC_START + 15)	/* Ethernet controller */
#define IRQ_VEXPRESS_USB	(IRQ_ARM_VEXPRESS_GIC_START + 16)	/* USB controller */

#define IRQ_VE_CA9_L220_EVENT	(IRQ_ARM_VEXPRESS_GIC_START + 43)	/* L220 Cache controller combined Int */
#define IRQ_VE_CA9_CLCD		(IRQ_ARM_VEXPRESS_GIC_START + 44)	/* CA9 Tile CLCD controller */

#define IRQ_VEXPRESS_PMU_CPU0	(IRQ_ARM_VEXPRESS_GIC_START + 60)	/* CPU PMU Interrupts */
#define IRQ_VEXPRESS_PMU_CPU1	(IRQ_ARM_VEXPRESS_GIC_START + 61)
#define IRQ_VEXPRESS_PMU_CPU2	(IRQ_ARM_VEXPRESS_GIC_START + 62)
#define IRQ_VEXPRESS_PMU_CPU3	(IRQ_ARM_VEXPRESS_GIC_START + 63)

#define IRQ_ARM_VEXPRESS_SMC	-1
#define IRQ_ARM_VEXPRESS_SCTL	-1

#define NR_GIC_ARM_VEXPRESS	1

/*
 * Only define NR_IRQS if less than NR_IRQS_ARM_VEXPRESS
 */
#define NR_IRQS_ARM_VEXPRESS		(IRQ_ARM_VEXPRESS_GIC_START + 64)

#if defined(CONFIG_MACH_VEXPRESS)

#if !defined(NR_IRQS) || (NR_IRQS < NR_IRQS_ARM_VEXPRESS)
#undef NR_IRQS
#define NR_IRQS			NR_IRQS_ARM_VEXPRESS
#endif

#if !defined(MAX_GIC_NR) || (MAX_GIC_NR < NR_GIC_ARM_VEXPRESS)
#undef MAX_GIC_NR
#define MAX_GIC_NR		NR_GIC_ARM_VEXPRESS
#endif

#endif	/* CONFIG_MACH_VEXPRESS */

#endif	/* __MACH_IRQS_VEXPRESS_H */
