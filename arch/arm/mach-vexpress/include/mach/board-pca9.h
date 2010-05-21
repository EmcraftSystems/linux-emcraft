/*
 * arch/arm/mach-vexpress/include/mach/board-pca9.h
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

#ifndef __ASM_ARCH_BOARD_ARM_VEXPRESS_H
#define __ASM_ARCH_BOARD_ARM_VEXPRESS_H

#include <mach/platform.h>

/*
 * VEXPRESS_SELECTED_CLCD 1 = Tile, 0 = Motherboard
 */

#if defined(CONFIG_VEXPRESS_USE_TILE_CLCD)
  #define VEXPRESS_SELECTED_CLCD 1
#else
  #define VEXPRESS_SELECTED_CLCD 0
#endif

/*
 * Peripheral addresses
 */
#define VEXPRESS_UART0_BASE		0x10009000	/* UART 0 */
#define VEXPRESS_UART1_BASE		0x1000A000	/* UART 1 */
#define VEXPRESS_UART2_BASE		0x1000B000	/* UART 2 */
#define VEXPRESS_UART3_BASE		0x1000C000	/* UART 3 */
#define VEXPRESS_SSP_BASE		0x1000D000	/* Synchronous Serial Port */
#define VEXPRESS_WATCHDOG0_BASE		0x1000F000	/* Watchdog 0 */
#define VEXPRESS_WATCHDOG_BASE		0x10010000	/* watchdog interface */
#define VEXPRESS_TIMER0_1_BASE		0x10011000	/* Timer 0 and 1 */
#define VEXPRESS_TIMER2_3_BASE		0x10012000	/* Timer 2 and 3 */
#define VEXPRESS_RTC_BASE		0x10017000	/* Real Time Clock */
#define VEXPRESS_TIMER4_5_BASE		0x10018000	/* Timer 4/5 */
#define VEXPRESS_TIMER6_7_BASE		0x10019000	/* Timer 6/7 */
#define VEXPRESS_CF_BASE		0x1001A000	/* CF Controller */
#define VEXPRESS_ONB_SRAM_BASE		0x10060000	/* On-board SRAM */
#define VEXPRESS_DMC_BASE		0x100E0000	/* DMC configuration */
#define VEXPRESS_SMC_BASE		0x100E1000	/* SMC configuration */
#define VEXPRESS_FLASH0_BASE		0x40000000
#define VEXPRESS_FLASH0_SIZE		SZ_64M
#define VEXPRESS_FLASH1_BASE		0x44000000
#define VEXPRESS_FLASH1_SIZE		SZ_64M
#define VEXPRESS_ETH_BASE		0x4E000000	/* Ethernet */
#define VEXPRESS_USB_BASE		0x4F000000	/* USB */
#define VEXPRESS_SDRAM5_BASE		0x60000000	/* SDRAM bank 5 256MB */
#define VEXPRESS_SDRAM6_BASE		0x70000000	/* SDRAM bank 6 256MB */

#define VEXPRESS_SCU_BASE		0x1E000000      /* SCU registers */
#define VEXPRESS_CA9_GIC_CPU_BASE	0x1E000100      /* CA9 Private Generic interrupt controller CPU interface */
#define VEXPRESS_CA5_GIC_CPU_BASE	0x1E000000      /* CA5 Private Generic interrupt controller CPU interface */
#define VEXPRESS_TWD_BASE		0x1E000600
#define VEXPRESS_TWD_PERCPU_BASE	0x1E000700
#define VEXPRESS_TWD_SIZE		0x00000100
#define VEXPRESS_GIC_DIST_BASE		0x1E001000      /* Private Generic interrupt controller distributor */
#define VEXPRESS_L220_BASE		0x1E00A000      /* L220 registers */

#define VEXPRESS_SYS_PLD_CTRL1		0x74

#if VEXPRESS_SELECTED_CLCD == 1
#define VEXPRESS_CLCD_BASE	0x10020000	/* Daughterboard CLCD */
#else
#define VEXPRESS_CLCD_BASE	0x1001F000	/* Motherboard CLCD */
#endif

#endif	/* __ASM_ARCH_BOARD_ARM_VEXPRESS_H */
