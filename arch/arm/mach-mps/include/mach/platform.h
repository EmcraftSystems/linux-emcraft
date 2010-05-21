/*
 * arch/arm/mach-mps/include/mach/platform.h
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

#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

/*
 * MPS system registers
 */
#define MPS_SYS_BASE		0x40000000
#define MPS_SYS_ID_OFFSET	0x00		/* Board and FPGA identifier */
#define MPS_SYS_PERCFG_OFFSET	0x04		/* Peripheral control signals */
#define MPS_SYS_SW_OFFSET	0x08		/* Indicates user switch settings */
#define MPS_SYS_LED_OFFSET	0x0C		/* Sets LED outputs */
#define MPS_SYS_7SEG_OFFSET	0x10		/* Sets LED outputs */
#define MPS_SYS_CNT25MHz_OFFSET	0x14		/* Free running counter incrementing at 25MHz */
#define MPS_SYS_CNT100Hz_OFFSET	0x18		/* Free running counter incrementing at 100Hz */

/*
 * MPS peripheral addresses
 */
#define MPS_FLASH_BASE		0x18000000
#define MPS_FLASH_SIZE		SZ_64M

#ifdef CONFIG_MACH_MPS_MODEL
#define MPS_SPI_BASE		0x1F004000	/* Touchscreen */
#define MPS_UART3_BASE		0x1F005000	/* UART 3 */
#else
#define MPS_SPI_BASE		0xdfff4000	/* Touchscreen */
#define MPS_UART3_BASE		0xdfff5000	/* UART 3 */
#endif

#define MPS_WATCHDOG_BASE	0x40000000	/* watchdog interface */
#define MPS_RTC_BASE		0x40001000	/* Real Time Clock */
#define MPS_TIMER0_1_BASE	0x40002000	/* Timer 0 and 1 */
#define MPS_TIMER2_3_BASE	0x40003000	/* Timer 2 and 3 */
#define MPS_SCTL_BASE		0x40004000	/* System controller */
#define MPS_MMCI_BASE		0x40005000	/* MMC interface */
#define MPS_UART0_BASE		0x40006000	/* UART 0 */
#define MPS_UART1_BASE		0x40007000	/* UART 1 */
#define MPS_UART2_BASE		0x40008000	/* UART 2 */
						/* Reserved */
#define MPS_AACI_BASE		0x4000A000	/* Audio */
#define MPS_I2C_BASE		0x4000B000	/* I2C control */
#define MPS_CHAR_LCD_BASE	0x4000C000	/* Character LCD */

#define MPS_ETH_BASE		0x4FFE0000	/* Ethernet */
#define MPS_CLCD_BASE		0x4FFF0000	/* CLCD */

#define MPS_DMC_BASE		0x60000000	/* Dynamic Memory Controller */

#define MPS_SMC_BASE		0xA0000000	/* Static Memory Controller */
//#define MPS_USB_BASE		0xA0000000	/* USB */

/*
 * System controller bit assignment
 */
#define MPS_REFCLK		0
#define MPS_TIMCLK		1

#define MPS_TIMER1_EnSel	15
#define MPS_TIMER2_EnSel	17
#define MPS_TIMER3_EnSel	19
#define MPS_TIMER4_EnSel	21

#define MAX_TIMER		2
#define MAX_PERIOD		699050
#define TICKS_PER_uSEC		1

/* 
 *  These are useconds NOT ticks.  
 */
#define mSEC_1                          1000
#define mSEC_5                          (mSEC_1 * 5)
#define mSEC_10                         (mSEC_1 * 10)
#define mSEC_25                         (mSEC_1 * 25)
#define SEC_1                           (mSEC_1 * 1000)

#endif	/* __ASM_ARCH_PLATFORM_H */
