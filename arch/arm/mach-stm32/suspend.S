/*
 * arch/arm/mach-stm32/suspend.S
 *
 * STM32 low-level suspend-to-RAM code. This code is relocated to
 * SRAM in stm32_sram_relocate() during stm32_init() execution in
 * arch/arm/mach-stm32/stm32_platform.c
 *
 * Copyright (C) 2016 Emcraft Systems
 * Yuri Tikhonov, Emcraft Systems, <yur@emcraft.com>
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

#include <mach/gpio.h>

/*
 * This code runs from eSRAM (on-chip SRAM).
 */

	.global stm32_suspend_to_ram
	.global stm32_suspend_moder
	.section .sram.text, "ax"
	.align 4
	.thumb_func

stm32_suspend_to_ram:
	/*
	 * Put DDR into self-refresh
	 */
	ldr r0, fmc_sdcmr
	ldr r1, fmc_sdsr
1:
	ldr r2, [r1]		/* Wait for !FMC_SDSR[BUSY] */
	ands r2, #0x20
	bne 1b

	mov r2, #0x15		/* Program FMC_SDMR[CTB1][MODE] */
	str r2, [r0]

	ldr r2, [r1]		/* Check FMC_SDSR[MODES1] */
	and r2, #0x6
	cmp r2, #0x2
	bne 1b


	/*
	 * Back-up GPIOs, and configure as Analog Inputs
	 */
	ldr r0, gpioa
	ldr r1, backup
	ldr r2, =stm32_suspend_moder
	mov r3, #0x0
2:
	ldr r12, [r0]
	str r12, [r1]
	ldr r12, [r2]
	str r12, [r0]

	add r0, #0x400
	add r1, #0x4
	add r2, #0x4
	add r3, #0x1
	cmp r3, #STM32_GPIO_PORTS
	bne 2b

	/*
	 * Pass memory barriers, and switch to Sleep
	 */
	dsb
	isb
	wfi

	/*
	 * When at this point, we have been woken from Deep Sleep.
	 */
	/*
	 * Restore GPIOs
	 */
	ldr r0, gpioa
	ldr r1, backup
	mov r3, #0x0
3:
	ldr r12, [r1]
	str r12, [r0]

	add r0, #0x400
	add r1, #0x4
	add r3, #0x1
	cmp r3, #STM32_GPIO_PORTS
	bne 3b

	/*
	 * Take DDR out of self-refresh
	 */
	ldr r0, fmc_sdcmr
	ldr r1, fmc_sdsr
4:
	ldr r2, [r1]		/* Wait for !FMC_SDSR[BUSY] */
	ands r2, #0x20
	bne 4b

	mov r2, #0x10		/* Program FMC_SDMR[CTB1][MODE] */
	str r2, [r0]

	ldr r2, [r1]		/* Check FMC_SDSR[MODES1] */
	ands r2, #0x6
	bne 4b

	/*
	 * Return to the kernel code running from DDR
	 */
	dsb
	isb
	bx lr

	.align 4
fmc_sdcmr:
	.word 0xa0000150
fmc_sdsr:
	.word 0xa0000158
gpioa:
	.word 0x40020000
backup:
	.space	STM32_GPIO_PORTS * 4
stm32_suspend_moder:
	.space	STM32_GPIO_PORTS * 4
