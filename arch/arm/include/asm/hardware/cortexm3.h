/*
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

#ifndef _ASM_ARCH_CORTEXM3_H_
#define _ASM_ARCH_CORTEXM3_H_

#ifdef CONFIG_ARM_CORTEXM3

extern void cortex_m3_reboot(void);

#if defined(CONFIG_ARCH_KINETIS) || defined(CONFIG_ARCH_STM32) || \
    defined(CONFIG_ARCH_LPC178X) || defined(CONFIG_ARCH_LPC18XX)
/*
 * The SysTick clocksource is not used on other Cortex-M3 targets,
 * they use other timers.
 */
#include <linux/types.h>
extern void cortex_m3_register_systick_clocksource(u32 systick_clk);
#endif /* defined(CONFIG_ARCH_XXX) */

#endif /* CONFIG_ARM_CORTEXM3 */

#endif /* _ASM_ARCH_CORTEXM3_H_ */
