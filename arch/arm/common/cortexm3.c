/*
 *  linux/arch/arm/common/cortexm3.c
 *
 * (C) Copyright 2011, 2012
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

#include <linux/types.h>
#include <linux/clockchips.h>

#include <asm/hardware/cortexm3.h>

struct cm3_scb {
	u32	cpuid;
	u32	icsr;
	u32	vtor;
	u32	aircr;
};

#define CM3_SCB_BASE	0xE000ED00
#define CM3_SCB		((volatile struct cm3_scb *)(CM3_SCB_BASE))

#define CM3_AIRCR_VECTKEY		0x5fa
#define CM3_AIRCR_VECTKEY_SHIFT		16
#define CM3_AIRCR_PRIGROUP_MSK		0x7
#define CM3_AIRCR_PRIGROUP_SHIFT	8
#define CM3_AIRCR_SYSRESET		(1<<2)

/*
 * SysTick timer
 */
struct cm3_systick {
	u32 ctrl;			/* Control and Status Register */
	u32 load;			/* Reload Value Register       */
	u32 val;			/* Current Value Register      */
	u32 cal;			/* Calibration Register        */
};

/* SysTick Base Address */
#define CM3_SYSTICK_BASE		0xE000E010
#define CM3_SYSTICK			((volatile struct cm3_systick *) \
					CM3_SYSTICK_BASE)

#define CM3_SYSTICK_LOAD_RELOAD_MSK	(0x00FFFFFF)
#define CM3_SYSTICK_LOAD_RELOAD_BITWIDTH	24
/* System Tick counter enable */
#define CM3_SYSTICK_CTRL_EN		(1 << 0)
/* System Tick clock source selection: 1=CPU, 0=STCLK (external clock pin) */
#define CM3_SYSTICK_CTRL_SYSTICK_CPU	(1 << 2)

/*
 * Perform the low-level reboot.
 */
void cortex_m3_reboot(void)
{
	/*
	 * Perform reset but keep priority group unchanged.
	 */
	CM3_SCB->aircr = (CM3_AIRCR_VECTKEY << CM3_AIRCR_VECTKEY_SHIFT) |
			 (CM3_SCB->aircr &
			  (CM3_AIRCR_PRIGROUP_MSK << CM3_AIRCR_PRIGROUP_SHIFT))
			 | CM3_AIRCR_SYSRESET;
}

#if defined(CONFIG_ARCH_KINETIS) || defined(CONFIG_ARCH_STM32) || \
    defined(CONFIG_ARCH_LPC178X) || defined(CONFIG_ARCH_LPC18XX)
/*
 * The SysTick clocksource is not used on other Cortex-M3 targets,
 * they use other timers.
 */
/*
 * Get current clock source timer value
 */
static cycle_t clocksource_systick_value_get(struct clocksource *c)
{
	/*
	 * The value should be inverted, because the SysTick timer counts down,
	 * and we need a value that counts up.
	 */
	return (cycle_t)(CM3_SYSTICK->val ^ CM3_SYSTICK_LOAD_RELOAD_MSK);
}

/*
 * SysTick clock source device
 */
static struct clocksource clocksource_systick = {
	.name		= "cm3-systick",
	.rating		= 200,
	.read		= clocksource_systick_value_get,
	.mask		= CLOCKSOURCE_MASK(CM3_SYSTICK_LOAD_RELOAD_BITWIDTH),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * Register the SysTick timer as a clocksource
 */
void cortex_m3_register_systick_clocksource(u32 systick_clk)
{
	/*
	 * Configure and enable the SysTick timer if it was not enabled
	 * in the bootloader.
	 */
	CM3_SYSTICK->load = CM3_SYSTICK_LOAD_RELOAD_MSK;
	CM3_SYSTICK->val = 0;
	CM3_SYSTICK->ctrl |= CM3_SYSTICK_CTRL_EN;

	/*
	 * Finalize clocksource initialization and register it
	 */
	clocksource_calc_mult_shift(&clocksource_systick, systick_clk, 4);
	clocksource_register(&clocksource_systick);
}
#endif /* defined(CONFIG_ARCH_XXX) */
