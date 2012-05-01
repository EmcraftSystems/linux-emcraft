/*
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

#include <linux/clockchips.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/hardware/cortexm3.h>
#include <asm/hardware/lpc_clockevents.h>
#include <mach/lpc18xx.h>
#include <mach/clock.h>

#define LPC18XX_TIMER0_IRQ	12
#define LPC18XX_TIMER0_BASE	0x40084000

/*
 * Source clock init
 */
static void clocksource_tmr_init(void)
{
	/*
	 * Use the Cortex-M3 SysTick timer
	 */
	cortex_m3_register_systick_clocksource(
		lpc18xx_clock_get(CLOCK_SYSTICK));
}

/*
 * Clockevents init (sys timer)
 */
static void clockevents_tmr_init(void)
{
	/*
	 * Init clockevents (sys timer)
	 */
	lpc_clockevents_tmr_init(
		LPC18XX_TIMER0_BASE,
		lpc18xx_clock_get(CLOCK_CCLK),
		LPC18XX_TIMER0_IRQ);
}

/*
 * Initialize the timer systems of the LPC18xx/LPC43xx
 */
void __init lpc18xx_timer_init(void)
{
	/*
	 * Configure the LPC18xx clocks, `lpc18xx_clock_get()` should
	 * now return correct values.
	 */
	lpc18xx_clock_init();

	/*
	 * Init clocksource
	 */
	clocksource_tmr_init();

	/*
	 * Init clockevents (sys timer)
	 */
	clockevents_tmr_init();
}
