/*
 * linux/arch/arm/mach-a2f/timer.c
 *
 * Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>

#include <mach/timer.h>
#include <mach/clock.h>
#include <mach/a2f.h>

/* 
 * Provide a description of the the SmartFusion timer hardware interfaces.
 */
#define MSS_TIMER_BASE	0x40005000
#define MSS_TIMER1_IRQ	20

struct mss_timer {
	unsigned int	tim1_val;
	unsigned int	tim1_loadval;
	unsigned int	tim1_bgloadval;
	unsigned int	tim1_ctrl;
	unsigned int	tim1_ris;
	unsigned int	tim1_mis;
	unsigned int	tim2_val;
	unsigned int	tim2_loadval;
	unsigned int	tim2_bgloadval;
	unsigned int	tim2_ctrl;
	unsigned int	tim2_ris;
	unsigned int	tim2_mis;
};

#define MSS_TIMER	((volatile struct mss_timer *)(MSS_TIMER_BASE))

#define TIMER_CTRL_ENBL		(1<<0)
#define TIMER_CTRL_ONESHOT	(1<<1)
#define TIMER_CTRL_INTR		(1<<2)
#define TIMER_RIS_ACK		(1<<0)
#define TIMER_RST_CLR		(1<<6)

/*
 * Reference clock for the SmartFusion Timers.
 */
static unsigned int timer_ref_clk;

/*
 * Calculates a clocksource shift from hz and # of bits a clock uses.
 * I took this routine from a kernel patch (apparently, it hasn't
 * made its way to this kernel yet).
 */
static u32 timer_clocksource_hz2shift(u32 bits, u32 hz)
{
	u64 temp;

	for (; bits > 0; bits--) {
		temp = (u64) NSEC_PER_SEC << bits;
		do_div(temp, hz);
		if ((temp >> 32) == 0)
			break;
	}
	return bits;
}

/*
 * Set the mode for the system tick timer
 */
static void timer_1_set_mode(
	enum clock_event_mode mode, struct clock_event_device *clk)
{
	unsigned long ctrl;
	unsigned long flags;

	switch(mode) {

	case CLOCK_EVT_MODE_PERIODIC:
	case CLOCK_EVT_MODE_RESUME:

		/*
		 * Kernel ticker rate: 100Hz.
		 * Enable interrupts, Periodic Mode, Timer Enabled
		 */
		raw_local_irq_save(flags);
		MSS_TIMER->tim1_loadval = timer_ref_clk / HZ;
		ctrl = TIMER_CTRL_ENBL | TIMER_CTRL_INTR;
		MSS_TIMER->tim1_ctrl = ctrl;
		raw_local_irq_restore(flags);
		break;

	case CLOCK_EVT_MODE_ONESHOT:

		raw_local_irq_save(flags);
		MSS_TIMER->tim1_loadval = 0xFFFFFFFF;
		ctrl = TIMER_CTRL_ENBL | TIMER_CTRL_ONESHOT | TIMER_CTRL_INTR;
		MSS_TIMER->tim1_ctrl = ctrl;
		raw_local_irq_restore(flags);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:

		/*
		 * Disable the timer.
		 */
		raw_local_irq_save(flags);
		ctrl = MSS_TIMER->tim1_ctrl & ~TIMER_CTRL_ENBL;
		MSS_TIMER->tim1_ctrl = ctrl;
		raw_local_irq_restore(flags);
		break;
	}
}

/*
 * TBD
 */
static int timer_1_set_next_event(
	unsigned long delta, struct clock_event_device *c)
{
	unsigned long flags;

	raw_local_irq_save(flags);
#if 0
	MSS_TIMER->tim1_loadval = 50 * delta;
#else
	MSS_TIMER->tim1_loadval = delta;
#endif
	raw_local_irq_restore(flags);

	return 0;
}

/*
 * A clock_event_device structure for MSS Timer1
 */
static struct clock_event_device timer_1_clockevent = {
	.name           = "mss_timer1_clockevent",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode       = timer_1_set_mode,
	.set_next_event	= timer_1_set_next_event,
	.rating         = 200,
	.cpumask        = cpu_all_mask,
};

/* 
 * Register clock_event_device to MSS Timer1
 */
static void __init timer_1_clockevents_init(unsigned int irq)
{
	timer_1_clockevent.irq = irq;
#if 1
	timer_1_clockevent.shift = 
		timer_clocksource_hz2shift(32, timer_ref_clk);
	timer_1_clockevent.mult =
		clocksource_hz2mult(timer_ref_clk, timer_1_clockevent.shift);
	timer_1_clockevent.max_delta_ns =
		clockevent_delta2ns(0xFFFFFFFF, &timer_1_clockevent);
	timer_1_clockevent.min_delta_ns =
		clockevent_delta2ns(0x1, &timer_1_clockevent);
#else
	timer_1_clockevent.shift = 0;
	timer_1_clockevent.mult = 1;
	timer_1_clockevent.min_delta_ns = 50;
	timer_1_clockevent.max_delta_ns = 0x00FFFFFF;
#endif

	clockevents_register_device(&timer_1_clockevent);
}

/*
 * IRQ handler for MSS Timer2 (system timer)
 */
static irqreturn_t timer_1_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer_1_clockevent;

	/*
	 * Clear the interrupt.
	 */
	MSS_TIMER->tim1_ris = TIMER_RIS_ACK;

	/*
 	 * Handle the event.
 	 */
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

/*
 * An irqaction data structure for the system timer interrupt
 */
static struct irqaction timer_1_irq = {
	.name           = "mss_timer1_irq",
	.flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler        = timer_1_interrupt,
};

/*
 * Start a systicker using Timer1 in 32-bit mode
 */
static void __init timer_ticker_init(void)
{	
	unsigned int irq;

	/*
	 * We will use the AFS System Timer 1 in the 32-bit mode for
	 * the System Ticker.
	 * ...
	 * Another (obvious) candidatate would the Cortex-M3 SysTick,
	 * however the current ARMv7m kernel doesn't use it for
	 * some reason.
	 */
	irq = MSS_TIMER1_IRQ;

	/*
	 * 
	 */
	timer_1_clockevents_init(irq);

	/*
	 * Unmask the Timer 1 interrupts at NVIC (interrupst are still
	 * disabled globally in the kernel). Provide the interrupt handler
	 * for the timer interrupt.
	 */
	setup_irq(irq, &timer_1_irq);
}

/*
 * Read the current value of MSS Timer2
 */
static cycle_t timer_2_read(struct clocksource *c)
{
	return ~MSS_TIMER->tim2_val;
}

/*
 * A clocksource structure for MSS Timer2
 */
static struct clocksource timer_2_clocksource= {
	.name	= "mss_timer2",
	.rating	= 200,
	.read	= timer_2_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.mult	= 0,
	.shift	= 0,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS | CLOCK_SOURCE_VALID_FOR_HRES,
};

/*
 * Start a clocksource using Timer2 in 32-bit mode
 */
static void __init timer_clocksource_init(void)
{	
	/*
 	 * No interrupts, periodic mode, load with the largest number
 	 * that fits into the 32-bit timer
 	 */
	MSS_TIMER->tim2_loadval = 0xFFFFFFFF;
	MSS_TIMER->tim2_ctrl = TIMER_CTRL_ENBL;

	/*
 	 * Calculate shift and mult using helper functions.
 	 * Supposedly, these helpers get us best values for
 	 * the number of bits in the timer and the reference clock
 	 */
	timer_2_clocksource.shift = 
		timer_clocksource_hz2shift(32, timer_ref_clk);
	timer_2_clocksource.mult =
		clocksource_hz2mult(timer_ref_clk, timer_2_clocksource.shift);

	/*
 	 * Register the clocksource with the timekeeper
 	 */
	clocksource_register(&timer_2_clocksource);
}

/*
 * Initialize the timer systems of the SmartFusion.
 */
void __init a2f_timer_init(void)
{
	/*
	 * Configure the SmartFusion clocks 
	 */
	a2f_clock_init();

	/*
	 * Timers of SmartFision are clocked by PCLK0
	 */
	timer_ref_clk = a2f_clock_get(CLCK_PCLK0);

	/*
 	 * Register and start a clocksource
 	 */
	timer_clocksource_init();

	/*
 	 * Register and start a system ticker
 	 */
	timer_ticker_init();

	/*
	 * Allow SystemTimer (including Timer1 & Timer2) to count
	 * (bring it out of the power-up reset)
	 */
	A2F_SYSREG->soft_rst_cr &= ~TIMER_RST_CLR;
}
