/*
 * linux/arch/arm/mach-m2s/timer.c
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
#include <mach/m2s.h>

/*
 * Provide a description of the the SmartFusion2 timer hardware interfaces.
 */
#define MSS_TIMER_BASE	0x40004000
#define MSS_TIMER1_IRQ	14

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
 * Clock event device set mode function
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
 * Configure the timer to generate an interrupt in the specified amount of ticks
 */
static int timer_1_set_next_event(
	unsigned long delta, struct clock_event_device *c)
{
	unsigned long ctrl;
	unsigned long flags;

	raw_local_irq_save(flags);
	MSS_TIMER->tim1_loadval = delta;
	ctrl = TIMER_CTRL_ENBL | TIMER_CTRL_ONESHOT | TIMER_CTRL_INTR;
	MSS_TIMER->tim1_ctrl = ctrl;
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
	const u64 max_delay_in_sec = 5;
	timer_1_clockevent.irq = irq;

	/*
	 * Set the fields required for the set_next_event method (tickless kernel support)
	 */
	clockevents_calc_mult_shift(&timer_1_clockevent, timer_ref_clk,
			max_delay_in_sec);
	timer_1_clockevent.max_delta_ns = max_delay_in_sec * NSEC_PER_SEC;
	timer_1_clockevent.min_delta_ns =
		clockevent_delta2ns(0x1, &timer_1_clockevent);

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
 * Configure clock event device on Timer 1
 */
static void __init timer_clockevent_init(void)
{
	unsigned int irq;

	irq = MSS_TIMER1_IRQ;

	/*
	 * Init clock event device
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
	 * Calculate shift and mult using a helper function.
	 * Supposedly, this helper gets us best values for
	 * conversion between time in nanoseconds and timer ticks
	 */
	clocksource_calc_mult_shift(&timer_2_clocksource, timer_ref_clk, 4);

	/*
	 * Register the clocksource with the timekeeper
	 */
	clocksource_register(&timer_2_clocksource);
}

/*
 * Initialize the timer systems of the SmartFusion.
 */
void __init m2s_timer_init(void)
{
	/*
	 * Allow SystemTimer (including Timer1 & Timer2) to count
	 * (bring it out of the power-up reset)
	 */
	M2S_SYSREG->soft_reset_cr &= ~TIMER_RST_CLR;

	/*
	 * Configure the SmartFusion clocks
	 */
	m2s_clock_init();

	/*
	 * Timers of SmartFision are clocked by PCLK0
	 */
	timer_ref_clk = m2s_clock_get(CLCK_PCLK0);

	/*
	 * Register and start a clocksource
	 */
	timer_clocksource_init();

	/*
	 * Register and start a clock event device
	 */
	timer_clockevent_init();
}
