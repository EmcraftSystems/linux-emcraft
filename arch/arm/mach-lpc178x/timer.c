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

#include <linux/clockchips.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <mach/lpc178x.h>
#include <mach/power.h>
#include <mach/clock.h>

/* "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70). */
#define LPC178X_TIMER0_IRQ	1

#define LPC178X_TIMER0_BASE	(LPC178X_APB0PERIPH_BASE + 0x00004000)
#define LPC178X_TIMER1_BASE	(LPC178X_APB0PERIPH_BASE + 0x00008000)

#define LPC178X_CLOCKEVENT_TIMER	0	/* TIMER0 */
#define LPC178X_CLOCKSOURCE_TIMER	1	/* TIMER1 */

/*
 * Timer registers definitions
 */
/*
 * Interrupt Register
 */
#define LPC178X_TIMER_IR_MR0_MSK	(1 << 0)
/*
 * Timer Control Register
 */
#define LPC178X_TIMER_TCR_EN_MSK	(1 << 0)
#define LPC178X_TIMER_TCR_RESET_MSK	(1 << 1)
/*
 * Match Control Register
 */
#define LPC178X_TIMER_MCR_MR0I		(1 << 0)	/* Interrupt */
#define LPC178X_TIMER_MCR_MR0R		(1 << 1)	/* Counter reset */
/*
 * Count Control Register
 */
#define LPC178X_TIMER_CTCR_MODE_TIMER_MSK	0

struct lpc178x_timer_regs {
	u32 ir;		/* Interrupt Register */
	u32 tcr;	/* Timer Control Register */
	u32 tc;		/* Timer Counter */
	u32 pr;		/* Prescale Register */
	u32 pc;		/* Prescale Counter */
	u32 mcr;	/* Match Control Register */
	u32 mr0;	/* Match Register 0 */
	u32 mr1;	/* Match Register 1 */
	u32 mr2;	/* Match Register 2 */
	u32 mr3;	/* Match Register 3 */
	u32 ccr;	/* Capture Control Register */
	u32 cr0;	/* Capture Register 0 */
	u32 cr1;	/* Capture Register 1 */
	u32 rsv0[2];
	u32 emr;	/* External Match Register */
	u32 rsv1[12];
	u32 ctcr;	/* Count Control Register */
};

/*
 * Peripheral clock (PCLK)
 */
static u32 periph_clk;

/*
 * Base addresses of registers for timers
 */
static const u32 timer_base[] = {
	LPC178X_TIMER0_BASE, LPC178X_TIMER1_BASE
};

/*
 * PCONP masks for enabling power on timers using `lpc178x_periph_enable()`
 */
static const u32 timer_pconp_msk[] = {
	LPC178X_SCC_PCONP_PCTIM0_MSK, LPC178X_SCC_PCONP_PCTIM1_MSK
};

static void lpc178x_timer_reset(unsigned int timer)
{
	volatile struct lpc178x_timer_regs *timer_regs;

	/*
	 * Enable power on TIMER0
	 */
	lpc178x_periph_enable(timer_pconp_msk[timer], 1);

	/*
	 * Setup registers base for TIMER0
	 */
	timer_regs = (volatile struct lpc178x_timer_regs *)timer_base[timer];

	/* Reset timer */
	timer_regs->tcr = LPC178X_TIMER_TCR_RESET_MSK;
	timer_regs->tcr = 0;

	timer_regs->tc = 0;	/* Reset the counter */
	timer_regs->pc = 0;	/* Reset the prescale counter */
	timer_regs->mcr = 0;	/* Disable all operations on match events */
	timer_regs->ccr = 0;	/* Disable all capture events */
	timer_regs->emr = 0;	/* Disable all external match events */

	/*
	 * Clear the interrupt for the match channel 0
	 */
	timer_regs->ir = LPC178X_TIMER_IR_MR0_MSK;

	/*
	 * Count mode is PCLK edge
	 */
	timer_regs->ctcr = LPC178X_TIMER_CTCR_MODE_TIMER_MSK;

	/*
	 * Set prescale counter limit to 0 (no prescale)
	 */
	timer_regs->pr = 0;
}

/*
 * Enable or disable a timer
 */
static void lpc178x_timer_enable(unsigned int timer, int enable)
{
	volatile struct lpc178x_timer_regs *timer_regs;
	timer_regs = (volatile struct lpc178x_timer_regs *)timer_base[timer];

	if (enable)
		timer_regs->tcr |= LPC178X_TIMER_TCR_EN_MSK;
	else
		timer_regs->tcr &= ~LPC178X_TIMER_TCR_EN_MSK;
}

/*
 * Clocksource device
 */

/*
 * Get current clock source timer value
 */
static cycle_t clocksource_tmr_value_get(struct clocksource *c)
{
	volatile struct lpc178x_timer_regs *timer_regs;
	timer_regs = (volatile struct lpc178x_timer_regs *)timer_base[LPC178X_CLOCKSOURCE_TIMER];

	return (((cycle_t)timer_regs->tc) << 32) | (cycle_t)timer_regs->pc;
}

/*
 * LPC178x/7x clock source device
 */
static struct clocksource clocksource_tmr = {
	.name		= "lpc178x-timer1",
	.rating		= 200,
	.read		= clocksource_tmr_value_get,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * Calculates a clocksource shift from hz and # of bits a clock uses.
 * Taken from A2F code, and there it had been taken from some kernel
 * patch
 */
static u32 clocksource_hz2shift(u32 bits, u32 hz)
{
	u64	temp;

	for (; bits > 0; bits--) {
		temp = (u64)NSEC_PER_SEC << bits;
		do_div(temp, hz);
		if ((temp >> 32) == 0)
			break;
	}

	return bits;
}

/*
 * Source clock init
 */
static void clocksource_tmr_init(void)
{
	volatile struct lpc178x_timer_regs *timer_regs;
	timer_regs = (volatile struct lpc178x_timer_regs *)timer_base[LPC178X_CLOCKSOURCE_TIMER];

	/* Initialize TIMER1 */
	lpc178x_timer_reset(LPC178X_CLOCKSOURCE_TIMER);

	/*
	 * Make the Timer Counter and the Prescale Counter work together as a
	 * 64-bit counter.
	 */
	timer_regs->pr = 0xFFFFFFFF;

	/*
	 * Finalize clocksource initialization and register it
	 */
	clocksource_tmr.shift = clocksource_hz2shift(32, periph_clk);
	clocksource_tmr.mult  = clocksource_hz2mult(periph_clk, clocksource_tmr.shift);

	clocksource_register(&clocksource_tmr);

	/* Enable TIMER1 */
	lpc178x_timer_enable(LPC178X_CLOCKSOURCE_TIMER, 1);
}

/*
 * Clockevent device
 */
/*
 * Clock event device set mode function
 */
static void clockevent_tmr_set_mode(
	enum clock_event_mode mode, struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		lpc178x_timer_enable(LPC178X_CLOCKEVENT_TIMER, 1);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		lpc178x_timer_enable(LPC178X_CLOCKEVENT_TIMER, 0);
		break;
	}
}

/*
 * LPC178x/7x System Timer device
 */
static struct clock_event_device clockevent_tmr = {
	.name		= "lpc178x-timer0",
	.rating		= 200,
	.irq		= LPC178X_TIMER0_IRQ,
	.features	= CLOCK_EVT_FEAT_PERIODIC,
	.set_mode	= clockevent_tmr_set_mode,
	.cpumask	= cpu_all_mask,
};

/*
 * Timer IRQ handler
 */
static irqreturn_t clockevent_tmr_irq_handler(int irq, void *dev_id)
{
	volatile struct lpc178x_timer_regs *timer_regs;
	timer_regs = (volatile struct lpc178x_timer_regs *)timer_base[LPC178X_CLOCKEVENT_TIMER];

	/*
	 * Clear the interrupt
	 */
	timer_regs->ir = LPC178X_TIMER_IR_MR0_MSK;

	/*
	 * Handle Event
	 */
	clockevent_tmr.event_handler(&clockevent_tmr);

	return IRQ_HANDLED;
}

/*
 * System timer IRQ action
 */
static struct irqaction	clockevent_tmr_irqaction = {
	.name		= "LPC178x/7x Kernel Time Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= clockevent_tmr_irq_handler,
};

/*
 * Clockevents init (sys timer)
 */
static void clockevents_tmr_init(void)
{
	volatile struct lpc178x_timer_regs *timer_regs;
	timer_regs = (volatile struct lpc178x_timer_regs *)timer_base[LPC178X_CLOCKEVENT_TIMER];

	/* Initialize TIMER0 */
	lpc178x_timer_reset(LPC178X_CLOCKEVENT_TIMER);

	/*
	 * Initialize match channel 0
	 */
	/* Set match register */
	timer_regs->mr0 = periph_clk / HZ - 1;
	/* Enable necessary match channels and operations for them */
	timer_regs->mcr = LPC178X_TIMER_MCR_MR0I | LPC178X_TIMER_MCR_MR0R;

	/* Enable TIMER0 */
	lpc178x_timer_enable(LPC178X_CLOCKEVENT_TIMER, 1);

	/* Setup, and enable IRQ */
	setup_irq(LPC178X_TIMER0_IRQ, &clockevent_tmr_irqaction);

	/*
	 * For system timer we don't provide set_next_event method,
	 * so, I guess, setting mult, shift, max_delta_ns, min_delta_ns
	 * makes no sense (I verified that kernel works well without these).
	 * Nevertheless, some clocksource drivers with periodic-mode only do
	 * this. So, let's set them to some values too.
	 */
	clockevents_calc_mult_shift(&clockevent_tmr, periph_clk / HZ, 5);
	clockevent_tmr.max_delta_ns = clockevent_delta2ns(0xFFFFFFF0, &clockevent_tmr);
	clockevent_tmr.min_delta_ns = clockevent_delta2ns(0xF, &clockevent_tmr);

	clockevents_register_device(&clockevent_tmr);
}

/*
 * Initialize the timer systems of the LPC178x/7x
 */
void __init lpc178x_timer_init(void)
{
	/*
	 * Configure the LPC178x/7x clocks, `lpc178x_clock_get()` should
	 * now return correct values.
	 */
	lpc178x_clock_init();

	/*
	 * Get PCLK frequency in Hz
	 */
	periph_clk = lpc178x_clock_get(CLOCK_PCLK);

	/*
	 * Init clocksource
	 */
	clocksource_tmr_init();

	/*
	 * Init clockevents (sys timer)
	 */
	clockevents_tmr_init();
}
