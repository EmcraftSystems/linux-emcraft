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

static void lpc178x_timer_reset(volatile struct lpc178x_timer_regs *timer_regs)
{
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
static void lpc178x_timer_enable(volatile struct lpc178x_timer_regs *timer_regs, int enable)
{
	if (enable)
		timer_regs->tcr |= LPC178X_TIMER_TCR_EN_MSK;
	else
		timer_regs->tcr &= ~LPC178X_TIMER_TCR_EN_MSK;
}

/*
 * Clockevent-specific code
 */

/*
 * We use this container structure to access `timer_regs` when we only have
 * a pointer to `struct clock_event_device`, for example
 * in `clockevent_tmr_set_mode()`.
 *
 * We also add here `tmr_irqaction` to simplify memory allocation for these
 * structures.
 */
struct lpc_clockevent_device {
	struct clock_event_device clockevent;
	struct irqaction tmr_irqaction;
	volatile struct lpc178x_timer_regs *timer_regs;
};

/*
 * Clock event device set mode function
 */
static void clockevent_tmr_set_mode(
	enum clock_event_mode mode, struct clock_event_device *clk)
{
	volatile struct lpc178x_timer_regs *timer_regs = container_of(
		clk, struct lpc_clockevent_device, clockevent)->timer_regs;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		lpc178x_timer_enable(timer_regs, 1);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		lpc178x_timer_enable(timer_regs, 0);
		break;
	}
}

/*
 * Timer IRQ handler
 */
static irqreturn_t clockevent_tmr_irq_handler(int irq, void *dev_id)
{
	/*
	 * `dev_id` was defined during initialization of irqaction
	 */
	struct lpc_clockevent_device *tmr = dev_id;

	/*
	 * Clear the interrupt
	 */
	tmr->timer_regs->ir = LPC178X_TIMER_IR_MR0_MSK;

	/*
	 * Handle event
	 */
	tmr->clockevent.event_handler(&tmr->clockevent);

	return IRQ_HANDLED;
}

/*
 * Clockevents init (sys timer)
 */
void lpc_clockevents_tmr_init(u32 timer_regs_base, u32 src_clk, int irq)
{
	struct lpc_clockevent_device *tmr;
	volatile struct lpc178x_timer_regs *timer_regs =
		(volatile struct lpc178x_timer_regs *)timer_regs_base;

	tmr = kzalloc(sizeof(struct lpc_clockevent_device), GFP_KERNEL);
	tmr->timer_regs = timer_regs;

	/* Initialize timer */
	lpc178x_timer_reset(timer_regs);

	/*
	 * Initialize match channel 0
	 */
	/* Set match register */
	timer_regs->mr0 = src_clk / HZ - 1;
	/* Enable necessary match channels and operations for them */
	timer_regs->mcr = LPC178X_TIMER_MCR_MR0I | LPC178X_TIMER_MCR_MR0R;

	/* Enable timer */
	lpc178x_timer_enable(timer_regs, 1);

	/* Setup and enable IRQ */
	tmr->tmr_irqaction.name = "LPC Kernel Time Tick";
	tmr->tmr_irqaction.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL;
	tmr->tmr_irqaction.handler = clockevent_tmr_irq_handler;
	tmr->tmr_irqaction.dev_id = tmr;
	setup_irq(irq, &tmr->tmr_irqaction);

	/*
	 * Configure clockevents timer
	 */
	tmr->clockevent.name = "lpc-clockevent";
	tmr->clockevent.rating = 200;
	tmr->clockevent.irq = irq;
	tmr->clockevent.features = CLOCK_EVT_FEAT_PERIODIC;
	tmr->clockevent.set_mode = clockevent_tmr_set_mode;
	tmr->clockevent.cpumask = cpu_all_mask;

	/*
	 * For system timer we don't provide set_next_event method,
	 * so, I guess, setting mult, shift, max_delta_ns, min_delta_ns
	 * makes no sense (I verified that kernel works well without these).
	 * Nevertheless, some clocksource drivers with periodic-mode only do
	 * this. So, let's set them to some values too.
	 */
	clockevents_calc_mult_shift(&tmr->clockevent, src_clk / HZ, 5);
	tmr->clockevent.max_delta_ns = clockevent_delta2ns(0xFFFFFFF0, &tmr->clockevent);
	tmr->clockevent.min_delta_ns = clockevent_delta2ns(0xF, &tmr->clockevent);

	clockevents_register_device(&tmr->clockevent);
}
