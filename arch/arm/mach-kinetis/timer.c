/*
 * (C) Copyright 2012
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
#include <linux/sched.h>

#include <asm/hardware/cortexm3.h>
#include <mach/kinetis.h>
#include <mach/power.h>
#include <mach/clock.h>

/* PIT Channel 0 IRQ */
#define KINETIS_PIT0_IRQ	68

/* PIT Channer number used for clockevent */
#define KINETIS_CLOCKEVENT_PIT	0
/* IRQ for the PIT channel used by clockevent */
#define KINETIS_CLOCKEVENT_IRQ	KINETIS_PIT0_IRQ

/*
 * PIT Timer Control Register
 */
/* Timer Interrupt Enable Bit */
#define KINETIS_PIT_TCTRL_TIE_MSK	(1 << 1)
/* Timer Enable Bit */
#define KINETIS_PIT_TCTRL_TEN_MSK	(1 << 0)
/*
 * PIT Timer Flag Register
 */
/* Timer Interrupt Flag */
#define KINETIS_PIT_TFLG_TIF_MSK	(1 << 0)

/*
 * Periodic Interrupt Timer (PIT) registers
 */
struct kinetis_pit_channel_regs {
	u32 ldval;	/* Timer Load Value Register */
	u32 cval;	/* Current Timer Value Register */
	u32 tctrl;	/* Timer Control Register */
	u32 tflg;	/* Timer Flag Register */
};

struct kinetis_pit_regs {
	u32 mcr;	/* PIT Module Control Register */
	u32 rsv0[63];
	struct kinetis_pit_channel_regs ch[4];
};

/*
 * PIT registers base
 */
#define KINETIS_PIT_BASE		(KINETIS_AIPS0PERIPH_BASE + 0x00037000)
#define KINETIS_PIT			((volatile struct kinetis_pit_regs *) \
					KINETIS_PIT_BASE)

/*
 * Initialize a PIT channel, but do not enable it
 */
static void kinetis_pit_init(unsigned int timer, u32 ticks)
{
	volatile struct kinetis_pit_channel_regs *timer_regs =
		&KINETIS_PIT->ch[timer];

	/*
	 * Enable power on PIT
	 */
	kinetis_periph_enable(KINETIS_CG_PIT, 1);

	/*
	 * Enable the PIT module clock
	 */
	KINETIS_PIT->mcr = 0;

	/* Disable the timer and its interrupts */
	timer_regs->tctrl = 0;
	/* Clear the interrupt for the match channel 0 */
	timer_regs->tflg = KINETIS_PIT_TFLG_TIF_MSK;

	/* Set the Load Value register */
	timer_regs->ldval = ticks;
	/* Load the current timer value */
	timer_regs->cval = 0;
	/* Enable the interrupt for the PIT channel */
	timer_regs->tctrl = KINETIS_PIT_TCTRL_TIE_MSK;
}

/*
 * Enable or disable a PIT channel
 */
static void kinetis_pit_enable(unsigned int timer, int enable)
{
	volatile struct kinetis_pit_channel_regs *timer_regs =
		&KINETIS_PIT->ch[timer];

	if (enable)
		timer_regs->tctrl |= KINETIS_PIT_TCTRL_TEN_MSK;
	else
		timer_regs->tctrl &= ~KINETIS_PIT_TCTRL_TEN_MSK;
}

/*
 * Clocksource device
 */
/*
 * Source clock init
 */
static void clocksource_tmr_init(void)
{
	/*
	 * Use the Cortex-M3 SysTick timer
	 */
	cortex_m3_register_systick_clocksource(
		kinetis_clock_get(CLOCK_SYSTICK));
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
		kinetis_pit_enable(KINETIS_CLOCKEVENT_PIT, 1);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		kinetis_pit_enable(KINETIS_CLOCKEVENT_PIT, 0);
		break;
	}
}

/*
 * Configure the timer to generate an interrupt in the specified amount of ticks
 */
static int clockevent_tmr_set_next_event(
	unsigned long delta, struct clock_event_device *c)
{
	unsigned long flags;

	raw_local_irq_save(flags);
	kinetis_pit_init(KINETIS_CLOCKEVENT_PIT, delta);
	kinetis_pit_enable(KINETIS_CLOCKEVENT_PIT, 1);
	raw_local_irq_restore(flags);

	return 0;
}

/*
 * Kinetis System Timer device
 */
static struct clock_event_device clockevent_tmr = {
	.name		= "kinetis-pit0",
	.rating		= 200,
	.irq		= KINETIS_CLOCKEVENT_IRQ,
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= clockevent_tmr_set_mode,
	.set_next_event	= clockevent_tmr_set_next_event,
	.cpumask	= cpu_all_mask,
};

/*
 * Timer IRQ handler
 */
static irqreturn_t clockevent_tmr_irq_handler(int irq, void *dev_id)
{
	volatile struct kinetis_pit_channel_regs *timer_regs =
		&KINETIS_PIT->ch[KINETIS_CLOCKEVENT_PIT];

	/*
	 * Clear the interrupt
	 */
	timer_regs->tflg = KINETIS_PIT_TFLG_TIF_MSK;

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
	.name		= "Kinetis Kernel Time Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= clockevent_tmr_irq_handler,
};

/*
 * Clockevents init (sys timer)
 */
static void clockevents_tmr_init(void)
{
	const u64 max_delay_in_sec = 5;

	/* PIT runs at the Bus clock rate */
	u32 pit_clk = kinetis_clock_get(CLOCK_PCLK);

	/* Initialize PIT channel 0 */
	kinetis_pit_init(KINETIS_CLOCKEVENT_PIT, pit_clk / HZ - 1);

	/* Enable PIT channel 0 */
	kinetis_pit_enable(KINETIS_CLOCKEVENT_PIT, 1);

	/* Setup, and enable IRQ */
	setup_irq(KINETIS_CLOCKEVENT_IRQ, &clockevent_tmr_irqaction);

	/*
	 * Set the fields required for the set_next_event method (tickless kernel support)
	 */
	clockevents_calc_mult_shift(&clockevent_tmr, pit_clk, max_delay_in_sec);
	clockevent_tmr.max_delta_ns = max_delay_in_sec * NSEC_PER_SEC;
	clockevent_tmr.min_delta_ns = clockevent_delta2ns(0xF, &clockevent_tmr);

	clockevents_register_device(&clockevent_tmr);
}

/*
 * Initialize the timer systems of the Freescale Kinetis MCU
 */
void __init kinetis_timer_init(void)
{
	/*
	 * Configure the Kinetis clocks, `kinetis_clock_get()` should
	 * now return correct values.
	 */
	kinetis_clock_init();

	/*
	 * Init clocksource
	 */
	clocksource_tmr_init();

	/*
	 * Init clockevents (sys timer)
	 */
	clockevents_tmr_init();
}
