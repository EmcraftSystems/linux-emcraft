/*
 *  linux/arch/arm/mach-a2f/timer.c
 *
 *  Copyright (C) 2010 Vladimir Khusainov, Emcraft Systems
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
 * We will user Timer1 of the SmartFusion (in the 32 bit mode)
 * as the system ticker.
 * Provide a description of the the timer hardware interfaces.
 */

#define MSS_TIMER1_BASE	0x40005000
#define MSS_TIMER1_IRQ	20

struct mss_timer1 {
	unsigned int	val;
	unsigned int	loadval;
	unsigned int	bgloadval;
	unsigned int	ctrl;
	unsigned int	ris;
	unsigned int	mis;
};

#define MSS_TIMER1	((volatile struct mss_timer1 *)(MSS_TIMER1_BASE))

#define TIMER_CTRL_ENBL	(1<<0)
#define TIMER_CTRL_PRDC	(0<<1)
#define TIMER_CTRL_INTR	(1<<2)
#define TIMER_RIS_ACK	(1<<0)
#define TIMER_RST_CLR	(1<<6)

/*
 * Set the frequency and periodic mode for the system tick timer.
 */

static void mss_timer1_set_mode(enum clock_event_mode mode,
                                struct clock_event_device *clk)
{
	unsigned long ctrl;

	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		/*
		 * Kernel ticker rate. Timer1 is clocked from PCLK0.
		 * TO-DO: is the Linux system clock rate configurable?
		 */
		MSS_TIMER1->loadval = a2f_clock_get(CLCK_PCLK0) / HZ;
		/*
		 * Enable interrupts, Periodic Mode, Timer Enabled
		 */
		ctrl = TIMER_CTRL_ENBL | TIMER_CTRL_PRDC | TIMER_CTRL_INTR;
		break;
	default:
		/*
		 * Disable the timer.
		 */
		ctrl = MSS_TIMER1->ctrl & ~TIMER_CTRL_ENBL;
		break;
	}

	MSS_TIMER1->ctrl = ctrl;
}

/*
 * Description of the system clock timer operating parameters.
 */

static struct clock_event_device mss_timer1_clockevent = {
	.name           = "A2F MSS Timer1 ClockEvent",
	.shift          = 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC,
	.set_mode       = mss_timer1_set_mode,
	.rating         = 300,
	.cpumask        = cpu_all_mask,
};

/* 
 * Register and start the system clock timer.
 */

static void __init mss_timer1_clockevents_init(unsigned int timer_irq)
{
	mss_timer1_clockevent.irq = timer_irq;
	mss_timer1_clockevent.mult =
		div_sc(1000000, NSEC_PER_SEC, mss_timer1_clockevent.shift);
	mss_timer1_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &mss_timer1_clockevent);
	mss_timer1_clockevent.min_delta_ns =
		clockevent_delta2ns(0xf, &mss_timer1_clockevent);

	clockevents_register_device(&mss_timer1_clockevent);
}

/*
 * IRQ handler for the system clock timer.
 */

static irqreturn_t mss_timer1_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &mss_timer1_clockevent;

	/*
	 * Clear the interrupt.
	 */
	MSS_TIMER1->ris = TIMER_RIS_ACK;

	/*
 	 * Handle the event.
 	 */
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

/*
 * The IRQ handler data structure for the system clock interrupt.
 */

static struct irqaction mss_timer1_irq = {
	.name           = "A2F MSS Timer1 IRQ",
	.flags          = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler        = mss_timer1_interrupt,
};

/*
 * Initialize the timer systems of the SmartFusion.
 */

void __init a2f_timer_init(void)
{
	unsigned int timer_irq;

	/*
	 * We will use the AFS System Timer 1 in the 32-bit mode for
	 * the System Ticker.
	 * ...
	 * Another (obvious) candidatate would the Cortex-M3 SysTick,
	 * however the current ARMv7m kernel doesn't use it for
	 * some reason.
	 */
	timer_irq = MSS_TIMER1_IRQ;

	/*
	 * Unmask the Timer 1 interrupts at NVIC (interrupst are still
	 * disabled globally in the kernel). Provide the interrupt handler
	 * for the timer interrupt.
	 */
	setup_irq(timer_irq, &mss_timer1_irq);

	/*
	 * Register and start the System Ticker.
	 */
	mss_timer1_clockevents_init(timer_irq);

	/*
	 * Allow SystemTimer (Timer1 included) to count
	 * (bring it out of the power-up reset)
	 */
	A2F_SYSREG->soft_rst_cr &= ~TIMER_RST_CLR;
}

/*
 * End of File
 */
