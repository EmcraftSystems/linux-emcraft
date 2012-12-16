/*
 * Copyright (C) 2012 Emcraft Systems
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "musb_core.h"

/******************************************************************************
 * Local prototypes
 ******************************************************************************/

static void m2s_usb_vbus_power(struct musb *musb, int is_on, int sleeping);
static void m2s_usb_vbus_set(struct musb *musb, int is_on);
static int  m2s_usb_platform_resume(struct musb *musb);

/******************************************************************************
 * MUSB framework API
 ******************************************************************************/

void musb_platform_enable(struct musb *musb)
{

}
void musb_platform_disable(struct musb *musb)
{

}

int musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	return 0;
}

int __init musb_platform_init(struct musb *musb)
{
	usb_nop_xceiv_register();

	/* We require some kind of external transceiver, hooked
	 * up through ULPI.  TWL4030-family PMICs include one,
	 * which needs a driver, drivers aren't always needed.
	 */
	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv) {
		pr_err("HS USB OTG: no transceiver configured\n");
		return -ENODEV;
	}

	m2s_usb_platform_resume(musb);

	m2s_usb_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);

	if (is_host_enabled(musb))
		musb->board_set_vbus = m2s_usb_vbus_set;

	return 0;
}

int musb_platform_suspend(struct musb *musb)
{
	return 0;
}

int musb_platform_exit(struct musb *musb)
{

	m2s_usb_vbus_power(musb, 0, 1);

	musb_platform_suspend(musb);

	clk_put(musb->clock);
	musb->clock = NULL;

	return 0;
}

/******************************************************************************
 * Functions local to this module
 ******************************************************************************/

static void m2s_usb_vbus_power(struct musb *musb, int is_on, int sleeping)
{

}

static void m2s_usb_vbus_set(struct musb *musb, int is_on)
{
	u8	devctl;

	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */
	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv->default_a = 1;
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */
		musb->xceiv->default_a = 0;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x\n",
		otg_state_string(musb),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static int m2s_usb_platform_resume(struct musb *musb)
{
	return 0;
}
