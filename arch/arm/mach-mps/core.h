/*
 *  linux/arch/arm/mach-mps/core.h
 *
 *  Copyright (C) 2004 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
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

#ifndef __ASM_ARCH_MPS_H
#define __ASM_ARCH_MPS_H

#include <linux/amba/bus.h>
#include <linux/io.h>

#include <asm/leds.h>

#define AMBA_DEVICE(name,busid,base,plat)			\
static struct amba_device name##_device = {			\
	.dev		= {					\
		.coherent_dma_mask = ~0,			\
		.init_name = busid,				\
		.platform_data = plat,				\
	},							\
	.res		= {					\
		.start	= MPS_##base##_BASE,			\
		.end	= (MPS_##base##_BASE) + SZ_4K - 1,	\
		.flags	= IORESOURCE_MEM,			\
	},							\
	.dma_mask	= ~0,					\
	.irq		= base##_IRQ,				\
	/* .dma		= base##_DMA,*/				\
}

extern struct platform_device mps_flash_device;
extern struct platform_device mps_cf_device;
extern struct platform_device mps_i2c_device;
extern struct mmci_platform_data mps_mmc0_plat_data;
extern struct clk mps_clcd_clk;
extern struct clcd_board clcd_plat_data;
extern void __iomem *gic_cpu_base_addr;
#ifdef CONFIG_LOCAL_TIMERS
extern void __iomem *twd_base;
#endif
extern void __iomem *timer0_va_base;
extern void __iomem *timer1_va_base;
extern void __iomem *timer2_va_base;
extern void __iomem *timer3_va_base;

extern void mps_leds_event(led_event_t ledevt);
extern void mps_timer_init(unsigned int timer_irq);
extern int mps_flash_register(struct resource *res, u32 num);
extern int mps_eth_register(const char *name, struct resource *res);
extern int mps_usb_register(struct resource *res);

#endif
