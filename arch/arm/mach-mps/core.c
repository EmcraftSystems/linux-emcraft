/*
 *  linux/arch/arm/mach-mps/core.c
 *
 *  Copyright (C) 2009 ARM Limited
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
#include <linux/dma-mapping.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/amba/mmci.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/smsc911x.h>
#include <linux/ata_platform.h>
#include <linux/delay.h>

#include <asm/clkdev.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/icst307.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include <mach/platform.h>
#include <mach/hardware.h>

#include <asm/hardware/gic.h>

#include "clock.h"
#include "core.h"

#define MPS_REFCOUNTER	(__io_address(MPS_SYS_BASE) + MPS_SYS_CNT25MHz_OFFSET)

/*
 * This is the MPS sched_clock implementation. This has a resolution of 40ns.
 */
unsigned long long sched_clock(void)
{
	return (unsigned long long)readl(MPS_REFCOUNTER) * 40;
}

#define MPS_FLASHCTRL    (__io_address(MPS_SYS_BASE) + MPS_SYS_FLASH_OFFSET)

static struct flash_platform_data mps_flash_data = {
	.map_name		= "cfi_probe",
	.width			= 4,
};

struct platform_device mps_flash_device = {
	.name			= "armflash",
	.id			= 0,
	.dev			= {
		.platform_data	= &mps_flash_data,
	},
};

int mps_flash_register(struct resource *res, u32 num)
{
	mps_flash_device.resource = res;
	mps_flash_device.num_resources = num;
	return platform_device_register(&mps_flash_device);
}

static struct smsc911x_platform_config smsc911x_config = {
	.flags		= SMSC911X_USE_32BIT,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct platform_device mps_eth_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= 2,
};

int mps_eth_register(const char *name, struct resource *res)
{
	if (name)
		mps_eth_device.name = name;
	mps_eth_device.resource = res;
	if (strcmp(mps_eth_device.name, "smsc911x") == 0)
		mps_eth_device.dev.platform_data = &smsc911x_config;

	return platform_device_register(&mps_eth_device);
}

struct platform_device mps_usb_device = {
	.name			= "isp1760",
	.num_resources		= 2,
};

int mps_usb_register(struct resource *res)
{
	mps_usb_device.resource = res;
	return platform_device_register(&mps_usb_device);
}

static struct resource mps_i2c_resource = {
	.start		= MPS_I2C_BASE,
	.end		= MPS_I2C_BASE + SZ_4K - 1,
	.flags		= IORESOURCE_MEM,
};

struct platform_device mps_i2c_device = {
	.name		= "versatile-i2c",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &mps_i2c_resource,
};

static struct i2c_board_info mps_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("rtc-ds1307", 0xd0 >> 1),
		.type = "ds1338",
	},
};

static int __init mps_i2c_init(void)
{
	return i2c_register_board_info(0, mps_i2c_board_info,
				       ARRAY_SIZE(mps_i2c_board_info));
}
arch_initcall(mps_i2c_init);

static unsigned int mps_mmc_status(struct device *dev)
{
	return readl(__io_address(MPS_MMCI_BASE)) & 1;
}

struct mmci_platform_data mps_mmc0_plat_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.status		= mps_mmc_status,
};

/* TODO: check whether CLCD is present on this board */
static struct clk oscvco_clk = {
};

/*
 * MPS fixed clocks.
 */
static struct clk uart_clk = {
	.rate	= 25000000,
};

static struct clk mmci_clk = {
	.rate	= 25000000,
};

static struct clk_lookup lookups[] = {
	{	/* UART0 */
		.dev_id		= "dev:uart0",
		.clk		= &uart_clk,
	}, {	/* UART1 */
		.dev_id		= "dev:uart1",
		.clk		= &uart_clk,
	}, {	/* UART2 */
		.dev_id		= "dev:uart2",
		.clk		= &uart_clk,
	}, {	/* UART3 */
		.dev_id		= "fpga:uart3",
		.clk		= &uart_clk,
	}, {	/* MMC0 */
		.dev_id		= "fpga:mmc0",
		.clk		= &mmci_clk,
	}, {	/* CLCD */
		.dev_id		= "dev:clcd",
		.clk		= &oscvco_clk,
	}
};

static int __init clk_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);
	return 0;
}
arch_initcall(clk_init);

static struct clcd_panel vga = {
	.mode		= {
		.name		= "VGA",
		.refresh	= 60,
		.xres		= 640,
		.yres		= 480,
		.pixclock	= 39721,
		.left_margin	= 64,
		.right_margin	= 16,
		.upper_margin	= 13,
		.lower_margin	= 3,
		.hsync_len	= 80,
		.vsync_len	= 4,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_BCD | TIM2_IPC,
	.cntl		= CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
	.bpp		= 16,
};

static int mps_clcd_setup(struct clcd_fb *fb)
{
	fb->panel		= &vga;
	fb->fb.fix.smem_start	= (unsigned long)MPS_DMC_BASE;
	fb->fb.screen_base	= (char __iomem *)MPS_DMC_BASE;
	fb->fb.fix.smem_len	= 640 * 480 * 2;	/* VGA, 16bpp */

	return 0;
}

static int mps_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	return dma_mmap_writecombine(&fb->dev->dev, vma,
				     fb->fb.screen_base,
				     fb->fb.fix.smem_start,
				     fb->fb.fix.smem_len);
}

static void mps_clcd_remove(struct clcd_fb *fb)
{
}

struct clcd_board clcd_plat_data = {
	.name		= "MPS",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.setup		= mps_clcd_setup,
	.mmap		= mps_clcd_mmap,
	.remove		= mps_clcd_remove,
};

/*
 * Where is the timer (VA)?
 */
void __iomem *timer0_va_base;
void __iomem *timer1_va_base;
void __iomem *timer2_va_base;
void __iomem *timer3_va_base;

/*
 * How long is the timer interval?
 */
#define TIMER_INTERVAL	(TICKS_PER_uSEC * mSEC_10)
#if TIMER_INTERVAL >= 0x100000
#define TIMER_RELOAD	(TIMER_INTERVAL >> 8)
#define TIMER_DIVISOR	(TIMER_CTRL_DIV256)
#define TICKS2USECS(x)	(256 * (x) / TICKS_PER_uSEC)
#elif TIMER_INTERVAL >= 0x10000
#define TIMER_RELOAD	(TIMER_INTERVAL >> 4)		/* Divide by 16 */
#define TIMER_DIVISOR	(TIMER_CTRL_DIV16)
#define TICKS2USECS(x)	(16 * (x) / TICKS_PER_uSEC)
#else
#define TIMER_RELOAD	(TIMER_INTERVAL)
#define TIMER_DIVISOR	(TIMER_CTRL_DIV1)
#define TICKS2USECS(x)	((x) / TICKS_PER_uSEC)
#endif

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	unsigned long ctrl;

	switch(mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(TIMER_RELOAD, timer0_va_base + TIMER_LOAD);

		ctrl = TIMER_CTRL_PERIODIC;
		ctrl |= TIMER_CTRL_32BIT | TIMER_CTRL_IE | TIMER_CTRL_ENABLE;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl = TIMER_CTRL_ONESHOT;
		ctrl |= TIMER_CTRL_32BIT | TIMER_CTRL_IE;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		ctrl = 0;
	}

	writel(ctrl, timer0_va_base + TIMER_CTRL);
}

static int timer_set_next_event(unsigned long evt,
				struct clock_event_device *unused)
{
	unsigned long ctrl = readl(timer0_va_base + TIMER_CTRL);

	writel(evt, timer0_va_base + TIMER_LOAD);
	writel(ctrl | TIMER_CTRL_ENABLE, timer0_va_base + TIMER_CTRL);

	return 0;
}

static struct clock_event_device timer0_clockevent =	 {
	.name		= "timer0",
	.shift		= 32,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= timer_set_mode,
	.set_next_event	= timer_set_next_event,
	.rating		= 300,
	.cpumask	= cpu_all_mask,
};

static void __init mps_clockevents_init(unsigned int timer_irq)
{
	timer0_clockevent.irq = timer_irq;
	timer0_clockevent.mult =
		div_sc(1000000, NSEC_PER_SEC, timer0_clockevent.shift);
	timer0_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &timer0_clockevent);
	timer0_clockevent.min_delta_ns =
		clockevent_delta2ns(0xf, &timer0_clockevent);

	clockevents_register_device(&timer0_clockevent);
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t mps_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer0_clockevent;

	/* clear the interrupt */
	writel(1, timer0_va_base + TIMER_INTCLR);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction mps_timer_irq = {
	.name		= "MPS Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= mps_timer_interrupt,
};

static cycle_t mps_get_cycles(struct clocksource *cs)
{
	return ~readl(timer3_va_base + TIMER_VALUE);
}

static struct clocksource clocksource_mps = {
	.name	= "timer3",
	.rating	= 200,
	.read	= mps_get_cycles,
	.mask	= CLOCKSOURCE_MASK(32),
	.shift	= 20,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init mps_clocksource_init(void)
{
	/* setup timer 0 as free-running clocksource */
	writel(0, timer3_va_base + TIMER_CTRL);
	writel(0xffffffff, timer3_va_base + TIMER_LOAD);
	writel(0xffffffff, timer3_va_base + TIMER_VALUE);
	writel(TIMER_CTRL_32BIT | TIMER_CTRL_ENABLE | TIMER_CTRL_PERIODIC,
		timer3_va_base + TIMER_CTRL);

	clocksource_mps.mult =
		clocksource_khz2mult(1000, clocksource_mps.shift);
	clocksource_register(&clocksource_mps);
}

/*
 * Set up the clock source and clock events devices
 */
void __init mps_timer_init(unsigned int timer_irq)
{
	u32 val;

	/* 
	 * set clock frequency: 
	 *	MPS_REFCLK is 32KHz
	 *	MPS_TIMCLK is 1MHz
	 */
	val = readl(__io_address(MPS_SCTL_BASE));
	writel((MPS_TIMCLK << MPS_TIMER1_EnSel) |
	       (MPS_TIMCLK << MPS_TIMER2_EnSel) | 
	       (MPS_TIMCLK << MPS_TIMER3_EnSel) |
	       (MPS_TIMCLK << MPS_TIMER4_EnSel) | val,
	       __io_address(MPS_SCTL_BASE));

	/*
	 * Initialise to a known state (all timers off)
	 */
	writel(0, timer0_va_base + TIMER_CTRL);
	writel(0, timer1_va_base + TIMER_CTRL);
	writel(0, timer2_va_base + TIMER_CTRL);
	writel(0, timer3_va_base + TIMER_CTRL);

	/* 
	 * Make irqs happen for the system timer
	 */
	setup_irq(timer_irq, &mps_timer_irq);

	mps_clocksource_init();
	mps_clockevents_init(timer_irq);
}
