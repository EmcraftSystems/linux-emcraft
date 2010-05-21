/*
 *  linux/arch/arm/mach-vexpress/core.c
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
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/cnt32_to_63.h>
#include <linux/io.h>
#include <linux/smsc911x.h>
#include <linux/ata_platform.h>
#include <linux/delay.h>
#include <linux/amba/mmci.h>
#include <linux/spinlock.h>
#include <linux/usb/isp1760.h>

#include <asm/clkdev.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/icst307.h>
#include <asm/setup.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include <asm/hardware/gic.h>

#include <mach/hardware.h>
#include <mach/board-pca9.h>
#include <mach/sri.h>
#include "core.h"
#include "clock.h"

/* used by entry-macro.S and platsmp.c */
void __iomem *gic_cpu_base_addr;

static spinlock_t sri_lock;

/*
 * Versatile Express System Register Interface
 */
unsigned int vexpress_sri_transfer( _sri_info_t * sri_info)
{
	unsigned int status;
	unsigned int command;
	unsigned int timeout;

	status = 0;

	/* Make sure we have exclusive access to the micro until
	 * after busy is set */
	spin_lock(&sri_lock);

	/* check if busy */
	if (readl(__io_address(ARM_VEXPRESS_SYS_CFG_CTRL)) & 0x80000000) {
		spin_unlock(&sri_lock);
		return 1;
	}

	/* create cfg command */
	command  = sri_info->function << 20;
	command |= (sri_info->board & 0x3) << 16;
	command |= (sri_info->card & 0xf) << 12;
	command |= (sri_info->device & 0xf);
	command |= (sri_info->direction << 30);
	command |= 0x80000000;

	/* Clear previous error and done condition bits */
	writel(0x0, __io_address(ARM_VEXPRESS_SYS_CFG_STAT));

	if (sri_info->direction == SRI_CFG_WRITE)
	{
		writel(sri_info->data, __io_address(ARM_VEXPRESS_SYS_CFG_DATA));
		writel(command, __io_address(ARM_VEXPRESS_SYS_CFG_CTRL));

		spin_unlock(&sri_lock);

		/* wait for complete */
		timeout = SRI_TIMEOUT;
		while (!(readl(__io_address(ARM_VEXPRESS_SYS_CFG_STAT)) & 0x1) && (timeout > 0))
		{
			udelay(1);
			timeout--;
		}

		if (timeout == 0)
			status = 2;
		else
		{
		/* check error status */
			if (readl(__io_address(ARM_VEXPRESS_SYS_CFG_STAT)) & 0x2)
				status = 4;
		}
	}
	else
	{
		writel(command, __io_address(ARM_VEXPRESS_SYS_CFG_CTRL));
		spin_unlock(&sri_lock);

		timeout = SRI_TIMEOUT;
		/* wait for complete */
		while (!(readl(__io_address(ARM_VEXPRESS_SYS_CFG_STAT)) & 0x1) && (timeout > 0))
		{
			udelay(1);
			timeout--;
		}

		if (timeout == 0)
			status = 0xA;
		else
		{
		/* check error status */
			if (readl(__io_address(ARM_VEXPRESS_SYS_CFG_STAT)) & 0x2)
				status = 0xC;
			else
			{
			/* read data */
			sri_info->data = readl(__io_address(ARM_VEXPRESS_SYS_CFG_DATA));
			}
		}
	}

	return status;
}

/*
 * Versatile Express Set CLCD Oscillator Frequency
 */
void vexpress_SetOsc(u32 freq)
{
	_sri_info_t sri_info;
	unsigned int status;

	/* initialise transfer info */
	sri_info.function = SRI_CFG_OSC;
	sri_info.direction = SRI_CFG_WRITE;
	sri_info.board = VEXPRESS_SELECTED_CLCD;
	sri_info.card = 0;
	sri_info.device = 1;
	sri_info.data = freq;

	do {
		status = vexpress_sri_transfer(&sri_info);
	} while (status == 1);

	if (status != 0)
		printk(KERN_ERR "CLCD: unable to set Oscillator, status %d\n", status);
}

static void vexpress_reboot(char str, const char *cmd)
{
	_sri_info_t sri_info;
	unsigned int status;

	/* initialise transfer info */
	sri_info.function = SRI_CFG_REBOOT;

	do {
		status = vexpress_sri_transfer(&sri_info);
	} while (status == 1);
}

static void vexpress_power_off(void)
{
	_sri_info_t sri_info;
	unsigned int status;

	/* initialise transfer info */
	sri_info.function = SRI_CFG_SHUTDOWN;

	do {
		status = vexpress_sri_transfer(&sri_info);
	} while (status == 1);
}

static int __init vexpress_shutdown_fns_init(void)
{
	pm_power_off = vexpress_power_off;
	arm_pm_restart = vexpress_reboot;

	return 0;
}
arch_initcall(vexpress_shutdown_fns_init);

/*
 * This is the Realview and Versatile sched_clock implementation.  This
 * has a resolution of 41.7ns, and a maximum value of about 35583 days.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 89 seconds between successive
 * calls to this function.
 */
unsigned long long sched_clock(void)
{
	unsigned long long v = cnt32_to_63(readl(__io_address(VEXPRESS_SYS_24MHz)));

	/* the <<1 gets rid of the cnt_32_to_63 top bit saving on a bic insn */
	v *= 125<<1;
	do_div(v, 3<<1);

	return v;
}


#define VEXPRESS_FLASHCTRL	(__io_address(VEXPRESS_SYS_FLASH))

static int vexpress_flash_init(void)
{
	u32 val;

	val = __raw_readl(VEXPRESS_FLASHCTRL);
	val &= ~VEXPRESS_FLASHPROG_FLVPPEN;
	__raw_writel(val, VEXPRESS_FLASHCTRL);

	return 0;
}

static void vexpress_flash_exit(void)
{
	u32 val;

	val = __raw_readl(VEXPRESS_FLASHCTRL);
	val &= ~VEXPRESS_FLASHPROG_FLVPPEN;
	__raw_writel(val, VEXPRESS_FLASHCTRL);
}

static void vexpress_flash_set_vpp(int on)
{
	u32 val;

	val = __raw_readl(VEXPRESS_FLASHCTRL);
	if (on)
		val |= VEXPRESS_FLASHPROG_FLVPPEN;
	else
		val &= ~VEXPRESS_FLASHPROG_FLVPPEN;
	__raw_writel(val, VEXPRESS_FLASHCTRL);
}

static struct flash_platform_data vexpress_flash_data = {
	.map_name		= "cfi_probe",
	.width			= 4,
	.init			= vexpress_flash_init,
	.exit			= vexpress_flash_exit,
	.set_vpp		= vexpress_flash_set_vpp,
};

struct platform_device vexpress_flash_device = {
	.name			= "armflash",
	.id			= 0,
	.dev			= {
		.platform_data	= &vexpress_flash_data,
	},
};

int vexpress_flash_register(struct resource *res, u32 num)
{
	vexpress_flash_device.resource = res;
	vexpress_flash_device.num_resources = num;
	return platform_device_register(&vexpress_flash_device);
}

static struct smsc911x_platform_config smsc911x_config = {
       .flags          = SMSC911X_USE_32BIT,
       .irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
       .irq_type       = SMSC911X_IRQ_TYPE_PUSH_PULL,
       .phy_interface  = PHY_INTERFACE_MODE_MII,
};

static struct platform_device vexpress_eth_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= 2,
};

int vexpress_eth_register(const char *name, struct resource *res)
{
	if (name)
		vexpress_eth_device.name = name;
	vexpress_eth_device.resource = res;
	if (strcmp(vexpress_eth_device.name, "smsc911x") == 0)
		vexpress_eth_device.dev.platform_data = &smsc911x_config;

	return platform_device_register(&vexpress_eth_device);
}

static struct isp1760_platform_data isp1760_priv = {
	.is_isp1761 = 1,
	.bus_width_16 = 0,
	.port1_otg = 1,
	.analog_oc = 0,
	.dack_polarity_high = 1,
	.dreq_polarity_high = 1,
};

struct platform_device vexpress_usb_device = {
	.name		= "isp1760",
	.id		= 0,
	.dev = {
		.platform_data = &isp1760_priv,
	},
	.num_resources	= 2,
};

int vexpress_usb_register(struct resource *res)
{
	vexpress_usb_device.resource = res;
	return platform_device_register(&vexpress_usb_device);
}

static struct pata_platform_info pata_platform_data = {
	.ioport_shift		= 2,
};

static struct resource pata_resources[] = {
	[0] = {
		.start		= VEXPRESS_CF_BASE,
		.end		= VEXPRESS_CF_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= VEXPRESS_CF_BASE + 0x100,
		.end		= VEXPRESS_CF_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

struct platform_device vexpress_cf_device = {
	.name			= "pata_platform",
	.id			= -1,
	.num_resources		= ARRAY_SIZE(pata_resources),
	.resource		= pata_resources,
	.dev			= {
	.platform_data		= &pata_platform_data,
	},
};

static unsigned int vexpress_mmc_status(struct device *dev)
{
	struct amba_device *adev = container_of(dev, struct amba_device, dev);
	u32 mask;

	if (adev->res.start == VEXPRESS_MMCI0_BASE)
		mask = 1;
	else
		mask = 2;

	return !(readl(__io_address(VEXPRESS_SYS_MCI)) & mask);
}

struct mmci_platform_data vexpress_mmc0_plat_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.status		= vexpress_mmc_status,
};

/*
 * Clock handling
 */
static const struct icst307_params vexpress_oscvco_params = {
	.ref		= 24000,
	.vco_max	= 200000,
	.vd_min		= 4 + 8,
	.vd_max		= 511 + 8,
	.rd_min		= 1 + 2,
	.rd_max		= 127 + 2,
};

static void vexpress_oscvco_set(struct clk *clk, struct icst307_vco vco)
{
	/* Convert OD values to actual values */
	const int ODVAL[] = { 10, 2, 8, 4, 5, 7, 3, 6 };
	u32 val;

	val = ((24 * 2 * 1024 * (vco.v + 8)) / ((vco.r + 2) * ODVAL[vco.s])) * 1024;

	vexpress_SetOsc(val);
}

static struct clk oscvco_clk = {
	.params	= &vexpress_oscvco_params,
	.setvco = vexpress_oscvco_set,
};

/*
 * These are fixed clocks.
 */
static struct clk ref24_clk = {
        .rate   = 24000000,
};

static struct clk_lookup lookups[] = {
        {       /* UART0 */
                .dev_id         = "fpga:uart0",
                .clk            = &ref24_clk,
        }, {    /* UART1 */
                .dev_id         = "fpga:uart1",
                .clk            = &ref24_clk,
        }, {    /* UART2 */
                .dev_id         = "fpga:uart2",
                .clk            = &ref24_clk,
        }, {    /* UART3 */
                .dev_id         = "fpga:uart3",
                .clk            = &ref24_clk,
        }, {    /* KMI0 */
                .dev_id         = "fpga:kmi0",
                .clk            = &ref24_clk,
        }, {    /* KMI1 */
                .dev_id         = "fpga:kmi1",
                .clk            = &ref24_clk,
        }, {    /* MMC0 */
                .dev_id         = "fpga:mmc0",
                .clk            = &ref24_clk,
        }, {    /* CLCD */
                .dev_id         = "dev:clcd",
                .clk            = &oscvco_clk,
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

/*
 * CLCD support.
 */

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

static struct clcd_panel svga = {
	.mode		= {
		.name		= "SVGA",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 600,
		.pixclock	= 27440,
		.left_margin	= 20,
		.right_margin	= 20,
		.upper_margin	= 5,
		.lower_margin	= 5,
		.hsync_len	= 164,
		.vsync_len	= 62,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= TIM2_BCD | TIM2_IPC,
	.cntl		= CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
	.bpp		= 16,
};


static struct clcd_panel xvga = {
	.mode		= {
		.name		= "XVGA",
		.refresh	= 60,
		.xres		= 1024,
		.yres		= 768,
		.pixclock	= 15748,
		.left_margin	= 152,
		.right_margin	= 48,
		.upper_margin	= 23,
		.lower_margin	= 3,
		.hsync_len	= 104,
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

struct clcd_panel *panel_type = &xvga;

static void __init early_clcd(char **p)
{
        if (memcmp(*p, "svga", 4) == 0) {
                panel_type = &svga;
        } else if (memcmp(*p, "xvga", 4) == 0) {
                panel_type = &xvga;
        } else if (memcmp(*p, "vga", 3) == 0) {
                panel_type = &vga;
        }
}
__early_param("clcd=", early_clcd);

static struct clcd_panel *vexpress_clcd_panel(void)
{
	return panel_type;
}

/*
 * Disable all display connectors on the interface module.
 */
static void vexpress_clcd_disable(struct clcd_fb *fb)
{
}

/*
 * Enable the relevant CLCD.
 */
static void vexpress_clcd_enable(struct clcd_fb *fb)
{
	_sri_info_t sri_info;
	unsigned int status;

	/* initialise transfer info */
	sri_info.function = SRI_CFG_MUXFPGA;
	sri_info.direction = SRI_CFG_WRITE;
	sri_info.board = VEXPRESS_SELECTED_CLCD;
	sri_info.card = 0;
	sri_info.device = 0;
	sri_info.data = 0;

	do {
		status = vexpress_sri_transfer(&sri_info);
	} while (status == 1);

	if (status != 0)
		printk(KERN_ERR "CLCD: unable to select CLCD, status %d\n", status);
}

static int vexpress_clcd_setup(struct clcd_fb *fb)
{
	unsigned long framesize;
	dma_addr_t dma;

	/* XVGA, 16bpp */
	framesize = 1024 * 768 * 2;

	fb->panel = vexpress_clcd_panel();

#if VEXPRESS_SELECTED_CLCD == 0
	dma = 0x4C000000;
	printk(KERN_INFO "CLCD: framebuffer fixed at: 0x%08x size: 0x%08lx\n",
		dma, framesize);
	fb->fb.screen_base = ioremap(dma, framesize);
#else
	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, framesize,
						    &dma, GFP_KERNEL | GFP_DMA);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		return -ENOMEM;
	}
#endif
	fb->fb.fix.smem_start = dma;
	fb->fb.fix.smem_len = framesize;
	return 0;
}

static int vexpress_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
#if VEXPRESS_SELECTED_CLCD == 0
	unsigned long off, start;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;

	start = fb->fb.fix.smem_start;
	len = PAGE_ALIGN(start & ~PAGE_MASK) + fb->fb.fix.smem_len;
	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	/*
	 * Don't alter the page protection flags; we want to keep the area
	 * cached for better performance.  This does mean that we may miss
	 * some updates to the screen occasionally, but process switches
	 * should cause the caches and buffers to be flushed often enough.
	 */
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot))
		return -EAGAIN;
	return 0;
#else
	return dma_mmap_writecombine(&fb->dev->dev, vma,
				     fb->fb.screen_base,
				     fb->fb.fix.smem_start,
				     fb->fb.fix.smem_len);
#endif
}

static void vexpress_clcd_remove(struct clcd_fb *fb)
{
#if VEXPRESS_SELECTED_CLCD == 0
	iounmap(fb->fb.screen_base);
#else
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
			      fb->fb.screen_base, fb->fb.fix.smem_start);
#endif
}

struct clcd_board clcd_plat_data = {
	.name		= "VEXPRESS",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.disable	= vexpress_clcd_disable,
	.enable		= vexpress_clcd_enable,
	.setup		= vexpress_clcd_setup,
	.mmap		= vexpress_clcd_mmap,
	.remove		= vexpress_clcd_remove,
};

#ifdef CONFIG_LEDS

void vexpress_leds_event(led_event_t ledevt)
{
	unsigned long flags;
	u32 val;
	u32 led = 1 << smp_processor_id();

	local_irq_save(flags);
	val = readl(__io_address(VEXPRESS_SYS_LED));

	switch (ledevt) {
	case led_idle_start:
		val = val & ~led;
		break;

	case led_idle_end:
		val = val | led;
		break;

	case led_timer:
		val = val ^ VEXPRESS_SYS_LED7;
		break;

	case led_halted:
		val = 0;
		break;

	default:
		break;
	}

	writel(val, __io_address(VEXPRESS_SYS_LED));
	local_irq_restore(flags);
}
#endif	/* CONFIG_LEDS */

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
	.features	= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= timer_set_mode,
	.set_next_event	= timer_set_next_event,
	.rating		= 300,
	.cpumask	= cpu_all_mask,
};

static void __init vexpress_clockevents_init(unsigned int timer_irq)
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
static irqreturn_t vexpress_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer0_clockevent;

	/* clear the interrupt */
	writel(1, timer0_va_base + TIMER_INTCLR);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction vexpress_timer_irq = {
	.name		= "VEXPRESS Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= vexpress_timer_interrupt,
};

static cycle_t vexpress_get_cycles(struct clocksource *cs)
{
	return ~readl(timer3_va_base + TIMER_VALUE);
}

static struct clocksource clocksource_vexpress = {
	.name	= "timer3",
	.rating	= 200,
	.read	= vexpress_get_cycles,
	.mask	= CLOCKSOURCE_MASK(32),
	.shift	= 20,
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init vexpress_clocksource_init(void)
{
	/* setup timer 0 as free-running clocksource */
	writel(0, timer3_va_base + TIMER_CTRL);
	writel(0xffffffff, timer3_va_base + TIMER_LOAD);
	writel(0xffffffff, timer3_va_base + TIMER_VALUE);
	writel(TIMER_CTRL_32BIT | TIMER_CTRL_ENABLE | TIMER_CTRL_PERIODIC,
		timer3_va_base + TIMER_CTRL);

	clocksource_vexpress.mult =
		clocksource_khz2mult(1000, clocksource_vexpress.shift);
	clocksource_register(&clocksource_vexpress);
}

/*
 * Set up the clock source and clock events devices
 */
void __init vexpress_timer_init(unsigned int timer_irq)
{
	u32 val;

#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
	/*
	 * The dummy clock device has to be registered before the main device
	 * so that the latter will broadcast the clock events
	 */
	local_timer_setup();
#endif

	/*
	 * set clock frequency:
	 *	VEXPRESS_REFCLK is 32KHz
	 *	VEXPRESS_TIMCLK is 1MHz
	 */
	val = readl(__io_address(VEXPRESS_SCTL_BASE));
	writel((VEXPRESS_TIMCLK << VEXPRESS_TIMER1_EnSel) |
	       (VEXPRESS_TIMCLK << VEXPRESS_TIMER2_EnSel) |
	       (VEXPRESS_TIMCLK << VEXPRESS_TIMER3_EnSel) |
	       (VEXPRESS_TIMCLK << VEXPRESS_TIMER4_EnSel) | val,
	       __io_address(VEXPRESS_SCTL_BASE));

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
	setup_irq(timer_irq, &vexpress_timer_irq);

	vexpress_clocksource_init();
	vexpress_clockevents_init(timer_irq);
}

/*
 * Setup the memory banks.
 */
void vexpress_fixup(struct machine_desc *mdesc, struct tag *tags, char **from,
		                        struct meminfo *meminfo)
{
	void __iomem *procid = __io_address(VEXPRESS_SYS_PROCID);
	/*
	 * Most Versatile Express systems have 1024MB of contiguous RAM at 0x60000000.
	 * 64MB of this is mirrored at 0,
	 * Unfortunately it's the memory that starts at 0x80000000.
	 */
#ifdef CONFIG_VEXPRESS_HIGH_PHYS_OFFSET
	meminfo->bank[0].start = 0x60000000;
	meminfo->bank[0].size = SZ_1G;
	meminfo->nr_banks = 1;
#else
	meminfo->bank[0].start = 0;
	meminfo->bank[0].size = SZ_64M;
	meminfo->nr_banks = 1;
#endif
}
