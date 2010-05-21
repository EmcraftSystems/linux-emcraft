/*
 *  arch/arm/mach-vexpress/vexpress_pca9.c
 *
 *  Copyright (C) 2009 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/amba/bus.h>
#include <linux/amba/pl061.h>
#include <linux/amba/mmci.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/pmu.h>
#include <asm/smp_twd.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/icst307.h>
#include <asm/hardware/cache-l2x0.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/board-pca9.h>
#include <mach/irqs.h>

#include "core.h"
#include "clock.h"

static struct map_desc arm_vexpress_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(VEXPRESS_SYS_BASE),
		.pfn		= __phys_to_pfn(VEXPRESS_SYS_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(VEXPRESS_SCTL_BASE),
		.pfn		= __phys_to_pfn(VEXPRESS_SCTL_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(VEXPRESS_TIMER0_1_BASE),
		.pfn		= __phys_to_pfn(VEXPRESS_TIMER0_1_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= IO_ADDRESS(VEXPRESS_TIMER2_3_BASE),
		.pfn		= __phys_to_pfn(VEXPRESS_TIMER2_3_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		/*
		 * The A5 GIC is overlayed with the A9 SCU
		 * and is overlapped with the A9 GIC.
		 * So as a bodge we map it instead of the A9 one
		 * since 4K is enough to cover both.
		 */
		.virtual        = IO_ADDRESS(VEXPRESS_CA5_GIC_CPU_BASE),
		.pfn            = __phys_to_pfn(VEXPRESS_CA5_GIC_CPU_BASE),
		.length         = SZ_4K,
		.type           = MT_DEVICE,
	}, {
		.virtual        = IO_ADDRESS(VEXPRESS_GIC_DIST_BASE),
		.pfn            = __phys_to_pfn(VEXPRESS_GIC_DIST_BASE),
		.length         = SZ_4K,
		.type           = MT_DEVICE,
	},
#ifdef CONFIG_CACHE_L2X0
	{
		.virtual        = IO_ADDRESS(VEXPRESS_L220_BASE),
		.pfn            = __phys_to_pfn(VEXPRESS_L220_BASE),
		.length         = SZ_8K,
		.type           = MT_DEVICE,
	},
#endif
#ifdef CONFIG_DEBUG_LL
	{
		.virtual	= IO_ADDRESS(VEXPRESS_UART0_BASE),
		.pfn		= __phys_to_pfn(VEXPRESS_UART0_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	},
#endif
};

static void __init arm_vexpress_map_io(void)
{
	iotable_init(arm_vexpress_io_desc, ARRAY_SIZE(arm_vexpress_io_desc));
}

/*
 * Versatile Express Core AMBA devices
 */

#define AACI_IRQ		{ IRQ_VEXPRESS_AACI, NO_IRQ }
#define AACI_DMA		{ 0x80, 0x81 }
#define MMCI0_IRQ		{ IRQ_VEXPRESS_MMCI0A, IRQ_VEXPRESS_MMCI0B }
#define MMCI0_DMA		{ 0x84, 0 }
#define KMI0_IRQ		{ IRQ_VEXPRESS_KMI0, NO_IRQ }
#define KMI0_DMA		{ 0, 0 }
#define KMI1_IRQ		{ IRQ_VEXPRESS_KMI1, NO_IRQ }
#define KMI1_DMA		{ 0, 0 }
#define SMC_IRQ			{ NO_IRQ, NO_IRQ }
#define SMC_DMA			{ 0, 0 }
#define MPMC_IRQ		{ NO_IRQ, NO_IRQ }
#define MPMC_DMA		{ 0, 0 }
#if VEXPRESS_SELECTED_CLCD == 1
#define CLCD_IRQ		{ IRQ_VE_CA9_CLCD, NO_IRQ }
#else
#define CLCD_IRQ		{ IRQ_VEXPRESS_CLCD, NO_IRQ }
#endif
#define CLCD_DMA		{ 0, 0 }
#define SCTL_IRQ		{ NO_IRQ, NO_IRQ }
#define SCTL_DMA		{ 0, 0 }
#define WATCHDOG_IRQ		{ IRQ_VEXPRESS_WATCHDOG, NO_IRQ }
#define WATCHDOG_DMA		{ 0, 0 }
#define RTC_IRQ			{ IRQ_VEXPRESS_RTC, NO_IRQ }
#define RTC_DMA			{ 0, 0 }
#define UART0_IRQ		{ IRQ_VEXPRESS_UART0, NO_IRQ }
#define UART0_DMA		{ 15, 14 }
#define UART1_IRQ		{ IRQ_VEXPRESS_UART1, NO_IRQ }
#define UART1_DMA		{ 13, 12 }
#define UART2_IRQ		{ IRQ_VEXPRESS_UART2, NO_IRQ }
#define UART2_DMA		{ 11, 10 }
#define UART3_IRQ		{ IRQ_VEXPRESS_UART3, NO_IRQ }
#define UART3_DMA		{ 0x86, 0x87 }

/* FPGA Primecells */
AMBA_DEVICE(aaci,	"fpga:aaci",	AACI,		NULL);
AMBA_DEVICE(mmc0,	"fpga:mmc0",	MMCI0,		&vexpress_mmc0_plat_data);
AMBA_DEVICE(kmi0,	"fpga:kmi0",	KMI0,		NULL);
AMBA_DEVICE(kmi1,	"fpga:kmi1",	KMI1,		NULL);
AMBA_DEVICE(uart0,	"fpga:uart0",	UART0,		NULL);
AMBA_DEVICE(uart1,	"fpga:uart1",	UART1,		NULL);
AMBA_DEVICE(uart2,	"fpga:uart2",	UART2,		NULL);
AMBA_DEVICE(uart3,	"fpga:uart3",	UART3,		NULL);

/* DevChip Primecells */
AMBA_DEVICE(smc,	"dev:smc",	SMC,		NULL);
AMBA_DEVICE(sctl,	"dev:sctl",	SCTL,		NULL);
AMBA_DEVICE(wdog,	"dev:wdog",	WATCHDOG, 	NULL);
AMBA_DEVICE(rtc,	"dev:rtc",	RTC,		NULL);
AMBA_DEVICE(clcd,	"dev:clcd",	CLCD,		&clcd_plat_data);

static struct amba_device *amba_devs[] __initdata = {
	&uart0_device,
	&uart1_device,
	&uart2_device,
	&uart3_device,
	&smc_device,
	&clcd_device,
	&sctl_device,
	&wdog_device,
	&rtc_device,
	&aaci_device,
	&mmc0_device,
	&kmi0_device,
	&kmi1_device,
};

/*
 * ARM Versatile Express platform devices
 */
static struct resource vexpress_flash_resources[] = {
	[0] = {
		.start          = VEXPRESS_FLASH0_BASE,
		.end            = VEXPRESS_FLASH0_BASE + VEXPRESS_FLASH0_SIZE - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = VEXPRESS_FLASH1_BASE,
		.end            = VEXPRESS_FLASH1_BASE + VEXPRESS_FLASH1_SIZE - 1,
		.flags          = IORESOURCE_MEM,
	},
};

static struct resource vexpress_smsc911x_resources[] = {
	[0] = {
		.start		= VEXPRESS_ETH_BASE,
		.end		= VEXPRESS_ETH_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_VEXPRESS_ETH,
		.end		= IRQ_VEXPRESS_ETH,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct resource vexpress_isp1761_resources[] = {
	[0] = {
		.start		= VEXPRESS_USB_BASE,
		.end		= VEXPRESS_USB_BASE + SZ_128K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_VEXPRESS_USB,
		.end		= IRQ_VEXPRESS_USB,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct resource pmu_resources[] = {
	[0] = {
		.start		= IRQ_VEXPRESS_PMU_CPU0,
		.end		= IRQ_VEXPRESS_PMU_CPU0,
		.flags		= IORESOURCE_IRQ,
	},
	[1] = {
		.start		= IRQ_VEXPRESS_PMU_CPU1,
		.end		= IRQ_VEXPRESS_PMU_CPU1,
		.flags		= IORESOURCE_IRQ,
	},
	[2] = {
		.start		= IRQ_VEXPRESS_PMU_CPU2,
		.end		= IRQ_VEXPRESS_PMU_CPU2,
		.flags		= IORESOURCE_IRQ,
	},
	[3] = {
		.start		= IRQ_VEXPRESS_PMU_CPU3,
		.end		= IRQ_VEXPRESS_PMU_CPU3,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device pmu_device = {
	.name			= "arm-pmu",
	.id			= ARM_PMU_DEVICE_CPU,
	.num_resources		= ARRAY_SIZE(pmu_resources),
	.resource		= pmu_resources,
};

static void __init gic_init_irq(void)
{
	/*
	 * The A5 and the A9 currently have the GIC at a different address
	 */
	void __iomem *procid = __io_address(VEXPRESS_SYS_PROCID);

	if ((readl(procid) & 0xff000000) == 0x0c000000) {
		gic_cpu_base_addr = __io_address(VEXPRESS_CA9_GIC_CPU_BASE);
		gic_dist_init(0, __io_address(VEXPRESS_GIC_DIST_BASE), 29);
		gic_cpu_init(0, __io_address(VEXPRESS_CA9_GIC_CPU_BASE));
	} else {
		gic_cpu_base_addr = __io_address(VEXPRESS_CA5_GIC_CPU_BASE);
		gic_dist_init(0, __io_address(VEXPRESS_GIC_DIST_BASE), 29);
		gic_cpu_init(0, __io_address(VEXPRESS_CA5_GIC_CPU_BASE));
	}
}

static void __init arm_vexpress_timer_init(void)
{
	timer0_va_base = __io_address(VEXPRESS_TIMER0_1_BASE);
	timer1_va_base = __io_address(VEXPRESS_TIMER0_1_BASE) + 0x20;
	timer2_va_base = __io_address(VEXPRESS_TIMER2_3_BASE);
	timer3_va_base = __io_address(VEXPRESS_TIMER2_3_BASE) + 0x20;

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = __io_address(VEXPRESS_TWD_BASE);
#endif
	vexpress_timer_init(IRQ_VEXPRESS_TIMER0_1);
}

static struct sys_timer arm_vexpress_timer = {
	.init		= arm_vexpress_timer_init,
};

static void __init arm_vexpress_init(void)
{
	int i;

#ifdef CONFIG_CACHE_L2X0
	void __iomem *l2x0_base =
		__io_address(VEXPRESS_L220_BASE);

	if (!(readl(l2x0_base + L2X0_CTRL) & 1)) {
		/* set RAM latencies */
		writel(0x000, l2x0_base + L2X0_TAG_LATENCY_CTRL);
		writel(0x000, l2x0_base + L2X0_DATA_LATENCY_CTRL);
	}

	/*
	 * I Prefetch, 64KB way size, 8-way Assoc
	 * Bits:  .. 1 0 | 0 0 . 0|0 0 0 0 | 011 0 | ... 0 | .... | .... | ....
	 * Bits shown as . are reserved
	 */

	l2x0_init(l2x0_base, 0x20060000, 0xc200efff);
#endif

	vexpress_flash_register(vexpress_flash_resources,
				ARRAY_SIZE(vexpress_flash_resources));
	vexpress_eth_register(NULL, vexpress_smsc911x_resources);
	platform_device_register(&vexpress_cf_device);
	vexpress_usb_register(vexpress_isp1761_resources);
	platform_device_register(&pmu_device);

	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}

#ifdef CONFIG_LEDS
	writel(0, __io_address(VEXPRESS_SYS_LED));
	leds_event = vexpress_leds_event;
#endif
}

static void __init vexpress_init_irq (void)
{
    gic_init_irq();
}


MACHINE_START(VEXPRESS, "ARM-VE-CA9")
	/* Maintainer: ARM Ltd */
	.phys_io	= VEXPRESS_UART0_BASE,
	.io_pg_offst	= (IO_ADDRESS(VEXPRESS_UART0_BASE) >> 18) & 0xfffc,
	.boot_params	= PHYS_OFFSET + 0x00000100,
	.fixup		= vexpress_fixup,
	.map_io		= arm_vexpress_map_io,
	.init_irq	= vexpress_init_irq,
	.timer		= &arm_vexpress_timer,
	.init_machine	= arm_vexpress_init,
MACHINE_END
