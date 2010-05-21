/*
 *  linux/arch/arm/mach-mps/board_mps.c
 *
 *  Copyright (C) 2009 ARM Limited
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

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach-types.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/nvic.h>
#include <asm/hardware/icst307.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>

#include <mach/platform.h>
#include <mach/irqs.h>

#include "core.h"
#include "clock.h"

#if PAGE_OFFSET != PHYS_OFFSET
#error "PAGE_OFFSET != PHYS_OFFSET"
#endif

static void __init mps_map_io(void)
{
}

/*
 * MPS AMBA devices
 */

#define GPIO2_IRQ	{ IRQ_MPS_GPIO2, NO_IRQ }
#define GPIO3_IRQ	{ IRQ_MPS_GPIO3, NO_IRQ }

#define AACI_IRQ	{ IRQ_MPS_AACI, NO_IRQ }
#define MMCI_IRQ	{ IRQ_MPS_MMCIA, IRQ_MPS_MMCIB }

#define SMC_IRQ		{ NO_IRQ, NO_IRQ }
#define MPMC_IRQ	{ NO_IRQ, NO_IRQ }
#define CLCD_IRQ	{ IRQ_MPS_CLCD, NO_IRQ }

#define SCTL_IRQ	{ NO_IRQ, NO_IRQ }
#define WATCHDOG_IRQ	{ IRQ_MPS_WDOG, NO_IRQ }
#define GPIO0_IRQ	{ IRQ_MPS_GPIO0, NO_IRQ }
#define GPIO1_IRQ	{ IRQ_MPS_GPIO1, NO_IRQ }
#define RTC_IRQ		{ IRQ_MPS_RTC, NO_IRQ }

#define UART0_IRQ	{ IRQ_MPS_UART0, NO_IRQ }
#define UART1_IRQ	{ IRQ_MPS_UART1, NO_IRQ }
#define UART2_IRQ	{ IRQ_MPS_UART2, NO_IRQ }
#define UART3_IRQ	{ IRQ_MPS_UART3, NO_IRQ }
#define SPI_IRQ		{ IRQ_MPS_SPI, NO_IRQ }

/* FPGA Primecells */
AMBA_DEVICE(aaci,  "fpga:04",		AACI,	NULL);
AMBA_DEVICE(mmc0,  "fpga:mmc0",		MMCI,	&mps_mmc0_plat_data);
AMBA_DEVICE(uart3, "fpga:uart3",	UART3,	NULL);

/* DevChip Primecells */
AMBA_DEVICE(smc,   "dev:00",		SMC,	NULL);
AMBA_DEVICE(clcd,  "dev:clcd",		CLCD,	&clcd_plat_data);
AMBA_DEVICE(sctl,  "dev:e0",		SCTL,	NULL);
AMBA_DEVICE(wdog,  "dev:e1",		WATCHDOG, NULL);
AMBA_DEVICE(rtc,   "dev:e8",		RTC,	NULL);
AMBA_DEVICE(uart0, "dev:uart0",		UART0,	NULL);
AMBA_DEVICE(uart1, "dev:uart1",		UART1,	NULL);
AMBA_DEVICE(uart2, "dev:uart2",		UART2,	NULL);
AMBA_DEVICE(spi,   "dev:f4",		SPI,	NULL);

static struct amba_device *amba_devs[] __initdata = {
	&mmc0_device,
	&uart0_device,
	&uart1_device,
	&uart2_device,
	&uart3_device,
	&smc_device,
	&clcd_device,
	&sctl_device,
	&wdog_device,
	&rtc_device,
	&spi_device,
	&aaci_device,
};

/*
 * MPS platform devices
 */
static struct resource mps_flash_resource = {
	.start			= MPS_FLASH_BASE,
	.end			= MPS_FLASH_BASE + MPS_FLASH_SIZE - 1,
	.flags			= IORESOURCE_MEM,
};

static struct resource mps_eth_resources[] = {
	[0] = {
		.start		= MPS_ETH_BASE,
		.end		= MPS_ETH_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_MPS_ETH,
		.end		= IRQ_MPS_ETH,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device mps_eth_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(mps_eth_resources),
	.resource	= mps_eth_resources,
};

static void __init gic_init_irq(void)
{
	nvic_init();
}

static void __init timer_init(void)
{
	unsigned int timer_irq;

	timer0_va_base = __io_address(MPS_TIMER0_1_BASE);
	timer1_va_base = __io_address(MPS_TIMER0_1_BASE) + 0x20;
	timer2_va_base = __io_address(MPS_TIMER2_3_BASE);
	timer3_va_base = __io_address(MPS_TIMER2_3_BASE) + 0x20;

	timer_irq = IRQ_MPS_TIMER0_1;

	mps_timer_init(timer_irq);
}

static struct sys_timer mps_timer = {
	.init		= timer_init,
};

static void __init mps_init(void)
{
    int i;

    mps_flash_register(&mps_flash_resource, 1);
    platform_device_register(&mps_i2c_device);
    platform_device_register(&mps_eth_device);

    for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
        struct amba_device *d = amba_devs[i];
        amba_device_register(d, &iomem_resource);
    }

#ifdef CONFIG_LEDS
    leds_event = mps_leds_event;
#endif
}

MACHINE_START(MPS, "ARM MPS")
	.phys_io	= MPS_UART0_BASE,
	.io_pg_offst	= (IO_ADDRESS(MPS_UART0_BASE) >> 18) & 0xfffc,
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= mps_map_io,
	.init_irq	= gic_init_irq,
	.timer		= &mps_timer,
	.init_machine	= mps_init,
MACHINE_END
