/*
 * linux/arch/arm/mach-a2f/a2f_platform.c
 *
 * Copyright (C) 2010,2011,2012 Vladimir Khusainov, Emcraft Systems
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
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <mach/hardware.h>

#include <asm/clkdev.h>
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

#include <asm/setup.h>

#include <mach/platform.h>
#include <mach/irqs.h>
#include <mach/iomux.h>
#include <mach/timer.h>
#include <mach/clock.h>
#include <mach/fpga.h>
#include <mach/uart.h>
#include <mach/eth.h>
#include <mach/spi.h>
#include <mach/i2c.h>
#include <mach/flash.h>
#include <mach/gpio.h>

/*
 * Define a particular platform (board)
 */
static int a2f_platform = PLATFORM_A2F_LNX_EVB;

/*
 * Interface to get the platform
 */
EXPORT_SYMBOL(a2f_platform_get);
int a2f_platform_get(void)
{
	return a2f_platform;
}

/*
 * Interface to get the SmartFusion device
 */
EXPORT_SYMBOL(a2f_device_get);
int a2f_device_get(void)
{
	int r;

	switch (a2f_platform) {
	case PLATFORM_A2F_ACTEL_DEV_BRD:
	case PLATFORM_A2F_HOERMANN_BRD:
	case PLATFORM_A2F500_SOM:
		r = DEVICE_A2F_500;
		break;
	case PLATFORM_A2F200_SOM:
	case PLATFORM_A2F_LNX_EVB:
	default:
		r = DEVICE_A2F_200;
		break;
	}
	return r;
}

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init a2f_platform_parse(char *s)
{
	if (!strcmp(s, "a2f-lnx-evb")) {
		a2f_platform = PLATFORM_A2F_LNX_EVB;
	}
	else if (!strcmp(s, "a2f200-som")) {
		a2f_platform = PLATFORM_A2F200_SOM;
	}
	else if (!strcmp(s, "a2f500-som")) {
		a2f_platform = PLATFORM_A2F500_SOM;
	}
	else if (!strcmp(s, "a2f-actel-dev-brd")) {
		a2f_platform = PLATFORM_A2F_ACTEL_DEV_BRD;
	}
	else if (!strcmp(s, "a2f-hoermann-brd")) {
		a2f_platform = PLATFORM_A2F_HOERMANN_BRD;
	}

	return 1;
}
__setup("a2f_platform=", a2f_platform_parse);

/*
 * Forward declarations.
 */
static void __init a2f_map_io(void);
static void __init a2f_init_irq(void);
static void __init a2f_init(void);

/*
 * Data structure for the timer system.
 */
static struct sys_timer a2f_timer = {
	.init           = a2f_timer_init,
};

/*
 * A2F plaform machine description.
 */
MACHINE_START(A2F, "Actel A2F")
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging (CONFIG_DEBUG_LL=y).
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.phys_io	= MSS_UART0_BASE,
	.io_pg_offst	= (IO_ADDRESS(MSS_UART0_BASE) >> 18) & 0xfffc,
	.map_io		= a2f_map_io,
	.init_irq	= a2f_init_irq,
	.timer		= &a2f_timer,
	.init_machine	= a2f_init,
MACHINE_END

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init a2f_map_io(void)
{
}

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init a2f_init_irq(void)
{
	/*
 	 * Initialize NVIC. All interrupts are masked initially.
 	 */
	nvic_init();
}

/*
 * A2F plaform initialization.
 */
static void __init a2f_init(void)
{
	/*
	 * Configure the IOMUXes of SmartFusion
	 */
	a2f_iomux_init();

	/*
	 * Enable DMA-style accesses from FPGA-based IP
	 */
	a2f_fpga_dma_init();

#if defined(CONFIG_A2F_FPGA_DEMUX)
	/*
	 * Configure the FPGA IRQ demultiplexer
	 */
	a2f_fpga_demux_init();
#endif

#if defined(CONFIG_SERIAL_8250)
	/*
	 * Configure the UART devices
	 */
	a2f_uart_init();
#endif

#if defined(CONFIG_CORE10100)
	/*
	 * Configure the Core10100 adapter
	 */
	a2f_eth_init();
#endif

#if defined(CONFIG_SPI_A2F)
	/*
 	 * Configure the SPI master interfaces (and possibly,
 	 * SPI slave devices).
 	 */
	a2f_spi_init();
#endif

#if defined(CONFIG_I2C_A2F)
	/*
 	 * Configure the I2C controllers and devices
 	 */
	a2f_i2c_init();
#endif

#if defined(CONFIG_MTD_PHYSMAP)
	/*
	 * Configure external Flash
	 */
	a2f_flash_init();
#endif

#if defined(CONFIG_GPIOLIB)
	/*
	 * Configure GPIO
	 */
	a2f_gpio_init();
#endif
}
