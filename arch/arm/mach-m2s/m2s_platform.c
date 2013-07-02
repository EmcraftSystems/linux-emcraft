/*
 * linux/arch/arm/mach-m2s/m2s_platform.c
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
#include <linux/module.h>

#include <asm/mach-types.h>
#include <asm/hardware/nvic.h>

#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/platform.h>
#include <mach/hardware.h>
#include <mach/iomux.h>
#include <mach/timer.h>
#include <mach/uart.h>
#include <mach/spi.h>
#include <mach/eth.h>
#include <mach/i2c.h>
#include <mach/usb.h>
#include <mach/gpio.h>

/*
 * Define a particular platform (board)
 */
static int m2s_platform = PLATFORM_G4M_VB;

/*
 * Interface to get the platform
 */
int m2s_platform_get(void)
{
	return m2s_platform;
}
EXPORT_SYMBOL(m2s_platform_get);

/*
 * Interface to get the SmartFusion2 device
 */
int m2s_device_get(void)
{
	int r;

	switch (m2s_platform) {
	case PLATFORM_M2S_SOM:
	case PLATFORM_M2S_FG484_SOM:
	case PLATFORM_G4M_VB:
	case PLATFORM_SF2_DEV_KIT:
	default:
		r = DEVICE_M2S_050;
		break;
	}
	return r;
}
EXPORT_SYMBOL(m2s_device_get);

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init m2s_platform_parse(char *s)
{
	if (!strcmp(s, "g4m-vb"))
		m2s_platform = PLATFORM_G4M_VB;
	else if (!strcmp(s, "m2s-som"))
		m2s_platform = PLATFORM_M2S_SOM;
	else if (!strcmp(s, "sf2-dev-kit"))
		m2s_platform = PLATFORM_SF2_DEV_KIT;
	else if (!strcmp(s, "m2s-fg484-som"))
		m2s_platform = PLATFORM_M2S_FG484_SOM;

	return 1;
}
__setup("m2s_platform=", m2s_platform_parse);

/*
 * Forward declarations.
 */
static void __init m2s_map_io(void);
static void __init m2s_init_irq(void);
static void __init m2s_init(void);

/*
 * Data structure for the timer system.
 */
static struct sys_timer m2s_timer = {
	.init		= m2s_timer_init,
};

/*
 * M2S plaform machine description.
 */
MACHINE_START(M2S, "Actel M2S")
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging (CONFIG_DEBUG_LL=y).
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.phys_io	= MSS_UART1_BASE,
	.io_pg_offst	= (IO_ADDRESS(MSS_UART1_BASE) >> 18) & 0xfffc,
	.map_io		= m2s_map_io,
	.init_irq	= m2s_init_irq,
	.timer		= &m2s_timer,
	.init_machine	= m2s_init,
MACHINE_END

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init m2s_map_io(void)
{
}

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init m2s_init_irq(void)
{
	/*
	 * Initialize NVIC. All interrupts are masked initially.
	 */
	nvic_init();
}

/*
 * M2S platform initialization.
 */
static void __init m2s_init(void)
{
	/*
	 * Configure the IOMUXes of SmartFusion2
	 */
	m2s_iomux_init();

#if defined(CONFIG_SERIAL_8250)
	/*
	 * Configure the UART devices
	 */
	m2s_uart_init();
#endif

#if defined(CONFIG_SPI_M2S)
	/*
	 * Configure the SPI master interfaces (and possibly,
	 * SPI slave devices).
	 */
	m2s_spi_init();
#endif

#if defined(CONFIG_M2S_ETH)
	m2s_eth_init();
#endif

#if defined(CONFIG_I2C_A2F)
	/*
	 * Configure the I2C controllers (and possible I2C devices).
	 */
	m2s_i2c_init();
#endif

#if defined(CONFIG_GPIOLIB)
	/*
	 * Register M2S GPIO lines
	 */
	m2s_gpio_init();
#endif

#if defined(CONFIG_M2S_MSS_USB)
	m2s_usb_init();
#endif
}
