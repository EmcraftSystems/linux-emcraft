/*
 * (C) Copyright 2012, 2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusianov <vlad@emcraft.com>
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

#include <linux/init.h>

#include <asm/mach-types.h>

#include <asm/hardware/nvic.h>

#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/clock.h>
#include <mach/iomux.h>
#include <mach/platform.h>
#include <mach/timer.h>
#include <mach/uart.h>
#include <mach/eth.h>
#include <mach/spi.h>
#include <mach/nor-flash.h>
#include <mach/i2c.h>

/*
 * Prototypes
 */
static void __init lpc18xx_map_io(void);
static void __init lpc18xx_init_irq(void);
static void __init lpc18xx_init(void);

/*
 * Define a particular platform (board)
 */
static int lpc18xx_platform = PLATFORM_LPC18XX_HITEX_LPC4350_EVAL;

/*
 * Data structure for the timer system.
 */
static struct sys_timer lpc18xx_timer = {
	.init		= lpc18xx_timer_init,
};

/*
 * Interface to get the platform
 */
EXPORT_SYMBOL(lpc18xx_platform_get);
int lpc18xx_platform_get(void)
{
	return lpc18xx_platform;
}

/*
 * Interface to get the SmartFusion device
 */
EXPORT_SYMBOL(lpc18xx_device_get);
int lpc18xx_device_get(void)
{
	int r;

	switch (lpc18xx_platform) {
	case PLATFORM_LPC18XX_HITEX_LPC4350_EVAL:
		r = DEVICE_LPC4350;
		break;
	case PLATFORM_LPC18XX_HITEX_LPC1850_EVAL:
		r = DEVICE_LPC1850;
		break;
	default:
		r = DEVICE_LPC4350;
		break;
	}
	return r;
}

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init lpc18xx_platform_parse(char *s)
{
	if (!strcmp(s, "hitex-lpc4350"))
		lpc18xx_platform = PLATFORM_LPC18XX_HITEX_LPC4350_EVAL;
	else if (!strcmp(s, "hitex-lpc1850"))
		lpc18xx_platform = PLATFORM_LPC18XX_HITEX_LPC1850_EVAL;

	return 1;
}
__setup("lpc18xx_platform=", lpc18xx_platform_parse);

/*
 * LPC18xx plaform machine description.
 */
MACHINE_START(LPC18XX, "NXP LPC18xx")
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging (CONFIG_DEBUG_LL=y).
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.phys_io	= 0 /* TBD */,
	.io_pg_offst	= 0 /* TBD */,
	.map_io		= lpc18xx_map_io,
	.init_irq	= lpc18xx_init_irq,
	.timer		= &lpc18xx_timer,
	.init_machine	= lpc18xx_init,
MACHINE_END

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init lpc18xx_map_io(void)
{

}

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init lpc18xx_init_irq(void)
{
	/*
	 * Initialize NVIC. All interrupts are masked initially.
	 */
	nvic_init();
}

/*
 * LPC18xx platform initialization.
 */
static void __init lpc18xx_init(void)
{
	/*
	 * Configure the IOMUXes of LPC18xx
	 */
	lpc18xx_iomux_init();

#if defined(CONFIG_SERIAL_8250)
	/*
	 * Configure the UART devices
	 */
	lpc18xx_uart_init();
#endif

#if defined(CONFIG_STM32_ETHER)
	/*
	 * Configure the LPC18xx MAC
	 */
	lpc18xx_eth_init();
#endif

#if defined(CONFIG_SPI_PL022)
	/*
	 * Configure the SSP/SPI
	 */
	lpc18xx_spi_init();
#endif

#if defined(CONFIG_MTD_PHYSMAP)
	/*
	 * Configure external NOR flash
	 */
	lpc18xx_nor_flash_init();
#endif

#if defined(CONFIG_I2C_LPC2K)
	/*
	 * Configure the I2C bus
	 */
	lpc18xx_i2c_init();
#endif
}
