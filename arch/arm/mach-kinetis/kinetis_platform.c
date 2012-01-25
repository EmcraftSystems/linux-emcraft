/*
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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
#include <mach/nand.h>

/*
 * Prototypes
 */
static void __init kinetis_map_io(void);
static void __init kinetis_init_irq(void);
static void __init kinetis_init(void);

/*
 * Define a particular platform (board)
 */
static int kinetis_platform = PLATFORM_KINETIS_TWR_K70F120M;

/*
 * Data structure for the timer system.
 */
static struct sys_timer kinetis_timer = {
	.init		= kinetis_timer_init,
};

/*
 * Interface to get the platform
 */
EXPORT_SYMBOL(kinetis_platform_get);
int kinetis_platform_get(void)
{
	return kinetis_platform;
}

/*
 * Interface to get the Freescale Kinetis device
 */
EXPORT_SYMBOL(kinetis_device_get);
int kinetis_device_get(void)
{
	int r;

	switch (kinetis_platform) {
	case PLATFORM_KINETIS_TWR_K70F120M:
	default:
		r = DEVICE_TWR_K70F120M;
		break;
	}
	return r;
}

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init kinetis_platform_parse(char *s)
{
	if (!strcmp(s, "twr-k70f120m")) {
		kinetis_platform = PLATFORM_KINETIS_TWR_K70F120M;
	}

	return 1;
}
__setup("kinetis_platform=", kinetis_platform_parse);

/*
 * Freescale Kinetis platform machine description
 */
MACHINE_START(KINETIS, "Freescale Kinetis")
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging (CONFIG_DEBUG_LL=y).
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.phys_io	= 0 /* TBD */,
	.io_pg_offst	= 0 /* TBD */,
	.map_io		= kinetis_map_io,
	.init_irq	= kinetis_init_irq,
	.timer		= &kinetis_timer,
	.init_machine	= kinetis_init,
MACHINE_END

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init kinetis_map_io(void)
{

}

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init kinetis_init_irq(void)
{
	/*
	 * Initialize NVIC. All interrupts are masked initially.
	 */
	nvic_init();
}

/*
 * Freescale Kinetis platform initialization
 */
static void __init kinetis_init(void)
{
	/*
	 * Configure the IOMUXes of the Freescale Kinetis MCU
	 */
	kinetis_iomux_init();

#if defined(CONFIG_SERIAL_KINETIS)
	/*
	 * Configure the UART devices
	 */
	kinetis_uart_init();
#endif

#if defined(CONFIG_KINETIS_MAC)
	/*
	 * Configure the Freescale Kinetis MAC
	 */
	kinetis_eth_init();
#endif

#if defined(CONFIG_MTD_NAND_FSL_NFC)
	/*
	 * Configure the external NAND Flash
	 */
	kinetis_nand_init();
#endif
}
