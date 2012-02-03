/*
 * (C) Copyright 2011
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
#include <mach/flash.h>
#include <mach/sdcard.h>

/*
 * Prototypes
 */
static void __init lpc178x_map_io(void);
static void __init lpc178x_init_irq(void);
static void __init lpc178x_init(void);

/*
 * Define a particular platform (board)
 */
static int lpc178x_platform = PLATFORM_LPC178X_EA_LPC1788;

/*
 * Data structure for the timer system.
 */
static struct sys_timer lpc178x_timer = {
	.init		= lpc178x_timer_init,
};

/*
 * Interface to get the platform
 */
EXPORT_SYMBOL(lpc178x_platform_get);
int lpc178x_platform_get(void)
{
	return lpc178x_platform;
}

/*
 * Interface to get the SmartFusion device
 */
EXPORT_SYMBOL(lpc178x_device_get);
int lpc178x_device_get(void)
{
	int r;

	switch (lpc178x_platform) {
	case PLATFORM_LPC178X_EA_LPC1788:
	default:
		r = DEVICE_EA_LPC1788;
		break;
	}
	return r;
}

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init lpc178x_platform_parse(char *s)
{
	if (!strcmp(s, "ea-lpc1788")) {
		lpc178x_platform = PLATFORM_LPC178X_EA_LPC1788;
	}

	return 1;
}
__setup("lpc178x_platform=", lpc178x_platform_parse);

/*
 * LPC178x/7x plaform machine description.
 */
MACHINE_START(LPC178X, "NXP LPC178x/7x")
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging (CONFIG_DEBUG_LL=y).
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.phys_io	= 0 /* TBD */,
	.io_pg_offst	= 0 /* TBD */,
	.map_io		= lpc178x_map_io,
	.init_irq	= lpc178x_init_irq,
	.timer		= &lpc178x_timer,
	.init_machine	= lpc178x_init,
MACHINE_END

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init lpc178x_map_io(void)
{

}

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init lpc178x_init_irq(void)
{
	/*
	 * Initialize NVIC. All interrupts are masked initially.
	 */
	nvic_init();
}

/*
 * LPC178x/7x platform initialization.
 */
static void __init lpc178x_init(void)
{
	/*
	 * Configure the IOMUXes of LPC178x/7x
	 */
	lpc178x_iomux_init();

#if defined(CONFIG_SERIAL_8250)
	/*
	 * Configure the UART devices
	 */
	lpc178x_uart_init();
#endif

#if defined(CONFIG_LPC178X_MAC)
	/*
	 * Configure the LPC178x/7x MAC
	 */
	lpc178x_eth_init();
#endif

#if defined(CONFIG_MTD_PHYSMAP)
	/*
	 * Configure external Flash
	 */
	lpc178x_flash_init();
#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	/*
	 * Configure the OHCI USB Host Controller
	 */
	lpc178x_ohci_init();
#endif /* defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE) */

#if defined(CONFIG_MMC_ARMMMCI)
	/*
	 * Configure the SD Card Interface
	 */
	lpc178x_sdcard_init();
#endif
}
