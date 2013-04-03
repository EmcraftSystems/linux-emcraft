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
#include <mach/dmainit.h>
#include <mach/iomux.h>
#include <mach/platform.h>
#include <mach/timer.h>
#include <mach/uart.h>
#include <mach/eth.h>
#include <mach/nand.h>
#include <mach/fb.h>
#include <mach/esdhc.h>

#if defined(CONFIG_GPIOLIB)
#include <mach/i2c-gpio.h>
#include <mach/mxc_ehci.h>
#include <mach/spi-gpio.h>
#include <mach/gpio.h>
#endif /* CONFIG_GPIOLIB */

/*
 * Prototypes
 */
static void __init kinetis_map_io(void);
static void __init kinetis_init_irq(void);
static void __init kinetis_init(void);

/*
 * Define a particular platform (board) and other default parameters
 */
static int kinetis_platform = PLATFORM_KINETIS_TWR_K70F120M;
static int kinetis_lcdtype = LCD_TWR_LCD_RGB;

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
	case PLATFORM_KINETIS_K61_SOM:
		r = DEVICE_PK61FN1M0VMJ;
		break;
	case PLATFORM_KINETIS_TWR_K70F120M:
	case PLATFORM_KINETIS_K70_SOM:
	default:
		r = DEVICE_PK70FN1M0VMJ;
		break;
	}
	return r;
}

/*
 * Interface to get the type of LCD screen to the Kinetis-based board
 */
EXPORT_SYMBOL(kinetis_lcdtype_get);
int kinetis_lcdtype_get(void)
{
	return kinetis_lcdtype;
}

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init kinetis_platform_parse(char *s)
{
	if (!strcmp(s, "twr-k70f120m")) {
		kinetis_platform = PLATFORM_KINETIS_TWR_K70F120M;
	} else if (!strcmp(s, "k70-som")) {
		kinetis_platform = PLATFORM_KINETIS_K70_SOM;
	} else if (!strcmp(s, "k61-som")) {
		kinetis_platform = PLATFORM_KINETIS_K61_SOM;
	} else {
		pr_err("%s: An unknown platform requested: %s\n", __func__, s);
	}

	return 1;
}
__setup("kinetis_platform=", kinetis_platform_parse);

/*
 * User can define the type of connected LCD from U-Boot
 */
static int __init kinetis_lcdtype_parse(char *s)
{
	if (!strcmp(s, "twr-lcd-rgb")) {
		kinetis_lcdtype = LCD_TWR_LCD_RGB;
	} else if (!strcmp(s, "ea-lcd-004")) {
		kinetis_lcdtype = LCD_EA_LCD_004;
	} else if (!strcmp(s, "fut-twr-nl8048")) {
		kinetis_lcdtype = LCD_FUT_TWR_NL8048;
#ifdef CONFIG_KINETIS_SPI2_GPIO
	} else if (!strcmp(s, "fut-twr-pim-41wvga")) {
		kinetis_lcdtype = LCD_FUT_TWR_PIM_41WVGA;
#endif
	} else {
		pr_err("%s: An unknown LCD type requested: %s\n", __func__, s);
	}

	return 1;
}
__setup("platform_lcd=", kinetis_lcdtype_parse);

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
#if defined(CONFIG_KINETIS_EDMA)
	/*
	 * Configure DMA controller and its driver's API
	 */
	kinetis_dma_init();
#endif

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

#if defined(CONFIG_GPIOLIB)
	/*
	 * Register Kinetis GPIO lines
	 */
	kinetis_gpio_init();
#endif /* CONFIG_GPIOLIB */

#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	/*
	 * Configure some of the I2C interfaces to be controlled by
	 * the `i2c-gpio` GPIO-emulated I2C driver.
	 */
	kinetis_i2c_gpio_init();
#endif /* defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE) */

#if defined(CONFIG_KINETIS_FB)
	/*
	 * Configure the Freescale Kinetis LCD Controller
	 */
	kinetis_fb_init();
#endif

#if defined(CONFIG_USB_EHCI_MXC)
	/*
	 * Configure the Freescale Kinetis USB High Speed controller
	 */
	kinetis_ehci_init();
#endif

#if defined(CONFIG_KINETIS_SPI_GPIO)
	/*
	 * Configure SPI buses emulated using GPIOs
	 */
	kinetis_spi_gpio_init();
#endif

#if defined(CONFIG_MMC_ESDHC)
	/*
	 * Configure SPI buses emulated using GPIOs
	 */
	kinetis_esdhc_init();
#endif
}
