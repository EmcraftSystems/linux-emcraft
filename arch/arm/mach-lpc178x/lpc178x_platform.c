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
#include <linux/i2c.h>
#include <sound/uda1380.h>

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
#include <mach/flash.h>
#include <mach/dma.h>
#include <mach/sdcard.h>
#include <mach/i2c.h>
#include <mach/fb.h>
#include <mach/rtc.h>
#include <mach/wdt.h>

#if defined(CONFIG_GPIOLIB)
#include <mach/i2c-gpio.h>
#include <mach/gpio.h>
#include <mach/ea-lpc1788-pca9532.h>
#endif /* CONFIG_GPIOLIB */

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
	case PLATFORM_LPC178X_LNX_EVB:
		r = DEVICE_EMCRAFT_LPC_LNX_EVB;
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
	if (!strcmp(s, "ea-lpc1788"))
		lpc178x_platform = PLATFORM_LPC178X_EA_LPC1788;
	else if (!strcmp(s, "lpc-lnx-evb"))
		lpc178x_platform = PLATFORM_LPC178X_LNX_EVB;

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

#if defined(CONFIG_SND_LPC3XXX_SOC) || defined(CONFIG_SND_LPC3XXX_SOC_MODULE)
/*
 * Platform data for the UDA1380 audio codec.
 *
 * There are no GPIOs connected to codec power and reset pins on EA-LPC1788.
 * Use PLL integrated in the codec for clocking.
 */
static struct uda1380_platform_data uda1380_info = {
	.gpio_power	= -1,
	.gpio_reset	= -1,
	.dac_clk	= UDA1380_DAC_CLK_WSPLL,
};

/*
 * UDA1380 registration info for the EA-LPC1788 board
 */
static struct i2c_board_info __initdata ealpc1788_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("uda1380", 0x1A),
		.platform_data = &uda1380_info,
	},
};
#endif

/*
 * LPC178x/7x platform initialization.
 */
static void __init lpc178x_init(void)
{
	/*
	 * Configure the IOMUXes of LPC178x/7x
	 */
	lpc178x_iomux_init();

	/*
	 * Initialize the General Purpose DMA controller
	 */
	lpc178x_dma_init();

#if defined(CONFIG_SERIAL_8250)
	/*
	 * Configure the UART devices
	 */
	lpc178x_uart_init();
#endif

#if defined(CONFIG_LPC178X_ETHER)
	/*
	 * Configure the LPC178x/7x MAC
	 */
	lpc178x_eth_init();
#endif

#if defined(CONFIG_SPI_PL022)
	/*
	 * Configure the SPI devices
	 */
	lpc178x_spi_init();
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

#if defined(CONFIG_I2C_LPC2K) || defined(CONFIG_I2C_LPC2K_MODULE)
	/*
	 * Configure the available I2C interfaces
	 */
	lpc178x_i2c_init();
#endif /* defined(CONFIG_I2C_LPC2K) || defined(CONFIG_I2C_LPC2K_MODULE) */

#if defined(CONFIG_GPIOLIB)
	/*
	 * Register LPC178x GPIO lines
	 */
	lpc178x_gpio_init();
#endif /* CONFIG_GPIOLIB */

#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	/*
	 * Configure some of the I2C interfaces to be controlled by
	 * the `i2c-gpio` GPIO-emulated I2C driver.
	 */
	lpc178x_i2c_gpio_init();
#endif /* defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE) */

#if defined(CONFIG_FB_ARMCLCD)
	/*
	 * Configure the LPC178x/7x LCD Controller
	 */
	lpc178x_fb_init();
#endif /* defined(CONFIG_FB_ARMCLCD) */

#if defined(CONFIG_LEDS_PCA9532) && defined(CONFIG_LEDS_PCA9532_GPIO)
	if (lpc178x_platform_get() == PLATFORM_LPC178X_EA_LPC1788)
		ea_lpc1788_pca9532_init();
#endif /* CONFIG_LEDS_PCA9532 && CONFIG_LEDS_PCA9532_GPIO */

#if defined(CONFIG_RTC_DRV_LPC178X)
	/*
	 * Initialize the on-chip real-time clock
	 */
	lpc178x_rtc_init();
#endif /* CONFIG_RTC_DRV_LPC178x */

#if defined(CONFIG_SND_LPC3XXX_SOC) || defined(CONFIG_SND_LPC3XXX_SOC_MODULE)
	if (lpc178x_platform_get() == PLATFORM_LPC178X_EA_LPC1788) {
		i2c_register_board_info(0, ealpc1788_i2c_board_info,
			ARRAY_SIZE(ealpc1788_i2c_board_info));
	}
#endif /* CONFIG_SND_LPC3XXX_SOC || CONFIG_SND_LPC3XXX_SOC_MODULE */
#if defined(CONFIG_LPC2K_WATCHDOG)
	/*
	 * Initialize the on-chip wdt
	 */
	lpc178x_wdt_init();
#endif /* CONFIG_LPC178X_WATCHDOG */

}
