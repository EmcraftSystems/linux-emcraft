/*
 * (C) Copyright 2011-2013
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
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
#include <mach/eth.h>
#include <mach/hardware.h>
#include <mach/iomux.h>
#include <mach/platform.h>
#include <mach/timer.h>
#include <mach/uart.h>
#include <mach/spi.h>
#include <mach/i2c.h>
#include <mach/flash.h>
#include <mach/sdcard.h>
#include <mach/dmainit.h>
#include <mach/rtc.h>
#include <mach/usb.h>
#include <mach/gpio.h>

/*
 * Prototypes
 */
static void __init stm32_map_io(void);
static void __init stm32_init_irq(void);
static void __init stm32_init(void);

/*
 * Define a particular platform (board)
 */
#ifdef CONFIG_ARCH_STM32F1
/* STM32F1 default platform */
static int stm32_platform = PLATFORM_STM32_SWISSEMBEDDED_COMM;
#else
/* STM32F2 default platform */
static int stm32_platform = PLATFORM_STM32_STM3220G_EVAL;
#endif

/*
 * Data structure for the timer system.
 */
static struct sys_timer stm32_timer = {
	.init		= stm32_timer_init,
};

/*
 * Interface to get the platform
 */
EXPORT_SYMBOL(stm32_platform_get);
int stm32_platform_get(void)
{
	return stm32_platform;
}

/*
 * Interface to get the SmartFusion device
 */
EXPORT_SYMBOL(stm32_device_get);
int stm32_device_get(void)
{
	int r;

	switch (stm32_platform) {
#ifdef CONFIG_ARCH_STM32F1
	/* STM32F1-based platforms */
	case PLATFORM_STM32_SWISSEMBEDDED_COMM:
		r = DEVICE_STM32F103ZE;
		break;
#else
	/* STM32F2-based platforms */
	case PLATFORM_STM32_STM3220G_EVAL:
		r = DEVICE_STM32F207IG;
		break;
	case PLATFORM_STM32_STM3240G_EVAL:
		r = DEVICE_STM32F407IG;
		break;
	case PLATFORM_STM32_STM_SOM:
		r = DEVICE_STM32F437II;
		break;
	case PLATFORM_STM32_STM_STM32F439_SOM:
	case PLATFORM_STM32_STM_DISCO:
		r = DEVICE_STM32F439II;
		break;
#endif
	default:
#ifdef CONFIG_ARCH_STM32F1
		r = DEVICE_STM32F103ZE;
#else
		r = DEVICE_STM32F207IG;
#endif
		break;
	}
	return r;
}

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init stm32_platform_parse(char *s)
{
#ifdef CONFIG_ARCH_STM32F1
	/* STM32F1-based platforms */
	if (!strcmp(s, "stm32f1-se-comm"))
		stm32_platform = PLATFORM_STM32_SWISSEMBEDDED_COMM;
#else
	/* STM32F2-based platforms */
	if (!strcmp(s, "stm3220g-eval"))
		stm32_platform = PLATFORM_STM32_STM3220G_EVAL;
	else if (!strcmp(s, "stm3240g-eval"))
		stm32_platform = PLATFORM_STM32_STM3240G_EVAL;
	else if (!strcmp(s, "stm-som"))
		stm32_platform = PLATFORM_STM32_STM_SOM;
	else if (!strcmp(s, "stm32f4x9-som"))
		stm32_platform = PLATFORM_STM32_STM_STM32F439_SOM;
	else if (!strcmp(s, "stm-disco"))
		stm32_platform = PLATFORM_STM32_STM_DISCO;
#endif

	return 1;
}
__setup("stm32_platform=", stm32_platform_parse);

/*
 * STM32 plaform machine description.
 */
MACHINE_START(STM32, "STMicro STM32")
	/*
	 * Physical address of the serial port used for the early
	 * kernel debugging (CONFIG_DEBUG_LL=y).
	 * This address is actually never used in the MMU-less kernel
	 * (since no mapping is needed to access this port),
	 * but let's keep these fields filled out for consistency.
	 */
	.phys_io	= STM32_USART3_BASE,
	.io_pg_offst	= (IO_ADDRESS(STM32_USART3_BASE) >> 18) & 0xfffc,
	.map_io		= stm32_map_io,
	.init_irq	= stm32_init_irq,
	.timer		= &stm32_timer,
	.init_machine	= stm32_init,
MACHINE_END

/*
 * Map required regions.
 * This being the no-MMU Linux, I am not mapping anything
 * since all I/O registers are available at their physical addresses.
 */
static void __init stm32_map_io(void)
{

}

/*
 * Initialize the interrupt processing subsystem.
 */
static void __init stm32_init_irq(void)
{
	/*
	 * Initialize NVIC. All interrupts are masked initially.
	 */
	nvic_init();
}

/*
 * STM32 plaform initialization.
 */
static void __init stm32_init(void)
{
	/*
	 * Configure the IOMUXes of STM32
	 */
	stm32_iomux_init();

#if defined(CONFIG_STM32_DMA)
	/*
	 * Configure DMA controller and its driver's API
	 */
	stm32_dma_init();
#endif

#if defined(CONFIG_SERIAL_STM32)
	/*
	 * Configure the USART devices
	 */
	stm32_uart_init();
#endif

#if defined(CONFIG_STM32_MAC)
	/*
	 * Configure the STM32 MAC
	 */
	stm32_eth_init();
#endif

#if defined(CONFIG_SPI_STM32)
	/*
	 * Configure the STM32 SPI devices
	 */
	stm32_spi_init();
#endif

#if defined(CONFIG_I2C_STM32)
	/*
	 * Configure the STM32 I2C devices
	 */
	stm32_i2c_init();
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_STM32F4_MAP)
	/*
	 * Configure external Flash
	 */
	stm32_flash_init();
#endif

#if defined(CONFIG_MMC_ARMMMCI)
	/*
	 * Configure SD card controller
	 */
	stm32_sdcard_init();
#endif

#if defined(CONFIG_RTC_DRV_STM32F2)
	/*
	 * Initialize the on-chip real-time clock
	 */
	stm32_rtc_init();
#endif

#if defined(CONFIG_STM32_USB_OTG_FS)
       /*
        * Initialize the USB OTG FS controller
        */
       stm32_usb_otg_fs_init();
#endif

#if defined(CONFIG_GPIOLIB)
	/*
	 * Register the MCU GPIO chip
	 */
	stm32_gpio_init();
#endif
}
