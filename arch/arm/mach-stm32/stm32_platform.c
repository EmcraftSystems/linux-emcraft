/*
 * (C) Copyright 2011
 * Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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
#include <mach/hardware.h>
#include <mach/iomux.h>
#include <mach/platform.h>
#include <mach/timer.h>
#include <mach/uart.h>

/*
 * Prototypes
 */
static void __init stm32_map_io(void);
static void __init stm32_init_irq(void);
static void __init stm32_init(void);

/*
 * Define a particular platform (board)
 */
static int stm32_platform = PLATFORM_STM32_STM3220G_EVAL;

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
	case PLATFORM_STM32_STM3220G_EVAL:
	default:
		r = DEVICE_STM32F207IG;
		break;
	}
	return r;
}

/*
 * User can (and should) define the platform from U-Boot
 */
static int __init stm32_platform_parse(char *s)
{
	if (!strcmp(s, "stm3220g-eval")) {
		stm32_platform = PLATFORM_STM32_STM3220G_EVAL;
	}

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

#if defined(CONFIG_STM32_USART)
	/*
	 * Configure the USART devices
	 */
	stm32_uart_init();
#endif

#if defined(CONFIG_STM32_MAC)
	/*
	 * Configure the STM32 MAC
	 */
#endif

#if defined(CONFIG_MTD_PHYSMAP)
	/*
	 * Configure external Flash
	 */
#endif
}
