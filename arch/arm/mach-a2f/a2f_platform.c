/*
 *  linux/arch/arm/mach-a2f/a2f_platform.c
 *
 *  Copyright (C) 2010 Vladimir Khusainov, Emcraft Systems
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
#include <mach/a2fxxxm3.h>
#include <mach/timer.h>
#include <mach/clock.h>
#include <mach/uart.h>

static void __init a2f_map_io(void);
static void __init a2f_init_irq(void);
static void __init a2f_init(void);

 /*
  * Kernel configuration data. This works as follows:
  * - if r2 is <> zero at the kernel entry, this is treated
  *   as a pointer to the config data tag list;
  * - otherwise, if machine_desc->boot_params <> 0, this is treated
  *   as a pointer to the config data tag list;
  * I use the second option above to provide reasonable 
  * settings for the kernel.
  */

static struct init_tags {
	struct tag_header	h1;
	struct tag_core		core;
	struct tag_header	h2;
	struct tag_mem32	mem;
	struct tag_header	h3;
} a2f_init_tags __initdata = {
	{ tag_size(tag_core), ATAG_CORE },
	{ 1, PAGE_SIZE, 0xFF },
	{ tag_size(tag_mem32), ATAG_MEM },
	{ CONFIG_DRAM_SIZE, CONFIG_DRAM_BASE },
	{ 0, ATAG_NONE }
};

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
 	 * but let's keep these fields filled for consistency.
 	 */
	.phys_io	= MSS_UART0_BASE,
	.io_pg_offst	= (IO_ADDRESS(MSS_UART0_BASE) >> 18) & 0xfffc,

	/* As explained above, I have this to point to ATAG-based
 	 * machine specific parameters.
 	 */
	.boot_params	= (unsigned int) &a2f_init_tags,
	.map_io		= a2f_map_io,
	.init_irq	= a2f_init_irq,
	.timer		= &a2f_timer, 
	.init_machine	= a2f_init,
MACHINE_END

/*
 * Map required regions. 
 * This being the no-MMU Linux, I am not mapping anything
 * since any I/O registers are available at their physical addresses.
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
	 * Configure the A2F clocks. 
	 */
	a2f_clock_init();

	/*
 	 * Configure the UART devices.
 	 */
	a2f_uart_init();
}

/*
 * End of File
 */
