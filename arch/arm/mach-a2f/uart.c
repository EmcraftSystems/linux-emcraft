/*
 * linux/arch/arm/mach-a2f/uart.c
 *
 * Copyright (C) 2010,2011 Vladimir Khusainov, Emcraft Systems
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
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/serial_8250.h>
#include <mach/a2f.h>
#include <mach/clock.h>
#include <mach/uart.h>
#include <mach/fpga.h>

/*
 * The MSS subsystem of SmartFusion contains two UART ports that
 * provide the s/w compatibility with the 16550 device.
 */

#define	MSS_UART_RGSZ  (0x20 - 1)

/*
 * MSS UART_0
 */
#if defined(CONFIG_A2F_MSS_UART0)

#define MSS_UART0_IRQ	10
#define UART0_RST_CLR   (1<<7)

static struct resource mss_uart0_resources[] = {
	{
		.start          = MSS_UART0_BASE,
		.end            = MSS_UART0_BASE + MSS_UART_RGSZ,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = MSS_UART0_IRQ,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port mss_uart0_data[] = {
	{
		.membase     	= (char *) MSS_UART0_BASE,
		.mapbase     	= MSS_UART0_BASE,
		.irq         	= MSS_UART0_IRQ,
		.regshift    	= 2,
		.iotype      	= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{  },
};

static struct platform_device mss_uart0_device = {
	.name           = "serial8250",
	.id             = 0,
	.dev.platform_data = mss_uart0_data,
	.num_resources  = 2,
	.resource       = mss_uart0_resources,
};
#endif	/* CONFIG_A2F_MSS_UART0 */

/*
 * MSS UART_1
 */
#if defined(CONFIG_A2F_MSS_UART1)

#define MSS_UART1_IRQ	11
#define UART1_RST_CLR   (1<<8)

static struct resource mss_uart1_resources[] = {
	{
		.start          = MSS_UART1_BASE,
		.end            = MSS_UART1_BASE + MSS_UART_RGSZ,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = MSS_UART1_IRQ,
		.flags          = IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port mss_uart1_data[] = {
	{
		.membase        = (char *) MSS_UART1_BASE,
		.mapbase        = MSS_UART1_BASE,
		.irq            = MSS_UART1_IRQ,
		.regshift       = 2,
		.iotype         = UPIO_MEM,
		.flags          = UPF_SKIP_TEST,
	},
	{  },
};

static struct platform_device mss_uart1_device = {
	.name           = "serial8250",
	.id             = 1,
	.dev.platform_data = mss_uart1_data,
	.num_resources  = 2,
	.resource       = mss_uart1_resources,
};
#endif	/* CONFIG_A2F_MSS_UART1 */

/*
 * The FPGA may have several additional Core16550 UARTs
 */

#define FPGA_UART0_IRQ	A2F_FPGA_DEMUX_IRQ_MAP(CONFIG_A2F_FPGA_UART0_IRQ_SRC)
#define FPGA_UART1_IRQ	A2F_FPGA_DEMUX_IRQ_MAP(CONFIG_A2F_FPGA_UART1_IRQ_SRC)

/*
 * FPGA UART_0
 */
#if defined(CONFIG_A2F_FPGA_UART0)

static struct resource fpga_uart0_resources[] = {
	{
		.start		= CONFIG_A2F_FPGA_UART0_BASE,
		.end		= CONFIG_A2F_FPGA_UART0_BASE + MSS_UART_RGSZ,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= FPGA_UART0_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port fpga_uart0_data[] = {
	{
		.membase	= (char *) CONFIG_A2F_FPGA_UART0_BASE,
		.mapbase	= CONFIG_A2F_FPGA_UART0_BASE,
		.irq		= FPGA_UART0_IRQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{  },
};

static struct platform_device fpga_uart0_device = {
	.name			= "serial8250",
	.id			= 2,
	.dev.platform_data	= fpga_uart0_data,
	.num_resources		= 2,
	.resource		= fpga_uart0_resources,
};
#endif	/* CONFIG_A2F_FPGA_UART0 */

/*
 * FPGA UART_1
 */
#if defined(CONFIG_A2F_FPGA_UART1)

static struct resource fpga_uart1_resources[] = {
	{
		.start		= CONFIG_A2F_FPGA_UART1_BASE,
		.end		= CONFIG_A2F_FPGA_UART1_BASE + MSS_UART_RGSZ,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= FPGA_UART1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct plat_serial8250_port fpga_uart1_data[] = {
	{
		.membase	= (char *) CONFIG_A2F_FPGA_UART1_BASE,
		.mapbase	= CONFIG_A2F_FPGA_UART1_BASE,
		.irq		= FPGA_UART1_IRQ,
		.regshift	= 2,
		.iotype		= UPIO_MEM,
		.flags		= UPF_SKIP_TEST,
	},
	{  },
};

static struct platform_device fpga_uart1_device = {
	.name			= "serial8250",
	.id			= 3,
	.dev.platform_data	= fpga_uart1_data,
	.num_resources		= 2,
	.resource		= fpga_uart1_resources,
};
#endif	/* CONFIG_A2F_FPGA_UART1 */

#if defined(CONFIG_SERIAL_A2F_CORE_UART)
/*
 * CUART platform device resources
 *
 * Note that the base addresses and irq numbers of the resources
 * below are defined by an FPGA image installed onto SmartFusion.
 *
 */

#define A2F_CUART_0_BASE    0x40050400
#define A2F_CUART_0_IRQ	    0
#define A2F_CUART_1_BASE    0x40050500
#define A2F_CUART_1_IRQ	    1
#define A2F_CUART_2_BASE    0x40050600
#define A2F_CUART_2_IRQ	    2
#define A2F_CUART_3_BASE    0x40050700
#define A2F_CUART_3_IRQ	    3
#define A2F_CUART_4_BASE    0x40050800
#define A2F_CUART_4_IRQ	    4
#define A2F_CUART_5_BASE    0x40050900
#define A2F_CUART_5_IRQ	    5
#define A2F_CUART_6_BASE    0x40050A00
#define A2F_CUART_6_IRQ	    6
#define A2F_CUART_7_BASE    0x40050B00
#define A2F_CUART_7_IRQ	    7

#define CUART_PLAT_RESOURCE(uid)                                            \
static struct resource                  a2f_cuart_## uid ##_resources[] =   \
{                                                                           \
        {                                                                   \
                .start  = A2F_CUART_## uid ##_BASE,                         \
                .end    = A2F_CUART_## uid ##_BASE + 0x14,                     \
                .flags  = IORESOURCE_MEM,                                   \
        },                                                                  \
        {                                                                   \
                .start  = A2F_FPGA_DEMUX_IRQ_MAP(A2F_CUART_## uid ##_IRQ),  \
                .flags  = IORESOURCE_IRQ,                                   \
        },                                                                  \
}

#define CUART_PLAT_DEVICE(uid)                                         \
static struct platform_device           a2f_cuart## uid ##_device =   \
{                                                                      \
        .name                   = "a2f_cuart",		\
        .id                     = uid,                             \
	.dev.platform_data	= &a2f_cuart_clk,                      \
        .resource               = a2f_cuart_## uid ##_resources,       \
        .num_resources          = 2,                                   \
}

static unsigned int a2f_cuart_clk;

#if defined(CONFIG_A2F_CUART0)
CUART_PLAT_RESOURCE(0);
CUART_PLAT_DEVICE(0);
#endif

#if defined(CONFIG_A2F_CUART1)
CUART_PLAT_RESOURCE(1);
CUART_PLAT_DEVICE(1);
#endif

#if defined(CONFIG_A2F_CUART2)
CUART_PLAT_RESOURCE(2);
CUART_PLAT_DEVICE(2);
#endif

#if defined(CONFIG_A2F_CUART3)
CUART_PLAT_RESOURCE(3);
CUART_PLAT_DEVICE(3);
#endif

#if defined(CONFIG_A2F_CUART4)
CUART_PLAT_RESOURCE(4);
CUART_PLAT_DEVICE(4);
#endif

#if defined(CONFIG_A2F_CUART5)
CUART_PLAT_RESOURCE(5);
CUART_PLAT_DEVICE(5);
#endif

#if defined(CONFIG_A2F_CUART6)
CUART_PLAT_RESOURCE(6);
CUART_PLAT_DEVICE(6);
#endif

#if defined(CONFIG_A2F_CUART7)
CUART_PLAT_RESOURCE(7);
CUART_PLAT_DEVICE(7);
#endif
#endif

/*
 * Register the A2F specific UART devices with the kernel.
 */
void __init a2f_uart_init(void)
{
#if defined(CONFIG_A2F_MSS_UART0)
	/*
	 * Bring UART_0 out of the power-up reset.
	 */
	A2F_SYSREG->soft_rst_cr &= ~UART0_RST_CLR;

	/*
	 * Get the reference clock for this UART port
	 */
	mss_uart0_data[0].uartclk = a2f_clock_get(CLCK_PCLK0);

	/*
	 * Register device for UART_0.
	 */
	(void) platform_device_register(&mss_uart0_device);
#endif
#if defined(CONFIG_A2F_MSS_UART1)
	/*
	 * Bring UART_1 out of the power-up reset.
	 */
	A2F_SYSREG->soft_rst_cr &= ~UART1_RST_CLR;

	/*
	 * Get the reference clock for this UART port
	 */
	mss_uart1_data[0].uartclk = a2f_clock_get(CLCK_PCLK1);

	/*
	 * Register device for UART_1.
	 */
	(void) platform_device_register(&mss_uart1_device);
#endif

#if defined(CONFIG_A2F_FPGA_UART0)
	/*
	 * Enable IRQ source within CoreInterrupt
	 */
	a2f_fpga_demux_irq_source_enable(CONFIG_A2F_FPGA_UART0_IRQ_SRC);

	/*
	 * Get the reference clock for this UART port
	 */
	fpga_uart0_data[0].uartclk = a2f_clock_get(CLCK_FPGA);

	/*
	 * Register device for FPGA UART_0
	 */
	(void) platform_device_register(&fpga_uart0_device);
#endif

#if defined(CONFIG_A2F_FPGA_UART1)
	/*
	 * Enable IRQ source within CoreInterrupt
	 */
	a2f_fpga_demux_irq_source_enable(CONFIG_A2F_FPGA_UART1_IRQ_SRC);

	/*
	 * Get the reference clock for this UART port
	 */
	fpga_uart1_data[0].uartclk = a2f_clock_get(CLCK_FPGA);

	/*
	 * Register device for FPGA UART_1
	 */
	(void) platform_device_register(&fpga_uart1_device);
#endif
#if defined(CONFIG_SERIAL_A2F_CORE_UART)
	a2f_cuart_clk = a2f_clock_get(CLCK_FPGA);
#if defined(CONFIG_A2F_CUART0)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_0_IRQ);
	(void) platform_device_register(&a2f_cuart0_device);
#endif
#if defined(CONFIG_A2F_CUART1)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_1_IRQ);
	(void) platform_device_register(&a2f_cuart1_device);
#endif
#if defined(CONFIG_A2F_CUART2)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_2_IRQ);
	(void) platform_device_register(&a2f_cuart2_device);
#endif
#if defined(CONFIG_A2F_CUART3)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_3_IRQ);
	(void) platform_device_register(&a2f_cuart3_device);
#endif
#if defined(CONFIG_A2F_CUART4)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_4_IRQ);
	(void) platform_device_register(&a2f_cuart4_device);
#endif
#if defined(CONFIG_A2F_CUART5)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_5_IRQ);
	(void) platform_device_register(&a2f_cuart5_device);
#endif
#if defined(CONFIG_A2F_CUART6)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_6_IRQ);
	(void) platform_device_register(&a2f_cuart6_device);
#endif
#if defined(CONFIG_A2F_CUART7)
	a2f_fpga_demux_irq_source_enable(A2F_CUART_7_IRQ);
	(void) platform_device_register(&a2f_cuart7_device);
#endif
#endif
}
