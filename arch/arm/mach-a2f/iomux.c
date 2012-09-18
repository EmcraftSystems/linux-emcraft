/*
 * linux/arch/arm/mach-a2f/iomux.c
 *
 * Copyright (C) 2011 Vladimir Khusainov, Emcraft Systems
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
#include <linux/clockchips.h>

#include <mach/iomux.h>

/*
 * Provide a description of the the SmartFusion timer hardware interfaces.
 */
#define MSS_IOMUX_BASE	0xE0042100

struct mss_iomux {
	unsigned int	cr[82];
};

#define MSS_IOMUX	((volatile struct mss_iomux *)(MSS_IOMUX_BASE))

#define MSS_IOMUX_INOUT	0x00
#define MSS_IOMUX_IN	0x0A
#define MSS_IOMUX_OUT	0x18

/*
 * Initialize the IOMUXes of the SmartFusion.
 * Conceptually, there are 3 ways to configure the IOMUXes:
 * - in the Libero project
 * - in the U-boot firmware
 * - in the kernel (here).
 * For some of the "key" I/O interfaces of SmartFusion, such as UART
 * or Ethernet, the IOMUXes are configured in the Libero project
 * installed onto the A2F-LNX-EVB board (or included into the A2F-Linux
 * distribtuion otherwise). 
 * Some other interfaces, such as SPI, as not enabled (at the IOMUX
 * level) by the Libero project. For the sake of consistency, it would
 * have been more correct to enable these interface in the Libero
 * project, however, we don't want to have the user update the Libero
 * project, or firmware, on their boards without compelling need. Hence,
 * the decision to do configuration for SPI and other "non-key"
 * interfaces here.
 */
void __init a2f_iomux_init(void)
{
#if defined(CONFIG_A2F_MSS_UART0)
	writel(MSS_IOMUX_OUT,	&MSS_IOMUX->cr[4]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[5]);
#endif
#if defined(CONFIG_A2F_MSS_UART1)
	writel(MSS_IOMUX_OUT,	&MSS_IOMUX->cr[12]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[13]);
#endif
#if defined(CONFIG_A2F_MSS_ETHER)
	writel(MSS_IOMUX_OUT,	&MSS_IOMUX->cr[16]);
	writel(MSS_IOMUX_OUT,	&MSS_IOMUX->cr[17]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[18]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[19]);
	writel(MSS_IOMUX_OUT,	&MSS_IOMUX->cr[20]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[21]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[22]);
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[23]);
	writel(MSS_IOMUX_OUT,	&MSS_IOMUX->cr[24]);
#endif
#if defined(CONFIG_A2F_MSS_SPI0)
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[0]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[1]);
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[2]);
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[3]);
#endif
#if defined(CONFIG_A2F_MSS_SPI1)
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[8]);
	writel(MSS_IOMUX_IN,	&MSS_IOMUX->cr[9]);
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[10]);
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[11]);
#endif
#if defined(CONFIG_A2F_MSS_I2C0)
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[6]);
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[7]);
#endif
#if defined(CONFIG_A2F_MSS_I2C1)
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[14]);
	writel(MSS_IOMUX_INOUT,	&MSS_IOMUX->cr[15]);
#endif
}
