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

#ifndef _MACH_LPC178X_POWER_H_
#define _MACH_LPC178X_POWER_H_

/*
 * 1-bit masks for PCONP (Power Control for Peripherals register) for different
 * peripherals that enable power on them. One of these masks should be passed
 * as the first argument of `lpc178x_periph_enable`.
 */
#define LPC178X_SCC_PCONP_PCLCD_MSK	(1 << 0)
#define LPC178X_SCC_PCONP_PCTIM0_MSK	(1 << 1)
#define LPC178X_SCC_PCONP_PCTIM1_MSK	(1 << 2)
#define LPC178X_SCC_PCONP_PCUART0_MSK	(1 << 3)
#define LPC178X_SCC_PCONP_PCUART1_MSK	(1 << 4)
#define LPC178X_SCC_PCONP_PCI2C0_MSK	(1 << 7)
#define LPC178X_SCC_PCONP_PCUART4_MSK	(1 << 8)
#define LPC178X_SCC_PCONP_PCEMC_MSK	(1 << 11)
#define LPC178X_SCC_PCONP_PCGPIO_MSK	(1 << 15)
#define LPC178X_SCC_PCONP_PCI2C1_MSK	(1 << 19)
#define LPC178X_SCC_PCONP_PCSSP0_MSK	(1 << 21)
#define LPC178X_SCC_PCONP_PCUART2_MSK	(1 << 24)
#define LPC178X_SCC_PCONP_PCUART3_MSK	(1 << 25)
#define LPC178X_SCC_PCONP_PCI2C2_MSK	(1 << 26)
#define LPC178X_SCC_PCONP_PCI2S_MSK	(1 << 27)
#define LPC178X_SCC_PCONP_PCSDC_MSK	(1 << 28)
#define LPC178X_SCC_PCONP_PCGPDMA_MSK	(1 << 29)
#define LPC178X_SCC_PCONP_PCENET_MSK	(1 << 30)
#define LPC178X_SCC_PCONP_PCUSB_MSK	(1 << 31)

#ifndef __ASSEMBLY__

/*
 * Enable or disable power on a peripheral device (timers, UARTs, USB, etc)
 */
extern void lpc178x_periph_enable(u32 pconp_mask, int enable);

#endif /* __ASSEMBLY__ */

#endif	/*_MACH_LPC178X_POWER_H_ */
