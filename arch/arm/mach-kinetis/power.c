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

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/module.h>

#include <mach/kinetis.h>
#include <mach/power.h>

/*
 * Enable or disable the clock on a peripheral device (timers, UARTs, USB, etc)
 */
int kinetis_periph_enable(kinetis_clock_gate_t gate, int enable)
{
	volatile u32 *scgc;
	u32 mask;
	int rv;

	/*
	 * Verify the function arguments
	 */
	if (KINETIS_CG_REG(gate) >= KINETIS_SIM_CG_NUMREGS ||
	    KINETIS_CG_IDX(gate) >= KINETIS_SIM_CG_NUMBITS) {
		printk("%s: wrong gate = %x\n", __func__, gate);
		rv = -EINVAL;
		goto out;
	}

	scgc = &KINETIS_SIM->scgc[KINETIS_CG_REG(gate)];
	mask = 1 << KINETIS_CG_IDX(gate);

	if (gate == KINETIS_CG_PORTF && enable) { /* K70 Errata #5234 */
		mask |= (1 << KINETIS_CG_IDX(KINETIS_CG_PORTE));
	}

	if (enable)
		*scgc |= mask;
	else
		*scgc &= ~mask;

	rv = 0;
out:
	return rv;
}
EXPORT_SYMBOL(kinetis_periph_enable);

/*
 * Disable all clocks, except for those that are absolutely required
 * to run the OS kernel. We have to do that because firmware may
 * have enabled some of the clocks for its own operation.
 */
void kinetis_periph_set_minimum(void)
{
	kinetis_periph_enable(KINETIS_CG_OSC1, 0);
	kinetis_periph_enable(KINETIS_CG_UART4, 0);
	kinetis_periph_enable(KINETIS_CG_UART5, 0);
	kinetis_periph_enable(KINETIS_CG_ENET, 0);
	kinetis_periph_enable(KINETIS_CG_DAC0, 0);
	kinetis_periph_enable(KINETIS_CG_DAC1, 0);
	kinetis_periph_enable(KINETIS_CG_RNGA, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXCAN1, 0);
	kinetis_periph_enable(KINETIS_CG_NFC, 0);
	kinetis_periph_enable(KINETIS_CG_SPI2, 0);
	// kinetis_periph_enable(KINETIS_CG_DDR, 0);
	kinetis_periph_enable(KINETIS_CG_SAI1, 0);
	kinetis_periph_enable(KINETIS_CG_ESDHC, 0);
	kinetis_periph_enable(KINETIS_CG_LCDC, 0);
	kinetis_periph_enable(KINETIS_CG_FTM2, 0);
	kinetis_periph_enable(KINETIS_CG_FTM3, 0);
	kinetis_periph_enable(KINETIS_CG_ADC1, 0);
	kinetis_periph_enable(KINETIS_CG_ADC3, 0);
	kinetis_periph_enable(KINETIS_CG_EWM, 0);
	kinetis_periph_enable(KINETIS_CG_CMT, 0);
	kinetis_periph_enable(KINETIS_CG_I2C0, 0);
	kinetis_periph_enable(KINETIS_CG_I2C1, 0);
	kinetis_periph_enable(KINETIS_CG_UART0, 0);
	kinetis_periph_enable(KINETIS_CG_UART1, 0);
	kinetis_periph_enable(KINETIS_CG_UART2, 0);
	kinetis_periph_enable(KINETIS_CG_UART3, 0);
	kinetis_periph_enable(KINETIS_CG_USBFS, 0);
	kinetis_periph_enable(KINETIS_CG_CMP, 0);
	kinetis_periph_enable(KINETIS_CG_VREF, 0);
	kinetis_periph_enable(KINETIS_CG_LLWU, 0);
	kinetis_periph_enable(KINETIS_CG_LPTIMER, 0);
	// kinetis_periph_enable(KINETIS_CG_REGFILE, 0);
	kinetis_periph_enable(KINETIS_CG_DRYICE, 0);
	kinetis_periph_enable(KINETIS_CG_DRYICESECREG, 0);
	kinetis_periph_enable(KINETIS_CG_TSI, 0);
	kinetis_periph_enable(KINETIS_CG_PORTA, 0);
	kinetis_periph_enable(KINETIS_CG_PORTB, 0);
	kinetis_periph_enable(KINETIS_CG_PORTC, 0);
	kinetis_periph_enable(KINETIS_CG_PORTD, 0);
	kinetis_periph_enable(KINETIS_CG_PORTE, 0);
	kinetis_periph_enable(KINETIS_CG_PORTF, 0);
	kinetis_periph_enable(KINETIS_CG_DMAMUX0, 0);
	kinetis_periph_enable(KINETIS_CG_DMAMUX1, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXCAN0, 0);
	kinetis_periph_enable(KINETIS_CG_SPI0, 0);
	kinetis_periph_enable(KINETIS_CG_SPI1, 0);
	kinetis_periph_enable(KINETIS_CG_SAI0, 0);
	kinetis_periph_enable(KINETIS_CG_CRC, 0);
	kinetis_periph_enable(KINETIS_CG_USBHS, 0);
	kinetis_periph_enable(KINETIS_CG_USBDCD, 0);
	kinetis_periph_enable(KINETIS_CG_PDB, 0);
	kinetis_periph_enable(KINETIS_CG_PIT, 0);
	kinetis_periph_enable(KINETIS_CG_FTM0, 0);
	kinetis_periph_enable(KINETIS_CG_FTM1, 0);
	kinetis_periph_enable(KINETIS_CG_ADC0, 0);
	kinetis_periph_enable(KINETIS_CG_ADC2, 0);
	kinetis_periph_enable(KINETIS_CG_RTC, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXBUS, 0);
	kinetis_periph_enable(KINETIS_CG_DMA, 0);
	kinetis_periph_enable(KINETIS_CG_MPU, 0);

	// printk("\nAFTER:\n"); kinetis_periph_print_all();
}
EXPORT_SYMBOL(kinetis_periph_set_minimum);

/*
 * Print status of a peripheral clock gate
 */
void kinetis_periph_print(kinetis_clock_gate_t gate, char *name)
{
	volatile u32 *scgc;
	u32 mask;

	/*
	 * Verify the function arguments
	 */
	if (KINETIS_CG_REG(gate) >= KINETIS_SIM_CG_NUMREGS ||
	    KINETIS_CG_IDX(gate) >= KINETIS_SIM_CG_NUMBITS) {
		goto out;
	}

	scgc = &KINETIS_SIM->scgc[KINETIS_CG_REG(gate)];
	mask = 1 << KINETIS_CG_IDX(gate);

	printk("%s=%d\n", name, (*scgc & mask) ? 1 : 0);

out:
	;
}
EXPORT_SYMBOL(kinetis_periph_print);

/*
 * Print status of all peripheral clock gates
 */
void kinetis_periph_print_all(void)
{
	kinetis_periph_print(KINETIS_CG_OSC1, "OSC1");
	kinetis_periph_print(KINETIS_CG_UART4, "UART4");
	kinetis_periph_print(KINETIS_CG_UART5, "UART5");
	kinetis_periph_print(KINETIS_CG_ENET, "ENET");
	kinetis_periph_print(KINETIS_CG_DAC0, "DAC0");
	kinetis_periph_print(KINETIS_CG_DAC1, "DAC1");
	kinetis_periph_print(KINETIS_CG_RNGA, "RNGA");
	kinetis_periph_print(KINETIS_CG_FLEXCAN1, "FLEXCAN1");
	kinetis_periph_print(KINETIS_CG_NFC, "NFC");
	kinetis_periph_print(KINETIS_CG_SPI2, "SPI2");
	kinetis_periph_print(KINETIS_CG_DDR, "DDR");
	kinetis_periph_print(KINETIS_CG_SAI1, "SAI1");
	kinetis_periph_print(KINETIS_CG_ESDHC, "ESDHC");
	kinetis_periph_print(KINETIS_CG_LCDC, "LCDC");
	kinetis_periph_print(KINETIS_CG_FTM2, "FTM2");
	kinetis_periph_print(KINETIS_CG_FTM3, "FTM3");
	kinetis_periph_print(KINETIS_CG_ADC1, "ADC1");
	kinetis_periph_print(KINETIS_CG_ADC3, "ADC3");
	kinetis_periph_print(KINETIS_CG_EWM, "EWM");
	kinetis_periph_print(KINETIS_CG_CMT, "CMT");
	kinetis_periph_print(KINETIS_CG_I2C0, "I2C0");
	kinetis_periph_print(KINETIS_CG_I2C1, "I2C1");
	kinetis_periph_print(KINETIS_CG_UART0, "UART0");
	kinetis_periph_print(KINETIS_CG_UART1, "UART1");
	kinetis_periph_print(KINETIS_CG_UART2, "UART2");
	kinetis_periph_print(KINETIS_CG_UART3, "UART3");
	kinetis_periph_print(KINETIS_CG_USBFS, "USBFS");
	kinetis_periph_print(KINETIS_CG_CMP, "CMP");
	kinetis_periph_print(KINETIS_CG_VREF, "VREF");
	kinetis_periph_print(KINETIS_CG_LLWU, "LLWU");
	kinetis_periph_print(KINETIS_CG_LPTIMER, "LPTIMER");
	kinetis_periph_print(KINETIS_CG_REGFILE, "REGFILE");
	kinetis_periph_print(KINETIS_CG_DRYICE, "DRYICE");
	kinetis_periph_print(KINETIS_CG_DRYICESECREG, "DRYICESECREG");
	kinetis_periph_print(KINETIS_CG_TSI, "TSI");
	kinetis_periph_print(KINETIS_CG_PORTA, "PORTA");
	kinetis_periph_print(KINETIS_CG_PORTB, "PORTB");
	kinetis_periph_print(KINETIS_CG_PORTC, "PORTC");
	kinetis_periph_print(KINETIS_CG_PORTD, "PORTD");
	kinetis_periph_print(KINETIS_CG_PORTE, "PORTE");
	kinetis_periph_print(KINETIS_CG_PORTF, "PORTF");
	kinetis_periph_print(KINETIS_CG_DMAMUX0, "DMAMUX0");
	kinetis_periph_print(KINETIS_CG_DMAMUX1, "DMAMUX1");
	kinetis_periph_print(KINETIS_CG_FLEXCAN0, "FLEXCAN0");
	kinetis_periph_print(KINETIS_CG_SPI0, "SPI0");
	kinetis_periph_print(KINETIS_CG_SPI1, "SPI1");
	kinetis_periph_print(KINETIS_CG_SAI0, "SAI0");
	kinetis_periph_print(KINETIS_CG_CRC, "CRC");
	kinetis_periph_print(KINETIS_CG_USBHS, "USBHS");
	kinetis_periph_print(KINETIS_CG_USBDCD, "USBDCD");
	kinetis_periph_print(KINETIS_CG_PDB, "PDB");
	kinetis_periph_print(KINETIS_CG_PIT, "PIT");
	kinetis_periph_print(KINETIS_CG_FTM0, "FTM0");
	kinetis_periph_print(KINETIS_CG_FTM1, "FTM1");
	kinetis_periph_print(KINETIS_CG_ADC0, "ADC0");
	kinetis_periph_print(KINETIS_CG_ADC2, "ADC2");
	kinetis_periph_print(KINETIS_CG_RTC, "RTC");
	kinetis_periph_print(KINETIS_CG_FLEXBUS, "FLEXBUS");
	kinetis_periph_print(KINETIS_CG_DMA, "DMA");
	kinetis_periph_print(KINETIS_CG_MPU, "MPU");
}
EXPORT_SYMBOL(kinetis_periph_print_all);
