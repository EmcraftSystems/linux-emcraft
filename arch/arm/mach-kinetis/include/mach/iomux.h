/*
 * (C) Copyright 2012
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

#ifndef _MACH_KINETIS_IOMUX_H_
#define _MACH_KINETIS_IOMUX_H_

#include <linux/init.h>

/*
 * Bits and bit groups inside the PCR registers (Pin Control Registers)
 */
/* Pin Mux Control (selects pin function) */
#define KINETIS_GPIO_CONFIG_MUX_BITS	8
/* Pull Enable (pull-down by default) */
#define KINETIS_GPIO_CONFIG_PE_BIT	1
#define KINETIS_GPIO_CONFIG_PE_MSK	(1 << KINETIS_GPIO_CONFIG_PE_BIT)
/* Pull Select (0=pull-down, 1=pull-up) */
#define KINETIS_GPIO_CONFIG_PS_BIT	0
#define KINETIS_GPIO_CONFIG_PS_MSK	(1 << KINETIS_GPIO_CONFIG_PS_BIT)
/* Drive Strength Enable (high drive strength) */
#define KINETIS_GPIO_CONFIG_DSE_MSK	(1 << 6)

/* Interrupt */
#define KINETIS_GPIO_CONFIG_IRQC_MSK	(0xf << 16)
#define KINETIS_GPIO_CONFIG_IRQC_LOW	(0x8 << 16)
#define KINETIS_GPIO_CONFIG_IRQC_RISE	(0x9 << 16)
#define KINETIS_GPIO_CONFIG_IRQC_FALL	(0xa << 16)
#define KINETIS_GPIO_CONFIG_IRQC_EITHER	(0xb << 16)
#define KINETIS_GPIO_CONFIG_IRQC_HIGH	(0xc << 16)

/*
 * These macros should be used to compute the value for the second argument of
 * `kinetis_gpio_config()` (`regval`). This value will be copied into a PCR
 * register.
 */
/* The simplest macro that only allow to configure the MUX bits */
#define KINETIS_GPIO_CONFIG_MUX(mux) \
	(mux << KINETIS_GPIO_CONFIG_MUX_BITS)
/* Also enable the pull-down register */
#define KINETIS_GPIO_CONFIG_PULLDOWN(mux) \
	(KINETIS_GPIO_CONFIG_MUX(mux) | KINETIS_GPIO_CONFIG_PE_MSK)
/* Also enable the pull-up register */
#define KINETIS_GPIO_CONFIG_PULLUP(mux) \
	(KINETIS_GPIO_CONFIG_MUX(mux) | \
	KINETIS_GPIO_CONFIG_PE_MSK | KINETIS_GPIO_CONFIG_PS_MSK)
/* Also enable high drive strength */
#define KINETIS_GPIO_CONFIG_DSE(mux) \
	(KINETIS_GPIO_CONFIG_MUX(mux) | KINETIS_GPIO_CONFIG_DSE_MSK)
/*
 * TBD: similar macros with more options
 */

/*
 * Number of pins in all ports
 */
#define KINETIS_GPIO_PORT_PINS		32
/*
 * Maximum possible number of GPIO ports (A..F on K70)
 */
#define KINETIS_GPIO_PORTS	6

/*
 * PORTx register map
 */
struct kinetis_port_regs {
	u32 pcr[32];	/* Pin Control Registers */
	u32 gpclr;	/* Global Pin Control Low Register */
	u32 gpchr;	/* Global Pin Control High Register */
	u32 rsv0[6];
	u32 isfr;	/* Interrupt Status Flag Register */
	u32 rsv1[7];
	u32 dfer;	/* Digital Filter Enable Register */
	u32 dfcr;	/* Digital Filter Clock Register */
	u32 dfwr;	/* Digital Filter Width Register */
};

/*
 * PORTx registers base
 */
#define KINETIS_PORT_BASE(port)		(KINETIS_AIPS0PERIPH_BASE + \
					0x00049000 + (port) * 0x1000)
#define KINETIS_PORT(port)		((volatile struct kinetis_port_regs *) \
					KINETIS_PORT_BASE(port))

/*
 * GPIO ports
 */
#define KINETIS_GPIO_PORT_A	0
#define KINETIS_GPIO_PORT_B	1
#define KINETIS_GPIO_PORT_C	2
#define KINETIS_GPIO_PORT_D	3
#define KINETIS_GPIO_PORT_E	4
#define KINETIS_GPIO_PORT_F	5

/*
 * GPIO descriptor
 */
struct kinetis_gpio_dsc {
	unsigned int port;	/* GPIO port */
	unsigned int pin;	/* GPIO pin */
};

struct kinetis_gpio_pin_config {
	struct kinetis_gpio_dsc dsc;
	u32 regval;	/* Value for writing into the PCR register */
};

int kinetis_gpio_config(const struct kinetis_gpio_dsc *dsc, u32 regval);
int kinetis_gpio_config_mask(const struct kinetis_gpio_dsc *dsc, u32 mask);
int kinetis_gpio_config_unmask(const struct kinetis_gpio_dsc *dsc, u32 mask);

void __init kinetis_iomux_init(void);

#endif /* _MACH_KINETIS_IOMUX_H_ */
