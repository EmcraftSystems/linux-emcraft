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
#include <linux/kernel.h>
#include <linux/errno.h>

#include <mach/kinetis.h>
#include <mach/power.h>
#include <mach/platform.h>

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
 * Clock gates for the I/O ports: 0..5 <-> A..F
 *
 * These values can be passed into the `kinetis_periph_enable()` function.
 */
static const kinetis_clock_gate_t port_clock_gate[] = {
	KINETIS_CG_PORTA, KINETIS_CG_PORTB, KINETIS_CG_PORTC,
	KINETIS_CG_PORTD, KINETIS_CG_PORTE, KINETIS_CG_PORTF,
};

/*
 * Bits and bit groups inside the PCR registers (Pin Control Registers)
 */
/* Pin Mux Control (selects pin function) */
#define KINETIS_GPIO_CONFIG_MUX_BITS	8
/* Pull Enable (pull-down by default) */
#define KINETIS_GPIO_CONFIG_PE_BIT	1
#define KINETIS_GPIO_CONFIG_PE_MSK	(1 << KINETIS_GPIO_CONFIG_PE_BIT)
/* Drive Strength Enable (high drive strength) */
#define KINETIS_GPIO_CONFIG_DSE_MSK	(1 << 6)

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

/*
 * Check that the given (port, pin) pair is a valid Kinetis GPIO pin.
 * Returns 0 on success, -EINVAL otherwise.
 */
static inline int kinetis_validate_gpio(const struct kinetis_gpio_dsc *dsc)
{
	int rv;

	rv = 0;

	/*
	 * A[31:0]; B[31:0]; C[31:0]; D[31:0]; E[31:0]; optionally F[31:0]
	 */
	if (!dsc || dsc->port >= KINETIS_GPIO_PORTS ||
	    dsc->pin >= KINETIS_GPIO_PORT_PINS) {
		printk(KERN_ERR "%s: incorrect params %d.%d.\n", __func__,
			dsc ? dsc->port : -1,
			dsc ? dsc->pin  : -1);
		rv = -EINVAL;
	}

	return rv;
}

/*
 * Configure the specified GPIO pin.
 * The clocks on the necessary ports will be enabled automatically.
 *
 * Returns 0 on success, -EINVAL otherwise.
 */
int kinetis_gpio_config(const struct kinetis_gpio_dsc *dsc, u32 regval)
{
	int rv;

	/*
	 * Verify the function arguments
	 */
	rv = kinetis_validate_gpio(dsc);
	if (rv != 0)
		goto out;

	/*
	 * Enable the clock on the port we are going to use
	 */
	rv = kinetis_periph_enable(port_clock_gate[dsc->port], 1);
	if (rv != 0)
		goto out;

	/*
	 * Configure the pin
	 */
	KINETIS_PORT(dsc->port)->pcr[dsc->pin] = regval;

	rv = 0;
out:
	return rv;
}

/*
 * Configure a set of GPIO pins using the given configuration table.
 * Returns 0 on success.
 */
int kinetis_gpio_config_table(
	const struct kinetis_gpio_pin_config *table, unsigned int len)
{
	unsigned int i;
	int rv;

	for (i = 0; i < len; i ++) {
		rv = kinetis_gpio_config(&table[i].dsc, table[i].regval);
		if (rv != 0)
			goto out;
	}

	rv = 0;
out:
	return rv;
}

/*
 * GPIO pin configuration table for TWR-K70F120M + TWR-SER + TWR-LCD-RGB
 */
static const struct kinetis_gpio_pin_config twr_k70f120m_gpio[] = {
};

/*
 * Initialize the GPIO Alternative Functions of the Freescale Kinetis MCU
 */
void __init kinetis_iomux_init(void)
{
	int platform;

	/*
	 * Configure IOs depending on the board we're running on, and
	 * the configuration options we're using.
	 * Let's control platform strictly: if some of it does not need to
	 * play with iomux, it must be present in switch below (otherwise,
	 * the warning message will be printed-out)
	 */
	platform = kinetis_platform_get();
	switch (platform) {
	case PLATFORM_KINETIS_TWR_K70F120M:
		kinetis_gpio_config_table(
			twr_k70f120m_gpio, ARRAY_SIZE(twr_k70f120m_gpio));
		break;
	default:
		printk(KERN_WARNING "%s: unsupported platform %d\n", __func__,
			platform);
		break;
	}
}
