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
 * Pin configuration table for K70-SOM, excluding LCD signals.
 *
 * This part of pin configuration will also be used on TWR-K70F120M.
 */
static const struct kinetis_gpio_pin_config k70som_iomux[] = {
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	/* E.18 = GPIO (for I2C_SDA) */
	{{KINETIS_GPIO_PORT_E, 18}, KINETIS_GPIO_CONFIG_MUX(1)},
	/* E.19 = GPIO (for I2C_SCL) */
	{{KINETIS_GPIO_PORT_E, 19}, KINETIS_GPIO_CONFIG_MUX(1)},
#endif /* defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE) */
};

/*
 * GPIO pin configuration table for TWR-K70F120M + TWR-SER + TWR-LCD-RGB
 */
static const struct kinetis_gpio_pin_config twr_lcd_rgb_iomux[] = {
#if defined(CONFIG_KINETIS_FB)
	/* F.0 = GLCD_PCLK */
	{{KINETIS_GPIO_PORT_F,  0}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.1 = GLCD_DE */
	{{KINETIS_GPIO_PORT_F,  1}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.2 = GLCD_HFS */
	{{KINETIS_GPIO_PORT_F,  2}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.3 = GLCD_VFS */
	{{KINETIS_GPIO_PORT_F,  3}, KINETIS_GPIO_CONFIG_DSE(7)},

	/* F.4 = GLCD_D0 */
	{{KINETIS_GPIO_PORT_F,  4}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.5 = GLCD_D1 */
	{{KINETIS_GPIO_PORT_F,  5}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.6 = GLCD_D2 */
	{{KINETIS_GPIO_PORT_F,  6}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.7 = GLCD_D3 */
	{{KINETIS_GPIO_PORT_F,  7}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.8 = GLCD_D4 */
	{{KINETIS_GPIO_PORT_F,  8}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.9 = GLCD_D5 */
	{{KINETIS_GPIO_PORT_F,  9}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.10 = GLCD_D6 */
	{{KINETIS_GPIO_PORT_F, 10}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.11 = GLCD_D7 */
	{{KINETIS_GPIO_PORT_F, 11}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.12 = GLCD_D8 */
	{{KINETIS_GPIO_PORT_F, 12}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.13 = GLCD_D9 */
	{{KINETIS_GPIO_PORT_F, 13}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.14 = GLCD_D10 */
	{{KINETIS_GPIO_PORT_F, 14}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.15 = GLCD_D11 */
	{{KINETIS_GPIO_PORT_F, 15}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.16 = GLCD_D12 */
	{{KINETIS_GPIO_PORT_F, 16}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.17 = GLCD_D13 */
	{{KINETIS_GPIO_PORT_F, 17}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.18 = GLCD_D14 */
	{{KINETIS_GPIO_PORT_F, 18}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.19 = GLCD_D15 */
	{{KINETIS_GPIO_PORT_F, 19}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.20 = GLCD_D16 */
	{{KINETIS_GPIO_PORT_F, 20}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.21 = GLCD_D17 */
	{{KINETIS_GPIO_PORT_F, 21}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.22 = GLCD_D18 */
	{{KINETIS_GPIO_PORT_F, 22}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.23 = GLCD_D19 */
	{{KINETIS_GPIO_PORT_F, 23}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.24 = GLCD_D20 */
	{{KINETIS_GPIO_PORT_F, 24}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.25 = GLCD_D21 */
	{{KINETIS_GPIO_PORT_F, 25}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.26 = GLCD_D22 */
	{{KINETIS_GPIO_PORT_F, 26}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.27 = GLCD_D23 */
	{{KINETIS_GPIO_PORT_F, 27}, KINETIS_GPIO_CONFIG_DSE(7)},
#endif /* CONFIG_KINETIS_FB */
};

/*
 * LCD pin configuration table for K70-SOM + SOM-BSB + EA-LCD-004
 */
static const struct kinetis_gpio_pin_config k70som_ealcd004_iomux[] = {
#if defined(CONFIG_KINETIS_FB)
	/*
	 * Control signals
	 */
	/* F.0 = GLCD_PCLK */
	{{KINETIS_GPIO_PORT_F,  0}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.1 = GLCD_DE */
	{{KINETIS_GPIO_PORT_F,  1}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.2 = GLCD_HFS */
	{{KINETIS_GPIO_PORT_F,  2}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.3 = GLCD_VFS */
	{{KINETIS_GPIO_PORT_F,  3}, KINETIS_GPIO_CONFIG_DSE(7)},

	/*
	 * Blue
	 */
	/* F.5 = GLCD_D1 */
	{{KINETIS_GPIO_PORT_F,  5}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.6 = GLCD_D2 */
	{{KINETIS_GPIO_PORT_F,  6}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.7 = GLCD_D3 */
	{{KINETIS_GPIO_PORT_F,  7}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.8 = GLCD_D4 */
	{{KINETIS_GPIO_PORT_F,  8}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.9 = GLCD_D5 */
	{{KINETIS_GPIO_PORT_F,  9}, KINETIS_GPIO_CONFIG_DSE(7)},

	/*
	 * Green
	 */
	/* F.10 = GLCD_D6 */
	{{KINETIS_GPIO_PORT_F, 10}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.11 = GLCD_D7 */
	{{KINETIS_GPIO_PORT_F, 11}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.12 = GLCD_D8 */
	{{KINETIS_GPIO_PORT_F, 12}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.13 = GLCD_D9 */
	{{KINETIS_GPIO_PORT_F, 13}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.14 = GLCD_D10 */
	{{KINETIS_GPIO_PORT_F, 14}, KINETIS_GPIO_CONFIG_DSE(7)},
	/* F.15 = GLCD_D11 */
	{{KINETIS_GPIO_PORT_F, 15}, KINETIS_GPIO_CONFIG_DSE(7)},

	/*
	 * Red
	 */
	/* F.17 = GLCD_D13 */
	{{KINETIS_GPIO_PORT_F, 17}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.18 = GLCD_D14 */
	{{KINETIS_GPIO_PORT_F, 18}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.19 = GLCD_D15 */
	{{KINETIS_GPIO_PORT_F, 19}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.20 = GLCD_D16 */
	{{KINETIS_GPIO_PORT_F, 20}, KINETIS_GPIO_CONFIG_DSE(5)},
	/* F.21 = GLCD_D17 */
	{{KINETIS_GPIO_PORT_F, 21}, KINETIS_GPIO_CONFIG_DSE(7)},
#endif /* CONFIG_KINETIS_FB */
};

/*
 * Initialize the IOMUX Alternative Functions of the Freescale Kinetis MCU
 */
void __init kinetis_iomux_init(void)
{
	int platform;
	int lcdtype;

	/*
	 * Configure IOs depending on the board we're running on, and
	 * the configuration options we're using.
	 * Let's control platform strictly: if some of it does not need to
	 * play with iomux, it must be present in switch below (otherwise,
	 * the warning message will be printed-out)
	 */
	platform = kinetis_platform_get();
	lcdtype = kinetis_lcdtype_get();
	/*
	 * LCD signals
	 */
	if (lcdtype == LCD_TWR_LCD_RGB) {
		kinetis_gpio_config_table(
			twr_lcd_rgb_iomux, ARRAY_SIZE(twr_lcd_rgb_iomux));
	} else if (lcdtype == LCD_EA_LCD_004 &&
		 platform == PLATFORM_KINETIS_K70_SOM) {
		kinetis_gpio_config_table(
			k70som_ealcd004_iomux,
			ARRAY_SIZE(k70som_ealcd004_iomux));
	} else if (lcdtype == LCD_EA_LCD_004 &&
		 platform == PLATFORM_KINETIS_TWR_K70F120M) {
		pr_err("%s: Configuration of TWR-K70F120M with EA-LCD-004 "
		       "is not supported yet.", __func__);
	} else {
		pr_err("%s: unsupported platform (%d) or type of LCD (%d)\n",
		       __func__, platform, lcdtype);
	}
	/*
	 * Signals other than LCD
	 */
	switch (platform) {
	case PLATFORM_KINETIS_TWR_K70F120M:
	case PLATFORM_KINETIS_K70_SOM:
		kinetis_gpio_config_table(
			k70som_iomux, ARRAY_SIZE(k70som_iomux));
		break;
	default:
		break;
	}
}
