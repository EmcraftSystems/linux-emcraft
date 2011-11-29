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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>

#include <mach/lpc178x.h>
#include <mach/power.h>
#include <mach/platform.h>

/*
 * Bits and bit groups inside the IOCON registers
 */
/* Selects pin function */
#define LPC178X_GPIO_CONFIG_FUNC_BITS	0
/*
 * Selects function mode (LPC178X_NO_PULLUP, LPC178X_PULLDOWN, LPC178X_PULLUP
 * or LPC178X_REPEATER)
 */
#define LPC178X_GPIO_CONFIG_MODE_BITS	3
/* Hysteresis (1=enable, 0=disable) */
#define LPC178X_GPIO_CONFIG_HYS_BIT	5
/* Input polarity (1=input is inverted) */
#define LPC178X_GPIO_CONFIG_INV_BIT	6
/* Select Analog/Digital mode (0=analog, 1=digital) */
#define LPC178X_GPIO_CONFIG_ADMODE_BIT	7
#define LPC178X_GPIO_CONFIG_FILTER_BIT	8	/* Types A and W */
#define LPC178X_GPIO_CONFIG_HS_BIT	8	/* Type I */
#define LPC178X_GPIO_CONFIG_SLEW_BIT	9	/* Types D and W */
#define LPC178X_GPIO_CONFIG_HIDRIVE_BIT	9	/* Type I */
#define LPC178X_GPIO_CONFIG_OD_BITS	10
#define LPC178X_GPIO_CONFIG_DACEN_BIT	16

/*
 * These macros should be used to compute the value for the second argument of
 * `lpc178x_gpio_config()` (`regval`). This value will be copied into an IOCON
 * register.
 */
/*
 * Type D pins (digital pins)
 */
#define LPC178X_GPIO_CONFIG_D(func,mode,hys,inv,slew,od) \
	((func << LPC178X_GPIO_CONFIG_FUNC_BITS) | \
	(mode  << LPC178X_GPIO_CONFIG_MODE_BITS) | \
	(hys   << LPC178X_GPIO_CONFIG_HYS_BIT  ) | \
	(inv   << LPC178X_GPIO_CONFIG_INV_BIT  ) | \
	(slew  << LPC178X_GPIO_CONFIG_SLEW_BIT ) | \
	(od    << LPC178X_GPIO_CONFIG_OD_BITS  ))
/*
 * TBD: similar macros for other pin types (A, U, I, W)
 */

/*
 * Function mode for pins.
 *
 * One of these should be passed into LPC178X_GPIO_CONFIG_[DAUIW]() macros as
 * the "mode" argument.
 */
#define LPC178X_NO_PULLUP	0
#define LPC178X_PULLDOWN	1
#define LPC178X_PULLUP		2
#define LPC178X_REPEATER	3

/*
 * GPIO descriptor
 */
struct lpc178x_gpio_dsc {
	unsigned int port;	/* GPIO port */
	unsigned int pin;	/* GPIO pin */
};

struct lpc178x_gpio_pin_config {
	struct lpc178x_gpio_dsc dsc;
	u32 regval;	/* Value for writing into the IOCON register */
};

/*
 * List of available GPIO pins:
 *    P0[31:0]; P1[31:0]; P2[31:0]; P3[31:0]; P4[31:0]; P5[4:0]
 */
/* Number of GPIO ports */
#define LPC178X_GPIO_PORTS		6
/* Number of pins in all ports except the last one */
#define LPC178X_GPIO_NORMAL_PORT_PINS	32
/* Number of pins in the last port */
#define LPC178X_GPIO_LAST_PORT_PINS	5

/*
 * IOCON (pin connect) registers base
 */
#define LPC178X_IOCON_BASE		(LPC178X_APB0PERIPH_BASE + 0x0002C000)
/*
 * Address of the IOCON register for the given pin
 */
#define LPC178X_IOCON_PIN_ADDR(port,pin) \
	(LPC178X_IOCON_BASE + (port) * 0x80 + (pin) * 4)
/*
 * Reference to the IOCON register for the given pin
 */
#define LPC178X_IOCON(port,pin) \
	(*(volatile u32 *)LPC178X_IOCON_PIN_ADDR(port,pin))

/*
 * GPIO register map
 * Should be mapped at (0x20098000 + port * 0x20).
 */
struct lpc178x_gpio_regs {
	u32 fiodir;	/* Fast GPIO Port Direction control register */
	u32 rsv0[3];
	u32 fiomask;	/* Fast Mask register for port */
	u32 fiopin;	/* Fast Port Pin value register using FIOMASK */
	u32 fioset;	/* Fast Port Output Set register using FIOMASK */
	u32 fioclr;	/* Fast Port Output Clear register using FIOMASK */
};

/*
 * GPIO registers base
 */
#define LPC178X_GPIO_BASE		(LPC178X_AHB_PERIPH_BASE + 0x00018000)
#define LPC178X_GPIO_PORT_ADDR(port)	(LPC178X_GPIO_BASE + (port) * 0x20)
#define LPC178X_GPIO(port) \
	((volatile struct lpc178x_gpio_regs *)LPC178X_GPIO_PORT_ADDR(port))

/*
 * Check that the given (port, pin) pair is a valid LPC178x/7x GPIO pin.
 * Returns 0 on success, -EINVAL otherwise.
 */
static inline int lpc178x_validate_gpio(const struct lpc178x_gpio_dsc *dsc)
{
	int rv;

	rv = 0;

	/*
	 * P0[31:0]; P1[31:0]; P2[31:0]; P3[31:0]; P4[31:0]; P5[4:0]
	 */
	if (!dsc || dsc->port >= LPC178X_GPIO_PORTS ||
	    dsc->pin >= LPC178X_GPIO_NORMAL_PORT_PINS ||
	    (dsc->port == LPC178X_GPIO_PORTS - 1 &&
	     dsc->pin >= LPC178X_GPIO_LAST_PORT_PINS)) {
		printk(KERN_ERR "%s: incorrect params %d.%d.\n", __func__,
			dsc ? dsc->port : -1,
			dsc ? dsc->pin  : -1);
		rv = -EINVAL;
	}

	return rv;
}

/*
 * Configure the specified GPIO pin.
 * Returns 0 on success, -EINVAL otherwise.
 */
int lpc178x_gpio_config(const struct lpc178x_gpio_dsc *dsc, u32 regval)
{
	int rv;

	rv = lpc178x_validate_gpio(dsc);
	if (rv == 0)
		LPC178X_IOCON(dsc->port, dsc->pin) = regval;

	return rv;
}

/*
 * Configure a set of GPIO pins using the given configuration table.
 * Returns 0 on success.
 */
int lpc178x_gpio_config_table(
	const struct lpc178x_gpio_pin_config *table, unsigned int len)
{
	unsigned int i;
	int rv;

	for (i = 0; i < len; i ++) {
		rv = lpc178x_gpio_config(&table[i].dsc, table[i].regval);
		if (rv != 0)
			goto out;
	}

	rv = 0;
out:
	return rv;
}

/*
 * output=0: Set a GPIO pin as an input.
 * output=1: Set a GPIO pin as an output.
 *
 * Returns 0 on success, -EINVAL otherwise.
 */
int lpc178x_gpio_config_direction(const struct lpc178x_gpio_dsc *dsc, int output)
{
	int rv;

	rv = lpc178x_validate_gpio(dsc);
	if (rv == 0) {
		if (output)
			LPC178X_GPIO(dsc->port)->fiodir |= (1 << dsc->pin);
		else
			LPC178X_GPIO(dsc->port)->fiodir &= ~(1 << dsc->pin);
	}

	return rv;
}

/*
 * Set an output GPIO pin to the state specified (1, 0).
 * Returns 0 on success, -EINVAL otherwise.
 */
int lpc178x_gpout_set(const struct lpc178x_gpio_dsc *dsc, int state)
{
	int rv;

	rv = lpc178x_validate_gpio(dsc);
	if (rv == 0) {
		if (state)
			 LPC178X_GPIO(dsc->port)->fioset = (1 << dsc->pin);
		else
			 LPC178X_GPIO(dsc->port)->fioclr = (1 << dsc->pin);
	}

	return rv;
}

/*
 * Return the state of an input GPIO.
 * Returns 0 or 1 on success, -EINVAL otherwise.
 */
int lpc178x_gpin_get(const struct lpc178x_gpio_dsc *dsc)
{
	int rv;

	rv = lpc178x_validate_gpio(dsc);
	if (rv == 0)
		rv = (LPC178X_GPIO(dsc->port)->fiopin & (1 << dsc->pin)) ? 1 : 0;

	return rv;
}

/*
 * GPIO pin configuration table for EA-LPC1788-32
 */
static const struct lpc178x_gpio_pin_config ea_lpc1788_gpio[] = {
	/*
	 * GPIO configuration for UART
	 */
#ifdef CONFIG_LPC178X_UART0
	/* P0.2 (D) = UART0 TXD */
	{{0,  2}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.3 (D) = UART0 RXD */
	{{0,  3}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#endif /* CONFIG_LPC178X_UART0 */

#ifdef CONFIG_LPC178X_UART1
#error Configuration of GPIO pins for UART1 is not available
#endif /* CONFIG_LPC178X_UART1 */

#ifdef CONFIG_LPC178X_UART2
	/* P0.10 (D) = U2_TXD */
	{{0, 10}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.11 (D) = U2_RXD */
	{{0, 11}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#endif /* CONFIG_LPC178X_UART2 */

#ifdef CONFIG_LPC178X_UART3
#error Configuration of GPIO pins for UART3 is not available
#endif /* CONFIG_LPC178X_UART3 */

#ifdef CONFIG_LPC178X_UART4
#error Configuration of GPIO pins for UART4 is not available
#endif /* CONFIG_LPC178X_UART4 */

#ifdef CONFIG_LPC178X_ETHER
	/*
	 * GPIO configuration for Ethernet
	 */
	/* P1.0 (D) = RMII ENET_TXD0 */
	{{1,  0}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.1 (D) = RMII ENET_TXD1 */
	{{1,  1}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.4 (D) = RMII ENET_TX_EN */
	{{1,  4}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.8 (D) = RMII CRS */
	{{1,  8}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.9 (D) = RMII RXD0 */
	{{1,  9}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.10 (D) = RMII RXD1 */
	{{1, 10}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.14 (D) = RMII RXER */
	{{1, 14}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.15 (D) = RMII CLK */
	{{1, 15}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.16 (D) = RMII MCD */
	{{1, 16}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.17 (D) = RMII MDIO */
	{{1, 17}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#endif /* CONFIG_LPC178X_ETHER */
};

/*
 * Initialize the GPIO Alternative Functions of the LPC178x/7x.
 */
void __init lpc178x_iomux_init(void)
{
	int platform;

	/*
	 * Enable power on GPIO. This is not really necessary, because power
	 * on GPIO is enabled on SoC reset.
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCGPIO_MSK, 1);

	/*
	 * Configure IOs depending on the board we're running on, and
	 * the configuration options we're using.
	 * Let's control platform strictly: if some of it does not need to
	 * play with iomux, it must be present in switch below (otherwise,
	 * the warning message will be printed-out)
	 */
	platform = lpc178x_platform_get();
	switch (platform) {
	case PLATFORM_LPC178X_EA_LPC1788:
		lpc178x_gpio_config_table(
			ea_lpc1788_gpio, ARRAY_SIZE(ea_lpc1788_gpio));
		break;
	default:
		printk(KERN_WARNING "%s: unsupported platform %d\n", __func__,
			platform);
		break;
	}
}
