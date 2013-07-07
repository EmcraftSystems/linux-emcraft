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
#include <linux/module.h>
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
 * Type A pins (analog pins)
 */
#define LPC178X_GPIO_CONFIG_A(func,mode,inv,admode,filter,od,dacen) \
	((func  << LPC178X_GPIO_CONFIG_FUNC_BITS ) | \
	(mode   << LPC178X_GPIO_CONFIG_MODE_BITS ) | \
	(inv    << LPC178X_GPIO_CONFIG_INV_BIT   ) | \
	(admode << LPC178X_GPIO_CONFIG_ADMODE_BIT) | \
	(filter << LPC178X_GPIO_CONFIG_FILTER_BIT) | \
	(od     << LPC178X_GPIO_CONFIG_OD_BITS   ) | \
	(dacen  << LPC178X_GPIO_CONFIG_DACEN_BIT ))
/*
 * Type U pins (USB D+ or D- function)
 */
#define LPC178X_GPIO_CONFIG_U(func) \
	(func << LPC178X_GPIO_CONFIG_FUNC_BITS)
/*
 * Type I pins (I2C pins)
 */
#define LPC178X_GPIO_CONFIG_I(func,inv,hs,hidrive) \
	((func   << LPC178X_GPIO_CONFIG_FUNC_BITS  ) | \
	(inv     << LPC178X_GPIO_CONFIG_INV_BIT    ) | \
	(hs      << LPC178X_GPIO_CONFIG_HS_BIT     ) | \
	(hidrive << LPC178X_GPIO_CONFIG_HIDRIVE_BIT))
/*
 * Type W pins (digital pins with selectable input glitch filter)
 *
 * Bit 7 (ADMODE) must be set to 1 for normal operation.
 */
#define LPC178X_GPIO_CONFIG_W(func,mode,hys,inv,filter,slew,od) \
	((func  << LPC178X_GPIO_CONFIG_FUNC_BITS ) | \
	(mode   << LPC178X_GPIO_CONFIG_MODE_BITS ) | \
	(hys    << LPC178X_GPIO_CONFIG_HYS_BIT   ) | \
	(inv    << LPC178X_GPIO_CONFIG_INV_BIT   ) | \
	(1      << LPC178X_GPIO_CONFIG_ADMODE_BIT) | \
	(filter << LPC178X_GPIO_CONFIG_FILTER_BIT) | \
	(slew   << LPC178X_GPIO_CONFIG_SLEW_BIT  ) | \
	(od     << LPC178X_GPIO_CONFIG_OD_BITS   ))

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
 * Set up direction of a GPIO: 1-> out; 0-> in
 */
void lpc178x_gpio_dir(int port, int pin, int d)
{
	struct lpc178x_gpio_dsc gpio_dsc;
	gpio_dsc.port = port;
	gpio_dsc.pin = pin;
	lpc178x_gpio_config_direction(&gpio_dsc, d);
}
EXPORT_SYMBOL(lpc178x_gpio_dir);

/*
 * Define the value of a general-purpose output
 */
void lpc178x_gpio_out(int port, int pin, int v)
{
	struct lpc178x_gpio_dsc gpio_dsc;
	gpio_dsc.port = port;
	gpio_dsc.pin = pin;
	lpc178x_gpout_set(&gpio_dsc, v);
}
EXPORT_SYMBOL(lpc178x_gpio_out);

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

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	/*
	 * Pin configuration for USB
	 */
	/* P0.14 (D) = USB_CONNECT2 */
	{{0, 14}, LPC178X_GPIO_CONFIG_D(3, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.19 (D) = USB_PPWR1 */
	{{1, 19}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.27 (D) = USB_INT1 */
	{{1, 27}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.28 (D) = USB_SCL1 */
	{{1, 28}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.29 (D) = USB_SDA1 */
	{{1, 29}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},

	/* P0.12 (A) = USB_PPWR2 */
	{{0, 12}, LPC178X_GPIO_CONFIG_A(1, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},
	/* P0.13 (A) = USB_UP_LED2 */
	{{0, 13}, LPC178X_GPIO_CONFIG_A(1, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},

	/* P0.29 (U) = USB_D+1 */
	{{0, 29}, LPC178X_GPIO_CONFIG_U(1)},
	/* P0.30 (U) = USB_D-1 */
	{{0, 30}, LPC178X_GPIO_CONFIG_U(1)},
	/* P0.31 (U) = USB_D+2 */
	{{0, 31}, LPC178X_GPIO_CONFIG_U(1)},

	/* P1.30 (A) = USB_VBUS */
	{{1, 30}, LPC178X_GPIO_CONFIG_A(2, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},
	/* P1.31 (A) = USB_OVRCR2 */
	{{1, 31}, LPC178X_GPIO_CONFIG_A(1, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},

	/* P1.18 (A) = USB_UP_LED1 */
	{{1, 18}, LPC178X_GPIO_CONFIG_A(1, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},
#endif /* defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE) */

#if defined(CONFIG_MMC_ARMMMCI) || defined(CONFIG_MMC_ARMMMCI_MODULE)
	/*
	 * Pin configuration for the SD Card Interface
	 */
	/* P1.2 (D) = MCI_CLK */
	{{1,  2}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.3 (D) = MCI_CMD */
	{{1,  3}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.5 (D) = MCI_PWR */
	{{1,  5}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},

	/* P1.6 (D) = MCI_DAT0 */
	{{1,  6}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.7 (D) = MCI_DAT1 */
	{{1,  7}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.11 (D) = MCI_DAT2 */
	{{1, 11}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.12 (D) = MCI_DAT3 */
	{{1, 12}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#endif /* defined(CONFIG_MMC_ARMMMCI) || defined(CONFIG_MMC_ARMMMCI_MODULE) */

#if defined(CONFIG_LPC178X_I2C0) && (defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE))
#error CONFIG_LPC178X_I2C0 and CONFIG_I2C_GPIO cannot be used at the same time
#endif

#if defined(CONFIG_LPC178X_I2C0)
	/*
	 * Pin configuration for the I2C0 interface. I2C1 and I2C2 are not used
	 * on EA-LPC1788.
	 */
	/* P0.27 (I) = I2C0_SDA */
	{{0, 27}, LPC178X_GPIO_CONFIG_I(1, 0, 1, 1)},
	/* P0.28 (I) = I2C0_SDL */
	{{0, 28}, LPC178X_GPIO_CONFIG_I(1, 0, 1, 1)},
#endif

#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	/*
	 * Pin configuration to work with the I2C0 bus in the GPIO mode
	 */
	/* P0.27 (I) = I2C0_SDA (GPIO) */
	{{0, 27}, LPC178X_GPIO_CONFIG_I(0, 0, 0, 0)},
	/* P0.28 (I) = I2C0_SDL (GPIO) */
	{{0, 28}, LPC178X_GPIO_CONFIG_I(0, 0, 0, 0)},
#endif

#if defined(CONFIG_FB_ARMCLCD) || defined(CONFIG_FB_ARMCLCD_MODULE)
	/*
	 * Pin configuration for the LCD interface
	 */
	/* P2.0 (D) = LCDPWR */
	{{2,  0}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.1 (D) = LCDLE */
	{{2,  1}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.2 (D) = LCDDCLK */
	{{2,  2}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.3 (D) = LCDFP */
	{{2,  3}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.4 (D) = LCDENAB */
	{{2,  4}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.5 (D) = LCDLP */
	{{2,  5}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},

	/* P1.20 (D) = LCD D10 */
	{{1, 20}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.21 (D) = LCD D11 */
	{{1, 21}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.22 (D) = LCD D12 */
	{{1, 22}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.23 (D) = LCD D13 */
	{{1, 23}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.24 (D) = LCD D14 */
	{{1, 24}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.25 (D) = LCD D15 */
	{{1, 25}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.26 (D) = LCD D20 */
	{{1, 26}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.27 (D) = LCD D21 */
	{{1, 27}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.28 (D) = LCD D22 */
	{{1, 28}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.29 (D) = LCD D23 */
	{{1, 29}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.6 (D) = LCD D4 */
	{{2,  6}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.7 (D) = LCD D5 */
	{{2,  7}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.8 (D) = LCD D6 */
	{{2,  8}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.9 (D) = LCD D7 */
	{{2,  9}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.12 (D) = LCD D3 */
	{{2, 12}, LPC178X_GPIO_CONFIG_D(5, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P2.13 (D) = LCD D19 */
	{{2, 13}, LPC178X_GPIO_CONFIG_D(7, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#endif /* defined(CONFIG_FB_ARMCLCD) || defined(CONFIG_FB_ARMCLCD_MODULE) */

#if defined(CONFIG_SND_LPC3XXX_SOC) || defined(CONFIG_SND_LPC3XXX_SOC_MODULE)
	/*
	 * Pin configuration for the I2S audio interface
	 */
	/* P0.4 (D) = I2S_RX_SCK */
	{{0,  4}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.5 (D) = I2S_RX_WS */
	{{0,  5}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.6 (D) = I2S_RX_SDA */
	{{0,  6}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.7 (W) = I2S_TX_SCK */
	{{0,  7}, LPC178X_GPIO_CONFIG_W(1, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},
	/* P0.8 (W) = I2S_TX_WS */
	{{0,  8}, LPC178X_GPIO_CONFIG_W(1, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},
	/* P0.9 (W) = I2S_TX_SDA */
	{{0,  9}, LPC178X_GPIO_CONFIG_W(1, LPC178X_NO_PULLUP, 0, 0, 0, 0, 0)},
#endif /* CONFIG_SND_LPC3XXX_SOC || CONFIG_SND_LPC3XXX_SOC_MODULE */
};

/*
 * GPIO pin configuration table for the Emcraft LPC-LNX-EVB board
 */
static const struct lpc178x_gpio_pin_config lpc_lnx_evb_gpio[] = {
#ifdef CONFIG_LPC178X_UART0
	/*
	 * GPIO configuration for UART0
	 */
	/* P0.2 (D) = UART0 TXD */
	{{0,  2}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.3 (D) = UART0 RXD */
	{{0,  3}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#endif /* CONFIG_LPC178X_UART0 */

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

#ifdef CONFIG_LPC178X_SPI0
	/*
	 * GPIO configuration for SPI/SSP0
	 */
	/* P0.15 (D) = SSP0_SCK */
	{{0, 15}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.16 (D) = P0[16] (driver controlled GPIO) */
	{{0, 16}, LPC178X_GPIO_CONFIG_D(0, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.17 (D) = SSP0_MISO */
	{{0, 17}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.18 (D) = SSP0_MOSI */
	{{0, 18}, LPC178X_GPIO_CONFIG_D(2, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#endif
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
	case PLATFORM_LPC178X_LNX_EVB:
		lpc178x_gpio_config_table(
			lpc_lnx_evb_gpio, ARRAY_SIZE(lpc_lnx_evb_gpio));
		break;
	default:
		printk(KERN_WARNING "%s: unsupported platform %d\n", __func__,
			platform);
		break;
	}
}
