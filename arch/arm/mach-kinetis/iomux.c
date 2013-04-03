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
#include <mach/iomux.h>

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
 * Configuration table for pins used for same functions on both TWR-K70F120M
 * and K70-SOM. The LCD signals are excluded from this tables, because they may
 * change depending on the LCD type.
 */
static const struct kinetis_gpio_pin_config common_iomux[] = {
#if defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE)
	/* E.18 = GPIO (for I2C_SDA) */
	{{KINETIS_GPIO_PORT_E, 18}, KINETIS_GPIO_CONFIG_MUX(1)},
	/* E.19 = GPIO (for I2C_SCL) */
	{{KINETIS_GPIO_PORT_E, 19}, KINETIS_GPIO_CONFIG_MUX(1)},
#endif /* defined(CONFIG_I2C_GPIO) || defined(CONFIG_I2C_GPIO_MODULE) */

#if defined(CONFIG_USB_EHCI_MXC) || defined(CONFIG_USB_EHCI_MXC_MODULE)
	/* A.6 = ULPI_CLK */
	{{KINETIS_GPIO_PORT_A, 6}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.7 = ULPI_DIR */
	{{KINETIS_GPIO_PORT_A, 7}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.8 = ULPI_NXT */
	{{KINETIS_GPIO_PORT_A, 8}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.9 = ULPI_STP */
	{{KINETIS_GPIO_PORT_A, 9}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.10 = ULPI_DATA0 */
	{{KINETIS_GPIO_PORT_A, 10}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.11 = ULPI_DATA1 */
	{{KINETIS_GPIO_PORT_A, 11}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.24 = ULPI_DATA2 */
	{{KINETIS_GPIO_PORT_A, 24}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.25 = ULPI_DATA3 */
	{{KINETIS_GPIO_PORT_A, 25}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.26 = ULPI_DATA4 */
	{{KINETIS_GPIO_PORT_A, 26}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.27 = ULPI_DATA5 */
	{{KINETIS_GPIO_PORT_A, 27}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.28 = ULPI_DATA6 */
	{{KINETIS_GPIO_PORT_A, 28}, KINETIS_GPIO_CONFIG_MUX(2)},
	/* A.29 = ULPI_DATA7 */
	{{KINETIS_GPIO_PORT_A, 29}, KINETIS_GPIO_CONFIG_MUX(2)},

	/* B.8 = RSTOUT_B on TWR-SER2, for resetting the USB PHY */
	{{KINETIS_GPIO_PORT_B, 8}, KINETIS_GPIO_CONFIG_MUX(1)},
#endif /* CONFIG_USB_EHCI_MXC || CONFIG_USB_EHCI_MXC_MODULE */

#if defined(CONFIG_KINETIS_SPI2_GPIO)
	/*
	 * SPI2 bus signals in GPIO mode
	 */
	/* D.11 = SPI2_nCS0 */
	{{KINETIS_GPIO_PORT_D, 11}, KINETIS_GPIO_CONFIG_MUX(1)},
	/* D.12 = SPI2_CLK */
	{{KINETIS_GPIO_PORT_D, 12}, KINETIS_GPIO_CONFIG_MUX(1)},
	/* D.13 = SPI2_MOSI */
	{{KINETIS_GPIO_PORT_D, 13}, KINETIS_GPIO_CONFIG_MUX(1)},
	/* D.14 = SPI2_MISO */
	{{KINETIS_GPIO_PORT_D, 14}, KINETIS_GPIO_CONFIG_MUX(1)},
#endif /* CONFIG_KINETIS_SPI2_GPIO */
};

/*
 * Pin configuration table for TWR-K70F120M, excluding LCD signals.
 */
static const struct kinetis_gpio_pin_config twrk70f120m_iomux[] = {
#if defined(CONFIG_KINETIS_UART2)
	/* E.16 = UART2_TX */
	{{KINETIS_GPIO_PORT_E, 16}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* E.17 = UART2_RX */
	{{KINETIS_GPIO_PORT_E, 17}, KINETIS_GPIO_CONFIG_MUX(3)},
#endif

#if defined(CONFIG_MMC_ESDHC)
	/* E.0 = SDHC0_D1 */
	{{KINETIS_GPIO_PORT_E, 0}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.1 = SDHC0_D0 */
	{{KINETIS_GPIO_PORT_E, 1}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.2 = SDHC0_DCLK */
	{{KINETIS_GPIO_PORT_E, 2}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.3 = SDHC0_CMD */
	{{KINETIS_GPIO_PORT_E, 3}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.4 = SDHC0_D3 */
	{{KINETIS_GPIO_PORT_E, 4}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.5 = SDHC0_D2 */
	{{KINETIS_GPIO_PORT_E, 5}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.28 = GPIO (Card detect) */
//	{{KINETIS_GPIO_PORT_E, 28}, KINETIS_GPIO_CONFIG_MUX(1) | (0xb << 16) | 3},
	{{KINETIS_GPIO_PORT_E, 28}, KINETIS_GPIO_CONFIG_MUX(1) | (0xb << 16)},
#endif
};

/*
 * Pin configuration table for K70-SOM, excluding LCD signals.
 */
static const struct kinetis_gpio_pin_config k70som_iomux[] = {
#if defined(CONFIG_KINETIS_UART0)
	/* D.7 = UART0_TX */
	{{KINETIS_GPIO_PORT_D, 7}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* D.6 = UART0_RX */
	{{KINETIS_GPIO_PORT_D, 6}, KINETIS_GPIO_CONFIG_MUX(3)},
#if defined(CONFIG_KINETIS_UART0_CTSRTS)
	/* B.3 = UART0_CTS */
	{{KINETIS_GPIO_PORT_B, 3}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* B.2 = UART0_RTS */
	{{KINETIS_GPIO_PORT_B, 2}, KINETIS_GPIO_CONFIG_MUX(3)},
#endif /* CONFIG_KINETIS_UART0_CTSRTS */
#endif /* CONFIG_KINETIS_UART0 */

#if defined(CONFIG_KINETIS_UART1)
	/* C.4 = UART1_TX */
	{{KINETIS_GPIO_PORT_C, 4}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* C.3 = UART1_RX */
	{{KINETIS_GPIO_PORT_C, 3}, KINETIS_GPIO_CONFIG_MUX(3)},
#if defined(CONFIG_KINETIS_UART1_CTSRTS)
	/* TBD: UART1_CTS/RTS signals are MUXed with SD card, avoid collision */
	/* E.2 = UART1_CTS */
	{{KINETIS_GPIO_PORT_E, 2}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* E.3 = UART1_RTS */
	{{KINETIS_GPIO_PORT_E, 3}, KINETIS_GPIO_CONFIG_MUX(3)},
#endif /* CONFIG_KINETIS_UART1_CTSRTS */
#endif /* CONFIG_KINETIS_UART1 */

#if defined(CONFIG_KINETIS_UART2)
	/* E.16 = UART2_TX (USB-serial on Emcraft SOM-BSB baseboard) */
	{{KINETIS_GPIO_PORT_E, 16}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* E.17 = UART2_RX (USB-serial on Emcraft SOM-BSB baseboard) */
	{{KINETIS_GPIO_PORT_E, 17}, KINETIS_GPIO_CONFIG_MUX(3)},
#if defined(CONFIG_KINETIS_UART2_CTSRTS)
	/* CTS/RTS are not supported by USB-serial on Emcraft K70-SOM */
#endif /* CONFIG_KINETIS_UART2_CTSRTS */
#endif /* CONFIG_KINETIS_UART2 */

#if defined(CONFIG_KINETIS_UART3)
	/* B.11 = UART3_TX */
	{{KINETIS_GPIO_PORT_B, 11}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* B.10 = UART3_RX */
	{{KINETIS_GPIO_PORT_B, 10}, KINETIS_GPIO_CONFIG_MUX(3)},
#if defined(CONFIG_KINETIS_UART3_CTSRTS)
	/* B.9 = UART3_CTS */
	{{KINETIS_GPIO_PORT_B, 9}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* B.8 = UART3_RTS */
	{{KINETIS_GPIO_PORT_B, 8}, KINETIS_GPIO_CONFIG_MUX(3)},
#endif /* CONFIG_KINETIS_UART3_CTSRTS */
#endif /* CONFIG_KINETIS_UART3 */

#if defined(CONFIG_KINETIS_UART4)
	/* C.15 = UART4_TX */
	{{KINETIS_GPIO_PORT_C, 15}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* C.14 = UART4_RX */
	{{KINETIS_GPIO_PORT_C, 14}, KINETIS_GPIO_CONFIG_MUX(3)},
#if defined(CONFIG_KINETIS_UART4_CTSRTS)
	/* C.13 = UART4_CTS */
	{{KINETIS_GPIO_PORT_C, 13}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* C.12 = UART4_RTS */
	{{KINETIS_GPIO_PORT_C, 12}, KINETIS_GPIO_CONFIG_MUX(3)},
#endif /* CONFIG_KINETIS_UART4_CTSRTS */
#endif /* CONFIG_KINETIS_UART4 */

#if defined(CONFIG_KINETIS_UART5)
	/* E.8 = UART5_TX */
	{{KINETIS_GPIO_PORT_E, 8}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* E.9 = UART5_RX */
	{{KINETIS_GPIO_PORT_E, 9}, KINETIS_GPIO_CONFIG_MUX(3)},
#if defined(CONFIG_KINETIS_UART5_CTSRTS)
	/* E.10 = UART5_CTS */
	{{KINETIS_GPIO_PORT_E, 10}, KINETIS_GPIO_CONFIG_MUX(3)},
	/* E.11 = UART5_RTS */
	{{KINETIS_GPIO_PORT_E, 11}, KINETIS_GPIO_CONFIG_MUX(3)},
#endif /* CONFIG_KINETIS_UART5_CTSRTS */
#endif /* CONFIG_KINETIS_UART5 */

#if defined(CONFIG_GPIOLIB)
	/*
	 * Pin configuration for the User LEDs and the User Button installed
	 * on the SOM-BSB baseboard. Other baseboards may need different pin
	 * configurations.
	 */
#if 0
	/* E.9 = GPIO 137: User button (S2) */
	{{KINETIS_GPIO_PORT_E,  9}, KINETIS_GPIO_CONFIG_PULLUP(1)},
	/* E.11 = GPIO 139: LED DS3 */
	{{KINETIS_GPIO_PORT_E, 11}, KINETIS_GPIO_CONFIG_MUX(1)},
	/* E.12 = GPIO 140: LED DS4 */
	{{KINETIS_GPIO_PORT_E, 12}, KINETIS_GPIO_CONFIG_MUX(1)},
#endif
#endif /* CONFIG_GPIOLIB */


#if defined(CONFIG_MMC_ESDHC)
	/* E.0 = SDHC0_D1 */
	{{KINETIS_GPIO_PORT_E, 0}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.1 = SDHC0_D0 */
	{{KINETIS_GPIO_PORT_E, 1}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.2 = SDHC0_DCLK */
	{{KINETIS_GPIO_PORT_E, 2}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.3 = SDHC0_CMD */
	{{KINETIS_GPIO_PORT_E, 3}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.4 = SDHC0_D3 */
	{{KINETIS_GPIO_PORT_E, 4}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.5 = SDHC0_D2 */
	{{KINETIS_GPIO_PORT_E, 5}, KINETIS_GPIO_CONFIG_MUX(4)},
	/* E.28 = GPIO (Card detect) */
//	{{KINETIS_GPIO_PORT_E, 28}, KINETIS_GPIO_CONFIG_MUX(1) | (0xb << 16) | 3},
	{{KINETIS_GPIO_PORT_E, 28}, KINETIS_GPIO_CONFIG_MUX(1) | (0xb << 16)},
#endif
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
	if (lcdtype == LCD_TWR_LCD_RGB ||
		((lcdtype == LCD_FUT_TWR_NL8048 ||
			lcdtype == LCD_FUT_TWR_PIM_41WVGA) &&
		(platform == PLATFORM_KINETIS_K70_SOM ||
			platform == PLATFORM_KINETIS_TWR_K70F120M))) {
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
		kinetis_gpio_config_table(
			common_iomux, ARRAY_SIZE(common_iomux));
		kinetis_gpio_config_table(
			twrk70f120m_iomux, ARRAY_SIZE(twrk70f120m_iomux));
		break;
	case PLATFORM_KINETIS_K70_SOM:
	case PLATFORM_KINETIS_K61_SOM:
		kinetis_gpio_config_table(
			common_iomux, ARRAY_SIZE(common_iomux));
		kinetis_gpio_config_table(
			k70som_iomux, ARRAY_SIZE(k70som_iomux));
		break;
	default:
		break;
	}
}
