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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/stm32_eth.h>

#include <mach/eth.h>
#include <mach/platform.h>
#include <mach/clock.h>

/*
 * MCU-specific parameters of the Ethernet MAC module
 */
#define LPC18XX_MAC_IRQ		5
#define LPC18XX_MAC_BASE	0x40010000

/*
 * Ethernet platform device resources
 */
static struct resource eth_resources[] = {
	{
		.start	= LPC18XX_MAC_BASE,
		.end	= LPC18XX_MAC_BASE + SZ_8K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= LPC18XX_MAC_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

/*
 * Hitex LPC4350 Eval board Ethernet platform data.
 *
 * Use the same platform data also for the Hitex LPC1850 Eval board
 * since these two boards are compatible.
 */
static struct stm32_eth_data hitex_lpc4350_eth_data = {
	.frame_max_size	= 2044,
	.tx_buf_num	= 24,
	.rx_buf_num	= 32,
	.phy_id		= 1,
};

/*
 * Ethernet platform device instance
 */
static struct platform_device eth_device = {
	.name		= STM32_ETH_DRV_NAME,
	.num_resources	= ARRAY_SIZE(eth_resources),
	.resource	= eth_resources,
/*
 * The .id should be set to -1 to indicate the only instance,
 * but it results in the "PHY: ffffffff:01 - Link is Down"
 * ugly printout from the eth driver, so we set it to 0.
 */
	.id		= 0,
};

/*
 * PLL0* register map
 * Used for PLL0USB at 0x4005001C and for PLL0AUDIO at 0x4005002C.
 *
 * This structure is 0x10 bytes long, it is important when it embedding into
 * `struct lpc18xx_cgu_regs`.
 */
struct lpc18xx_pll0_regs {
	u32 stat;	/* PLL status register */
	u32 ctrl;	/* PLL control register */
	u32 mdiv;	/* PLL M-divider register */
	u32 np_div;	/* PLL N/P-divider register */
};

/*
 * CGU (Clock Generation Unit) register map
 * Should be mapped at 0x40050000.
 */
struct lpc18xx_cgu_regs {
	u32 rsv0[5];
	u32 freq_mon;		/* Frequency monitor */
	u32 xtal_osc_ctrl;	/* XTAL oscillator control */
	struct lpc18xx_pll0_regs pll0usb;	/* PLL0USB registers */
	struct lpc18xx_pll0_regs pll0audio;	/* PLL0AUDIO registers */
	u32 pll0audio_frac;	/* PLL0AUDIO fractional divider */
	u32 pll1_stat;		/* PLL1 status register */
	u32 pll1_ctrl;		/* PLL1 control register */
	u32 idiv[5];		/* IDIVA_CTRL .. IDIVE_CTRL */

	/* BASE_* clock configuration registers */
	u32 safe_clk;
	u32 usb0_clk;
	u32 periph_clk;
	u32 usb1_clk;
	u32 m4_clk;
	u32 spifi_clk;
	u32 spi_clk;
	u32 phy_rx_clk;
	u32 phy_tx_clk;
	u32 apb1_clk;
	u32 apb3_clk;
	u32 lcd_clk;
	u32 vadc_clk;
	u32 sdio_clk;
	u32 ssp0_clk;
	u32 ssp1_clk;
	u32 uart0_clk;
	u32 uart1_clk;
	u32 uart2_clk;
	u32 uart3_clk;
	u32 out_clk;
	u32 rsv1[4];
	u32 apll_clk;
	u32 cgu_out0_clk;
	u32 cgu_out1_clk;
};

/*
 * CGU registers base
 */
#define LPC18XX_CGU_BASE		0x40050000
#define LPC18XX_CGU			((volatile struct lpc18xx_cgu_regs *) \
					LPC18XX_CGU_BASE)

/*
 * Bit offsets in Clock Generation Unit (CGU) registers
 */
/*
 * Crystal oscillator control register (XTAL_OSC_CTRL)
 */
/* Oscillator-pad enable */
#define LPC18XX_CGU_XTAL_ENABLE		(1 << 0)
/* Select frequency range */
#define LPC18XX_CGU_XTAL_HF		(1 << 2)

/*
 * For all CGU clock registers
 */
/* CLK_SEL: Clock source selection */
#define LPC18XX_CGU_CLKSEL_BITS		24
#define LPC18XX_CGU_CLKSEL_MSK		(0x1f << LPC18XX_CGU_CLKSEL_BITS)
/* ENET_RX_CLK */
#define LPC18XX_CGU_CLKSEL_ENET_RX	(0x02 << LPC18XX_CGU_CLKSEL_BITS)
/* ENET_TX_CLK */
#define LPC18XX_CGU_CLKSEL_ENET_TX	(0x03 << LPC18XX_CGU_CLKSEL_BITS)
/* Crystal oscillator */
#define LPC18XX_CGU_CLKSEL_XTAL		(0x06 << LPC18XX_CGU_CLKSEL_BITS)
/* PLL1 */
#define LPC18XX_CGU_CLKSEL_PLL1		(0x09 << LPC18XX_CGU_CLKSEL_BITS)
/* Block clock automatically during frequency change */
#define LPC18XX_CGU_AUTOBLOCK_MSK	(1 << 11)
/*
 * PLL1 control register
 */
/* Power-down */
#define LPC18XX_CGU_PLL1CTRL_PD_MSK		(1 << 0)
/* Input clock bypass control */
#define LPC18XX_CGU_PLL1CTRL_BYPASS_MSK		(1 << 1)
/* PLL feedback select */
#define LPC18XX_CGU_PLL1CTRL_FBSEL_MSK		(1 << 6)
/* PLL direct CCO output */
#define LPC18XX_CGU_PLL1CTRL_DIRECT_MSK		(1 << 7)
/* Post-divider division ratio P. The value applied is 2**P. */
#define LPC18XX_CGU_PLL1CTRL_PSEL_BITS		8
#define LPC18XX_CGU_PLL1CTRL_PSEL_MSK \
	(3 << LPC18XX_CGU_PLL1CTRL_PSEL_BITS)
/* Pre-divider division ratio */
#define LPC18XX_CGU_PLL1CTRL_NSEL_BITS		12
#define LPC18XX_CGU_PLL1CTRL_NSEL_MSK \
	(3 << LPC18XX_CGU_PLL1CTRL_NSEL_BITS)
/* Feedback-divider division ratio (M) */
#define LPC18XX_CGU_PLL1CTRL_MSEL_BITS		16
#define LPC18XX_CGU_PLL1CTRL_MSEL_MSK \
	(0xff << LPC18XX_CGU_PLL1CTRL_MSEL_BITS)
/*
 * PLL1 status register
 */
/* PLL1 lock indicator */
#define LPC18XX_CGU_PLL1STAT_LOCK	(1 << 0)

/*
 * RGU (Reset Generation Unit) register map
 */
struct lpc18xx_rgu_regs {
	u32 rsv0[64];

	u32 ctrl0;		/* Reset control register 0 */
	u32 ctrl1;		/* Reset control register 1 */
	u32 rsv1[2];

	u32 status0;		/* Reset status register 0 */
	u32 status1;		/* Reset status register 1 */
	u32 status2;		/* Reset status register 2 */
	u32 status3;		/* Reset status register 3 */
	u32 rsv2[12];

	u32 active_status0;	/* Reset active status register 0 */
	u32 active_status1;	/* Reset active status register 1 */
	u32 rsv3[170];

	u32 ext_stat[64];	/* Reset external status registers */
};

/*
 * RGU registers base
 */
#define LPC18XX_RGU_BASE		0x40053000
#define LPC18XX_RGU			((volatile struct lpc18xx_rgu_regs *) \
					LPC18XX_RGU_BASE)

/*
 * RESET_CTRL0 register
 */
/* ETHERNET_RST */
#define LPC18XX_RGU_CTRL0_ETHERNET	(1 << 22)

/*
 * CREG (Configuration Registers) register map
 */
struct lpc18xx_creg_regs {
	u32 rsv0;
	u32 creg0;		/* Chip configuration register 0 */
	u32 rsv1[62];
	u32 m4memmap;		/* ARM Cortex-M4 memory mapping */
	u32 rsv2;
	u32 creg1;		/* Chip configuration register 1 */
	u32 creg2;		/* Chip configuration register 2 */
	u32 creg3;		/* Chip configuration register 3 */
	u32 creg4;		/* Chip configuration register 4 */
	u32 creg5;		/* Chip configuration register 5 */
	u32 dmamux;		/* DMA muxing control */
	u32 rsv3[2];
	u32 etbcfg;		/* ETB RAM configuration */
	u32 creg6;		/* Chip configuration register 6 */
	u32 m4txevent;		/* Cortex-M4 TXEV event clear */
	u32 rsv4[51];
	u32 chipid;		/* Part ID */
	u32 rsv5[127];
	u32 m0txevent;		/* Cortex-M0 TXEV event clear */
	u32 m0appmemmap;	/* ARM Cortex-M0 memory mapping */
};

/*
 * CREG registers base
 */
#define LPC18XX_CREG_BASE		0x40043000
#define LPC18XX_CREG			((volatile struct lpc18xx_creg_regs *) \
					LPC18XX_CREG_BASE)

/*
 * CREG6 register
 */
/* Selects the Ethernet mode */
#define LPC18XX_CREG_CREG6_ETHMODE_BITS		0
#define LPC18XX_CREG_CREG6_ETHMODE_MSK \
	(7 << LPC18XX_CREG_CREG6_ETHMODE_BITS)
#define LPC18XX_CREG_CREG6_ETHMODE_MII \
	(0 << LPC18XX_CREG_CREG6_ETHMODE_BITS)
/* EMC_CLK divided clock select */
#define LPC18XX_CREG_CREG6_EMCCLKSEL_MSK	(1 << 16)

/*
 * Initialize the Ethernet controller
 */
static int __init eth_setup(void)
{
	int timeout;
	int rv;

	/*
	 * Initialize clock driver to make Ethernet clock available
	 * to the driver.
	 */
	lpc18xx_clock_init();

	/*
	 * This clock configuration is valid only for MII
	 */
	LPC18XX_CGU->phy_rx_clk =
		LPC18XX_CGU_CLKSEL_ENET_RX | LPC18XX_CGU_AUTOBLOCK_MSK;
	LPC18XX_CGU->phy_tx_clk =
		LPC18XX_CGU_CLKSEL_ENET_TX | LPC18XX_CGU_AUTOBLOCK_MSK;

	/*
	 * Choose the MII Ethernet mode
	 */
	LPC18XX_CREG->creg6 =
		(LPC18XX_CREG->creg6 & ~LPC18XX_CREG_CREG6_ETHMODE_MSK) |
		LPC18XX_CREG_CREG6_ETHMODE_MII;

	/*
	 * Reset the Ethernet module of the MCU
	 */
	LPC18XX_RGU->ctrl0 = LPC18XX_RGU_CTRL0_ETHERNET;

	/*
	 * Wait for the Ethernet module to exit the reset state
	 */
	timeout = 10;
	rv = -ETIMEDOUT;
	while (timeout-- > 0) {
		if (!(LPC18XX_RGU->active_status0 &
		    LPC18XX_RGU_CTRL0_ETHERNET)) {
			mdelay(1);
		} else {
			timeout = 0;
			rv = 0;
		}
	}

	if (rv < 0) {
		pr_err("%s: Reset of the Ethernet module timed out.\n",
			__func__);
	}

	return rv;
}

void __init lpc18xx_eth_init(void)
{
	int	platform;

	/*
	 * Initialize the Ethernet controller
	 */
	if (eth_setup() < 0)
		goto out;

	/*
	 * Fix-up and register platform device
	 */
	platform = lpc18xx_platform_get();
	switch (platform) {
	case PLATFORM_LPC18XX_HITEX_LPC4350_EVAL:
	case PLATFORM_LPC18XX_HITEX_LPC1850_EVAL:
		eth_device.dev.platform_data = &hitex_lpc4350_eth_data;
		break;
	default:
		break;
	}

	platform_device_register(&eth_device);
out:
	;
}
