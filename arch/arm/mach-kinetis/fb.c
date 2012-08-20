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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/imxfb.h>
#include <linux/spi/spi.h>

#include <mach/platform.h>
#include <mach/kinetis.h>
#include <mach/fb.h>
#include <mach/power.h>
#include <mach/gpio.h>
#include <mach/clock.h>

/*
 * Freescale Kinetis LCD Controller register base
 */
#define KINETIS_LCDC_BASE		(KINETIS_AIPS1PERIPH_BASE + 0x00036000)

/*
 * Freescale Kinetis LCD Controller interrupt
 */
#define KINETIS_LCDC_IRQ	97

/*
 * LCDC DMA control register
 */
#define KINETIS_LCDC_LDCR_HM_BITS	16
#define KINETIS_LCDC_LDCR_TM_BITS	0

/*
 * Framebuffer platform device resources
 */
static struct resource kinetis_fb_resources[] = {
	{
		.start	= KINETIS_LCDC_BASE,
		.end	= KINETIS_LCDC_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= KINETIS_LCDC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/*
 * TWR-K70F120M (or other compatible board) with the TWR-LCD-RGB module
 */
static struct imx_fb_videomode twr_lcd_rgb_fb_modes[] = {
	{
		.mode = {
			.name		= "Seiko 43WQW3T",
			.refresh	= 66,	/* VSYNC rate in Hz */
			.xres		= 480,
			.yres		= 272,
			.pixclock	= KHZ2PICOS(10000),	/* 10 MHz */
			.hsync_len	= 41,	/* Horiz. pulse width */
			.left_margin	= 3,	/* Horiz. front porch */
			.right_margin	= 2,	/* Horiz. back porch */
			.vsync_len	= 10,	/* Vert. pulse width */
			.upper_margin	= 2,	/* Vert. front porch */
			.lower_margin	= 2,	/* Vert. back porch */
		},
		/*
		 * The screen is 24bpp, but every pixel occupies 32 bits
		 * of memory.
		 */
		.bpp		= 32,
		.pcr		=
			PCR_END_SEL |		/* Big Endian */
			PCR_TFT | PCR_COLOR |	/* Color TFT */
			PCR_SCLK_SEL |		/* Always enable LSCLK */
			PCR_SCLKIDLE |
			PCR_CLKPOL |		/* Polarities */
			PCR_FLMPOL |
			PCR_LPPOL,
	},
};

static struct imx_fb_platform_data twr_lcd_rgb_fb_data = {
	.mode = twr_lcd_rgb_fb_modes,
	.num_modes = ARRAY_SIZE(twr_lcd_rgb_fb_modes),

	/* LSCR1 is not supported on Kinetis */
	.lscr1		= 0x00000000,
	/* Disable PWM contrast control */
	.pwmr		= 0x00000000,
	/*
	 * DMA control register value. We use default values for the
	 * `DMA high mark` and the `DMA trigger mark`. The burst length is
	 * dynamic.
	 */
	.dmacr		=
		(0x04 << KINETIS_LCDC_LDCR_HM_BITS) |
		(0x60 << KINETIS_LCDC_LDCR_TM_BITS),
};

/*
 * Future Electronics TWR-PIM-NL8048BC19-02C
 */
static struct imx_fb_videomode fut_twr_nl8048_fb_modes[] = {
	{
		.mode = {
			.name		= "NEC NL8048BC19-02",
			.refresh	= 56,	/* VSYNC rate in Hz */
			.xres		= 800,
			.yres		= 480,
			.pixclock	= KHZ2PICOS(30000),	/* 30 MHz */
			.hsync_len	= 44,	/* Horiz. pulse width */
			.left_margin	= 90,	/* Horiz. front porch */
			.right_margin	= 90,	/* Horiz. back porch */
			.vsync_len	= 15,	/* Vert. pulse width */
			.upper_margin	= 15,	/* Vert. front porch */
			.lower_margin	= 15,	/* Vert. back porch */
		},
		/*
		 * The screen is 24bpp, but every pixel occupies 32 bits
		 * of memory.
		 */
		.bpp		= 32,
		.pcr		=
			PCR_END_SEL |		/* Big Endian */
			PCR_TFT | PCR_COLOR |	/* Color TFT */
			PCR_SCLK_SEL |		/* Always enable LSCLK */
			PCR_SCLKIDLE |
			PCR_CLKPOL |		/* Polarities */
			PCR_FLMPOL |
			PCR_LPPOL,
	},
};

static struct imx_fb_platform_data fut_twr_nl8048_fb_data = {
	.mode = fut_twr_nl8048_fb_modes,
	.num_modes = ARRAY_SIZE(fut_twr_nl8048_fb_modes),

	/* LSCR1 is not supported on Kinetis */
	.lscr1		= 0x00000000,
	/* Disable PWM contrast control */
	.pwmr		= 0x00000000,
	/*
	 * DMA control register value. We use default values for the
	 * `DMA high mark` and the `DMA trigger mark`. The burst length is
	 * dynamic.
	 */
	.dmacr		=
		(0x04 << KINETIS_LCDC_LDCR_HM_BITS) |
		(0x60 << KINETIS_LCDC_LDCR_TM_BITS),
};

#ifdef CONFIG_KINETIS_SPI2_GPIO
/*
 * Future Electronics TWR-PIM-41WVGA
 */
static struct imx_fb_videomode fut_twr_pim_41wvga_fb_modes[] = {
	{
		.mode = {
			.name		= "NEC NL8048HL11",
			.refresh	= 60,	/* VSYNC rate in Hz */
			.xres		= 800,
			.yres		= 480,
			.pixclock	= KHZ2PICOS(23800),	/* 23.8 MHz */
			.left_margin	= 4,	/* Horiz. front porch */
			.hsync_len	= 1,	/* Horiz. pulse width */
			.right_margin	= 4,	/* Horiz. back porch */
			.upper_margin	= 7,	/* Vert. front porch */
			.vsync_len	= 3,	/* Vert. pulse width */
			.lower_margin	= 4,	/* Vert. back porch */
		},
		/*
		 * The screen is 24bpp, but every pixel occupies 32 bits
		 * of memory.
		 */
		.bpp		= 32,
		.pcr		=
			PCR_END_SEL |		/* Big Endian */
			PCR_TFT | PCR_COLOR |	/* Color TFT */
			PCR_SCLK_SEL |		/* Always enable LSCLK */
			PCR_SCLKIDLE |
			PCR_CLKPOL |		/* Polarities */
			PCR_FLMPOL |
			PCR_LPPOL,
	},
};

static struct imx_fb_platform_data fut_twr_pim_41wvga_fb_data = {
	.mode = fut_twr_pim_41wvga_fb_modes,
	.num_modes = ARRAY_SIZE(fut_twr_pim_41wvga_fb_modes),

	/* LSCR1 is not supported on Kinetis */
	.lscr1		= 0x00000000,
	/* Disable PWM contrast control */
	.pwmr		= 0x00000000,
	/*
	 * DMA control register value. We use default values for the
	 * `DMA high mark` and the `DMA trigger mark`. The burst length is
	 * dynamic.
	 */
	.dmacr		=
		(0x04 << KINETIS_LCDC_LDCR_HM_BITS) |
		(0x60 << KINETIS_LCDC_LDCR_TM_BITS),
};
#endif /* CONFIG_KINETIS_SPI2_GPIO */

/*
 * EA-LCD-004 with the K70-SOM module on SOM-BSB
 */
static struct imx_fb_videomode ea_lcd_004_fb_modes[] = {
	{
		.mode = {
			.name		= "HSD043I9W1-A00-0299",
			.refresh	= 66,	/* VSYNC rate in Hz */
			.xres		= 480,
			.yres		= 272,
			.pixclock	= KHZ2PICOS(10000),	/* 10 MHz */
			.hsync_len	= 2,	/* Horiz. pulse width */
			.left_margin	= 5,	/* Horiz. front porch */
			.right_margin	= 40,	/* Horiz. back porch */
			.vsync_len	= 2,	/* Vert. pulse width */
			.upper_margin	= 8,	/* Vert. front porch */
			.lower_margin	= 8,	/* Vert. back porch */
		},
		/*
		 * The screen is 16bpp
		 */
		.bpp		= 16,
		.pcr		=
			PCR_TFT | PCR_COLOR |	/* Color TFT */
			PCR_SCLK_SEL |		/* Always enable LSCLK */
			PCR_SCLKIDLE |
			PCR_CLKPOL |		/* Polarities */
			PCR_FLMPOL |
			PCR_LPPOL,
	},
};

static struct imx_fb_platform_data ea_lcd_004_fb_data = {
	.mode = ea_lcd_004_fb_modes,
	.num_modes = ARRAY_SIZE(ea_lcd_004_fb_modes),

	/* LSCR1 is not supported on Kinetis */
	.lscr1		= 0x00000000,
	/* Disable PWM contrast control */
	.pwmr		= 0x00000000,
	/*
	 * DMA control register value. We use default values for the
	 * `DMA high mark` and the `DMA trigger mark`. The burst length is
	 * dynamic.
	 */
	.dmacr		=
		(0x04 << KINETIS_LCDC_LDCR_HM_BITS) |
		(0x60 << KINETIS_LCDC_LDCR_TM_BITS),
};

/*
 * Framebuffer platform device instance
 */
static struct platform_device kinetis_fb_device = {
	.name = "imx-fb",
	.id = 0,
	.num_resources = ARRAY_SIZE(kinetis_fb_resources),
	.resource = kinetis_fb_resources,
	.dev = {
		.coherent_dma_mask = 0xFFFFFFFF,
	},
};

/*
 * I2C SMBus register of the PCA9532 chip
 */
#define PCA9532_REG_LS0		0x6
#define PCA9532_LED_REG(led)	(PCA9532_REG_LS0 + (led >> 2))
#define PCA9532_LED_NUM(led)	(led & 0x3)

/*
 * Indices of outputs of PCA9532 on the EA-LCD-004 board
 */
#define EALCD004_LCD_3V3_EN	0
#define EALCD004_LCD_DISP_EN	4
#define EALCD004_BL_CONTRAST	8

/*
 * We use this function to run the LCD initialization code.
 * This function will be called by the kernel when the I2C bus driver is
 * registered.
 */
static int ealcd004_pca9532_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	/*
	 * Configure the LCD screen on the EA-LCD-004 board
	 */
	/*
	 * Set 0% contrast to hide blinking of the LCD screen during
	 * initialization. Zero contrast can be set by driving the BL_CONTRAST
	 * output low.
	 */
	ret = i2c_smbus_write_byte_data(client,
		PCA9532_LED_REG(EALCD004_BL_CONTRAST),
		1 << (2 * PCA9532_LED_NUM(EALCD004_BL_CONTRAST)));
	if (ret < 0)
		goto out;

	/*
	 * Enable display of the input data.
	 * This can be done by driving the LCD_DISP_EN output high.
	 */
	ret = i2c_smbus_write_byte_data(client,
		PCA9532_LED_REG(EALCD004_LCD_DISP_EN),
		0x00);
	if (ret < 0)
		goto out;

	/*
	 * Enable the 3.3V power.
	 * This can be done by driving the LCD_3V3_EN low.
	 */
	ret = i2c_smbus_write_byte_data(client,
		PCA9532_LED_REG(EALCD004_LCD_3V3_EN),
		1 << (2 * PCA9532_LED_NUM(EALCD004_LCD_3V3_EN)));
	if (ret < 0)
		goto out;

	/*
	 * Wait for the configuration to settle. This is required to avoid the
	 * white blinking. A 100ms delay is not enough.
	 */
	msleep(200);

	/*
	 * Finally set the 100% contrast for the LCD screen.
	 * Full contrast can be set by driving the BL_CONTRAST output high.
	 */
	ret = i2c_smbus_write_byte_data(client,
		PCA9532_LED_REG(EALCD004_BL_CONTRAST),
		0x00);
	if (ret < 0)
		goto out;

out:
	if (ret < 0) {
		printk(KERN_ERR "kinetis fb: Could not configure "
			"the LCD screen on the EA-LCD-004 board.\n");
	}

	return 0;
}

static int ealcd004_pca9532_remove(struct i2c_client *client)
{
	return 0;
}

/*
 * Use our custom driver `ea-lcd-004-pca9532` for the PCA9532 chip at I2C
 * address 0x64 located at on EA-LCD-004 board.
 */
static struct i2c_board_info __initdata ea_lcd_004_dimmer = {
	I2C_BOARD_INFO("ea-lcd-004-pca9532", 0x64),
};

static const struct i2c_device_id ealcd004_pca9532_id[] = {
	{ "ea-lcd-004-pca9532", 0 },
	{ }
};

static struct i2c_driver ealcd004_pca9532_driver = {
	.driver = {
		.name = "ea-lcd-004-pca9532",
	},
	.probe = ealcd004_pca9532_probe,
	.remove = ealcd004_pca9532_remove,
	.id_table = ealcd004_pca9532_id,
};

#if defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT) || \
    defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT_MODULE)
/*
 * I2C-connected Freescale CRTouch touchscreen device installed
 * on the TWR-LCD-RGB board. We register it with the driver `crtouch_mt`.
 */
static struct i2c_board_info __initdata twr_lcd_rgb_crtouch = {
	I2C_BOARD_INFO("crtouchId", 0x49),
};
#endif /* CONFIG_TOUCHSCREEN_CRTOUCH_MT */

#ifdef CONFIG_KINETIS_SPI2_GPIO
static struct spi_board_info nec_8048_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "nec_8048_spi",
		.bus_num		= 0,
		.chip_select		= 0,
		.max_speed_hz		= 375000,
		/* Chip select GPIO */
		.controller_data	=
			(void *) KINETIS_GPIO_MKPIN(KINETIS_GPIO_PORT_D, 11),
	},
};
#endif /* CONFIG_KINETIS_SPI2_GPIO */

void __init kinetis_fb_init(void)
{
	int platform;
	int lcdtype;
	int ret;

	/*
	 * Check if there is an LCD display on our platform
	 */
	ret = 0;
	platform = kinetis_platform_get();
	lcdtype = kinetis_lcdtype_get();
	if (lcdtype == LCD_TWR_LCD_RGB) {
		kinetis_fb_device.dev.platform_data = &twr_lcd_rgb_fb_data;

#if defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT) || \
    defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT_MODULE)
		/*
		 * Register the I2C-connected CRTouch touchscreen installed
		 * on TWR-LCD-RGB.
		 */
		ret = i2c_register_board_info(0, &twr_lcd_rgb_crtouch, 1);
		if (ret < 0)
			goto out;
#endif /* CONFIG_TOUCHSCREEN_CRTOUCH_MT */
	} else if (lcdtype == LCD_EA_LCD_004 &&
		 platform == PLATFORM_KINETIS_K70_SOM) {
		kinetis_fb_device.dev.platform_data = &ea_lcd_004_fb_data;

		/*
		 * The LCD screen on the EA-LCD-004 board must be configured
		 * via I2C.
		 *
		 * This code finally makes the function
		 * `ealcd004_pca9532_probe()` to execute.
		 */
		ret = i2c_add_driver(&ealcd004_pca9532_driver);
		if (ret < 0)
			goto out;
		ret = i2c_register_board_info(0, &ea_lcd_004_dimmer, 1);
		if (ret < 0)
			goto out;

	} else if (lcdtype == LCD_EA_LCD_004 &&
		 platform == PLATFORM_KINETIS_TWR_K70F120M) {
		pr_err("%s: Configuration of TWR-K70F120M with EA-LCD-004 "
		       "is not supported yet.", __func__);
	} else if (lcdtype == LCD_FUT_TWR_NL8048 &&
		   (platform == PLATFORM_KINETIS_K70_SOM ||
		    platform == PLATFORM_KINETIS_TWR_K70F120M)) {
		kinetis_fb_device.dev.platform_data = &fut_twr_nl8048_fb_data;
#ifdef CONFIG_KINETIS_SPI2_GPIO
	} else if (lcdtype == LCD_FUT_TWR_PIM_41WVGA &&
		   (platform == PLATFORM_KINETIS_K70_SOM ||
		    platform == PLATFORM_KINETIS_TWR_K70F120M)) {
		kinetis_fb_device.dev.platform_data =
			&fut_twr_pim_41wvga_fb_data;

		spi_register_board_info(nec_8048_spi_board_info,
			ARRAY_SIZE(nec_8048_spi_board_info));

#if defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT) || \
    defined(CONFIG_TOUCHSCREEN_CRTOUCH_MT_MODULE)
		/*
		 * Register the I2C-connected CRTouch touchscreen installed
		 * on TWR-PIM-41WVGA.
		 */
		ret = i2c_register_board_info(0, &twr_lcd_rgb_crtouch, 1);
		if (ret < 0)
			goto out;
#endif /* CONFIG_TOUCHSCREEN_CRTOUCH_MT */
#endif /* CONFIG_KINETIS_SPI2_GPIO */
	}

	/*
	 * Initialize the LCD controller and register the platform device
	 */
	if (kinetis_fb_device.dev.platform_data) {
		/*
		 * Enable power on the LCD Controller module to make its
		 * register map accessible.
		 */
		kinetis_periph_enable(KINETIS_CG_LCDC, 1);
		/*
		 * Adjust the LCDC clock divider values
		 */
		kinetis_lcdc_adjust_clock_divider(PICOS2KHZ(
			((struct imx_fb_platform_data *)
				kinetis_fb_device.dev.platform_data)->
			mode[0].mode.pixclock) * 1000,
			kinetis_clock_get(CLOCK_CCLK));

		platform_device_register(&kinetis_fb_device);
	}

out:
	if (ret < 0) {
		pr_err("kinetis fb: Could not initialize the LCD screen "
		       "(%d).\n", ret);
	}
}
