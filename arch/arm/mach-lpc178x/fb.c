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
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/delay.h>

#include <mach/lpc178x.h>
#include <mach/fb.h>
#include <mach/power.h>
#include <mach/platform.h>

/*
 * "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70)
 */
#define LPC178X_LCD_IRQ		37

/*
 * LCD panel-specific configuration.
 *
 * Note that left_margin/right_margin and upper_margin/lower_margin are swapped.
 * This is so because these variables are mistakenly swapped
 * in `clcdfb_decode()` in `include/linux/amba/clcd.h`.
 */
static struct clcd_panel ea_lpc1788_lcd_004_panel = {
	.mode		= {
		.name		= "HSD043I9W1-A00-0299",
		.refresh	= 60,
		.xres		= 480,
		.yres		= 272,
		.pixclock	= KHZ2PICOS(9000),
		.left_margin	= 40,	/* Back porch actually */
		.right_margin	= 5,	/* Front porch actually */
		.upper_margin	= 8,	/* Back porch actually */
		.lower_margin	= 8,	/* Front porch actually */
		.hsync_len	= 2,
		.vsync_len	= 2,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
	.width		= -1,
	.height		= -1,
	.tim2		= (TIM2_IVS | TIM2_IHS),
	.cntl		= (CNTL_BGR | CNTL_LCDTFT | CNTL_LCDVCOMP(1) |
				CNTL_LCDBPP16_565),
	.bpp		= 16,
};

static int lpc178x_clcd_setup(struct clcd_fb *fb)
{
	int platform;
	int ret;
	dma_addr_t dma;

	ret = 0;
	fb->panel = NULL;

	/*
	 * Find the correct panel configuration
	 */
	platform = lpc178x_platform_get();
	switch (platform) {
	case PLATFORM_LPC178X_EA_LPC1788:
		fb->panel = &ea_lpc1788_lcd_004_panel;
		break;
	default:
		break;
	}

	if (!fb->panel) {
		ret = -ENODEV;
		goto out;
	}

	fb->fb.fix.smem_len = fb->panel->bpp *
		fb->panel->mode.xres * fb->panel->mode.yres / 8;
	fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev,
		fb->fb.fix.smem_len, &dma, GFP_KERNEL);
	if (!fb->fb.screen_base) {
		printk(KERN_ERR "CLCD: unable to map framebuffer\n");
		ret = -ENOMEM;
		goto out;
	}

	fb->fb.fix.smem_start = dma;

out:
	return ret;
}

static int lpc178x_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
	/*
	 * There is no MMU on LPC178x/7x, no need to call
	 * `dma_mmap_writecombine`.
	 */
	return 0;
}

static void lpc178x_clcd_remove(struct clcd_fb *fb)
{
	dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
		fb->fb.screen_base, fb->fb.fix.smem_start);
}

static struct clcd_board lpc178x_fb_data = {
	.name		= "LPC178x/7x LCDC",
	.check		= clcdfb_check,
	.decode		= clcdfb_decode,
	.disable	= NULL,
	.enable		= NULL,
	.setup		= lpc178x_clcd_setup,
	.mmap		= lpc178x_clcd_mmap,
	.remove		= lpc178x_clcd_remove,
};

static struct amba_device lpc178x_fb_device = {
	.dev = {
		.coherent_dma_mask	= ~0,
		.init_name		= "dev:clcd",
		.platform_data		= &lpc178x_fb_data,
	},
	.res = {
		.start			= LPC178X_LCD_BASE,
		.end			= LPC178X_LCD_BASE + SZ_16K - 1,
		.flags			= IORESOURCE_MEM,
	},
	.dma_mask			= ~0,
	.irq				= { LPC178X_LCD_IRQ, NO_IRQ },
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
		printk(KERN_ERR "lpc178x fb: Could not configure "
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
static struct i2c_board_info __initdata ea_lpc1788_lcd_004_dimmer = {
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

void __init lpc178x_fb_init(void)
{
	int platform;
	int have_lcd;
	int ret;

	/*
	 * Run board-specific code
	 */
	have_lcd = 0;
	ret = 0;
	platform = lpc178x_platform_get();
	switch (platform) {
	case PLATFORM_LPC178X_EA_LPC1788:
		have_lcd = 1;
		/*
		 * This code finally makes the function
		 * `ealcd004_pca9532_probe()` to execute.
		 */
		ret = i2c_add_driver(&ealcd004_pca9532_driver);
		if (ret < 0)
			goto out;
		ret = i2c_register_board_info(0, &ea_lpc1788_lcd_004_dimmer, 1);
		if (ret < 0)
			goto out;
		break;
	case PLATFORM_LPC178X_LNX_EVB:
		/* Do not configure LCD on LPC-LNX-EVB for now */
		break;
	default:
		break;
	}

	if (!have_lcd)
		goto out;

	/*
	 * Enable the power on the LCD controller module of the MCU
	 * before we call `amba_device_register()`. This is required, because
	 * `amba_device_register()` will try to read the AMBA device ID
	 * from the LCD controller module's register map; if the power on
	 * the module is off, its registers are not accessible.
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCLCD_MSK, 1);

	amba_device_register(&lpc178x_fb_device, &iomem_resource);

out:
	if (ret < 0) {
		printk(KERN_ERR "lpc178x fb: Could not initialize "
			"the LCD screen.\n");
	}
}
