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

#include <mach/platform.h>
#include <mach/kinetis.h>
#include <mach/fb.h>
#include <mach/power.h>
#include <linux/imxfb.h>

/*
 * Freescale Kinetis LCD Controller register base
 */
#define KINETIS_LCDC_BASE		(KINETIS_AIPS1PERIPH_BASE + 0x00036000)

/*
 * Freescale Kinetis LCD Controller interrupt
 */
#define KINETIS_LCDC_IRQ	97

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
 * TWR-K70F120M with the TWR-LCD-RGB module
 */
static struct imx_fb_videomode twr_k70f120m_fb_modes[] = {
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

#define KINETIS_LCDC_LDCR_HM_BITS	16
#define KINETIS_LCDC_LDCR_TM_BITS	0

static struct imx_fb_platform_data twr_k70f120m_fb_data = {
	.mode = twr_k70f120m_fb_modes,
	.num_modes = ARRAY_SIZE(twr_k70f120m_fb_modes),

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

void __init kinetis_fb_init(void)
{
	int platform;
	int have_lcd;

	/*
	 * Check if there is an LCD display on our platform
	 */
	have_lcd = 0;
	platform = kinetis_platform_get();
	switch (platform) {
	case PLATFORM_KINETIS_TWR_K70F120M:
		have_lcd = 1;
		kinetis_fb_device.dev.platform_data = &twr_k70f120m_fb_data;
		break;
	case PLATFORM_KINETIS_K70_SOM:
		have_lcd = 0;
		break;
	default:
		break;
	}

	/*
	 * Initialize the LCD controller and register the platform device
	 */
	if (have_lcd) {
		/*
		 * Enable power on the LCD Controller module to make its
		 * register map accessible.
		 */
		kinetis_periph_enable(KINETIS_CG_LCDC, 1);

		platform_device_register(&kinetis_fb_device);
	}
}
