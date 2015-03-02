/*
 * (C) Copyright 2015
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * STM32F4x9 LTDC Frame Buffer device driver
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/dmamem.h>
#include <video/stm32f4_fb.h>
#include <asm/mach-types.h>
#include <mach/stm32.h>
#include <mach/fb.h>


#define DRIVER_NAME	"stm32f4-ltdc"

/* STM32F4 LTDC registers */
#define LTDC_SSCR	0x08
#define LTDC_BPCR	0x0c
#define LTDC_AWCR	0x10
#define LTDC_TWCR	0x14
#define LTDC_GCR	0x18
#define LTDC_SRCR	0x24
#define LTDC_BCCR	0x2c

/* STM32F4 LTDC per-layer registers */
#define LTDC_LAYER_CR(i)	(0x84 + 0x80 * (i))
#define LTDC_LAYER_WHPCR(i)	(0x88 + 0x80 * (i))
#define LTDC_LAYER_WVPCR(i)	(0x8c + 0x80 * (i))
#define LTDC_LAYER_PFCR(i)	(0x94 + 0x80 * (i))
#define LTDC_LAYER_CFBAR(i)	(0xac + 0x80 * (i))
#define LTDC_LAYER_CFBLR(i)	(0xb0 + 0x80 * (i))
#define LTDC_LAYER_CFBLNR(i)	(0xb4 + 0x80 * (i))

/* LTDC GCR Mask */
#define GCR_MASK	((u32)0x0FFE888F)

static struct fb_videomode __devinitdata default_mode_db[] = {
	{
		.name		= "480x272",
		.xres		= 480,
		.yres		= 272,
		.left_margin	= 2,
		.right_margin	= 2,
		.upper_margin	= 2,
		.lower_margin	= 2,
		.hsync_len	= 41,
		.vsync_len	= 10,
		.sync		= FB_SYNC_COMP_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
		.pixclock	= KHZ2PICOS(9000),
	},
};

static struct fb_videomode __devinitdata *fb_mode_db = default_mode_db;
static unsigned int __devinitdata fb_mode_db_size = ARRAY_SIZE(default_mode_db);

static DEFINE_SPINLOCK(fb_lock);

#define LTDC_LAYER_NUM	2

/*
 * Structure containing the private data of the driver
 */
struct stm32f4_ltdc_fb_data {
	struct fb_info *layer_info[LTDC_LAYER_NUM - 1];
	void __iomem *base;

	struct clk *clk;
	struct clk *pix_clk;
	int fb_enabled;
};

struct mfb_info {
	int index;
	int enabled;
	char *id;
	int registered;
	const char *mode_str;
	int default_bpp;
	struct stm32f4_layer_desc *layer_desc;
	unsigned int count;

	/* layer display x offset to physical screen */
	int x_layer_d;
	/* layer display y offset to physical screen */
	int y_layer_d;

	struct stm32f4_ltdc_fb_data *parent;
};

static struct mfb_info mfb_template[] = {
	{
		.index = 0,
		.enabled = 1,
		.id = "Layer0",
		.registered = 0,
		.count = 0,
		.x_layer_d = 0,
		.y_layer_d = 0,
	},
	{
		.index = 1,
		.enabled = 1,
		.id = "Layer1",
		.registered = 0,
		.count = 0,
		.x_layer_d = 50,
		.y_layer_d = 50,
	},
};

static int total_open_layers = 0;

static void ltdc_reload_config(struct stm32f4_ltdc_fb_data *fb)
{
	/* Reload configutation immediately */
	writel(1, fb->base + LTDC_SRCR);
}

static int fb_enable_panel(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct stm32f4_ltdc_fb_data *fb = mfbi->parent;
	struct stm32f4_layer_desc *layer_desc = mfbi->layer_desc;
	int i;
	u32 acc_h_bporch;
	u32 acc_v_bporch;
	int ret = 0;

	if (mfbi->enabled) {
		i = mfbi->index;

		acc_h_bporch = var->hsync_len + var->left_margin;
		acc_v_bporch = var->vsync_len + var->upper_margin;
		writel(
			(acc_h_bporch << 0) |
			((acc_h_bporch + layer_desc->width) << 16),
			fb->base + LTDC_LAYER_WHPCR(i));
		writel(
			(acc_v_bporch << 0) |
			((acc_v_bporch + layer_desc->height) << 16),
			fb->base + LTDC_LAYER_WVPCR(i));

		/* Set pixel format to ARGB8888 */
		writel(0, fb->base + LTDC_LAYER_PFCR(i));

		writel(layer_desc->addr, fb->base + LTDC_LAYER_CFBAR(i));

		writel(((var->xres * 4) << 16) | (var->xres * 4 + 7),
			fb->base + LTDC_LAYER_CFBLR(i));
		writel(layer_desc->height, fb->base + LTDC_LAYER_CFBLNR(i));

		/* Enable layer */
		writel(readl(fb->base + LTDC_LAYER_CR(i)) | 1,
			fb->base + LTDC_LAYER_CR(i));

		ltdc_reload_config(fb);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int fb_disable_panel(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct stm32f4_ltdc_fb_data *fb = mfbi->parent;
	int ret = 0;
	int i;

	i = mfbi->index;

	/* Disable layer */
	writel(readl(fb->base + LTDC_LAYER_CR(i)) & ~1, fb->base + LTDC_LAYER_CR(i));

	ltdc_reload_config(fb);

	return ret;
}

static void enable_lcdc(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct stm32f4_ltdc_fb_data *fb = mfbi->parent;

	if (!fb->fb_enabled) {
		clk_enable(fb->pix_clk);

		/* Enable LTDC */
		writel(readl(fb->base + LTDC_GCR) | (1 << 0), fb->base + LTDC_GCR);

		fb->fb_enabled++;
	}
}

static void disable_lcdc(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct stm32f4_ltdc_fb_data *fb = mfbi->parent;

	if (fb->fb_enabled) {
		/* Disable LTDC */
		writel(readl(fb->base + LTDC_GCR) & ~(1 << 0), fb->base + LTDC_GCR);

		clk_disable(fb->pix_clk);

		fb->fb_enabled = 0;
	}
}

static int fb_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if (var->xoffset < 0)
		var->xoffset = 0;

	if (var->yoffset < 0)
		var->yoffset = 0;

	if (var->xoffset + info->var.xres > info->var.xres_virtual)
		var->xoffset = info->var.xres_virtual - info->var.xres;

	if (var->yoffset + info->var.yres > info->var.yres_virtual)
		var->yoffset = info->var.yres_virtual - info->var.yres;

	/* This driver currently supports only ARGB8888 */
	if (var->bits_per_pixel != 32)
		var->bits_per_pixel = 32;

	switch (var->bits_per_pixel) {
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;

		break;
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	return 0;
}

static void set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = info->par;

	strncpy(fix->id, mfbi->id, strlen(mfbi->id));
	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ypanstep = 1;
}

static void update_lcdc(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = info->par;
	struct stm32f4_ltdc_fb_data *fb = mfbi->parent;
	u32 acc_h_cycles;
	u32 acc_v_cycles;

	if (!mfbi->enabled) {
		fb_disable_panel(info);
		return;
	}

	disable_lcdc(info);

	/* Configure PLLSAI prescalers for LCD */
	clk_set_rate(fb->pix_clk, PICOS2KHZ(var->pixclock) * 1000);
	clk_enable(fb->pix_clk);

	/*
	 * Accumulated cycles starting with back porch:
	 *   sync_len - 1 + back_porch + resolution + front_porch
	 * We substract one to simplify writing to registers.
	 */
	acc_h_cycles = var->hsync_len - 1;
	acc_v_cycles = var->vsync_len - 1;

	/* Sets Synchronization size */
	writel((acc_h_cycles << 16) | acc_v_cycles, fb->base + LTDC_SSCR);

	/* Sets Accumulated Back porch */
	acc_h_cycles += var->left_margin;
	acc_v_cycles += var->upper_margin;
	writel((acc_h_cycles << 16) | acc_v_cycles, fb->base + LTDC_BPCR);

	/* Sets Accumulated Active Width */
	acc_h_cycles += var->xres;
	acc_v_cycles += var->yres;
	writel((acc_h_cycles << 16) | acc_v_cycles, fb->base + LTDC_AWCR);

	/* Sets Total Width */
	acc_h_cycles += var->right_margin;
	acc_v_cycles += var->lower_margin;
	writel((acc_h_cycles << 16) | acc_v_cycles, fb->base + LTDC_TWCR);

	/* Disable uncommon features of LTDC, and invert input pixclock */
	writel((readl(fb->base + LTDC_GCR) & GCR_MASK) | (1 << 28),
		fb->base + LTDC_GCR);

	/* Set background color to black */
	writel(0, fb->base + LTDC_BCCR);


	ltdc_reload_config(fb);

	/* Enable the LCD controller */
	enable_lcdc(info);
}

static int map_video_memory(struct fb_info *info)
{
	u32 smem_len = info->fix.line_length * info->var.yres_virtual;
	dma_addr_t dmem;

#if defined(CONFIG_DMAMEM)
	unsigned long dmem_len;

	if (dmamem_fb_get(&dmem, &dmem_len)) {
		printk(KERN_ERR "Unable to allocate fb memory\n");
		return -ENOMEM;
	}
	if (dmem_len < smem_len) {
		printk(KERN_ERR "No enough fb memory (%ld<%d)\n",
			dmem_len, smem_len);
		return -ENOMEM;
	}
	info->screen_base = (void *)dmem;
	/* Use DMAMEM when want to reuse bootloader FB, don't clear it */
#else
	info->screen_base = kzalloc(smem_len, GFP_KERNEL);
	if (info->screen_base == NULL) {
		printk(KERN_ERR "Unable to allocate fb memory\n");
		return -ENOMEM;
	}
	dmem = virt_to_phys(info->screen_base);
#endif

	mutex_lock(&info->mm_lock);
	info->fix.smem_start = dmem;
	info->fix.smem_len = smem_len;
	mutex_unlock(&info->mm_lock);
	info->screen_size = info->fix.smem_len;

	return 0;
}

static void unmap_video_memory(struct fb_info *info)
{
	mutex_lock(&info->mm_lock);

#if !defined(CONFIG_DMAMEM)
	if (info->screen_base)
		kfree(info->screen_base);
#endif

	info->screen_base = NULL;
	info->fix.smem_start = 0;
	info->fix.smem_len = 0;
	mutex_unlock(&info->mm_lock);
}

/*
 * Using the fb_var_screeninfo in fb_info we set the resolution of this
 * particular framebuffer. This function alters the fb_fix_screeninfo stored
 * in fb_info. It does not alter var in fb_info since we are using that
 * data. This means we depend on the data in var inside fb_info to be
 * supported by the hardware. check_var is always called before
 * set_par to ensure this.
 */
static int fb_set_par(struct fb_info *info)
{
	unsigned long len;
	struct fb_var_screeninfo *var = &info->var;
	struct mfb_info *mfbi = info->par;
	struct stm32f4_layer_desc *layer_desc = mfbi->layer_desc;

	set_fix(info);

	len = info->var.yres_virtual * info->fix.line_length;
	if (len != info->fix.smem_len) {
		if (info->fix.smem_start)
			unmap_video_memory(info);

		/* Memory allocation for framebuffer */
		if (map_video_memory(info)) {
			printk(KERN_ERR "Failed to allocate frame buffer\n");
			return -ENOMEM;
		}
	}

	layer_desc->addr = info->fix.smem_start;

	/* Layer should not be greater than display size */
	layer_desc->width = var->xres_virtual;
	layer_desc->height = var->yres_virtual;
	layer_desc->posx = mfbi->x_layer_d;
	layer_desc->posy = mfbi->y_layer_d;

	if (var->bits_per_pixel != 32) {
		printk(KERN_ERR "Unable to support other bpp now\n");
	}

	layer_desc->en = 1;

	/* Only layer 0 could update LCDC */
	if (mfbi->index == 0)
		update_lcdc(info);

	fb_enable_panel(info);
	return 0;
}

static int fb_open(struct fb_info *info, int user)
{
	struct mfb_info *mfbi = info->par;
	int ret = 0;

	mfbi->index = info->node;
	spin_lock(&fb_lock);

	mfbi->count++;
	if (mfbi->count == 1) {
		pr_debug("open layer index %d\n", mfbi->index);
		fb_check_var(&info->var, info);
		ret = fb_set_par(info);
		if (ret < 0)
			mfbi->count--;
		else
			total_open_layers++;
	}

	spin_unlock(&fb_lock);
	return ret;
}

static int fb_release(struct fb_info *info, int user)
{
	struct mfb_info *mfbi = info->par;
	int ret = 0;

	spin_lock(&fb_lock);
	mfbi->count--;
	if (mfbi->count == 0) {
		pr_debug("release layer index %d\n", mfbi->index);
		ret = fb_disable_panel(info);
		if (ret < 0)
			mfbi->count++;
		else
			total_open_layers--;
	}

	spin_unlock(&fb_lock);
	return ret;
}

static struct fb_ops stm32f4_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = fb_check_var,
	.fb_set_par = fb_set_par,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_open = fb_open,
	.fb_release = fb_release,
};

static int init_fbinfo(struct fb_info *info)
{
	info->device = NULL;
	info->var.activate = FB_ACTIVATE_NOW;
	info->fbops = &stm32f4_fb_ops;
	info->flags = FBINFO_FLAG_DEFAULT;

	/* Allocate colormap */
	fb_alloc_cmap(&info->cmap, 16, 0);
	return 0;
}

static int __devinit install_fb(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;
	struct fb_videomode *db = fb_mode_db;
	unsigned int dbsize = fb_mode_db_size;
	int rc;

	if (init_fbinfo(info))
		return -EINVAL;

	rc = fb_find_mode(&info->var, info, mfbi->mode_str, db, dbsize,
			  &db[0], mfbi->default_bpp);

	if (fb_check_var(&info->var, info)) {
		printk(KERN_ERR "fb_check_var failed");
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	if (register_framebuffer(info) < 0) {
		printk(KERN_ERR "register_framebuffer failed");
		unmap_video_memory(info);
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
	}

	mfbi->registered = 1;
	printk(KERN_INFO "fb%d: %s fb device registered successfully.\n",
		info->node, info->fix.id);

	return 0;
}

static void uninstall_fb(struct fb_info *info)
{
	struct mfb_info *mfbi = info->par;

	if (!mfbi->registered)
		return;

	unregister_framebuffer(info);
	unmap_video_memory(info);
	if (&info->cmap)
		fb_dealloc_cmap(&info->cmap);

	mfbi->registered = 0;
}

#ifdef CONFIG_PM
static int fb_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct stm32f4_ltdc_fb_data *fb = dev_get_drvdata(&pdev->dev);

	clk_disable(fb->pix_clk);
	clk_disable(fb->clk);
	return 0;
}

static int fb_resume(struct platform_device *pdev)
{
	struct stm32f4_ltdc_fb_data *fb = dev_get_drvdata(&pdev->dev);

	clk_enable(fb->clk);
	clk_enable(fb->pix_clk);
	return 0;
}
#else
#define fb_suspend	NULL
#define fb_resume	NULL
#endif

static int __devinit fb_probe(struct platform_device *pdev)
{
	struct stm32f4_fb_platform_data *plat_data = pdev->dev.platform_data;
	struct stm32f4_ltdc_fb_data *fb;
	struct mfb_info *mfbi;
	struct resource *res;
	int ret = 0;
	int i;

	fb = kmalloc(sizeof(struct stm32f4_ltdc_fb_data), GFP_KERNEL);
	if (!fb)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(fb->layer_info); i++) {
		fb->layer_info[i] =
			framebuffer_alloc(sizeof(struct mfb_info),
			&pdev->dev);
		if (!fb->layer_info[i]) {
			dev_err(&pdev->dev, "cannot allocate memory\n");
			ret = ENOMEM;
			goto failed_alloc_framebuffer;
		}
		mfbi = fb->layer_info[i]->par;
		memcpy(mfbi, &mfb_template[i], sizeof(struct mfb_info));
		mfbi->parent = fb;
		mfbi->mode_str = plat_data->mode_str;
		mfbi->default_bpp = 32;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto failed_alloc_framebuffer;
	}
	if (!request_mem_region(res->start, resource_size(res), DRIVER_NAME)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto failed_alloc_framebuffer;
	}

	fb->base = ioremap_nocache(res->start, resource_size(res));
	if (!fb->base) {
		dev_err(&pdev->dev, "cannot map LTDC registers!\n");
		ret = -EFAULT;
		goto failed_ioremap;
	}

	/*
	 * Actually we should do this if !stm32f4_fb_is_running(), but
	 * 'init()' func may configure some LTDC-unrelated stuff (e.g.
	 * LCD touchscreen)
	 */
	plat_data->init(0);

	if (plat_data->modes) {
		fb_mode_db = plat_data->modes;
		fb_mode_db_size = plat_data->modes_size;
	}

	fb->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(fb->clk)) {
		dev_err(&pdev->dev, "unable to get clock\n");
		goto failed_getclock;
	}

	fb->pix_clk = clk_get(NULL, "sai_r_clk");
	if (IS_ERR(fb->pix_clk)) {
		dev_err(&pdev->dev, "unable to get pix clock\n");
		goto failed_getclock;
	}

	if (!stm32f4_fb_is_running())
		clk_enable(fb->clk);
	fb->fb_enabled = 0;

	for (i = 0; i < ARRAY_SIZE(fb->layer_info); i++) {
		fb->layer_info[i]->fix.smem_start = 0;
		mfbi = fb->layer_info[i]->par;
		mfbi->layer_desc =
			kzalloc(sizeof(struct stm32f4_layer_desc), GFP_KERNEL);
		ret = install_fb(fb->layer_info[i]);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to register framebuffer %d\n",
				i);
			goto failed_install_fb;
		}
	}

	dev_set_drvdata(&pdev->dev, fb);
	return 0;

failed_install_fb:
	kfree(mfbi->layer_desc);
failed_getclock:
	iounmap(fb->base);
failed_ioremap:
	release_mem_region(res->start, resource_size(res));
failed_alloc_framebuffer:
	for (i = 0; i < ARRAY_SIZE(fb->layer_info); i++) {
		if (fb->layer_info[i])
			framebuffer_release(fb->layer_info[i]);
	}
	kfree(fb);
	return ret;
}

static int fb_remove(struct platform_device *pdev)
{
	struct stm32f4_ltdc_fb_data *fb = dev_get_drvdata(&pdev->dev);
	struct resource *res;
	int i;

	disable_lcdc(fb->layer_info[0]);
	clk_disable(fb->clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	release_mem_region(res->start, resource_size(res));

	for (i = ARRAY_SIZE(fb->layer_info); i > 0; i--)
		uninstall_fb(fb->layer_info[i - 1]);

	iounmap(fb->base);
	for (i = 0; i < ARRAY_SIZE(fb->layer_info); i++)
		if (fb->layer_info[i])
			framebuffer_release(fb->layer_info[i]);
	kfree(fb);

	return 0;
}

static struct platform_driver fb_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = fb_probe,
	.remove = fb_remove,
	.suspend = fb_suspend,
	.resume = fb_resume,
};

static int __init stm32f4_ltdc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&fb_driver);
	if (ret)
		printk(KERN_ERR
			"stm32f4-fb: failed to register platform driver\n");

	return ret;
}

static void __exit stm32f4_ltdc_exit(void)
{
	platform_driver_unregister(&fb_driver);
}

module_init(stm32f4_ltdc_init);
module_exit(stm32f4_ltdc_exit);

MODULE_AUTHOR("Alexander Potashev");
MODULE_DESCRIPTION("STM32F4xx LCD-TFT Controller framebuffer driver");
MODULE_LICENSE("GPL");
