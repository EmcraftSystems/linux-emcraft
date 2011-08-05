/*
 * nokia6100fb.c - framebuffer driver for the Nokia6100 LCD.
 *
 * Author: Vladimir Khusainov, vlad@emcraft.com
 * Copyright 2011 Emcraft Systems
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/nokia6100fb.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>

/*
 * Driver verbosity level: 0->silent; >0->verbose (the higher,
 * the more verbose, up to 4).
 */
static int nokia6100fb_debug = 0;

/*
 * User can change verbosity of the driver when installing the module
 */
module_param(nokia6100fb_debug, int, S_IRUGO);
MODULE_PARM_DESC(nokia6100fb_debug, "Nokia6100 LCD driver verbosity level");

/*
 * ... or when booting the kernel if the driver is linked in statically
 */
#ifndef MODULE
static int __init nokia6100fb_debug_setup(char *str)
{
	get_option(&str, &nokia6100fb_debug);
	return 1;
}
__setup("nokia6100fb_debug=", nokia6100fb_debug_setup);
#endif

/*
 * Service to print debug messages
 */
#define d_printk(level, fmt, args...)					\
	if (nokia6100fb_debug >= level) printk(KERN_INFO "%s: " fmt,	\
		  			       __func__, ## args)

/*
 * Define the geometry of the LCD software and hardware buffers.
 * Code in this driver makes the following assumptions:
 * - In the software framebuffer (which is seen by user-space
 *   applications) pixels are 16 bits each, with RGB taking 4 
 *   bits each, and the left-most 4 bits being unused
 * - In the hardware LCD "memory buffer", which mimics the format
 *   of a bytes sequnece that can be fed directly to the LCD over SPI,
 *   pixels are 12 bits each and are packed 2 pixels into 3 bytes.
 * - In the SPI data sequence, each byte of the pixel data is preceeded
 *   by a service byte, which contains a 1 in the right-most bit,
 *   indicating that the following byte is a "data byte" (rather
 *   than a "command byte", which has a 0 in the right-most bit).
 * - SPI frames are 9 bits each = 1 bit of the service data (command or
 *   data byte) + 8 bits of acommand or data byte.
 *   ...
 * Resolution of the SPI LCD in pixels:
 */
#define NOKIA6100FB_X_RS	132
#define NOKIA6100FB_Y_RS	132
#define NOKIA6100FB_BUF_LEN	(NOKIA6100FB_X_RS * NOKIA6100FB_Y_RS)

/*
 * Size of the LCD "hardware memory buffer" in bytes:
 * - each pixel is 12 bits (3 bytes per 2 pixels)
 * - each pixel byte is preceded by a service byte.
 */
#define NOKIA6100FB_SPI_BUF_LEN	(((NOKIA6100FB_X_RS * NOKIA6100FB_Y_RS * 12) \
				 / 8) * 2)

/*
 * Size (in bytes) of a sequence of the LCD commands
 * required to fill in a rectangle
 */
#define NOKIA6100FB_SPI_CMD_LEN	(7 * 2)

/*
 * struct nokia6100fb_par - Device-specific data for Nokia6100
 * @cntrl	LCD controller type (EPSON or PHILIPS)
 * @mode	Mode of operation (ON_DEMAND or TRUE_FB)
 * @freq	Specifies how often the sync-up thread runs (times in second)
 * @volctr	LCD brigthness (0-63)
 * @reset	GPIO to control LCD reset (0-31)
 * @lock:	Exclusive access to critical code
 * @spi:	SPI slave device representing the LCD 
 * @task:	Kernel thread that syncs up the framebuffer with
 * 		the LCD h/w when @mode is TRUE_FB
 * @m:		SPI message corresponding to an LCD rectangle update
 * @x0:		SPI transfer corresponding to an LCD
 * 		rectangle update (LCD commands part)
 * @x1:		SPI transfer corresponding to an LCD
 * 		rectangle update (rectangle pixels part)
 * @buffer:	Main software framebuffer. This is what user-space
 * 		applications get access to via mmap().
 * @shadow:	Previous copy (shadow) of the software framebuffer
 * @spi_buf:	SPI LCD "hardware memory buffer" in the format ready to
 * 		be transferred over SPI.
 */
struct nokia6100fb_par {
	int 			cntrl;
	int 			mode;
	int			freq;
	int 			volctr;
	int 			reset;
	spinlock_t		lock;
	struct spi_device	*spi;
	struct task_struct	*task;
	struct spi_transfer	x0;
	struct spi_transfer	x1;
	struct spi_message	m;
	unsigned short		*buffer;
	unsigned short		shadow[NOKIA6100FB_BUF_LEN];
	unsigned char		spi_buf[NOKIA6100FB_SPI_BUF_LEN];
	unsigned char		spi_cmd[NOKIA6100FB_SPI_CMD_LEN];
};

/*
 * Busy-loop delay for a specified number of jiffies
 * @param n		Jiffies to delay for
 */
static void nokia6100fb_delay(int n)
{
	int end = get_jiffies_64() + n;
	int now;

	do
	{
		now = get_jiffies_64();
		cpu_relax();
	} while(now < end);
}

/*
 * Transmit a frame (9 bits) to the LCD over SPI.
 * @param s		SPI slave device
 * @param h		2-bytes array represeting the frame
 */
static void inline nokia6100fb_spi_frame(
	struct spi_device *s, unsigned char *h)
{
	struct spi_message m;
	struct spi_transfer x;

	spi_message_init(&m);

	memset(&x, 0, sizeof(x));
	x.tx_buf = h;
	x.rx_buf = NULL;
	x.len = 2;
	spi_message_add_tail(&x, &m);

	spi_sync(s, &m);
}

/*
 * Transmit a command to the LCD
 * @param s		SPI slave device
 * @param b		Command byte
 */
static void inline nokia6100fb_spi_command(
	struct spi_device *s, unsigned char b)
{
	unsigned char h[2];

	h[0] = 0x00;
	h[1] = b;
	nokia6100fb_spi_frame(s, h);
}

/*
 * Transmit a data byte to the LCD
 * @param s		SPI slave device
 * @param b		Data byte
 */
static void inline nokia6100fb_spi_data(
	struct spi_device *s, unsigned char b)
{
	unsigned char h[2];

	h[0] = 0x01;
	h[1] = b;
	nokia6100fb_spi_frame(s, h);
}

/*
 * Command codes for the Epson LCD controller
 */
#define DISON			0xaf
#define DISOFF			0xae
#define DISNOR			0xa6
#define DISINV			0xa7
#define COMSCN			0xbb
#define DISCTL			0xca
#define SLPIN			0x95
#define SLPOUT			0x94
#define PASET			0x75
#define CASET			0x15
#define DATCTL			0xbc
#define RGBSET8			0xce
#define RAMWR			0x5c
#define RAMRD			0x5d
#define PTLIN			0xa8
#define PTLOUT			0xa9
#define RMWIN			0xe0
#define RMWOUT			0xee
#define ASCSET			0xaa
#define SCSTART			0xab
#define OSCON			0xd1
#define OSCOFF			0xd2
#define PWRCTR			0x20
#define VOLCTR			0x81
#define VOLUP			0xd6
#define VOLDOWN			0xd7
#define TMPGRD			0x82
#define EPCTIN			0xcd
#define EPCOUT			0xcc
#define EPMWR			0xfc
#define EPMRD			0xfd
#define EPSRRD1			0x7c
#define EPSRRD2			0x7d
#define NOP			0x25

/*
 * One-time preparation of the SPI "LCD hardware memory buffer".
 * Fills in those bytes that are the same for each transfer.
 * @param p		LCD device data structure
 */
static void nokia6100fb_spi_buf_prepare(struct nokia6100fb_par *p)
{
	int i;

	/*
	 * Mark up the data bytes of the SPI buffers as such
	 */
	for (i = 0; i < NOKIA6100FB_SPI_BUF_LEN; i++) {
		if (i % 2 == 0) {
			p->spi_buf[i] = 0x1;
		}
	}

	/*
 	 * Prepare the SPI message. Some fields will be updated at run time.
 	 */
	spi_message_init(&p->m);

	/*
	 * The first transfer defines the sequence (commands / data) to
	 * fill in a rectangle. Coordinates are filled in at run time
	 */
	p->spi_cmd[0] = 0x00;
	p->spi_cmd[1] = PASET;
	p->spi_cmd[2] = 0x01; 
	p->spi_cmd[4] = 0x1;
	p->spi_cmd[6] = 0x00; 
	p->spi_cmd[7] = CASET;
	p->spi_cmd[8] = 0x01; 
	p->spi_cmd[10] = 0x1;
	p->spi_cmd[12] = 0x00; 
	p->spi_cmd[13] = RAMWR;
	memset(&p->x0, 0, sizeof(p->x0));
	p->x0.tx_buf = p->spi_cmd;
	p->x0.rx_buf = NULL;
	p->x0.len = sizeof(p->spi_cmd);
	spi_message_add_tail(&p->x0, &p->m);

	/*
	 * The second transfer is the sequence that defines data (pixels)
	 * for the rectangle. Actual data (pixels) are filled in at
	 * run time.
	 */
	memset(&p->x1, 0, sizeof(p->x1));
	p->x1.tx_buf = p->spi_buf;
	p->x1.rx_buf = NULL;
	spi_message_add_tail(&p->x1, &p->m);
}

/*
 * Write a rectangle onto the LCD display.
 * This is called when the corresponding rectangle has been
 * updated in the LCD software buffer but hasn't been yet 
 * written onto the LCD hardware display.
 * @param p		LCD device data structure
 * @param x1		x1 coordinate
 * @param y1		y1 coordinate
 * @param x2		x2 coordinate
 * @param y2		y2 coordinate
 */
static void nokia6100fb_spi_buf_write(
	struct nokia6100fb_par *p, int x1, int y1, int x2, int y2)
{
	unsigned char b;
	unsigned short c;
	unsigned char *pp;
	int i, j, t;

	/*
 	 * Fill in the pixel memory in the SPI transfer buffer
 	 */
	for (t = 0, pp = p->spi_buf, i = x1; i <= x2; i++) {
		for (j = y1; j <= y2; j++) {
			c = p->buffer[j * NOKIA6100FB_X_RS + i];
			if (t == 0) {
				pp[1] = c >> 4;
				b = pp[3];
				pp[3] = ((c & 0xF) << 4) | (b & 0xF);
				t = 1;
			}
			else {
				pp[5] = c & 0xFF;
				b = pp[3];
				pp[3] = (c >> 8) | (b & 0xF0);
				t = 0;
				pp += 6;
			}
		}
	}
	if (t) {
		pp += 6;
	}

	/*
 	 * Tell the LCD that the specific rectangle will be updated
 	 * Fill in the coordinates & data sequence length.
 	 */
	p->spi_cmd[3] = x1; p->spi_cmd[5] = x2;
	p->spi_cmd[9] = y1; p->spi_cmd[11] = y2;
	p->x1.len = pp - p->spi_buf;

	/*
	 * Send the whole message over SPI thus updating
	 * the rectangle in the LCD hardware.
 	 */
	spi_sync(p->spi, &p->m);

	d_printk(3, "x1,y1=%d,%d x2,y2=%d,%d s=%d\n",
		 x1, y1, x2, y2, pp - p->spi_buf);
}

/* 
 * SmartFusion IOMUX and IOCFG hardware interfaces
 */
#define MSS_IOMUX_BASE		0xE0042100
struct mss_iomux {
	unsigned int		cr[83];
};
#define MSS_IOMUX		((struct mss_iomux *)(MSS_IOMUX_BASE))

#define MSS_GPIO_BASE		0x40013000
struct mss_gpio {
	unsigned int		cfg[32];
	unsigned int 		irq;
	unsigned int 		in;
	unsigned int 		out;
	
};
#define MSS_GPIO		((struct mss_gpio *)(MSS_GPIO_BASE))

/*
 * Do the one-time initialization of the Nokia LCD hardware. Specifically:
 * Set up platform specific GPIO for control of the Nokia LCD.
 * @param p		LCD data structure
 */
static void nokia6100fb_init(struct nokia6100fb_par *p)
{
	/*
 	 * Configure this GPIO at IOMUX
 	 */
	int r = p->reset;
	int b = r < 16 ? 25 : 0;
	int d = r < 16 ?  r : r - 16;
	writel(0x14, &MSS_IOMUX->cr[b + d]);

	/*
	 * Configure and enable this GPIO as output
	 */
	writel(0x5, &MSS_GPIO->cfg[r]);

	d_printk(2, "r=%d\n", r);
}

/*
 * Reset Nokia LCD to a known state
 * @param p		LCD data structure
 */
static void nokia6100fb_reset(struct nokia6100fb_par *p)
{
	struct spi_device *s = p->spi;
	int r = p->reset;

	/*
	 * Perform the LCD reset by strobing the RESET pin
	 */
	writel(readl(&MSS_GPIO->out) & ~(1<<r), &MSS_GPIO->out);
	nokia6100fb_delay(10);
	writel(readl(&MSS_GPIO->out)| (1<<r), &MSS_GPIO->out);
	nokia6100fb_delay(10);

	/*
 	 * Controller-specific initialization sequence
 	 */
	nokia6100fb_spi_command(s, DISCTL);
	nokia6100fb_spi_data(s, 0x00);
	nokia6100fb_spi_data(s, 0x20);
	nokia6100fb_spi_data(s, 0x00);

	nokia6100fb_spi_command(s, COMSCN);
	nokia6100fb_spi_data(s, 0x01);

	nokia6100fb_spi_command(s, OSCON);

	nokia6100fb_spi_command(s, SLPOUT);

	nokia6100fb_spi_command(s, PWRCTR);
	nokia6100fb_spi_data(s, 0x0F);

	nokia6100fb_spi_command(s, DATCTL);
	nokia6100fb_spi_data(s, 0x00);
	nokia6100fb_spi_data(s, 0x00);
	nokia6100fb_spi_data(s, 0x02);

	nokia6100fb_spi_command(s, VOLCTR);
	nokia6100fb_spi_data(s, p->volctr);
	nokia6100fb_spi_data(s, 3);

	nokia6100fb_spi_command(s, DISON);
	nokia6100fb_delay(10);

	d_printk(2, "ok\n");
}

/*
 * Sync up the hardware LCD with the main software framebuffer.
 * @param p		LCD data structure
 * @returns		0->success, <0->error code
 */
static int inline nokia6100fb_sync(struct nokia6100fb_par *p)
{
	int i, j;
	int x1, x2, y1, y2;
	unsigned short *b, *s;
	unsigned long f;
	int ret = 0;

	/* 
	 * In exclusive manner
	 */
	spin_lock_irqsave(&p->lock, f);

	/*
 	 * First compare the main software buffer with the shadow
 	 * and determine geometry of changes, if any. 
 	 */
	x1 = NOKIA6100FB_X_RS;
	y1 = NOKIA6100FB_Y_RS;
	x2 = 0;
	y2 = 0;
	b = p->buffer;
	s = p->shadow;

	for (i = 0; i < NOKIA6100FB_Y_RS; i++) {
		for (j = 0; j < NOKIA6100FB_X_RS; j++) {
			if (*b != *s) {
				if (i < y1) y1 = i;
				if (j < x1) x1 = j;
				if (i > y2) y2 = i;
				if (j > x2) x2 = j;
				*s = *b;
			}
			b++; s++;
		}
	}

	/*
	 * If there were changes, update the affected
	 * part (rectangle) in the hardware LCD
	 */
	if (x2 != 0) {
		nokia6100fb_spi_buf_write(p, x1, y1, x2, y2);
	}

	/*
	 * Release the lock
	 */
	spin_unlock_irqrestore(&p->lock, f);

	return ret;
}

/*
 * LCD driver kernel thread.
 * @param n		LCD-specific data structure
 * @returns		0->success, <0->error code
 */
static int nokia6100fb_task(void *n)
{
	struct nokia6100fb_par *p = (struct nokia6100fb_par *)n;
	int ret = 0;

	/*
 	 * Increase priority of this thread. It is important
 	 * that it does its job quickly.
 	 */
	set_user_nice(current, 5);

	/*
 	 * In an endless loop, until the driver is done
 	 * Re-fresh the LCD, on an as-needed basis.
 	 */
	while (!kthread_should_stop()) {

		/* 
		 * Sync up the LCD hardware with the software buffer
		 */
		nokia6100fb_sync(p);

		/*
 		 * Wait for a while.
 		 */
		schedule_timeout_interruptible(HZ / p->freq);
	}

	d_printk(2, "ret=%d\n", ret);
	return ret;
}

/*
 * Issue an explicit command to sync up the framebuffer with
 * the LCD hardware. From the standards perspective,
 * a more appopriate interface would be fsync(). The framebuffer
 * framework does allow a framebuffer driver to define a fsync
 * callback, however it is targetted towards running deferred
 * I/O jobs and MMU-based implementations of trackable lists
 * of affected framebuffer pages. This could be made work with
 * the low-end LCDs too, however the overhead is a bit too much
 * for low-end no-MMU microprocessors these low-end LCDs are
 * typically used with. 
 *
 * Hence, an ioctl().
 *
 * With ioctl, we could use the existing FBIOPAN_DISPLAY,
 * which makes little sense for the low-end LCDs, 
 * but eventually calls a framebuffer-specific pan_display hook,
 * if defined. However, this ioctl has the overhead of 
 * copying a rather large data structure from the user space
 * to the kernel and back, on each call. 
 *
 * So, we just use the rude-force approach below, which
 * is to define a new ioctl command to sync up with the LCD hardware. 
 * @param i		LCD-specific data structure
 * @param cmd		ioctl command
 * @param arg		ioctl argument
 * @returns		0->success, <0->error code
 */
static int nokia6100fb_ioctl(
	struct fb_info *i, unsigned int cmd, unsigned long arg)
{
	struct nokia6100fb_par *p = i->par;
	int ret = 0;

	switch (cmd) {
	case FBIO_SYNC:
		ret = nokia6100fb_sync(p);
		break;
	default:
		ret = -ENOTTY;
		break;
	}	

	d_printk(2, "ret=%d\n", ret);
	return ret;
}

/*
 * Framebuffer ops data structure.
 * Everything is actually done by the generic code,
 * with the exception of the ioctl call to sync up
 * with the LCD hardware. 
 */
static struct fb_ops nokia6100fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= fb_sys_write,
	.fb_ioctl	= nokia6100fb_ioctl,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
};

/*
 * Framebuffer fix data structure
 */
static struct fb_fix_screeninfo nokia6100fb_fix __devinitdata = {
	.id		= "nokia6100fb",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};

/*
 * Framebuffer var data structure
 */
static struct fb_var_screeninfo nokia6100fb_var __devinitdata = {
	.xres		= NOKIA6100FB_X_RS,
	.yres		= NOKIA6100FB_Y_RS,
	.xres_virtual	= NOKIA6100FB_X_RS,
	.yres_virtual	= NOKIA6100FB_Y_RS,
	.height		= -1,
	.width		= -1,
	.vmode		= FB_VMODE_NONINTERLACED,
	.bits_per_pixel	= 16,
	.red		= { 8, 4, 0 },
	.green		= { 4, 4, 0 },
	.blue		= { 0, 4, 0 },
	.nonstd		= 0,
};

/*
 * Framebuffer device instance probe & initialization
 * @param s		SPI slave
 * @returns		0->success, <0->error code
 */
static int __devinit nokia6100fb_probe(struct spi_device *s)
{
	int j;
	struct fb_info *i;
	struct nokia6100fb_par *p;
	struct nokia6100fb_plat *f = s->dev.platform_data;
	char *n = nokia6100fb_fix.id;
	int ret = 0;

	/*
	 * At this time, the driver supports only the EPSON controller.
	 * If the controller is Philips, bail out with a message
	 */
	if (f->cntrl == NOKIA6100FB_PHILIPS) {
		ret = -EIO;
		dev_err(&s->dev,
		       "nokia6100fb: controller type (%d) not supported\n", 
		       f->cntrl);
		goto Done_release_nothing;
	}

	/*
 	 * Set the frame size for this slave and notify the SPI
 	 * framework that it will be different from the default 8 bits.
 	 */
	s->bits_per_word = 9;
	spi_setup(s);

	/*
 	 * Allocate a framebuffer info structure for this LCD
 	 */
	i = framebuffer_alloc(sizeof(struct nokia6100fb_par), &s->dev);
	if (!i) {
		ret = -ENOMEM;
		dev_err(&s->dev, "allocation of framebuffer info failed"
			" for %s\n", n);
		goto Done_release_nothing;
	}

	/*
 	 * Store the device data structure handle in the SPI slave data
 	 */
	dev_set_drvdata(&s->dev, i);

	/*
 	 * Get access to the device-specific data structure (which
 	 * is a part of the framebuffer data structure).
 	 */
	p = i->par;

	/*
 	 * Allocate the main software buffer for the LCD.
 	 * This has to be page-aligned so we have to allocate
 	 * is separately (unlike the shadow, which we allocate 
 	 * as a part of the device data strcuture).
 	 */
	p->buffer = alloc_pages_exact(sizeof(p->shadow), GFP_DMA | __GFP_ZERO);
	if (!p->buffer) {
		ret = -ENOMEM;
		dev_err(&s->dev, "allocation of buffer failed for %s\n", n);
		goto Done_release_framebuffer;
	}
	
	/*
	 * Fill the main buffer and the shadow with some improbable
	 * colors.
	 */
	for (j = 0; j < NOKIA6100FB_BUF_LEN; j++) {
		p->buffer[j] = 
		p->shadow[j] = 0xF123;
	}

	/*
 	 * Fill in the framebuffer data structure
 	 */
	i->fbops = &nokia6100fb_ops;
	i->var = nokia6100fb_var;
	i->fix = nokia6100fb_fix;
	i->screen_base = (char *) p->buffer;
	i->screen_size = sizeof(p->shadow);
	i->flags = FBINFO_DEFAULT;
	i->fix.smem_start = virt_to_phys(p->buffer);
	i->fix.smem_len	= sizeof(p->shadow);
	i->fix.line_length = i->fix.smem_len / NOKIA6100FB_X_RS;

	/*
 	 * Remember the SPI slave handle
 	 */
	p->spi = s;

	/*
 	 * Remember the platform configuration
 	 */
	p->cntrl = f->cntrl;
	p->mode = f->mode;
	p->freq = f->freq;
	p->volctr = f->volctr;
	p->reset = f->reset;
	
	/*
	 * Do the hardware initialization and reset of the Nokia LCD
	 */	
	nokia6100fb_init(p);
	nokia6100fb_reset(p);

	/*
 	 * Do the one-time preparation of the SPI transferbuffer
 	 */
	nokia6100fb_spi_buf_prepare(p);

	/*
 	 * Register the framebuffer driver
 	 */
	ret = register_framebuffer(i);
	if (ret < 0) {
		dev_err(&s->dev, "failed to register framebuffer for %s\n", n);
		goto Done_release_buffer;
	}

	/*
 	 * Start the LCD driver kernel thread (only for TRUE_FB)
 	 */
	if (p->mode == NOKIA6100FB_TRUE_FB) {
		p->task = kthread_run(nokia6100fb_task, p, "nokia6100fb");
		if (IS_ERR(p->task)) {
			ret = -EIO;
			dev_err(&s->dev, "failed to create the kernel thread"
				" for %s\n", n);
			goto Done_release_registration;
		}
	}

	/*
 	 * Tell the world we have arrived
 	 */
	dev_info(&s->dev, "fb%d: %s (%s) framebuffer\n", i->node, n,
		 f->cntrl == NOKIA6100FB_EPSON ? "Epson" : "Philips");

	/*
 	 * If here, we have been successful
 	 */
	goto Done_release_nothing;

Done_release_registration:
	unregister_framebuffer(i);
Done_release_buffer:
	free_pages_exact(p->buffer, sizeof(p->shadow));
Done_release_framebuffer:
	framebuffer_release(i);
Done_release_nothing:
	d_printk(1, "name=%s,bits=%d,ret=%d\n", n, s->bits_per_word, ret);
	return ret;
}

/*
 * Device instance shutdown
 * @param s		SPI slave
 * @returns		0->success, <0->error code
 */
static int __devexit nokia6100fb_remove(struct spi_device *s)
{
	struct nokia6100fb_par *p;
	struct fb_info *i = dev_get_drvdata(&s->dev);
	int ret = 0;

	if (i) {
		p = i->par;
		if (p->task) {
			kthread_stop(p->task);
		}
		unregister_framebuffer(i);
		free_pages_exact(p->buffer, sizeof(p->shadow));
		framebuffer_release(i);
	}

	d_printk(1, "ret=%d\n", ret);
	return ret;
}

/* 
 * SPI slave device driver data structure for the LCD device
 */
static struct spi_driver nokia6100fb_driver = {
	.driver 	= {
		.name		= "nokia6100fb",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe		= nokia6100fb_probe,
	.remove		= __devexit_p(nokia6100fb_remove),
};

/*
 * Module start-up
 * @returns		0->success, <0->error code
 */
static int __init nokia6100fb_init_module(void)
{
	int ret;

	/*
	 * Register the SPI slave device driver with the SPI framework
	 */
	ret = spi_register_driver(&nokia6100fb_driver);

	d_printk(1, "ret=%d\n", ret);
	return ret;
}

/*
 * Module shut-down
 */
static void __exit nokia6100fb_cleanup_module(void)
{

	/*
	 * Unregister SPI slave device driver from the SPI framework
	 */
	spi_unregister_driver(&nokia6100fb_driver);

	d_printk(1, "%s\n", "clean-up successful");
}

/*
 * Module registration data
 */
module_init(nokia6100fb_init_module);
module_exit(nokia6100fb_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vladimir Khusainov, vlad@emcraft.com");
MODULE_DESCRIPTION("LCD Nokia6100 framebuffer");

