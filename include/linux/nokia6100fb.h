/*
 * nokia6100fb.h - interface to the framebuffer driver for the Nokia6100 LCD.
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

#ifndef _LINUX_NOKIA6100FB_H
#define _LINUX_NOKIA6100FB_H 1

#include <linux/ioctl.h>

#ifdef __KERNEL__

/* 
 * LCD controller ID.
 * Sadly, there appears to be no programming interface to figure
 * out which controller is used on a module at run time. Moreover,
 * it is hard to find out which controller is used in a specific
 * module either (i.e. respective vendors do not disclose this info).
 * So it might take a bit of guessing to determine the right 
 * controller for a specific platform.
 */
#define NOKIA6100FB_EPSON	0
#define NOKIA6100FB_PHILIPS	1

/*
 * Mode of operation
 * - TRUE_FB - this is a "true" framebuffer. User applications update
 * the software framebuffer via mmap, write, etc and the driver's kernel
 * task syncs up the content of the software buffer with the hardware LCD
 * in background. The disadvantage of this mode is that the kernel sync-up
 * task may preempt an application updating the LCD framebuffer at any
 * point, redrawing just a part of an area being updated; sometimes it
 * may be noticable, especially, if there is lots of other activity in
 * the system. Plus, the driver's sync-up task keeps running in the background
 * requiring some CPU bandwidth.
 * - ON_DEMAND - User applications write to the software buffer via mmap,
 * write, etc, however the driver doesn't sync up the framebuffer with
 * the hardware LCD until explicitly asked to do so. The drawback of this
 * mode is that user applications must explicity call a command (ioctl)
 * when they need a sync-up of the framebuffer with the LCD hardware.
 */
#define NOKIA6100FB_TRUE_FB	0
#define NOKIA6100FB_ON_DEMAND	1

/*
 * struct nokia6100fb_plat - Nokia6100 LCD platform data
 * @cntrl:           		LCD controller ID (Epson or Philips)
 * @mode:           		Mode of operation (ON_DEMAND or TRUE_FB)
 * @freq:           		When in the TRUE_FB mode, @freq specifies
 * 				the number of times the driver syncs up
 * 				the content of the software framebuffer
 * 				w/ the LCD hardware. 
 * @volctr:			Brightness of the LCD (0-63). Apparently,
 * 				an appropriate value depends on
 * 				a particular LCD unit. Reasonable values
 * 				appear to be in a range from 30 to 50.
 * @reset:           		GPIO to control LCD reset (0-31, corresponding
 * 				to MSS_GPIO[0-31]).
 */
struct nokia6100fb_plat {
	int cntrl;
	int mode;
	int freq;
	int volctr;	
	int reset;
};

#endif /* __KERNEL__ */

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
 *
 * Refer to include/linux/fb.h for why the magic value below and
 * how the NR number was chosen
 */
#define FBIO_MAGIC		0x46
#define FBIO_SYNC		_IO(FBIO_MAGIC, 0x32)

#endif

