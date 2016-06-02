/*
 * (C) Copyright 2015
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

#ifndef _MACH_STM32_FB_H_
#define _MACH_STM32_FB_H_

#include <linux/init.h>

/*
 * LTDC regs
 */
#define STM32F4_LTDC_BASE	0x40016800
#define STM32F4_LTDC_LENGTH	0x400

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

void __init stm32f4x9_fb_init(void);

struct stm32f4_fb_platform_data {
	const char *mode_str;
	int (*init) (int);

	struct fb_videomode *modes;
	unsigned int modes_size;
};

static inline int stm32f4_fb_is_running(void)
{
	/* Check LTDC_GCR[LTDCEN] */
	return *(volatile u32 *)(STM32F4_LTDC_BASE + 0x18) & 1;
}

#endif /* _MACH_STM32_FB_H_ */
