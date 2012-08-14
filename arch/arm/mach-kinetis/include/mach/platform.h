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

#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

/* Freescale TWR-K70F120M + Freescale TWR-SER */
#define PLATFORM_KINETIS_TWR_K70F120M	0
/* Emcraft K70-SOM + Emcraft SOM-BSB */
#define PLATFORM_KINETIS_K70_SOM	1
/* Emcraft K61-SOM + Emcraft SOM-BSB */
#define PLATFORM_KINETIS_K61_SOM	2

int kinetis_platform_get(void);

/* Freescale Kinetis K70 */
#define DEVICE_PK70FN1M0VMJ		0
/* Freescale Kinetis K61 */
#define DEVICE_PK61FN1M0VMJ		1

int kinetis_device_get(void);

int kinetis_lcdtype_get(void);

/* Freescale TWR-LCD-RGB */
#define LCD_TWR_LCD_RGB			0
/* Embedded Artists EA-LCD-004 */
#define LCD_EA_LCD_004			1
/* Future Electronics TWR-PIM-NL8048BC19-02C */
#define LCD_FUT_TWR_NL8048		2
/* Future Electronics TWR-PIM-41WVGA */
#define LCD_FUT_TWR_PIM_41WVGA		3

#endif /* __ASM_ARCH_PLATFORM_H */
