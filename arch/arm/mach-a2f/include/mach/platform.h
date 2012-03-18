/*
 * arch/arm/mach-a2f/include/mach/platform.h
 *
 * Copyright (C) 2009 ARM Ltd.
 * 2011, Vladimir Khusainov, Emcraft Systems, vlad@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_PLATFORM_H
#define __ASM_ARCH_PLATFORM_H

#define PLATFORM_A2F_LNX_EVB		0
#define PLATFORM_A2F_ACTEL_DEV_BRD	1
#define PLATFORM_A2F_HOERMANN_BRD	2
#define PLATFORM_A2F200_SOM		3
#define PLATFORM_A2F500_SOM		4

extern int a2f_platform_get(void);

#define DEVICE_A2F_060			0
#define DEVICE_A2F_200			1
#define DEVICE_A2F_500			2

extern int a2f_device_get(void);

#endif	/* __ASM_ARCH_PLATFORM_H */
