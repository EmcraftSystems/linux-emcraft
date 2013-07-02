/*
 * arch/arm/mach-m2s/include/mach/platform.h
 *
 * Copyright (C) 2009 ARM Ltd.
 *
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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

#define PLATFORM_G4M_VB			0
#define PLATFORM_M2S_SOM		1
#define PLATFORM_SF2_DEV_KIT		2
#define PLATFORM_M2S_FG484_SOM		3

extern int m2s_platform_get(void);

#define DEVICE_M2S_120		0
#define DEVICE_M2S_050		1

extern int m2s_device_get(void);

#endif /* __ASM_ARCH_PLATFORM_H */
