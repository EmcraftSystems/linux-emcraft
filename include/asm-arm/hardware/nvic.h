/*
 *  linux/include/asm-arm/hardware/nvic.h
 *
 *  Copyright (C) 2008 ARM Limited, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARM_HARDWARE_NVIC_H
#define __ASM_ARM_HARDWARE_NVIC_H

#include <linux/compiler.h>

#define V7M_SCS				0xe000e000
#define NVIC_INTR_CTRL			(V7M_SCS + 0x004)
#define NVIC_SYSTICK_CTRL		(V7M_SCS + 0x010)
#define NVIC_SYSTICK_RELOAD		(V7M_SCS + 0x014)
#define NVIC_SYSTICK_CURRENT		(V7M_SCS + 0x018)
#define NVIC_SYSTICK_CALIBRATION	(V7M_SCS + 0x01c)
#define NVIC_SET_ENABLE			(V7M_SCS + 0x100)
#define NVIC_CLEAR_ENABLE		(V7M_SCS + 0x180)
#define NVIC_SET_PENDING		(V7M_SCS + 0x200)
#define NVIC_CLEAR_PENDING		(V7M_SCS + 0x280)
#define NVIC_ACTIVE_BIT			(V7M_SCS + 0x300)
#define NVIC_PRIORITY			(V7M_SCS + 0x400)
#define NVIC_INTR_CTRL_STATE		(V7M_SCS + 0xd04)
#define NVIC_SOFTWARE_INTR		(V7M_SCS + 0xf00)

#ifndef __ASSEMBLY__
void nvic_init(void);
#endif

#endif
