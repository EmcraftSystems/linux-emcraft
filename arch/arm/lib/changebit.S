/*
 *  linux/arch/arm/lib/changebit.S
 *
 *  Copyright (C) 1995-1996 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <asm/unified.h>
	
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "bitops.h"
                .text

/* Purpose  : Function to change a bit
 * Prototype: int change_bit(int bit, void *addr)
 */
	.type	_change_bit_be, %function
ENTRY(_change_bit_be)
		eor	r0, r0, #0x18		@ big endian byte ordering
	.type	_change_bit_le, %function
ENTRY(_change_bit_le)
	bitop	eor
