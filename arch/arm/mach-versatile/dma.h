/*
 * linux/include/asm-arm/hardware/amba.h
 *
 * Copyright (C) 2006 ARM Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Pass dma symbols to the core.c code
 */

extern int versatile_init_dma_channels(void __iomem *base);

/*
 * Board variant specific functions
 */
typedef struct _variant_ops{
	const int (*versatile_dma_configure	)(dmach_t chan_num, dma_t * chan_data, struct amba_device * client);
	const int (*versatile_dma_transfer_setup)(dmach_t chan_num, dma_t * chan_data, struct amba_device * client);
} variant_ops;

extern variant_ops vops;

#define VA_SYS_BASE (__io(IO_ADDRESS(VERSATILE_SYS_BASE)))


