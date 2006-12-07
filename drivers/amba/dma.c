/*
 * drivers/amba/dma.c
 *
 * Copyright (C) 2006 ARM Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Wrapper for the AMBA DMA API
 * Calls the board functions which may also call the controller, if any.
 *
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/amba/bus.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/sizes.h>

DEFINE_SPINLOCK(amba_dma_spin_lock);

/* DMA operations of the board */
struct dma_ops * board_ops = NULL;

void amba_set_ops(struct dma_ops * ops){
	board_ops = ops;
}

/*
 * Successful int functions return 0
 * - unless otherwise detailed
 */

int	amba_request(dmach_t chan_num, dma_t * chan_data)
{
	int status = 0;

	if((board_ops) && (board_ops->request)){
		status = board_ops->request(chan_num, chan_data);
	}

	return status;
}

void	amba_free(dmach_t chan_num, dma_t * chan_data)
{
	if((board_ops) && (board_ops->free)){
		board_ops->free(chan_num, chan_data);
	}
}

void	amba_enable(dmach_t chan_num, dma_t * chan_data)
{
	if((board_ops) && (board_ops->enable)){
		board_ops->enable(chan_num, chan_data);
	}
}

void 	amba_disable(dmach_t chan_num, dma_t * chan_data)
{
	if((board_ops) && (board_ops->disable)){
		board_ops->disable(chan_num, chan_data);
	}
}

/* ASSUME returns number of bytes */
int	amba_residue(dmach_t chan_num, dma_t * chan_data)
{
	int res = 0;
	if((board_ops) && (board_ops->residue)){
		res = board_ops->residue(chan_num, chan_data);
	}
	return res;
}

int	amba_setspeed(dmach_t chan_num, dma_t * chan_data, int speed)
{
	int new_speed = 0;
	if((board_ops) && (board_ops->setspeed)){
		new_speed = board_ops->setspeed(chan_num, chan_data, speed);
	}
	return new_speed;
}

/*
 * AMBA ops to be called by kernel DMA functions
 */
static struct dma_ops amba_dma_ops = {
	amba_request,	/* optional */
	amba_free, 	/* optional */
	amba_enable, 	/* mandatory */
	amba_disable,	/* mandatory */
	amba_residue,	/* optional */
	amba_setspeed,	/* optional */
	"AMBA DMA"
};

/*
 * Initialize the dma channels, as far as we can
 */
void __init amba_init_dma(dma_t *channels)
{
	int i;

	for(i = 0; i < MAX_DMA_CHANNELS; i++){
		channels[i].sgcount = 0;
		channels[i].sg = NULL;
		channels[i].active = 0;
		channels[i].invalid = 1;
		// Disappeared channels[i].using_sg = 0;
		channels[i].dma_mode = DMA_NONE;
		channels[i].speed = 0;
		channels[i].lock = 0;
		channels[i].device_id = "UNASSIGNED";
		channels[i].dma_base = (unsigned int)NULL;
		channels[i].dma_irq = IRQ_DMAINT;
		channels[i].state = 0;
		channels[i].d_ops = &amba_dma_ops;
	}
}


