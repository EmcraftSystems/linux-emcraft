/*
 * linux/arch/arm/mach-versatile/dma.c
 *
 * Copyright (C) 2006 ARM Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Board specific dma code.
 * This board knows
 * - it has an AMBA bus with
 * AMBA pl080 bus DMA controller
 * as device dev:30
 *
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>

#include <linux/amba/bus.h>
#include <linux/amba/pl080.h>
#include <asm/arch/hardware.h>
#include <asm/mach-types.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "core.h"
#include "dma.h"

/* The pl080 ops to be called by the versatile */
static struct dma_ops dmac_ops;

variant_ops vops;

/*
 * Entries must be in the same order as the amba_devs array
 * - only aaci is implemented at present
 * TODO:: Some parts of the values should be constructed
 * from the parts which might vary.....
 */
setting settings[] = {
{0x80,0,0},	 /* dmac_device */
{0,0},		 /* uart0_devic */
{0,0},		 /* uart1_devic */
{0,0},		 /* uart2_devic */
{0,0},		 /* smc_device, */
{0,0},		 /* mpmc_device */
{0,0},		 /* clcd_device */
{0,0},		 /* sctl_device */
{0,0},		 /* wdog_device */
{0,0},		 /* gpio0_devic */
{0,0},		 /* gpio1_devic */
{0,0},		 /* rtc_device, */
{0,0},		 /* sci0_device */
{0,0},		 /* ssp0_device */
{0x41,VDMA_AACI_CCTL, VDMA_AACI_CCFG}, /* aaci_device */
{0,0},		 /* mmc0_device */
{0,0},		 /* kmi0_device */
{0,0}		 /* kmi1_device */
};

/*
 * Return 0 for success
 */
static int versatile_request(dmach_t chan_num, dma_t * chan_data)
{
	int status = -EINVAL;
	struct amba_device * client;

	/*
	 * Find the driver data for the named device
	 */
	client = amba_get_device_with_name((char*)chan_data->device_id);

	/* Configure the DMA mechanism e.g. the AMBA DMA controller */
	if(!vops.versatile_dma_configure(chan_num, chan_data, client))
		status =	-EINVAL;
	else {
		/*
		 * Check pl080 is happy before doing anything complicated
		 */
		if(!dmac_ops.request(chan_num, chan_data)) {

			if(!vops.versatile_dma_transfer_setup(chan_num, chan_data, client))
				status = -EINVAL;
			else
				status = 0;
		}
	}
	return status;
}

static void versatile_free(dmach_t chan_num, dma_t * chan_data)
{
	struct amba_device * client;

	/*
	 * Find the driver data for the named device
	 */
	client = amba_get_device_with_name((char*)chan_data->device_id);

	/*
	 *Deconfigure the DMA mechanism e.g. the AMBA DMA controller
	 * and any machine settings e.g. DMA mapping register
	 */
	switch(amba_part(client)) {
	case AMBA_PERIPHID_AACI_97:
		switch(chan_num){
		case 0:
		case 1:
		case 2:
		{
#ifdef CONFIG_ARCH_VERSATILE_PB
			if(machine_is_versatile_pb()){
				void __iomem **map_base = __io_address(VERSATILE_SYS_BASE) + VERSATILE_SYS_DMAPSR0_OFFSET + (chan_num * 4);
				/*
				 * Clear down the V/PB DMA mapping register connection
				 */
				writel(
		 			// [31:8] 	Reserved
		 			(0 << 7) |	// 0 = Disable this mapping
		 			// [6:5] 	Reserved
		 			(0 << 0)	// 0b00000 = AACI Tx
					, &map_base[chan_num]);
			}
#endif

		}
			break;
		default:
	 			printk(KERN_ERR "mach-versatile/dma.c::versatile_free() Invalid DMA channel 0x%02x for AACI \n", chan_num);
			break;
		}
		break;

	default:
	 		printk(KERN_ERR "mach-versatile/dma.c::versatile_free() Unexpected device %p, periphid part number 0x%03x - AACI is 0x%03x\n", client, amba_part(client), AMBA_PERIPHID_AACI_97);
	 		break;
	}

	dmac_ops.free(chan_num, chan_data);
}

void	versatile_enable(dmach_t chan_num, dma_t * chan_data)
{
	pl080_reset_cycle(chan_num);
	dmac_ops.enable(chan_num, chan_data);
}

void 	versatile_disable(dmach_t chan_num, dma_t * chan_data)
{
	dmac_ops.disable(chan_num, chan_data);
}

/* ASSUME returns bytes */
int	versatile_residue(dmach_t chan_num, dma_t * chan_data)
{
	int res = 0;
	return res;
}

int	versatile_setspeed(dmach_t chan_num, dma_t * chan_data, int speed)
{
	int new_speed = 0;
	return new_speed;
}


/*
 * Board ops to be called by AMBA functions
 */
static struct dma_ops versatile_dma_ops = {
	versatile_request,	/* optional */
	versatile_free,		/* optional */
	versatile_enable, 	/* mandatory */
	versatile_disable,	/* mandatory */
	versatile_residue,	/* optional */
	versatile_setspeed,	/* optional */
	"VERSATILE DMA"
};


void __init arch_dma_init(dma_t *dma)
{
#ifdef CONFIG_ARM_AMBA_DMA
	/* Adjust the DMA channel configuration values for the peripherals */
	if(machine_is_versatile_ab()){
		/*
		 * AACI is peripheral 1 on the VAB, 0 on the VPB
		 * VDMA_AACI_CCFG is VPB (peripheral 0) value
		 */
		settings[0x0E].cfg |= VDMA_CCFG_PERIPHAL_NUM_1;
	}

	amba_init_dma(dma);

	pl080_init_dma(dma, &dmac_ops);

	amba_set_ops(&versatile_dma_ops);

#endif
}



