/*
 *  linux/arch/arm/mach-versatile/versatile_ab.c
 *
 *  Copyright (C) 2004 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
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

#include <linux/config.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/amba/bus.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>

#include "core.h"

MACHINE_START(VERSATILE_AB, "ARM-Versatile AB")
	/* Maintainer: ARM Ltd/Deep Blue Solutions Ltd */
	.phys_io	= 0x101f1000,
	.io_pg_offst	= ((0xf11f1000) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= versatile_map_io,
	.init_irq	= versatile_init_irq,
	.timer		= &versatile_timer,
	.init_machine	= versatile_init,
MACHINE_END

#ifdef CONFIG_ARM_AMBA_DMA
#include "dma.h"
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/amba/pl080.h>
extern setting settings[];
/*
 * Configure the board && the dma controller channel for the requesting peripheral
 * as far as possible when the actual transfer
 * (source address, size, dest address, etc.) is not known
 *
 * Versatile AB :
 *
 *	DMA	Peripheral
 *
 *	 15 	UART0 Tx
 *	 14 	UART0 Rx
 *	 13 	UART1 Tx
 *	 12 	UART1 Rx
 *	 11 	UART2 Tx
 *	 10 	UART2 Rx
 *	 9 	SSP Tx
 *	 8 	SSP Rx
 *	 7 	SCI Tx
 *	 6 	SCI Rx
 *
 *	5-3 	Unused
 *
 *	FPGA peripherals using DMA channels 0,1,2
 *
 *	 2	MMCI
 *	 1	AACI	Tx
 *	 0	AACI	Rx
 *
 *	Return 1 for success
 *
 */
const int versatileab_dma_configure(dmach_t chan_num, dma_t * chan_data, struct amba_device * client){
	int retval = 0;

	/* Switch by client AMBA device part number*/
	switch(amba_part(client)) {
	case AMBA_PERIPHID_AACI_97:
		/*
		 * Only DMA channel 1 can be used for AACI DMA TX
		 */
		switch(chan_num){
		case 1:
			pl080_configure_chan(chan_num, &client->dma_data);
			retval = 1;
			break;
		default:
			break;
		}
		break;

	default:
		printk(KERN_ERR "mach-versatile/dma.c::versatile_dma_configure() Unexpected device %p, periphid part number 0x%03x\n", client, amba_part(client));
	}
	return retval;
}

const int versatileab_dma_transfer_setup(dmach_t chan_num, dma_t * chan_data, struct amba_device * client){
	pl080_lli setting;

	unsigned int ccfg = 0;
	dma_addr_t cllis = 0;
	int retval = 0;
	unsigned int periph_index = 0;

	switch(amba_part(client)) {
	case AMBA_PERIPHID_AACI_97:
	{
		// TODO:: RX DMA
		while(amba_part(client) != settings[periph_index].periphid){
			periph_index++;
		}
		ccfg 		= settings[periph_index].cfg;
		/* Destination is the FIFO read/write register */
		setting.dest 	= VERSATILE_AACI_BASE + (unsigned int)client->dma_data.dmac_data;
		setting.cword 	= settings[periph_index].ctl;

		/*
		 * Construct the LLIs
		 */
		// TODO:: determine whether the bus address needs the bus distinguishing bit set
		// - hard code the 1 for now
		setting.next = 1;
		cllis = pl080_make_llis(chan_num, chan_data->buf.dma_address, chan_data->count ,client->dma_data.packet_size, &setting);

		if(cllis) {
			retval = 1;
		} else {
			printk(KERN_ERR "mach-versatile/dma.c::versatile_dma_transfer_setup() No memory for LLIs\n");
		}
	}
		break;

	default:
		printk(KERN_ERR "mach-versatile/dma.c::versatile_dma_transfer_setup() - Unexpected device %p, periphid part number 0x%03x\n", client, amba_part(client));
		return 0;
		break;
	}

	if(retval)
		pl080_transfer_configure(chan_num, &setting, ccfg);

	return retval;
}

static int __init versatile_ab_init(void)
{
	if (machine_is_versatile_ab()) {
		vops.versatile_dma_configure = versatileab_dma_configure ;
		vops.versatile_dma_transfer_setup = versatileab_dma_transfer_setup;
	}
	return 0;
}

arch_initcall(versatile_ab_init);

#endif

