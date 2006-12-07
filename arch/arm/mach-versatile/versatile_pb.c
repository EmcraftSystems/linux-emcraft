/*
 *  linux/arch/arm/mach-versatile/versatile_pb.c
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
#include <asm/mach/mmc.h>

#include "core.h"

#if 1
#define IRQ_MMCI1A	IRQ_VICSOURCE23
#else
#define IRQ_MMCI1A	IRQ_SIC_MMCI1A
#endif

static struct mmc_platform_data mmc1_plat_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.status		= mmc_status,
};

#define UART3_IRQ	{ IRQ_SIC_UART3, NO_IRQ }
#define UART3_DMA	{ 0x86, 0x87 }
#define SCI1_IRQ	{ IRQ_SIC_SCI3, NO_IRQ }
#define SCI1_DMA	{ 0x88, 0x89 }
#define MMCI1_IRQ	{ IRQ_MMCI1A, IRQ_SIC_MMCI1B }
#define MMCI1_DMA	{ 0x85, 0 }

/*
 * These devices are connected via the core APB bridge
 */
#define GPIO2_IRQ	{ IRQ_GPIOINT2, NO_IRQ }
#define GPIO2_DMA	{ 0, 0 }
#define GPIO3_IRQ	{ IRQ_GPIOINT3, NO_IRQ }
#define GPIO3_DMA	{ 0, 0 }

/*
 * These devices are connected via the DMA APB bridge
 */

/* FPGA Primecells */
AMBA_DEVICE(uart3, "fpga:09", UART3,    NULL);
AMBA_DEVICE(sci1,  "fpga:0a", SCI1,     NULL);
AMBA_DEVICE(mmc1,  "fpga:0b", MMCI1,    &mmc1_plat_data);

/* DevChip Primecells */
AMBA_DEVICE(gpio2, "dev:e6",  GPIO2,    NULL);
AMBA_DEVICE(gpio3, "dev:e7",  GPIO3,    NULL);

static struct amba_device *amba_devs[] __initdata = {
	&uart3_device,
	&gpio2_device,
	&gpio3_device,
	&sci1_device,
	&mmc1_device,
};

#ifdef CONFIG_ARM_AMBA_DMA
#include "dma.h"
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/amba/pl080.h>
extern setting  settings[];
/*
 * Configure the board && the dma controller channel for the requesting peripheral
 * as far as possible when the actual transfer
 * (source address, size, dest address, etc.) is not known
 *
 * Versatile PB :
 *
 *	Other possible assignments:
 *	DMA	Peripheral
 *
 *	 15 	UART0 Tx
 *	 14 	UART0 Rx
 *	 13 	UART1 Tx
 *	 12 	UART1 Rx
 *	 11 	UART2 Tx
 *	 10 	UART2 Rx
 *	  9 	SSP Tx
 *	  8 	SSP Rx
 *	  7 	SCI Tx
 *	  6 	SCI Rx
 *
 *	5-3 	I/O device in RealView Logic Tile
 *
 *	2-0 	I/O device in RealView Logic Tile or FPGA
 *
 *	FPGA peripherals using DMA channels 0,1,2
 *
 *	AACI	Tx
 *	AACI	Rx
 *	USB	A
 *	USB	B
 *	MCI	0
 *	MCI	1
 *	UART3	Tx
 *	UART3	Rx
 *	SCI0	int A
 *	SCI0	int B
 *
 *	Return 1 for success
 *
 */
const int versatilepb_dma_configure(dmach_t chan_num, dma_t * chan_data, struct amba_device * client){
	int retval = 0;

	/*
	 *  Versatile DMA mapping register for assigned DMA channel
	 */
	void __iomem **map_base = __io_address(VERSATILE_SYS_BASE) + VERSATILE_SYS_DMAPSR0_OFFSET + (chan_num * 4);

	struct amba_dma_data * data = &client->dma_data;

	/* Switch by client AMBA device part number*/
	switch(amba_part(client)) {
	case AMBA_PERIPHID_AACI_97:
		/*
		 * Only DMA channels 0,1,2 can be used for AACI DMA
		 */
		switch(chan_num){
		case 0:
		case 1:
		case 2:
			/*
			 * Set V/PB DMA mapping register to connect
			 * AACI Tx DMAC request signals to DMAC peripheral #0 request lines
			 *
			 * ASSUMES Tx
			 * TODO:: Distinguish Tx/Rx
			 */
			writel(
				// [31:8] 	Reserved
				(1 << 7) |	// 1 = Enable this mapping
				// [6:5] 	Reserved
				(0 << 0)	// 0b00000 = AACI Tx
				, &map_base[chan_num]);

			pl080_configure_chan(chan_num, data);
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

const int versatilepb_dma_transfer_setup(dmach_t chan_num, dma_t * chan_data, struct amba_device * client){
	pl080_lli setting;

	unsigned int ccfg = 0;
	dma_addr_t cllis = 0;
	int retval = 0;
	unsigned int periph_index = 0;

	switch(amba_part(client)) {
	case AMBA_PERIPHID_AACI_97:
	{
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

#endif


static int __init versatile_pb_init(void)
{
	int i;

	if (machine_is_versatile_pb()) {
		for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
			struct amba_device *d = amba_devs[i];
			amba_device_register(d, &iomem_resource);
		}

#ifdef CONFIG_ARM_AMBA_DMA
		{
		volatile unsigned int r;
		/*
		 * Initial disposition of the DMA select signals
		 * - later a contention mechanism must be implemented to allow
		 *   apps/drivers of the 10 FPGA sources to contend for the 3 lines
		 */
		/* AACI TX is line 0 */
		r  = readl(VA_SYS_BASE + VERSATILE_SYS_DMAPSR0_OFFSET);
		mb();
		r &= VSYSMASK_DMAPSR;
		r |= VSYS_VAL_DMAPSR_AACI_TX;
		r |= VSYS_VAL_DMAPSR_ENABLE;
		writel(r, VA_SYS_BASE + VERSATILE_SYS_DMAPSR0_OFFSET);
		mb();


		vops.versatile_dma_configure      = versatilepb_dma_configure     ;
		vops.versatile_dma_transfer_setup = versatilepb_dma_transfer_setup;
		}
#endif
	}
	return 0;
}

arch_initcall(versatile_pb_init);

MACHINE_START(VERSATILE_PB, "ARM-Versatile PB")
	/* Maintainer: ARM Ltd/Deep Blue Solutions Ltd */
	.phys_io	= 0x101f1000,
	.io_pg_offst	= ((0xf11f1000) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= versatile_map_io,
	.init_irq	= versatile_init_irq,
	.timer		= &versatile_timer,
	.init_machine	= versatile_init,
MACHINE_END
