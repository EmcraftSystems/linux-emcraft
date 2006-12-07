/*
 *  linux/include/asm-arm/arch-versatile/hardware.h
 *
 *  This file contains the hardware definitions of the Versatile boards.
 *
 *  Copyright (C) 2003 ARM Limited.
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <asm/arch/platform.h>

/*
 * PCI space virtual addresses
 */
#define VERSATILE_PCI_VIRT_BASE		0xe8000000
#define VERSATILE_PCI_CFG_VIRT_BASE	0xe9000000

#if 0
#define VERSATILE_PCI_VIRT_MEM_BASE0	0xf4000000
#define VERSATILE_PCI_VIRT_MEM_BASE1	0xf5000000
#define VERSATILE_PCI_VIRT_MEM_BASE2	0xf6000000

#define PCIO_BASE			VERSATILE_PCI_VIRT_MEM_BASE0
#define PCIMEM_BASE			VERSATILE_PCI_VIRT_MEM_BASE1
#endif

/* CIK guesswork */
#define PCIBIOS_MIN_IO			0x44000000
#define PCIBIOS_MIN_MEM			0x50000000

#define pcibios_assign_all_busses()     1

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x)		(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)

#ifdef CONFIG_ARM_AMBA_DMA
/* DMA defines for AACI/DMAC transfers */
// Packet is the data transferred during the run of one LLI
// Memory boundary size means the byte size of the data packet transferred by one LLI
// Burst sizes are in bytes
// Transfer size is destination burst sizes in a source burst == run of one LLI
/* Register settings - see PrimeCell DMA Controller (PL080) TRM && AACI PL041 TRM */
#define	DMABlockSize	0x40 	// 64 transfers @ 32 bit width == 256(0x100) bytes
#define	DWidth		2 	// 32 bits  0/1/2
#define	SWidth		2	// 32 bits  0/1/2
#define	DBSize		3 	// 16 transfers @ 16 bit width ==  32(0x20) bytes AACI FIFO width /0 == 1/1 == 4/2 == 8/3 == 16/4 == 32/5 == 64/6 == 128/7 == 256/
#define	SBSize		5 	// Set to half of the AACI 512 byte FIFO depth == 64 transfers @ 32 bit width == 256(0x100) bytes Memory
				// - later check with FIFO status whether we could use full depth
/*
 * TODO::
 * - some from DMAC == same for all device DMA
 * - some AACI device specific
 */

#define VDMA_AACI_CCTL 																   \
	((1 << 31) | 		/* ALL  'I'    - Terminal count interrupt enable 								*/ \
	 (0 << 28) | 		/* ALL  'Prot' - AHB HPROT information to destination slave. (Set to Not Cacheable, Not bufferable, User mode). */ \
	 (0 << 27) | 		/* AACI 'DI'   - Destination address NOT incremented after each transfer 					*/ \
	 (1 << 26) | 		/* AACI 'SI'   - Source address incremented after each transfer 						*/ \
	 (0 << 25) | 		/* AACI 'D'    - Destination AHB master select; 0=AHB1, 1=AHB2. (Transferring to AACI on V/PB M2 bus) 		*/ \
	 (1 << 24) | 		/* AACI 'S'    - Source AHB master select; 0=AHB1, 1=AHB2. (Transferring from memory on V/PB DMA1 bus) 		*/ \
	 (DWidth << 21) | 	/* AACI 'DWidth' - Destination transfer width. 									*/ \
	 (SWidth << 18) | 	/* ALL  'SWidth' - Source transfer width. 									*/ \
	 (DBSize << 15) | 	/* AACI 'DBSize' - Destination burst size. 									*/ \
	 			/*		   The number of bytes transferred when the peripheral requests a burst of data. 		*/ \
	                        /*                 AACI requests a burst when the FIFO has four or less words.  				*/ \
	 (SBSize << 12) | 	/* AACI 'SBSize' - Source burst size. (Memory boundary size when transferring from memory). (Set to 256 bytes) 	*/ \
	 DMABlockSize) 		/* AACI Number of 'destination width' transfers in one DMA 'packet'. (2^12)-1 is the maximum. 			*/

#define VDMA_AACI_CCFG 																   \
			/* [31:19]          ALL   'Reserved'       - Write as zero. 								*/ \
	((0 << 18) | 	/* 'Halt'           ALL    - 0 = Allow DMA requests, 1 = Finish current request and ignore others 			*/ \
	 (0 << 17) |	/* [17]        	    ALL    'Active'         - Read only bit. 0 = No data in FIFO. 					*/ \
	 (0 << 16) | 	/* 'L'              ALL  - Lock. Generates locked tranfers on the AHB. 							*/ \
	 (1 << 15) | 	/* 'ITC'            ALL  - Terminal Count Interrupt mask. Masks TC interrupt when cleared. 				*/ \
	 (1 << 14) | 	/* 'IE'             ALL  - Interrupt error mask. Masks error interrupt when cleared. 					*/ \
	 (1 << 11) | 	/* 'FlowCntrl'      AACI - Flow control method. We're using Mem -> Peripheral, with the DMAC as the flow controller. 	*/ \
	 (0 << 10) | 	/* 'Reserved'       ALL  - Write as zero 										*/ \
	 (0 << 6)  | 	/* 'DestPeripheral' AACI - Destination peripheral number to associate with this channel. 				*/ \
	 		/*			   Set zero here, OR in the correct value in the machine initialisation				*/ \
	                /*                         VPB uses #0, which is shared with a logic tile DMA line on the V/PB. 			*/ \
			/*                         AACI Must be selected in V/PB SYS_DMAPSR0 reg. 						*/ \
			/*                         VAB uses #1 AACI TX which is fixed to DMA channel 1						*/ \
	 (0 << 5)  | 	/* 'Reserved'       ALL  - Write as zero 										*/ \
	 (0 << 1)  | 	/* 'SrcPeripheral'  AACI - Source peripheral number to associate with this channel. Ignored for 'from memory' transfers.*/ \
	 (0 << 0))   	/* 'E'              ALL  - Enables this DMA channel. (Channels automatically disabled when a transfer completes) 	*/
#endif /* CONFIG_ARM_AMBA_DMA */

#define VDMA_CCFG_PERIPHAL_NUM_0	0x00000000
#define VDMA_CCFG_PERIPHAL_NUM_1	0x00000040
#define VDMA_CCFG_PERIPHAL_NUM_2	0x00000080
#define VDMA_CCFG_PERIPHAL_NUM_3	0x000000C0
#define VDMA_CCFG_PERIPHAL_NUM_4	0x00000100
#define VDMA_CCFG_PERIPHAL_NUM_5	0x00000140
#define VDMA_CCFG_PERIPHAL_NUM_6	0x00000180
#define VDMA_CCFG_PERIPHAL_NUM_7	0x000001C0
#define VDMA_CCFG_PERIPHAL_NUM_8	0x00000200
#define VDMA_CCFG_PERIPHAL_NUM_9	0x00000240
#define VDMA_CCFG_PERIPHAL_NUM_A	0x00000280
#define VDMA_CCFG_PERIPHAL_NUM_B	0x000002C0
#define VDMA_CCFG_PERIPHAL_NUM_C	0x00000300
#define VDMA_CCFG_PERIPHAL_NUM_D	0x00000340
#define VDMA_CCFG_PERIPHAL_NUM_E	0x00000380
#define VDMA_CCFG_PERIPHAL_NUM_F	0x000003C0


#endif
