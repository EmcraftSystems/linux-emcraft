/*
 * arch/arm/mach-lpc178x/include/mach/dma.h
 *
 * (was named asm-arm/arch-lpc32xx/dmac.h)
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
 *
 * Customized for LPC178x/7x by:
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

#ifndef _MACH_LPC178X_DMAC_H_
#define _MACH_LPC178X_DMAC_H_

#include <mach/lpc178x.h>
#include <mach/hardware.h>

/*
 * DMA registers base
 */
#define LPC178X_DMA_BASE	(LPC178X_AHB_PERIPH_BASE + 0x00000000)
/*
 * "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70)
 */
#define LPC178X_DMA_IRQ		26

/**********************************************************************
* DMA register offsets
**********************************************************************/

/* DMA controller register structures */
#define DMA_INT_STAT(x)			(x + 0x00)
#define DMA_INT_TC_STAT(x)		(x + 0x04)
#define DMA_INT_TC_CLEAR(x)		(x + 0x08)
#define DMA_INT_ERR_STAT(x)		(x + 0x0C)
#define DMA_INT_ERR_CLEAR(x)		(x + 0x10)
#define DMA_RAW_TC_STAT(x)		(x + 0x14)
#define DMA_RAW_ERR_STAT(x)		(x + 0x18)
#define DMA_CH_ENABLE(x)		(x + 0x1C)
#define DMA_SW_BURST_REQ(x)		(x + 0x20)
#define DMA_SW_SINGLE_REQ(x)		(x + 0x24)
#define DMA_SW_LAST_BURST_REQ(x)	(x + 0x28)
#define DMA_SW_LAST_SINGLE_REQ(x)	(x + 0x2C)
#define DMA_CONFIG(x)			(x + 0x30)
#define DMA_SYNC(x)			(x + 0x34)

/* DMA controller channel register structure */
#define DMA_CH_OFFS(c)			((c * 0x20) + 0x100)
#define DMACH_SRC_ADDR(x, c)		(x + DMA_CH_OFFS(c) + 0x00)
#define DMACH_DEST_ADDR(x, c)		(x + DMA_CH_OFFS(c) + 0x04)
#define DMACH_LLI(x, c)			(x + DMA_CH_OFFS(c) + 0x08)
#define DMACH_CONTROL(x, c)		(x + DMA_CH_OFFS(c) + 0x0C)
#define DMACH_CONFIG_CH(x, c)		(x + DMA_CH_OFFS(c) + 0x10)

/* DMA linked list structure */
#define DMA_LL_SRC(x)			(x + 0x0)
#define DMA_LL_DEST(x)			(x + 0x4)
#define DMA_LL_NEXT_LLI(x)		(x + 0x8)
#define DMA_LL_NEXT_CTRL(x)		(x + 0xC)

#define DMA_LL_SIZE 16

/**********************************************************************
* int_stat, int_tc_stat, int_tc_clear, int_err_stat, raw_tc_stat,
* raw_err_stat, and chan_enable register definitions
**********************************************************************/
/* Macro for determining a bit position for a channel */
#define DMAC_GET_CHAN_POS(chan)     (0x1 << ((chan) & 0x7))

/**********************************************************************
* sw_burst_req, sw_single_req, sw_last_burst_req, sw_last_single_req,
* and sync register definitions
**********************************************************************/
/* Peripheral DMA bit position for I2S0 DMA0 */
#define DMA_PER_I2S0_DMA0           _BIT(0)

/* Peripheral DMA bit position for NAND FLASH (same as 12) */
#define DMA_PER_NAND1               _BIT(1)

/* Peripheral DMA bit position for I2S1 DMA0 */
#define DMA_PER_I2S1_DMA0           _BIT(2)

/* Peripheral DMA bit position for SPI2 (RX and TX) */
#define DMA_PER_SPI2_TXRX           _BIT(3)

/* Peripheral DMA bit position for SSP1 (RX) */
#define DMA_PER_SSP1_RX             _BIT(3)

/* Peripheral DMA bit position for SD card */
#define DMA_PER_SDCARD              _BIT(4)

/* Peripheral DMA bit position for HSUART1 TX */
#define DMA_PER_HSUART1_TX          _BIT(5)

/* Peripheral DMA bit position for HSUART1 RX */
#define DMA_PER_HSUART1_RX          _BIT(6)

/* Peripheral DMA bit position for HSUART2 TX */
#define DMA_PER_HSUART2_TX          _BIT(7)

/* Peripheral DMA bit position for HSUART2 RX */
#define DMA_PER_HSUART2_RX          _BIT(8)

/* Peripheral DMA bit position for HSUART7 TX */
#define DMA_PER_HSUART7_TX          _BIT(9)

/* Peripheral DMA bit position for HSUART7 RX */
#define DMA_PER_HSUART7_RX          _BIT(10)

/* Peripheral DMA bit position for I2S1 DMA1 */
#define DMA_PER_I2S1_DMA1           _BIT(10)

/* Peripheral DMA bit position for SPI1 (RX and TX) */
#define DMA_PER_SPI1_TXRX           _BIT(11)

/* Peripheral DMA bit position for SSP1 (TX) */
#define DMA_PER_SSP1_TX             _BIT(11)

/* Peripheral DMA bit position for NAND FLASH (same as 1) */
#define DMA_PER_NAND2               _BIT(12)

/* Peripheral DMA bit position for I2S0 DMA1 */
#define DMA_PER_I2S0_DMA1           _BIT(13)

/* Peripheral DMA bit position for SSP0 (RX) */
#define DMA_PER_SSP0_RX             _BIT(14)

/* Peripheral DMA bit position for SSP0 (TX) */
#define DMA_PER_SSP0_TX             _BIT(15)

/**********************************************************************
* config register definitions
**********************************************************************/
/* Bit for enabling big endian mode on AHB 1 */
#define DMAC_BIG_ENDIAN_AHB1        _BIT(2)

/* Bit for enabling big endian mode on AHB 0 */
#define DMAC_BIG_ENDIAN_AHB0        _BIT(1)

/* Bit for enabling the DMA controller */
#define DMAC_CTRL_ENABLE            _BIT(0)

/**********************************************************************
* lli register definitions
**********************************************************************/
/* Bit for selecting AHB0 (0) or AHB1 (1) */
#define DMAC_CHAN_LLI_SEL_AHB1      _BIT(0)

/**********************************************************************
* control register definitions
**********************************************************************/
/* Bit for enabling a channel terminal count interrupt */
#define DMAC_CHAN_INT_TC_EN         _BIT(31)

/* Bit for indicating address is cacheable */
#define DMAC_CHAN_PROT3             _BIT(30)

/* Bit for indicating address is bufferable */
#define DMAC_CHAN_PROT2             _BIT(29)

/* Bit for indicating address is privelaged mode (1) or user
   mode (0) */
#define DMAC_CHAN_PROT1             _BIT(28)

/* Bit for enabling automatic destination increment */
#define DMAC_CHAN_DEST_AUTOINC      _BIT(27)

/* Bit for enabling automatic source increment */
#define DMAC_CHAN_SRC_AUTOINC       _BIT(26)

/* Destination data width selection defines */
#define DMAC_CHAN_DEST_WIDTH_8      0x0
#define DMAC_CHAN_DEST_WIDTH_16     _BIT(21)
#define DMAC_CHAN_DEST_WIDTH_32     _BIT(22)

/* Source data width selection defines */
#define DMAC_CHAN_SRC_WIDTH_8       0x0
#define DMAC_CHAN_SRC_WIDTH_16      _BIT(18)
#define DMAC_CHAN_SRC_WIDTH_32      _BIT(19)

/* Destination data burst size defines (in transfer width) */
#define DMAC_CHAN_DEST_BURST_1      0
#define DMAC_CHAN_DEST_BURST_4      _BIT(15)
#define DMAC_CHAN_DEST_BURST_8      _BIT(16)
#define DMAC_CHAN_DEST_BURST_16     (_BIT(16) | _BIT(15))
#define DMAC_CHAN_DEST_BURST_32     _BIT(17)
#define DMAC_CHAN_DEST_BURST_64     (_BIT(17) | _BIT(15))
#define DMAC_CHAN_DEST_BURST_128    (_BIT(17) | _BIT(16))
#define DMAC_CHAN_DEST_BURST_256    (_BIT(17) | _BIT(16) | _BIT(15))

/* Macro for direct loading of destination burst size field */
#define DMAC_CHAN_DEST_BURST_LOAD(n) (((n) & 0x7) << 15)

/* Source data burst size defines (in transfer width) */
#define DMAC_CHAN_SRC_BURST_1       0
#define DMAC_CHAN_SRC_BURST_4       _BIT(12)
#define DMAC_CHAN_SRC_BURST_8       _BIT(13)
#define DMAC_CHAN_SRC_BURST_16      (_BIT(13) | _BIT(12))
#define DMAC_CHAN_SRC_BURST_32      _BIT(14)
#define DMAC_CHAN_SRC_BURST_64      (_BIT(14) | _BIT(12))
#define DMAC_CHAN_SRC_BURST_128     (_BIT(14) | _BIT(13))
#define DMAC_CHAN_SRC_BURST_256     (_BIT(14) | _BIT(13) | _BIT(12))

/* Macro for direct loading of source burst size field */
#define DMAC_CHAN_SRC_BURST_LOAD(n) (((n) & 0x7) << 12)

/* Macro for loading transfer size */
#define DMAC_CHAN_TRANSFER_SIZE(n)  ((n) & 0xFFF)

/**********************************************************************
* config_ch register definitions
**********************************************************************/
/* Bit for halting a DMA transfer */
#define DMAC_CHAN_HALT              _BIT(18)

/* Bit for checking active status of the DMA channel */
#define DMAC_CHAN_ACTIVE            _BIT(17)

/* Bit for enabling locked transfers */
#define DMAC_CHAN_LOCK              _BIT(16)

/* Terminal count interrupt mask bit */
#define DMAC_CHAN_ITC               _BIT(15)

/* Interrupt error mask bit */
#define DMAC_CHAN_IE                _BIT(14)

/* Defines for flow control with DMA as the controller */
#define DMAC_CHAN_FLOW_D_M2M        (0x0 << 11)
#define DMAC_CHAN_FLOW_D_M2P        (0x1 << 11)
#define DMAC_CHAN_FLOW_D_P2M        (0x2 << 11)
#define DMAC_CHAN_FLOW_D_SP2DP      (0x3 << 11)

/* Defines for flow control with destination peripheral as the
   controller */
#define DMAC_CHAN_FLOW_DP_SP2DP     (0x4 << 11)

/* Defines for flow control with peripheral as the controller */
#define DMAC_CHAN_FLOW_P_M2P        (0x5 << 11)
#define DMAC_CHAN_FLOW_P_P2M        (0x6 << 11)

/* Defines for flow control with source peripheral as the
   controller */
#define DMAC_CHAN_FLOW_SP_SP2DP     (0x7 << 11)

/* Macro for loading destination peripheral */
#define DMAC_DEST_PERIP(n)          (((n) & 0x1F) << 6)

/* Macro for loading source peripheral */
#define DMAC_SRC_PERIP(n)           (((n) & 0x1F) << 1)

/* Channel enable bit */
#define DMAC_CHAN_ENABLE            _BIT(0)

/**********************************************************************
* config_ch register definitions (source and destination
* peripheral ID numbers). These can be used with the DMAC_DEST_PERIP
* and DMAC_SRC_PERIP macros.
*
* For the correct values for these macros see section
* `34.4.2.3 DMA request connections` of the LPC178x/7x User Manual.
**********************************************************************/
#define DMA_PERID_SDCARD            1
#define DMA_PERID_I2S0_DMA0		6
#define DMA_PERID_I2S0_DMA1		7

#endif /* _MACH_LPC178X_DMAC_H_ */
