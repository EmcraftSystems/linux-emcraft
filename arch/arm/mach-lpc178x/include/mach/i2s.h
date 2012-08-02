/*
 * asm-arm/arch-lpc32xx/i2c.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2008 NXP Semiconductors
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

#ifndef  __ASM_ARCH_I2S_H
#define  __ASM_ARCH_I2S_H

#include <mach/hardware.h>

/*
 * I2S registers base address on LPC178x/7x
 */
#define LPC32XX_I2S0_BASE	0x400A8000

/**********************************************************************
* I2S controller register offsets
**********************************************************************/

#define I2S_DAO(x)			(x + 0x00)
#define I2S_DAI(x)			(x + 0x04)
#define I2S_TX_FIFO(x)			(x + 0x08)
#define I2S_RX_FIFO(x)			(x + 0x0C)
#define I2S_STAT(x)			(x + 0x10)
#define I2S_DMA0(x)			(x + 0x14)
#define I2S_DMA1(x)			(x + 0x18)
#define I2S_IRQ(x)			(x + 0x1C)
#define I2S_TX_RATE(x)			(x + 0x20)
#define I2S_RX_RATE(x)			(x + 0x24)

/**********************************************************************
* i2s_daO i2s_dai register definitions
**********************************************************************/
#define I2S_WW8      _SBF(0,0) /* Word width is 8bit*/
#define I2S_WW16     _SBF(0,1) /* Word width is 16bit*/
#define I2S_WW32     _SBF(0,3) /* Word width is 32bit*/
#define I2S_MONO     _BIT(2)   /* Mono */
#define I2S_STOP     _BIT(3)   /* Stop, diables the access to FIFO,
                                  mutes the channel */
#define I2S_RESET    _BIT(4)   /* Reset the channel */
#define I2S_WS_SEL   _BIT(5)   /* Channel Master(0) or slave(1)
                                  mode select*/
#define I2S_WS_HP(s) _SBF(6,s) /* Word select half period - 1 */

#define I2S_MUTE     _BIT(15)  /* Mute the channel,
                                  Transmit channel only */

#define I2S_WW32_HP  0x1f /* Word select half period for 32bit
                             word width */
#define I2S_WW16_HP  0x0f /* Word select half period for 16bit
                             word width */
#define I2S_WW8_HP   0x7  /* Word select half period for 8bit
                             word width */

#define WSMASK_HP	  0X7FC /* Mask for WS half period bits */

/**********************************************************************
* i2s_tx_fifo register definitions
**********************************************************************/
#define I2S_FIFO_TX_WRITE(d)              (d)

/**********************************************************************
* i2s_rx_fifo register definitions
**********************************************************************/
#define I2S_FIFO_RX_WRITE(d)              (d)

/**********************************************************************
* i2s_stat register definitions
**********************************************************************/
#define I2S_IRQ_STAT     _BIT(0)
#define I2S_DMA0_REQ     _BIT(1)
#define I2S_DMA1_REQ     _BIT(2)

#define I2S_RX_STATE_MASK	0x0000ff00
#define I2S_TX_STATE_MASK	0x00ff0000

/**********************************************************************
* i2s_dma0 Configuration register definitions
**********************************************************************/
#define I2S_DMA0_RX_EN     _BIT(0)       /* Enable RX DMA1*/
#define I2S_DMA0_TX_EN     _BIT(1)       /* Enable TX DMA1*/
#define I2S_DMA0_RX_DEPTH(s)  _SBF(8,s)  /* Set the level for DMA1
                                            RX Request */
#define I2S_DMA0_TX_DEPTH(s)  _SBF(16,s) /* Set the level for DMA1
                                            TX Request */

/**********************************************************************
* i2s_dma1 Configuration register definitions
**********************************************************************/
#define I2S_DMA1_RX_EN     _BIT(0)       /* Enable RX DMA1*/
#define I2S_DMA1_TX_EN     _BIT(1)       /* Enable TX DMA1*/
#define I2S_DMA1_RX_DEPTH(s)  _SBF(8,s)	 /* Set the level for DMA1
                                            RX Request */
#define I2S_DMA1_TX_DEPTH(s)  _SBF(16,s) /* Set the level for DMA1
                                            TX Request */

/**********************************************************************
* i2s_irq register definitions
**********************************************************************/
#define I2S_RX_IRQ_EN     _BIT(0)       /* Enable RX IRQ*/
#define I2S_TX_IRQ_EN     _BIT(1)       /* Enable TX IRQ*/
#define I2S_IRQ_RX_DEPTH(s)  _SBF(8,s)  /* valid values ar 0 to 7 */
#define I2S_IRQ_TX_DEPTH(s)  _SBF(16,s) /* valid values ar 0 to 7 */

/**********************************************************************
* define audio rates for i2s_tx_rate/i2s_rx_rate register definitions
**********************************************************************/

#define A96KHZ104MHZ8BIT 0x7ed  // 7, 237
#define A48KHZ104MHZ8BIT 0x3cb  // 3, 203
#define A44KHZ104MHZ8BIT 0x14a  // 1, 74
#define A32KHZ104MHZ8BIT 0x5fe	// 5, 254
#define A22KHZ104MHZ8BIT 0x194  // 1, 148
#define A16KHZ104MHZ8BIT 0x1cb	// 1, 203

#define A96KHZ104MHZ16BIT 0xeed	//  14, 237
#define A48KHZ104MHZ16BIT 0x7ED //  7, 237
#define A44KHZ104MHZ16BIT 0x6dd //  6, 221
#define A32KHZ104MHZ16BIT 0x5fe //  5, 254
#define A22KHZ104MHZ16BIT 0x14a //  1, 74
#define A16KHZ104MHZ16BIT 0x2cb //  2, 203

#define A96KHZ104MHZ32BIT 0x1ced// 28, 237
#define A48KHZ104MHZ32BIT 0xeed // 14, 237
#define A44KHZ104MHZ32BIT 0xdf0 // 13, 240
#define A32KHZ104MHZ32BIT 0x57f	// 5, 127
#define A22KHZ104MHZ32BIT 0x125 // 1, 37
#define A16KHZ104MHZ32BIT 0x5fe // 5, 254

/**********************************************************************
* i2s_tx_rate register definitions
**********************************************************************/
#define I2S_SET_TX_RATE(d)          (d)

/**********************************************************************
* i2s_rx_rate register definitions
**********************************************************************/
#define I2S_SET_RX_RATE(d)          (d)

/**********************************************************************
* i2s channel select
**********************************************************************/
#define I2S_CH0	0
#define I2S_CH1	1

#endif /*  __ASM_ARCH_I2S_H */
