/*
 * (C) Copyright 2014 Emcraft Systems, <www.emcraft.com>
 * Pavel Boldin <paboldin@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef _MACH_LPC18XX_SPIFI_H_
#define _MACH_LPC18XX_SPIFI_H_

/* SPIFI Control register CTRL */
#define LPC18XX_SPIFI_CTRL_OFFSET		(0x0)

#define LPC18XX_SPIFI_CTRL_TIMEOUT_MSK		(0xFFFF)
#define LPC18XX_SPIFI_CTRL_TIMEOUT_OFF		(0)

#define LPC18XX_SPIFI_CTRL_CSHIGH_MSK		(0xF)
#define LPC18XX_SPIFI_CTRL_CSHIGH_OFF		(16)

#define LPC18XX_SPIFI_CTRL_DPRFTCH_DIS_MSK	(0x1)
#define LPC18XX_SPIFI_CTRL_DPRFTCH_DIS_OFF	(21)
#define LPC18XX_SPIFI_CTRL_DPRFTCH_DIS		(1 << 21)

#define LPC18XX_SPIFI_CTRL_INTEN_MSK		(0x1)
#define LPC18XX_SPIFI_CTRL_INTEN_OFF		(22)

#define LPC18XX_SPIFI_CTRL_MODE3_MSK		(0x1)
#define LPC18XX_SPIFI_CTRL_MODE3_OFF		(23)

#define LPC18XX_SPIFI_CTRL_PRFTCH_DIS_MSK	(0x1)
#define LPC18XX_SPIFI_CTRL_PRFTCH_DIS_OFF	(27)
#define LPC18XX_SPIFI_CTRL_PRFTCH_DIS		(1 << 27)

#define LPC18XX_SPIFI_CTRL_DUAL_MSK		(0x1)
#define LPC18XX_SPIFI_CTRL_DUAL_OFF		(28)

#define LPC18XX_SPIFI_CTRL_RFCLK_MSK		(0x1)
#define LPC18XX_SPIFI_CTRL_RFCLK_OFF		(29)

#define LPC18XX_SPIFI_CTRL_FBCLK_MSK		(0x1)
#define LPC18XX_SPIFI_CTRL_FBCLK_OFF		(30)

#define LPC18XX_SPIFI_CTRL_DMAEN_MSK		(0x1)
#define LPC18XX_SPIFI_CTRL_DMAEN_OFF		(31)

/* SPIFI Command register CMD */
#define LPC18XX_SPIFI_CMD_OFFSET		(0x4)

#define LPC18XX_SPIFI_CMD_DATALEN_MSK		(0x3fff)
#define LPC18XX_SPIFI_CMD_DATALEN_OFF		(0)

#define LPC18XX_SPIFI_CMD_POLL_MSK		(1)
#define LPC18XX_SPIFI_CMD_POLL_OFF		(14)

#define LPC18XX_SPIFI_CMD_DOUT_MSK		(1)
#define LPC18XX_SPIFI_CMD_DOUT_OFF		(15)

#define LPC18XX_SPIFI_CMD_INTLEN_MSK		(0x7)
#define LPC18XX_SPIFI_CMD_INTLEN_OFF		(16)

#define LPC18XX_SPIFI_CMD_FIELDFORM_MSK		(0x3)
#define LPC18XX_SPIFI_CMD_FIELDFORM_OFF		(19)

#define LPC18XX_SPIFI_CMD_FRAMEFORM_MSK		(0x7)
#define LPC18XX_SPIFI_CMD_FRAMEFORM_OFF		(21)

#define LPC18XX_SPIFI_CMD_OPCODE_MSK		(0xFF)
#define LPC18XX_SPIFI_CMD_OPCODE_OFF		(24)

/* SPIFI Address register ADDR */
#define LPC18XX_SPIFI_ADDR_OFFSET		(0x8)

/* SPIFI Intermediate Data register IDATA */
#define LPC18XX_SPIFI_IDATA_OFFSET		(0xC)

/* SPIFI cache limit register CLIMIT */
#define LPC18XX_SPIFI_CLIMIT_OFFSET		(0x10)

/* SPIFI Data register DATA (FIFO end) */
#define LPC18XX_SPIFI_DATA_OFFSET		(0x14)

/* SPIFI Memory mode command register MCMD */
#define LPC18XX_SPIFI_MCMD_OFFSET		(0x18)

#define LPC18XX_SPIFI_MCMD_POLL_MSK		(0x1)
#define LPC18XX_SPIFI_MCMD_POLL_OFF		(14)

#define LPC18XX_SPIFI_MCMD_DOUT_MSK		(0x1)
#define LPC18XX_SPIFI_MCMD_DOUT_OFF		(15)

#define LPC18XX_SPIFI_MCMD_INTLEN_MSK		(0x7)
#define LPC18XX_SPIFI_MCMD_INTLEN_OFF		(16)

#define LPC18XX_SPIFI_MCMD_FIELDFORM_MSK	(0x3)
#define LPC18XX_SPIFI_MCMD_FIELDFORM_OFF	(19)

#define LPC18XX_SPIFI_MCMD_FRAMEFORM_MSK	(0x7)
#define LPC18XX_SPIFI_MCMD_FRAMEFORM_OFF	(21)

#define LPC18XX_SPIFI_MCMD_OPCODE_MSK		(0xFF)
#define LPC18XX_SPIFI_MCMD_OPCODE_OFF		(24)

/* SPIFI Status register STAT */
#define LPC18XX_SPIFI_STAT_OFFSET		(0x1C)

#define LPC18XX_SPIFI_STAT_MCINIT_MSK		(0x1)
#define LPC18XX_SPIFI_STAT_MCINIT_OFF		(0)

#define LPC18XX_SPIFI_STAT_CMD_MSK		(0x1)
#define LPC18XX_SPIFI_STAT_CMD_OFF		(1)

#define LPC18XX_SPIFI_STAT_RESET_MSK		(0x1)
#define LPC18XX_SPIFI_STAT_RESET_OFF		(4)

#define LPC18XX_SPIFI_STAT_INTRQ_MSK		(0x1)
#define LPC18XX_SPIFI_STAT_INTRQ_OFF		(5)

#define SPIFI_RESET (1 << LPC18XX_SPIFI_STAT_RESET_OFF)


/* SPIFI frameforms and fieldforms */
#define SPIFI_FRAMEFORM_OP		(1)
#define SPIFI_FRAMEFORM_OP_3ADDRESS	(4)
#define SPIFI_FRAMEFORM_OP_4ADDRESS	(5)

#define SPIFI_FIELDFORM_ALL_SERIAL	(0)
#define SPIFI_FIELDFORM_DUAL_DATA	(1)

/* SPIFI CMD macros */
#define SPIFI_CMD_DATALEN(x) (x & LPC18XX_SPIFI_CMD_DATALEN_MSK)
#define SPIFI_CMD_POLL(x) ((x & LPC18XX_SPIFI_CMD_POLL_MSK) << LPC18XX_SPIFI_CMD_POLL_OFF)
#define SPIFI_CMD_DOUT(x) ((x & LPC18XX_SPIFI_CMD_DOUT_MSK) << LPC18XX_SPIFI_CMD_DOUT_OFF)
#define SPIFI_CMD_INTLEN(x) ((x & LPC18XX_SPIFI_CMD_INTLEN_MSK) << LPC18XX_SPIFI_CMD_INTLEN_OFF)
#define SPIFI_CMD_FIELDFORM(x) ((x & LPC18XX_SPIFI_CMD_FIELDFORM_MSK) << LPC18XX_SPIFI_CMD_FIELDFORM_OFF)
#define SPIFI_CMD_FRAMEFORM(x) ((x & LPC18XX_SPIFI_CMD_FRAMEFORM_MSK) << LPC18XX_SPIFI_CMD_FRAMEFORM_OFF)
#define SPIFI_CMD_OPCODE(x) ((x & LPC18XX_SPIFI_CMD_OPCODE_MSK) << LPC18XX_SPIFI_CMD_OPCODE_OFF)


/* Register m25p80_spifi device */
void __init lpc18xx_spifi_init(void);

#endif /* _MACH_LPC18XX_SPIFI_H_ */
