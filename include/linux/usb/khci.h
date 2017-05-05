/*
 * Freescale USB-FS Host Controller (Kirin Host Controller) Driver header
 *
 * Copyright 2013 EmCraft Systems www.emcraft.com
 * Author: Yuri Tikhonov <yur@emcraft.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _KHCI_H_
#define _KHCI_H_

/*****************************************************************************
 * Constants and macros:
 *****************************************************************************/

#undef DEBUG
#undef dbg

/*
 * Debug level here
 */
#define DEBUG			1
#if defined(DEBUG)
# define dbg(lvl, fmt...)	if (lvl <= DEBUG) printk("KHCI " fmt)
#else
# define dbg(lvl, fmt...)
#endif

/*
 * Buffer descriptor flags
 */
#define KHCI_BD_OWN		(1 << 7)
#define KHCI_BD_DATA_OFS	6
#define KHCI_BD_DATA_MSK	0x1
#define KHCI_BD_BC_OFS		16
#define KHCI_BD_BC_MSK		0x3FF

#define KHCI_BD_TOK_DATA0	0x3
#define KHCI_BD_TOK_DATA1	0xB
#define KHCI_BD_TOK_ACK		0x2
#define KHCI_BD_TOK_STALL	0xE
#define KHCI_BD_TOK_NAK		0xA
#define KHCI_BD_TOK_BUS_TOUT	0x0
#define KHCI_BD_TOK_DATA_ERR	0xF

#define KHCI_BD_DATA_SET(x)	(((x) & KHCI_BD_DATA_MSK) << KHCI_BD_DATA_OFS)
#define KHCI_BD_BC_SET(x)	(((x) & KHCI_BD_BC_MSK) << KHCI_BD_BC_OFS)

#define KHCI_BD_DATA_GET(x)	(((x) & KHCI_BD_DATA_MSK) >> KHCI_BD_DATA_OFS)
#define KHCI_BD_BC_GET(x)	(((x) >> KHCI_BD_BC_OFS) & KHCI_BD_BC_MSK)
#define KHCI_BD_TOK_GET(x)	(((x) >> 2) & 0xF)
#define KHCI_BD_FLG_SET(ln,dt)	(KHCI_BD_DATA_SET(dt) |			       \
				 KHCI_BD_BC_SET(ln) |			       \
				 KHCI_BD_OWN)

#define KHCI_BD_PTR(udc, ep, tx, odd)	(struct khci_bd *)		       \
	((u32)(udc)->bdt | (((ep) & 0x0f) << 5) | (((tx) & 1) << 4) |	\
	 (((odd) & 1) << 3))
#define KHCI_BD_BUF(udc, ep, tx, odd) \
	(udc->dma_bufs + 64 * (4 * ep + ((tx) & 1) * 2 + odd))

/*
 * Register fields
 */
#define KHCI_INT_STALL		(1 << 7)
#define KHCI_INT_ATTACH		(1 << 6)
#define KHCI_INT_RESUME		(1 << 5)
#define KHCI_INT_SLEEP		(1 << 4)
#define KHCI_INT_TOKDNE		(1 << 3)
#define KHCI_INT_SOFTOK		(1 << 2)
#define KHCI_INT_ERROR		(1 << 1)
#define KHCI_INT_RST		(1 << 0)

#define KHCI_STAT_EP(x)		((x) >> 4)
#define KHCI_STAT_TX(x)		((x) & (1 << 3) ? 1 : 0)
#define KHCI_STAT_ODD(x)	((x) & (1 << 2) ? 1 : 0)

#define KHCI_ERR_BTSERR		(1 << 7)
#define KHCI_ERR_DMAERR		(1 << 5)
#define KHCI_ERR_BTOERR		(1 << 4)
#define KHCI_ERR_DFN8		(1 << 3)
#define KHCI_ERR_CRC16		(1 << 2)
#define KHCI_ERR_CRC5EOF	(1 << 1)
#define KHCI_ERR_PIDERR		(1 << 0)

/*
 * Don't process the following (like in MQX driver):
 * - BTOERR: happen if e.g. connect mouse
 * - CRC5EOF: these shouldn't happen with our SOFTHLD scheduling
 */
#define KHCI_ERR_MSK		(KHCI_ERR_BTSERR | KHCI_ERR_DMAERR |	       \
				 KHCI_ERR_DFN8 | KHCI_ERR_CRC16 |	       \
				 KHCI_ERR_PIDERR)

#define KHCI_CTL_JSTATE		(1 << 7)
#define KHCI_CTL_SE0		(1 << 6)
#define KHCI_CTL_TOKENBUSY	(1 << 5)
#define KHCI_CTL_RESET		(1 << 4)
#define KHCI_CTL_HOSTMODEEN	(1 << 3)
#define KHCI_CTL_ODDRST		(1 << 1)
#define KHCI_CTL_USBENSOFEN	(1 << 0)

#define KHCI_ADDR_LSEN		(1 << 7)

#define KHCI_TOKEN_OUT		(0x1 << 4)
#define KHCI_TOKEN_IN		(0x9 << 4)
#define KHCI_TOKEN_SETUP	(0xD << 4)
#define KHCI_TOKEN_MSK		(0xF << 4)

#define KHCI_EP_HOSTWOHUB	(1 << 7)
#define KHCI_EP_RETRYDIS	(1 << 6)
#define KHCI_EP_EPCTLDIS	(1 << 4)
#define KHCI_EP_EPRXEN		(1 << 3)
#define KHCI_EP_EPTXEN		(1 << 2)
#define KHCI_EP_STALL		(1 << 1)
#define KHCI_EP_EPHSHK		(1 << 0)

#define KHCI_USBCTRL_SUSP	(1 << 7)
#define KHCI_USBCTRL_PDE	(1 << 6)

#define KHCI_OTGCTL_OTGEN	(1 << 2)
#define KHCI_OTGCTL_DMLOW	(1 << 4)
#define KHCI_OTGCTL_DPLOW	(1 << 5)
#define KHCI_OTGCTL_DPHIGH	(1 << 7)

/*****************************************************************************
 * C-types:
 *****************************************************************************/

/*
 * Register map
 */
struct khci_reg {
	u8	perid;		/* Peripheral ID Register */
	u8			rsv0[3];
	u8	idcomp;		/* Peripheral ID Complement Register */
	u8			rsv1[3];
	u8	rev;		/* Peripheral Revision Register */
	u8			rsv2[3];
	u8	addinfo;	/* Peripheral Additional Info Register */
	u8			rsv3[3];
	u8	otgistat;	/* OTG Interrupt Status Register */
	u8			rsv4[3];
	u8	otgicr;		/* OTG Interrupt Control Register */
	u8			rsv5[3];
	u8	otgstat;	/* OTG Status Register */
	u8			rsv6[3];
	u8	otgctl;		/* OTG Control Register */
	u8			rsv7[99];
	u8	istat;		/* Interrupt Status Register */
	u8			rsv8[3];
	u8	inten;		/* Interrupt Enable Register */
	u8			rsv9[3];
	u8	errstat;	/* Error Interrupt Status Register */
	u8			rsv10[3];
	u8	erren;		/* Error Interrupt Enable Register */
	u8			rsv11[3];
	u8	stat;		/* Status Register */
	u8			rsv12[3];
	u8	ctl;		/* Control Register */
	u8			rsv13[3];
	u8	addr;		/* Address Register */
	u8			rsv14[3];
	u8	bdtpage1;	/* BDT Page Register 1 */
	u8			rsv15[3];
	u8	frmnuml;	/* Frame Number Register Low */
	u8			rsv16[3];
	u8	frmnumh;	/* Frame Number Register High */
	u8			rsv17[3];
	u8	token;		/* Token Register */
	u8			rsv18[3];
	u8	softhld;	/* SOF Threshold Register */
	u8			rsv19[3];
	u8	bdtpage2;	/* BDT Page Register 2 */
	u8			rsv20[3];
	u8	bdtpage3;	/* BDT Page Register 3 */
	u8			rsv21[11];
	struct {
		u8	endpt;	/* Endpoint Control Register */
		u8		rsv0[3];
	} ep[16];
	u8	usbctrl;	/* USB Control Register */
	u8			rsv22[3];
	u8	observe;	/* USB OTG Observe Register */
	u8			rsv23[3];
	u8	control;	/* USB OTG Control Register */
	u8			rsv24[3];
	u8	usbtrc0;	/* USB Transceiver Control Register 0 */
	u8			rsv25[7];
	u8	usbfrmadjust;	/* Frame Adjust Register */
} __attribute__((packed));

/*
 * Buffer descriptor
 */
struct khci_bd {
	u32	flg;		/* Buffer descriptor flags KHCI_BD_xxx */
	u32	adr;		/* Buffer address */
} __attribute__((packed));

#endif /* _KHCI_H_ */
