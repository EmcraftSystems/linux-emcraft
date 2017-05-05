/*
 * Freescale USB-FS OTG Host Controller Driver (Kirin Host Controller).
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

/*****************************************************************************
 * Include files:
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/list.h>

#include "khci-hcd.h"

/*****************************************************************************
 * Exported API:
 *****************************************************************************/
/*
 * Dump reg values, and non-empty queues
 */
void khci_dbg_dump_reg(struct khci_hcd *khci)
{
	volatile struct khci_reg	*reg = khci->reg;
	struct list_head		*lst[] = { &khci->ctrl_lst,
						   &khci->intr_lst,
						   &khci->bulk_lst };
	char				*lst_name[] = {"CTRL", "INTR", "BULK"};
	struct khci_td			*td;
	struct khci_urb			*urb;
	struct khci_ep			*ep;
	int				i;

	printk("istat:%02x,inten:%02x,errstat:%02x,erren:%02x,stat:%02x,\n",
		reg->istat, reg->inten, reg->errstat, reg->erren, reg->stat);
	printk("ctl:%02x,addr:%02x,bdtpage:%02x%02x%02x00,frmnum:%02x%02x\n",
		reg->ctl, reg->addr, reg->bdtpage3, reg->bdtpage2,
		reg->bdtpage1, reg->frmnumh, reg->frmnuml);
	printk("token:%02x,softhld:%02x,endpt:%02x,usbctrl:%02x\n",
		reg->token, reg->softhld, reg->ep[0].endpt, reg->usbctrl);
#if defined(CONFIG_ARCH_KINETIS)
	printk("icache:%04x(%04x),dcache:%04x(%04x).mpu:%04x\n",
		*(volatile u32 *)0xe0082000, *(volatile u32 *)0xe0082020,
		*(volatile u32 *)0xe0082800, *(volatile u32 *)0xe0082820,
		*(volatile u32 *)0x4000D000);
#endif
	printk("stat: setup%d,in%d,out%d; errs %d,%d,%d,%d,%d\n",
		khci->stat.setup, khci->stat.in, khci->stat.out,
		khci->stat.own, khci->stat.nak, khci->stat.bus,
		khci->stat.err, khci->stat.len);
	printk("BDs: %08x.%08x %08x.%08x %08x.%08x %08x.%08x\n",
		khci->bdt[0].flg, khci->bdt[0].adr,
		khci->bdt[1].flg, khci->bdt[1].adr,
		khci->bdt[2].flg, khci->bdt[2].adr,
		khci->bdt[3].flg, khci->bdt[3].adr);

	if (khci->td) {
		td = khci->td;
		printk("Curr TD: %d/%d %s %d/%d %08x.%08x %02x.%02x.%02x.%02x "
		    "%d.%d.%d.%d.%d %s\n",
		    td->tries, td->status, td->tx ? "TX" : "RX",
		    KHCI_BD_BC_GET(td->bd_flg), KHCI_BD_BC_GET(td->org_flg),
		    td->bd_flg, td->bd_adr,
		    td->token, td->err, td->stat, td->istat,
		    td->retry.own, td->retry.nak, td->retry.bus,
		    td->retry.err, td->retry.len,
		    khci_dbg_flg2tock(td->bd_flg));

		if (KHCI_BD_BC_GET(td->bd_flg)) {
			khci_dbg_dump_buf((u8 *)td->bd_adr,
					 KHCI_BD_BC_GET(td->bd_flg));
		}
	}

	list_for_each_entry(td, &khci->td_done_lst, node) {
		printk("Done TD: %d/%d %s %d/%d %08x.%08x %02x.%02x.%02x.%02x "
		    "%d.%d.%d.%d.%d %s\n",
		    td->tries, td->status, td->tx ? "TX" : "RX",
		    KHCI_BD_BC_GET(td->bd_flg), KHCI_BD_BC_GET(td->org_flg),
		    td->bd_flg, td->bd_adr,
		    td->token, td->err, td->stat, td->istat,
		    td->retry.own, td->retry.nak, td->retry.bus,
		    td->retry.err, td->retry.len,
		    khci_dbg_flg2tock(td->bd_flg));

		if (KHCI_BD_BC_GET(td->bd_flg)) {
			khci_dbg_dump_buf((u8 *)td->bd_adr,
					 KHCI_BD_BC_GET(td->bd_flg));
		}
	}

	for (i = 0; i < ARRAY_SIZE(lst); i++) {
		list_for_each_entry(ep, lst[i], node) {
			printk("%s.EP.%d.%p.%p: %d\n", lst_name[i],
				ep->type, ep, ep->hep, ep->state);
			list_for_each_entry(urb, &ep->urb_lst, node) {
				printk(" URB.%p.%p td:%d/%d len:%d/%d "
					"st:%d %d,%d,%d,%d "
					"ss:%d/%d\n",
					urb, urb->urb,
					urb->td_done, urb->td_todo,
					urb->urb->actual_length,
					urb->urb->transfer_buffer_length,
					urb->stat.ok, urb->stat.own, urb->stat.nak,
					urb->stat.bus, urb->stat.err,
					urb->state, urb->status);
			}
		}
	}
}

/*
 * Dump buffer
 */
void khci_dbg_dump_buf(u8 *p, u32 len)
{
	int	i;

	if (!len)
		goto out;

	for (i = 0; i < len; i++) {
		if (!i || !(i % 32))
			printk("KHCI %08x: ", (u32)p + i);
		printk("%02x", p[i]);
		if (!((i + 1) % 8))
			printk(".");
		if (!((i + 1) % 32) && i + 1 < len)
			printk("\n");
	}

	printk("\n");
out:
	return;
}

/*
 * Convert BD flag TOK value to ASCII description
 */
char *khci_dbg_flg2tock(u32 bd_flg)
{
	static struct {
		u8	code;
		char	*str;
	} tock_dsc[] = {
		{ KHCI_BD_TOK_DATA0, "DATA0" }, { KHCI_BD_TOK_DATA1, "DATA1" },
		{ KHCI_BD_TOK_ACK, "ACK" }, { KHCI_BD_TOK_STALL, "STALL" },
		{ KHCI_BD_TOK_NAK, "NAK" },
		{ KHCI_BD_TOK_BUS_TOUT, "BUS TIMEOUT" },
		{ KHCI_BD_TOK_DATA_ERR, "DATA ERROR" },
	};

	int	i;
	char	*str = "Unknown";
	u8	tock = (bd_flg >> 2) & 0xF;

	for (i = 0; i < sizeof(tock_dsc) / sizeof(tock_dsc[0]); i++) {
		if (tock_dsc[i].code == tock) {
			str = tock_dsc[i].str;
			break;
		}
	}

	return str;
}
