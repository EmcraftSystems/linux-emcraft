/*
 * Copyright (C) 2013. All rights reserved.
 *
 * Author: Axel Utech axel.utech@gmail.com
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __FSL_KHCI_UDC_H
#define __FSL_KHCI_UDC_H

#define USB_MAX_CTRL_PAYLOAD		64

#define USB_NUM_ENDPOINTS 16
#define USB_NUM_ENDPOINTS_BUFFERS (USB_NUM_ENDPOINTS * 4)
#define USB_NUM_PIPES (USB_NUM_ENDPOINTS * 2)

/*
 * Registers for USB0
 * according to: K70P256M150SF3RM.pdf page 1549 ff.
 */

struct endpoint_register{
	union{
		struct{
			unsigned handshake:1; /* enable handshaking */
			unsigned stall:1;
			unsigned txen:1;
			unsigned rxen:1;
			unsigned disablecontrol:1; /* disable control transfer*/
			unsigned:1;
			unsigned retrydis:1;
			unsigned hostwohub:1;
		} __attribute__ ((packed));
		u8 reg_value;
	} __attribute__ ((packed));
} __attribute__ ((packed)) ;

#define INT_USBRST 0
#define INT_ERROR 1
#define INT_SOFTOK 2
#define INT_TOKDNE 3
#define INT_SLEEP 4
#define INT_RESUME 5
#define INT_ATTACH 6
#define INT_STALL 7

struct k70_usb0_regs{
	u8 perid;
	u8 x0[3];
	u8 idcomp;
	u8 x1[3];
	u8 rev;
	u8 x2[3];
	struct{
		unsigned iehost:1;
		unsigned:2;
		unsigned irqnum:5;
	} __attribute__ ((packed)) addinfo;

	u8 x30[3];
	u8 otgistat;
	u8 x3[3];
	u8 otgicr;
	u8 x4[3];
	u8 otgstat;
	u8 x5[3];
	u8 otgctl;
	u8 x6[3+64+32];

	union int_status{
		struct {
			unsigned usbrst:1;
			unsigned error:1;
			unsigned softok:1;
			unsigned tokdne:1;
			unsigned sleep:1;
			unsigned resume:1;
			unsigned attach:1;
			unsigned stall:1;
		} __attribute__ ((packed));
		u8 regvalue;
	} __attribute__ ((packed)) istat;
	u8 x7[3];
	union{
		struct{
			unsigned usbrst:1;
			unsigned error:1;
			unsigned softok:1;
			unsigned tokdne:1;
			unsigned sleep:1;
			unsigned resume:1;
			unsigned attach:1;
			unsigned stall:1;
		} __attribute__ ((packed));
		u8 regvalue;
	} __attribute__ ((packed)) inten;
	u8 x8[3];

	union{
		struct{
			unsigned piderr:1;
			unsigned crc5:1;
			unsigned crc16:1;
			unsigned dfn8:1;
			unsigned btoerr:1;
			unsigned dmaerr:1;
			unsigned resv:1;
			unsigned btserr:1;
		} __attribute__ ((packed));
		u8 regvalue;
	} __attribute__ ((packed)) errstat;
	u8 x9[3];
	union{
		struct{
			unsigned piderr:1;
			unsigned crc5:1;
			unsigned crc16:1;
			unsigned dfn8:1;
			unsigned btoerr:1;
			unsigned dmaerr:1;
			unsigned resv:1;
			unsigned btserr:1;
		} __attribute__ ((packed));
		u8 regvalue;
	} __attribute__ ((packed)) erren;
	u8 x10[3];

	union status_register_union{
		struct status_register{
			unsigned:2;
			unsigned odd:1;
			unsigned tx:1;
			unsigned endp:4;
		} __attribute__ ((packed)) stat;
		u8 regvalue;
	} __attribute__ ((packed)) stat;
	u8 x11[3];
	struct{
		unsigned usben:1;
		unsigned oddrst:1;
		unsigned resume:1;
		unsigned hostmode:1;
		unsigned reseten:1;
		unsigned suspend_busy:1;
		unsigned se0:1;
		unsigned jstate:1;
	} __attribute__ ((packed)) ctl;
	u8 x12[3];
	struct{
		unsigned addr:7;
		unsigned lsen:1;
	} __attribute__ ((packed)) addr;
	u8 x13[3];
	u8 bdtpage1;
	u8 x14[3];

	u8 frmnuml;
	u8 x15[3];
	u8 frmnumh;
	u8 x16[3];

	struct{
		unsigned endpt:4;
		unsigned pid:4;
	} __attribute__ ((packed)) token;
	u8 x17[3];

	u8 softhld;
	u8 x18[3];

	u8 bdtpage2;
	u8 x19[3];
	u8 bdtpage3;
	u8 x20[3+8];

	struct endpoint_register endpoints[16*4];

	struct {
		unsigned:6;
		unsigned pde:1; /* enable weak pulldowns on D+ and D- */
		unsigned susp:1; /* USb transceiver suspend state */
	} __attribute__ ((packed)) usbctrl;
	u8 x21[3];
	struct{
		unsigned:4;
		unsigned dmpd:1; /* D- pulldown state (0 = disable) */
		unsigned:1;
		unsigned dppd:1; /* D+ pulldown state (0 = disable) */
		unsigned dppu:1; /* D+ pullup state (0 = disable) */
	} __attribute__ ((packed)) observe;
	u8 x22[3];
	struct{
		unsigned:4;
		unsigned dppluuup:1; /* D+ pullup enable (non OTG mode) */
		unsigned:3;
	} __attribute__ ((packed)) control;
	u8 x23[3];
	struct{
		unsigned resume_int:1;
		unsigned sync_det:1;
		unsigned:3;
		unsigned async_resume_int_en:1;
		unsigned strange:1;
		unsigned reset:1;
	} __attribute__ ((packed)) usbtrc0;
	u8 x24[7];
	u8 usbfrmadjust;
};

/* Representation for Buffer Descriptor Table */

struct k70_bd_entry{
	union{
		struct{
			unsigned:2;
			unsigned stall:1;
			unsigned dts:1;
			unsigned no_adress_increment:1;
			unsigned keep:1;
			unsigned data0_1:1;
			unsigned own:1;
		} __attribute__ ((packed));

		struct{
			unsigned:2;
			unsigned tok_pid:4;
		} __attribute__ ((packed));
	} __attribute__ ((packed));


    u8 dummy;
    u16 bytecount; 		/* Bit 0..9: Byte Count */
    u32 addr;		/* Buffer Adress */
} __attribute__ ((packed));

#define REQ_PACKET_PENDING 1
#define REQ_DATA_BUFFER 2
#define REQ_LAST_PACKET_BUFFERED 3
#define REQ_ZERO_LENGTH_PACKET 4
#define REQ_READY 5

struct fsl_req {
	struct usb_request req;
	struct list_head queue;
	/* ep_queue() func will add
	   a request->queue into a udc_ep->queue 'd tail */
	struct fsl_ep *ep;

	unsigned stall:1;
	unsigned data0_1:1;
	unsigned send_zero_packet:1;

	unsigned planned;

	int state;
};

struct fsl_buffer_descriptor{
	struct k70_bd_entry *bd_entry;
	struct fsl_buffer_descriptor *alt_buffer;
	struct fsl_req *req;
	unsigned in_use:1;
};

struct fsl_counter{
	u32 buffers_set;
	u32 buffers_freed;

	u32 requests_set;
	u32 requests_complete;

	u32 bytes_transfered;
};

struct fsl_ep {
	struct usb_ep ep;
	struct list_head queue;
	const struct usb_endpoint_descriptor *desc;
	struct usb_gadget *gadget;

	/* for each pipe are two buffers available */
	struct fsl_buffer_descriptor *ep_bd[2];
	/* next buffer for planning */
	struct fsl_buffer_descriptor *next_free_bd;
	/* next buffer to be used by the SIE */
	struct fsl_buffer_descriptor *next_use_bd;

	volatile struct endpoint_register *endpt_reg;

	char name[14];
	unsigned stopped:1;
	unsigned dir:1;

	struct fsl_req *stall_req;
	unsigned stalled:1;

	struct fsl_counter stat;
};

#define EP_DIR_IN	1
#define EP_DIR_OUT	0

/* ep0 transfer state */
#define WAIT_FOR_SETUP          0

#define DATA_IN         1
#define DATA_IN_ZERO_LENGTH     2

#define DATA_OUT     3

#define WAIT_FOR_OUT_STATUS         4
#define WAIT_FOR_IN_STATUS         5

struct fsl_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct completion *done;	/* to make sure release() is done */
	struct fsl_ep *eps;
	unsigned int irq;

	/*
	 * Spinlock to save hardware registers and buffer descriptors
	 *
	 * all fsl_* functions can be called from gadget layer
	 * without the lock held
	 * they do spin_locK_irqsave() to disable the usb interrupt
	 *
	 * all callback_* functions are called in interrupt context and
	 * use spin_lock()
	 *
	 * all dr_* functions are internal and called with the spinlock held
	 */
	spinlock_t lock;

	unsigned softconnect:1;
	unsigned vbus_active:1;
	unsigned stopped:1;
	unsigned remote_wakeup:1;

	struct ep_queue_head *ep_qh;	/* Endpoints Queue-Head */
	struct fsl_req *ep0_req_out, *ep0_req_in, *ep0_req_setup;

	/* Buffer Descriptor Table (must at a 512-byte memory boundary!) */
	struct k70_bd_entry *bd_table;
	dma_addr_t bd_table_dma;

	struct clk		*clk;	/* USB clock */

	u32 resume_state;	/* USB state to resume */
	u32 usb_state;		/* USB current state */
	u32 ep0_state;		/* Endpoint zero state */
	u32 ep0_dir;		/* Endpoint zero direction: can be
				   USB_DIR_IN or USB_DIR_OUT */
	u8 device_address;	/* Device USB address */

	u32 error_count;
	u32 mark_count;
};


/*
 * ### pipe direction macro from device view
 */
#define USB_RECV	0	/* OUT EP */
#define USB_SEND	1	/* IN EP */

/* USb PID defines */
#define USB_PID_SETUP 0xD
#define USB_PID_IN 0x9
#define USB_PID_OUT 0x1
#define USB_PID_DATA0 0x3
#define USB_PID_DATA1 0xB


/*
 * ### internal used help routines.
 */
#define ep_index(EP)		((EP)->desc->bEndpointAddress&0xF)
#define ep_maxpacket(EP)	((EP)->ep.maxpacket)

#define get_pipe_by_windex(windex)	((windex & USB_ENDPOINT_NUMBER_MASK) \
					* 2 + ((windex & USB_DIR_IN) ? 1 : 0))

/*-------------------------------------------------------------------------*/
/*
#undef DEBUG
#define DEBUG 1
*/

/*
#define VERBOSE 1
*/

#ifdef DEBUG
#define DBG(fmt, args...) 	printk(KERN_DEBUG "[%s] " fmt "\n", \
				__func__, ## args)

#define MARKER() printk(KERN_ALERT \
		"DEBUG: Passed %s %d \n", __func__, __LINE__)
#else
#define DBG(fmt, args...)	do {} while (0)

#define MARKER() do {} while (0)
#endif

#ifdef VERBOSE
#define VDBG		DBG
#else
#define VDBG(stuff...)	do {} while (0)
#endif

#define ERR(stuff...)		pr_err("khci-udc: " stuff)
#define WARNING(stuff...)		pr_warning("khci-udc: " stuff)
#define INFO(stuff...)		pr_info("khci-udc: " stuff)

#endif /* __FSL_K70_UDC_H */
