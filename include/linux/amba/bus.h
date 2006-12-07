/*
 *  linux/include/asm-arm/hardware/amba.h
 *
 *  Copyright (C) 2003 Deep Blue Solutions Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef ASMARM_AMBA_H
#define ASMARM_AMBA_H

#ifdef CONFIG_ARM_AMBA_DMA
# include <asm/dma.h>
# include <asm/mach/dma.h>
# include <linux/amba/dma.h>
#endif

#define AMBA_NR_IRQS	2

struct amba_device {
	struct device		dev;
	struct resource		res;
	u64			dma_mask;
	unsigned int		periphid;
	unsigned int		irq[AMBA_NR_IRQS];
#ifdef CONFIG_ARM_AMBA_DMA
	struct amba_dma_data	dma_data;
#endif
};

struct amba_id {
	unsigned int		id;
	unsigned int		mask;
	void			*data;
};

struct amba_driver {
	struct device_driver	drv;
	int			(*probe)(struct amba_device *, void *);
	int			(*remove)(struct amba_device *);
	void			(*shutdown)(struct amba_device *);
	int			(*suspend)(struct amba_device *, pm_message_t);
	int			(*resume)(struct amba_device *);
	struct amba_id		*id_table;
};

#define amba_get_drvdata(d)	dev_get_drvdata(&d->dev)
#define amba_set_drvdata(d,p)	dev_set_drvdata(&d->dev, p)

int amba_driver_register(struct amba_driver *);
void amba_driver_unregister(struct amba_driver *);
int amba_device_register(struct amba_device *, struct resource *);
void amba_device_unregister(struct amba_device *);
struct amba_device *amba_find_device(const char *, struct device *, unsigned int, unsigned int);
int amba_request_regions(struct amba_device *, const char *);
void amba_release_regions(struct amba_device *);

#ifdef CONFIG_ARM_AMBA_DMA
struct amba_device * amba_get_device_with_name(char * name);
#endif

#define amba_config(d)	(((d)->periphid >> 24) & 0xff)
#define amba_rev(d)	(((d)->periphid >> 20) & 0x0f)
#define amba_manf(d)	(((d)->periphid >> 12) & 0xff)
#define amba_part(d)	((d)->periphid & 0xfff)

/* AMBA PrimeCell peripheral ids */
/*
 * Note that the hex number is that from the prime cell id
 * e.g. pl041 has id 0x41 NOT 0x29
 * hence OEM peripherals may use 0x<n>[a - f]
 */
#define AMBA_PERIPHID_UART	0x11
#define AMBA_PERIPHID_SSI	0x22
#define AMBA_PERIPHID_RTC	0x31
#define AMBA_PERIPHID_AACI	0x40
#define AMBA_PERIPHID_AACI_97	0x41
#define AMBA_PERIPHID_KMI	0x50
#define AMBA_PERIPHID_GPIO	0x61
#define AMBA_PERIPHID_DMAC2	0x80
#define AMBA_PERIPHID_DMAC1	0x81
#define AMBA_PERIPHID_CLCD	0x110
#define AMBA_PERIPHID_VIC	0x190
#define AMBA_PERIPHID_CIM	0x321

#endif
