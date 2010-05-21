/*
 * Realview PCIX Support
 *
 * Copyright (C) 2007-2010 ARM
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
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <mach/platform.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>

#define PCIX_READ_ACCESS		0x06000000
#define PCIX_WRITE_ACCESS		0x07000000

/* PCI-X Configuration registers */
#define PCI_VENID			0x00
#define PCI_DEVID			0x02
#define PCI_COMMAND			0x04
#define PCI_STATUS			0x06
#define PCI_REVID			0x08
#define PCI_CLCODE			0x09
#define PCI_CALISIZE			0x0C
#define PCI_LTTIMER			0x0D
#define PCI_HDTYPE			0x0E
#define PCI_PTBADDR0L			0x10
#define PCI_PTBADDR0M			0x14
#define PCI_PTBADDR1L			0x18
#define PCI_PTBADDR1M			0x1C
#define PCI_PCIIOLMT			0x1D
#define PCI_PCISSR			0x1E
#define PCI_PCIMBAR			0x20
#define PCI_PCIMLMT			0x22
#define PCI_PCIPMBAR			0x24
#define PCI_PCIPMLMT			0x26
#define PCI_PCIPMBARU32			0x28
#define PCI_SUBSYSVENID			0x2C
#define PCI_PCIPMLMTU32			0x2C
#define PCI_SUBSYSID			0x2E
#define PCI_CAPPOINT			0x34
#define PCI_INTLINE			0x3C
#define PCI_INTPIN			0x3D
#define PCI_BCNTRL			0x3E
#define PCI_CFGADDR			0x40
#define PCI_CFGDATA			0x44
#define PCI_CFGATTR			0x48
#define PCI_PINTACNT			0x50
#define PCI_PINTXSTAT			0x54
#define PCI_PINTXMASK			0x58
#define PCI_SERRSTAT			0x5C
#define PCI_PERRADDRL			0x60
#define PCI_PERRADDRH			0x64
#define PCI_PERRSTAT			0x68
#define PCI_PERRMASK			0x6C
#define PCI_OCBERRSTAT			0x78
#define PCI_OCBERRMASK			0x7C
#define PCI_MSICAPID			0x80
#define PCI_MSINXTCAP			0x81
#define PCI_MSGCNT			0x82
#define PCI_MSGADDR			0x84
#define PCI_MSGUPADDR			0x88
#define PCI_MSGDATA			0x8C
#define PCI_PCIXCAPID			0x90
#define PCI_PCIXNXTCAP			0x91
#define PCI_PCIXCOMMAND			0x92
#define PCI_PCIXSTAT			0x94
#define PCI_SPCMPERRMSG			0x98
#define PCI_PTMEMSEG0			0xA0
#define PCI_PTMEMMSK0			0xA4
#define PCI_PTMEMENSW0			0xA8
#define PCI_PTMEMSEG1			0xB0
#define PCI_PTMEMMSK1			0xB4
#define PCI_PTMEMENSW1			0xB8
#define PCI_PMSEGL			0xC0
#define PCI_PMSEGH			0xC4
#define PCI_DRSTCNT			0xEC
#define PCI_PRST			0xF0
#define PCI_UNITCNT			0xF4
#define PCI_CNTOTIMER			0xF8
#define PCI_PSLADAFLASH			0xFC

static int irq_gic_start;

/*
 * Software is exposed to downstream half of the PCI-X bridge registers
 * rather than a real PCI interface. Therefore there's no config memory
 * region as such.
 *
 *	PCI Regions:
 *	MEM		0xA0000000 - 0xBFFFFFFF
 *	IO		0x00000000 - 0x0000FFFF
 *	CONFIG		-
 */

static void realview_pb_pcix_sync(void)
{
	readl(PCIX_UNIT_BASE + PCI_VENID);
}

void realview_pb_pcix_unit_init(void)
{
	u32 data = readl(PCIX_UNIT_BASE + PCI_UNITCNT);

	if (data & 0x10)
		writel(0x00000000, PCIX_UNIT_BASE + PCI_PRST); /* Assert PCI reset */
	else {
		printk(KERN_ERR "Error: PCI-X unit not in PCI-X mode.\n");
		writel(data | 0x80000000, PCIX_UNIT_BASE + PCI_UNITCNT);
	}

	writew(0x0006, PCIX_UNIT_BASE + PCI_COMMAND); /* Master-Memory enable */
	writew(0xfb30, PCIX_UNIT_BASE + PCI_STATUS);  /* Error bit clear */

	writeb(0x08, PCIX_UNIT_BASE + PCI_CALISIZE); /* Cache line size */
	writeb(0x40, PCIX_UNIT_BASE + PCI_LTTIMER);  /* Latency Timer */

	/* Master Segment Address[31:24] */
	writel(0x00000003, PCIX_UNIT_BASE + PCI_PMSEGL); /* no swap */
	/* Master Segment Address[63:32] */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PMSEGH);

	/* Data endian swap mode */
	writel(0x00000003, PCIX_UNIT_BASE + PCI_PTMEMENSW0); /* no swap */
	writel(0x00000003, PCIX_UNIT_BASE + PCI_PTMEMENSW1); /* no swap */

#ifdef CONFIG_REALVIEW_HIGH_PHYS_OFFSET
	/* 512MB of DMA capable RAM is available - mapped at 0x70000000
	 * Note :- 0x70000000 is *not* on a 512MB boundary so it
	 * must be mapped in two parts */
	/* Window#0 256MB and enable */
	writel(0x0fff0001, PCIX_UNIT_BASE + PCI_PTMEMMSK0);
	/* Window#1 256MB and enable */
	writel(0x0fff0001, PCIX_UNIT_BASE + PCI_PTMEMMSK1);

	/* Window base address setting */
	/* Memory Base Address, Ptrefetch Disable, 64bit */
	writel(0x70000004, PCIX_UNIT_BASE + PCI_PTBADDR0L);
	/* Memory Base Address[63:32] */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PTBADDR0M);

	/* Memory Base Address, Ptrefetch Disable, 64bit */
	writel(0x80000004, PCIX_UNIT_BASE + PCI_PTBADDR1L);
	/* Memory Base Address[63:32] */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PTBADDR1M);
#else
	/* Only 256MB of RAM is available - mapped at zero */
	/* Window#0 256MB and enable */
	writel(0x0fff0001, PCIX_UNIT_BASE + PCI_PTMEMMSK0);
	/* Window#1 Disable */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PTMEMMSK1);

	/* Window base address setting */
	/* Memory Base Address, Ptrefetch Disable, 64bit */
	writel(0x00000004, PCIX_UNIT_BASE + PCI_PTBADDR0L);
	/* Memory Base Address[63:32] */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PTBADDR0M);

	/* Memory Base Address, Ptrefetch Disable, 64bit */
	writel(0x00000004, PCIX_UNIT_BASE + PCI_PTBADDR1L);
	/* Memory Base Address[63:32] */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PTBADDR1M);
#endif

	/* OnChipBus Address#0[35:24] */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PTMEMSEG0);
	/* OnChipBus Address#1[35:24] */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PTMEMSEG1);

	/* 66MHz, 64bit device */
	writel(0x00010000, PCIX_UNIT_BASE + PCI_PCIXSTAT);
	/* Interrupt Mask */
	writel(0x00000000, PCIX_UNIT_BASE + PCI_PINTXMASK);
	/* Enable PCI error status */
	writel(0x000307f7, PCIX_UNIT_BASE + PCI_PERRMASK);
	/* Clear PCI error status */
	writel(0x000307f7, PCIX_UNIT_BASE + PCI_PERRSTAT);
	/* Enable count out */
	writel(0x10FFFFFF, PCIX_UNIT_BASE + PCI_CNTOTIMER);

	realview_pb_pcix_sync();

	udelay(1500);
	/* Deassert PCI reset */
	writel(0x00000001, PCIX_UNIT_BASE + PCI_PRST);
	/* Initial end, Enable arbiter */
	writel(data | 0x80000020, PCIX_UNIT_BASE + PCI_UNITCNT);
	realview_pb_pcix_sync();
	udelay(1500);

	/* Enable bursts on PCI-X */
	writel(0x1, __io_address(REALVIEW_ISSP_REG_BASE + 0x18));
}

void realview_pb_pcix_set_attr(unsigned int tag)
{
	/* Get Device and Funcion number */
	u32 data = readl(PCIX_UNIT_BASE + PCI_PCIXSTAT) & 0x0ff;
	/* attribute set */
	writel(tag | (data << 8), PCIX_UNIT_BASE + PCI_CFGATTR);
}

int realview_pb_pcix_set_config(u32 bus, u32 dev, u32 func, int offset)
{
	u32 mode;
	u32 config_addr;

	writel(0x000307F7, PCIX_UNIT_BASE + PCI_PERRSTAT); /* clear error bit */
	writew(0xfb30, PCIX_UNIT_BASE + PCI_STATUS);	   /* error bit clear */

	if (bus == 0) {
		/* Type0 Configuration cycle */
		mode = readl(PCIX_UNIT_BASE + PCI_UNITCNT);
		if (mode & 0x1) {
			/* check PCI mode */
			printk(KERN_ERR "PCI mode detected during config cycle"
			       "attempt. Should really be in PCI-X mode.\n");
			BUG();
		} else {
			/* PCI-X Mode
			 *
			 * Note that the root complex appears as id 0x1f
			 * since we don't want to allocate this as a device
			 * we stop scanning before we find it.
			 */
			if (dev < 0x10 || dev > 0x1e) {
				realview_pb_pcix_sync();
				return -1;
			}
			config_addr = (1 << dev) | ((dev & 0x1f) << 11) |
				      ((func & 0x7) << 8) | (offset & 0xfc);
			writel(config_addr, PCIX_UNIT_BASE + PCI_CFGADDR);
		}
	} else {
		config_addr = ((bus & 0xff) << 16) | ((dev & 0x1f) << 11) |
			      ((func & 0x7) << 8) | (offset & 0xfc) | 0x1;
		/* Type1 Configuration cycle */
		writel(config_addr, PCIX_UNIT_BASE + PCI_CFGADDR);
	}
	realview_pb_pcix_sync();
	return 0;
}

int realview_pb_pcix_check_error(void)
{
	u32 data;
	realview_pb_pcix_sync();
	data = readl(PCIX_UNIT_BASE + PCI_PERRSTAT);
	if (data) {
		writel(data, PCIX_UNIT_BASE + PCI_PERRSTAT);
		return data;
	}
	return 0;
}

int realview_pb_pci_read_config(struct pci_bus *pbus, u32 devfn,
				     int offset, int size, u32 *data)
{
	u32 bus = pbus->number;
	u32 slot = PCI_SLOT(devfn);
	u32 function = PCI_FUNC(devfn);

	realview_pb_pcix_set_attr(PCIX_READ_ACCESS);
	if (realview_pb_pcix_set_config(bus, slot, function, offset) != 0) {
		*data = ((size == 1) ? 0xFF : ((size == 2) ? 0xFFFF : 0xFFFFFFFF));
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	switch (size) {
	case 1:
		*data = (readl(PCIX_UNIT_BASE + PCI_CFGDATA) >>
			 (8 * (offset & 3))) & 0xff;
		break;
	case 2:
		*data = (u32)readw(PCIX_UNIT_BASE + PCI_CFGDATA +
				   (offset & 3));
		break;
	default:
		*data = readl(PCIX_UNIT_BASE + PCI_CFGDATA);
	}
	/* Error codes from downstream bus don't mean much for software. */
	if (!realview_pb_pcix_check_error())
		return PCIBIOS_SUCCESSFUL;

	return PCIBIOS_DEVICE_NOT_FOUND;
}

int realview_pb_pci_write_config(struct pci_bus *pbus, u32 devfn,
				      int offset, int size, u32 data)
{
	u32 slot = PCI_SLOT(devfn);
	u32 function = PCI_FUNC(devfn);
	u32 bus = pbus->number;
	int err;

	realview_pb_pcix_set_attr(PCIX_WRITE_ACCESS);
	if (realview_pb_pcix_set_config(bus, slot, function, offset) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	switch (size) {
	case 1:
		writeb(data, PCIX_UNIT_BASE + PCI_CFGDATA + (offset & 3));
		break;
	case 2:
		writew(data, PCIX_UNIT_BASE + PCI_CFGDATA + (offset & 3));
		break;
	default:
		writel(data, PCIX_UNIT_BASE + PCI_CFGDATA);
		break;
	}
	/* Write errors don't really matter */
	err = realview_pb_pcix_check_error();
	realview_pb_pcix_set_attr(PCIX_READ_ACCESS);
	return err;
}

static struct pci_ops pci_realview_pb_ops = {
	.read	= realview_pb_pci_read_config,
	.write	= realview_pb_pci_write_config,
};

int __init pci_realview_pb_setup(int nr, struct pci_sys_data *sys)
{
	if (machine_is_realview_pb11mp())
		irq_gic_start = IRQ_PB11MP_GIC_START;
	else
		irq_gic_start = IRQ_PBA8_GIC_START;

	realview_pb_pcix_unit_init();

	/* set bus->cpu mapping */
	sys->mem_offset = 0;
	sys->io_offset = REALVIEW_PB_PCI_IO_BASE;

	/* Note: we don't reserve the addresses for PCI space here because the
	 * window allocator won't be able to get any if we do.
	 */

	return 1;
}


struct pci_bus *pci_realview_pb_scan_bus(int nr, struct pci_sys_data *sys)
{
	return pci_scan_bus(sys->busnr, &pci_realview_pb_ops, sys);
}

void __init pci_realview_pb_preinit(void)
{
	u32 data = readl(__io_address(REALVIEW_SYS_PCI_STAT));
	data &= ~0x00000100; /* Clear the Clock Control bit */
	writel(data, __io_address(REALVIEW_SYS_PCI_STAT));
	udelay(1500);
}

static struct pci_dev *pcie_bridge;

/* Maps scrambled IRQ pins to IRQ lines. Scrambling depends on device slot. */
static int __init pci_realview_pb_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq;
	int devslot = PCI_SLOT(dev->devfn);

	/* Upstream port of 8518 device */
	if (unlikely(dev->device == 0x8518 && dev->bus->primary == 0)) {
		BUG_ON(pcie_bridge);	/* Can't be already assigned */
		pcie_bridge = dev;
	}

	if (pcie_bridge) {
		if (dev->bus->primary >= pcie_bridge->bus->secondary &&
		    dev->bus->primary <= pcie_bridge->bus->subordinate)
			goto pcie;
	}

	/*
	 * slot	  pin	 irq
	 * 0	  #A(3)  53
	 * 0	  #B(0)  50
	 * 0	  #C(1)  51
	 * 0	  #D(2)  52
	 *
	 * 1	  #A(2)  52
	 * 1	  #B(3)  53
	 * 1	  #C(0)  50
	 * 1	  #D(1)  51
	 *
	 * 2	  1	 51
	 * 2	  2	 52
	 * 2	  3	 53
	 * 2	  0	 50
	 *
	 * 3	  0	 50
	 * 3	  1	 51
	 * 3	  2	 52
	 * 3	  3	 53
	 *
	 * Note: pin is actually 1..4 not 0..3
	 */

	irq = irq_gic_start + 50 + (((pin - 1) + 3 - (devslot & 3)) & 3);

	printk(KERN_INFO "PCI map irq: slot %d, pin %d, devslot %d, irq: %d\n",
	       slot, pin, devslot, irq);

	return irq;

pcie:
	/*
	 * The problem is that in PCIE in different slots a device
	 * could still get the same slot/pin/devslot triplet. Therefore
	 * we check the slot/pin/devslot triplet of an underlying PCIE port.
	 *
	 * slot   pin	 irq
	 * 0	  #A(1)  56
	 * 0	  #B(2)  57
	 * 0	  #C(3)  54
	 * 0	  #D(4)  55
	 * 1	  #A(1)  55
	 * 1	  #B(2)  56
	 * 1	  #C(3)  57
	 * 1	  #D(4)  54
	 * 2	  1	 54
	 * 2	  2	 55
	 * 2	  3	 56
	 * 2	  4	 57
	 * 3	  1	 57
	 * 3	  2	 54
	 * 3	  3	 55
	 * 3	  4	 56
	 * 4	  1	 56
	 * 4	  2	 57
	 * 4	  3	 54
	 * 4	  4	 55
	 */

	if (dev->bus->self) {
		/* We do have a bus above us
		 * Device is right behind a PCIE hub port, and its not a PCIE
		 * hub port itself. */
		if (dev->bus->self->device == 0x8518 &&
		    dev->device != 0x8518) {
			/* Depend on devslot of 8518 port we're connected to */
			devslot = PCI_SLOT(dev->bus->self->devfn);
			/* These are the two cases for PCIE ports on PB boards.
			 * Any other downstream bus topology (e.g. with further
			 * PCIE bridges) does not scramble, and get the same
			 * irq number as the upstream bus. */
			irq = irq_gic_start + 54 + ((devslot & 3) + (pin - 1)) % 4;
		} else if ((dev->bus->self->class & 0xff0000) == 0x060000 &&
			   (dev->class & 0xff0000) != 0x060000) {
			/* It's a device behind a bridge that isn't an 8518 */
			irq = irq_gic_start + 54 + ((devslot & 3) + pin +
			      PCI_SLOT(dev->bus->self->bus->self->devfn) - 1) % 4;
		} else {
			/* It's another bridge */
			irq = dev->bus->self->irq;
		}
	} else
		irq = 0;

	printk(KERN_INFO "PCI Express irq mapping: device: 0x%x, slot %d, pin %d, devslot %d, irq: %d\n",
	       dev->device, slot, pin, devslot, irq);

	return irq;
}

static struct hw_pci realview_pb_pci __initdata = {
	.swizzle		= NULL,
	.map_irq		= pci_realview_pb_map_irq,
	.nr_controllers		= 1,
	.setup			= pci_realview_pb_setup,
	.scan			= pci_realview_pb_scan_bus,
	.preinit		= pci_realview_pb_preinit
};

static int __init realview_pb_pci_init(void)
{
	if (machine_is_realview_pb11mp() ||
	    machine_is_realview_pba8() || machine_is_realview_pbx())
		pci_common_init(&realview_pb_pci);
	return 0;
}

subsys_initcall(realview_pb_pci_init);


static void pcie_fix_sizes(void)
{
	struct pci_dev *pdev = NULL;
	int rrq;
	int max_rrq = 4096;

	/* Set the max read request size for all devices to the
	 * smallest in the tree. So far, the only device we've seen
	 * that fails without this is the Marvell Yukon 88E8053 but
	 * this may fix other devices too.
	 */
	while ((pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pdev)) != NULL) {
		rrq = pcie_get_readrq(pdev);
		if (rrq > 0 && rrq < max_rrq)
			max_rrq = rrq;
	}

	pdev = NULL;	/* reset scan */
	while ((pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pdev)) != NULL) {
		rrq = pcie_get_readrq(pdev);
		printk(KERN_DEBUG "%s: %02x:%02x %04x/%04x : RRQSZ %d -> %d\n",
				__FUNCTION__, pdev->bus->number, pdev->devfn,
				pdev->vendor, pdev->device, rrq, max_rrq);
		pcie_set_readrq(pdev, max_rrq);
	}
}

int __init vexpress_pcie_fixups(void)
{
	pcie_fix_sizes();
	return 0;
}
late_initcall(vexpress_pcie_fixups);
