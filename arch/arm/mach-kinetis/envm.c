/*
 * (C) Copyright 2018, Emcraft Systems
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

#include <mach/kinetis.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <mach/platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

/*
 * Size of a sector in the MCU internal flash is 4 Kbytes (K70) or
 * 2 Kbytes (K60) on the program flash only MCUs.
 */
#define FLASH_SECTOR_SIZE	(4 * 1024)

/*
 * The measurement unit for the section size used by the Program Section
 * command.
 */
#define KINETIS_ENVM_SIZE_UNIT_BITS	4	/* 16 bytes */

/*
 * Address of the flash programming acceleration RAM
 */
#define FLASH_PROG_ACCEL_BASE	0x14000000
#define FLASH_PROG_ACCEL_PTR	((u8 *)FLASH_PROG_ACCEL_BASE)

/*
 * Pointer to the beginning of flash
 */
#define KINETIS_NVM_PTR	((u8 *)ENVM_PHYS_OFFSET)

/*
 * FTFL commands
 */
#define FTFL_CMD_ERASE_SECTOR		0x09
#define FTFL_CMD_PROGRAM_SECTION	0x0B

/*
 * Flash Memory Controller (FMC) register map
 */
struct kinetis_fmc_regs {
	u32 pfapr;	/* Flash Access Protection Register */
	u32 pfb0cr;	/* Flash Bank 0 Control Register */
	u32 pfb1cr;	/* Flash Bank 1 Control Register */
};

/*
 * FMC registers base
 */
#define KINETIS_FMC_BASE		(KINETIS_AIPS0PERIPH_BASE + 0x0001F000)
#define KINETIS_FMC			((volatile struct kinetis_fmc_regs *) \
					KINETIS_FMC_BASE)

/*
 * Flash Bank Control Registers (FMC_PFB0CR, FMC_PFB1CR)
 */
/* Bank Data Cache Enable */
#define KINETIS_FMC_PFBCR_BDCE_MSK	(1 << 4)
/* Bank Single Entry Buffer Enable */
#define KINETIS_FMC_PFBCR_BSEBE_MSK	(1 << 0)

/* Command timeout */
#define KINETIS_FTFL_CMD_TIMEOUT	1000000		/* 1 second */

/*
 * Flash Memory Module (FTFL) register map
 */
struct kinetis_ftfl_regs {
	u8 fstat;	/* Flash Status Register */
	u8 fcnfg;	/* Flash Configuration Register */
	u8 fsec;	/* Flash Security Register */
	u8 fopt;	/* Flash Option Register */

	/* Flash Common Command Object Registers (3:0) */
	u8 fccob3;
	u8 fccob2;
	u8 fccob1;
	u8 fccob0;

	/* Flash Common Command Object Registers (7:4) */
	u8 fccob7;
	u8 fccob6;
	u8 fccob5;
	u8 fccob4;

	/* Flash Common Command Object Registers (0xB:8) */
	u8 fccobB;
	u8 fccobA;
	u8 fccob9;
	u8 fccob8;
};

/*
 * FTFL registers base
 */
#define KINETIS_FTFL_BASE		(KINETIS_AIPS0PERIPH_BASE + 0x00020000)
#define KINETIS_FTFL			((volatile struct kinetis_ftfl_regs *) \
					KINETIS_FTFL_BASE)

/*
 * Flash Status Register (FTFL_FSTAT)
 */
/* Command Complete Interrupt Flag */
#define KINETIS_FTFL_FSTAT_CCIF_MSK	(1 << 7)
/* FTFL Read Collision Error Flag */
#define KINETIS_FTFL_FSTAT_RDCOLERR_MSK	(1 << 6)
/* Flash Access Error Flag */
#define KINETIS_FTFL_FSTAT_ACCERR_MSK	(1 << 5)
/* Flash Protection Violation Flag */
#define KINETIS_FTFL_FSTAT_FPVIOL_MSK	(1 << 4)

static struct mtd_erase_region_info k70_region_info = {
	.offset = 0,
	.erasesize = FLASH_SECTOR_SIZE,
	.numblocks = 256,
};

/*
 * Wait for the command completion
 */
static int kinetis_ftfl_command_wait(int tout)
{
	int t = tout / 100;

	while (!(KINETIS_FTFL->fstat & KINETIS_FTFL_FSTAT_CCIF_MSK) && (t > 0)) {
		udelay(100);
		t --;
	}

	if (!t)
		return -ETIMEDOUT;
	else
		return 0;
}

/*
 * Execute an FTFL command
 *
 * `flash_addr` goes to FCCOB[3:1], converted to big-endian.
 * `data0` goes FCCOB[7:4], converted to big-endian.
 * `data1` goes FCCOB[0xB:8], converted to big-endian.
 */
static int kinetis_ftfl_command(u8 command, u32 flash_addr, u32 data0, u32 data1)
{
	int rv;

	/*
	 * This problem exists only in first released product version (mask 0M33Z)
	 *
	 * Single entry buffer disable; Data Cache disable.
	 */
	KINETIS_FMC->pfb0cr &=
		~(KINETIS_FMC_PFBCR_BDCE_MSK | KINETIS_FMC_PFBCR_BSEBE_MSK);
	KINETIS_FMC->pfb1cr &=
		~(KINETIS_FMC_PFBCR_BDCE_MSK | KINETIS_FMC_PFBCR_BSEBE_MSK);

	/*
	 * Wait until the previous command is finished
	 */

	rv = kinetis_ftfl_command_wait(KINETIS_FTFL_CMD_TIMEOUT);
	if (rv != 0) {
		goto out;
	}

	/*
	 * Clear the error bits before starting the next command
	 */
	KINETIS_FTFL->fstat = (KINETIS_FTFL_FSTAT_ACCERR_MSK | KINETIS_FTFL_FSTAT_FPVIOL_MSK);

	/* Write the command code to FCCOB[0] */
	KINETIS_FTFL->fccob0 = command;

	/* Write the flash address to FCCOB[3:1] */
	KINETIS_FTFL->fccob1 = (u8)(flash_addr >> 16);	/* flash_addr[23:16] */
	KINETIS_FTFL->fccob2 = (u8)(flash_addr >> 8);	/* flash_addr[15:8] */
	KINETIS_FTFL->fccob3 = (u8)flash_addr;		/* flash_addr[7:0] */

	/* Write the data word 0 to FCCOB[7:4] */
	KINETIS_FTFL->fccob4 = (u8)(data0 >> 24);	/* data0[31:24] */
	KINETIS_FTFL->fccob5 = (u8)(data0 >> 16);	/* data0[23:16] */
	KINETIS_FTFL->fccob6 = (u8)(data0 >> 8);	/* data0[15:8] */
	KINETIS_FTFL->fccob7 = (u8)data0;		/* data0[7:0] */

	/* Write the data word 1 to FCCOB[7:4] */
	KINETIS_FTFL->fccob8 = (u8)(data1 >> 24);	/* data1[31:24] */
	KINETIS_FTFL->fccob9 = (u8)(data1 >> 16);	/* data1[23:16] */
	KINETIS_FTFL->fccobA = (u8)(data1 >> 8);	/* data1[15:8] */
	KINETIS_FTFL->fccobB = (u8)data1;		/* data1[7:0] */

	/*
	 * Start command execution
	 */
	KINETIS_FTFL->fstat = KINETIS_FTFL_FSTAT_CCIF_MSK;

	rv = kinetis_ftfl_command_wait(KINETIS_FTFL_CMD_TIMEOUT);
	if (rv != 0) {
		goto out;
	}

	if (KINETIS_FTFL->fstat &
	    (KINETIS_FTFL_FSTAT_ACCERR_MSK | KINETIS_FTFL_FSTAT_FPVIOL_MSK |
	     KINETIS_FTFL_FSTAT_RDCOLERR_MSK))
		rv = -EIO;

 out:
	return rv;
}

/*
 * Copy memory buffer to a sector in flash
 */
static int kinetis_flash_program_sector(u32 sect, u32 sect_offset,
					const u_char *src, u32 size)
{
	int rv;
	/*
	 * Save the existing data before the requested region
	 */
	if (sect_offset > 0) {
		memcpy(FLASH_PROG_ACCEL_PTR,
		       KINETIS_NVM_PTR + sect * FLASH_SECTOR_SIZE,
		       sect_offset);
	}
	/*
	 * Save the existing data after the requested region
	 */
	if (sect_offset + size < FLASH_SECTOR_SIZE) {
		memcpy(FLASH_PROG_ACCEL_PTR + sect_offset + size,
		       KINETIS_NVM_PTR + sect * FLASH_SECTOR_SIZE +
		       sect_offset + size,
		       FLASH_SECTOR_SIZE - (sect_offset + size));
	}
	/*
	 * Copy the input data into the flash programming acceleration RAM
	 */
	memcpy(FLASH_PROG_ACCEL_PTR + sect_offset, src, size);

	/*
	 * Program the sector
	 */
	rv = kinetis_ftfl_command(FTFL_CMD_PROGRAM_SECTION,
				  sect * FLASH_SECTOR_SIZE,
				  FLASH_SECTOR_SIZE << (16 - KINETIS_ENVM_SIZE_UNIT_BITS), 0);

	return rv;
}

/*
 * Copy memory buffer to flash
 */
static int kinetis_flash_program(u32 dest_addr, const u_char *src, u32 size)
{
	int rv;
	u32 sect, first_sect, last_sect;
	u32 sect_offset, sect_len;
	int src_offset = 0;

	first_sect = dest_addr / FLASH_SECTOR_SIZE;
	last_sect = (dest_addr + size - 1) / FLASH_SECTOR_SIZE;

	for (sect = first_sect; sect <= last_sect; sect ++) {
		if (first_sect == last_sect) {
			sect_offset = dest_addr - first_sect * FLASH_SECTOR_SIZE;
			sect_len = size;
		} else if (sect == first_sect) {
			sect_offset = dest_addr - first_sect * FLASH_SECTOR_SIZE;
			sect_len = FLASH_SECTOR_SIZE - sect_offset;
		} else if (sect == last_sect) {
			sect_offset = 0;
			sect_len = dest_addr + size - last_sect * FLASH_SECTOR_SIZE;
		} else { /* a sector in the middle */
			sect_offset = 0;
			sect_len = FLASH_SECTOR_SIZE;
		}

		rv = kinetis_flash_program_sector(sect, sect_offset,
						  src + src_offset, sect_len);
		src_offset += sect_len;

		if (rv != 0)
			break;
	}

	return rv;
}

/*
 * Write a data buffer to internal Flash.
 */
static int kinetis_envm_mtd_write(struct mtd_info *mtd, loff_t offset, size_t size,
				  size_t *retlen, const u_char *buf)
{
	int rv = 0;

	if (offset < ENVM_PHYS_OFFSET ||
	    offset + size > ENVM_PHYS_OFFSET + ENVM_PHYS_SIZE) {
		printk("%s: Address %llx is not in flash or "
			"size 0x%x is too big\n",
			__func__, offset, size);
		rv = -EINVAL;
		goto out;
	}

	if ((rv = kinetis_flash_program(offset, buf, size)) < 0)
		goto out;

	if (rv == 0)
		*retlen = size;
out:

	return rv;
}

static int kinetis_envm_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
			size_t *retlen, u_char *buf)
{
	struct map_info *map = mtd->priv;
	int rv;

	map_copy_from(map, buf, from, len);

	*retlen = len;

	rv = 0;

	return rv;
}

static int kinetis_flash_erase(u32 dest_addr, u32 size)
{
	int rv;
	u32 sect, first_sect, last_sect;

	first_sect = dest_addr / FLASH_SECTOR_SIZE;
	last_sect = (dest_addr + size - 1) / FLASH_SECTOR_SIZE;

	for (sect = first_sect; sect <= last_sect; sect ++) {
		/*
		 * Erase the sector
		 */
		rv = kinetis_ftfl_command(FTFL_CMD_ERASE_SECTOR,
					  sect * FLASH_SECTOR_SIZE, 0, 0);
		if (rv != 0)
			break;
	}

	return rv;
}

static int kinetis_envm_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int rv = 0;
	loff_t offset = instr->addr;
	size_t size = instr->len;

	if (offset < ENVM_PHYS_OFFSET ||
	    offset + size > ENVM_PHYS_OFFSET + ENVM_PHYS_SIZE) {
		printk("%s: Address %llx is not in flash or "
			"size 0x%x is too big\n",
			__func__, offset, size);
		rv = -EINVAL;
		goto out;
	}

	if ((rv = kinetis_flash_erase(offset, size)) < 0)
		goto out;

 out:
	if (rv == 0) {
		instr->state = MTD_ERASE_DONE;
		mtd_erase_callback(instr);
	}
	return rv;
}

static struct mtd_info *kinetis_envm_probe(struct map_info *map)
{
	struct mtd_info *mtd = NULL;

	mtd = kzalloc(sizeof(*mtd), GFP_KERNEL);

	if (!mtd) {
		printk(KERN_ERR "%s: mtd info alloc failure\n", __func__);
		goto done;
	}

	mtd->size = ENVM_PHYS_SIZE;
	mtd->priv = map;
	mtd->type = MTD_NORFLASH;

	mtd->erase = kinetis_envm_mtd_erase;
	mtd->read  = kinetis_envm_mtd_read;
	mtd->write = kinetis_envm_mtd_write;

	mtd->name = map->name;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->writesize = 1;

	map->fldrv_priv = NULL;

	mtd->numeraseregions = 1;
	mtd->eraseregions = &k70_region_info;
	mtd->erasesize = k70_region_info.erasesize;

 done:
	return mtd;
}

static struct mtd_chip_driver kinetis_envm_chipdrv = {
	.probe	= kinetis_envm_probe,
	.name	= "kinetis_envm",
	.module	= THIS_MODULE
};

static struct mtd_partition flash_partitions[] = {
	{
		.name	= "envm-uboot",
		.offset = 0x00000,
		.size	= 0x40000,
	},
	{
		.name	= "envm-uboot-env",
		.offset = 0x40000,
		.size	= 0x08000,
	},
	{
		.name	= "envm-keys",
		.offset = 0x48000,
		.size	= 0x08000,
	},
	{
		.name	= "envm-unused",
		.offset = 0x50000,
		.size	= 0xb0000,
	},
};

struct physmap_flash_data flash_data = {
	.nr_parts       = ARRAY_SIZE(flash_partitions),
	.parts          = flash_partitions,
	.width		= 2,
};

static struct resource envm_resources[] = {
	[0] = {
		.start = ENVM_PHYS_OFFSET,
		.end   = ENVM_PHYS_OFFSET + ENVM_PHYS_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

/*
 * Platform device for the physmap-flash
 */
static struct platform_device envm_device = {
	.name           = "physmap-flash",
	.id             = -1,
	.resource       = envm_resources,
	.num_resources  = ARRAY_SIZE(envm_resources),
	.dev		= {
		.platform_data = &flash_data,
	},
};

void __init kinetis_envm_init(void)
{
	register_mtd_chip_driver(&kinetis_envm_chipdrv);
	platform_device_register(&envm_device);
}
