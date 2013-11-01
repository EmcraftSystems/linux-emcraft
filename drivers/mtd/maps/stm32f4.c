/*
 * Custom memory mapper for STM32F4X9 to workaround bug that
 * SDRAM and static memory (NOR flash) should not be accessed simultaneously
 * See Errata 2.8.7 for details.
 *
 * We put SDRAM in self-refresh before accessing NOR flash
 * and wakeup SDRAM after we have finished access.
 *
 * Based on phys-map code.
 *
 * Copyright (C) 2013
 * Pavel Boldin, Emcraft Systems, paboldin@emcraft.com
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


#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/xip.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/stm32f4.h>
#include <linux/mtd/concat.h>
#include <linux/io.h>

#define CYCLES_PER_US	168


#define MAX_RESOURCES		4

struct stm32f4_flash_info {
	struct mtd_info		*mtd[MAX_RESOURCES];
	struct mtd_info		*cmtd;
	struct map_info		map[MAX_RESOURCES];
#ifdef CONFIG_MTD_PARTITIONS
	int			nr_parts;
	struct mtd_partition	*parts;
#endif
};


struct stm32_fmc_regs {
	/* Control registers */
	u32 sdcr1;
	u32 sdcr2;

	/* Timing registers */
	u32 sdtr1;
	u32 sdtr2;

	/* Mode register */
	u32 sdcmr;

	/* Refresh timing register */
	u32 sdrtr;

	/* Status register */
	u32 sdsr;
};

/*
 * FMC registers base
 */
#define STM32_SDRAM_FMC_BASE			0xA0000140
#define STM32_SDRAM_FMC			((volatile struct stm32_fmc_regs *) \
						STM32_SDRAM_FMC_BASE)

#define FMC_SDCMR_MODE_NORMAL		0
#define FMC_SDCMR_MODE_START_CLOCK	1
#define FMC_SDCMR_MODE_PRECHARGE	2
#define FMC_SDCMR_MODE_AUTOREFRESH	3
#define FMC_SDCMR_MODE_WRITE_MODE	4
#define FMC_SDCMR_MODE_SELFREFRESH	5

#define FMC_SDCMR_BANK_1		(1 << 4)
#define FMC_SDCMR_BANK_2		(1 << 3)

#define FMC_SDSR_BUSY			(1 << 5)

#define FMC_BUSY_WAIT()		do { \
		__asm__ __volatile__ ("dsb" : : : "memory"); \
		while(STM32_SDRAM_FMC->sdsr & FMC_SDSR_BUSY); \
	} while(0);


#define SRAM_DATA	__attribute__((section (".sram.data")))
#define SRAM_TEXT	__attribute__((section (".sram.text"),__long_call__))

static volatile unsigned long SRAM_DATA sr_flash, sr_len;
static volatile map_word SRAM_DATA sr_val;

static volatile unsigned long SRAM_DATA sram_buffer_start, sram_buffer_size;

extern char _sram_start, _sram_end;
extern char __sram_loc, _esram_loc;

static inline int enter_critical_code(void)
{
	int flags;
	local_fiq_disable();
	local_irq_save(flags);
	return flags;
}

static inline void leave_critical_code(int flags)
{
	local_irq_restore(flags);
	local_fiq_enable();
}

inline void loop_delay(volatile unsigned int u)
{
	/* Cycle is two instructions each */
	u >>= 1;
	while(u-- != 0);
}

inline void stop_ram(void)
{
	STM32_SDRAM_FMC->sdcmr = FMC_SDCMR_BANK_1 | FMC_SDCMR_MODE_SELFREFRESH;
	FMC_BUSY_WAIT();
	loop_delay(CYCLES_PER_US);
}

inline void start_ram(void)
{
	STM32_SDRAM_FMC->sdcmr = FMC_SDCMR_BANK_1 | FMC_SDCMR_MODE_PRECHARGE;
	FMC_BUSY_WAIT();
	loop_delay(CYCLES_PER_US);

	STM32_SDRAM_FMC->sdcmr = FMC_SDCMR_BANK_1 | FMC_SDCMR_MODE_NORMAL;
	FMC_BUSY_WAIT();
	loop_delay(CYCLES_PER_US * 60);
}

SRAM_TEXT void sram_stm32f4_flash_read(void)
{
	stop_ram();
	sr_val.x[0] = *(volatile uint16_t __force*)(sr_flash);
	loop_delay(10);
	start_ram();
}

SRAM_TEXT void sram_stm32f4_flash_write(void)
{
	stop_ram();
	*(volatile uint16_t __force*)(sr_flash) = *(uint16_t*)&sr_val.x[0];
	loop_delay(20);
	start_ram();
}

SRAM_TEXT void sram_stm32f4_flash_copy_from(void)
{
	int i;
	stop_ram();
	for(i = sram_buffer_start;sr_len;)
	{
		*(volatile uint16_t __force*)i = *(volatile uint16_t*)sr_flash;

		sr_len -= 2;
		sr_flash += 2;
		i += 2;

		loop_delay(10);
	}
	start_ram();
}

SRAM_TEXT void sram_stm32f4_flash_copy_to(void)
{
	int i;
	stop_ram();
	for(i = sram_buffer_start;sr_len;)
	{
		*(volatile uint16_t __force*)sr_flash = *(volatile uint16_t*)i;

		sr_len -= 2;
		sr_flash += 2;
		i += 2;

		loop_delay(20);
	}
	start_ram();
}

map_word stm32f4_flash_read(struct map_info *map, unsigned long ofs)
{
	int flags;

	sr_flash = map->phys + ofs;

	flags = enter_critical_code();
	sram_stm32f4_flash_read();
	leave_critical_code(flags);

	return sr_val;
}

void stm32f4_flash_write(struct map_info *map, const map_word datum, unsigned long ofs)
{
	int flags;
	sr_flash = map->phys + ofs;
	sr_val = datum;

	flags = enter_critical_code();
	sram_stm32f4_flash_write();
	leave_critical_code(flags);
}

void stm32f4_flash_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	int read_len;
	unsigned long lto = (unsigned long)to;
	int flags;

	for(; len; ) {
		read_len = min(len, sram_buffer_size);

		sr_flash = map->phys + from;
		sr_len = ALIGN(read_len, 2);

		flags = enter_critical_code();
		sram_stm32f4_flash_copy_from();
		leave_critical_code(flags);

		memcpy((void*)lto, (void*)sram_buffer_start, read_len);

		from += read_len;
		lto += read_len;
		len -= read_len;
	}
}

void stm32f4_flash_copy_to(struct map_info *map, unsigned long to, void *from, ssize_t len)
{
	int read_len;
	unsigned long lfrom = (unsigned long)from;
	int flags;

	for(; len; ) {
		read_len = min(len, sram_buffer_size);

		sr_flash = map->phys + to;
		sr_len = ALIGN(read_len, 2);

		memcpy((void*)sram_buffer_start, (void*)lfrom, read_len);

		flags = enter_critical_code();
		sram_stm32f4_flash_copy_to();
		leave_critical_code(flags);

		lfrom += read_len;
		to += read_len;
		len -= read_len;
	}
}

int stm32f4_flash_remove(struct platform_device *dev)
{
	struct stm32f4_flash_info *info;
	struct stm32f4_flash_data *stm32f4_data;
	int i;

	info = platform_get_drvdata(dev);
	if (info == NULL)
		return 0;
	platform_set_drvdata(dev, NULL);

	stm32f4_data = dev->dev.platform_data;

	if (info->cmtd) {
#ifdef CONFIG_MTD_PARTITIONS
		if (info->nr_parts || stm32f4_data->nr_parts) {
			del_mtd_partitions(info->cmtd);

			if (info->nr_parts)
				kfree(info->parts);
		} else {
			del_mtd_device(info->cmtd);
		}
#else
		del_mtd_device(info->cmtd);
#endif
#ifdef CONFIG_MTD_CONCAT
		if (info->cmtd != info->mtd[0])
			mtd_concat_destroy(info->cmtd);
#endif
	}

	for (i = 0; i < MAX_RESOURCES; i++) {
		if (info->mtd[i] != NULL)
			map_destroy(info->mtd[i]);
	}
	return 0;
}


static const char *rom_probe_types[] = {
					"cfi_probe",
					"jedec_probe",
					"qinfo_probe",
					"map_rom",
					NULL };
#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probe_types[] = { "cmdlinepart", "RedBoot", NULL };
#endif

static int stm32f4_flash_probe(struct platform_device *dev)
{
	struct stm32f4_flash_data *stm32f4_data;
	struct stm32f4_flash_info *info;
	const char **probe_type;
	int err = 0;
	int i;
	int devices_found = 0;

	stm32f4_data = dev->dev.platform_data;
	if (stm32f4_data == NULL)
		return -ENODEV;

	info = devm_kzalloc(&dev->dev, sizeof(struct stm32f4_flash_info),
			    GFP_KERNEL);
	if (info == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	platform_set_drvdata(dev, info);

	for (i = 0; i < dev->num_resources; i++) {
		printk(KERN_NOTICE "stm32f4 platform flash device: %.8llx at %.8llx\n",
		       (unsigned long long)(dev->resource[i].end - dev->resource[i].start + 1),
		       (unsigned long long)dev->resource[i].start);

		if (!devm_request_mem_region(&dev->dev,
			dev->resource[i].start,
			dev->resource[i].end - dev->resource[i].start + 1,
			dev_name(&dev->dev))) {
			dev_err(&dev->dev, "Could not reserve memory region\n");
			err = -ENOMEM;
			goto err_out;
		}

		info->map[i].name = dev_name(&dev->dev);
		info->map[i].phys = dev->resource[i].start;
		info->map[i].size = dev->resource[i].end - dev->resource[i].start + 1;
		info->map[i].bankwidth = stm32f4_data->width;
		info->map[i].set_vpp = stm32f4_data->set_vpp;
		info->map[i].pfow_base = stm32f4_data->pfow_base;

		info->map[i].read = stm32f4_flash_read;
		info->map[i].write = stm32f4_flash_write;
		info->map[i].copy_from = stm32f4_flash_copy_from;
		info->map[i].copy_to = stm32f4_flash_copy_to;

		probe_type = rom_probe_types;
		for (; info->mtd[i] == NULL && *probe_type != NULL; probe_type++)
			info->mtd[i] = do_map_probe(*probe_type, &info->map[i]);
		if (info->mtd[i] == NULL) {
			dev_err(&dev->dev, "map_probe failed\n");
			err = -ENXIO;
			goto err_out;
		} else {
			devices_found++;
		}
		info->mtd[i]->owner = THIS_MODULE;
		info->mtd[i]->dev.parent = &dev->dev;
	}

	if (devices_found == 1) {
		info->cmtd = info->mtd[0];
	} else if (devices_found > 1) {
		/*
		 * We detected multiple devices. Concatenate them together.
		 */
#ifdef CONFIG_MTD_CONCAT
		info->cmtd = mtd_concat_create(info->mtd, devices_found, dev_name(&dev->dev));
		if (info->cmtd == NULL)
			err = -ENXIO;
#else
		printk(KERN_ERR "stm32f4-flash: multiple devices "
		       "found but MTD concat support disabled.\n");
		err = -ENXIO;
#endif
	}
	if (err)
		goto err_out;

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(info->cmtd, part_probe_types,
				&info->parts, 0);
	if (err > 0) {
		add_mtd_partitions(info->cmtd, info->parts, err);
		info->nr_parts = err;
		return 0;
	}

	if (stm32f4_data->nr_parts) {
		printk(KERN_NOTICE "Using stm32f4 partition information\n");
		add_mtd_partitions(info->cmtd, stm32f4_data->parts,
				   stm32f4_data->nr_parts);
		return 0;
	}
#endif

	add_mtd_device(info->cmtd);
	return 0;

err_out:
	stm32f4_flash_remove(dev);
	return err;
}

#ifdef CONFIG_PM
static void stm32f4_flash_shutdown(struct platform_device *dev)
{
	struct stm32f4_flash_info *info = platform_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX_RESOURCES && info->mtd[i]; i++)
		if (info->mtd[i]->suspend && info->mtd[i]->resume)
			if (info->mtd[i]->suspend(info->mtd[i]) == 0)
				info->mtd[i]->resume(info->mtd[i]);
}
#else
#define stm32f4_flash_shutdown NULL
#endif

static struct platform_driver stm32f4_flash_driver = {
	.probe		= stm32f4_flash_probe,
	.remove		= stm32f4_flash_remove,
	.shutdown	= stm32f4_flash_shutdown,
	.driver		= {
		.name	= "stm32f4-flash",
		.owner	= THIS_MODULE,
	},
};


static int __init stm32f4_init(void)
{
	int err;
	unsigned long p;

	printk(KERN_INFO "Initializing STM32F4 mapper, copying code from %p to %p, size %d", &__sram_loc, &_sram_start, &_esram_loc - &__sram_loc);

	/*
	 * Copy code to SRAM
	 */
	memcpy((void*)&_sram_start, (void*)&__sram_loc, &_esram_loc - &__sram_loc);

	p = (unsigned long)&_sram_end;
	p = ALIGN(p, 0x100);

	sram_buffer_start = p;
	sram_buffer_size = CONFIG_MTD_STM32F4_SRAM_BUFFER_SIZE;
	printk(KERN_INFO "Using SRAM as buffer with start %p and size %x\n",
		sram_buffer_start, sram_buffer_size);

	err = platform_driver_register(&stm32f4_flash_driver);

	return err;
}

static void __exit stm32f4_exit(void)
{
	platform_driver_unregister(&stm32f4_flash_driver);
}

module_init(stm32f4_init);
module_exit(stm32f4_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pavel Boldin <paboldin@emcraft.com>");
MODULE_DESCRIPTION("STM32F4XX workaround for SDRAM/flash");

/* legacy platform drivers can't hotplug or coldplg */
#ifndef CONFIG_MTD_PHYSMAP_COMPAT
/* work with hotplug and coldplug */
MODULE_ALIAS("platform:stm32f4-flash");
#endif
