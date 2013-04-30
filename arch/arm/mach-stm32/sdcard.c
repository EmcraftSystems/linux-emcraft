/*
 * (C) Copyright 2012,2013
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Vladimir Khusainov <vlad@emcraft.com>
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>

#include <mach/stm32.h>
#include <mach/sdcard.h>
#include <mach/platform.h>

#if defined(CONFIG_STM32_SD)

/*
 * SD card controller register map base and length
 */
#define STM32_SD_BASE		0x40012C00
#define STM32_SD_SIZE		0x400
/*
 * SD card controller IRQ number
 */
#define STM32_SD_IRQ		49
/*
 * STM32 SD card controller is almost compatible to ux500
 */
#define U8500_SDI_V2_PERIPHID	0x10480180

/*
 * This function will be called in the beginning of mmci_set_ios()
 */
static int stm32_ios_handler(struct device *dev, struct mmc_ios *ios)
{
	/*
	 * The STM32 SD card controller does has no POWER[OD] bit. If we let
	 * the MMCI driver set this bit, the SD card will not work. We hide
	 * the MMC_BUSMODE_OPENDRAIN bus mode from the driver to prevent it
	 * from writing into that bit.
	 */
	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		ios->bus_mode = MMC_BUSMODE_PUSHPULL;

	return 0;
}

/*
 * Returns !0 when card is removed, 0 when present
 */
static unsigned int mmc_card_detect(struct device *dev)
{
	return 0;
}

static struct mmci_platform_data stm32_mci_data = {
	.ios_handler = stm32_ios_handler,
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
#ifdef CONFIG_STM32_SD_DMA
	.f_max = 25000000,
#else
	.f_max = 400000,
#endif /* CONFIG_STM32_SD_DMA */
	.capabilities = MMC_CAP_4_BIT_DATA,
	.gpio_cd = -1,
	.gpio_wp = -1,
	.status = mmc_card_detect,
};

#endif /* CONFIG_STM32_SD */

void __init stm32_sdcard_init(void)
{
#if defined(CONFIG_STM32_SD)
	int platform;
	int have_sd;

	/*
	 * Initialize platform-specific parameters
	 */
	platform = stm32_platform_get();
	have_sd = 0;
	switch (platform) {
	case PLATFORM_STM32_STM3220G_EVAL:
	case PLATFORM_STM32_STM3240G_EVAL:
		have_sd = 1;
		break;
	default:
		break;
	}

	if (!have_sd)
		goto out;

	/*
	 * Register the SD card interface of STM32
	 */
	amba_ahb_device_add(
		NULL, "mmci0",
		STM32_SD_BASE, STM32_SD_SIZE, STM32_SD_IRQ, NO_IRQ,
		&stm32_mci_data, U8500_SDI_V2_PERIPHID);

out:
	;
#endif
}
