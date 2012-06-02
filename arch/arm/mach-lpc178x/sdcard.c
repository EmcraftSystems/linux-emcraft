/*
 * (C) Copyright 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
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

#include <mach/sdcard.h>
#include <mach/power.h>
#include <mach/platform.h>

/*
 * SD Card controller registers base
 */
#define LPC178X_SD_BASE		(LPC178X_APB1PERIPH_BASE + 0x00040000)
/*
 * "Interrupt ID" in Table 43 in the LPC178x/7x User Manual (page 70)
 */
#define LPC178X_SD_IRQ		29

/*
 * System Controls and Status register
 */
/* Selects the active level of the SD card interface signal SD_PWR */
#define LPC178X_SCC_SCS_MCIPWR_MSK	(1 << 3)

/*
 * Returns !0 when card is removed, 0 when present
 */
static unsigned int mmc_card_detect(struct device *dev)
{
	return 0;
}

static struct amba_device lpc178x_mci_device = {
	.dev = {
		.coherent_dma_mask	= ~0,
		.init_name		= "dev:mmc0",
	},
	.res = {
		.start	= LPC178X_SD_BASE,
		.end	= LPC178X_SD_BASE + SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
	.irq = { LPC178X_SD_IRQ, NO_IRQ },
};

static struct mmci_platform_data lpc178x_mci_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.capabilities = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_LPC178X_SD_DMA
	.f_max = 25000000,
#else
	.f_max = 100000,
#endif /* CONFIG_LPC178X_SD_DMA */
	.gpio_cd = -1,
	.gpio_wp = -1,
	.status = mmc_card_detect,
};

void __init lpc178x_sdcard_init(void)
{
	int platform;
	int have_sd;
	/* Active level of the SD card interface signal SD_PWR */
	int pwr_high;

	/*
	 * Initialize platform-specific parameters
	 */
	platform = lpc178x_platform_get();
	have_sd = 0;
	switch (platform) {
	case PLATFORM_LPC178X_EA_LPC1788:
		have_sd = 1;
		/* SD_PWR: active low */
		pwr_high = 0;

		/* Use the largest SRAM region for SD card DMA Tx buffer */
		lpc178x_mci_data.dma_tx_v_base = (void *)0x10000000;
		lpc178x_mci_data.dma_tx_size = SZ_64K;
		break;
	case PLATFORM_LPC178X_LNX_EVB:
		/* SD Card interface is not supported on LPC-LNX-EVB */
		break;
	default:
		break;
	}

	if (!have_sd)
		goto out;

	/*
	 * Enable the power on the SD Card Interface module of the MCU
	 * before we call `amba_device_register()`. This is required, because
	 * `amba_device_register()` will try to read the AMBA device ID
	 * from the SD Card module's register map; if the power on the module
	 * is off, its registers are not accessible.
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCSDC_MSK, 1);

	if (pwr_high)
		LPC178X_SCC->scs |= LPC178X_SCC_SCS_MCIPWR_MSK;
	else
		LPC178X_SCC->scs &= ~LPC178X_SCC_SCS_MCIPWR_MSK;

	lpc178x_mci_device.dev.platform_data = &lpc178x_mci_data;
	amba_device_register(&lpc178x_mci_device, &iomem_resource);

out:
	;
}
