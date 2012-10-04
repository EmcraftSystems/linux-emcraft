/*
 * ADC driver for Freescale Kinetis MCUs
 *
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

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/kinetis_adc.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/power.h>

/*
 * Plus-side or minus-side calibration values in the ADC register map
 */
struct kinetis_adc_calib_regs {
	u32 cld;
	u32 cls;
	u32 cl4;
	u32 cl3;
	u32 cl2;
	u32 cl1;
	u32 cl0;
};

/*
 * Kinetis ADC module register map
 */
struct kinetis_adc_regs {
	u32 sc1a;	/* Status and control register 1 (channel A) */
	u32 sc1b;	/* Status and control register 1 (channel B) */
	u32 cfg1;	/* Configuration register 1 */
	u32 cfg2;	/* Configuration register 1 */
	u32 ra;		/* Data result register (channel A) */
	u32 rb;		/* Data result register (channel B) */
	u32 cv1;	/* Compare value register 1 */
	u32 cv2;	/* Compare value register 2 */
	u32 sc2;	/* Status and control register 2 */
	u32 sc3;	/* Status and control register 3 */
	u32 ofs;	/* Offset correction register */
	u32 pg;		/* Plus-side gain register */
	u32 mg;		/* Minus-side gain register */
	struct kinetis_adc_calib_regs plus_cal;
	u32 pga;	/* Programmable Gain Amplifier register */
	struct kinetis_adc_calib_regs minus_cal;
};

/* Conversion complete flag */
#define KINETIS_ADC_SC1_COCO_MSK	(1 << 7)
/* Clock divide select */
#define KINETIS_ADC_CFG1_ADIV_BITS	5
#define KINETIS_ADC_CFG1_ADIV_8		(3 << KINETIS_ADC_CFG1_ADIV_BITS)
/* Sample time configuration: 1=long sample time */
#define KINETIS_ADC_CFG1_ADLSMP_MSK	(1 << 4)
/* Conversion mode selection (bit count) */
#define KINETIS_ADC_CFG1_MODE_BITS	2
#define KINETIS_ADC_CFG1_MODE_16BIT	(3 << KINETIS_ADC_CFG1_MODE_BITS)
/* Input clock select */
#define KINETIS_ADC_CFG1_ADICLK_BITS	0
#define KINETIS_ADC_CFG1_ADICLK_BUSCLK	(0 << KINETIS_ADC_CFG1_ADICLK_BITS)
/* High speed configuration */
#define KINETIS_ADC_CFG2_ADHSC_MSK	(1 << 2)
/* Conversion trigger select */
#define KINETIS_ADC_SC2_ADTRG_MSK	(1 << 6)
/* Calibration */
#define KINETIS_ADC_SC3_CAL_MSK		(1 << 7)
/* Calibration failed flag */
#define KINETIS_ADC_SC3_CALF_MSK	(1 << 6)
/* Continuous conversion enable */
#define KINETIS_ADC_SC3_ADCO_MSK	(1 << 3)
/* Hardware average enable */
#define KINETIS_ADC_SC3_AVGE_MSK	(1 << 2)
/* Hardware average select */
#define KINETIS_ADC_SC3_AVGS_BITS	0
#define KINETIS_ADC_SC3_AVGS_MSK	(3 << KINETIS_ADC_SC3_AVGS_BITS)
#define KINETIS_ADC_SC3_AVGS_32		(3 << KINETIS_ADC_SC3_AVGS_BITS)

/* Number of ADC modules: ADC0, ADC1, ADC2, ADC3 */
#define N_ADC_MODULES			4

/*
 * Internal structure to store information about an ADC module
 */
struct kinetis_adc_module {
	spinlock_t lock;
	int enabled;
	volatile struct kinetis_adc_regs *regs;
};

static struct kinetis_adc_module adc_modules[N_ADC_MODULES];

/*
 * Kinetis clock gate descriptors for the kinetis_periph_enable() function
 */
const static kinetis_clock_gate_t kinetis_adc_cg[N_ADC_MODULES] = {
	KINETIS_CG_ADC0, KINETIS_CG_ADC1, KINETIS_CG_ADC2, KINETIS_CG_ADC3
};

/*
 * Plus-side or minus-side calibration
 */
static void adc_calibrate_side(
	volatile struct kinetis_adc_calib_regs *cal_regs, volatile u32 *gain)
{
	u16 cal_var;

	cal_var =
		cal_regs->cl0 + cal_regs->cl1 + cal_regs->cl2 +
		cal_regs->cl3 + cal_regs->cl4 + cal_regs->cls;
	cal_var /= 2;
	cal_var |= 0x8000;
	*gain = cal_var;
}

/*
 * Calibrate an ADC module
 */
static long adc_calibrate_module(volatile struct kinetis_adc_regs *regs)
{
	long rv;

	/* Enable software conversion trigger */
	regs->sc2 &= ~KINETIS_ADC_SC2_ADTRG_MSK;
	/* Set single conversion; turn on averaging */
	regs->sc3 = (regs->sc3 &
		~(KINETIS_ADC_SC3_ADCO_MSK | KINETIS_ADC_SC3_AVGS_MSK)) |
		KINETIS_ADC_SC3_AVGE_MSK | KINETIS_ADC_SC3_AVGS_32;
	/* Start calibration */
	regs->sc3 |= KINETIS_ADC_SC3_CAL_MSK;
	while (!(regs->sc1a & KINETIS_ADC_SC1_COCO_MSK));

	if (regs->sc3 & KINETIS_ADC_SC3_CALF_MSK) {
		/* Calibration failed */
		rv = -EAGAIN;
		goto out;
	}

	/* Plus-side calibration */
	adc_calibrate_side(&regs->plus_cal, &regs->pg);
	/* Minus-side calibration */
	adc_calibrate_side(&regs->minus_cal, &regs->mg);

	/* Finish calibration */
	regs->sc3 &= ~KINETIS_ADC_SC3_CAL_MSK;

	rv = 0;
out:
	return rv;
}

/*
 * Enable or disable an ADC module
 */
static long adc_enable_module(unsigned int mod, int enable)
{
	long rv;
	volatile struct kinetis_adc_regs *regs;

	if (mod >= N_ADC_MODULES) {
		rv = -ENODEV;
		goto out;
	}

	spin_lock(&adc_modules[mod].lock);

	if (!!adc_modules[mod].enabled == !!enable) {
		rv = -EBUSY;
		goto unlock;
	}

	/* Change the recorded state of ADC module */
	adc_modules[mod].enabled = enable;

	/* Enable or disable the ADC module */
	kinetis_periph_enable(kinetis_adc_cg[mod], enable);
	if (enable) {
		regs = adc_modules[mod].regs;

		/* Calibrate the ADC module */
		rv = adc_calibrate_module(regs);
		if (rv != 0)
			goto unlock;

		/* Bus clock/4; long sample time */
		regs->cfg1 =
			KINETIS_ADC_CFG1_ADIV_8 | KINETIS_ADC_CFG1_ADLSMP_MSK |
			KINETIS_ADC_CFG1_MODE_16BIT |
			KINETIS_ADC_CFG1_ADICLK_BUSCLK;
		/* High speed configuration */
		regs->cfg2 = KINETIS_ADC_CFG2_ADHSC_MSK;
		/* Software triggering; no DMA; disable compare function */
		regs->sc2 = 0;
		/* Enable 32-sample averaging */
		regs->sc3 =
			KINETIS_ADC_SC3_AVGE_MSK | KINETIS_ADC_SC3_AVGS_32;
		/* Disable PGA */
		regs->pga = 0;
	}

	rv = 0;
unlock:
	spin_unlock(&adc_modules[mod].lock);
out:
	return rv;
}

/*
 * Perform a single measurement on an ADC channel
 */
static long adc_measure(struct kinetis_adc_request *req)
{
	long rv;
	unsigned int mod;
	volatile struct kinetis_adc_regs *regs;

	mod = req->adc_module;
	if (mod >= N_ADC_MODULES) {
		rv = -ENODEV;
		goto out;
	}

	spin_lock(&adc_modules[mod].lock);
	regs = adc_modules[mod].regs;

	if (!adc_modules[mod].enabled) {
		rv = -EACCES;
		goto unlock;
	}

	/* Select ADxxa or ADxxb channels */
	if (req->channel_id & 0x20)
		regs->cfg2 |= KINETIS_ADC_CFG2_ADHSC_MSK;
	else
		regs->cfg2 &= ~KINETIS_ADC_CFG2_ADHSC_MSK;

	/* Start conversion on the requested channel */
	regs->sc1a = req->channel_id & 0x1F;
	/* Wait until conversion is complete */
	while (!(regs->sc1a & KINETIS_ADC_SC1_COCO_MSK));

	/* Read measurement result */
	req->result = regs->ra;

	rv = 0;
unlock:
	spin_unlock(&adc_modules[mod].lock);
out:
	return rv;
}

/*
 * "ioctl()"-based interface to the ADC driver available to user-space programs
 */
static long adc_ioctl(
	struct file *file, unsigned int cmd, unsigned long arg)
{
	long rv = 0;
	struct kinetis_adc_request adc_req;

	switch (cmd) {
	case IOCTL_KINETISADC_ENABLE_ADC:
		/* Enable an ADC module */
		rv = adc_enable_module(arg, 1);
		break;
	case IOCTL_KINETISADC_DISABLE_ADC:
		/* Disable an ADC module */
		rv = adc_enable_module(arg, 0);
		break;
	case IOCTL_KINETISADC_MEASURE:
		/* Copy the request info from user-space */
		rv = copy_from_user(&adc_req, (const void __user *)arg,
			sizeof(struct kinetis_adc_request));
		if (rv != 0) {
			rv = -EFAULT;
			break;
		}

		/* Make a synchronous measurement */
		rv = adc_measure(&adc_req);
		if (rv != 0)
			break;

		/* Copy the updated request info back to user-space */
		rv = copy_to_user((void __user *)arg, &adc_req,
			sizeof(struct kinetis_adc_request));
		if (rv != 0)
			rv = -EFAULT;
		break;
	default:
		rv = -EINVAL;
		break;
	}

	return rv;
}

/*
 * Register bases for ADC modules (ADC0, ADC1, ADC2, ADC3)
 */
const static u32 kinetis_adc_phys_base[N_ADC_MODULES] = {
	0x4003B000, 0x400BB000, 0x4003C000, 0x400BC000
};

static const struct file_operations adc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= adc_ioctl,
};

static struct miscdevice adc_dev = {
	.minor	= KINETIS_ADC_MINOR,
	.name	= "kinetis_adc",
	.fops	= &adc_fops,
};

/*
 * Kernel module initialization function
 */
static int __init kinetis_adc_init(void)
{
	int i;

	for (i = 0; i < N_ADC_MODULES; i++) {
		spin_lock_init(&adc_modules[i].lock);
		adc_modules[i].enabled = 0;
		adc_modules[i].regs = ioremap(kinetis_adc_phys_base[i], SZ_4K);
	}

	return misc_register(&adc_dev);
}
module_init(kinetis_adc_init);

/*
 * Kernel module cleanup function
 */
static void __exit kinetis_adc_exit(void)
{
	int i;

	misc_deregister(&adc_dev);

	for (i = 0; i < N_ADC_MODULES; i++) {
		if (adc_modules[i].regs)
			iounmap(adc_modules[i].regs);
	}
}
module_exit(kinetis_adc_exit);

MODULE_AUTHOR("Alexander Potashev <aspotashev@emcraft.com>");
MODULE_DESCRIPTION("Freescale Kinetis ADC driver");
MODULE_LICENSE("GPL v2");
