/*
 * Copyright (C) 2017 Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * STM32 ADCx driver
 *
 * License terms: GNU General Public License (GPL), version 2
 */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/pwm.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/completion.h>
#include <linux/stm32_adc.h>

#include <mach/clock.h>
#include <mach/dmac.h>
#include <mach/adc.h>

/******************************************************************************
 * Constants local to this module
 ******************************************************************************/
/*
 * ADC driver name
 */
#define STM32_ADC_NAME		"stm32_adc"

/*
 * ADC is running at PBLK2/STM32_ADC_FRQ_DIV (/2, /4, /6, /8)
 */
#define STM32_ADC_FRQ_DIV	2

/*
 * ADC register constants
 */
#define STM32_ADC_SR_OVR	(1 << 5)

#define STM32_ADC_CR1_OVRIE	(1 << 26)
#define STM32_ADC_CR1_SCAN	(1 << 8)

#define STM32_ADC_CR2_EXTEN_RI	(1 << 28)
#define STM32_ADC_CR2_EXTSEL(x)	((x) << 24)
#define STM32_ADC_CR2_DDS	(1 << 9)
#define STM32_ADC_CR2_DMA	(1 << 8)
#define STM32_ADC_CR2_ADON	(1 << 0)

#define STM32_ADC_SMPR(x, p)	((x) << ((p) * 3))

#define STM32_ADC_SQR_L(x)	((x) << 20)
#define STM32_ADC_SQR_SQ(x, p)	((x) << ((p) * 5))

#define STM32_ADC_CCR_TSVREFE	(1 << 23)
#define STM32_ADC_CCR_PRE(x)	(((x >> 1) - 1) << 16)

/******************************************************************************
 * C-types local to this module
 ******************************************************************************/

/*
 * STM32 ADC registers
 */
struct stm32_adc_reg_adcx {
	u32	sr;
	u32	cr[2];
	u32	smpr[2];
	u32	jofr[4];
	u32	htr;
	u32	ltr;
	u32	sqr[3];
	u32	jsqr;
	u32	jdr[4];
	u32	dr;
	u32	_rsv[44];
} __attribute__((packed));

struct stm32_adc_reg_common {
	u32	csr;
	u32	ccr;
	u32	cdr;
} __attribute__((packed));

struct stm32_adc_regs {
	struct stm32_adc_reg_adcx	adc[STM32_ADC_NUM];
	struct stm32_adc_reg_common	com;
} __attribute__((packed));

/*
 * STM32 ADC device descriptor
 */
struct stm32_adc_dsc {
	char			name[32];	/* Device name		      */
	struct device		*dev;		/* Pointer to device instance */
	struct cdev		cdev;		/* Char device		      */

	unsigned int		regs_base;	/* Register base (phys)	      */
	unsigned int		regs_size;	/* Register area size	      */
	struct stm32_adc_regs __iomem *regs;	/* Register base (virt)	      */

	struct completion	eve;		/* Completion event	      */
	int			irq;		/* IRQ#			      */
	int			dma;		/* DMA channel		      */
	int			trig;		/* Trigger		      */
	int			loop;		/* Continuous mode	      */
	int			run;		/* Running state	      */
	int			stop;		/* Stop command		      */

	int			id;		/* ADC ID		      */
	int			fd;		/* Device ID		      */

	int			seq[STM32_ADC_SEQ_MAX];	/* Channel sequence   */
	enum stm32_adc_smp	smp[STM32_ADC_SEQ_MAX];	/* Sampling times     */
	int			chan;		/* Size of sequence	      */
	int			meas;		/* Number of measurements     */

	struct stm32_adc_mmap	*mmap;		/* Mmaped buffer	      */
	dma_addr_t		dma_mmap;	/* DMA buffer address	      */
	u32			mmap_size;	/* Size of mmaped buf	      */
};

/******************************************************************************
 * Variables local to this module
 ******************************************************************************/

static struct class *stm32_adc_class;
static int stm32_adc_major;

static struct stm32_adc_dsc stm32_adc_dsc[STM32_ADC_NUM];
static int stm32_adc_smp_val[STM32_ADC_SMP_NUM] = {
	3, 15, 28, 56, 85, 112, 144, 480
};

/******************************************************************************
 * Hw control routines
 ******************************************************************************/

static void stm32_adc_dump_regs(const char *who, struct stm32_adc_dsc *dsc)
{
	struct stm32_adc_reg_adcx __iomem *regs = &dsc->regs->adc[dsc->id];

	printk("%s ADC%d@%p:SR=%02x;CR=%08x.%08x;SQR=%08x.%08x.%08x;CCR=%08x\n",
		who, dsc->id + 1, regs,
		readl(&regs->sr), readl(&regs->cr[0]), readl(&regs->cr[1]),
		readl(&regs->sqr[0]), readl(&regs->sqr[1]),
		readl(&regs->sqr[2]),
		readl(&dsc->regs->com.ccr));
}

static int stm32_adc_configure(struct stm32_adc_dsc *dsc, int cont)
{
	struct stm32_adc_reg_adcx __iomem *regs = &dsc->regs->adc[dsc->id];
	unsigned long flags;
	u32 val;
	int imax[] = { 16, 12, 6 };
	int i, k, s, rv;

	local_irq_save(flags);

	if (dsc->run) {
		rv = -EBUSY;
		goto out;
	}

	/*
	 * Program, and enable DMA
	 */
	rv  = stm32_dma_ch_init(dsc->dma, DMA_DIR_DEV_TO_MEM,
		DMA_FLOW_CONTROL_DISABLE, DMA_PRIORITY_VERY_HIGH,
		DMA_DOUBLE_BUFFER_DISABLE, cont ? DMA_CIRCULAR_MODE_ENABLE :
						  DMA_CIRCULAR_MODE_DISABLE);
	rv += stm32_dma_ch_init_fifo(dsc->dma, DMA_BURST_SINGLE,
		DMA_THRESH_1_4_FIFO);
	rv += stm32_dma_ch_set_periph(dsc->dma, (u32)&regs->dr,
		DMA_INCREMENT_DISABLE, DMA_BITWIDTH_32, DMA_BURST_SINGLE);
	rv += stm32_dma_ch_set_memory(dsc->dma, (u32)dsc->mmap->data,
		DMA_INCREMENT_ENABLE, DMA_BITWIDTH_32, DMA_BURST_SINGLE);
	rv += stm32_dma_ch_set_nitems(dsc->dma, dsc->meas * dsc->chan);
	if (rv || stm32_dma_ch_enable(dsc->dma)) {
		pr_err("%s: dma configuration error\n", __func__);
		stm32_adc_dump_regs(__func__, dsc);
		rv = -EFAULT;
		goto out;
	}

	/*
	 * Program sequence in SQRx registers
	 */
	for (s = 2, i = 0; s >= 0; s--) {
		writel(0, &regs->sqr[s]);
		for (k = 0; i < imax[s] && i < dsc->chan; i++, k++) {
			val = readl(&regs->sqr[s]);
			val |= STM32_ADC_SQR_SQ(dsc->seq[i], k);
			writel(val, &regs->sqr[s]);
		}
	}
	val = readl(&regs->sqr[0]);
	val |= STM32_ADC_SQR_L(dsc->chan - 1);
	writel(val, &regs->sqr[0]);

	/*
	 * Set SMPR
	 */
	writel(0, &regs->smpr[0]);
	writel(0, &regs->smpr[1]);
	for (i = 0; i < dsc->chan; i++) {
		s = dsc->seq[i] / 10;
		k = dsc->seq[i] % 10;

		val = readl(&regs->smpr[s]);
		val |= STM32_ADC_SMPR(dsc->smp[i], k);
		writel(val, &regs->smpr[s]);
	}

	/*
	 * Program controls, and run ADC
	 */
	writel(STM32_ADC_CR1_OVRIE | STM32_ADC_CR1_SCAN, &regs->cr[0]);
	writel(STM32_ADC_CR2_EXTEN_RI | STM32_ADC_CR2_EXTSEL(dsc->trig) |
		STM32_ADC_CR2_DDS | STM32_ADC_CR2_DMA | STM32_ADC_CR2_ADON,
		&regs->cr[1]);

	dsc->loop = cont;
	dsc->stop = 0;
	dsc->run = 1;

	rv = 0;
out:
	local_irq_restore(flags);

	return rv;
}

static irqreturn_t stm32_adc_irq_handler(int irq, void *ptr)
{
	struct stm32_adc_dsc *dsc;
	irqreturn_t rv = IRQ_NONE;
	u32 sr;
	int i;

	for (i = 0; i < STM32_ADC_NUM; i++) {
		dsc = &stm32_adc_dsc[i];
		if (!dsc->dev)
			continue;

		sr = readl(&dsc->regs->adc[dsc->id].sr);
		if (!(sr & STM32_ADC_SR_OVR))
			continue;
		pr_warning("%s: overrun\n", dsc->name);

		sr &= ~STM32_ADC_SR_OVR;
		writel(sr, &dsc->regs->adc[dsc->id].sr);

		rv = IRQ_HANDLED;
	}

	return rv;
}

static void stm32_adc_dma_handler(int ch, unsigned long flags, void *data)
{
	struct stm32_adc_dsc *dsc = data;

	if (!dsc->loop || dsc->stop) {
		writel(0, &dsc->regs->adc[dsc->id].cr[1]);
		stm32_dma_ch_disable(dsc->dma);
		dsc->run = 0;
	}

	complete_all(&dsc->eve);
}

/******************************************************************************
 * Char device file operations
 ******************************************************************************/

static int stm32_adc_fops_open(struct inode *inode, struct file *file)
{
	int rv, adc = iminor(inode);

	if (adc < 0 || adc >= STM32_ADC_NUM || !stm32_adc_dsc[adc].dev) {
		rv = -ENODEV;
		goto out;
	}

	file->private_data = &stm32_adc_dsc[adc];
	rv = 0;
out:
	return rv;
}

static ssize_t stm32_adc_fops_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct stm32_adc_dsc *dsc = file->private_data;
	unsigned int *data;
	char str[32];
	int i, k, n, pos, rv;

	/*
	 * If not running, then configure, start, and wait for complete
	 * If already running, then just print current values
	 */
	if (!dsc->run) {
		init_completion(&dsc->eve);
		rv = stm32_adc_configure(dsc, 0);
		if (rv)
			goto out;

		/*
		 * Wait for completion
		 */
		rv = wait_for_completion_interruptible(&dsc->eve);
		if (rv)
			goto out;
	}

	/*
	 * Print out current values
	 */
	rv = -EFAULT;
	pos = 0;

	for (k = 0; k < dsc->chan; k++) {
		n = sprintf(str, "%s<%3u>%s",
			k != 0 ? "" : "       ", dsc->seq[k],
			k != dsc->chan - 1 ? "," : "\n");

		if (pos + n > count || copy_to_user(&buf[pos], str, n))
			goto out;
		pos += n;
	}

	data = (void *)dsc->mmap->data;
	for (i = 0; i < dsc->meas; i++) {
		n = sprintf(str, "[%3u]: ", i + 1);
		if (pos + n > count || copy_to_user(&buf[pos], str, n))
			goto out;
		pos += n;

		for (k = 0; k < dsc->chan; k++) {
			n = sprintf(str, "%5u%s", *data++,
				k != dsc->chan - 1 ? "," : "\n");
			if (i == dsc->meas - 1 && k == dsc->chan - 1)
				n += 1;

			if (pos + n > count || copy_to_user(&buf[pos], str, n))
				goto out;
			pos += n;
		}
	}

	rv = pos;
out:
	if (rv < 0)
		pr_err("%s: error %d\n", __func__, rv);

	return rv;
}

static unsigned long stm32_adc_fops_get_unmapped_area(struct file *file,
	unsigned long addr, unsigned long len, unsigned long pgoff,
	unsigned long flags)
{
	struct stm32_adc_dsc *dsc = file->private_data;

	return (unsigned long)dsc->mmap->data;
}

static int stm32_adc_fops_mmap(struct file *file, struct vm_area_struct *vma)
{
	return vma->vm_flags & VM_SHARED ? 0 : -EACCES;;
}

static long stm32_adc_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct stm32_adc_dsc *dsc = file->private_data;
	int rv;

	/*
	 * On any success IOCTL below we wait until measurement completes
	 * before return
	 */
	init_completion(&dsc->eve);

	switch (cmd) {
	case STM32_ADC_SINGLE_SEQ:
	case STM32_ADC_LOOP_SEQ:
		/*
		 * If ADC is already running, then just wait for next measure;
		 * otherwise run in a single/loop mode.
		 */
		if (dsc->run)
			break;
		rv = stm32_adc_configure(dsc,
					 cmd == STM32_ADC_LOOP_SEQ ? 1 : 0);
		if (rv)
			goto out;
		break;
	case STM32_ADC_STOP_SEQ:
		if (!dsc->run) {
			rv = 0;
			goto out;
		}
		dsc->stop = 1;
		break;
	default:
		rv = EINVAL;
		goto out;
	}

	rv = wait_for_completion_interruptible(&dsc->eve);
out:
	return rv;
}

static const struct file_operations stm32_adc_fops = {
	.owner			= THIS_MODULE,
	.open			= stm32_adc_fops_open,
	.read			= stm32_adc_fops_read,
	.get_unmapped_area	= stm32_adc_fops_get_unmapped_area,
	.mmap			= stm32_adc_fops_mmap,
	.unlocked_ioctl		= stm32_adc_fops_ioctl,
};

/******************************************************************************
 * Platform driver interface
 ******************************************************************************/

static int stm32_adc_plat_probe(struct platform_device *pdev)
{
	unsigned long frq, smp_sum, smp_ns, pwm_ns, val;
	struct stm32_adc_platform_data *pdat;
	struct stm32_adc_regs __iomem *regs;
	struct resource *mem, *irq, *dma;
	struct stm32_adc_dsc *dsc;
	struct pwm_device *pwm;
	int i, n, inum, rv;

	/*
	 * Check if PWM configuration is OK
	 */
	pdat = pdev->dev.platform_data;
	pwm = pwm_request(pdat->trig_pwm_id, NULL);
	if (!pwm) {
		pr_err("%s: unable to get pwm%d\n", __func__,
			pdat->trig_pwm_id);
		rv = -ENODEV;
		goto out;
	}
	pwm_ns = pdat->trig_usec * 1000;

	frq = stm32_clock_get(CLOCK_PCLK2) / STM32_ADC_FRQ_DIV;
	smp_ns = 1000000000 / frq;

	/*
	 * Configure ADC
	 */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		pr_err("%s: unable to get mem resource\n", __func__);
		rv = -ENODEV;
		goto err_mem;
	}
	if (!request_mem_region(mem->start, resource_size(mem),
				STM32_ADC_NAME)) {
		pr_err("%s: unable to request mem region\n", __func__);
		rv = -EBUSY;
		goto err_mem;
	}
	regs = ioremap_nocache(mem->start, resource_size(mem));
	if (!regs) {
		pr_err("%s: unable to map regs\n", __func__);
		rv = -EFAULT;
		goto err_ioremap;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		pr_err("%s: unable to get irq resource\n", __func__);
		rv = -ENODEV;
		goto err_irq;
	}
	rv = request_irq(irq->start, stm32_adc_irq_handler, 0, STM32_ADC_NAME,
			regs);
	if (rv) {
		pr_err("%s: unable to request irq\n", __func__);
		goto err_irq;
	}

	for (i = 0, inum = 0; i < STM32_ADC_NUM; i++) {
		if (!pdat->adc[i].meas)
			continue;

		dsc = &stm32_adc_dsc[i];
		inum++;

		n = snprintf(dsc->name, sizeof(dsc->name), "adc%d", i + 1);
		if (n >= sizeof(dsc->name)) {
			pr_err("%s: unable to name %s.%d\n", __func__,
				STM32_ADC_NAME, i);
			goto err_dev;
		}

		dma = platform_get_resource(pdev, IORESOURCE_DMA, i);
		if (!dma) {
			pr_err("%s: unable to get dma%d resource\n", __func__,
				i);
			goto err_dev;
		}
		dsc->dma = dma->start;

		dsc->id = i;
		dsc->fd = MKDEV(stm32_adc_major, i);
		dsc->trig = pdat->trig_adc_code;
		dsc->regs_base = mem->start;
		dsc->regs_size = resource_size(mem);
		dsc->regs = regs;
		dsc->irq = irq->start;
		init_completion(&dsc->eve);

		dsc->meas = pdat->adc[i].meas;
		dsc->chan = pdat->adc[i].chan;
		if (dsc->chan > STM32_ADC_SEQ_MAX)
			dsc->chan = STM32_ADC_SEQ_MAX;
		for (n = 0, smp_sum = 0; n < dsc->chan; n++) {
			dsc->seq[n] = pdat->adc[i].seq[n];
			dsc->smp[n] = pdat->smp;
			smp_sum += stm32_adc_smp_val[dsc->smp[n]];
		}
		if (2 * smp_sum * smp_ns >= pwm_ns) {
			pr_err("%s: ADC%d sampling time (2x%ldx%ldns) is more "
				"than PWM trigger rate (%ldns)\n", __func__,
				i + 1, smp_sum, smp_ns, pwm_ns);
		}

		dsc->dev = device_create(stm32_adc_class, NULL, dsc->fd, NULL,
					dsc->name);
		if (!dsc->dev) {
			pr_err("%s: unable to create device '%s'\n", __func__,
				dsc->name);
			rv = -ENODEV;
			goto err_dev;
		}

		dsc->mmap_size = dsc->meas * dsc->chan *
				 sizeof(dsc->mmap->data[0]);
		dsc->mmap = dma_alloc_coherent(dsc->dev, dsc->mmap_size,
					&dsc->dma_mmap, GFP_KERNEL);
		if (!dsc->mmap) {
			pr_err("%s: unabled to alloca dmamem\n", __func__);
			rv = -ENOMEM;
			goto err_dev;
		}
		memset(dsc->mmap, 0, dsc->mmap_size);

		rv = stm32_dma_ch_get(dsc->dma);
		if (rv) {
			pr_err("%s: unable to get DMA channel\n", __func__);
			dsc->dma = -ENODEV;
			rv = -ENODEV;
			goto err_dev;
		}
		rv = stm32_dma_ch_request_irq(dsc->dma, stm32_adc_dma_handler,
				STM32_DMA_INTCOMPLETE, dsc);
		if (rv) {
			pr_err("%s: unable to reques DMA irq\n", __func__);
			rv = -ENODEV;
			goto err_dev;
		}

		cdev_init(&dsc->cdev, &stm32_adc_fops);
		rv = cdev_add(&dsc->cdev, dsc->fd, 1);
		if (rv < 0) {
			pr_err("%s: unabled to add device '%s' (%d)\n",
				__func__, dsc->name, rv);
			goto err_dev;
		}

		dev_set_drvdata(dsc->dev, dsc);

		pr_info("STM32 ADC%d run @%ld.%ldMHz/%dsmp/%dchan; "
			"char device {%d,%d}\n",
			i + 1, frq / 1000000, (frq / 100000) % 10,
			stm32_adc_smp_val[pdat->smp],
			dsc->chan, stm32_adc_major, i);
	}
	if (!inum) {
		rv = -ENODEV;
		goto err_dev;
	}

	/*
	 * Configure common settings
	 */
	val = STM32_ADC_CCR_TSVREFE | STM32_ADC_CCR_PRE(STM32_ADC_FRQ_DIV);
	writel(val, &regs->com.ccr);

	/*
	 * Run PWM
	 */
	rv = pwm_config(pwm, pwm_ns, pwm_ns / 2);
	if (rv) {
		pr_err("%s: unable to config(%ld,%ld) pwm%d\n", __func__,
			pwm_ns, pwm_ns / 2, pdat->trig_pwm_id);
		goto err_dev;
	}
	rv = pwm_enable(pwm);
	if (rv) {
		pr_err("%s: unable to enable pwm%d\n", __func__,
			pdat->trig_pwm_id);
		goto err_dev;
	}

	rv =  0;
	goto out;
err_dev:
	for (i = 0; i < STM32_ADC_NUM; i++) {
		dsc = &stm32_adc_dsc[i];
		if (!dsc->dev)
			continue;

		dev_set_drvdata(dsc->dev, NULL);
		cdev_del(&dsc->cdev);

		if (dsc->mmap) {
			dma_free_coherent(dsc->dev, dsc->mmap_size,
					dsc->mmap, dsc->dma_mmap);
		}
		device_destroy(stm32_adc_class, dsc->fd);

		stm32_dma_ch_free_irq(dsc->dma, dsc);
		stm32_dma_ch_put(dsc->dma);
		memset(dsc, 0, sizeof(struct stm32_adc_dsc));

		pr_info("STM32 ADC%d removed\n", i + 1);
	}

	free_irq(irq->start, regs);
err_irq:
	iounmap(regs);
err_ioremap:
	release_mem_region(mem->start, resource_size(mem));
err_mem:
out:
	return rv;
}

static int stm32_adc_plat_remove(struct platform_device *pdev)
{
	struct stm32_adc_dsc *dsc;
	int i, freed = 0;

	for (i = 0; i < STM32_ADC_NUM; i++) {
		dsc = &stm32_adc_dsc[i];
		if (!dsc->dev)
			continue;

		dev_set_drvdata(dsc->dev, NULL);
		cdev_del(&dsc->cdev);

		if (dsc->mmap) {
			dma_free_coherent(dsc->dev, dsc->mmap_size,
					dsc->mmap, dsc->dma_mmap);
		}
		device_destroy(stm32_adc_class, dsc->fd);

		if (!freed) {
			free_irq(dsc->irq, dsc->regs);
			iounmap(dsc->regs);
			release_mem_region(dsc->regs_base, dsc->regs_size);
			freed = 1;
		}

		stm32_dma_ch_free_irq(dsc->dma, dsc);
		stm32_dma_ch_put(dsc->dma);
		memset(dsc, 0, sizeof(struct stm32_adc_dsc));
	}

	return 0;
}

static struct platform_driver stm32_adc_plat = {
	.probe	= stm32_adc_plat_probe,
	.remove	= stm32_adc_plat_remove,
	.driver	= {
		.name	= "stm32_adc",
		.owner	= THIS_MODULE,
	},
};

/******************************************************************************
 * Kernel module interface
 ******************************************************************************/

static int __init stm32_adc_drv_init(void)
{
	dev_t dev;
	int rv;

	stm32_adc_class = class_create(THIS_MODULE, STM32_ADC_NAME);
	if (IS_ERR(stm32_adc_class)) {
		rv = PTR_ERR(stm32_adc_class);
		pr_err("%s: unable to register class (%d)\n", __func__, rv);
		goto out;
	}

	rv = alloc_chrdev_region(&dev, 0, STM32_ADC_NUM, STM32_ADC_NAME);
	if (rv) {
		pr_err("%s: can't register chrdev region (%d)\n", __func__, rv);
		goto err_class;
	}
	stm32_adc_major = MAJOR(dev);

	rv = platform_driver_register(&stm32_adc_plat);
	if (rv) {
		pr_err("%s: can't register plat driver (%d)\n", __func__, rv);
		goto err_chrdev;
	}

	rv = 0;
	goto out;

err_chrdev:
	unregister_chrdev_region(dev, STM32_ADC_NUM);
err_class:
	class_destroy(stm32_adc_class);
	stm32_adc_class = NULL;
out:
	return rv;
}

static void __exit stm32_adc_drv_exit(void)
{
	platform_driver_unregister(&stm32_adc_plat);
	unregister_chrdev_region(MKDEV(stm32_adc_major,0), STM32_ADC_NUM);
	class_destroy(stm32_adc_class);
	stm32_adc_class = NULL;
}

module_init(stm32_adc_drv_init);
module_exit(stm32_adc_drv_exit);

MODULE_AUTHOR("Yuri Tikhonov <yur@emcraft.com>");
MODULE_DESCRIPTION("STM32 ADC driver");
MODULE_LICENSE("GPL");
