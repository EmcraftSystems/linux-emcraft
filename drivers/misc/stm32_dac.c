/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmamem.h>
#include <linux/uaccess.h>
#include <linux/stm32_dac.h>

#include <mach/dac.h>
#include <mach/dmac.h>
#include <mach/clock.h>
#include <mach/dmaregs.h>

#define MAX_NR_CHANNELS			2

#define STM32_CR__DMAUDRIE		BIT(13)
#define STM32_CR__DMAEN			BIT(12)
#define STM32_CR__MAMP(x)		(((x) & 0xf) << 8)
#define STM32_CR__WAVE(x)		(((x) & 0x3) << 6)
#define STM32_CR__TSEL(x)		(((x) & 0x7) << 3)
#define STM32_CR__TEN			BIT(2)
#define STM32_CR__BOFF			BIT(1)
#define STM32_CR__EN			BIT(0)

#define STM32_SWTRIGR__SWTRIG2		BIT(1)
#define STM32_SWTRIGR__SWTRIG1		BIT(0)

#define STM32_TSEL__TIM6		0
#define STM32_TSEL__TIM8		1
#define STM32_TSEL__TIM7		2
#define STM32_TSEL__TIM5		3
#define STM32_TSEL__TIM2		4
#define STM32_TSEL__TIM4		5
#define STM32_TSEL__EXT_LINE9		6
#define STM32_TSEL__SW			7

#define STM32_WAVE__DISABLED		0
#define STM32_WAVE__NOISE		1
#define STM32_WAVE__TRIANGLE		2

struct stm32_dac_regs {
	u32	cr;
	u32	swtrigr;
	u32	dhr12r1;
	u32	dhr12l1;
	u32	dhr8r1;
	u32	dhr12r2;
	u32	dhr12l2;
	u32	dhr8r2;
	u32	dhr12rd;
	u32	dhr12ld;
	u32	dhr8rd;
	u32	dor1;
	u32	dor2;
	u32	sr;
};

struct stm32_dac_channel {
	bool active;
	struct device *dev;
	int timer_id;
	struct resource *dma_res;
	int dma_chan;
	unsigned char *buf;
	dma_addr_t dma_buf;
	struct mutex lock;
	size_t buf_len;
	void __iomem *hold_reg;
	void __iomem *in_reg;
};

struct stm32_dac {
	struct device *dev;
	struct stm32_dac_regs __iomem *regs;
	struct cdev cdev;
	struct stm32_dac_channel channels[MAX_NR_CHANNELS];
};

struct stm32_dac_vdev {
	int channel;
	struct stm32_dac *dac;
};

static struct class *stm32_dac_class;
static dev_t stm32_dac_devt;
static int stm32_dac_major;
static int stm32_dac_tsel_table[] = {
	-1,
	-1,
	STM32_TSEL__TIM2,
	-1,
	STM32_TSEL__TIM4,
	STM32_TSEL__TIM5,
	STM32_TSEL__TIM6,
	STM32_TSEL__TIM7,
	STM32_TSEL__TIM8,
	-1,
	-1,
	-1,
	-1,
	-1,
};
static u32 stm32_dac_cr_shift[MAX_NR_CHANNELS] = {
	0,
	16,
};
static u32 stm32_dac_hold_regs[MAX_NR_CHANNELS] = {
	offsetof(struct stm32_dac_regs, dhr12r1),
	offsetof(struct stm32_dac_regs, dhr12r2),
};
static u32 stm32_dac_in_regs[MAX_NR_CHANNELS] = {
	offsetof(struct stm32_dac_regs, dor1),
	offsetof(struct stm32_dac_regs, dor2),
};

static int stm32_dac_simple(struct stm32_dac *dac, int channel, unsigned value)
{
	struct stm32_dac_channel *dac_ch = &dac->channels[channel];
	u32 cr_shift = stm32_dac_cr_shift[channel];
	u32 orig_cr = readl(&dac->regs->cr);
	u32 cr = orig_cr;

	if (value > STM32_DAC_MAX_VALUE) {
		dev_err(dac->dev, "%s: channel %d, value %u must not be greater than %d\n",
			__func__, channel, value, STM32_DAC_MAX_VALUE);
		return -EINVAL;
	}

	dev_dbg(dac->dev, "%s: channel %d, value %u\n", __func__, channel, value);

	if (dac_ch->timer_id) {
		stm32_dma_ch_disable(dac_ch->dma_chan);
		stm32_dma_ch_put(dac_ch->dma_chan);
	}

	cr &= ~(0xffff << cr_shift);
	cr |= STM32_CR__EN << cr_shift;

	writel(cr, &dac->regs->cr);
	writel(value, dac_ch->hold_reg);

	return 0;
}

static int stm32_dac_pwm(struct stm32_dac_vdev *vdac, bool loop, size_t len)
{
	struct stm32_dac *dac = vdac->dac;
	int channel = vdac->channel;
	struct stm32_dac_channel *dac_ch = &dac->channels[channel];
	u32 cr_shift = stm32_dac_cr_shift[channel];
	u32 orig_cr = readl(&dac->regs->cr);
	u32 cr = orig_cr;
	int err;

	dev_dbg(dac->dev, "%s: channel %d, loop %d, len %zu\n", __func__, channel, loop, len);

	stm32_dma_ch_disable(dac_ch->dma_chan);
	stm32_dma_ch_put(dac_ch->dma_chan);

	cr &= ~(0xffff << cr_shift);
	cr |= (STM32_CR__EN | STM32_CR__DMAEN) << cr_shift;
	cr |= STM32_CR__TSEL(stm32_dac_tsel_table[dac_ch->timer_id]) << cr_shift;
	writel(cr, &dac->regs->cr);
	cr |= STM32_CR__TEN << cr_shift;

	dac_ch->buf_len = len;
	dma_sync_single_for_device(dac->dev, dac_ch->dma_buf, len, DMA_TO_DEVICE);

	err = stm32_dma_ch_get(dac_ch->dma_chan);
	if (err) {
		dev_err(dac->dev, "%s[%d]: dma request failed: %d\n", __func__, __LINE__, err);
		goto fail;
	}

	err = stm32_dma_ch_init(dac_ch->dma_chan,
		DMA_DIR_MEM_TO_DEV,
		DMA_FLOW_CONTROL_DISABLE,
		DMA_PRIORITY_LOW,
		DMA_DOUBLE_BUFFER_DISABLE,
		loop ? DMA_CIRCULAR_MODE_ENABLE : DMA_CIRCULAR_MODE_DISABLE);
	if (err) {
		dev_err(dac->dev, "%s[%d]: dma channel init failed: %d\n", __func__, __LINE__, err);
		goto fail;
	}

	err = stm32_dma_ch_init_fifo(dac_ch->dma_chan,
		DMA_DIRECT_MODE_ENABLE,
		DMA_THRESH_FULL_FIFO);
	if (err) {
		dev_err(dac->dev, "%s[%d]: dma channel setup failed: %d\n", __func__, __LINE__, err);
		goto fail;
	}

	err = stm32_dma_ch_set_periph(dac_ch->dma_chan,
				      (uint32_t)dac_ch->hold_reg,
				      DMA_INCREMENT_DISABLE,
				      DMA_BITWIDTH_16,
				      DMA_BURST_SINGLE);
	if (err) {
		dev_err(dac->dev, "%s[%d]: dma channel setup failed: %d\n", __func__, __LINE__, err);
		goto fail;
	}

	err = stm32_dma_ch_set_memory(dac_ch->dma_chan,
				      (u32)dac_ch->dma_buf,
				      DMA_INCREMENT_ENABLE,
				      DMA_BITWIDTH_16,
				      DMA_BURST_SINGLE);
	if (err) {
		dev_err(dac->dev, "%s[%d]: dma channel setup failed: %d\n", __func__, __LINE__, err);
		goto fail;
	}

	err = stm32_dma_ch_set_nitems(dac_ch->dma_chan, len / 2);
	if (err) {
		dev_err(dac->dev, "%s[%d]: failed: %d\n", __func__, __LINE__, err);
		goto fail;
	}

	err = stm32_dma_ch_enable(dac_ch->dma_chan);
	if (err) {
		dev_err(dac->dev, "%s[%d]: unable to enable dma: %d\n", __func__, __LINE__, err);
		goto fail;
	}

	writel(cr, &dac->regs->cr);

	dev_dbg(dac->dev, "%s: cr 0x%x -> 0x%x\n", __func__, orig_cr, cr);

	return 0;

fail:
	dev_dbg(dac->dev, "%s: failed: %d\n", __func__, err);
	return err;
}

static int stm32_dac_fop_open(struct inode *inode, struct file *file)
{
	int channel = iminor(inode);
	struct stm32_dac *dac = container_of(inode->i_cdev, struct stm32_dac, cdev);
	struct stm32_dac_vdev *vdac = devm_kzalloc(dac->dev, sizeof(*vdac), GFP_KERNEL);

	if (!vdac)
		return -ENOMEM;

	vdac->dac = dac;
	vdac->channel = channel;
	file->private_data = vdac;

	return 0;
}

static int stm32_dac_fop_release(struct inode *inode, struct file *file)
{
	struct stm32_dac_vdev *vdac = file->private_data;

	devm_kfree(vdac->dac->dev, vdac);
	return 0;
}

static ssize_t stm32_dac_fop_write(struct file *file, const char __user *buf,
			       size_t len, loff_t *off)
{
	struct stm32_dac_vdev *vdac = file->private_data;
	struct stm32_dac_channel *dac_ch = &vdac->dac->channels[vdac->channel];
	char data[16];
	unsigned value;
	int err;

	mutex_lock(&dac_ch->lock);

	err = copy_from_user(data, buf, min(sizeof(data), len));
	if (err)
		goto fail;

	if (1 != sscanf(data, "%u", &value)) {
		err = -EINVAL;
		goto fail;
	}

	err = stm32_dac_simple(vdac->dac, vdac->channel, value);
	if (err)
		goto fail;

	mutex_unlock(&dac_ch->lock);
	return len;

fail:
	mutex_unlock(&dac_ch->lock);
	dev_err(vdac->dac->dev, "%s: failed\n", __func__);
	return err;
}

static ssize_t stm32_dac_fop_read(struct file * file, char __user * buf,
				  size_t count, loff_t *ppos)
{
	struct stm32_dac_vdev *vdac = file->private_data;
	struct stm32_dac_channel *dac_ch = &vdac->dac->channels[vdac->channel];
	char data[16];
	ssize_t len;
	u32 value;
	int err;

	value = readl(dac_ch->in_reg);
	sprintf(data, "%u\n", value);
	len = strlen(data) + 1;

	if (count < len)
		return -EINVAL;

	err = copy_to_user(buf, data, len);
	if (err)
		return -EFAULT;

	return len;
}

static long stm32_dac_fop_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	struct stm32_dac_vdev *vdac = file->private_data;
	struct stm32_dac_channel *dac_ch = &vdac->dac->channels[vdac->channel];
	struct device *dev = vdac->dac->dev;
	size_t len = arg;
	int err;

	if (!dac_ch->timer_id) {
		dev_err(dev, "%s: channel %d: not supported without timer\n", __func__, vdac->channel);
		return -EOPNOTSUPP;
	}

	mutex_lock(&dac_ch->lock);

	switch (cmd) {
	case STM32_DAC_SINGLE_SEQ:
		err = stm32_dac_pwm(vdac, false, len);
		break;

	case STM32_DAC_LOOP_SEQ:
		if (len & 1) {
			dev_err(dev, "%s: lenght (%zu) must be even\n", __func__, len);
			err = -EINVAL;
		} else {
			err = stm32_dac_pwm(vdac, true, len);
		}
		break;

	default:
		dev_err(dev, "%s: channel %d: invalid cmd 0x%x\n", __func__, vdac->channel, cmd);
		err = -EINVAL;
	}

	mutex_unlock(&dac_ch->lock);

	return err;
}

static unsigned long stm32_dac_fop_get_unmapped_area(struct file *file,
						     unsigned long addr, unsigned long len, unsigned long pgoff,
						     unsigned long flags)
{
	struct stm32_dac_vdev *vdac = file->private_data;
	struct stm32_dac_channel *dac_ch = &vdac->dac->channels[vdac->channel];

	if (!dac_ch->timer_id) {
		dev_err(vdac->dac->dev, "%s: channel %d: not supported without assigned timer\n",
			__func__, vdac->channel);
		return 0;
	}

	if (len > STM32_DAC_MAX_BUF_LEN) {
		dev_err(vdac->dac->dev, "%s: too large buffer is requested: %lu (max is %d)\n",
			__func__, len, STM32_DAC_MAX_BUF_LEN);
		return 0;
	}

	return (unsigned long)dac_ch->buf;
}

static int stm32_dac_fop_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct stm32_dac_vdev *vdac = file->private_data;
	struct stm32_dac_channel *dac_ch = &vdac->dac->channels[vdac->channel];

	if (!dac_ch->timer_id) {
		dev_err(vdac->dac->dev, "%s: channel %d: not supported without assigned timer\n",
			__func__, vdac->channel);
		return -EOPNOTSUPP;
	}

	return vma->vm_flags & VM_SHARED ? 0 : -EACCES;
}

static const struct file_operations stm32_dac_file_ops = {
	.owner			= THIS_MODULE,
	.open			= stm32_dac_fop_open,
	.release		= stm32_dac_fop_release,
	.write			= stm32_dac_fop_write,
	.read			= stm32_dac_fop_read,
	.unlocked_ioctl		= stm32_dac_fop_ioctl,
	.get_unmapped_area	= stm32_dac_fop_get_unmapped_area,
	.mmap			= stm32_dac_fop_mmap,
};

static int stm32_dac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct stm32_dac *dac;
	struct resource *res;
	int timer_id = STM32_GET_TIMER_ID(pdev->id);
	int nr_active = 0;
	int err;
	int i;

	dac = devm_kzalloc(&pdev->dev, sizeof(*dac), GFP_KERNEL);
	if (!dac)
		return -ENOMEM;
	dac->dev = dev;

	if (pdev->id & STM32_DAC_ID_1) {
		dac->channels[0].active = true;
		dac->channels[0].dma_res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "dac1_dma_tx_channel");
		++nr_active;
	}
	if (pdev->id & STM32_DAC_ID_2) {
		dac->channels[1].active = true;
		dac->channels[1].dma_res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "dac2_dma_tx_channel");
		++nr_active;
	}

	if (!nr_active) {
		dev_warn(dev, "No dac devices defined\n");
		return 0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dac->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(dac->regs)) {
		dev_err(dac->dev, "unable to map registers\n");
		err = PTR_ERR(dac->regs);
		goto fail;
	}

	cdev_init(&dac->cdev, &stm32_dac_file_ops);
	dac->cdev.owner = THIS_MODULE;
	err = cdev_add(&dac->cdev, MKDEV(stm32_dac_major, 0), MAX_NR_CHANNELS);
	if (err) {
		dev_err(dac->dev, "unable to add cdev: %d\n", err);
		goto fail;
	}

	for (i = 0; i < MAX_NR_CHANNELS; ++i) {
		int dac_id = i;
		struct stm32_dac_channel *ch = &dac->channels[dac_id];

		if (!ch->active)
			break;

		ch->hold_reg = ((unsigned char __iomem*)dac->regs) + stm32_dac_hold_regs[dac_id];
		ch->in_reg = ((unsigned char __iomem*)dac->regs) + stm32_dac_in_regs[dac_id];
		mutex_init(&ch->lock);

		if (ch->dma_res) {
			ch->timer_id = timer_id;
			ch->dma_chan = ch->dma_res->start;
			if (stm32_dma_ch_get(ch->dma_chan) != 0) {
				err = -ENODEV;
				dev_err(dev, "dma request failed\n");
				goto fail_dacx;
			}

			ch->buf = dma_alloc_coherent(dev, STM32_DAC_MAX_BUF_LEN, &ch->dma_buf, GFP_KERNEL);
			if (!ch->buf) {
				dev_err(dev, "unable to allocate buffer for dma\n");
				err = -ENOMEM;
				goto fail_dacx;
			}
			dev_info(dev, "dac%d: assigned to timer %d\n", i, ch->timer_id);
		} else {
			dev_warn(dev, "dac%d: no timer assigned, DMA functionality is not available\n", i);
		}

		ch->dev = device_create(stm32_dac_class, dev,
					MKDEV(stm32_dac_major, dac_id), NULL,
					"dac%u", dac_id);
		if (IS_ERR(ch->dev)) {
			err = PTR_ERR(ch->dev);
			dev_err(dev, "dac%d: unable to create device: %d\n", i, err);
			goto fail_dacx;
		}
	}

	dev_info(dev, "created %d character devices with MAJOR %d\n", nr_active, stm32_dac_major);
	return 0;

fail_dacx:
	for (i = 0; i < MAX_NR_CHANNELS; ++i) {
		struct stm32_dac_channel *ch = &dac->channels[i];
		if (ch->active) {
			if (!IS_ERR_OR_NULL(ch->dev))
				device_destroy(stm32_dac_class, MKDEV(stm32_dac_major, i));
			if (ch->buf)
				dma_free_coherent(dev, STM32_DAC_MAX_BUF_LEN, ch->buf, ch->dma_buf);
		}
	}
	cdev_del(&dac->cdev);
fail:
	dev_err(dev, "probe failed: %d\n", err);
	return err;
}

static struct platform_driver stm32_dac_driver = {
	.driver = {
		.name	= "stm32_dac",
		.owner	= THIS_MODULE,
	},
	.probe	= stm32_dac_probe,
};


static int __init stm32_dac_register(void)
{
	int ret;

	stm32_dac_class = class_create(THIS_MODULE, "stm32_dac");
	if (IS_ERR(stm32_dac_class)) {
		ret = PTR_ERR(stm32_dac_class);
		pr_err("%s: unable to register class: %d\n", __func__, ret);
		goto fail;
	}

	ret = alloc_chrdev_region(&stm32_dac_devt, 0, MAX_NR_CHANNELS, "dac");
	if (ret < 0) {
		pr_err("%s: unable to allocate char dev region for STM32 DAC\n", __func__);
		goto fail_chrdev;
	}
	stm32_dac_major = MAJOR(stm32_dac_devt);

	ret = platform_driver_register(&stm32_dac_driver);
	if (ret)
		goto fail_platform;

	return 0;

fail_platform:
	unregister_chrdev_region(stm32_dac_devt, 1);
fail_chrdev:
	class_destroy(stm32_dac_class);
fail:
	return ret;
}

module_init(stm32_dac_register);
