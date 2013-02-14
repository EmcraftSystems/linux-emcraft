/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Skvortsov <aspotashev@emcraft.com>
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
#include <linux/spinlock.h>

#include <asm/io.h>

#include <mach/gpio.h>

/*
 * MSS GPIO registers map
 */
#define MSS_GPIO_BASE		0x40013000
#define MSS_GPIO_CFG(port) ((MSS_GPIO_BASE) + ((port) << 2))
#define MSS_GPIO_IRQ		0x40013080
#define MSS_GPIO_IN			0x40013084
#define MSS_GPIO_OUT		0x40013088

/*
 * MSS GPIO configuration register bits
 */
#define MSS_GPIO_OUT_REG_EN	(1 << 0)
#define MSS_GPIO_IN_REG_EN	(1 << 1)
#define MSS_GPIO_OUT_BUF_EN	(1 << 2)
#define MSS_GPIO_INT_EN		(1 << 3)

#define MSS_GPIO_INT_TYPE_MSK	(7 << 5)
#define MSS_GPIO_INT_TYPE_LH	(0 << 5)	/* Level High */
#define MSS_GPIO_INT_TYPE_LL	(1 << 5)	/* Level Low */
#define MSS_GPIO_INT_TYPE_EP	(2 << 5)	/* Edge Positive */
#define MSS_GPIO_INT_TYPE_EN	(3 << 5)	/* Edge Negotive */
#define MSS_GPIO_INT_TYPE_EB	(4 << 5)	/* Edge Both */


/* Lock access to MSS GPIO block. */
static	spinlock_t	mss_gpio_lock = SPIN_LOCK_UNLOCKED;

/*
 * Get the current state of a GPIO input pin
 */
static int mss_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	u32 mss_gpio_in = readl(MSS_GPIO_IN);
	return (mss_gpio_in >> gpio) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static int mss_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	u32 mss_gpio_cfg;
	unsigned long f;

	spin_lock_irqsave(&mss_gpio_lock, f);

	mss_gpio_cfg = readl(MSS_GPIO_CFG(gpio));
	mss_gpio_cfg |= MSS_GPIO_IN_REG_EN;
	mss_gpio_cfg &= ~(MSS_GPIO_OUT_REG_EN | MSS_GPIO_OUT_BUF_EN);
	writel(mss_gpio_cfg, MSS_GPIO_CFG(gpio));

	spin_unlock_irqrestore(&mss_gpio_lock, f);

	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static void mss_gpio_set_value_locked(
	struct gpio_chip *chip, unsigned gpio, int value)
{
	u32 mss_gpio_out = readl(MSS_GPIO_OUT);
	if (value) {
		mss_gpio_out |=  1 << gpio;
	} else {
		mss_gpio_out &=  ~(1 << gpio);
	}
	writel(mss_gpio_out, MSS_GPIO_OUT);
}

static void mss_gpio_set_value(
	struct gpio_chip *chip, unsigned gpio, int value)
{
	unsigned long f;

	spin_lock_irqsave(&mss_gpio_lock, f);

	mss_gpio_set_value_locked(chip, gpio, value);

	spin_unlock_irqrestore(&mss_gpio_lock, f);
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin.
 */
static int mss_gpio_direction_output(
	struct gpio_chip *chip, unsigned gpio, int level)
{
	u32 mss_gpio_cfg;
	unsigned long f;

	spin_lock_irqsave(&mss_gpio_lock, f);

	mss_gpio_cfg = readl(MSS_GPIO_CFG(gpio));
	mss_gpio_cfg |= MSS_GPIO_OUT_REG_EN | MSS_GPIO_OUT_BUF_EN;
	mss_gpio_cfg &= ~(MSS_GPIO_IN_REG_EN);
	writel(0, MSS_GPIO_CFG(gpio));
	mss_gpio_set_value(chip, gpio, level);
	writel(mss_gpio_cfg, MSS_GPIO_CFG(gpio));

	spin_unlock_irqrestore(&mss_gpio_lock, f);

	return 0;
}

static struct gpio_chip mss_gpio_chip = {
	.label				= "mss_gpio",
	.direction_input	= mss_gpio_direction_input,
	.get				= mss_gpio_get_value,
	.direction_output	= mss_gpio_direction_output,
	.set				= mss_gpio_set_value,
	.base				= 0,
	.ngpio				= 32,
	.can_sleep			= 0,
};

void __init m2s_gpio_init(void)
{
	if (gpiochip_add(&mss_gpio_chip) < 0) {
		pr_err("%s: gpiochip_add failed.\n", __func__);
	}
}
