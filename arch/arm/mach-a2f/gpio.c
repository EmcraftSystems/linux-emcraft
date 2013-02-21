/*
 * (C) Copyright 2013
 * Emcraft Systems, <www.emcraft.com>
 * Vladimir Skvortsov <vskvortsov@emcraft.com>
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
 * GPIO registers map
 */
#define MSS_GPIO_CFG		0x40013000
#define MSS_GPIO_IRQ		0x40013080
#define MSS_GPIO_IN		0x40013084
#define MSS_GPIO_OUT		0x40013088

#define FPGA_GPIO_CFG		CONFIG_A2F_FPGA_GPIO_ADDR
#define FPGA_GPIO_IRQ		(FPGA_GPIO_CFG + 0x80)
#define FPGA_GPIO_IN		(FPGA_GPIO_CFG + 0x90)
#define FPGA_GPIO_OUT		(FPGA_GPIO_CFG + 0xa0)

#define FPGA_GPIO_BASE	32 /* Base number of the FPGA GPIOs in Linux */

#define A2F_GPIO_CFG(base, port) ((base) + ((port) << 2))

/*
 * GPIO configuration register bits
 */
#define GPIO_OUT_REG_EN		(1 << 0)
#define GPIO_IN_REG_EN		(1 << 1)
#define GPIO_OUT_BUF_EN		(1 << 2)
#define GPIO_INT_EN		(1 << 3)

struct a2f_gpio_chip {
	struct gpio_chip chip;
	u32 gpio_cfg;
	u32 gpio_irq;
	u32 gpio_in;
	u32 gpio_out;
	spinlock_t lock;
};

/*
 * Get the current state of a GPIO input pin
 */
static int a2f_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct a2f_gpio_chip *a2f_chip
		= container_of(chip, struct a2f_gpio_chip, chip);
	return (readl(a2f_chip->gpio_in) >> gpio) & 1;
}

/*
 * Change the direction of a GPIO pin to input
 */
static int a2f_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	struct a2f_gpio_chip *a2f_chip
		= container_of(chip, struct a2f_gpio_chip, chip);
	u32 gpio_cfg;
	unsigned long f;

	spin_lock_irqsave(&a2f_chip->lock, f);

	gpio_cfg = readl(A2F_GPIO_CFG(a2f_chip->gpio_cfg, gpio));
	gpio_cfg |= GPIO_IN_REG_EN;
	gpio_cfg &= ~(GPIO_OUT_REG_EN | GPIO_OUT_BUF_EN);
	writel(gpio_cfg, A2F_GPIO_CFG(a2f_chip->gpio_cfg, gpio));

	spin_unlock_irqrestore(&a2f_chip->lock, f);

	return 0;
}

/*
 * Set the state of a GPIO output pin
 */
static void a2f_gpio_set_value_locked(struct a2f_gpio_chip *a2f_chip,
				      unsigned gpio, int value)
{
	u32 gpio_out = readl(a2f_chip->gpio_out);
	if (value) {
		gpio_out |=  1 << gpio;
	} else {
		gpio_out &=  ~(1 << gpio);
	}
	writel(gpio_out, a2f_chip->gpio_out);
}

static void a2f_gpio_set_value(struct gpio_chip *chip,
			       unsigned gpio, int value)
{
	struct a2f_gpio_chip *a2f_chip
		= container_of(chip, struct a2f_gpio_chip, chip);
	unsigned long f;

	spin_lock_irqsave(&a2f_chip->lock, f);

	a2f_gpio_set_value_locked(a2f_chip, gpio, value);

	spin_unlock_irqrestore(&a2f_chip->lock, f);
}

/*
 * Change the direction of a GPIO pin to output and
 * set the level on this pin.
 */
static int a2f_gpio_direction_output(struct gpio_chip *chip,
				     unsigned gpio, int level)
{
	struct a2f_gpio_chip *a2f_chip
		= container_of(chip, struct a2f_gpio_chip, chip);
	u32 gpio_cfg;
	unsigned long f;

	spin_lock_irqsave(&a2f_chip->lock, f);

	gpio_cfg = readl(A2F_GPIO_CFG(a2f_chip->gpio_cfg, gpio));
	gpio_cfg |= GPIO_OUT_REG_EN | GPIO_OUT_BUF_EN;
	gpio_cfg &= ~(GPIO_IN_REG_EN);
	writel(0, A2F_GPIO_CFG(a2f_chip->gpio_cfg, gpio));
	a2f_gpio_set_value_locked(a2f_chip, gpio, level);
	writel(gpio_cfg, A2F_GPIO_CFG(a2f_chip->gpio_cfg, gpio));

	spin_unlock_irqrestore(&a2f_chip->lock, f);

	return 0;
}

#if defined (CONFIG_A2F_MSS_GPIO)
static struct a2f_gpio_chip mss_gpio_chip = {
	.chip = {
		.label			= "mss_gpio",
		.direction_input	= a2f_gpio_direction_input,
		.get			= a2f_gpio_get_value,
		.direction_output	= a2f_gpio_direction_output,
		.set			= a2f_gpio_set_value,
		.base			= 0,
		.ngpio			= 32,
		.can_sleep		= 0,
	},
	.gpio_cfg = MSS_GPIO_CFG,
	.gpio_irq = MSS_GPIO_IRQ,
	.gpio_in = MSS_GPIO_IN,
	.gpio_out = MSS_GPIO_OUT,
	.lock = SPIN_LOCK_UNLOCKED,
};
#endif

#if defined (CONFIG_A2F_FPGA_GPIO)
static struct a2f_gpio_chip fpga_gpio_chip = {
	.chip = {
		.label			= "fpga_gpio",
		.direction_input	= a2f_gpio_direction_input,
		.get			= a2f_gpio_get_value,
		.direction_output	= a2f_gpio_direction_output,
		.set			= a2f_gpio_set_value,
		.base			= FPGA_GPIO_BASE,
		.ngpio			= 32,
		.can_sleep		= 0,
	},
	.gpio_cfg = FPGA_GPIO_CFG,
	.gpio_irq = FPGA_GPIO_IRQ,
	.gpio_in = FPGA_GPIO_IN,
	.gpio_out = FPGA_GPIO_OUT,
	.lock = SPIN_LOCK_UNLOCKED,
};
#endif

void __init a2f_gpio_init(void)
{
#if defined (CONFIG_A2F_MSS_GPIO)
	if (gpiochip_add(&mss_gpio_chip.chip) < 0) {
		pr_err("%s: gpiochip_add failed.\n", __func__);
	}
#endif

#if defined (CONFIG_A2F_FPGA_GPIO)
	if (gpiochip_add(&fpga_gpio_chip.chip) < 0) {
		pr_err("%s: gpiochip_add failed.\n", __func__);
	}
#endif
}
