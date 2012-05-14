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
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/leds-pca9532.h>
#include <linux/gpio_keys.h>

#include <mach/gpio.h>

static int ea_lpc1788_pca9532_setup(unsigned gpio, unsigned ngpio);

/*
 * Describe connections of the PCA9532 chip on the EA OEM Base Board
 */
static struct pca9532_platform_data ea_lpc1788_pca9532_data = {
	.leds = {
		{ .type = PCA9532_TYPE_GPIO }, /* KEY1 */
		{ .type = PCA9532_TYPE_GPIO }, /* KEY2 */
		{ .type = PCA9532_TYPE_GPIO }, /* KEY3 */
		{ .type = PCA9532_TYPE_GPIO }, /* KEY4 */
		{ .type = PCA9532_TYPE_GPIO }, /* MMC card detect (MMC_CD) */
		{ .type = PCA9532_TYPE_GPIO }, /* MMC write protect (MMC_WP) */

		{ .type = PCA9532_TYPE_NONE }, /* not connected */
		{ .type = PCA9532_TYPE_NONE }, /* not connected */

		{
			.name = "i2c-led1",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
		{
			.name = "i2c-led2",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
		{
			.name = "i2c-led3",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
		{
			.name = "i2c-led4",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
		{
			.name = "i2c-led5",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
		{
			.name = "i2c-led6",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
		{
			.name = "i2c-led7",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
		{
			.name = "i2c-led8",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
	},
	.psc = { 0, 0 },
	.pwm = { 0, 0 },
	.gpio_base = LPC178X_GPIO_OFF_EA_LPC1788_PCA9532,
	.setup_gpio = ea_lpc1788_pca9532_setup,
};

/*
 * Use the `leds-pca9532` driver for the I2C device at address 0x60
 */
static struct i2c_board_info __initdata ea_lpc1788_pca9532[] = {
	{
		I2C_BOARD_INFO("pca9532", 0x60),
		.platform_data = &ea_lpc1788_pca9532_data,
	},
};

/*
 * GPIO lines connected to the 4 keys
 */
#define PCA9532_GPIO_KEY1	(LPC178X_GPIO_OFF_EA_LPC1788_PCA9532 + 0)
#define PCA9532_GPIO_KEY2	(LPC178X_GPIO_OFF_EA_LPC1788_PCA9532 + 1)
#define PCA9532_GPIO_KEY3	(LPC178X_GPIO_OFF_EA_LPC1788_PCA9532 + 2)
#define PCA9532_GPIO_KEY4	(LPC178X_GPIO_OFF_EA_LPC1788_PCA9532 + 3)

/*
 * Describe keys: emulate 'A', 'B', 'C', 'D'; active low.
 */
static struct gpio_keys_button pca9532_polled_button_table[] = {
	{ KEY_A, PCA9532_GPIO_KEY1, 1, "key1" },
	{ KEY_B, PCA9532_GPIO_KEY2, 1, "key2" },
	{ KEY_C, PCA9532_GPIO_KEY3, 1, "key3" },
	{ KEY_D, PCA9532_GPIO_KEY4, 1, "key4" },
};

static struct gpio_keys_platform_data pca9532_polled_keys_data = {
	.buttons = pca9532_polled_button_table,
	.nbuttons = ARRAY_SIZE(pca9532_polled_button_table),
	.poll_interval = 50,
};

static struct platform_device pca9532_polled_keys = {
	.name = "gpio-keys-polled",
	.dev = {
		.platform_data = &pca9532_polled_keys_data,
	},
};

/*
 * Register the polled keys virtual device after the PCA9532 device is probed.
 * Otherwise the `gpio-keys-polled` driver will not find the necessary GPIO
 * lines.
 */
static int ea_lpc1788_pca9532_setup(unsigned gpio, unsigned ngpio)
{
	if (platform_device_register(&pca9532_polled_keys) < 0)
		pr_err("%s: pca9532_polled_keys not registered.\n", __func__);

	return 0;
}

void __init ea_lpc1788_pca9532_init(void)
{
	/*
	 * Register the PCA9532 chip with the `leds-pca9532` driver
	 */
	if (i2c_register_board_info(0, ea_lpc1788_pca9532, 1) < 0)
		pr_err("%s: i2c_register_board_info failed.\n", __func__);
}
