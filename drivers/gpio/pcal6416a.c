/*
 *  pcal6416a.c - 16 bit I/O port
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *  Copyright (C) 2013 NXP Semiconductors
 *
 *  Derived from drivers/i2c/chips/pca953x.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/pcal6416a.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#endif

#define PCAL6416A_INPUT			0x00 /* Input port [RO] */
#define PCAL6416A_DAT_OUT      		0x02 /* GPIO DATA OUT [R/W] */
#define PCAL6416A_POLARITY     		0x04 /* Polarity Inversion port [R/W] */
#define PCAL6416A_CONFIG       		0x06 /* Configuration port [R/W] */
#define PCAL6416A_DRIVE0		0x40 /* Output drive strength Port0 [R/W] */
#define PCAL6416A_DRIVE1		0x42 /* Output drive strength Port1 [R/W] */
#define PCAL6416A_INPUT_LATCH		0x44 /* Port0 Input latch [R/W] */
#define PCAL6416A_EN_PULLUPDOWN		0x46 /* Port0 Pull-up/Pull-down enbl [R/W] */
#define PCAL6416A_SEL_PULLUPDOWN	0x48 /* Port0 Pull-up/Pull-down slct [R/W] */
#define PCAL6416A_INT_MASK		0x4A /* Interrupt mask [R/W] */ 
#define PCAL6416A_INT_STATUS		0x4C /* Interrupt status [RO] */ 
#define PCAL6416A_OUTPUT_CONFIG 	0x4F /* Output port config [R/W] */ 

static const struct i2c_device_id pcal6416a_id[] = {
	{ "pcal6416a", 16, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcal6416a_id);

struct pcal6416a_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_polarity;
	uint16_t reg_config;
	uint16_t reg_drive0;
	uint16_t reg_drive1;
	uint16_t reg_inputlatch;
	uint16_t reg_enpullupdown;
	uint16_t reg_selpullupdown;
	uint16_t reg_intmask;
	uint16_t reg_outputconfig;	
	uint16_t reg_direction;	
	
	struct i2c_client *client;
	struct pcal6416a_platform_data *dyn_pdata;
	struct gpio_chip gpio_chip;
	char **names;
};

/* write a 16-bit value to the PCAL6416A
	 reg: register address
   val: the value read back from the PCAL6416A
*/
static int pcal6416a_write_reg(struct pcal6416a_chip *chip, int reg, uint16_t val)
{
	int ret;

	ret = i2c_smbus_write_word_data(chip->client, reg, val);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

/* read the 16-bit register from the PCAL6416A
	 reg: register address
   val: the value read back from the PCAL6416A
*/
static int pcal6416a_read_reg(struct pcal6416a_chip *chip, int reg, uint16_t *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(chip->client, reg);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = (uint16_t)ret;
	return 0;
}

/* set the CONFIGURATION register of a port pin as an input
   off: bit number (0..15) 
*/
static int pcal6416a_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	reg_val = chip->reg_config | (1u << off);
	ret = pcal6416a_write_reg(chip, PCAL6416A_CONFIG, reg_val);
	if (ret)
		return ret;

	chip->reg_config = reg_val;

	return 0;
}

/* set the DIRECTION (CONFIGURATION register) of a port pin as an output
   off: bit number (0..15)
   val = 1 or 0
   return: 0 if successful
*/
static int pcal6416a_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	/* set output level */
	reg_val = chip->reg_output & ~(1u << off);

	ret = pcal6416a_write_reg(chip, PCAL6416A_DAT_OUT, reg_val);
	if (ret)
		return ret;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_config & ~(1u << off);
	ret = pcal6416a_write_reg(chip, PCAL6416A_CONFIG, reg_val);
	if (ret)
		return ret;

	chip->reg_config = reg_val;
	return 0;
}

/* read a port pin value (INPUT register) from the PCAL6416A
   off: bit number (0..15)
   return: bit value 0 or 1
*/
static int pcal6416a_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	ret = pcal6416a_read_reg(chip, PCAL6416A_INPUT, &reg_val);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

/* write a port pin value (INPUT register) from the PCAL6416A
   off: bit number (0..15)
   val: 0 or 1
   return: none
*/
static void pcal6416a_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_output | (1u << off);		
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pcal6416a_write_reg(chip, PCAL6416A_DAT_OUT, reg_val);
	if (ret)
		return;

	chip->reg_output = reg_val;
}

#if defined(__UNUSED__)

/* write a value to the DRIVE0 register
   val: 16-bit value to write to the OUTPUT_DRIVE_STRENGTH0 register
   return: none
*/
static void pcal6416a_gpio_set_drive0(struct gpio_chip *gc, int val)
{
	struct pcal6416a_chip *chip;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	ret = pcal6416a_write_reg(chip, PCAL6416A_DRIVE0, val);
	if (ret)
		return;

	chip->reg_drive0 = val;
}

/* write a value to the DRIVE1 register
   val: 16-bit value to write to the OUTPUT_DRIVE_STRENGTH0 register
   return: none
*/
static void pcal6416a_gpio_set_drive1(struct gpio_chip *gc, int val)
{
	struct pcal6416a_chip *chip;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	ret = pcal6416a_write_reg(chip, PCAL6416A_DRIVE1, val);
	if (ret)
		return;

	chip->reg_drive1 = val;
}


/* write a port pin value INPUT_LATCH register from the PCAL6416A
   off: bit number (0..15)
   val: 0 or 1
   return: none
*/
static void pcal6416a_gpio_set_latch(struct gpio_chip *gc, unsigned off, int val)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_inputlatch | (1u << off);		
	else
		reg_val = chip->reg_inputlatch & ~(1u << off);

	ret = pcal6416a_write_reg(chip, PCAL6416A_INPUT_LATCH, reg_val);
	if (ret)
		return;

	chip->reg_inputlatch = reg_val;
}

/* write a bit in the enable pull-up/down register [0x45:0x44]
   off: bit number (0..15)
   val: 0 or 1
   return: none
*/
static void pcal6416a_gpio_set_enpullupdown(struct gpio_chip *gc, unsigned off, int val)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_enpullupdown | (1u << off);		
	else
		reg_val = chip->reg_enpullupdown & ~(1u << off);

	ret = pcal6416a_write_reg(chip, PCAL6416A_EN_PULLUPDOWN, reg_val);
	if (ret)
		return;

	chip->reg_enpullupdown = reg_val;
}

/* write a bit in the select pull-up/down registers [0x47:0x46]
   off: bit number (0..15)
   val: 0 or 1
   return: none
*/
static void pcal6416a_gpio_set_selpullupdown(struct gpio_chip *gc, unsigned off, int val)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_selpullupdown | (1u << off);		
	else
		reg_val = chip->reg_selpullupdown & ~(1u << off);

	ret = pcal6416a_write_reg(chip, PCAL6416A_EN_PULLUPDOWN, reg_val);
	if (ret)
		return;

	chip->reg_selpullupdown = reg_val;
}

/* write a bit in the interrupt mask registers [0x49:0x48]
   off: bit number (0..15)
   val: 0 or 1
   return: none
*/
static void pcal6416a_gpio_set_intmask(struct gpio_chip *gc, unsigned off, int val)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_intmask | (1u << off);		
	else
		reg_val = chip->reg_intmask & ~(1u << off);

	ret = pcal6416a_write_reg(chip, PCAL6416A_INT_MASK, reg_val);
	if (ret)
		return;

	chip->reg_intmask = reg_val;
}

/* read a bit in the interrupt status register
   off: bit number (0..15)
   return: bit value 0 or 1
*/
static int pcal6416a_gpio_get_intstatus_bit(struct gpio_chip *gc, unsigned off)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	ret = pcal6416a_read_reg(chip, PCAL6416A_INT_STATUS, &reg_val);
	if (ret < 0) {
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

/* read the interrupt status register
   return: 16-bit value from the int status register
*/
static int pcal6416a_gpio_get_intstatus(struct gpio_chip *gc)
{
	struct pcal6416a_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pcal6416a_chip, gpio_chip);

	ret = pcal6416a_read_reg(chip, PCAL6416A_INT_STATUS, &reg_val);
	if (ret < 0) {
		return 0;
	}

	return (reg_val);
}

#endif /* __UNUSED__ */

static void pcal6416a_setup_gpio(struct pcal6416a_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pcal6416a_gpio_direction_input;
	gc->direction_output = pcal6416a_gpio_direction_output;
	gc->get = pcal6416a_gpio_get_value;
	gc->set = pcal6416a_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF_GPIO
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct pcal6416a_platform_data *
pcal6416a_get_alt_pdata(struct i2c_client *client)
{
	struct pcal6416a_platform_data *pdata;
	struct device_node *node;
	const uint16_t *val;

	node = dev_archdata_get_node(&client->dev.archdata);
	if (node == NULL)
		return NULL;

	pdata = kzalloc(sizeof(struct pcal6416a_platform_data), GFP_KERNEL);
	if (pdata == NULL) {
		dev_err(&client->dev, "Unable to allocate platform_data\n");
		return NULL;
	}

	pdata->gpio_base = -1;
	val = of_get_property(node, "linux,gpio-base", NULL);
	if (val) {
		if (*val < 0)
			dev_warn(&client->dev,
				 "invalid gpio-base in device tree\n");
		else
			pdata->gpio_base = *val;
	}

	val = of_get_property(node, "polarity", NULL);
	if (val)
		pdata->invert = *val;

	return pdata;
}
#else
static struct pcal6416a_platform_data *
pcal6416a_get_alt_pdata(struct i2c_client *client)
{
	return NULL;
}
#endif

static int __devinit pcal6416a_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pcal6416a_platform_data *pdata;
	struct pcal6416a_chip *chip;
	int ret;

	chip = kzalloc(sizeof(struct pcal6416a_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		pdata = pcal6416a_get_alt_pdata(client);
		/*
		 * Unlike normal platform_data, this is allocated
		 * dynamically and must be freed in the driver
		 */
		chip->dyn_pdata = pdata;
	}

	if (pdata == NULL) {
		dev_dbg(&client->dev, "no platform data\n");
		ret = -EINVAL;
		goto out_failed;
	}

	chip->client = client;

	chip->gpio_start = pdata->gpio_base;

	chip->names = pdata->names;

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pcal6416a_setup_gpio(chip, id->driver_data);

	ret = pcal6416a_read_reg(chip, PCAL6416A_DAT_OUT, &chip->reg_output);
	if (ret)
		goto out_failed;

	ret = pcal6416a_read_reg(chip, PCAL6416A_CONFIG, &chip->reg_direction);
	if (ret)
		goto out_failed;

	/* set platform specific polarity inversion */
	ret = pcal6416a_write_reg(chip, PCAL6416A_POLARITY, pdata->invert);
	if (ret)
		goto out_failed;

	ret = pcal6416a_read_reg(chip, PCAL6416A_CONFIG, &chip->reg_config);
	if (ret)
		goto out_failed;

	ret = pcal6416a_read_reg(chip, PCAL6416A_DRIVE0, &chip->reg_drive0);
	if (ret)
		goto out_failed;
	
	ret = pcal6416a_read_reg(chip, PCAL6416A_DRIVE1, &chip->reg_drive1);
	if (ret)
		goto out_failed;

	ret = pcal6416a_read_reg(chip, PCAL6416A_INPUT_LATCH, &chip->reg_inputlatch);
	if (ret)
		goto out_failed;
	
	ret = pcal6416a_read_reg(chip, PCAL6416A_EN_PULLUPDOWN, &chip->reg_enpullupdown);
	if (ret)
		goto out_failed;

	ret = pcal6416a_read_reg(chip, PCAL6416A_SEL_PULLUPDOWN, &chip->reg_selpullupdown);
	if (ret)
		goto out_failed;
	
	ret = pcal6416a_read_reg(chip, PCAL6416A_INT_MASK, &chip->reg_intmask);
	if (ret)
		goto out_failed;
	
	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		goto out_failed;

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);
	return 0;

out_failed:
	kfree(chip->dyn_pdata);
	kfree(chip);
	return ret;
}

static int pcal6416a_remove(struct i2c_client *client)
{
	struct pcal6416a_platform_data *pdata = client->dev.platform_data;
	struct pcal6416a_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	kfree(chip->dyn_pdata);
	kfree(chip);
	return 0;
}

static struct i2c_driver pcal6416a_driver = {
	.driver = {
		.name	= "pcal6416a",
	},
	.probe		= pcal6416a_probe,
	.remove		= pcal6416a_remove,
	.id_table	= pcal6416a_id,
};

static int __init pcal6416a_init(void)
{
	return i2c_add_driver(&pcal6416a_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(pcal6416a_init);

static void __exit pcal6416a_exit(void)
{
	i2c_del_driver(&pcal6416a_driver);
}
module_exit(pcal6416a_exit);

MODULE_DESCRIPTION("GPIO expander driver for PCAL6416A");
MODULE_LICENSE("GPL");
