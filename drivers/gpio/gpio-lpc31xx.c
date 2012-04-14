/* linux/arch/arm/mach-lpc313x/gpiolib.c
 *
 * Copyright (c) 2011 Jon Smirl <jonsmirl@gmail.com>
 *
 * LPC313X GPIOlib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/gpio.h>

#define GPIO_STATE	0x00
#define GPIO_STATE_M0	0x10
#define GPIO_M0_SET	0x14
#define GPIO_M0_RESET	0x18
#define GPIO_STATE_M1	0x20
#define GPIO_M1_SET	0x24
#define GPIO_M1_RESET	0x28


/**
 * struct lpc313x_gpio_chip - wrapper for specific implementation of gpio
 * @chip: The chip structure to be exported via gpiolib.
 * @base: The base pointer to the gpio configuration registers.
 * @config: special function and pull-resistor control information.
 * @pm_save: Save information for suspend/resume support.
 *
 * This wrapper provides the necessary information for the Samsung
 * specific gpios being registered with gpiolib.
 */
struct lpc313x_gpio_chip {
	struct gpio_chip	chip;
	struct lpc313x_gpio_cfg	*config;
	struct lpc313x_gpio_pm	*pm;
	int			base;
#ifdef CONFIG_PM
	uint32_t		pm_save[4];
#endif
};

static int inline *gpc(void __iomem *base, int reg)
{
	return (int *)(base + reg);
}

static int lpc3131_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(chip);
	void __iomem *base = mm_gc->regs;
	int pin = 1 << gpio;
	unsigned long flags;

	raw_local_irq_save(flags);
	*gpc(base, GPIO_M1_RESET) = pin;
	*gpc(base, GPIO_M0_RESET) = pin;
	raw_local_irq_restore(flags);
	return 0;
}

static int lpc3131_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(chip);
	int __iomem *base = mm_gc->regs;
	int pin = 1 << gpio;
	unsigned long flags;

	raw_local_irq_save(flags);
	*gpc(base, GPIO_M1_SET) = pin;
	if (value)
		*gpc(base, GPIO_M0_SET) = pin;
	else
		*gpc(base, GPIO_M0_RESET) = pin;
	raw_local_irq_restore(flags);
	return 0;
}

static int lpc3131_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(chip);
	int __iomem *base = mm_gc->regs;
	int pin = 1 << gpio;
	int value;

	value = ((*base & pin) != 0);
	return value;
}

static void lpc3131_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(chip);
	int __iomem *base = mm_gc->regs;
	int pin = 1 << gpio;

	if (value)
		*gpc(base, GPIO_M0_SET) = pin;
	else
		*gpc(base, GPIO_M0_RESET) = pin;
}

static int lpc3131_gpio_to_irq(struct gpio_chip *chip, int irq)
{
	printk("------------- implement lpc3131_gpio_to_irq -------------\n");
	return -ENOENT;
}

static int lpc313x_gpiochip_remove(struct platform_device *ofdev)
{
	return -EBUSY;
}

static int __devinit lpc313x_simple_gpiochip_probe(struct platform_device *ofdev)
{
	struct of_mm_gpio_chip *chip;
	struct gpio_chip *gc;
	int ret;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	gc = &chip->gc;

	gc->ngpio            = 32;
	gc->direction_input  = lpc3131_gpio_direction_input;
	gc->direction_output = lpc3131_gpio_direction_output;
	gc->get              = lpc3131_gpio_get_value;
	gc->set              = lpc3131_gpio_set_value;
	gc->to_irq	     = lpc3131_gpio_to_irq;

	ret = of_mm_gpiochip_add(ofdev->dev.of_node, chip);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id lpc313x_simple_gpiochip_match[] = {
	{ .compatible = "nxp,lpc31xx-gpio", },
	{}
};

static struct platform_driver lpc313x_simple_gpiochip_driver = {
	.driver = {
		.name = "lpc31xx-gpio",
		.owner = THIS_MODULE,
		.of_match_table = lpc313x_simple_gpiochip_match,
	},
	.probe = lpc313x_simple_gpiochip_probe,
	.remove = lpc313x_gpiochip_remove,
};

static __init int lpc313x_gpiolib_init(void)
{
	if (platform_driver_register(&lpc313x_simple_gpiochip_driver))
		printk(KERN_ERR "Unable to register simple GPIO driver\n");

	return 0;
}

/* Make sure we get initialised before anyone else tries to use us */
core_initcall(lpc313x_gpiolib_init);
