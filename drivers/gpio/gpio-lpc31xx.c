/* linux/arch/arm/mach-lpc31xx/gpiolib.c
 *
 * Copyright (c) 2011 Jon Smirl <jonsmirl@gmail.com>
 *
 * LPC31XX GPIOlib support
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

struct lpc31xx_gpio_chip {
	struct of_mm_gpio_chip mmchip;
	int index;
};

/* this table maps from gpio register bits into event router bits
 * the numbers correspond to the 128 event bits in the event router
 * look up the gpio bit in this table to get an event bit then
 * ask the event router driver to tell you which irq it is mapped to
 */
static const uint8_t ebi_mci[] = {0x38,0x35,0x08,0x05,0x03,0x15,0x4F,0x50,0x2B,0x2D,0x01,0x2C,0x12,0x02,0x13,0x11,
	0x14,0x04,0x06,0x07,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x34,0x36,0x37,0x39};
static const uint8_t ebi_i2stx_0[] = {0x16,0x17,0x18,0x4D,0x4E,0x51,0x52,0x2E,0x2A,0x29};
static const uint8_t i2srx_0[] = {0x53,0x54,0x55};
static const uint8_t i2srx_1[] = {0x56,0x57,0x58};
static const uint8_t i2stx_1[] = {0x59,0x5A,0x5B,0x5C};
static const uint8_t ebi[] = {0x22,0x23,0x24,0x25,0x26,0x27,0x1D,0x19,0x1A,0x1B,0x1C,0x1E,0x1F,0x20,0x21,0x28};
static const uint8_t gpio[] = {0x30,0x2F,0x31,0x32,0x33,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,0x40,0x41,0x60,0x61};
static const uint8_t i2c1[] = {0x5D,0x5E};
static const uint8_t spi[] = {0x46,0x47,0x48,0x49,0x4A};
static const uint8_t nand_ctrl[] = {0x45,0x42,0x43,0x44};
static const uint8_t pwm[] = {0x5F};
static const uint8_t uart[] = {0x4B,0x4C};

static struct {
	const uint8_t *evt;
	int count;
	struct lpc31xx_gpio_chip *chip;
} gpio_evt[] = {
	{ebi_mci, 	sizeof(ebi_mci)},
	{ebi_i2stx_0, 	sizeof(ebi_i2stx_0)},
	{NULL, 		1},  /* cgu one is odd, not mapped into event router */
	{i2srx_0, 	sizeof(i2srx_0)},
	{i2srx_1, 	sizeof(i2srx_1)},
	{i2stx_1, 	sizeof(i2stx_1)},
	{ebi, 		sizeof(ebi)},
	{gpio, 		sizeof(gpio)},
	{i2c1, 		sizeof(i2c1)},
	{spi, 		sizeof(spi)},
	{nand_ctrl, 	sizeof(nand_ctrl)},
	{pwm, 		sizeof(pwm)},
	{uart, 		sizeof(uart)},
};

#if 0
/**
 * struct lpc31xx_gpio_chip - wrapper for specific implementation of gpio
 * @chip: The chip structure to be exported via gpiolib.
 * @base: The base pointer to the gpio configuration registers.
 * @config: special function and pull-resistor control information.
 * @pm_save: Save information for suspend/resume support.
 *
 * This wrapper provides the necessary information for the NXP
 * specific gpios being registered with gpiolib.
 */
struct lpc31xx_gpio_chip {
	struct gpio_chip	chip;
	struct lpc31xx_gpio_cfg	*config;
	struct lpc31xx_gpio_pm	*pm;
	int			base;
#ifdef CONFIG_PM
	uint32_t		pm_save[4];
#endif
};
#endif

static int inline *gpc(void __iomem *base, int reg)
{
	return (int *)(base + reg);
}

static int lpc31xx_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
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

static int lpc31xx_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
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

static int lpc31xx_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(chip);
	int __iomem *base = mm_gc->regs;
	int pin = 1 << gpio;
	int value;

	value = ((*base & pin) != 0);
	return value;
}

static void lpc31xx_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(chip);
	int __iomem *base = mm_gc->regs;
	int pin = 1 << gpio;

	if (value)
		*gpc(base, GPIO_M0_SET) = pin;
	else
		*gpc(base, GPIO_M0_RESET) = pin;
}

extern int lpc31xx_event_to_irq(int event);

static int lpc31xx_gpio_to_irq(struct gpio_chip *gc, unsigned gpio)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct lpc31xx_gpio_chip *chip = container_of(mm_gc, struct lpc31xx_gpio_chip, mmchip);

	return lpc31xx_event_to_irq(gpio_evt[chip->index].evt[gpio]);
}

int lpc31xx_reg_to_gpio(unsigned index, unsigned gpio)
{
	struct lpc31xx_gpio_chip *chip;

	chip = gpio_evt[index].chip;
	return chip->mmchip.gc.base + gpio;
}
EXPORT_SYMBOL(lpc31xx_reg_to_gpio);


static int lpc31xx_gpiochip_remove(struct platform_device *ofdev)
{
	return -EBUSY;
}

static int __devinit lpc31xx_simple_gpiochip_probe(struct platform_device *pdev)
{
	struct lpc31xx_gpio_chip *chip;
	struct gpio_chip *gc;
	struct resource *res;
	int ret;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->index = (res->start >> 6) & 0xF;
	gpio_evt[chip->index].chip = chip;

	gc = &chip->mmchip.gc;
	gc->ngpio            = gpio_evt[chip->index].count;
	gc->direction_input  = lpc31xx_gpio_direction_input;
	gc->direction_output = lpc31xx_gpio_direction_output;
	gc->get              = lpc31xx_gpio_get_value;
	gc->set              = lpc31xx_gpio_set_value;
	gc->to_irq	     = lpc31xx_gpio_to_irq;

	ret = of_mm_gpiochip_add(pdev->dev.of_node, &chip->mmchip);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id lpc31xx_simple_gpiochip_match[] = {
	{ .compatible = "nxp,lpc31xx-gpio", },
	{}
};

static struct platform_driver lpc31xx_simple_gpiochip_driver = {
	.driver = {
		.name = "lpc31xx-gpio",
		.owner = THIS_MODULE,
		.of_match_table = lpc31xx_simple_gpiochip_match,
	},
	.probe = lpc31xx_simple_gpiochip_probe,
	.remove = lpc31xx_gpiochip_remove,
};

static __init int lpc31xx_gpiolib_init(void)
{
	if (platform_driver_register(&lpc31xx_simple_gpiochip_driver))
		printk(KERN_ERR "Unable to register simple GPIO driver\n");

	return 0;
}

/* Make sure we get initialised before anyone else tries to use us */
core_initcall(lpc31xx_gpiolib_init);
