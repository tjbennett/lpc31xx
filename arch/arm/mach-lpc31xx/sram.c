/*
 * LPC313x sram driver
 *
 * Copyright (C) 2012 Jon Smirl <jonsmirl@gmail.com?
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>

/* Note that these seven registers can be assigned sequentially */
#define MPMC_SEQ_COUNT        7
#define MPMC_STCONFIG      0x00
#define MPMC_STWTWEN       0x04
#define MPMC_STWTOEN       0x08
#define MPMC_STWTRD        0x0C
#define MPMC_STWTPG        0x10
#define MPMC_STWTWR        0x14
#define MPMC_STWTTURN      0x18

static int lpc313x_sram_probe(struct platform_device *pdev)
{
	const unsigned int *prop;
	const unsigned int *ranges;
	int len, count, i;
	volatile u32 *base;

	prop = of_get_property(pdev->dev.of_node, "mpmc-config", &len);
	if (!prop)
		return 0;
	count = len / sizeof(*prop);

	ranges = of_get_property(pdev->dev.of_node, "ranges", &len);
	if (!prop) {
		dev_err(&pdev->dev, "Ranges property missing on SRAM DT");
		return -EINVAL;
	}

	if (__be32_to_cpup(ranges) == EXT_SRAM0_PHYS) {
		base = &MPMC_STCONFIG0;
	} else if (__be32_to_cpup(ranges) == EXT_SRAM1_PHYS) {
		base = &MPMC_STCONFIG1;
	} else {
		dev_err(&pdev->dev, "SRAM Ranges is not a valid base address");
		return  -EINVAL;
	}

	/* Note that the seven MPMC register are sequential */
	for (i = 0; (i < MPMC_SEQ_COUNT) && (count >= 0); i++) {
		*base = __be32_to_cpup(prop);
		base++;prop++;count--;
		if (count <= 0)
			return 0;
	}
	/* enable OE toggle between consecutive reads */
	if (count == 1) {
		if (__be32_to_cpup(ranges) == EXT_SRAM0_PHYS)
			SYS_MPMC_WTD_DEL0 = __be32_to_cpup(prop);
		else
			SYS_MPMC_WTD_DEL1 = __be32_to_cpup(prop);
		return 0;
	}
	dev_err(&pdev->dev, "SRAM too many parameters");
	return  -EINVAL;
}

static const struct of_device_id lpc313x_sram_of_match[] = {
	{ .compatible = "nxp,lpc31xx-sram" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc313x_sram_of_match);

static struct platform_driver lpc313x_sram_driver = {
	.probe = lpc313x_sram_probe,
	.driver = {
			.owner = THIS_MODULE,
			.name = "lpc313x-sram",
			.of_match_table = lpc313x_sram_of_match,
		   },
};

static int __init lpc313x_sram_init(void)
{
	return platform_driver_register(&lpc313x_sram_driver);
}

static void __exit lpc313x_sram_exit(void)
{
	platform_driver_unregister(&lpc313x_sram_driver);
}

module_init(lpc313x_sram_init);
module_exit(lpc313x_sram_exit);

MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_DESCRIPTION("Driver for the LPC313x sram");
MODULE_LICENSE("GPL");
