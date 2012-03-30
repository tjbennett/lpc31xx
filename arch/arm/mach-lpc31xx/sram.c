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
#include <linux/platform_device.h>


static int lpc313x_sram_probe(struct platform_device *pdev)
{
#if 0
	/* number of slave select bits is required */
	prop = of_get_property(pdev->dev.of_node, "mpmc-config", &len);
	if (prop && len >= sizeof(*prop))
		num_cs = __be32_to_cpup(prop);

	MPMC_STCONFIG1 = 0x81;
	MPMC_STWTWEN1 = 1;
	MPMC_STWTOEN1 = 1;
	MPMC_STWTRD1 = 4;
	MPMC_STWTPG1 = 1;
	MPMC_STWTWR1 = 1;
	MPMC_STWTTURN1 = 2;
	/* enable oe toggle between consec reads */
	SYS_MPMC_WTD_DEL1 = _BIT(5) | 4;
#endif
	return 0;
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
