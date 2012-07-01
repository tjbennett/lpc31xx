/*  arch/arm/mach-lpc31xx/generic.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  Common code for machines with LPC31xx SoCs.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/console.h>
#include <linux/serial_8250.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/errno.h>
#include <asm/pgtable.h>

#include <mach/hardware.h>

#include <mach/gpio.h>
#include <asm/mach/map.h>

/* local functions */

static const struct of_device_id wdt_of_match[] __initconst = {
	{ .compatible = "nxp,lpc31xx-wdt", },
	{},
};

#define wdt_read(reg) \
	__raw_readl(wdt_regs + reg)
#define wdt_write(reg, value) \
	__raw_writel(value, wdt_regs + reg);

void lpc31xx_arch_reset(char mode, const char *cmd)
{
	struct device_node *node;
	static void __iomem *wdt_regs;

	printk("arch_reset: attempting watchdog reset\n");

	/* Remap the necessary zones */
	node = of_find_matching_node(NULL, wdt_of_match);
	wdt_regs = of_iomap(node, 0);

	/* enable WDT clock */
	cgu_clk_en_dis(CGU_SB_WDOG_PCLK_ID, 1);

	/* Disable watchdog */
	wdt_write(WDT_TCR, 0);
	wdt_write(WDT_MCR, WDT_MCR_STOP_MR1 | WDT_MCR_INT_MR1);

	/*  If TC and MR1 are equal a reset is generated. */
	wdt_write(WDT_PR, 0x00000002);
	wdt_write(WDT_TC, 0x00000FF0);
	wdt_write(WDT_MR0, 0x0000F000);
	wdt_write(WDT_MR1, 0x00001000);
	wdt_write(WDT_EMR, WDT_EMR_CTRL1(0x3));
	/* Enable watchdog timer; assert reset at timer timeout */
	wdt_write(WDT_TCR, WDT_TCR_CNT_EN);
	cpu_reset (0);/* loop forever and wait for reset to happen */

	/*NOTREACHED*/
}

struct platform_device lpc31xx_pcm_device = {
	.name = "lpc31xx-pcm-audio",
	.id = -1,
};

static struct platform_device *devices[] __initdata = {
	&lpc31xx_pcm_device,
};

static struct map_desc lpc31xx_io_desc[] __initdata = {
	{
		.virtual	= io_p2v(IO_APB1_PHYS),
		.pfn		= __phys_to_pfn(IO_APB1_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_APB2_PHYS),
		.pfn		= __phys_to_pfn(IO_APB2_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_APB3_PHYS),
		.pfn		= __phys_to_pfn(IO_APB3_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_APB4_PHYS),
		.pfn		= __phys_to_pfn(IO_APB4_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_SDMMC_PHYS),
		.pfn		= __phys_to_pfn(IO_SDMMC_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_USB_PHYS),
		.pfn		= __phys_to_pfn(IO_USB_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(EXT_SRAM0_PHYS),
		.pfn		= __phys_to_pfn(EXT_SRAM0_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_INTC_PHYS),
		.pfn		= __phys_to_pfn(IO_INTC_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_NAND_BUF_PHYS),
		.pfn		= __phys_to_pfn(IO_NAND_BUF_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
};

void __init lpc31xx_map_io(void)
{
	iotable_init(lpc31xx_io_desc, ARRAY_SIZE(lpc31xx_io_desc));
}
extern int __init cgu_init(char *str);

void __init lpc31xx_init(void)
{
	/* cgu init */
	clk_init();
	cgu_init("");
	/* Switch on the UART clocks */
	cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_IOCONF_PCLK_ID, 1);

	/* Put adc block in low power state.
	 * Once ADC driver is added this should move to driver.
	 */
	SYS_ADC_PD = 1;
	/* Disable ring oscillators used by Random number generators */
	SYS_RNG_OSC_CFG = 0;

	/* fixme */
#if 0
	/* Mux I2S signals based on selected channel */
#if defined (CONFIG_SND_I2S_TX0_MASTER)
	/* I2S TX0 WS, DATA */
	GPIO_DRV_IP(IOCONF_EBI_I2STX_0, 0x60);

	/* I2S TX0 BCK */
	GPIO_DRV_IP(IOCONF_EBI_MCI, 0x80);
#endif

#if defined (CONFIG_SND_I2S_TX1_MASTER)
	/* I2S TX1 BCK, WS, DATA */
	GPIO_DRV_IP(IOCONF_I2STX_1, 0x7);
#endif

#if defined (CONFIG_SND_I2S_RX0_MASTER) | defined (CONFIG_SND_I2S_RX0_SLAVE)
	/* I2S RX0 BCK, WS, DATA */
	GPIO_DRV_IP(IOCONF_I2SRX_0, 0x7);
#endif
#if defined (CONFIG_SND_I2S_RX1_MASTER) | defined (CONFIG_SND_I2S_RX1_SLAVE)
	/* I2S RX1 BCK, WS, DATA */
	GPIO_DRV_IP(IOCONF_I2SRX_1, 0x7);
#endif
	/* AUDIO CODEC CLOCK (256FS) */
	GPIO_DRV_IP(IOCONF_I2STX_1, 0x8);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}



