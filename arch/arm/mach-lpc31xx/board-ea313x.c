/*  arch/arm/mach-lpc313x/ea313x.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  ea313x board init routines.
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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dm9000.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/leds-pca9532.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/sizes.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/i2c.h>
#include <mach/board.h>
#include <mach/system.h>
#include <mach/dt.h>


/*
 * DM9000 ethernet device
 */

/* ARM MPMC contoller as part of low power design doesn't de-assert nCS and nOE for consecutive
reads but just changes address. But DM9000 requires nCS and nOE change between address. So access
other chip select area (nCS0) to force de-assertion of nCS1 and nOE1. Or else wait for long time
such as 80 usecs.
LPC313x has external logic outside of MPMC IP to toggle nOE to split consecutive reads.
The latest Apex bootloader pacth makes use of this feture.
For this to work SYS_MPMC_WTD_DEL0 & SYS_MPMC_WTD_DEL1 should be programmed with MPMC_STWTRD0
& MPMC_STWTRD1 values. The logic only deactivates the nOE for one clock cycle which is
11nsec but DM9000 needs 80nsec between nOEs. So lets add some dummy instructions such as
reading a GPIO register to compensate for extra 70nsec.
*/
# define DM_IO_DELAY()	do { gpio_get_value(GPIO_MNAND_RYBN3);} while(0)

static void dm9000_dumpblk(void __iomem *reg, int count)
{
	int i;
	int tmp;

	count = (count + 1) >> 1;
	for (i = 0; i < count; i++) {
		DM_IO_DELAY();
		tmp = readw(reg);
	}
}

static void dm9000_inblk(void __iomem *reg, void *data, int count)
{
	int i;
	u16* pdata = (u16*)data;
	count = (count + 1) >> 1;
	for (i = 0; i < count; i++) {
		DM_IO_DELAY();
		*pdata++ = readw(reg);
	}
}

static struct dm9000_plat_data dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM | DM9000_PLATF_SIMPLE_PHY,
	.dumpblk = dm9000_dumpblk,
	.inblk = dm9000_inblk,
};


/* spi_board_info.controller_data for SPI slave devices,
 * copied to spi_device.platform_data ... mostly for dma tuning
 */
struct lpc313x_spi_chip {
	u8 tx_threshold;
	u8 rx_threshold;
	u8 dma_burst_size;
	u32 timeout;
	u8 enable_loopback;
	int gpio_cs;
	void (*cs_control)(u32 command);
};

static struct ads7846_platform_data ea313x_ads7846_info = {
	.model			= 7846,
	.vref_delay_usecs	= 100,
	.x_plate_ohms		= 419,
	.y_plate_ohms		= 486,
	.gpio_pendown		= GPIO_GPIO4,
};

static struct lpc313x_spi_chip ea313x_ads7846_chip = {
	.gpio_cs	= GPIO_MUART_CTS_N,
};

/* If both SPIDEV and MTD data flash are enabled with the same chip select, only 1 will work */
#if defined(CONFIG_SPI_SPIDEV)
/* SPIDEV driver registration */
static int __init lpc313x_spidev_register(void)
{
	struct spi_board_info info[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 0,
	}, {
		.modalias	= "ads7846",
		.max_speed_hz	= 1200000,
		.bus_num	= 0,
		.chip_select	= 1,
		.platform_data	= &ea313x_ads7846_info,
		.controller_data= &ea313x_ads7846_chip,
		.irq		= IRQ_PENDOWN,
	}};
	gpio_request(GPIO_MUART_CTS_N, "touchscreen CS");
	gpio_direction_output(GPIO_MUART_CTS_N, 1);

	return spi_register_board_info(info, 3);
}
//arch_initcall(lpc313x_spidev_register);
#endif



#if defined(CONFIG_MACH_EA3152)
static struct i2c_board_info ea3152_i2c1_devices[] __initdata = {
	{
		I2C_BOARD_INFO("lpc3152-psu", 0x0C),
	},
};
#endif

void lpc313x_vbus_power(int enable)
{
	printk (KERN_INFO "enabling USB host vbus_power %d\n", enable);
	//gpio_set_value(VBUS_PWR_EN, enable);
}

struct of_dev_auxdata ea3131_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("davicom,dm9000", EXT_SRAM1_PHYS, "dm9000", &dm9000_platdata),
	{}
};

static void __init ea3131_dt_init(void)
{
	lpc31xx_dt_init_common(ea3131_auxdata_lookup);
}

#if defined(CONFIG_USB_EHCI_HCD)
static int __init ea_usb_power(void)
{
	int ret = 0;

	//ret = gpio_request(VBUS_PWR_EN, "vbus power");
	//ret = gpio_direction_output(VBUS_PWR_EN, 1);
	return ret;
}
//late_initcall(ea_usb_power);
#endif

static const char *ea3131_dt_match[] __initconst = {
	"ea,ea3131",
	NULL,
};

DT_MACHINE_START(EA313X, "NXP EA3131 (Device Tree Support)")
	.map_io		= lpc313x_map_io,
	.init_early	= lpc31xx_init_early,
	.init_irq	= lpc313x_init_irq,
	.timer		= &lpc313x_timer,
	.init_machine	= ea3131_dt_init,
	.dt_compat	= ea3131_dt_match,
	.restart	= arch_reset,
MACHINE_END


