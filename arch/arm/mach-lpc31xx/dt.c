/*
 * NXP LPC3131 board support using the device tree
 *
 *  Copyright (C) 2010 Secret Lab Technologies Ltd.
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
 */

#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <asm/hardware/vic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/clock.h>
#include <mach/dt.h>

static void __init lpc31xx_dt_init(void)
{
	lpc31xx_dt_init_common(NULL);
}

void __init lpc31xx_dt_init_common(struct of_dev_auxdata* auxdata)
{
	lpc313x_init();
	of_platform_populate(NULL, of_default_bus_match_table,
			     auxdata, NULL);
}

void __init lpc31xx_init_early(void)
{
printk("JDS - lpc31xx_init_early\n");
}

void lpc31xx_restart(char mode, const char *cmd)
{
printk("JDS - lpc31xx_restart\n");
}

static const char *lpc31xx_dt_match[] __initconst = {
	"ncp,lpc3130",
	"nxp,lpc3131",
	"nxp,lpc3152",
	"nxp,lpc3153",
	NULL,
};

DT_MACHINE_START(EA313X, "NXP LPC31xx (Device Tree Support)")
	.map_io		= lpc313x_map_io,
	.init_early	= lpc31xx_init_early,
	.init_irq	= lpc313x_init_irq,
	.timer		= &lpc313x_timer,
	.init_machine	= lpc31xx_dt_init,
	.dt_compat	= lpc31xx_dt_match,
	.restart	= lpc31xx_restart,
MACHINE_END
