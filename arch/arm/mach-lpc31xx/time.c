/*  arch/arm/mach-lpc31xx/time.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  Timer driver for LPC31xx & LPC315x.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/leds.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/board.h>

/***********************************************************************
 * Timer register definitions
 **********************************************************************/
#define TIMER_LOAD      0x00
#define TIMER_VALUE     0x04
#define TIMER_CONTROL   0x08
#define TIMER_CLEAR     0x0c

#define TM_CTRL_ENABLE    _BIT(7)
#define TM_CTRL_MODE      _BIT(6)
#define TM_CTRL_PERIODIC  _BIT(6)
#define TM_CTRL_PS1       _SBF(2, 0)
#define TM_CTRL_PS16      _SBF(2, 1)
#define TM_CTRL_PS256     _SBF(2, 2)
#define TM_CTRL_PS_MASK   _SBF(2, 0x3)

static void __iomem *timer_regs;
#define timer_read(reg) \
	__raw_readl(timer_regs + reg)
#define timer_write(reg, value) \
	__raw_writel(value, timer_regs + reg);

static irqreturn_t lpc31xx_timer_interrupt(int irq, void *dev_id)
{
	timer_write(TIMER_CLEAR, 0);
	timer_tick();
	return IRQ_HANDLED;
}

static struct irqaction lpc31xx_timer_irq = {
	.name		= "LPC31xx Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= lpc31xx_timer_interrupt,
};

/*!
 * Returns number of us since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long lpc31xx_gettimeoffset(void)
{
	uint32_t elapsed = LATCH - timer_read(TIMER_VALUE);
	return ((elapsed * 100) / (XTAL_CLOCK / 20000));
}

static void lpc31xx_timer_suspend(void)
{
	timer_write(TIMER_CONTROL, timer_read(TIMER_CONTROL) & ~TM_CTRL_ENABLE); /* disable timers */
}

static void lpc31xx_timer_resume(void)
{
	timer_write(TIMER_CONTROL, timer_read(TIMER_CONTROL) | TM_CTRL_ENABLE);	/* enable timers */
}

static struct of_device_id timer_ids[] = {
	{ .compatible = "nxp,lpc31xx-timer" },
	{ /* sentinel */ }
};

static void __init lpc31xx_timer_init(void)
{
	struct device_node *node;
	int irq;

	node = of_find_matching_node(NULL, timer_ids);
	if (!node)
		goto err;

	timer_regs = of_iomap(node, 0);
	if (!timer_regs)
		goto node_err;

	/* Get the interrupts property */
	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_crit("LPC31xx: Timer -  unable to get IRQ from DT\n");
		goto ioremap_err;
	}
	of_node_put(node);

	/* Switch on needed Timer clocks & switch off others*/
	cgu_clk_en_dis(CGU_SB_TIMER0_PCLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_TIMER1_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_TIMER2_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_TIMER3_PCLK_ID, 0);

	/* Stop/disable all timers */
	timer_write(TIMER_CONTROL, 0);

	timer_write(TIMER_LOAD, LATCH);
	timer_write(TIMER_CONTROL, (TM_CTRL_ENABLE | TM_CTRL_PERIODIC));
	timer_write(TIMER_CLEAR, 0);
	setup_irq (irq, &lpc31xx_timer_irq);

	return;

ioremap_err:
	iounmap(timer_regs);
node_err:
	of_node_put(node);
err:
	return;
}

struct sys_timer lpc31xx_timer = {
	.init = lpc31xx_timer_init,
	.offset = lpc31xx_gettimeoffset,
	.suspend = lpc31xx_timer_suspend,
	.resume = lpc31xx_timer_resume,
};
