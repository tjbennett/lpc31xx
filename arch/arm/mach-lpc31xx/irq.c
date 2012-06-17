/*  linux/arch/arm/mach-lpc31xx/irq.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Interrupt controller and event router driver for LPC31xx & LPC315x.
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

#include <linux/init.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/irqs.h>
#include <mach/clock.h>

/***********************************************************************
 * Interrupt controller register definitions
 **********************************************************************/
#define INTC_IRQ_PRI_MASK     0x000
#define INTC_FIQ_PRI_MASK     0x004
#define INTC_IRQ_VEC_BASE     0x100
#define INTC_FIQ_VEC_BASE     0x104
#define INTC_REQ_REG(irq)     (0x400 + ((irq) << 2))

#define INTC_REQ_PEND         _BIT(31)
#define INTC_REQ_SET_SWINT    _BIT(30)
#define INTC_REQ_CLR_SWINT    _BIT(29)
#define INTC_REQ_WE_PRIO_LVL  _BIT(28)
#define INTC_REQ_WE_TARGET    _BIT(27)
#define INTC_REQ_WE_ENABLE    _BIT(26)
#define INTC_REQ_WE_ACT_LOW   _BIT(25)
#define INTC_REQ_ACT_LOW      _BIT(17)
#define INTC_REQ_ENABLE       _BIT(16)
#define INTC_REQ_TARGET(n)    _SBF(8, ((n) & 0x3F))
#define INTC_REQ_PRIO_LVL(n)  ((n) & 0xFF)
#define INTC_REQ_TARGET_IRQ   (INTC_REQ_WE_TARGET)
#define INTC_REQ_TARGET_FIQ   (INTC_REQ_WE_TARGET | _BIT(8))

#define NR_IRQ_CPU	30	/* IRQs directly recognized by CPU */

static void __iomem *intc_regs;
#define intc_read(reg) \
	__raw_readl(intc_regs + reg)
#define intc_write(reg, value) \
	__raw_writel(value, intc_regs + reg);


/* Macros to compute the bank based on EVENT_T */
#define EVT_arm926_nirq		0x6C

static struct irq_domain *intc_domain;

static void intc_mask_irq(struct irq_data *data)
{
	intc_write(INTC_REQ_REG(data->hwirq), INTC_REQ_WE_ENABLE);
}

static void intc_unmask_irq(struct irq_data *data)
{
	intc_write(INTC_REQ_REG(data->hwirq), INTC_REQ_ENABLE | INTC_REQ_WE_ENABLE);
}

extern int lpc31xx_set_cgu_wakeup(int enable, int event);

static int intc_set_wake(struct irq_data *data, unsigned int on)
{
	static uint32_t wake_ints = 0;

	if (on)
		/* save the irqs which wake */
		wake_ints |= _BIT(data->hwirq);
	else
		/* clear the irqs which don't wake */
		wake_ints &= ~_BIT(data->hwirq);

	/* Note: the clocks to corresponding blocks shouldn't be suspended
	 * by individual drivers for this logic to work.
	 */
	lpc31xx_set_cgu_wakeup(EVT_arm926_nirq, wake_ints);

	//printk("wake on irq=%d value=%d 0x%08x/0x%08x/0x%08x 0x%08x/0x%08x\r\n", irq, value, 
	//	EVRT_MASK(3), EVRT_APR(3), EVRT_ATR(3),
	//	EVRT_OUT_MASK(4,3), EVRT_OUT_PEND(4,3));

	return 0;
}

static struct irq_chip lpc31xx_internal_chip = {
	.name = "INTC",
	//.irq_ack = intc_mask_irq,
	.irq_mask = intc_mask_irq,
	.irq_unmask = intc_unmask_irq,
	.irq_set_wake = intc_set_wake,
};

static const struct of_device_id intc_of_match[] __initconst = {
	{ .compatible = "nxp,lpc31xx-intc", },
	{},
};

static int intc_irq_map(struct irq_domain *h, unsigned int virq, irq_hw_number_t hw)
{
	/* Set the initial control values */
	intc_write(INTC_REQ_REG(hw), INTC_REQ_WE_ENABLE);

	/* Initialize as high-active, Disable the interrupt,
	* Set target to IRQ , Set priority level to 1 (= lowest) for
	* all the interrupt lines */
	intc_write(INTC_REQ_REG(hw), INTC_REQ_WE_ACT_LOW |
		INTC_REQ_WE_ENABLE |
		INTC_REQ_TARGET_IRQ |
		INTC_REQ_PRIO_LVL(1) |
		INTC_REQ_WE_PRIO_LVL);

	irq_set_chip_and_handler(virq, &lpc31xx_internal_chip, handle_level_irq);
	set_irq_flags(virq, IRQF_VALID);

	//printk("intc hw=%ld virq=%d\n", hw, virq);
	return 0;
}

static struct irq_domain_ops intc_ops = {
	.map	= intc_irq_map,
	.xlate	= irq_domain_xlate_onecell,
};

void __init lpc31xx_init_irq(void)
{
	struct device_node *node;

	/* Remap the necessary zones */
	node = of_find_matching_node(NULL, intc_of_match);
	intc_regs = of_iomap(node, 0);
	if (!intc_regs)
		panic(__FILE__	": find_and_map failed on 'lpc31xx-intc'");

	/* enable clock to interrupt controller */
	cgu_clk_en_dis(CGU_SB_AHB2INTC_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_INTC_CLK_ID, 1);
	/* enable clock to Event router */
	cgu_clk_en_dis(CGU_SB_EVENT_ROUTER_PCLK_ID, 1);

	/* Set the vector base (we don't use direct vectoring, so this is 0) */
	intc_write(INTC_IRQ_VEC_BASE, 0x00000000);
	intc_write(INTC_FIQ_VEC_BASE, 0x00000000);

	/* mask all interrupt by setting high priority until init is done*/
	intc_write(INTC_IRQ_PRI_MASK, 0xFF);
	intc_write(INTC_FIQ_PRI_MASK, 0xFF);

	node = of_find_matching_node_by_address(NULL, intc_of_match, INTC_PHYS);
	intc_domain = irq_domain_add_legacy(node, NR_IRQ_CPU, 0, 0, &intc_ops, NULL);

	irq_set_default_host(intc_domain);

	/* Set the priority threshold to 0, i.e. don't mask any interrupt */
	/* on the basis of priority level, for both targets (IRQ/FIQ)    */
	intc_write(INTC_IRQ_PRI_MASK, 0);
	intc_write(INTC_FIQ_PRI_MASK, 0);
}


