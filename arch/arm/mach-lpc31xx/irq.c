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

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/irqs.h>
#include <mach/clock.h>

/* Macros to compute the bank based on EVENT_T */
#define EVT_GET_BANK(evt)	(((evt) >> 5) & 0x3)
#define EVT_arm926_nirq		0x6C

extern void __init lpc31xx_init_evtr(void);

static void intc_mask_irq(struct irq_data *data)
{
	INTC_REQ_REG(data->irq) = INTC_REQ_WE_ENABLE;
}

static void intc_unmask_irq(struct irq_data *data)
{
	INTC_REQ_REG(data->irq) = INTC_REQ_ENABLE | INTC_REQ_WE_ENABLE;
}

static int intc_set_wake(struct irq_data *data, unsigned int on)
{
	static u32 wake_ints = 0;

	if (on)
		/* save the irqs which wake */
		wake_ints |= _BIT(data->irq);
	else
		/* clear the irqs which don't wake */
		wake_ints &= ~_BIT(data->irq);

	/* Note: the clocks to corresponding blocks shouldn't be suspended
	 * by individual drivers for this logic to work.
	 */
	if (wake_ints) {
		/* enable ARM_IRQ routing to CGU_WAKEUP */
		EVRT_OUT_MASK_SET(4, EVT_GET_BANK(EVT_arm926_nirq)) = _BIT((EVT_arm926_nirq & 0x1F));
	} else {
		/* disable ARM_IRQ routing to CGU_WAKEUP */
		EVRT_OUT_MASK_CLR(4, EVT_GET_BANK(EVT_arm926_nirq)) = _BIT((EVT_arm926_nirq & 0x1F));
	}

	//printk("wake on irq=%d value=%d 0x%08x/0x%08x/0x%08x 0x%08x/0x%08x\r\n", irq, value, 
	//	EVRT_MASK(3), EVRT_APR(3), EVRT_ATR(3),
	//	EVRT_OUT_MASK(4,3), EVRT_OUT_PEND(4,3));

	return 0;
}

static struct irq_chip lpc31xx_internal_chip = {
	.name = "INTC",
	.irq_ack = intc_mask_irq,
	.irq_mask = intc_mask_irq,
	.irq_unmask = intc_unmask_irq,
	.irq_set_wake = intc_set_wake,
};

static const struct of_device_id intc_of_match[] __initconst = {
	{ .compatible = "nxp,lpc31xx-intc", },
	{},
};

void __init lpc31xx_init_irq(void)
{
	unsigned int irq;

	irq_domain_generate_simple(intc_of_match, 0x60000000, 0);

	/* enable clock to interrupt controller */
	cgu_clk_en_dis(CGU_SB_AHB2INTC_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_INTC_CLK_ID, 1);
	/* enable clock to Event router */
	cgu_clk_en_dis(CGU_SB_EVENT_ROUTER_PCLK_ID, 1);

	/* Set the vector base (we don't use direct vectoring, so this is 0) */
	INTC_IRQ_VEC_BASE = 0x00000000;
	INTC_FIQ_VEC_BASE = 0x00000000;

	/* mask all interrupt by setting high priority until init is done*/
	INTC_IRQ_PRI_MASK = 0xFF;
	INTC_FIQ_PRI_MASK = 0xFF;

	/* Clear and disable all interrupts. Start from index 1 since 0 is unused.*/
	for (irq = 1; irq < NR_IRQ_CPU; irq++) {
		/* Set the initial control values */
		INTC_REQ_REG(irq) = INTC_REQ_WE_ENABLE;

		/* Initialize as high-active, Disable the interrupt,
		* Set target to IRQ , Set priority level to 1 (= lowest) for
		* all the interrupt lines */
		INTC_REQ_REG(irq) = INTC_REQ_WE_ACT_LOW |
			INTC_REQ_WE_ENABLE |
			INTC_REQ_TARGET_IRQ |
			INTC_REQ_PRIO_LVL(1) |
			INTC_REQ_WE_PRIO_LVL;

		irq_set_chip_and_handler(irq, &lpc31xx_internal_chip,
					 handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	/* Set the priority threshold to 0, i.e. don't mask any interrupt */
	/* on the basis of priority level, for both targets (IRQ/FIQ)    */
	INTC_IRQ_PRI_MASK = 0;
	INTC_FIQ_PRI_MASK = 0;

}


