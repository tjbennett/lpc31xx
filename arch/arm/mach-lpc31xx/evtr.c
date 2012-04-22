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
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/irqs.h>
#include <mach/clock.h>

/* External interrupt type enumerations */
typedef enum
{
  EVT_ACTIVE_LOW,
  EVT_ACTIVE_HIGH,
  EVT_FALLING_EDGE,
  EVT_RISING_EDGE,
  EVT_BOTH_EDGE
} EVENT_TYPE_T;

/* Macros to compute the bank based on EVENT_T */
#define EVT_GET_BANK(evt)   (((evt) >> 5) & 0x3)

#define EVT_MAX_VALID_BANKS   4
#define EVT_MAX_VALID_INT_OUT 5

/* Activation polarity register defines */
#define EVT_APR_HIGH    1
#define EVT_APR_LOW     0
#define EVT_APR_BANK0_DEF 0x00000001
#define EVT_APR_BANK1_DEF 0x00000000
#define EVT_APR_BANK2_DEF 0x00000000
#define EVT_APR_BANK3_DEF 0x0FFFFFFC

/* Activation type register defines */
#define EVT_ATR_EDGE    1
#define EVT_ATR_LEVEL   0
#define EVT_ATR_BANK0_DEF 0x00000001
#define EVT_ATR_BANK1_DEF 0x00000000
#define EVT_ATR_BANK2_DEF 0x00000000
#define EVT_ATR_BANK3_DEF 0x077FFFFC

struct event_data {
	int event;
	int group;
	int edge;
	int virq;
};
static struct irq_domain *evtr_domain;
static struct event_data *events;
static int num_events;

/* table to map from event to gpio bit */
/* mask 0x1E0 reg, mask 0x1F bit */
int event_to_gpioreg[] = {
	0x000,0x00A,0x00D,0x004,0x011,0x003,0x012,0x013,
	0x002,0x014,0x015,0x016,0x017,0x018,0x019,0x01A,
	0x01B,0x00F,0x00C,0x00E,0x010,0x005,0x020,0x021,
	0x022,0x0C7,0x0C8,0x0C9,0x0CA,0x0C6,0x0CB,0x0CC,
	0x0CD,0x0CE,0x0C0,0x0C1,0x0C2,0x0C3,0x0C4,0x0C5,
	0x0CF,0x029,0x028,0x008,0x00B,0x009,0x027,0x0E1,
	0x0E0,0x0E2,0x0E3,0x0E4,0x01C,0x001,0x01D,0x01E,
	0x000,0x01F,0x0E5,0x0E6,0x0E7,0x0E8,0x0E9,0x0EA,
	0x0EB,0x0EC,0x141,0x142,0x143,0x140,0x120,0x121,
	0x122,0x123,0x124,0x180,0x181,0x023,0x024,0x006,
	0x007,0x025,0x026,0x060,0x061,0x062,0x080,0x081,
	0x082,0x0A0,0x0A1,0x0A2,0x0A3,0x100,0x101,0x160,
	0x0ED,0x0EE,0x000,0x000,0x000,0x000,0x000,0x000,
	0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,
	0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,
	0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000,
};

static void evt_mask_irq(struct irq_data *data)
{
	uint32_t bank = EVT_GET_BANK(events[data->hwirq].event);
	uint32_t bit_pos = events[data->hwirq].event& 0x1F;

	EVRT_MASK_CLR(bank) = _BIT(bit_pos);
}

static void evt_unmask_irq(struct irq_data *data)
{
	uint32_t bank = EVT_GET_BANK(events[data->hwirq].event);
	uint32_t bit_pos = events[data->hwirq].event & 0x1F;

	EVRT_MASK_SET(bank) = _BIT(bit_pos);
}

static void evt_ack_irq(struct irq_data *data)
{
	uint32_t bank = EVT_GET_BANK(events[data->hwirq].event);
	uint32_t bit_pos = events[data->hwirq].event & 0x1F;
	//EVRT_MASK_CLR(bank) = _BIT(bit_pos);
	EVRT_INT_CLR(bank) = _BIT(bit_pos);
}

static int evt_set_type(struct irq_data *data, unsigned int flow_type)
{
	uint32_t bank = EVT_GET_BANK(events[data->hwirq].event);
	uint32_t bit_pos = events[data->hwirq].event & 0x1F;

	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		EVRT_APR(bank) |= _BIT(bit_pos);
		EVRT_ATR(bank) |= _BIT(bit_pos);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		EVRT_APR(bank) &= ~_BIT(bit_pos);
		EVRT_ATR(bank) |= _BIT(bit_pos);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		EVRT_ATR(bank) |= _BIT(bit_pos);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		EVRT_APR(bank) |= _BIT(bit_pos);
		EVRT_ATR(bank) &= ~_BIT(bit_pos);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		EVRT_APR(bank) &= ~_BIT(bit_pos);
		EVRT_ATR(bank) &= ~_BIT(bit_pos);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int evt_set_wake(struct irq_data *data, unsigned int on)
{
	uint32_t bank = EVT_GET_BANK(events[data->hwirq].event);
	uint32_t bit_pos = events[data->hwirq].event & 0x1F;

	if (on)
		/* enable routing to CGU_WAKEUP */
		EVRT_OUT_MASK_SET(4, bank) = _BIT(bit_pos);
	else
		/* disable routing to CGU_WAKEUP */
		EVRT_OUT_MASK_CLR(4, bank) = _BIT(bit_pos);

	return 0;
}

extern int lpc3131_reg_to_gpio(unsigned index, unsigned gpio);

/* when a gpio pin is request as an interrupt source,
 * make sure it is input mode
 */
static int set_input(unsigned virq)
{
	int ret, reg, gpio, event, i;

	for(i = 0; i < num_events; i++) {
		if (events[i].virq == virq) {
			event = events[i].event;
			reg  = event_to_gpioreg[event];
			if (!reg) /* not a gpio pin */
				return -EINVAL;
			gpio = lpc3131_reg_to_gpio(reg >> 5, reg & 0x1F);
			printk("setting to input %d\n", gpio);
			ret = gpio_request(gpio, "IRQ");
			if (ret)
				return ret;
			return gpio_direction_input(gpio);
		}
	}
	return -EINVAL;
}

unsigned int evt_startup(struct irq_data *data)
{
	evt_unmask_irq(data);
	return set_input(data->hwirq);
}

static struct irq_chip lpc31xx_evtr_chip = {
	.name = "EVENTROUTER",
	.irq_ack = evt_ack_irq,
	.irq_mask = evt_mask_irq,
	.irq_unmask = evt_unmask_irq,
	.irq_set_type = evt_set_type,
	.irq_set_wake = evt_set_wake,
	.irq_startup = evt_startup,
};

#define ROUTER_HDLR(n) \
	static void router##n##_handler (unsigned int irq, struct irq_desc *desc) { \
		uint32_t i, status, bank, bit_pos; \
		for (i = 0; i <= num_events; i++) {  \
			if (events[i].group != n) \
				continue; \
			/* compute bank & bit position for the event_pin */ \
			bank = EVT_GET_BANK(events[i].event); \
			bit_pos = events[i].event & 0x1F; \
			status = EVRT_OUT_PEND(n, bank); \
			if (status & _BIT(bit_pos)) { \
				generic_handle_irq(events[i].virq); \
				break; \
			} \
		} \
	}


ROUTER_HDLR(0)
ROUTER_HDLR(1)
ROUTER_HDLR(2)
ROUTER_HDLR(3)

int event_to_irq(int event)
{
	int i;
	for (i = 0; i < num_events; i++) {
		if (events[i].event == event) {
			return events[i].virq;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(event_to_irq);

static const struct of_device_id evtr_of_match[] __initconst = {
	{ .compatible = "nxp,lpc31xx-evtr", },
	{},
};

static int evtr_irq_map(struct irq_domain *h, unsigned int virq, irq_hw_number_t hw)
{
	uint32_t bank, bit_pos;
	/* compute bank & bit position for the event_pin */
	//bank = EVT_GET_BANK(irq_2_event[irq].event_pin);
	//bit_pos = irq_2_event[irq].event_pin & 0x1F;
	bank = EVT_GET_BANK(events[hw].event);
	bit_pos = events[hw].event & 0x1F;

	irq_set_chip(virq, &lpc31xx_evtr_chip);
	set_irq_flags(virq, IRQF_VALID);
	/* configure the interrupt sensitivity */
	switch (events[hw].edge) {
		case EVT_ACTIVE_LOW:
			EVRT_APR(bank) &= ~_BIT(bit_pos);
			EVRT_ATR(bank) &= ~_BIT(bit_pos);
			irq_set_handler(virq, handle_level_irq);
			break;
		case EVT_ACTIVE_HIGH:
			EVRT_APR(bank) |= _BIT(bit_pos);
			EVRT_ATR(bank) &= ~_BIT(bit_pos);
			irq_set_handler(virq, handle_level_irq);
			break;
		case EVT_FALLING_EDGE:
			EVRT_APR(bank) &= ~_BIT(bit_pos);
			EVRT_ATR(bank) |= _BIT(bit_pos);
			irq_set_handler(virq, handle_edge_irq);
			break;
		case EVT_RISING_EDGE:
			EVRT_APR(bank) |= _BIT(bit_pos);
			EVRT_ATR(bank) |= _BIT(bit_pos);
			irq_set_handler(virq, handle_edge_irq);
			break;
		case EVT_BOTH_EDGE:
			EVRT_ATR(bank) |= _BIT(bit_pos);
			irq_set_handler(virq, handle_edge_irq);
			break;
		default:
			printk("Invalid Event type.\r\n");
			break;
	}
	EVRT_OUT_MASK_SET(events[hw].group, bank) = _BIT(bit_pos);
	printk("evtr hw=%ld virq=%d Event=0x%02x bank:%d bit:%02d type:%d vector %d\n", hw, virq,
		events[hw].event, bank,
		bit_pos, events[hw].edge, events[hw].group);
	events[hw].virq = virq;

	return 0;
}

static struct irq_domain_ops evtr_ops = {
	.map	= evtr_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static int __devinit lpc31xx_evtr_probe(struct platform_device *pdev)
{
	unsigned int irq;
	int i, j;
	const __be32 *ip;
	struct device_node *np = pdev->dev.of_node;
	int cells, length;

	printk("###### Event router probe ######\n");

	ip = of_get_property(np, "#event-cells", NULL);
	if (!ip)
		return -EINVAL;
	cells = be32_to_cpup(ip);
	if (cells != 3)
		return -EINVAL;

	ip = of_get_property(np, "events", &length);
	num_events = length / (sizeof(uint32_t) * cells);
	events = kzalloc(sizeof(*events) * num_events, GFP_KERNEL);
	for (i = 0; i < num_events; i++) {
		events[i].group = be32_to_cpup(ip++);
		events[i].event = be32_to_cpup(ip++);
		events[i].edge = be32_to_cpup(ip++);
		printk("group %d bit %02x edge %d\n", events[i].group, events[i].event, events[i].edge);
		//set_input(events[i].event);
	}

	/* mask all external events */
	for (i = 0; i < EVT_MAX_VALID_BANKS; i++)
	{
		/* mask all events */
		EVRT_MASK_CLR(i) = 0xFFFFFFFF;
		/* clear all pending events */
		EVRT_INT_CLR(i) = 0xFFFFFFFF;

		for (j = 0; j < EVT_MAX_VALID_INT_OUT; j++)
		{
			/* mask all events */
			EVRT_OUT_MASK_CLR(j,i) = 0xFFFFFFFF;
		}
	}
	evtr_domain = irq_domain_add_linear(np, num_events, &evtr_ops, NULL);

	/* for power management. Wake from internal irqs */
	EVRT_APR(3) &= ~_BIT(12);
	EVRT_ATR(3) &= ~_BIT(12);
	EVRT_MASK_SET(3) = _BIT(12);

	/* install IRQ_EVT_ROUTER0  chain handler */
	irq_set_chained_handler (IRQ_EVT_ROUTER0, router0_handler);
	irq_set_chained_handler (IRQ_EVT_ROUTER1, router1_handler);
	irq_set_chained_handler (IRQ_EVT_ROUTER2, router2_handler);
	irq_set_chained_handler (IRQ_EVT_ROUTER3, router3_handler);

	return 0;
}

static int lpc31xx_evtr_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static struct platform_driver lpc31xx_evtr_driver = {
	.driver = {
		.name = "lpc31xx-evtr",
		.owner = THIS_MODULE,
		.of_match_table = evtr_of_match,
	},
	.probe = lpc31xx_evtr_probe,
	.remove = lpc31xx_evtr_remove,
};

static __init int lpc31xx_evtr_init(void)
{
	if (platform_driver_register(&lpc31xx_evtr_driver))
		printk(KERN_ERR "Unable to register Event Router driver\n");

	return 0;
}

/* Make sure we get initialized before anyone else tries to use us */
core_initcall(lpc31xx_evtr_init);


