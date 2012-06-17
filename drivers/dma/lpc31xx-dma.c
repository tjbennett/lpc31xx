/*  arch/arm/mach-lpc313x/dma.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  DMA driver for machines with LPC313x and LPC315x SoCs.
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
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <mach/dma.h>

struct lpc31xx_dma_engine;

/**
 * struct lpc31xx_dma_desc - LPC31xx specific transaction descriptor
 * @src_addr: source address of the transaction
 * @dst_addr: destination address of the transaction
 * @size: size of the transaction (in bytes)
 * @complete: this descriptor is completed
 * @txd: dmaengine API descriptor
 * @tx_list: list of linked descriptors
 * @node: link used for putting this into a channel queue
 */
struct lpc31xx_dma_desc {
	uint32_t src_addr;
	uint32_t dst_addr;
	size_t size;
	bool complete;
	struct dma_async_tx_descriptor txd;
	struct list_head tx_list;
	struct list_head node;
};
#define DMA_MAX_CHAN_DESCRIPTORS 32

/**
 * struct lpc31xx_dma_chan - an LPC31xx DMA channel
 * @chan: dmaengine API channel
 * @number: number of the channel
 * @edma: pointer to to the engine device
 * @regs: memory mapped registers
 * @tasklet: channel specific tasklet used for callbacks
 * @lock: lock protecting the fields following
 * @flags: flags for the channel
 * @buffer: which buffer to use next (0/1)
 * @last_completed: last completed cookie value
 * @active: flattened chain of descriptors currently being processed
 * @queue: pending descriptors which are handled next
 * @free_list: list of free descriptors which can be used
 * @runtime_addr: physical address currently used as dest/src (M2M only). This
 *                is set via %DMA_SLAVE_CONFIG before slave operation is
 *                prepared
 * @runtime_ctrl: M2M runtime values for the control register.
 *
 * As LPC31xx DMA controller doesn't support real chained DMA descriptors we
 * will have slightly different scheme here: @active points to a head of
 * flattened DMA descriptor chain.
 *
 * @queue holds pending transactions. These are linked through the first
 * descriptor in the chain. When a descriptor is moved to the @active queue,
 * the first and chained descriptors are flattened into a single list.
 *
 * @chan.private holds pointer to &struct lpc31xx_dma_data which contains
 * necessary channel configuration information. For memcpy channels this must
 * be %NULL.
 */
struct lpc31xx_dma_chan {
	struct dma_chan chan;
	int number;
	const struct lpc31xx_dma_engine	*edma;
	void __iomem *regs;
	struct tasklet_struct tasklet;
	unsigned long flags;
/* Channel is configured for cyclic transfers */
#define LPC31xx_DMA_IS_CYCLIC 0

	int buffer;
	dma_cookie_t last_completed;
	struct list_head active;
	struct list_head queue;
	struct list_head free_list;
	uint32_t runtime_addr;
	uint32_t runtime_ctrl;
};

/**
 * struct lpc31xx_dma_engine - the LPC31xx DMA engine instance
 * @dma_dev: holds the dmaengine device
 * @channels: array of channels
 */
struct lpc31xx_dma_engine {
	struct dma_device dma_dev;
#define INTERRUPT_UNKNOWN 0
#define INTERRUPT_DONE 1
#define INTERRUPT_NEXT_BUFFER 2

	struct lpc31xx_dma_chan	channels[DMA_MAX_CHANNELS];
};

static inline struct device *chan2dev(struct lpc31xx_dma_chan *edmac)
{
	return &edmac->chan.dev->device;
}

static struct lpc31xx_dma_chan *to_lpc31xx_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct lpc31xx_dma_chan, chan);
}

static spinlock_t driver_lock; /* to guard state variables */

static unsigned int dma_irq_mask = 0xFFFFFFFF;
static int sg_higher_channel[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int softirqmask[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int softirqen = 0;

static int dma_channels_requested = 0;

static inline void lpc31xx_dma_increment_usage(void)
{
	if (!dma_channels_requested++) {
		cgu_clk_en_dis(CGU_SB_DMA_CLK_GATED_ID, 1);
		cgu_clk_en_dis(CGU_SB_DMA_PCLK_ID, 1);
	}
}
static inline void lpc31xx_dma_decrement_usage(void)
{
	if (!--dma_channels_requested) {
		cgu_clk_en_dis(CGU_SB_DMA_CLK_GATED_ID, 0);
		cgu_clk_en_dis(CGU_SB_DMA_PCLK_ID, 0);
	}
}

/**
 * lpc31xx_dma_set_active - set new active descriptor chain
 * @edmac: channel
 * @desc: head of the new active descriptor chain
 *
 * Sets @desc to be the head of the new active descriptor chain. This is the
 * chain which is processed next. The active list must be empty before calling
 * this function.
 *
 * Called with @driver_lock held and interrupts disabled.
 */
static void lpc31xx_dma_set_active(struct lpc31xx_dma_chan *edmac,
				  struct lpc31xx_dma_desc *desc)
{
	printk("jds - lpc31xx_dma_set_active\n");
	BUG_ON(!list_empty(&edmac->active));

	list_add_tail(&desc->node, &edmac->active);

	/* Flatten the @desc->tx_list chain into @edmac->active list */
	while (!list_empty(&desc->tx_list)) {
		struct lpc31xx_dma_desc *d = list_first_entry(&desc->tx_list,
			struct lpc31xx_dma_desc, node);

		/*
		 * We copy the callback parameters from the first descriptor
		 * to all the chained descriptors. This way we can call the
		 * callback without having to find out the first descriptor in
		 * the chain. Useful for cyclic transfers.
		 */
		d->txd.callback = desc->txd.callback;
		d->txd.callback_param = desc->txd.callback_param;

		list_move_tail(&d->node, &edmac->active);
	}
}

/* Called with @driver_lock held and interrupts disabled */
static struct lpc31xx_dma_desc *
lpc31xx_dma_get_active(struct lpc31xx_dma_chan *edmac)
{
	printk("jds - lpc31xx_dma_get_active\n");
	if (list_empty(&edmac->active))
		return NULL;

	return list_first_entry(&edmac->active, struct lpc31xx_dma_desc, node);
}

/**
 * lpc31xx_dma_advance_active - advances to the next active descriptor
 * @edmac: channel
 *
 * Function advances active descriptor to the next in the @edmac->active and
 * returns %true if we still have descriptors in the chain to process.
 * Otherwise returns %false.
 *
 * When the channel is in cyclic mode always returns %true.
 *
 * Called with @driver_lock held and interrupts disabled.
 */
static bool lpc31xx_dma_advance_active(struct lpc31xx_dma_chan *edmac)
{
	struct lpc31xx_dma_desc *desc;

	printk("jds - lpc31xx_dma_advance_active\n");
	list_rotate_left(&edmac->active);

	if (test_bit(LPC31xx_DMA_IS_CYCLIC, &edmac->flags))
		return true;

	desc = lpc31xx_dma_get_active(edmac);
	if (!desc)
		return false;

	/*
	 * If txd.cookie is set it means that we are back in the first
	 * descriptor in the chain and hence done with it.
	 */
	return !desc->txd.cookie;
}

/*
 * M2P DMA implementation
 */

static void m2p_set_control(struct lpc31xx_dma_chan *edmac, uint32_t control)
{
	printk("jds - m2p_set_control\n");
#if 0
	writel(control, edmac->regs + M2P_CONTROL);
	/*
	 * LPC31xx User's Guide states that we must perform a dummy read after
	 * write to the control register.
	 */
	readl(edmac->regs + M2P_CONTROL);
#endif
}

static int m2p_hw_setup(struct lpc31xx_dma_chan *edmac)
{
	struct lpc31xx_dma_data *data = edmac->chan.private;
	uint32_t control;

	printk("jds - m2p_hw_setup\n");
#if 0
	writel(data->port & 0xf, edmac->regs + M2P_PPALLOC);

	control = M2P_CONTROL_CH_ERROR_INT | M2P_CONTROL_ICE
		| M2P_CONTROL_ENABLE;
	m2p_set_control(edmac, control);
#endif
	return 0;
}

static inline uint32_t m2p_channel_state(struct lpc31xx_dma_chan *edmac)
{
	printk("jds - m2p_channel_state\n");
#if 0
	return (readl(edmac->regs + M2P_STATUS) >> 4) & 0x3;
#endif
}

static void m2p_hw_shutdown(struct lpc31xx_dma_chan *edmac)
{
	uint32_t control;
	printk("jds - m2p_channel_state\n");
#if 0
	control = readl(edmac->regs + M2P_CONTROL);
	control &= ~(M2P_CONTROL_STALLINT | M2P_CONTROL_NFBINT);
	m2p_set_control(edmac, control);

	while (m2p_channel_state(edmac) >= M2P_STATE_ON)
		cpu_relax();

	m2p_set_control(edmac, 0);

	while (m2p_channel_state(edmac) == M2P_STATE_STALL)
		cpu_relax();
#endif
}

static void m2p_fill_desc(struct lpc31xx_dma_chan *edmac)
{
	struct lpc31xx_dma_desc *desc;
	uint32_t bus_addr;

	printk("jds - m2p_fill_desc\n");
	desc = lpc31xx_dma_get_active(edmac);
	if (!desc) {
		dev_warn(chan2dev(edmac), "M2P: empty descriptor list\n");
		return;
	}
#if 0
	if (lpc31xx_dma_chan_direction(&edmac->chan) == DMA_MEM_TO_DEV)
		bus_addr = desc->src_addr;
	else
		bus_addr = desc->dst_addr;

	if (edmac->buffer == 0) {
		writel(desc->size, edmac->regs + M2P_MAXCNT0);
		writel(bus_addr, edmac->regs + M2P_BASE0);
	} else {
		writel(desc->size, edmac->regs + M2P_MAXCNT1);
		writel(bus_addr, edmac->regs + M2P_BASE1);
	}
#endif
	edmac->buffer ^= 1;
}

static void m2p_hw_submit(struct lpc31xx_dma_chan *edmac)
{
	printk("jds - m2p_hw_submit\n");
#if 0
	uint32_t control = readl(edmac->regs + M2P_CONTROL);

	m2p_fill_desc(edmac);
	control |= M2P_CONTROL_STALLINT;

	if (lpc31xx_dma_advance_active(edmac)) {
		m2p_fill_desc(edmac);
		control |= M2P_CONTROL_NFBINT;
	}

	m2p_set_control(edmac, control);
#endif
}

static int m2p_hw_interrupt(struct lpc31xx_dma_chan *edmac)
{
	printk("jds - m2p_hw_interrupt\n");
#if 0
	uint32_t irq_status = readl(edmac->regs + M2P_INTERRUPT);
	uint32_t control;

	if (irq_status & M2P_INTERRUPT_ERROR) {
		struct lpc31xx_dma_desc *desc = lpc31xx_dma_get_active(edmac);

		/* Clear the error interrupt */
		writel(1, edmac->regs + M2P_INTERRUPT);

		/*
		 * It seems that there is no easy way of reporting errors back
		 * to client so we just report the error here and continue as
		 * usual.
		 *
		 * Revisit this when there is a mechanism to report back the
		 * errors.
		 */
		dev_err(chan2dev(edmac),
			"DMA transfer failed! Details:\n"
			"\tcookie	: %d\n"
			"\tsrc_addr	: 0x%08x\n"
			"\tdst_addr	: 0x%08x\n"
			"\tsize		: %zu\n",
			desc->txd.cookie, desc->src_addr, desc->dst_addr,
			desc->size);
	}

	switch (irq_status & (M2P_INTERRUPT_STALL | M2P_INTERRUPT_NFB)) {
	case M2P_INTERRUPT_STALL:
		/* Disable interrupts */
		control = readl(edmac->regs + M2P_CONTROL);
		control &= ~(M2P_CONTROL_STALLINT | M2P_CONTROL_NFBINT);
		m2p_set_control(edmac, control);

		return INTERRUPT_DONE;

	case M2P_INTERRUPT_NFB:
		if (lpc31xx_dma_advance_active(edmac))
			m2p_fill_desc(edmac);

		return INTERRUPT_NEXT_BUFFER;
	}
#endif
	return INTERRUPT_UNKNOWN;
}

/*
 * DMA engine API implementation
 */

static struct lpc31xx_dma_desc *
lpc31xx_dma_desc_get(struct lpc31xx_dma_chan *edmac)
{
	struct lpc31xx_dma_desc *desc, *_desc;
	struct lpc31xx_dma_desc *ret = NULL;
	unsigned long flags;

	printk("jds - lpc31xx_dma_desc_get\n");
	spin_lock_irqsave(&driver_lock, flags);
	list_for_each_entry_safe(desc, _desc, &edmac->free_list, node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del_init(&desc->node);

			/* Re-initialize the descriptor */
			desc->src_addr = 0;
			desc->dst_addr = 0;
			desc->size = 0;
			desc->complete = false;
			desc->txd.cookie = 0;
			desc->txd.callback = NULL;
			desc->txd.callback_param = NULL;

			ret = desc;
			break;
		}
	}
	spin_unlock_irqrestore(&driver_lock, flags);
	return ret;
}

static void lpc31xx_dma_desc_put(struct lpc31xx_dma_chan *edmac,
				struct lpc31xx_dma_desc *desc)
{
	if (desc) {
		unsigned long flags;

		spin_lock_irqsave(&driver_lock, flags);
		list_splice_init(&desc->tx_list, &edmac->free_list);
		list_add(&desc->node, &edmac->free_list);
		spin_unlock_irqrestore(&driver_lock, flags);
	}
}

/**
 * lpc31xx_dma_advance_work - start processing the next pending transaction
 * @edmac: channel
 *
 * If we have pending transactions queued and we are currently idling, this
 * function takes the next queued transaction from the @edmac->queue and
 * pushes it to the hardware for execution.
 */
static void lpc31xx_dma_advance_work(struct lpc31xx_dma_chan *edmac)
{
	struct lpc31xx_dma_desc *new;
	unsigned long flags;

	printk("jds - lpc31xx_dma_advance_work\n");
	spin_lock_irqsave(&driver_lock, flags);
	if (!list_empty(&edmac->active) || list_empty(&edmac->queue)) {
		spin_unlock_irqrestore(&driver_lock, flags);
		return;
	}

	/* Take the next descriptor from the pending queue */
	new = list_first_entry(&edmac->queue, struct lpc31xx_dma_desc, node);
	list_del_init(&new->node);

	lpc31xx_dma_set_active(edmac, new);

	/* Push it to the hardware */
	m2p_hw_submit(edmac);
	spin_unlock_irqrestore(&driver_lock, flags);
}

static void lpc31xx_dma_unmap_buffers(struct lpc31xx_dma_desc *desc)
{
	struct device *dev = desc->txd.chan->device->dev;

	printk("jds - lpc31xx_dma_unmap_buffers\n");
	if (!(desc->txd.flags & DMA_COMPL_SKIP_SRC_UNMAP)) {
		if (desc->txd.flags & DMA_COMPL_SRC_UNMAP_SINGLE)
			dma_unmap_single(dev, desc->src_addr, desc->size,
					 DMA_TO_DEVICE);
		else
			dma_unmap_page(dev, desc->src_addr, desc->size,
				       DMA_TO_DEVICE);
	}
	if (!(desc->txd.flags & DMA_COMPL_SKIP_DEST_UNMAP)) {
		if (desc->txd.flags & DMA_COMPL_DEST_UNMAP_SINGLE)
			dma_unmap_single(dev, desc->dst_addr, desc->size,
					 DMA_FROM_DEVICE);
		else
			dma_unmap_page(dev, desc->dst_addr, desc->size,
				       DMA_FROM_DEVICE);
	}
}

static void lpc31xx_dma_tasklet(unsigned long data)
{
	struct lpc31xx_dma_chan *edmac = (struct lpc31xx_dma_chan *)data;
	struct lpc31xx_dma_desc *desc, *d;
	dma_async_tx_callback callback = NULL;
	void *callback_param = NULL;
	LIST_HEAD(list);

	printk("jds - lpc31xx_dma_tasklet\n");
	spin_lock_irq(&driver_lock);
	/*
	 * If dma_terminate_all() was called before we get to run, the active
	 * list has become empty. If that happens we aren't supposed to do
	 * anything more than call lpc31xx_dma_advance_work().
	 */
	desc = lpc31xx_dma_get_active(edmac);
	if (desc) {
		if (desc->complete) {
			edmac->last_completed = desc->txd.cookie;
			list_splice_init(&edmac->active, &list);
		}
		callback = desc->txd.callback;
		callback_param = desc->txd.callback_param;
	}
	spin_unlock_irq(&driver_lock);

	/* Pick up the next descriptor from the queue */
	lpc31xx_dma_advance_work(edmac);

	/* Now we can release all the chained descriptors */
	list_for_each_entry_safe(desc, d, &list, node) {
		/*
		 * For the memcpy channels the API requires us to unmap the
		 * buffers unless requested otherwise.
		 */
		if (!edmac->chan.private)
			lpc31xx_dma_unmap_buffers(desc);

		lpc31xx_dma_desc_put(edmac, desc);
	}

	if (callback)
		callback(callback_param);
}

static irqreturn_t lpc31xx_dma_irq_handler(int irq, void *dev_id)
{
	struct lpc31xx_dma_chan *edmac = dev_id;
	struct lpc31xx_dma_desc *desc;
	irqreturn_t ret = IRQ_HANDLED;

	printk("jds - lpc31xx_dma_irq_handler\n");
	spin_lock(&driver_lock);

	desc = lpc31xx_dma_get_active(edmac);
	if (!desc) {
		dev_warn(chan2dev(edmac),
			 "got interrupt while active list is empty\n");
		spin_unlock(&driver_lock);
		return IRQ_NONE;
	}

	switch (m2p_hw_interrupt(edmac)) {
	case INTERRUPT_DONE:
		desc->complete = true;
		tasklet_schedule(&edmac->tasklet);
		break;

	case INTERRUPT_NEXT_BUFFER:
		if (test_bit(LPC31xx_DMA_IS_CYCLIC, &edmac->flags))
			tasklet_schedule(&edmac->tasklet);
		break;

	default:
		dev_warn(chan2dev(edmac), "unknown interrupt!\n");
		ret = IRQ_NONE;
		break;
	}

	spin_unlock(&driver_lock);
	return ret;
}

/**
 * lpc31xx_dma_tx_submit - set the prepared descriptor(s) to be executed
 * @tx: descriptor to be executed
 *
 * Function will execute given descriptor on the hardware or if the hardware
 * is busy, queue the descriptor to be executed later on. Returns cookie which
 * can be used to poll the status of the descriptor.
 */
static dma_cookie_t lpc31xx_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(tx->chan);
	struct lpc31xx_dma_desc *desc;
	dma_cookie_t cookie;
	unsigned long flags;

	printk("jds - lpc31xx_dma_tx_submit\n");
	spin_lock_irqsave(&driver_lock, flags);

	cookie = edmac->chan.cookie;

	if (++cookie < 0)
		cookie = 1;

	desc = container_of(tx, struct lpc31xx_dma_desc, txd);

	edmac->chan.cookie = cookie;
	desc->txd.cookie = cookie;

	/*
	 * If nothing is currently processed, we push this descriptor
	 * directly to the hardware. Otherwise we put the descriptor
	 * to the pending queue.
	 */
	if (list_empty(&edmac->active)) {
		lpc31xx_dma_set_active(edmac, desc);
		m2p_hw_submit(edmac);
	} else {
		list_add_tail(&desc->node, &edmac->queue);
	}

	spin_unlock_irqrestore(&driver_lock, flags);
	return cookie;
}

/**
 * lpc31xx_dma_alloc_chan_resources - allocate resources for the channel
 * @chan: channel to allocate resources
 *
 * Function allocates necessary resources for the given DMA channel and
 * returns number of allocated descriptors for the channel. Negative errno
 * is returned in case of failure.
 */
static int lpc31xx_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(chan);
	struct lpc31xx_dma_data *data = chan->private;
	const char *name = dma_chan_name(chan);
	int ret, i;

	printk("jds - lpc31xx_dma_alloc_chan_resources\n");
	/* Sanity check the channel parameters */
	if (data) {
		if (data->port) {
			if (data->direction != DMA_MEM_TO_DEV &&
				data->direction != DMA_DEV_TO_MEM)
				return -EINVAL;
		}
	}
	if (data && data->name)
		name = data->name;

	lpc31xx_dma_increment_usage();

	spin_lock_irq(&driver_lock);
	edmac->last_completed = 1;
	edmac->chan.cookie = 1;
	ret = m2p_hw_setup(edmac);
	spin_unlock_irq(&driver_lock);

	if (ret)
		goto fail;

	for (i = 0; i < DMA_MAX_CHAN_DESCRIPTORS; i++) {
		struct lpc31xx_dma_desc *desc;

		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc) {
			dev_warn(chan2dev(edmac), "not enough descriptors\n");
			break;
		}

		INIT_LIST_HEAD(&desc->tx_list);

		dma_async_tx_descriptor_init(&desc->txd, chan);
		desc->txd.flags = DMA_CTRL_ACK;
		desc->txd.tx_submit = lpc31xx_dma_tx_submit;

		lpc31xx_dma_desc_put(edmac, desc);
	}

	return i;

fail:
	lpc31xx_dma_decrement_usage();

	return ret;
}

/**
 * lpc31xx_dma_free_chan_resources - release resources for the channel
 * @chan: channel
 *
 * Function releases all the resources allocated for the given channel.
 * The channel must be idle when this is called.
 */
static void lpc31xx_dma_free_chan_resources(struct dma_chan *chan)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(chan);
	struct lpc31xx_dma_desc *desc, *d;
	unsigned long flags;
	LIST_HEAD(list);

	printk("jds - lpc31xx_dma_free_chan_resources\n");
	BUG_ON(!list_empty(&edmac->active));
	BUG_ON(!list_empty(&edmac->queue));

	spin_lock_irqsave(&driver_lock, flags);
	m2p_hw_shutdown(edmac);
	edmac->runtime_addr = 0;
	edmac->runtime_ctrl = 0;
	edmac->buffer = 0;
	list_splice_init(&edmac->free_list, &list);
	spin_unlock_irqrestore(&driver_lock, flags);

	list_for_each_entry_safe(desc, d, &list, node)
		kfree(desc);

	lpc31xx_dma_decrement_usage();
}

/**
 * lpc31xx_dma_prep_dma_memcpy - prepare a memcpy DMA operation
 * @chan: channel
 * @dest: destination bus address
 * @src: source bus address
 * @len: size of the transaction
 * @flags: flags for the descriptor
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
lpc31xx_dma_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest,
			   dma_addr_t src, size_t len, unsigned long flags)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(chan);
	struct lpc31xx_dma_desc *desc, *first;
	size_t bytes, offset;

	printk("jds - lpc31xx_dma_prep_dma_memcpy\n");
	first = NULL;
	for (offset = 0; offset < len; offset += bytes) {
		desc = lpc31xx_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		bytes = min_t(size_t, len - offset, DMA_MAX_TRANSFERS + 1);

		desc->src_addr = src + offset;
		desc->dst_addr = dest + offset;
		desc->size = bytes;

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;

	return &first->txd;
fail:
	lpc31xx_dma_desc_put(edmac, first);
	return NULL;
}

/**
 * lpc31xx_dma_prep_slave_sg - prepare a slave DMA operation
 * @chan: channel
 * @sgl: list of buffers to transfer
 * @sg_len: number of entries in @sgl
 * @dir: direction of tha DMA transfer
 * @flags: flags for the descriptor
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
lpc31xx_dma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
			 unsigned int sg_len, enum dma_transfer_direction dir,
			 unsigned long flags)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(chan);
	struct lpc31xx_dma_desc *desc, *first;
	struct scatterlist *sg;
	int i;

	printk("jds - lpc31xx_dma_prep_slave_sg\n");
#if 0
	if (!edmac->edma->m2m && dir != lpc31xx_dma_chan_direction(chan)) {
		dev_warn(chan2dev(edmac),
			 "channel was configured with different direction\n");
		return NULL;
	}
#endif
	if (test_bit(LPC31xx_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
			 "channel is already used for cyclic transfers\n");
		return NULL;
	}

	first = NULL;
	for_each_sg(sgl, sg, sg_len, i) {
		size_t sg_len = sg_dma_len(sg);

		if (sg_len > DMA_MAX_TRANSFERS) {
			dev_warn(chan2dev(edmac), "too big transfer size %d\n",
				 sg_len);
			goto fail;
		}

		desc = lpc31xx_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		if (dir == DMA_MEM_TO_DEV) {
			desc->src_addr = sg_dma_address(sg);
			desc->dst_addr = edmac->runtime_addr;
		} else {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = sg_dma_address(sg);
		}
		desc->size = sg_len;

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;

	return &first->txd;

fail:
	lpc31xx_dma_desc_put(edmac, first);
	return NULL;
}

/**
 * lpc31xx_dma_prep_dma_cyclic - prepare a cyclic DMA operation
 * @chan: channel
 * @dma_addr: DMA mapped address of the buffer
 * @buf_len: length of the buffer (in bytes)
 * @period_len: lenght of a single period
 * @dir: direction of the operation
 *
 * Prepares a descriptor for cyclic DMA operation. This means that once the
 * descriptor is submitted, we will be submitting in a @period_len sized
 * buffers and calling callback once the period has been elapsed. Transfer
 * terminates only when client calls dmaengine_terminate_all() for this
 * channel.
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
lpc31xx_dma_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t dma_addr,
			   size_t buf_len, size_t period_len,
			   enum dma_transfer_direction dir)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(chan);
	struct lpc31xx_dma_desc *desc, *first;
	size_t offset = 0;

	printk("jds - lpc31xx_dma_prep_dma_cyclic\n");
#if 0
	if (!edmac->edma->m2m && dir != lpc31xx_dma_chan_direction(chan)) {
		dev_warn(chan2dev(edmac),
			 "channel was configured with different direction\n");
		return NULL;
	}
#endif
	if (test_and_set_bit(LPC31xx_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
			 "channel is already used for cyclic transfers\n");
		return NULL;
	}

	if (period_len > DMA_MAX_TRANSFERS + 1) {
		dev_warn(chan2dev(edmac), "too big period length %d\n",
			 period_len);
		return NULL;
	}

	/* Split the buffer into period size chunks */
	first = NULL;
	for (offset = 0; offset < buf_len; offset += period_len) {
		desc = lpc31xx_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		if (dir == DMA_MEM_TO_DEV) {
			desc->src_addr = dma_addr + offset;
			desc->dst_addr = edmac->runtime_addr;
		} else {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = dma_addr + offset;
		}

		desc->size = period_len;

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;

	return &first->txd;

fail:
	lpc31xx_dma_desc_put(edmac, first);
	return NULL;
}

/**
 * lpc31xx_dma_terminate_all - terminate all transactions
 * @edmac: channel
 *
 * Stops all DMA transactions. All descriptors are put back to the
 * @edmac->free_list and callbacks are _not_ called.
 */
static int lpc31xx_dma_terminate_all(struct lpc31xx_dma_chan *edmac)
{
	struct lpc31xx_dma_desc *desc, *_d;
	unsigned long flags;
	LIST_HEAD(list);

	printk("jds - lpc31xx_dma_terminate_all\n");
	spin_lock_irqsave(&driver_lock, flags);
	/* First we disable and flush the DMA channel */
	m2p_hw_shutdown(edmac);
	clear_bit(LPC31xx_DMA_IS_CYCLIC, &edmac->flags);
	list_splice_init(&edmac->active, &list);
	list_splice_init(&edmac->queue, &list);
	/*
	 * We then re-enable the channel. This way we can continue submitting
	 * the descriptors by just calling m2p_hw_submit() again.
	 */
	m2p_hw_setup(edmac);
	spin_unlock_irqrestore(&driver_lock, flags);

	list_for_each_entry_safe(desc, _d, &list, node)
		lpc31xx_dma_desc_put(edmac, desc);

	return 0;
}

static int lpc31xx_dma_slave_config(struct lpc31xx_dma_chan *edmac,
				   struct dma_slave_config *config)
{
	enum dma_slave_buswidth width;
	unsigned long flags;
	uint32_t addr, ctrl;

	printk("jds - lpc31xx_dma_slave_config\n");
	switch (config->direction) {
	case DMA_DEV_TO_MEM:
		width = config->src_addr_width;
		addr = config->src_addr;
		break;

	case DMA_MEM_TO_DEV:
		width = config->dst_addr_width;
		addr = config->dst_addr;
		break;

	default:
		return -EINVAL;
	}

	switch (width) {
#if 0
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		ctrl = 0;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		ctrl = M2M_CONTROL_PW_16;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		ctrl = M2M_CONTROL_PW_32;
		break;
#endif
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&driver_lock, flags);
	edmac->runtime_addr = addr;
	edmac->runtime_ctrl = ctrl;
	spin_unlock_irqrestore(&driver_lock, flags);

	return 0;
}

/**
 * lpc31xx_dma_control - manipulate all pending operations on a channel
 * @chan: channel
 * @cmd: control command to perform
 * @arg: optional argument
 *
 * Controls the channel. Function returns %0 in case of success or negative
 * error in case of failure.
 */
static int lpc31xx_dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
			      unsigned long arg)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(chan);
	struct dma_slave_config *config;

	printk("jds - lpc31xx_dma_control\n");
	switch (cmd) {
	case DMA_TERMINATE_ALL:
		return lpc31xx_dma_terminate_all(edmac);

	case DMA_SLAVE_CONFIG:
		config = (struct dma_slave_config *)arg;
		return lpc31xx_dma_slave_config(edmac, config);

	default:
		break;
	}

	return -ENOSYS;
}

/**
 * lpc31xx_dma_tx_status - check if a transaction is completed
 * @chan: channel
 * @cookie: transaction specific cookie
 * @state: state of the transaction is stored here if given
 *
 * This function can be used to query state of a given transaction.
 */
static enum dma_status lpc31xx_dma_tx_status(struct dma_chan *chan,
					    dma_cookie_t cookie,
					    struct dma_tx_state *state)
{
	struct lpc31xx_dma_chan *edmac = to_lpc31xx_dma_chan(chan);
	dma_cookie_t last_used, last_completed;
	enum dma_status ret;
	unsigned long flags;

	printk("jds - lpc31xx_dma_tx_status\n");
	spin_lock_irqsave(&driver_lock, flags);
	last_used = chan->cookie;
	last_completed = edmac->last_completed;
	spin_unlock_irqrestore(&driver_lock, flags);

	ret = dma_async_is_complete(cookie, last_completed, last_used);
	dma_set_tx_state(state, last_completed, last_used, 0);

	return ret;
}

/**
 * lpc31xx_dma_issue_pending - push pending transactions to the hardware
 * @chan: channel
 *
 * When this function is called, all pending transactions are pushed to the
 * hardware and executed.
 */
static void lpc31xx_dma_issue_pending(struct dma_chan *chan)
{
	printk("jds - lpc31xx_dma_issue_pending\n");
	lpc31xx_dma_advance_work(to_lpc31xx_dma_chan(chan));
}

static int __init lpc31xx_dma_probe(struct platform_device *pdev)
{
	struct lpc31xx_dma_engine *edma;
	int ret, i;

	printk("JDS - lpc31xx_dma_probe 1\n");
	spin_lock_init(&driver_lock);

	edma = kzalloc(sizeof(*edma), GFP_KERNEL);
	if (!edma)
		return -ENOMEM;

	INIT_LIST_HEAD(&edma->dma_dev.channels);

	/* Initialize channel parameters */
	for (i = 0; i < DMA_MAX_CHANNELS; i++) {
		struct lpc31xx_dma_chan *edmac = &edma->channels[i];

		edmac->chan.device = &edma->dma_dev;
		edmac->edma = edma;
		edmac->number = i;

		INIT_LIST_HEAD(&edmac->active);
		INIT_LIST_HEAD(&edmac->queue);
		INIT_LIST_HEAD(&edmac->free_list);

		tasklet_init(&edmac->tasklet, lpc31xx_dma_tasklet,
			     (unsigned long)edmac);

		/* Add the channel to the DMAC list */
		list_add_tail(&edmac->chan.device_node, &edma->dma_dev.channels);
	}

	dma_cap_zero(edma->dma_dev.cap_mask);
	dma_cap_set(DMA_MEMCPY, edma->dma_dev.cap_mask);
	dma_cap_set(DMA_SLAVE, edma->dma_dev.cap_mask);
	dma_cap_set(DMA_CYCLIC, edma->dma_dev.cap_mask);

	edma->dma_dev.dev = &pdev->dev;
	edma->dma_dev.device_alloc_chan_resources = lpc31xx_dma_alloc_chan_resources;
	edma->dma_dev.device_free_chan_resources = lpc31xx_dma_free_chan_resources;
	edma->dma_dev.device_tx_status = lpc31xx_dma_tx_status;
	edma->dma_dev.device_prep_slave_sg = lpc31xx_dma_prep_slave_sg;
	edma->dma_dev.device_prep_dma_cyclic = lpc31xx_dma_prep_dma_cyclic;
	edma->dma_dev.device_prep_dma_memcpy = lpc31xx_dma_prep_dma_memcpy;
	edma->dma_dev.device_control = lpc31xx_dma_control;
	edma->dma_dev.device_issue_pending = lpc31xx_dma_issue_pending;

	platform_set_drvdata(pdev, edma);

	ret = dma_async_device_register(&edma->dma_dev);
	if (ret) {
		dev_err(&pdev->dev, "unable to register\n");
		goto err_init;
	}

	dma_irq_mask = 0xFFFFFFFF;
	DMACH_IRQ_MASK = dma_irq_mask;
	ret = request_irq (IRQ_DMA, lpc31xx_dma_irq_handler, 0, "DMAC", edma);
	if (ret)
		printk (KERN_ERR "request_irq() returned error %d\n", ret);

	printk("JDS - lpc31xx_dma_probe 2\n");
	return 0;

err_init:
	kfree(edma);
	printk("JDS - lpc31xx_dma_probe err %d\n", ret);
	return ret;
}

static int __exit lpc31xx_dma_remove(struct platform_device *pdev)
{
	struct lpc31xx_dma_engine *edma = platform_get_drvdata(pdev);

	dma_async_device_unregister(&edma->dma_dev);
	kfree(edma);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id lpc313x_dma_of_match[] = {
	{ .compatible = "nxp,lpc31xx-dma" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc313x_dma_of_match);
#endif

static struct platform_driver lpc31xx_dma_driver = {
	.driver = {
			.name	= "lpc31xx-dma",
			.owner = THIS_MODULE,
#ifdef CONFIG_OF
			.of_match_table = lpc313x_dma_of_match,
#endif
	},
	.remove		= __exit_p(lpc31xx_dma_remove),
};

static int __init lpc31xx_dma_module_init(void)
{
	printk("JDS - lpc31xx_dma_module_init\n");
	return platform_driver_probe(&lpc31xx_dma_driver, lpc31xx_dma_probe);
}
subsys_initcall(lpc31xx_dma_module_init);

MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_DESCRIPTION("lpc31xx dma driver");
MODULE_LICENSE("GPL");
