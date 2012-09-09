/*
 * LPC31xx MultiMedia Card Interface driver
 *
 * drivers/mmc/host/lpc31xx_mmc.c
 *
 * Copyright (C) 2009 NXP Semiconductors
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

#include <linux/blkdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "lpc31xx_mmc.h"
#include <mach/irqs.h>
#include <linux/mmc/host.h>
#include <mach/board.h>
/* for time being use arch specific DMA framework instead of generic framework */
#include <mach/dma.h>

//#define USE_DMA
//#define BURST_DMA

#define LPC31xx_MCI_DATA_ERROR_FLAGS	(SDMMC_INT_DTO | SDMMC_INT_DCRC | SDMMC_INT_HTO | SDMMC_INT_SBE | SDMMC_INT_EBE)
#define LPC31xx_MCI_CMD_ERROR_FLAGS	(SDMMC_INT_RTO | SDMMC_INT_RCRC | SDMMC_INT_RESP_ERR | SDMMC_INT_HLE)
#define LPC31xx_MCI_ERROR_FLAGS		(LPC31xx_MCI_DATA_ERROR_FLAGS | LPC31xx_MCI_CMD_ERROR_FLAGS | SDMMC_INT_HLE)
#define LPC31xx_MCI_SEND_STATUS		1
#define LPC31xx_MCI_RECV_STATUS		2
#define LPC31xx_MCI_DMA_THRESHOLD	16

enum {
	EVENT_CMD_COMPLETE = 0,
	EVENT_XFER_COMPLETE,
	EVENT_DATA_COMPLETE,
	EVENT_DATA_ERROR,
	EVENT_XFER_ERROR
};


enum lpc31xx_mci_state {
	STATE_IDLE = 0,
	STATE_SENDING_CMD,
	STATE_SENDING_DATA,
	STATE_DATA_BUSY,
	STATE_SENDING_STOP,
	STATE_DATA_ERROR,
};

/*forward declaration */
struct lpc31xx_mci_slot;

struct lpc31xx_mci {
	spinlock_t		lock;
	void __iomem		*regs;

	struct scatterlist	*sg;
	unsigned int		pio_offset;

	struct lpc31xx_mci_slot	*cur_slot;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;

#ifdef USE_DMA
	int			dma_chn;
	dma_addr_t		sg_dma;
	dma_sg_ll_t		*sg_cpu;
#endif
	uint32_t			cmd_status;
	uint32_t			data_status;
	uint32_t			stop_cmdr;
	uint32_t			dir_status;
	struct tasklet_struct	tasklet;
	unsigned long		pending_events;
	unsigned long		completed_events;
	enum lpc31xx_mci_state	state;
	struct list_head	queue;

	uint32_t			bus_hz;
	uint32_t			current_speed;
	struct platform_device	*pdev;
	int slot_count;
	struct lpc31xx_mci_slot	*slot[MAX_MCI_SLOTS];
};

struct lpc31xx_mci_slot {
	struct mmc_host		*mmc;
	struct lpc31xx_mci	*host;

	uint32_t			ctype;

	struct mmc_request	*mrq;
	struct list_head	queue_node;

	unsigned int		clock;
	unsigned long		flags;
#define LPC31xx_MMC_CARD_PRESENT	0
#define LPC31xx_MMC_CARD_NEED_INIT	1
#define LPC31xx_MMC_SHUTDOWN		2
	int			id;
	int			irq;

	int			gpio_cd;
	int			gpio_wp;
	int			gpio_power;
	int			gpio_select;

	struct timer_list	detect_timer;
};

/* Register access macros */
static inline uint32_t mci_readl(struct lpc31xx_mci *host, uint32_t reg)
{
	return __raw_readl(host->regs + reg);
}

static inline void mci_writel(struct lpc31xx_mci *host, uint32_t reg, uint32_t value)
{
	__raw_writel(value, host->regs + reg);
}


#define lpc31xx_mci_test_and_clear_pending(host, event)		\
	test_and_clear_bit(event, &host->pending_events)
#define lpc31xx_mci_set_completed(host, event)			\
	set_bit(event, &host->completed_events)

#define lpc31xx_mci_set_pending(host, event)				\
	set_bit(event, &host->pending_events)

#if defined (CONFIG_DEBUG_FS)
/*
 * The debugfs stuff below is mostly optimized away when
 * CONFIG_DEBUG_FS is not set.
 */
static int lpc31xx_mci_req_show(struct seq_file *s, void *v)
{
	struct lpc31xx_mci_slot	*slot = s->private;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_command	*stop;
	struct mmc_data		*data;

	/* Make sure we get a consistent snapshot */
	spin_lock(&slot->host->lock);
	mrq = slot->mrq;

	if (mrq) {
		cmd = mrq->cmd;
		data = mrq->data;
		stop = mrq->stop;

		if (cmd)
			seq_printf(s,
				"CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				cmd->opcode, cmd->arg, cmd->flags,
				cmd->resp[0], cmd->resp[1], cmd->resp[2],
				cmd->resp[2], cmd->error);
		if (data)
			seq_printf(s, "DATA %u / %u * %u flg %x err %d\n",
				data->bytes_xfered, data->blocks,
				data->blksz, data->flags, data->error);
		if (stop)
			seq_printf(s,
				"CMD%u(0x%x) flg %x rsp %x %x %x %x err %d\n",
				stop->opcode, stop->arg, stop->flags,
				stop->resp[0], stop->resp[1], stop->resp[2],
				stop->resp[2], stop->error);
	}

	spin_unlock(&slot->host->lock);

	return 0;
}

static int lpc31xx_mci_req_open(struct inode *inode, struct file *file)
{
	return single_open(file, lpc31xx_mci_req_show, inode->i_private);
}

static const struct file_operations lpc31xx_mci_req_fops = {
	.owner		= THIS_MODULE,
	.open		= lpc31xx_mci_req_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int lpc31xx_mci_regs_show(struct seq_file *s, void *v)
{
	seq_printf(s, "STATUS:\t0x%08x\n",SDMMC_STATUS);
	seq_printf(s, "RINTSTS:\t0x%08x\n",SDMMC_RINTSTS);
	seq_printf(s, "CMD:\t0x%08x\n", SDMMC_CMD);
	seq_printf(s, "CTRL:\t0x%08x\n", SDMMC_CTRL);
	seq_printf(s, "INTMASK:\t0x%08x\n", SDMMC_INTMASK);
	seq_printf(s, "CLKENA:\t0x%08x\n", SDMMC_CLKENA);

	return 0;
}

static int lpc31xx_mci_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lpc31xx_mci_regs_show, inode->i_private);
}

static const struct file_operations lpc31xx_mci_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= lpc31xx_mci_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void lpc31xx_mci_init_debugfs(struct lpc31xx_mci_slot *slot)
{
	struct mmc_host		*mmc = slot->mmc;
	struct lpc31xx_mci	*host = slot->host;
	struct dentry		*root;
	struct dentry		*node;

	root = mmc->debugfs_root;
	if (!root)
		return;

	node = debugfs_create_file("regs", S_IRUSR, root, host,
			&lpc31xx_mci_regs_fops);
	if (IS_ERR(node))
		return;
	if (!node)
		goto err;

	node = debugfs_create_file("req", S_IRUSR, root, slot, &lpc31xx_mci_req_fops);
	if (!node)
		goto err;

	node = debugfs_create_u32("state", S_IRUSR, root, (uint32_t *)&host->state);
	if (!node)
		goto err;

	node = debugfs_create_x32("pending_events", S_IRUSR, root,
				     (uint32_t *)&host->pending_events);
	if (!node)
		goto err;

	node = debugfs_create_x32("completed_events", S_IRUSR, root,
				     (uint32_t *)&host->completed_events);
	if (!node)
		goto err;

	return;

err:
	dev_err(&mmc->class_dev, "failed to initialize debugfs for slot\n");
}
#endif

static inline unsigned ns_to_clocks(unsigned clkrate, unsigned ns)
{
	uint32_t clks;
	if (clkrate > 1000000)
		clks =  (ns * (clkrate / 1000000) + 999) / 1000;
	else
		clks =  ((ns/1000) * (clkrate / 1000) + 999) / 1000;

	return clks;
}

static void lpc31xx_mci_set_timeout(struct lpc31xx_mci *host,
		struct lpc31xx_mci_slot *slot, struct mmc_data *data)
{
	unsigned timeout;

	timeout = ns_to_clocks(slot->clock, data->timeout_ns) + data->timeout_clks;

	dev_vdbg(&slot->mmc->class_dev, "tmo req:%d + %d reg:%d clk:%d\n",
		data->timeout_ns, data->timeout_clks, timeout, slot->clock);
	/* the standard response timeout value (Ncr) is 64 clocks.
	 * Let give 4 additional clocks for response.
	 */
	mci_writel(host, SDMMC_TMOUT, /*0xffffffff); */ (timeout << 8) | (70));
}

static uint32_t lpc31xx_mci_prepare_command(struct mmc_host *mmc,
				 struct mmc_command *cmd)
{
	struct mmc_data	*data;
	uint32_t		cmdr;

	cmd->error = -EINPROGRESS;
	cmdr = cmd->opcode;

	if(cmdr == 12)
		cmdr |= SDMMC_CMD_STOP;
	else
		cmdr |= SDMMC_CMD_PRV_DAT_WAIT;

	if (cmd->flags & MMC_RSP_PRESENT) {
		cmdr |= SDMMC_CMD_RESP_EXP; // expect the respond, need to set this bit
		if (cmd->flags & MMC_RSP_136)
			cmdr |= SDMMC_CMD_RESP_LONG; // expect long respond

		if(cmd->flags & MMC_RSP_CRC)
			cmdr |= SDMMC_CMD_RESP_CRC;
	}

	data = cmd->data;
	if (data) {
		cmdr |= SDMMC_CMD_DAT_EXP;
		if (data->flags & MMC_DATA_STREAM)
			cmdr |= SDMMC_CMD_STRM_MODE; //  set stream mode
		if (data->flags & MMC_DATA_WRITE)
		    cmdr |= SDMMC_CMD_DAT_WR;

#if 0 /* Jerry, need to confirm the specification does we need to set this bit if blocks > 1 */
		if(data->blocks > 1)
		    cmdr |= SDMMC_CMD_SEND_STOP;

#endif
	}
	return cmdr;
}


static void lpc31xx_mci_start_command(struct lpc31xx_mci *host,
		struct mmc_command *cmd, uint32_t cmd_flags)
{
 	int tmo = 50;
 	host->cmd = cmd;
	dev_vdbg(&host->pdev->dev,
			"start cmd:%d ARGR=0x%08x CMDR=0x%08x\n",
			cmd->opcode, cmd->arg, cmd_flags);
	mci_writel(host, SDMMC_CMDARG, cmd->arg); // write to CMDARG register
	mci_writel(host, SDMMC_CMD, cmd_flags | SDMMC_CMD_START); // write to CMD register

	/* wait until CIU accepts the command */
	while (--tmo && (mci_readl(host, SDMMC_CMD) & SDMMC_CMD_START))
		cpu_relax();
}

static void send_stop_cmd(struct lpc31xx_mci *host, struct mmc_data *data)
{
	lpc31xx_mci_start_command(host, data->stop, host->stop_cmdr);
}


#ifdef USE_DMA

static void lpc31xx_mci_dma_cleanup(struct lpc31xx_mci *host)
{
	struct mmc_data			*data = host->data;

	if (data)
		dma_unmap_sg(&host->pdev->dev, data->sg, data->sg_len,
		     ((data->flags & MMC_DATA_WRITE)
		      ? DMA_TO_DEVICE : DMA_FROM_DEVICE));
}

static void lpc31xx_mci_stop_dma(struct lpc31xx_mci *host)
{
	if (host->dma_chn > 0) {
		dma_stop_channel(host->dma_chn);
		lpc31xx_mci_dma_cleanup(host);
	} else {
		/* Data transfer was stopped by the interrupt handler */
		lpc31xx_mci_set_pending(host, EVENT_XFER_COMPLETE);
	}
}

/* This function is called by the DMA driver from tasklet context. */
static void lpc31xx_mci_dma_complete(int chn, dma_irq_type_t type, void *arg)
{
	struct lpc31xx_mci	*host = arg;
	struct mmc_data		*data = host->data;

	dev_vdbg(&host->pdev->dev, "DMA complete\n");

	spin_lock(&host->lock);
	lpc31xx_mci_dma_cleanup(host);

	/*
	 * If the card was removed, data will be NULL. No point trying
	 * to send the stop command or waiting for NBUSY in this case.
	 */
	if (data) {
		lpc31xx_mci_set_pending(host, EVENT_XFER_COMPLETE);
		tasklet_schedule(&host->tasklet);
	}
	spin_unlock(&host->lock);
}

static int lpc31xx_mci_submit_data_dma(struct lpc31xx_mci *host, struct mmc_data *data)
{
	struct scatterlist		*sg;
	unsigned int			i, direction, sg_len;
	unsigned int			j, trans_len;

	/* If we don't have a channel, we can't do DMA */
	if (host->dma_chn < 0)
		return -ENODEV;

	/*
	 * We don't do DMA on "complex" transfers, i.e. with
	 * non-word-aligned buffers or lengths. Also, we don't bother
	 * with all the DMA setup overhead for short transfers.
	 */
	if (data->blocks * data->blksz < LPC31xx_MCI_DMA_THRESHOLD)
		return -EINVAL;
	if (data->blksz & 3)
		return -EINVAL;

	for_each_sg(data->sg, sg, data->sg_len, i) {
		if ((sg->offset & 3) || (sg->length & 3))
			return -EINVAL;
	}

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	sg_len = dma_map_sg(&host->pdev->dev, data->sg, data->sg_len,
				   direction);

	dev_vdbg(&host->pdev->dev, "sd sg_cpu: 0x%08x sg_dma:0x%08x sg_len:%d \n",
		(uint32_t)host->sg_cpu, (uint32_t)host->sg_dma, sg_len);

	for (i = 0, j = 0; i < sg_len; i++) {
		unsigned int length = sg_dma_len(&data->sg[i]);
		uint32_t mem_addr = sg_dma_address(&data->sg[i]);

		while (length) {

			host->sg_cpu[j].setup.cfg = DMA_CFG_CMP_CH_EN | DMA_CFG_CMP_CH_NR(host->dma_chn);

			if (data->flags & MMC_DATA_READ) {
				host->sg_cpu[j].setup.src_address = SDMMC_DATA_ADR;
				host->sg_cpu[j].setup.dest_address = mem_addr;
				host->sg_cpu[j].setup.cfg |= DMA_CFG_RD_SLV_NR(DMA_SLV_SDMMC);
			} else {
				host->sg_cpu[j].setup.src_address = mem_addr;
				host->sg_cpu[j].setup.dest_address = SDMMC_DATA_ADR;
				host->sg_cpu[j].setup.cfg |= DMA_CFG_WR_SLV_NR(DMA_SLV_SDMMC);
			}
			host->sg_cpu[j].next_entry = host->sg_dma + (j + 1) *
						sizeof(dma_sg_ll_t);

#ifdef BURST_DMA
			host->sg_cpu[j].setup.cfg |= DMA_CFG_TX_BURST;
      /* 16 bytes per transfer */
			trans_len = (length >> 4) - 1;
#else
      /* 4 bytes per transfer */
			trans_len = (length >> 2) - 1;
#endif

			if (trans_len > DMA_MAX_TRANSFERS) {
#ifdef BURST_DMA
				trans_len = DMA_MAX_TRANSFERS;
				length -= (DMA_MAX_TRANSFERS + 1) << 4;
				mem_addr += ((DMA_MAX_TRANSFERS + 1) << 4);
#else
				trans_len = DMA_MAX_TRANSFERS;
				length -= (DMA_MAX_TRANSFERS + 1) << 2;
				mem_addr += ((DMA_MAX_TRANSFERS + 1) << 2);
#endif
			}
			else {
				length = 0;
			}

			host->sg_cpu[j].setup.trans_length = trans_len;

			dev_vdbg(&host->pdev->dev, "sd src: 0x%08x dest:0x%08x cfg:0x%08x nxt:0x%08x len:%d \n",
				host->sg_cpu[j].setup.src_address, host->sg_cpu[j].setup.dest_address,
				host->sg_cpu[j].setup.cfg, host->sg_cpu[j].next_entry,
				host->sg_cpu[j].setup.trans_length);

			/* move to next transfer descriptor */
			j++;
		}
	}
	host->sg_cpu[j].setup.src_address = host->sg_dma;
	host->sg_cpu[j].setup.dest_address = DMACH_SOFT_INT_PHYS;
	host->sg_cpu[j].setup.trans_length = 1;
	host->sg_cpu[j].setup.cfg = 0;
	// disable irq of RX & TX, let DMA handle it
	//SDMMC_INTMASK &= ~(SDMMC_INT_RXDR | SDMMC_INT_TXDR);
	SDMMC_CTRL |= SDMMC_CTRL_DMA_ENABLE; // enable dma
	dma_prog_sg_channel(host->dma_chn, host->sg_dma);
	wmb();
	/* Go! */
	dma_start_channel(host->dma_chn);

	return 0;
}

#else
static int lpc31xx_mci_submit_data_dma(struct lpc31xx_mci *host, struct mmc_data *data)
{
	return -ENOSYS;
}

static void lpc31xx_mci_stop_dma(struct lpc31xx_mci *host)
{
	/* Data transfer was stopped by the interrupt handler */
	lpc31xx_mci_set_pending(host, EVENT_XFER_COMPLETE);
}
#endif

static void lpc31xx_mci_submit_data(struct lpc31xx_mci *host, struct mmc_data *data)
{
	data->error = -EINPROGRESS;

	WARN_ON(host->data);
	host->sg = NULL;
	host->data = data;

	if (lpc31xx_mci_submit_data_dma(host, data)) {
		host->sg = data->sg;
		host->pio_offset = 0;
		if (data->flags & MMC_DATA_READ)
			host->dir_status = LPC31xx_MCI_RECV_STATUS;
		else
			host->dir_status = LPC31xx_MCI_SEND_STATUS;

		//SDMMC_INTMASK |= (SDMMC_INT_RXDR | SDMMC_INT_TXDR);
		mci_writel(host, SDMMC_CTRL, mci_readl(host, SDMMC_CTRL) & ~SDMMC_CTRL_DMA_ENABLE); // enable dma
	}

}

#define mci_send_cmd(host, cmd, arg) { \
    mci_writel(host, SDMMC_CMDARG, arg); \
    mci_writel(host, SDMMC_CMD, SDMMC_CMD_START | cmd); \
    while (mci_readl(host, SDMMC_CMD) & SDMMC_CMD_START); \
}

void lpc31xx_mci_setup_bus(struct lpc31xx_mci_slot *slot)
{
	struct lpc31xx_mci *host = slot->host;
	uint32_t div;

	if (slot->clock != host->current_speed) {
		div  = (((host->bus_hz + (host->bus_hz / 5)) / slot->clock)) >> 1;

		dev_dbg(&slot->mmc->class_dev, "Bus speed (slot %d) = %dHz div:%d (actual %dHz)\n",
			slot->id, slot->clock, div, (host->bus_hz / div) >> 1);

		/* store the actual clock for calculations */
		slot->clock = (host->bus_hz / div) >> 1;
		/* disable clock */
		mci_writel(host, SDMMC_CLKENA, 0);
		mci_writel(host, SDMMC_CLKSRC,0);
		/* inform CIU */
		mci_send_cmd(host, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
		/* set clock to desired speed */
		mci_writel(host, SDMMC_CLKDIV, div);
		/* inform CIU */
		mci_send_cmd(host, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);
		/* enable clock */
		mci_writel(host, SDMMC_CLKENA, SDMMC_CLKEN_ENABLE);
		/* inform CIU */
		 mci_send_cmd(host, SDMMC_CMD_UPD_CLK | SDMMC_CMD_PRV_DAT_WAIT, 0);

		host->current_speed = slot->clock;
	}

	/* Set the current slot bus width */
	mci_writel(host, SDMMC_CTYPE, slot->ctype);
}

static void lpc31xx_mci_select_slot(struct lpc31xx_mci_slot *slot, int enable)
{
	if (gpio_is_valid(slot->gpio_select)) {
		printk("lpc31xx_mci_select_slot %d\n", slot->gpio_select);
		gpio_set_value(slot->gpio_select, enable);
	}
}

static void lpc31xx_mci_set_power(struct lpc31xx_mci_slot *slot, int enable)
{
	/* on current version of EA board the card detect
	 * pull-up in on switched power side. So can't do
	 * power management so use the always enable power
	 * jumper.
	 */
	if (gpio_is_valid(slot->gpio_power)) {
		gpio_set_value(slot->gpio_power, !enable);
	}
}

static void lpc31xx_mci_start_request(struct lpc31xx_mci *host,
		struct lpc31xx_mci_slot *slot)
{
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	uint32_t			cmdflags;

	mrq = slot->mrq;
	/* now select the proper slot */
	if (host->cur_slot != slot) {
		if (host->cur_slot)
			lpc31xx_mci_select_slot(host->cur_slot, 0);
		lpc31xx_mci_select_slot(slot, 1);
	}

	/* Slot specific timing and width adjustment */
	lpc31xx_mci_setup_bus(slot);

	host->cur_slot = slot;
	host->mrq = mrq;

	host->pending_events = 0;
	host->completed_events = 0;
	host->data_status = 0;

	data = mrq->data;
	if (data) {
		lpc31xx_mci_set_timeout(host, slot, data);
		mci_writel(host, SDMMC_BYTCNT,data->blksz*data->blocks);
		mci_writel(host, SDMMC_BLKSIZ,data->blksz);
	}

	cmd = mrq->cmd;
	cmdflags = lpc31xx_mci_prepare_command(slot->mmc, cmd);

	if (unlikely(test_and_clear_bit(LPC31xx_MMC_CARD_NEED_INIT, &slot->flags)))
	    cmdflags |= SDMMC_CMD_INIT; //this is the first command, let set send the initializtion clock
	
	if (data) //we may need to move this code to mci_start_command
		lpc31xx_mci_submit_data(host, data);

	if (mrq->stop) 
		host->stop_cmdr = lpc31xx_mci_prepare_command(slot->mmc, mrq->stop);

	lpc31xx_mci_start_command(host, cmd, cmdflags);
}



static void lpc31xx_mci_queue_request(struct lpc31xx_mci *host,
		struct lpc31xx_mci_slot *slot, struct mmc_request *mrq)
{
	dev_vdbg(&slot->mmc->class_dev, "queue request: state=%d\n",
			host->state);

	//printk("#");
	spin_lock(&host->lock);
	slot->mrq = mrq;
	if (host->state == STATE_IDLE) {
		host->state = STATE_SENDING_CMD;
		lpc31xx_mci_start_request(host, slot);
	} else {
		list_add_tail(&slot->queue_node, &host->queue);
	}
	spin_unlock(&host->lock);
}


static void lpc31xx_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct lpc31xx_mci_slot	*slot = mmc_priv(mmc);
	struct lpc31xx_mci	*host = slot->host;

	WARN_ON(slot->mrq);

	if (!test_bit(LPC31xx_MMC_CARD_PRESENT, &slot->flags)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	lpc31xx_mci_queue_request(host, slot, mrq);
}

static void lpc31xx_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lpc31xx_mci_slot	*slot = mmc_priv(mmc);

	slot->ctype = 0; // set default 1 bit mode
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
		slot->ctype = 0;
		break;
	case MMC_BUS_WIDTH_4:
		slot->ctype = SDMMC_CTYPE_4BIT;
		break;
	}


	if (ios->clock) {
		spin_lock(&slot->host->lock);
		/*
		 * Use mirror of ios->clock to prevent race with mmc
		 * core ios update when finding the minimum.
		 */
		slot->clock = ios->clock;

		spin_unlock(&slot->host->lock);
	} else {
		spin_lock(&slot->host->lock);
		slot->clock = 0;
		spin_unlock(&slot->host->lock);
	}

	switch (ios->power_mode) {
	case MMC_POWER_UP:
		set_bit(LPC31xx_MMC_CARD_NEED_INIT, &slot->flags);
		break;
	default:
		break;
	}
}



static int lpc31xx_mci_get_wp(struct mmc_host *mmc)
{
	int			read_only = -ENOSYS;
	struct lpc31xx_mci_slot	*slot = mmc_priv(mmc);

	if (gpio_is_valid(slot->gpio_wp)) {
		read_only =  gpio_get_value(slot->gpio_wp);
		dev_dbg(&mmc->class_dev, "card is %s\n",
				read_only ? "read-only" : "read-write");
	}
	return read_only;
}


static int lpc31xx_mci_get_cd(struct lpc31xx_mci_slot *slot)
{
	int			present = -ENOSYS;

	if (gpio_is_valid(slot->gpio_cd)) {
		present = !gpio_get_value(slot->gpio_cd);
		dev_dbg(&slot->mmc->class_dev, "card is %spresent\n", present ? "" : "not ");
	}
	return present;
}

static void lpc31xx_mci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct lpc31xx_mci_slot	*slot = mmc_priv(mmc);
	struct lpc31xx_mci	*host = slot->host;
	unsigned int reg;

	if (enable) {
		reg = mci_readl(host, SDMMC_INTMASK) | SDMMC_INT_SDIO;
		mci_writel(host, SDMMC_INTMASK, reg);
	}
	else {
		reg = mci_readl(host, SDMMC_INTMASK) & ~SDMMC_INT_SDIO;
		mci_writel(host, SDMMC_INTMASK, reg);
	}
}

static const struct mmc_host_ops lpc31xx_mci_ops = {
	.request	= lpc31xx_mci_request,
	.set_ios	= lpc31xx_mci_set_ios,
	.get_ro		= lpc31xx_mci_get_wp,
	.enable_sdio_irq= lpc31xx_mci_enable_sdio_irq,
};

static void lpc31xx_mci_request_end(struct lpc31xx_mci *host, struct mmc_request *mrq)
	__releases(&host->lock)
	__acquires(&host->lock)
{
	struct lpc31xx_mci_slot	*slot = NULL;
	struct mmc_host		*prev_mmc = host->cur_slot->mmc;

	WARN_ON(host->cmd || host->data);

	host->cur_slot->mrq = NULL;
	host->mrq = NULL;
	if (!list_empty(&host->queue)) {
		slot = list_entry(host->queue.next,
				struct lpc31xx_mci_slot, queue_node);
		list_del(&slot->queue_node);
		dev_vdbg(&host->pdev->dev, "list not empty: %s is next\n",
				mmc_hostname(slot->mmc));
		host->state = STATE_SENDING_CMD;
		lpc31xx_mci_start_request(host, slot);
	} else {
		dev_vdbg(&host->pdev->dev, "list empty\n");
		host->state = STATE_IDLE;
	}

	//printk("-");

	spin_unlock(&host->lock);
	mmc_request_done(prev_mmc, mrq);

	spin_lock(&host->lock);
}

static void lpc31xx_mci_command_complete(struct lpc31xx_mci *host,
			struct mmc_command *cmd)
{
	uint32_t		status = host->cmd_status;

	host->cmd_status = 0;

	if(cmd->flags & MMC_RSP_PRESENT) {

	    if(cmd->flags & MMC_RSP_136) {

		/* Read the response from the card (up to 16 bytes).
		 * LPC31xx MMC controller saves bits 127-96 in RESP3
		 * for easy parsing. But the UNSTUFF_BITS macro in core/mmc.c
		 * core/sd.c expect those bits be in resp[0]. Hence
		 * reverse the response word order.
		 */
		cmd->resp[3] = mci_readl(host, SDMMC_RESP0);
		cmd->resp[2] = mci_readl(host, SDMMC_RESP1);
		cmd->resp[1] = mci_readl(host, SDMMC_RESP2);
		cmd->resp[0] = mci_readl(host, SDMMC_RESP3);
	    } else {
	        cmd->resp[0] = mci_readl(host, SDMMC_RESP0);
		cmd->resp[1] = 0;
		cmd->resp[2] = 0;
		cmd->resp[3] = 0;
	    }
	}

	if (status & SDMMC_INT_RTO)
		cmd->error = -ETIMEDOUT;
	else if ((cmd->flags & MMC_RSP_CRC) && (status & SDMMC_INT_RCRC))
		cmd->error = -EILSEQ;
	else if (status & SDMMC_INT_RESP_ERR)
		cmd->error = -EIO;
	else
		cmd->error = 0;

	if (cmd->error) {
		dev_vdbg(&host->pdev->dev,
			"command error: status=0x%08x resp=0x%08x\n"
			"cmd=0x%08x arg=0x%08x flg=0x%08x err=%d\n",
			status, cmd->resp[0],
			cmd->opcode, cmd->arg, cmd->flags, cmd->error);

		if (cmd->data) {
			host->data = NULL;
			lpc31xx_mci_stop_dma(host);
		}
	}
}

static void lpc31xx_mci_tasklet_func(unsigned long priv)
{
	struct lpc31xx_mci	*host = (struct lpc31xx_mci *)priv;
	struct mmc_request	*mrq = host->mrq;
	struct mmc_data		*data = host->data;
	struct mmc_command	*cmd = host->cmd;
	enum lpc31xx_mci_state	state = host->state;
	enum lpc31xx_mci_state	prev_state;
	uint32_t			status;

	spin_lock(&host->lock);

	state = host->state;
#if 0
	dev_vdbg(&host->pdev->dev,
		"tasklet: state %u pending/completed/mask %lx/%lx/%x\n",
		state, host->pending_events, host->completed_events,
		mci_readl(host, IMR)); // check reg
#endif
	do {
		prev_state = state;

		switch (state) {
		case STATE_IDLE:
			break;

		case STATE_SENDING_CMD:
			if (!lpc31xx_mci_test_and_clear_pending(host,
						EVENT_CMD_COMPLETE))
				break;

			host->cmd = NULL;
			lpc31xx_mci_set_completed(host, EVENT_CMD_COMPLETE);
			lpc31xx_mci_command_complete(host, mrq->cmd);
			if (!mrq->data || cmd->error) {
				lpc31xx_mci_request_end(host, host->mrq);
				goto unlock;
			}

			prev_state = state = STATE_SENDING_DATA;
			/* fall through */

		case STATE_SENDING_DATA:
			if (lpc31xx_mci_test_and_clear_pending(host,
						EVENT_DATA_ERROR)) {
				lpc31xx_mci_stop_dma(host);
				if (data->stop)
					send_stop_cmd(host, data);
				state = STATE_DATA_ERROR;
				break;
			}

			if (!lpc31xx_mci_test_and_clear_pending(host,
						EVENT_XFER_COMPLETE))
				break;

			lpc31xx_mci_set_completed(host, EVENT_XFER_COMPLETE);
			prev_state = state = STATE_DATA_BUSY;
			/* fall through */

		case STATE_DATA_BUSY:
			if (!lpc31xx_mci_test_and_clear_pending(host,
						EVENT_DATA_COMPLETE))
				break;

			host->data = NULL;
			lpc31xx_mci_set_completed(host, EVENT_DATA_COMPLETE);
			status = host->data_status;

			if (unlikely(status & LPC31xx_MCI_DATA_ERROR_FLAGS)) {
				if (status & SDMMC_INT_DTO) {
					dev_err(&host->pdev->dev,
							"data timeout error\n");
					data->error = -ETIMEDOUT;
				} else if (status & SDMMC_INT_DCRC) {
					dev_err(&host->pdev->dev,
							"data CRC error\n");
					data->error = -EILSEQ;
				} else {
					dev_err(&host->pdev->dev,
						"data FIFO error (status=%08x)\n",
						status);
					data->error = -EIO;
				}
			}
			else {
				data->bytes_xfered = data->blocks * data->blksz;
				data->error = 0;
			}

			if (!data->stop) {
				lpc31xx_mci_request_end(host, host->mrq);
				goto unlock;
			}

			prev_state = state = STATE_SENDING_STOP;
			if (!data->error)
				send_stop_cmd(host, data);
			/* fall through */

		case STATE_SENDING_STOP:
			if (!lpc31xx_mci_test_and_clear_pending(host,
						EVENT_CMD_COMPLETE))
				break;

			host->cmd = NULL;
			lpc31xx_mci_command_complete(host, mrq->stop);
			lpc31xx_mci_request_end(host, host->mrq);
			goto unlock;
		case STATE_DATA_ERROR:
			if (!lpc31xx_mci_test_and_clear_pending(host,
						EVENT_XFER_COMPLETE))
				break;

			state = STATE_DATA_BUSY;
			break;
		}
	} while (state != prev_state);

	host->state = state;

unlock:
	spin_unlock(&host->lock);

}



inline static void lpc31xx_mci_push_data(struct lpc31xx_mci *host, void *buf, int cnt)
{
    uint32_t* pData = (uint32_t*)buf;

    if (cnt % 4 != 0)
	    printk("error not align 4\n");

    cnt = cnt >> 2;
    while (cnt > 0) {
        mci_writel(host, SDMMC_DATA, *pData++);
        cnt--;
    }
}

inline static void lpc31xx_mci_pull_data(struct lpc31xx_mci *host, void *buf,int cnt)
{
    uint32_t* pData = (uint32_t*)buf;

    if (cnt % 4 != 0)
	    printk("error not align 4\n");
    cnt = cnt >> 2;
    while (cnt > 0) {
        *pData++ = mci_readl(host, SDMMC_DATA);
        cnt--;
    }
}

static void lpc31xx_mci_read_data_pio(struct lpc31xx_mci *host)
{
	struct scatterlist	*sg = host->sg;
	void			*buf = sg_virt(sg);
	unsigned int		offset = host->pio_offset;
	struct mmc_data		*data = host->data;
	uint32_t			status;
	unsigned int		nbytes = 0,len,old_len,count =0;

	do {
		len = SDMMC_GET_FCNT(mci_readl(host, SDMMC_STATUS)) << 2;
		if(count == 0)
			old_len = len;
		if (likely(offset + len <= sg->length)) {
			lpc31xx_mci_pull_data(host, (void *)(buf + offset),len);

			offset += len;
			nbytes += len;

			if (offset == sg->length) {
				flush_dcache_page(sg_page(sg));
				host->sg = sg = sg_next(sg);
				if (!sg)
					goto done;
				offset = 0;
				buf = sg_virt(sg);
			}
		} else {
			unsigned int remaining = sg->length - offset;
			lpc31xx_mci_pull_data(host, (void *)(buf + offset),remaining);
			nbytes += remaining;

			flush_dcache_page(sg_page(sg));
			host->sg = sg = sg_next(sg);
			if (!sg)
				goto done;
			offset = len - remaining;
			buf = sg_virt(sg);
			lpc31xx_mci_pull_data(host, buf,offset);
			nbytes += offset;
		}

		status = mci_readl(host, SDMMC_MINTSTS);
		mci_writel(host, SDMMC_RINTSTS,SDMMC_INT_RXDR); // clear RXDR interrupt
		if (status & LPC31xx_MCI_DATA_ERROR_FLAGS) {
			host->data_status = status;
			data->bytes_xfered += nbytes;
			smp_wmb();
			lpc31xx_mci_set_pending(host, EVENT_DATA_ERROR);
			tasklet_schedule(&host->tasklet);
			return;
		}
		count ++;
	} while (status & SDMMC_INT_RXDR); // if the RXDR is ready let read again
	len = SDMMC_GET_FCNT(mci_readl(host, SDMMC_STATUS));
	host->pio_offset = offset;
	data->bytes_xfered += nbytes;
	return;

done:
	data->bytes_xfered += nbytes;
	smp_wmb();
	lpc31xx_mci_set_pending(host, EVENT_XFER_COMPLETE);
}

static void lpc31xx_mci_write_data_pio(struct lpc31xx_mci *host)
{
	struct scatterlist	*sg = host->sg;
	void			*buf = sg_virt(sg);
	unsigned int		offset = host->pio_offset;
	struct mmc_data		*data = host->data;
	uint32_t			status;
	unsigned int		nbytes = 0,len;

	do {

		len = SDMMC_FIFO_SZ - (SDMMC_GET_FCNT(mci_readl(host, SDMMC_STATUS)) << 2);
		if (likely(offset + len <= sg->length)) {
			lpc31xx_mci_push_data(host, (void *)(buf + offset),len);

			offset += len;
			nbytes += len;
			if (offset == sg->length) {
				host->sg = sg = sg_next(sg);
				if (!sg)
					goto done;

				offset = 0;
				buf = sg_virt(sg);
			}
		} else {
			unsigned int remaining = sg->length - offset;

			lpc31xx_mci_push_data(host, (void *)(buf + offset), remaining);
			nbytes += remaining;

			host->sg = sg = sg_next(sg);
			if (!sg) {
				goto done;
			}

			offset = len - remaining;
			buf = sg_virt(sg);
			lpc31xx_mci_push_data(host, (void *)buf, offset);
			nbytes += offset;
		}

		status = mci_readl(host, SDMMC_MINTSTS);
		mci_writel(host, SDMMC_RINTSTS,SDMMC_INT_TXDR); // clear RXDR interrupt
		if (status & LPC31xx_MCI_DATA_ERROR_FLAGS) {
			host->data_status = status;
			data->bytes_xfered += nbytes;
			smp_wmb();
			lpc31xx_mci_set_pending(host, EVENT_DATA_ERROR);
			tasklet_schedule(&host->tasklet);
			return;
		}
	} while (status & SDMMC_INT_TXDR); // if TXDR, let write again

	host->pio_offset = offset;
	data->bytes_xfered += nbytes;

	return;

done:
	data->bytes_xfered += nbytes;
	smp_wmb();
	lpc31xx_mci_set_pending(host, EVENT_XFER_COMPLETE);
}

static void lpc31xx_mci_cmd_interrupt(struct lpc31xx_mci *host, uint32_t status)
{
	if(!host->cmd_status)
		host->cmd_status = status;

	smp_wmb();
	lpc31xx_mci_set_pending(host, EVENT_CMD_COMPLETE);
	tasklet_schedule(&host->tasklet);
}

static irqreturn_t lpc31xx_mci_interrupt(int irq, void *dev_id)
{
	struct lpc31xx_mci	*host = dev_id;
	uint32_t			status,  pending;
	unsigned int		pass_count = 0;

	spin_lock(&host->lock);
	do {
		status = mci_readl(host, SDMMC_RINTSTS);
		pending = mci_readl(host, SDMMC_MINTSTS);// read only mask reg
		if (!pending)
			break;
		if(pending & LPC31xx_MCI_CMD_ERROR_FLAGS) {
		    mci_writel(host, SDMMC_RINTSTS,LPC31xx_MCI_CMD_ERROR_FLAGS);  //  clear interrupt
		    host->cmd_status = status;
		    smp_wmb();
		    lpc31xx_mci_set_pending(host, EVENT_CMD_COMPLETE);
		    tasklet_schedule(&host->tasklet);
		}

		if (pending & LPC31xx_MCI_DATA_ERROR_FLAGS) { // if there is an error, let report DATA_ERROR
			mci_writel(host, SDMMC_RINTSTS,LPC31xx_MCI_DATA_ERROR_FLAGS);  // clear interrupt
			host->data_status = status;
			smp_wmb();
			lpc31xx_mci_set_pending(host, EVENT_DATA_ERROR);
			tasklet_schedule(&host->tasklet);
		}


		if(pending & SDMMC_INT_DATA_OVER) {
		    mci_writel(host, SDMMC_RINTSTS,SDMMC_INT_DATA_OVER);  // clear interrupt
		    if (!host->data_status)
			host->data_status = status;
		    smp_wmb();
		    if(host->dir_status == LPC31xx_MCI_RECV_STATUS) {
			if(host->sg != NULL)
				lpc31xx_mci_read_data_pio(host);
		    }
		    lpc31xx_mci_set_pending(host, EVENT_DATA_COMPLETE);
		    tasklet_schedule(&host->tasklet);
		}

		if (pending & SDMMC_INT_RXDR) {
		    mci_writel(host, SDMMC_RINTSTS,SDMMC_INT_RXDR);  //  clear interrupt
		    if(host->sg)
			    lpc31xx_mci_read_data_pio(host);
		}

		if (pending & SDMMC_INT_TXDR) {
		    mci_writel(host, SDMMC_RINTSTS,SDMMC_INT_TXDR);  //  clear interrupt
		    if(host->sg) {
			lpc31xx_mci_write_data_pio(host);
		    }
		}

		if (pending & SDMMC_INT_CMD_DONE) {
		    mci_writel(host, SDMMC_RINTSTS,SDMMC_INT_CMD_DONE);  //  clear interrupt
		    lpc31xx_mci_cmd_interrupt(host, status);
		}

		if (pending & SDMMC_INT_SDIO) {
		    mci_writel(host, SDMMC_RINTSTS,SDMMC_INT_SDIO);  //  clear interrupt
		    mmc_signal_sdio_irq(host->cur_slot->mmc);
		}
	} while (pass_count++ < 5);

	spin_unlock(&host->lock);

	return pass_count ? IRQ_HANDLED : IRQ_NONE;
}

/*
 *
 * MMC card detect thread, kicked off from detect interrupt, 1 timer per slot
 *
 */
static void lpc31xx_mci_detect_change(unsigned long slot_data)
{
	struct lpc31xx_mci_slot *slot = (struct lpc31xx_mci_slot *) slot_data;
	struct lpc31xx_mci *host;
	struct mmc_request *mrq;
	bool present;
	bool present_old;

	host = slot->host;
	/*
	 * lpc31xx_mci_cleanup_slot() sets the ATMCI_SHUTDOWN flag before
	 * freeing the interrupt. We must not re-enable the interrupt
	 * if it has been freed, and if we're shutting down, it
	 * doesn't really matter whether the card is present or not.
	 */
	smp_rmb();
	if (test_bit(LPC31xx_MMC_SHUTDOWN, &slot->flags))
		return;

	enable_irq(slot->irq);
	present = lpc31xx_mci_get_cd(slot);
	present_old = test_bit(LPC31xx_MMC_CARD_PRESENT, &slot->flags);
	dev_dbg(&slot->mmc->class_dev, "detect change: %d (was %d)\n",
			present, present_old);

	if (present != present_old) {

		dev_info(&slot->mmc->class_dev, "card %s\n",
			present ? "inserted" : "removed");

		spin_lock(&host->lock);

		lpc31xx_mci_set_power(slot, present);
		if (present) {
			set_bit(LPC31xx_MMC_CARD_PRESENT, &slot->flags);
			set_bit(LPC31xx_MMC_CARD_NEED_INIT, &slot->flags);
		} else {
			clear_bit(LPC31xx_MMC_CARD_PRESENT, &slot->flags);
		}


		/* Clean up queue if present */
		mrq = slot->mrq;
		if (mrq) {
			if (mrq == host->mrq) {
			  	/* reset all blocks */
			  	mci_writel(host, SDMMC_CTRL,(SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));
			  	/* wait till resets clear */
			  	while (mci_readl(host, SDMMC_CTRL) & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));

				host->data = NULL;
				host->cmd = NULL;

				switch (host->state) {
				case STATE_IDLE:
					break;
				case STATE_SENDING_CMD:
					mrq->cmd->error = -ENOMEDIUM;
					if (!mrq->data)
						break;
					/* fall through */
				case STATE_SENDING_DATA:
					mrq->data->error = -ENOMEDIUM;
					lpc31xx_mci_stop_dma(host);
					break;
				case STATE_DATA_BUSY:
				case STATE_DATA_ERROR:
					if (mrq->data->error == -EINPROGRESS)
						mrq->data->error = -ENOMEDIUM;
					if (!mrq->stop)
						break;
					/* fall through */
				case STATE_SENDING_STOP:
					mrq->stop->error = -ENOMEDIUM;
					break;
				}

				lpc31xx_mci_request_end(host, mrq);
			} else {
				list_del(&slot->queue_node);
				mrq->cmd->error = -ENOMEDIUM;
				if (mrq->data)
					mrq->data->error = -ENOMEDIUM;
				if (mrq->stop)
					mrq->stop->error = -ENOMEDIUM;

				spin_unlock(&host->lock);
				mmc_request_done(slot->mmc, mrq);
				spin_lock(&host->lock);
			}

		}

		spin_unlock(&host->lock);
		mmc_detect_change(slot->mmc, 0);
	}
}

static irqreturn_t lpc31xx_mci_detect_interrupt(int irq, void *dev_id)
{
	int level;
	struct lpc31xx_mci_slot	*slot = dev_id;

	/* select the opposite level sensitivity */
	level =  lpc31xx_mci_get_cd(slot) ? IRQ_TYPE_LEVEL_HIGH : IRQ_TYPE_LEVEL_LOW;
	irq_set_irq_type(slot->irq, level);

	/*
	 * Disable interrupts until the pin has stabilized and check
	 * the state then. Use mod_timer() since we may be in the
	 * middle of the timer routine when this interrupt triggers.
	 */
	disable_irq_nosync(irq);
	mod_timer(&slot->detect_timer, jiffies + msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

static int __init
lpc31xx_mci_init_slot(struct lpc31xx_mci *host, struct device_node *np)
{
	struct mmc_host			*mmc;
	struct lpc31xx_mci_slot		*slot;
	const uint32_t *voltage_ranges;
	const int *width;
	int i, ret, num_ranges, level;
	enum of_gpio_flags flags;

	mmc = mmc_alloc_host(sizeof(struct lpc31xx_mci_slot), &host->pdev->dev);

	if (!mmc)
		return -ENOMEM;

	slot = mmc_priv(mmc);
	slot->id = host->slot_count++;

	slot->mmc = mmc;
	slot->host = host;

	slot->gpio_cd = of_get_named_gpio_flags(np, "gpios", 0, &flags);
	if (gpio_is_valid(slot->gpio_cd)) {
		gpio_request(slot->gpio_cd, "mmc cd");
		gpio_direction_input(slot->gpio_cd);
	}
	slot->gpio_wp = of_get_named_gpio_flags(np, "gpios", 1, &flags);
	if (gpio_is_valid(slot->gpio_wp)) {
		gpio_request(slot->gpio_wp, "mmc wp");
		gpio_direction_input(slot->gpio_wp);
	}
	slot->gpio_power = of_get_named_gpio_flags(np, "gpios", 2, &flags);
	if (gpio_is_valid(slot->gpio_power)) {
		gpio_request(slot->gpio_power, "mmc power");
		gpio_direction_output(slot->gpio_power, 1);
	}
	slot->gpio_select = of_get_named_gpio_flags(np, "gpios", 3, &flags);
	if (gpio_is_valid(slot->gpio_select)) {
		gpio_request(slot->gpio_select, "mmc select");
		gpio_direction_input(slot->gpio_select);
	}

	mmc->ops = &lpc31xx_mci_ops;
	mmc->f_min = DIV_ROUND_UP(host->bus_hz, 510);
	mmc->f_max = host->bus_hz/2; //max f is clock to mmc_clk/2

	voltage_ranges = of_get_property(np, "voltage-ranges", &num_ranges);
	num_ranges = num_ranges / sizeof(*voltage_ranges) / 2;
	if (!voltage_ranges || !num_ranges) {
		dev_err(&host->pdev->dev, "OF: voltage-ranges unspecified\n");
		ret = -EINVAL;
		goto err_ocr;
	}

	for (i = 0; i < num_ranges; i++) {
		const int j = i * 2;
		uint32_t mask;

		mask = mmc_vddrange_to_ocrmask(be32_to_cpu(voltage_ranges[j]),
					       be32_to_cpu(voltage_ranges[j + 1]));
		if (!mask) {
			ret = -EINVAL;
			dev_err(&host->pdev->dev, "OF: voltage-range #%d is invalid\n", i);
			goto err_ocr;
		}
		mmc->ocr_avail |= mask;
	}

	mmc->caps = MMC_CAP_SDIO_IRQ;
	width = of_get_property(np, "width", NULL);
	if (*width == 4)
		mmc->caps |= MMC_CAP_4_BIT_DATA;
	else if (*width == 8)
		mmc->caps |= MMC_CAP_8_BIT_DATA;

	mmc->max_blk_size = 65536; /* BLKSIZ is 16 bits*/
	mmc->max_blk_count = 512;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size = mmc->max_req_size;

	/* Create card detect handler thread for the slot */
	setup_timer(&slot->detect_timer, lpc31xx_mci_detect_change,
			(unsigned long)slot);

	slot->irq = irq_of_parse_and_map(np, 0);
	/* select the opposite level sensitivity */
	level =  lpc31xx_mci_get_cd(slot) ? IRQ_TYPE_LEVEL_HIGH : IRQ_TYPE_LEVEL_LOW;

	if(lpc31xx_mci_get_cd(slot)) {
		lpc31xx_mci_set_power(slot, 1);
		set_bit(LPC31xx_MMC_CARD_PRESENT, &slot->flags);
		set_bit(LPC31xx_MMC_CARD_NEED_INIT, &slot->flags);
	} else {
		lpc31xx_mci_set_power(slot, 0);
		clear_bit(LPC31xx_MMC_CARD_PRESENT, &slot->flags);
	}

	host->slot[host->slot_count++] = slot;
	mmc_add_host(mmc);

#if defined (CONFIG_DEBUG_FS)
	lpc31xx_mci_init_debugfs(slot);
#endif
	/* set card detect irq info */
	irq_set_irq_type(slot->irq, level);
	ret = request_irq(slot->irq,
			lpc31xx_mci_detect_interrupt,
			level,
			"mmc-cd",
			slot);
	/****temporary for PM testing */
	enable_irq_wake(slot->irq);

	return 0;
err_ocr:
	return ret;
}

static void lpc31xx_mci_cleanup_slot(struct lpc31xx_mci_slot *slot,
		unsigned int id)
{
	/* Shutdown detect IRQ and kill detect thread */
	free_irq(slot->irq, slot);

	del_timer_sync(&slot->detect_timer);

	/* Debugfs stuff is cleaned up by mmc core */
	set_bit(LPC31xx_MMC_SHUTDOWN, &slot->flags);
	smp_wmb();
	mmc_remove_host(slot->mmc);
	slot->host->slot[id] = NULL;
	mmc_free_host(slot->mmc);
}

static const struct of_device_id lpc31xx_mci_of_match[] = {
	{ .compatible = "nxp,lpc31xx-sdmmc" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc31xx_mci_of_match);

static int lpc31xx_mci_probe(struct platform_device *pdev)
{
	struct device_node *node;
	struct device_node *np = pdev->dev.of_node;
	struct lpc31xx_mci *host;
	struct resource *regs;
	struct clk *clk;
	int irq;
	int ret = 0;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	host = kzalloc(sizeof(struct lpc31xx_mci), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->pdev = pdev;

	spin_lock_init(&host->lock);
	INIT_LIST_HEAD(&host->queue);

	ret = -ENOMEM;
	host->regs = ioremap(regs->start, regs->end - regs->start);
	if (!host->regs)
	    goto err_freehost;

	/* enable the clock to MCI module */
	clk = clk_get(NULL, "mmc_hclk");
	ret = clk_enable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "mmc_cclk_in");
	ret = clk_enable(clk);
	clk_put(clk);

	/* reset SD/MMC/MCI modules through CGU */
	/* clear and set the register */
	CGU_CFG->resetn_soft[SD_MMC_PNRES_SOFT] = 0;
	CGU_CFG->resetn_soft[SD_MMC_NRES_CCLK_IN_SOFT] = 0;
	/* introduce some delay */
	udelay(1);
	CGU_CFG->resetn_soft[SD_MMC_NRES_CCLK_IN_SOFT] = CGU_CONFIG_SOFT_RESET;
	CGU_CFG->resetn_soft[SD_MMC_PNRES_SOFT] = CGU_CONFIG_SOFT_RESET;

#ifdef USE_DMA
	host->dma_chn = dma_request_sg_channel("MCI",  lpc31xx_mci_dma_complete, host,
			0, 0, 1);
	if(host->dma_chn < 0) {
		dev_err(&pdev->dev, "Failed to allocate DMA SG channel\n");
		printk(KERN_CRIT "Failed to allocate DMA SG channel\n");
		ret = host->dma_chn;
		goto err_freemap;
	}

	host->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &host->sg_dma, GFP_KERNEL);
	if (host->sg_cpu == NULL) {
		dev_err(&pdev->dev,
			 "%s: could not alloc dma memory \n", __func__);
		ret = -ENOMEM;
		goto err_freemap;
	}
#endif

	clk = clk_get(NULL, "mmc_cclk_in");
	host->bus_hz = clk_get_rate(clk); //40000000;
	clk_put(clk);

	/* Set IOCONF to MCI pins */
	SYS_SDMMC_DELAYMODES = 0;
	SYS_MUX_GPIO_MCI = 1;

	/* set the pins as driven by IP in IOCONF */
#if 0
	/* fixme */
	GPIO_DRV_IP(IOCONF_EBI_MCI, 0xF0000003);
#endif

	/* set delay gates */
	SYS_SDMMC_DELAYMODES = 0x1B;

  	/* reset all blocks */
  	mci_writel(host, SDMMC_CTRL,(SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));
  	/* wait till resets clear */
  	while (mci_readl(host, SDMMC_CTRL) & (SDMMC_CTRL_RESET | SDMMC_CTRL_FIFO_RESET | SDMMC_CTRL_DMA_RESET));

	 /* Clear the interrupts for the host controller */
	mci_writel(host, SDMMC_RINTSTS, 0xFFFFFFFF);
	mci_writel(host, SDMMC_INTMASK, 0); // disable all mmc interrupt first

  	/* Put in max timeout */
  	mci_writel(host, SDMMC_TMOUT, 0xFFFFFFFF);

  	/* FIFO threshold settings  */
#ifdef BURST_DMA
	mci_writel(host, SDMMC_FIFOTH, ((0x1 << 28) | (0xB << 16) | (0xC << 0))); // RXMark = 11, TXMark = 12, DMA Size = 4
#else
  	mci_writel(host, SDMMC_FIFOTH, ((0x2 << 28) | (0x10 << 16) | (0x10 << 0))); // RXMark = 16, TXMark = 16, DMA Size = 8
#endif

	/* disable clock to CIU */
	mci_writel(host, SDMMC_CLKENA,0);
	mci_writel(host, SDMMC_CLKSRC,0);

	tasklet_init(&host->tasklet, lpc31xx_mci_tasklet_func, (unsigned long)host);
	ret = request_irq(irq, lpc31xx_mci_interrupt, 0, dev_name(&pdev->dev), host);
	if (ret)
	    goto err_dmaunmap;

	platform_set_drvdata(pdev, host);

	// enable interrupt for command done, data over, data empty, receive ready and error such as transmit, receive timeout, crc error
	mci_writel(host, SDMMC_RINTSTS, 0xFFFFFFFF);
	mci_writel(host, SDMMC_INTMASK,SDMMC_INT_CMD_DONE | SDMMC_INT_DATA_OVER | SDMMC_INT_TXDR | SDMMC_INT_RXDR | LPC31xx_MCI_ERROR_FLAGS);
	mci_writel(host, SDMMC_CTRL,SDMMC_CTRL_INT_ENABLE); // enable mci interrupt

	for_each_child_of_node(np, node) {
		ret = lpc31xx_mci_init_slot(host, node);
		if (ret) {
			ret = -ENODEV;
			goto err_init_slot;
		}
	}

	dev_info(&pdev->dev, "LPC31xx MMC controller at irq %d\n", irq);

	return 0;

err_init_slot:
	/* De-init any initialized slots */
	while (host->slot_count > 0) {
		if (host->slot[host->slot_count])
			lpc31xx_mci_cleanup_slot(host->slot[host->slot_count], host->slot_count);
		host->slot_count--;
	}
	free_irq(irq, host);
err_dmaunmap:
#ifdef USE_DMA
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	dma_release_sg_channel(host->dma_chn);
err_freemap:
#endif
	iounmap(host->regs);
err_freehost:
	kfree(host);
	return ret;
}



static int __exit lpc31xx_mci_remove(struct platform_device *pdev)
{
	struct lpc31xx_mci *host = platform_get_drvdata(pdev);
	struct clk *clk;
	int i;

	mci_writel(host, SDMMC_RINTSTS, 0xFFFFFFFF);
	mci_writel(host, SDMMC_INTMASK, 0); // disable all mmc interrupt first

	platform_set_drvdata(pdev, NULL);

	for (i = 0; i < host->slot_count; i++) {
		dev_dbg(&pdev->dev, "remove slot %d\n", i);
		if (host->slot[i])
			lpc31xx_mci_cleanup_slot(host->slot[i], i);
	}

	/* disable clock to CIU */
	mci_writel(host, SDMMC_CLKENA,0);
	mci_writel(host, SDMMC_CLKSRC,0);

	/*  turn off the mci clock here */
	clk = clk_get(NULL, "mmc_hclk");
	clk_disable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "mmc_cclk_in");
	clk_disable(clk);
	clk_put(clk);

	free_irq(platform_get_irq(pdev, 0), host);
#ifdef USE_DMA
	dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
	dma_release_sg_channel(host->dma_chn);
#endif
	iounmap(host->regs);

	kfree(host);
	return 0;
}

static int lpc31xx_mci_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM
	struct lpc31xx_mci *host = platform_get_drvdata(pdev);
	struct clk *clk;

	/* Disable Card clock */
	mci_writel(host, SDMMC_CLKENA, 0);

	/* Disable IP clocks */
	clk = clk_get(NULL, "mmc_hclk");
	clk_disable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "mmc_cclk_in");
	clk_disable(clk);
	clk_put(clk);
#endif
	return 0;
}

static int lpc31xx_mci_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	struct lpc31xx_mci *host = platform_get_drvdata(pdev);
	struct clk *clk;
	int ret;

	/* Enable IP Clocks */
	clk = clk_get(NULL, "mmc_hclk");
	ret = clk_enable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "mmc_cclk_in");
	ret = clk_enable(clk);
	clk_put(clk);

	/* Enable Card clock */
	mci_writel(host, SDMMC_CLKENA ,SDMMC_CLKEN_ENABLE);
#endif
	return 0;
}

static struct platform_driver lpc31xx_mci_driver = {
	.suspend    = lpc31xx_mci_suspend,
	.resume     = lpc31xx_mci_resume,
	.remove		= __exit_p(lpc31xx_mci_remove),
	.driver		= {
		.name	= "lpc31xx_mmc",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lpc31xx_mci_of_match,
#endif
	},
};

static int __init lpc31xx_mci_init(void)
{
	return platform_driver_probe(&lpc31xx_mci_driver, lpc31xx_mci_probe);
}

static void __exit lpc31xx_mci_exit(void)
{
	platform_driver_unregister(&lpc31xx_mci_driver);
}

module_init(lpc31xx_mci_init);
module_exit(lpc31xx_mci_exit);

MODULE_DESCRIPTION("LPC31xx Multimedia Card Interface driver");
MODULE_AUTHOR("NXP Semiconductor VietNam");
MODULE_LICENSE("GPL v2");
