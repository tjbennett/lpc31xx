/*
 * A driver for the LPC31xx SPI bus master.
 *
 *
 * Initial version inspired by:
 *	drivers/spi/spi-pl022.c
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/pm_runtime.h>

/***********************************************************************
 * SPI register definitions
 **********************************************************************/
#define SPI_CONFIG_REG    __REG (SPI_PHYS + 0x00)
#define SPI_SLV_ENAB_REG  __REG (SPI_PHYS + 0x04)
#define SPI_TXF_FLUSH_REG __REG (SPI_PHYS + 0x08)
#define SPI_FIFO_DATA_REG __REG (SPI_PHYS + 0x0C)
#define SPI_NHP_POP_REG   __REG (SPI_PHYS + 0x10)
#define SPI_NHP_MODE_REG  __REG (SPI_PHYS + 0x14)
#define SPI_DMA_SET_REG   __REG (SPI_PHYS + 0x18)
#define SPI_STS_REG       __REG (SPI_PHYS + 0x1C)
#define SPI_HWINFO_REG    __REG (SPI_PHYS + 0x20)
#define SPI_SLV_SET1_REG(slv) __REG (SPI_PHYS + 0x24 + (8 * slv))
#define SPI_SLV_SET2_REG(slv) __REG (SPI_PHYS + 0x28 + (8 * slv))
#define SPI_INT_TRSH_REG  __REG (SPI_PHYS + 0xFD4)
#define SPI_INT_CLRE_REG  __REG (SPI_PHYS + 0xFD8)
#define SPI_INT_SETE_REG  __REG (SPI_PHYS + 0xFDC)
#define SPI_INT_STS_REG   __REG (SPI_PHYS + 0xFE0)
#define SPI_INT_ENAB_REG  __REG (SPI_PHYS + 0xFE4)
#define SPI_INT_CLRS_REG  __REG (SPI_PHYS + 0xFE8)
#define SPI_INT_SETS_REG  __REG (SPI_PHYS + 0xFEC)
#define SPI_MOD_ID_REG    __REG (SPI_PHYS + 0xFFC)

/* SPI device contants */
#define SPI_FIFO_DEPTH  64 /* 64 words (16bit) deep */
#define SPI_NUM_SLAVES  3  /* number of slaves supported */
#define SPI_MAX_DIV2    254
#define SPI_MAX_DIVIDER 65024 /* = 254 * (255 + 1) */
#define SPI_MIN_DIVIDER 2

/* SPI Configuration register definitions (SPI_CONFIG_REG) */
#define SPI_CFG_INTER_DLY(n)      _SBF(16, ((n) & 0xFFFF))
#define SPI_CFG_INTER_DLY_GET(n)  (((n) >> 16) & 0xFFFF)
#define SPI_CFG_UPDATE_EN         _BIT(7)
#define SPI_CFG_SW_RESET          _BIT(6)
#define SPI_CFG_SLAVE_DISABLE     _BIT(4)
#define SPI_CFG_MULTI_SLAVE       _BIT(3)
#define SPI_CFG_LOOPBACK          _BIT(2)
#define SPI_CFG_SLAVE_MODE        _BIT(1)
#define SPI_CFG_ENABLE            _BIT(0)

/* SPI slave_enable register definitions (SPI_SLV_ENAB_REG) */
#define SPI_SLV_EN(n)             _SBF(((n) << 1), 0x1)
#define SPI_SLV_SUSPEND(n)        _SBF(((n) << 1), 0x3)

/* SPI tx_fifo_flush register definitions (SPI_TXF_FLUSH_REG) */
#define SPI_TXFF_FLUSH            _BIT(1)

/* SPI dma_settings register definitions (SPI_DMA_SET_REG) */
#define SPI_DMA_TX_EN             _BIT(1)
#define SPI_DMA_RX_EN             _BIT(0)

/* SPI status register definitions (SPI_STS_REG) */
#define SPI_ST_SMS_BUSY           _BIT(5)
#define SPI_ST_BUSY               _BIT(4)
#define SPI_ST_RX_FF              _BIT(3)
#define SPI_ST_RX_EMPTY           _BIT(2)
#define SPI_ST_TX_FF              _BIT(1)
#define SPI_ST_TX_EMPTY           _BIT(0)

/* SPI slv_setting registers definitions (SPI_SLV_SET1_REG) */
#define SPI_SLV1_INTER_TX_DLY(n)  _SBF(24, ((n) & 0xFF))
#define SPI_SLV1_NUM_WORDS(n)     _SBF(16, ((n) & 0xFF))
#define SPI_SLV1_CLK_PS(n)        _SBF(8, ((n) & 0xFF))
#define SPI_SLV1_CLK_PS_GET(n)    (((n) >> 8) & 0xFF)
#define SPI_SLV1_CLK_DIV1(n)      ((n) & 0xFF)
#define SPI_SLV1_CLK_DIV1_GET(n)  ((n) & 0xFF)

/* SPI slv_setting registers definitions (SPI_SLV_SET2_REG) */
#define SPI_SLV2_PPCS_DLY(n)      _SBF(9, ((n) & 0xFF))
#define SPI_SLV2_CS_HIGH          _BIT(8)
#define SPI_SLV2_SSI_MODE         _BIT(7)
#define SPI_SLV2_SPO              _BIT(6)
#define SPI_SLV2_SPH              _BIT(5)
#define SPI_SLV2_WD_SZ(n)         ((n) & 0x1F)

/* SPI int_threshold registers definitions (SPI_INT_TRSH_REG) */
#define SPI_INT_TSHLD_TX(n)       _SBF(8, ((n) & 0xFF))
#define SPI_INT_TSHLD_RX(n)       ((n) & 0xFF)

/* SPI intterrupt registers definitions ( SPI_INT_xxx) */
#define SPI_SMS_INT               _BIT(4)
#define SPI_TX_INT                _BIT(3)
#define SPI_RX_INT                _BIT(2)
#define SPI_TO_INT                _BIT(1)
#define SPI_OVR_INT               _BIT(0)
#define SPI_ALL_INTS              (SPI_SMS_INT | SPI_TX_INT | SPI_RX_INT | SPI_TO_INT | SPI_OVR_INT)

#define SPI_POLLING_TIMEOUT 1000

/*
 * Message State
 * we use the spi_message.state (void *) pointer to
 * hold a single state value, that's why all this
 * (void *) casting is done here.
 */
#define STATE_START			((void *) 0)
#define STATE_RUNNING			((void *) 1)
#define STATE_DONE			((void *) 2)
#define STATE_ERROR			((void *) -1)

/*
 * The type of reading going on on this chip
 */
enum spi_reading {
	READING_NULL,
	READING_U8,
	READING_U16,
	READING_U32
};

/**
 * The type of writing going on on this chip
 */
enum spi_writing {
	WRITING_NULL,
	WRITING_U8,
	WRITING_U16,
	WRITING_U32
};

/**
 * struct vendor_data - vendor-specific config parameters
 * for LPC31xx derivatives
 * @fifodepth: depth of FIFOs (both)
 * @max_bpw: maximum number of bits per word
 * @unidir: supports unidirectional transfers
 * @extended_cr: 32 bit wide control register 0 with extra
 * features and extra features in CR1 as found in the ST variants
 * @pl023: supports a subset of the ST extensions called "PL023"
 */
struct vendor_data {
	int fifodepth;
	int max_bpw;
	bool unidir;
	bool extended_cr;
	bool pl023;
	bool loopback;
};

/**
 * struct lpc31xx - This is the private SSP driver data structure
 * @pdev: Platform device model hookup
 * @vendor: vendor data for the IP block
 * @phybase: the physical memory where the SSP device resides
 * @virtbase: the virtual memory where the SSP is mapped
 * @clk: outgoing clock "SPICLK" for the SPI bus
 * @master: SPI framework hookup
 * @master_info: controller-specific data from machine setup
 * @kworker: thread struct for message pump
 * @kworker_task: pointer to task for message pump kworker thread
 * @pump_messages: work struct for scheduling work to the message pump
 * @queue_lock: spinlock to synchronize access to message queue
 * @queue: message queue
 * @busy: message pump is busy
 * @running: message pump is running
 * @pump_transfers: Tasklet used in Interrupt Transfer mode
 * @cur_msg: Pointer to current spi_message being processed
 * @cur_transfer: Pointer to current spi_transfer
 * @cur_chip: pointer to current clients chip(assigned from controller_state)
 * @next_msg_cs_active: the next message in the queue has been examined
 *  and it was found that it uses the same chip select as the previous
 *  message, so we left it active after the previous transfer, and it's
 *  active already.
 * @tx: current position in TX buffer to be read
 * @tx_end: end position in TX buffer to be read
 * @rx: current position in RX buffer to be written
 * @rx_end: end position in RX buffer to be written
 * @read: the type of read currently going on
 * @write: the type of write currently going on
 * @exp_fifo_level: expected FIFO level
 * @dma_rx_channel: optional channel for RX DMA
 * @dma_tx_channel: optional channel for TX DMA
 * @sgt_rx: scatter table for the RX transfer
 * @sgt_tx: scatter table for the TX transfer
 * @dummypage: a dummy page used for driving data on the bus with DMA
 */
struct lpc31xx {
	struct platform_device		*pdev;
	resource_size_t			phybase;
	void __iomem			*virtbase;
	struct clk			*clk;
	struct spi_master		*master;
	struct lpc31xx_spi_controller	*master_info;
	/* Message per-transfer pump */
	struct tasklet_struct		pump_transfers;
	struct spi_message		*cur_msg;
	struct spi_transfer		*cur_transfer;
	struct chip_data		*cur_chip;
	bool				next_msg_cs_active;
	void				*tx;
	void				*tx_end;
	void				*rx;
	void				*rx_end;
	enum spi_reading		read;
	enum spi_writing		write;
	u32				exp_fifo_level;
	enum spi_rx_level_trig		rx_lev_trig;
	enum spi_tx_level_trig		tx_lev_trig;
	/* DMA settings */
#ifdef CONFIG_DMA_ENGINE_X
	struct dma_chan			*dma_rx_channel;
	struct dma_chan			*dma_tx_channel;
	struct sg_table			sgt_rx;
	struct sg_table			sgt_tx;
	char				*dummypage;
	bool				dma_running;
#endif
};

/**
 * struct chip_data - To maintain runtime state of SSP for each client chip
 * @cr0: Value of control register CR0 of SSP - on later ST variants this
 *       register is 32 bits wide rather than just 16
 * @cr1: Value of control register CR1 of SSP
 * @dmacr: Value of DMA control Register of SSP
 * @cpsr: Value of Clock prescale register
 * @n_bytes: how many bytes(power of 2) reqd for a given data width of client
 * @enable_dma: Whether to enable DMA or not
 * @read: function ptr to be used to read when doing xfer for this chip
 * @write: function ptr to be used to write when doing xfer for this chip
 * @cs_control: chip select callback provided by chip
 * @xfer_type: polling/interrupt/DMA
 *
 * Runtime state of the SSP controller, maintained per chip,
 * This would be set according to the current message that would be served
 */
struct chip_data {
	u32 cr0;
	u16 cr1;
	u16 dmacr;
	u16 cpsr;
	u8 n_bytes;
	bool enable_dma;
	enum spi_reading read;
	enum spi_writing write;
	void (*cs_control) (u32 command);
	int xfer_type;
};

/**
 * null_cs_control - Dummy chip select function
 * @command: select/delect the chip
 *
 * If no chip select function is provided by client this is used as dummy
 * chip select
 */
static void null_cs_control(u32 command)
{
	pr_debug("lpc31xx: dummy chip select control, CS=0x%x\n", command);
}

/**
 * giveback - current spi_message is over, schedule next message and call
 * callback of this message. Assumes that caller already
 * set message->status; dma and pio irqs are blocked
 * @lpc31xx: SSP driver private data structure
 */
static void giveback(struct lpc31xx *lpc31xx)
{
	struct spi_transfer *last_transfer;
	lpc31xx->next_msg_cs_active = false;

	last_transfer = list_entry(lpc31xx->cur_msg->transfers.prev,
					struct spi_transfer,
					transfer_list);

	/* Delay if requested before any change in chip select */
	if (last_transfer->delay_usecs)
		/*
		 * FIXME: This runs in interrupt context.
		 * Is this really smart?
		 */
		udelay(last_transfer->delay_usecs);

	if (!last_transfer->cs_change) {
		struct spi_message *next_msg;

		/*
		 * cs_change was not set. We can keep the chip select
		 * enabled if there is message in the queue and it is
		 * for the same spi device.
		 *
		 * We cannot postpone this until pump_messages, because
		 * after calling msg->complete (below) the driver that
		 * sent the current message could be unloaded, which
		 * could invalidate the cs_control() callback...
		 */
		/* get a pointer to the next message, if any */
		next_msg = spi_get_next_queued_message(lpc31xx->master);

		/*
		 * see if the next and current messages point
		 * to the same spi device.
		 */
		if (next_msg && next_msg->spi != lpc31xx->cur_msg->spi)
			next_msg = NULL;
		if (!next_msg || lpc31xx->cur_msg->state == STATE_ERROR)
			lpc31xx->cur_chip->cs_control(SSP_CHIP_DESELECT);
		else
			lpc31xx->next_msg_cs_active = true;

	}

	lpc31xx->cur_msg = NULL;
	lpc31xx->cur_transfer = NULL;
	lpc31xx->cur_chip = NULL;
	spi_finalize_current_message(lpc31xx->master);
}

/**
 * flush - flush the FIFO to reach a clean state
 * @lpc31xx: SSP driver private data structure
 */
static int flush(struct lpc31xx *lpc31xx)
{
	unsigned long limit = loops_per_jiffy << 1;

	dev_dbg(&lpc31xx->pdev->dev, "flush\n");
	do {
		while (readw(SSP_SR(lpc31xx->virtbase)) & SSP_SR_MASK_RNE)
			readw(SSP_DR(lpc31xx->virtbase));
	} while ((readw(SSP_SR(lpc31xx->virtbase)) & SSP_SR_MASK_BSY) && limit--);

	lpc31xx->exp_fifo_level = 0;

	return limit;
}

/**
 * restore_state - Load configuration of current chip
 * @lpc31xx: SSP driver private data structure
 */
static void restore_state(struct lpc31xx *lpc31xx)
{
	struct chip_data *chip = lpc31xx->cur_chip;

#if JDS
	if (lpc31xx->vendor->extended_cr)
		writel(chip->cr0, SSP_CR0(lpc31xx->virtbase));
	else
		writew(chip->cr0, SSP_CR0(lpc31xx->virtbase));
	writew(chip->cr1, SSP_CR1(lpc31xx->virtbase));
	writew(chip->dmacr, SSP_DMACR(lpc31xx->virtbase));
	writew(chip->cpsr, SSP_CPSR(lpc31xx->virtbase));
	writew(DISABLE_ALL_INTERRUPTS, SSP_IMSC(lpc31xx->virtbase));
	writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(lpc31xx->virtbase));
#endif
}


/**
 * load_spi_default_config - Load default configuration for SSP
 * @lpc31xx: SSP driver private data structure
 */
static void load_spi_default_config(struct lpc31xx *lpc31xx)
{
#if JDS
	if (lpc31xx->vendor->pl023) {
		writel(DEFAULT_SSP_REG_CR0_ST_PL023, SSP_CR0(lpc31xx->virtbase));
		writew(DEFAULT_SSP_REG_CR1_ST_PL023, SSP_CR1(lpc31xx->virtbase));
	} else if (lpc31xx->vendor->extended_cr) {
		writel(DEFAULT_SSP_REG_CR0_ST, SSP_CR0(lpc31xx->virtbase));
		writew(DEFAULT_SSP_REG_CR1_ST, SSP_CR1(lpc31xx->virtbase));
	} else {
		writew(DEFAULT_SSP_REG_CR0, SSP_CR0(lpc31xx->virtbase));
		writew(DEFAULT_SSP_REG_CR1, SSP_CR1(lpc31xx->virtbase));
	}
	writew(DEFAULT_SSP_REG_DMACR, SSP_DMACR(lpc31xx->virtbase));
	writew(DEFAULT_SSP_REG_CPSR, SSP_CPSR(lpc31xx->virtbase));
	writew(DISABLE_ALL_INTERRUPTS, SSP_IMSC(lpc31xx->virtbase));
	writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(lpc31xx->virtbase));
#endif
}

/**
 * This will write to TX and read from RX according to the parameters
 * set in lpc31xx.
 */
static void readwriter(struct lpc31xx *lpc31xx)
{

	/*
	 * The FIFO depth is different between primecell variants.
	 * I believe filling in too much in the FIFO might cause
	 * errons in 8bit wide transfers on ARM variants (just 8 words
	 * FIFO, means only 8x8 = 64 bits in FIFO) at least.
	 *
	 * To prevent this issue, the TX FIFO is only filled to the
	 * unused RX FIFO fill length, regardless of what the TX
	 * FIFO status flag indicates.
	 */
	dev_dbg(&lpc31xx->pdev->dev,
		"%s, rx: %p, rxend: %p, tx: %p, txend: %p\n",
		__func__, lpc31xx->rx, lpc31xx->rx_end, lpc31xx->tx, lpc31xx->tx_end);

	/* Read as much as you can */
	while ((readw(SSP_SR(lpc31xx->virtbase)) & SSP_SR_MASK_RNE)
	       && (lpc31xx->rx < lpc31xx->rx_end)) {
		switch (lpc31xx->read) {
		case READING_NULL:
			readw(SSP_DR(lpc31xx->virtbase));
			break;
		case READING_U8:
			*(u8 *) (lpc31xx->rx) =
				readw(SSP_DR(lpc31xx->virtbase)) & 0xFFU;
			break;
		case READING_U16:
			*(u16 *) (lpc31xx->rx) =
				(u16) readw(SSP_DR(lpc31xx->virtbase));
			break;
		case READING_U32:
			*(u32 *) (lpc31xx->rx) =
				readl(SSP_DR(lpc31xx->virtbase));
			break;
		}
		lpc31xx->rx += (lpc31xx->cur_chip->n_bytes);
		lpc31xx->exp_fifo_level--;
	}
	/*
	 * Write as much as possible up to the RX FIFO size
	 */
	while ((lpc31xx->exp_fifo_level < lpc31xx->vendor->fifodepth)
	       && (lpc31xx->tx < lpc31xx->tx_end)) {
		switch (lpc31xx->write) {
		case WRITING_NULL:
			writew(0x0, SSP_DR(lpc31xx->virtbase));
			break;
		case WRITING_U8:
			writew(*(u8 *) (lpc31xx->tx), SSP_DR(lpc31xx->virtbase));
			break;
		case WRITING_U16:
			writew((*(u16 *) (lpc31xx->tx)), SSP_DR(lpc31xx->virtbase));
			break;
		case WRITING_U32:
			writel(*(u32 *) (lpc31xx->tx), SSP_DR(lpc31xx->virtbase));
			break;
		}
		lpc31xx->tx += (lpc31xx->cur_chip->n_bytes);
		lpc31xx->exp_fifo_level++;
		/*
		 * This inner reader takes care of things appearing in the RX
		 * FIFO as we're transmitting. This will happen a lot since the
		 * clock starts running when you put things into the TX FIFO,
		 * and then things are continuously clocked into the RX FIFO.
		 */
		while ((readw(SSP_SR(lpc31xx->virtbase)) & SSP_SR_MASK_RNE)
		       && (lpc31xx->rx < lpc31xx->rx_end)) {
			switch (lpc31xx->read) {
			case READING_NULL:
				readw(SSP_DR(lpc31xx->virtbase));
				break;
			case READING_U8:
				*(u8 *) (lpc31xx->rx) =
					readw(SSP_DR(lpc31xx->virtbase)) & 0xFFU;
				break;
			case READING_U16:
				*(u16 *) (lpc31xx->rx) =
					(u16) readw(SSP_DR(lpc31xx->virtbase));
				break;
			case READING_U32:
				*(u32 *) (lpc31xx->rx) =
					readl(SSP_DR(lpc31xx->virtbase));
				break;
			}
			lpc31xx->rx += (lpc31xx->cur_chip->n_bytes);
			lpc31xx->exp_fifo_level--;
		}
	}
	/*
	 * When we exit here the TX FIFO should be full and the RX FIFO
	 * should be empty
	 */
}

/**
 * next_transfer - Move to the Next transfer in the current spi message
 * @lpc31xx: SSP driver private data structure
 *
 * This function moves though the linked list of spi transfers in the
 * current spi message and returns with the state of current spi
 * message i.e whether its last transfer is done(STATE_DONE) or
 * Next transfer is ready(STATE_RUNNING)
 */
static void *next_transfer(struct lpc31xx *lpc31xx)
{
	struct spi_message *msg = lpc31xx->cur_msg;
	struct spi_transfer *trans = lpc31xx->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		lpc31xx->cur_transfer =
		    list_entry(trans->transfer_list.next,
			       struct spi_transfer, transfer_list);
		return STATE_RUNNING;
	}
	return STATE_DONE;
}

/*
 * This DMA functionality is only compiled in if we have
 * access to the generic DMA devices/DMA engine.
 */
#ifdef CONFIG_DMA_ENGINE_X
static void unmap_free_dma_scatter(struct lpc31xx *lpc31xx)
{
	/* Unmap and free the SG tables */
	dma_unmap_sg(lpc31xx->dma_tx_channel->device->dev, lpc31xx->sgt_tx.sgl,
		     lpc31xx->sgt_tx.nents, DMA_TO_DEVICE);
	dma_unmap_sg(lpc31xx->dma_rx_channel->device->dev, lpc31xx->sgt_rx.sgl,
		     lpc31xx->sgt_rx.nents, DMA_FROM_DEVICE);
	sg_free_table(&lpc31xx->sgt_rx);
	sg_free_table(&lpc31xx->sgt_tx);
}

static void dma_callback(void *data)
{
	struct lpc31xx *lpc31xx = data;
	struct spi_message *msg = lpc31xx->cur_msg;

	BUG_ON(!lpc31xx->sgt_rx.sgl);

#ifdef VERBOSE_DEBUG
	/*
	 * Optionally dump out buffers to inspect contents, this is
	 * good if you want to convince yourself that the loopback
	 * read/write contents are the same, when adopting to a new
	 * DMA engine.
	 */
	{
		struct scatterlist *sg;
		unsigned int i;

		dma_sync_sg_for_cpu(&lpc31xx->pdev->dev,
				    lpc31xx->sgt_rx.sgl,
				    lpc31xx->sgt_rx.nents,
				    DMA_FROM_DEVICE);

		for_each_sg(lpc31xx->sgt_rx.sgl, sg, lpc31xx->sgt_rx.nents, i) {
			dev_dbg(&lpc31xx->pdev->dev, "SPI RX SG ENTRY: %d", i);
			print_hex_dump(KERN_ERR, "SPI RX: ",
				       DUMP_PREFIX_OFFSET,
				       16,
				       1,
				       sg_virt(sg),
				       sg_dma_len(sg),
				       1);
		}
		for_each_sg(lpc31xx->sgt_tx.sgl, sg, lpc31xx->sgt_tx.nents, i) {
			dev_dbg(&lpc31xx->pdev->dev, "SPI TX SG ENTRY: %d", i);
			print_hex_dump(KERN_ERR, "SPI TX: ",
				       DUMP_PREFIX_OFFSET,
				       16,
				       1,
				       sg_virt(sg),
				       sg_dma_len(sg),
				       1);
		}
	}
#endif

	unmap_free_dma_scatter(lpc31xx);

	/* Update total bytes transferred */
	msg->actual_length += lpc31xx->cur_transfer->len;
	if (lpc31xx->cur_transfer->cs_change)
		lpc31xx->cur_chip->
			cs_control(SSP_CHIP_DESELECT);

	/* Move to next transfer */
	msg->state = next_transfer(lpc31xx);
	tasklet_schedule(&lpc31xx->pump_transfers);
}

static void setup_dma_scatter(struct lpc31xx *lpc31xx,
			      void *buffer,
			      unsigned int length,
			      struct sg_table *sgtab)
{
	struct scatterlist *sg;
	int bytesleft = length;
	void *bufp = buffer;
	int mapbytes;
	int i;

	if (buffer) {
		for_each_sg(sgtab->sgl, sg, sgtab->nents, i) {
			/*
			 * If there are less bytes left than what fits
			 * in the current page (plus page alignment offset)
			 * we just feed in this, else we stuff in as much
			 * as we can.
			 */
			if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE - offset_in_page(bufp);
			sg_set_page(sg, virt_to_page(bufp),
				    mapbytes, offset_in_page(bufp));
			bufp += mapbytes;
			bytesleft -= mapbytes;
			dev_dbg(&lpc31xx->pdev->dev,
				"set RX/TX target page @ %p, %d bytes, %d left\n",
				bufp, mapbytes, bytesleft);
		}
	} else {
		/* Map the dummy buffer on every page */
		for_each_sg(sgtab->sgl, sg, sgtab->nents, i) {
			if (bytesleft < PAGE_SIZE)
				mapbytes = bytesleft;
			else
				mapbytes = PAGE_SIZE;
			sg_set_page(sg, virt_to_page(lpc31xx->dummypage),
				    mapbytes, 0);
			bytesleft -= mapbytes;
			dev_dbg(&lpc31xx->pdev->dev,
				"set RX/TX to dummy page %d bytes, %d left\n",
				mapbytes, bytesleft);

		}
	}
	BUG_ON(bytesleft);
}

/**
 * configure_dma - configures the channels for the next transfer
 * @lpc31xx: SSP driver's private data structure
 */
static int configure_dma(struct lpc31xx *lpc31xx)
{
	struct dma_slave_config rx_conf = {
		.src_addr = SSP_DR(lpc31xx->phybase),
		.direction = DMA_DEV_TO_MEM,
		.device_fc = false,
	};
	struct dma_slave_config tx_conf = {
		.dst_addr = SSP_DR(lpc31xx->phybase),
		.direction = DMA_MEM_TO_DEV,
		.device_fc = false,
	};
	unsigned int pages;
	int ret;
	int rx_sglen, tx_sglen;
	struct dma_chan *rxchan = lpc31xx->dma_rx_channel;
	struct dma_chan *txchan = lpc31xx->dma_tx_channel;
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;

	/* Check that the channels are available */
	if (!rxchan || !txchan)
		return -ENODEV;

	/*
	 * If supplied, the DMA burstsize should equal the FIFO trigger level.
	 * Notice that the DMA engine uses one-to-one mapping. Since we can
	 * not trigger on 2 elements this needs explicit mapping rather than
	 * calculation.
	 */
	switch (lpc31xx->rx_lev_trig) {
	case SSP_RX_1_OR_MORE_ELEM:
		rx_conf.src_maxburst = 1;
		break;
	case SSP_RX_4_OR_MORE_ELEM:
		rx_conf.src_maxburst = 4;
		break;
	case SSP_RX_8_OR_MORE_ELEM:
		rx_conf.src_maxburst = 8;
		break;
	case SSP_RX_16_OR_MORE_ELEM:
		rx_conf.src_maxburst = 16;
		break;
	case SSP_RX_32_OR_MORE_ELEM:
		rx_conf.src_maxburst = 32;
		break;
	default:
		rx_conf.src_maxburst = lpc31xx->vendor->fifodepth >> 1;
		break;
	}

	switch (lpc31xx->tx_lev_trig) {
	case SSP_TX_1_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 1;
		break;
	case SSP_TX_4_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 4;
		break;
	case SSP_TX_8_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 8;
		break;
	case SSP_TX_16_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 16;
		break;
	case SSP_TX_32_OR_MORE_EMPTY_LOC:
		tx_conf.dst_maxburst = 32;
		break;
	default:
		tx_conf.dst_maxburst = lpc31xx->vendor->fifodepth >> 1;
		break;
	}

	switch (lpc31xx->read) {
	case READING_NULL:
		/* Use the same as for writing */
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_UNDEFINED;
		break;
	case READING_U8:
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case READING_U16:
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case READING_U32:
		rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	}

	switch (lpc31xx->write) {
	case WRITING_NULL:
		/* Use the same as for reading */
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_UNDEFINED;
		break;
	case WRITING_U8:
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case WRITING_U16:
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case WRITING_U32:
		tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	}

	/* SPI pecularity: we need to read and write the same width */
	if (rx_conf.src_addr_width == DMA_SLAVE_BUSWIDTH_UNDEFINED)
		rx_conf.src_addr_width = tx_conf.dst_addr_width;
	if (tx_conf.dst_addr_width == DMA_SLAVE_BUSWIDTH_UNDEFINED)
		tx_conf.dst_addr_width = rx_conf.src_addr_width;
	BUG_ON(rx_conf.src_addr_width != tx_conf.dst_addr_width);

	dmaengine_slave_config(rxchan, &rx_conf);
	dmaengine_slave_config(txchan, &tx_conf);

	/* Create sglists for the transfers */
	pages = DIV_ROUND_UP(lpc31xx->cur_transfer->len, PAGE_SIZE);
	dev_dbg(&lpc31xx->pdev->dev, "using %d pages for transfer\n", pages);

	ret = sg_alloc_table(&lpc31xx->sgt_rx, pages, GFP_ATOMIC);
	if (ret)
		goto err_alloc_rx_sg;

	ret = sg_alloc_table(&lpc31xx->sgt_tx, pages, GFP_ATOMIC);
	if (ret)
		goto err_alloc_tx_sg;

	/* Fill in the scatterlists for the RX+TX buffers */
	setup_dma_scatter(lpc31xx, lpc31xx->rx,
			  lpc31xx->cur_transfer->len, &lpc31xx->sgt_rx);
	setup_dma_scatter(lpc31xx, lpc31xx->tx,
			  lpc31xx->cur_transfer->len, &lpc31xx->sgt_tx);

	/* Map DMA buffers */
	rx_sglen = dma_map_sg(rxchan->device->dev, lpc31xx->sgt_rx.sgl,
			   lpc31xx->sgt_rx.nents, DMA_FROM_DEVICE);
	if (!rx_sglen)
		goto err_rx_sgmap;

	tx_sglen = dma_map_sg(txchan->device->dev, lpc31xx->sgt_tx.sgl,
			   lpc31xx->sgt_tx.nents, DMA_TO_DEVICE);
	if (!tx_sglen)
		goto err_tx_sgmap;

	/* Send both scatter lists */
	rxdesc = dmaengine_prep_slave_sg(rxchan,
				      lpc31xx->sgt_rx.sgl,
				      rx_sglen,
				      DMA_DEV_TO_MEM,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc)
		goto err_rxdesc;

	txdesc = dmaengine_prep_slave_sg(txchan,
				      lpc31xx->sgt_tx.sgl,
				      tx_sglen,
				      DMA_MEM_TO_DEV,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc)
		goto err_txdesc;

	/* Put the callback on the RX transfer only, that should finish last */
	rxdesc->callback = dma_callback;
	rxdesc->callback_param = lpc31xx;

	/* Submit and fire RX and TX with TX last so we're ready to read! */
	dmaengine_submit(rxdesc);
	dmaengine_submit(txdesc);
	dma_async_issue_pending(rxchan);
	dma_async_issue_pending(txchan);
	lpc31xx->dma_running = true;

	return 0;

err_txdesc:
	dmaengine_terminate_all(txchan);
err_rxdesc:
	dmaengine_terminate_all(rxchan);
	dma_unmap_sg(txchan->device->dev, lpc31xx->sgt_tx.sgl,
		     lpc31xx->sgt_tx.nents, DMA_TO_DEVICE);
err_tx_sgmap:
	dma_unmap_sg(rxchan->device->dev, lpc31xx->sgt_rx.sgl,
		     lpc31xx->sgt_tx.nents, DMA_FROM_DEVICE);
err_rx_sgmap:
	sg_free_table(&lpc31xx->sgt_tx);
err_alloc_tx_sg:
	sg_free_table(&lpc31xx->sgt_rx);
err_alloc_rx_sg:
	return -ENOMEM;
}

static int __devinit lpc31xx_dma_probe(struct lpc31xx *lpc31xx)
{
	dma_cap_mask_t mask;

	/* Try to acquire a generic DMA engine slave channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	/*
	 * We need both RX and TX channels to do DMA, else do none
	 * of them.
	 */
	lpc31xx->dma_rx_channel = dma_request_channel(mask,
					    lpc31xx->master_info->dma_filter,
					    lpc31xx->master_info->dma_rx_param);
	if (!lpc31xx->dma_rx_channel) {
		dev_dbg(&lpc31xx->pdev->dev, "no RX DMA channel!\n");
		goto err_no_rxchan;
	}

	lpc31xx->dma_tx_channel = dma_request_channel(mask,
					    lpc31xx->master_info->dma_filter,
					    lpc31xx->master_info->dma_tx_param);
	if (!lpc31xx->dma_tx_channel) {
		dev_dbg(&lpc31xx->pdev->dev, "no TX DMA channel!\n");
		goto err_no_txchan;
	}

	lpc31xx->dummypage = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!lpc31xx->dummypage) {
		dev_dbg(&lpc31xx->pdev->dev, "no DMA dummypage!\n");
		goto err_no_dummypage;
	}

	dev_info(&lpc31xx->pdev->dev, "setup for DMA on RX %s, TX %s\n",
		 dma_chan_name(lpc31xx->dma_rx_channel),
		 dma_chan_name(lpc31xx->dma_tx_channel));

	return 0;

err_no_dummypage:
	dma_release_channel(lpc31xx->dma_tx_channel);
err_no_txchan:
	dma_release_channel(lpc31xx->dma_rx_channel);
	lpc31xx->dma_rx_channel = NULL;
err_no_rxchan:
	dev_err(&lpc31xx->pdev->dev,
			"Failed to work in dma mode, work without dma!\n");
	return -ENODEV;
}

static void terminate_dma(struct lpc31xx *lpc31xx)
{
	struct dma_chan *rxchan = lpc31xx->dma_rx_channel;
	struct dma_chan *txchan = lpc31xx->dma_tx_channel;

	dmaengine_terminate_all(rxchan);
	dmaengine_terminate_all(txchan);
	unmap_free_dma_scatter(lpc31xx);
	lpc31xx->dma_running = false;
}

static void lpc31xx_dma_remove(struct lpc31xx *lpc31xx)
{
	if (lpc31xx->dma_running)
		terminate_dma(lpc31xx);
	if (lpc31xx->dma_tx_channel)
		dma_release_channel(lpc31xx->dma_tx_channel);
	if (lpc31xx->dma_rx_channel)
		dma_release_channel(lpc31xx->dma_rx_channel);
	kfree(lpc31xx->dummypage);
}

#else
static inline int configure_dma(struct lpc31xx *lpc31xx)
{
	return -ENODEV;
}

static inline int lpc31xx_dma_probe(struct lpc31xx *lpc31xx)
{
	return 0;
}

static inline void lpc31xx_dma_remove(struct lpc31xx *lpc31xx)
{
}
#endif

/**
 * lpc31xx_interrupt_handler - Interrupt handler for SSP controller
 *
 * This function handles interrupts generated for an interrupt based transfer.
 * If a receive overrun (ROR) interrupt is there then we disable SSP, flag the
 * current message's state as STATE_ERROR and schedule the tasklet
 * pump_transfers which will do the post processing of the current message by
 * calling giveback(). Otherwise it reads data from RX FIFO till there is no
 * more data, and writes data in TX FIFO till it is not full. If we complete
 * the transfer we move to the next transfer and schedule the tasklet.
 */
static irqreturn_t lpc31xx_interrupt_handler(int irq, void *dev_id)
{
	struct lpc31xx *lpc31xx = dev_id;
	struct spi_message *msg = lpc31xx->cur_msg;
	u16 irq_status = 0;
	u16 flag = 0;

	if (unlikely(!msg)) {
		dev_err(&lpc31xx->pdev->dev,
			"bad message state in interrupt handler");
		/* Never fail */
		return IRQ_HANDLED;
	}
#if JDS
	/* Read the Interrupt Status Register */
	irq_status = readw(SSP_MIS(lpc31xx->virtbase));

	if (unlikely(!irq_status))
		return IRQ_NONE;

	/*
	 * This handles the FIFO interrupts, the timeout
	 * interrupts are flatly ignored, they cannot be
	 * trusted.
	 */
	if (unlikely(irq_status & SSP_MIS_MASK_RORMIS)) {
		/*
		 * Overrun interrupt - bail out since our Data has been
		 * corrupted
		 */
		dev_err(&lpc31xx->pdev->dev, "FIFO overrun\n");
		if (readw(SSP_SR(lpc31xx->virtbase)) & SSP_SR_MASK_RFF)
			dev_err(&lpc31xx->pdev->dev,
				"RXFIFO is full\n");
		if (readw(SSP_SR(lpc31xx->virtbase)) & SSP_SR_MASK_TNF)
			dev_err(&lpc31xx->pdev->dev,
				"TXFIFO is full\n");

		/*
		 * Disable and clear interrupts, disable SSP,
		 * mark message with bad status so it can be
		 * retried.
		 */
		writew(DISABLE_ALL_INTERRUPTS,
		       SSP_IMSC(lpc31xx->virtbase));
		writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(lpc31xx->virtbase));
		writew((readw(SSP_CR1(lpc31xx->virtbase)) &
			(~SSP_CR1_MASK_SSE)), SSP_CR1(lpc31xx->virtbase));
		msg->state = STATE_ERROR;

		/* Schedule message queue handler */
		tasklet_schedule(&lpc31xx->pump_transfers);
		return IRQ_HANDLED;
	}

	readwriter(lpc31xx);

	if ((lpc31xx->tx == lpc31xx->tx_end) && (flag == 0)) {
		flag = 1;
		/* Disable Transmit interrupt, enable receive interrupt */
		writew((readw(SSP_IMSC(lpc31xx->virtbase)) &
		       ~SSP_IMSC_MASK_TXIM) | SSP_IMSC_MASK_RXIM,
		       SSP_IMSC(lpc31xx->virtbase));
	}

	/*
	 * Since all transactions must write as much as shall be read,
	 * we can conclude the entire transaction once RX is complete.
	 * At this point, all TX will always be finished.
	 */
	if (lpc31xx->rx >= lpc31xx->rx_end) {
		writew(DISABLE_ALL_INTERRUPTS,
		       SSP_IMSC(lpc31xx->virtbase));
		writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(lpc31xx->virtbase));
		if (unlikely(lpc31xx->rx > lpc31xx->rx_end)) {
			dev_warn(&lpc31xx->pdev->dev, "read %u surplus "
				 "bytes (did you request an odd "
				 "number of bytes on a 16bit bus?)\n",
				 (u32) (lpc31xx->rx - lpc31xx->rx_end));
		}
		/* Update total bytes transferred */
		msg->actual_length += lpc31xx->cur_transfer->len;
		if (lpc31xx->cur_transfer->cs_change)
			lpc31xx->cur_chip->
				cs_control(SSP_CHIP_DESELECT);
		/* Move to next transfer */
		msg->state = next_transfer(lpc31xx);
		tasklet_schedule(&lpc31xx->pump_transfers);
		return IRQ_HANDLED;
	}
#endif
	return IRQ_HANDLED;
}

/**
 * This sets up the pointers to memory for the next message to
 * send out on the SPI bus.
 */
static int set_up_next_transfer(struct lpc31xx *lpc31xx,
				struct spi_transfer *transfer)
{
	int residue;

	/* Sanity check the message for this bus width */
	residue = lpc31xx->cur_transfer->len % lpc31xx->cur_chip->n_bytes;
	if (unlikely(residue != 0)) {
		dev_err(&lpc31xx->pdev->dev,
			"message of %u bytes to transmit but the current "
			"chip bus has a data width of %u bytes!\n",
			lpc31xx->cur_transfer->len,
			lpc31xx->cur_chip->n_bytes);
		dev_err(&lpc31xx->pdev->dev, "skipping this message\n");
		return -EIO;
	}
	lpc31xx->tx = (void *)transfer->tx_buf;
	lpc31xx->tx_end = lpc31xx->tx + lpc31xx->cur_transfer->len;
	lpc31xx->rx = (void *)transfer->rx_buf;
	lpc31xx->rx_end = lpc31xx->rx + lpc31xx->cur_transfer->len;
	lpc31xx->write =
	    lpc31xx->tx ? lpc31xx->cur_chip->write : WRITING_NULL;
	lpc31xx->read = lpc31xx->rx ? lpc31xx->cur_chip->read : READING_NULL;
	return 0;
}

/**
 * pump_transfers - Tasklet function which schedules next transfer
 * when running in interrupt or DMA transfer mode.
 * @data: SSP driver private data structure
 *
 */
static void pump_transfers(unsigned long data)
{
	struct lpc31xx *lpc31xx = (struct lpc31xx *) data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;

	/* Get current state information */
	message = lpc31xx->cur_msg;
	transfer = lpc31xx->cur_transfer;

	/* Handle for abort */
	if (message->state == STATE_ERROR) {
		message->status = -EIO;
		giveback(lpc31xx);
		return;
	}

	/* Handle end of message */
	if (message->state == STATE_DONE) {
		message->status = 0;
		giveback(lpc31xx);
		return;
	}

	/* Delay if requested at end of transfer before CS change */
	if (message->state == STATE_RUNNING) {
		previous = list_entry(transfer->transfer_list.prev,
					struct spi_transfer,
					transfer_list);
		if (previous->delay_usecs)
			/*
			 * FIXME: This runs in interrupt context.
			 * Is this really smart?
			 */
			udelay(previous->delay_usecs);

		/* Reselect chip select only if cs_change was requested */
		if (previous->cs_change)
			lpc31xx->cur_chip->cs_control(SSP_CHIP_SELECT);
	} else {
		/* STATE_START */
		message->state = STATE_RUNNING;
	}

	if (set_up_next_transfer(lpc31xx, transfer)) {
		message->state = STATE_ERROR;
		message->status = -EIO;
		giveback(lpc31xx);
		return;
	}
	/* Flush the FIFOs and let's go! */
	flush(lpc31xx);

	if (lpc31xx->cur_chip->enable_dma) {
		if (configure_dma(lpc31xx)) {
			dev_dbg(&lpc31xx->pdev->dev,
				"configuration of DMA failed, fall back to interrupt mode\n");
			goto err_config_dma;
		}
		return;
	}

err_config_dma:
	/* enable all interrupts except RX */
	writew(ENABLE_ALL_INTERRUPTS & ~SSP_IMSC_MASK_RXIM, SSP_IMSC(lpc31xx->virtbase));
}

static void do_interrupt_dma_transfer(struct lpc31xx *lpc31xx)
{
#if JDS
	/*
	 * Default is to enable all interrupts except RX -
	 * this will be enabled once TX is complete
	 */
	u32 irqflags = ENABLE_ALL_INTERRUPTS & ~SSP_IMSC_MASK_RXIM;

	/* Enable target chip, if not already active */
	if (!lpc31xx->next_msg_cs_active)
		lpc31xx->cur_chip->cs_control(SSP_CHIP_SELECT);

	if (set_up_next_transfer(lpc31xx, lpc31xx->cur_transfer)) {
		/* Error path */
		lpc31xx->cur_msg->state = STATE_ERROR;
		lpc31xx->cur_msg->status = -EIO;
		giveback(lpc31xx);
		return;
	}
	/* If we're using DMA, set up DMA here */
	if (lpc31xx->cur_chip->enable_dma) {
		/* Configure DMA transfer */
		if (configure_dma(lpc31xx)) {
			dev_dbg(&lpc31xx->pdev->dev,
				"configuration of DMA failed, fall back to interrupt mode\n");
			goto err_config_dma;
		}
		/* Disable interrupts in DMA mode, IRQ from DMA controller */
		irqflags = DISABLE_ALL_INTERRUPTS;
	}
err_config_dma:
	/* Enable SSP, turn on interrupts */
	writew((readw(SSP_CR1(lpc31xx->virtbase)) | SSP_CR1_MASK_SSE),
	       SSP_CR1(lpc31xx->virtbase));
	writew(irqflags, SSP_IMSC(lpc31xx->virtbase));
#endif
}

static void do_polling_transfer(struct lpc31xx *lpc31xx)
{
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct chip_data *chip;
	unsigned long time, timeout;

	chip = lpc31xx->cur_chip;
	message = lpc31xx->cur_msg;

	while (message->state != STATE_DONE) {
		/* Handle for abort */
		if (message->state == STATE_ERROR)
			break;
		transfer = lpc31xx->cur_transfer;

		/* Delay if requested at end of transfer */
		if (message->state == STATE_RUNNING) {
			previous =
			    list_entry(transfer->transfer_list.prev,
				       struct spi_transfer, transfer_list);
			if (previous->delay_usecs)
				udelay(previous->delay_usecs);
			if (previous->cs_change)
				lpc31xx->cur_chip->cs_control(SSP_CHIP_SELECT);
		} else {
			/* STATE_START */
			message->state = STATE_RUNNING;
			if (!lpc31xx->next_msg_cs_active)
				lpc31xx->cur_chip->cs_control(SSP_CHIP_SELECT);
		}

		/* Configuration Changing Per Transfer */
		if (set_up_next_transfer(lpc31xx, transfer)) {
			/* Error path */
			message->state = STATE_ERROR;
			break;
		}
		/* Flush FIFOs and enable SSP */
		flush(lpc31xx);
		writew((readw(SSP_CR1(lpc31xx->virtbase)) | SSP_CR1_MASK_SSE),
		       SSP_CR1(lpc31xx->virtbase));

		dev_dbg(&lpc31xx->pdev->dev, "polling transfer ongoing ...\n");

		timeout = jiffies + msecs_to_jiffies(SPI_POLLING_TIMEOUT);
		while (lpc31xx->tx < lpc31xx->tx_end || lpc31xx->rx < lpc31xx->rx_end) {
			time = jiffies;
			readwriter(lpc31xx);
			if (time_after(time, timeout)) {
				dev_warn(&lpc31xx->pdev->dev,
				"%s: timeout!\n", __func__);
				message->state = STATE_ERROR;
				goto out;
			}
			cpu_relax();
		}

		/* Update total byte transferred */
		message->actual_length += lpc31xx->cur_transfer->len;
		if (lpc31xx->cur_transfer->cs_change)
			lpc31xx->cur_chip->cs_control(SSP_CHIP_DESELECT);
		/* Move to next transfer */
		message->state = next_transfer(lpc31xx);
	}
out:
	/* Handle end of message */
	if (message->state == STATE_DONE)
		message->status = 0;
	else
		message->status = -EIO;

	giveback(lpc31xx);
	return;
}

static int lpc31xx_transfer_one_message(struct spi_master *master,
				      struct spi_message *msg)
{
	struct lpc31xx *lpc31xx = spi_master_get_devdata(master);

	/* Initial message state */
	lpc31xx->cur_msg = msg;
	msg->state = STATE_START;

	lpc31xx->cur_transfer = list_entry(msg->transfers.next,
					 struct spi_transfer, transfer_list);

	/* Setup the SPI using the per chip configuration */
	lpc31xx->cur_chip = spi_get_ctldata(msg->spi);

	restore_state(lpc31xx);
	flush(lpc31xx);

	if (lpc31xx->cur_chip->xfer_type == POLLING_TRANSFER)
		do_polling_transfer(lpc31xx);
	else
		do_interrupt_dma_transfer(lpc31xx);

	return 0;
}

static int lpc31xx_prepare_transfer_hardware(struct spi_master *master)
{
	struct lpc31xx *lpc31xx = spi_master_get_devdata(master);

	/*
	 * Just make sure we have all we need to run the transfer by syncing
	 * with the runtime PM framework.
	 */
	pm_runtime_get_sync(&lpc31xx->pdev->dev);
	return 0;
}

static int lpc31xx_unprepare_transfer_hardware(struct spi_master *master)
{
	struct lpc31xx *lpc31xx = spi_master_get_devdata(master);

	/* nothing more to do - disable spi/spi and power off */
	writew((readw(SSP_CR1(lpc31xx->virtbase)) &
		(~SSP_CR1_MASK_SSE)), SSP_CR1(lpc31xx->virtbase));

	if (lpc31xx->master_info->autosuspend_delay > 0) {
		pm_runtime_mark_last_busy(&lpc31xx->pdev->dev);
		pm_runtime_put_autosuspend(&lpc31xx->pdev->dev);
	} else {
		pm_runtime_put(&lpc31xx->pdev->dev);
	}

	return 0;
}

static int verify_controller_parameters(struct lpc31xx *lpc31xx,
				struct lpc31xx_config_chip const *chip_info)
{
#if JDS
	if ((chip_info->iface < SSP_INTERFACE_MOTOROLA_SPI)
	    || (chip_info->iface > SSP_INTERFACE_UNIDIRECTIONAL)) {
		dev_err(&lpc31xx->pdev->dev,
			"interface is configured incorrectly\n");
		return -EINVAL;
	}
	if ((chip_info->iface == SSP_INTERFACE_UNIDIRECTIONAL) &&
	    (!lpc31xx->vendor->unidir)) {
		dev_err(&lpc31xx->pdev->dev,
			"unidirectional mode not supported in this "
			"hardware version\n");
		return -EINVAL;
	}
	if ((chip_info->hierarchy != SSP_MASTER)
	    && (chip_info->hierarchy != SSP_SLAVE)) {
		dev_err(&lpc31xx->pdev->dev,
			"hierarchy is configured incorrectly\n");
		return -EINVAL;
	}
	if ((chip_info->com_mode != INTERRUPT_TRANSFER)
	    && (chip_info->com_mode != DMA_TRANSFER)
	    && (chip_info->com_mode != POLLING_TRANSFER)) {
		dev_err(&lpc31xx->pdev->dev,
			"Communication mode is configured incorrectly\n");
		return -EINVAL;
	}
	switch (chip_info->rx_lev_trig) {
	case SSP_RX_1_OR_MORE_ELEM:
	case SSP_RX_4_OR_MORE_ELEM:
	case SSP_RX_8_OR_MORE_ELEM:
		/* These are always OK, all variants can handle this */
		break;
	case SSP_RX_16_OR_MORE_ELEM:
		if (lpc31xx->vendor->fifodepth < 16) {
			dev_err(&lpc31xx->pdev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	case SSP_RX_32_OR_MORE_ELEM:
		if (lpc31xx->vendor->fifodepth < 32) {
			dev_err(&lpc31xx->pdev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(&lpc31xx->pdev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
		return -EINVAL;
		break;
	}
	switch (chip_info->tx_lev_trig) {
	case SSP_TX_1_OR_MORE_EMPTY_LOC:
	case SSP_TX_4_OR_MORE_EMPTY_LOC:
	case SSP_TX_8_OR_MORE_EMPTY_LOC:
		/* These are always OK, all variants can handle this */
		break;
	case SSP_TX_16_OR_MORE_EMPTY_LOC:
		if (lpc31xx->vendor->fifodepth < 16) {
			dev_err(&lpc31xx->pdev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	case SSP_TX_32_OR_MORE_EMPTY_LOC:
		if (lpc31xx->vendor->fifodepth < 32) {
			dev_err(&lpc31xx->pdev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(&lpc31xx->pdev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
		return -EINVAL;
		break;
	}
	if (chip_info->iface == SSP_INTERFACE_NATIONAL_MICROWIRE) {
		if ((chip_info->ctrl_len < SSP_BITS_4)
		    || (chip_info->ctrl_len > SSP_BITS_32)) {
			dev_err(&lpc31xx->pdev->dev,
				"CTRL LEN is configured incorrectly\n");
			return -EINVAL;
		}
		if ((chip_info->wait_state != SSP_MWIRE_WAIT_ZERO)
		    && (chip_info->wait_state != SSP_MWIRE_WAIT_ONE)) {
			dev_err(&lpc31xx->pdev->dev,
				"Wait State is configured incorrectly\n");
			return -EINVAL;
		}
		/* Half duplex is only available in the ST Micro version */
		if (lpc31xx->vendor->extended_cr) {
			if ((chip_info->duplex !=
			     SSP_MICROWIRE_CHANNEL_FULL_DUPLEX)
			    && (chip_info->duplex !=
				SSP_MICROWIRE_CHANNEL_HALF_DUPLEX)) {
				dev_err(&lpc31xx->pdev->dev,
					"Microwire duplex mode is configured incorrectly\n");
				return -EINVAL;
			}
		} else {
			if (chip_info->duplex != SSP_MICROWIRE_CHANNEL_FULL_DUPLEX)
				dev_err(&lpc31xx->pdev->dev,
					"Microwire half duplex mode requested,"
					" but this is only available in the"
					" ST version of LPC31xx\n");
			return -EINVAL;
		}
	}
#endif
	return 0;
}

static inline u32 spi_rate(u32 rate, u16 cpsdvsr, u16 scr)
{
	return rate / (cpsdvsr * (1 + scr));
}

static int calculate_effective_freq(struct lpc31xx *lpc31xx, int freq, struct
				    spi_clock_params * clk_freq)
{
#if JDS
	/* Lets calculate the frequency parameters */
	u16 cpsdvsr = CPSDVR_MIN, scr = SCR_MIN;
	u32 rate, max_tclk, min_tclk, best_freq = 0, best_cpsdvsr = 0,
		best_scr = 0, tmp, found = 0;

	rate = clk_get_rate(lpc31xx->clk);
	/* cpsdvscr = 2 & scr 0 */
	max_tclk = spi_rate(rate, CPSDVR_MIN, SCR_MIN);
	/* cpsdvsr = 254 & scr = 255 */
	min_tclk = spi_rate(rate, CPSDVR_MAX, SCR_MAX);

	if (!((freq <= max_tclk) && (freq >= min_tclk))) {
		dev_err(&lpc31xx->pdev->dev,
			"controller data is incorrect: out of range frequency");
		return -EINVAL;
	}

	/*
	 * best_freq will give closest possible available rate (<= requested
	 * freq) for all values of scr & cpsdvsr.
	 */
	while ((cpsdvsr <= CPSDVR_MAX) && !found) {
		while (scr <= SCR_MAX) {
			tmp = spi_rate(rate, cpsdvsr, scr);

			if (tmp > freq)
				scr++;
			/*
			 * If found exact value, update and break.
			 * If found more closer value, update and continue.
			 */
			else if ((tmp == freq) || (tmp > best_freq)) {
				best_freq = tmp;
				best_cpsdvsr = cpsdvsr;
				best_scr = scr;

				if (tmp == freq)
					break;
			}
			scr++;
		}
		cpsdvsr += 2;
		scr = SCR_MIN;
	}

	clk_freq->cpsdvsr = (u8) (best_cpsdvsr & 0xFF);
	clk_freq->scr = (u8) (best_scr & 0xFF);
	dev_dbg(&lpc31xx->pdev->dev,
		"SSP Target Frequency is: %u, Effective Frequency is %u\n",
		freq, best_freq);
	dev_dbg(&lpc31xx->pdev->dev, "SSP cpsdvsr = %d, scr = %d\n",
		clk_freq->cpsdvsr, clk_freq->scr);
#endif
	return 0;
}

/*
 * A piece of default chip info unless the platform
 * supplies it.
 */
static const struct lpc31xx_config_chip lpc31xx_default_chip_info = {
	.com_mode = POLLING_TRANSFER,
	.iface = SSP_INTERFACE_MOTOROLA_SPI,
	.hierarchy = SSP_SLAVE,
	.slave_tx_disable = DO_NOT_DRIVE_TX,
	.rx_lev_trig = SSP_RX_1_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_1_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_8,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
	.cs_control = null_cs_control,
};

/**
 * lpc31xx_setup - setup function registered to SPI master framework
 * @spi: spi device which is requesting setup
 *
 * This function is registered to the SPI framework for this SPI master
 * controller. If it is the first time when setup is called by this device,
 * this function will initialize the runtime state for this chip and save
 * the same in the device structure. Else it will update the runtime info
 * with the updated chip info. Nothing is really being written to the
 * controller hardware here, that is not done until the actual transfer
 * commence.
 */
static int lpc31xx_setup(struct spi_device *spi)
{
	struct lpc31xx_config_chip const *chip_info;
	struct chip_data *chip;
	struct spi_clock_params clk_freq = { .cpsdvsr = 0, .scr = 0};
	int status = 0;
	struct lpc31xx *lpc31xx = spi_master_get_devdata(spi->master);
	unsigned int bits = spi->bits_per_word;
	u32 tmp;

	if (!spi->max_speed_hz)
		return -EINVAL;

	/* Get controller_state if one is supplied */
	chip = spi_get_ctldata(spi);

	if (chip == NULL) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip) {
			dev_err(&spi->dev,
				"cannot allocate controller state\n");
			return -ENOMEM;
		}
		dev_dbg(&spi->dev,
			"allocated memory for controller's runtime state\n");
	}

	/* Get controller data if one is supplied */
	chip_info = spi->controller_data;

	if (chip_info == NULL) {
		chip_info = &lpc31xx_default_chip_info;
		/* spi_board_info.controller_data not is supplied */
		dev_dbg(&spi->dev,
			"using default controller_data settings\n");
	} else
		dev_dbg(&spi->dev,
			"using user supplied controller_data settings\n");

#if JDS
	/*
	 * We can override with custom divisors, else we use the board
	 * frequency setting
	 */
	if ((0 == chip_info->clk_freq.cpsdvsr)
	    && (0 == chip_info->clk_freq.scr)) {
		status = calculate_effective_freq(lpc31xx,
						  spi->max_speed_hz,
						  &clk_freq);
		if (status < 0)
			goto err_config_params;
	} else {
		memcpy(&clk_freq, &chip_info->clk_freq, sizeof(clk_freq));
		if ((clk_freq.cpsdvsr % 2) != 0)
			clk_freq.cpsdvsr =
				clk_freq.cpsdvsr - 1;
	}
	if ((clk_freq.cpsdvsr < CPSDVR_MIN)
	    || (clk_freq.cpsdvsr > CPSDVR_MAX)) {
		status = -EINVAL;
		dev_err(&spi->dev,
			"cpsdvsr is configured incorrectly\n");
		goto err_config_params;
	}

	status = verify_controller_parameters(lpc31xx, chip_info);
	if (status) {
		dev_err(&spi->dev, "controller data is incorrect");
		goto err_config_params;
	}

	lpc31xx->rx_lev_trig = chip_info->rx_lev_trig;
	lpc31xx->tx_lev_trig = chip_info->tx_lev_trig;

	/* Now set controller state based on controller data */
	chip->xfer_type = chip_info->com_mode;
	if (!chip_info->cs_control) {
		chip->cs_control = null_cs_control;
		dev_warn(&spi->dev,
			 "chip select function is NULL for this chip\n");
	} else
		chip->cs_control = chip_info->cs_control;
#endif

	if (bits <= 3) {
		/* LPC31xx doesn't support less than 4-bits */
		status = -ENOTSUPP;
		goto err_config_params;
	} else if (bits <= 8) {
		dev_dbg(&spi->dev, "4 <= n <=8 bits per word\n");
		chip->n_bytes = 1;
		chip->read = READING_U8;
		chip->write = WRITING_U8;
	} else if (bits <= 16) {
		dev_dbg(&spi->dev, "9 <= n <= 16 bits per word\n");
		chip->n_bytes = 2;
		chip->read = READING_U16;
		chip->write = WRITING_U16;
	} else {
		if (lpc31xx->vendor->max_bpw >= 32) {
			dev_dbg(&spi->dev, "17 <= n <= 32 bits per word\n");
			chip->n_bytes = 4;
			chip->read = READING_U32;
			chip->write = WRITING_U32;
		} else {
			dev_err(&spi->dev,
				"illegal data size for this controller!\n");
			dev_err(&spi->dev,
				"a standard lpc31xx can only handle "
				"1 <= n <= 16 bit words\n");
			status = -ENOTSUPP;
			goto err_config_params;
		}
	}

#if JDS
	/* Now Initialize all register settings required for this chip */
	chip->cr0 = 0;
	chip->cr1 = 0;
	chip->dmacr = 0;
	chip->cpsr = 0;
	if ((chip_info->com_mode == DMA_TRANSFER)
	    && ((lpc31xx->master_info)->enable_dma)) {
		chip->enable_dma = true;
		dev_dbg(&spi->dev, "DMA mode set in controller state\n");
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_ENABLED,
			       SSP_DMACR_MASK_RXDMAE, 0);
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_ENABLED,
			       SSP_DMACR_MASK_TXDMAE, 1);
	} else {
		chip->enable_dma = false;
		dev_dbg(&spi->dev, "DMA mode NOT set in controller state\n");
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_DISABLED,
			       SSP_DMACR_MASK_RXDMAE, 0);
		SSP_WRITE_BITS(chip->dmacr, SSP_DMA_DISABLED,
			       SSP_DMACR_MASK_TXDMAE, 1);
	}

	chip->cpsr = clk_freq.cpsdvsr;

	/* Special setup for the ST micro extended control registers */
	if (lpc31xx->vendor->extended_cr) {
		u32 etx;

		if (lpc31xx->vendor->pl023) {
			/* These bits are only in the PL023 */
			SSP_WRITE_BITS(chip->cr1, chip_info->clkdelay,
				       SSP_CR1_MASK_FBCLKDEL_ST, 13);
		} else {
			/* These bits are in the LPC31xx but not PL023 */
			SSP_WRITE_BITS(chip->cr0, chip_info->duplex,
				       SSP_CR0_MASK_HALFDUP_ST, 5);
			SSP_WRITE_BITS(chip->cr0, chip_info->ctrl_len,
				       SSP_CR0_MASK_CSS_ST, 16);
			SSP_WRITE_BITS(chip->cr0, chip_info->iface,
				       SSP_CR0_MASK_FRF_ST, 21);
			SSP_WRITE_BITS(chip->cr1, chip_info->wait_state,
				       SSP_CR1_MASK_MWAIT_ST, 6);
		}
		SSP_WRITE_BITS(chip->cr0, bits - 1,
			       SSP_CR0_MASK_DSS_ST, 0);

		if (spi->mode & SPI_LSB_FIRST) {
			tmp = SSP_RX_LSB;
			etx = SSP_TX_LSB;
		} else {
			tmp = SSP_RX_MSB;
			etx = SSP_TX_MSB;
		}
		SSP_WRITE_BITS(chip->cr1, tmp, SSP_CR1_MASK_RENDN_ST, 4);
		SSP_WRITE_BITS(chip->cr1, etx, SSP_CR1_MASK_TENDN_ST, 5);
		SSP_WRITE_BITS(chip->cr1, chip_info->rx_lev_trig,
			       SSP_CR1_MASK_RXIFLSEL_ST, 7);
		SSP_WRITE_BITS(chip->cr1, chip_info->tx_lev_trig,
			       SSP_CR1_MASK_TXIFLSEL_ST, 10);
	} else {
		SSP_WRITE_BITS(chip->cr0, bits - 1,
			       SSP_CR0_MASK_DSS, 0);
		SSP_WRITE_BITS(chip->cr0, chip_info->iface,
			       SSP_CR0_MASK_FRF, 4);
	}

	/* Stuff that is common for all versions */
	if (spi->mode & SPI_CPOL)
		tmp = SSP_CLK_POL_IDLE_HIGH;
	else
		tmp = SSP_CLK_POL_IDLE_LOW;
	SSP_WRITE_BITS(chip->cr0, tmp, SSP_CR0_MASK_SPO, 6);

	if (spi->mode & SPI_CPHA)
		tmp = SSP_CLK_SECOND_EDGE;
	else
		tmp = SSP_CLK_FIRST_EDGE;
	SSP_WRITE_BITS(chip->cr0, tmp, SSP_CR0_MASK_SPH, 7);

	SSP_WRITE_BITS(chip->cr0, clk_freq.scr, SSP_CR0_MASK_SCR, 8);
	/* Loopback is available on all versions except PL023 */
	if (lpc31xx->vendor->loopback) {
		if (spi->mode & SPI_LOOP)
			tmp = LOOPBACK_ENABLED;
		else
			tmp = LOOPBACK_DISABLED;
		SSP_WRITE_BITS(chip->cr1, tmp, SSP_CR1_MASK_LBM, 0);
	}
	SSP_WRITE_BITS(chip->cr1, SSP_DISABLED, SSP_CR1_MASK_SSE, 1);
	SSP_WRITE_BITS(chip->cr1, chip_info->hierarchy, SSP_CR1_MASK_MS, 2);
	SSP_WRITE_BITS(chip->cr1, chip_info->slave_tx_disable, SSP_CR1_MASK_SOD,
		3);
#endif

	/* Save controller_state */
	spi_set_ctldata(spi, chip);
	return status;
 err_config_params:
	spi_set_ctldata(spi, NULL);
	kfree(chip);
	return status;
}

/**
 * lpc31xx_cleanup - cleanup function registered to SPI master framework
 * @spi: spi device which is requesting cleanup
 *
 * This function is registered to the SPI framework for this SPI master
 * controller. It will free the runtime state of chip.
 */
static void lpc31xx_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	spi_set_ctldata(spi, NULL);
	kfree(chip);
}

static int __devinit
lpc31xx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_master *master;
	struct lpc31xx *lpc31xx = NULL;	/*Data for this driver */
	int status = 0;

	dev_info(&pdev->dev, "NXP LPC31xx SPI driver\n");

	/* Allocate master with space for data */
	master = spi_alloc_master(dev, sizeof(struct lpc31xx));
	if (master == NULL) {
		dev_err(&pdev->dev, "probe - cannot alloc SPI master\n");
		status = -ENOMEM;
		goto err_no_master;
	}

	lpc31xx = spi_master_get_devdata(master);
	lpc31xx->master = master;
	lpc31xx->master_info = platform_info;
	lpc31xx->pdev = pdev;

	/*
	 * Bus Number Which has been Assigned to this SSP controller
	 * on this board
	 */
	master->bus_num = platform_info->bus_id;
	master->num_chipselect = platform_info->num_chipselect;
	master->cleanup = lpc31xx_cleanup;
	master->setup = lpc31xx_setup;
	master->prepare_transfer_hardware = lpc31xx_prepare_transfer_hardware;
	master->transfer_one_message = lpc31xx_transfer_one_message;
	master->unprepare_transfer_hardware = lpc31xx_unprepare_transfer_hardware;
	master->rt = platform_info->rt;

	/*
	 * Supports mode 0-3, loopback, and active low CS. Transfers are
	 * always MS bit first on the original lpc31xx.
	 */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LOOP;
	if (lpc31xx->vendor->extended_cr)
		master->mode_bits |= SPI_LSB_FIRST;

	dev_dbg(&pdev->dev, "BUSNO: %d\n", master->bus_num);

	status = amba_request_regions(pdev, NULL);
	if (status)
		goto err_no_ioregion;

	lpc31xx->phybase = pdev->res.start;
	lpc31xx->virtbase = ioremap(pdev->res.start, resource_size(&pdev->res));
	if (lpc31xx->virtbase == NULL) {
		status = -ENOMEM;
		goto err_no_ioremap;
	}
	printk(KERN_INFO "lpc31xx: mapped registers from 0x%08x to %p\n",
	       pdev->res.start, lpc31xx->virtbase);

	lpc31xx->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(lpc31xx->clk)) {
		status = PTR_ERR(lpc31xx->clk);
		dev_err(&pdev->dev, "could not retrieve SSP/SPI bus clock\n");
		goto err_no_clk;
	}

	status = clk_prepare(lpc31xx->clk);
	if (status) {
		dev_err(&pdev->dev, "could not prepare SSP/SPI bus clock\n");
		goto  err_clk_prep;
	}

	status = clk_enable(lpc31xx->clk);
	if (status) {
		dev_err(&pdev->dev, "could not enable SSP/SPI bus clock\n");
		goto err_no_clk_en;
	}

	/* Initialize transfer pump */
	tasklet_init(&lpc31xx->pump_transfers, pump_transfers,
		     (unsigned long)lpc31xx);

	/* Disable SSP */
	writew((readw(SSP_CR1(lpc31xx->virtbase)) & (~SSP_CR1_MASK_SSE)),
	       SSP_CR1(lpc31xx->virtbase));
	load_spi_default_config(lpc31xx);

	status = request_irq(pdev->irq[0], lpc31xx_interrupt_handler, 0, "lpc31xx",
			     lpc31xx);
	if (status < 0) {
		dev_err(&pdev->dev, "probe - cannot get IRQ (%d)\n", status);
		goto err_no_irq;
	}

	/* Get DMA channels */
	if (platform_info->enable_dma) {
		status = lpc31xx_dma_probe(lpc31xx);
		if (status != 0)
			platform_info->enable_dma = 0;
	}

	/* Register with the SPI framework */
	amba_set_drvdata(pdev, lpc31xx);
	status = spi_register_master(master);
	if (status != 0) {
		dev_err(&pdev->dev,
			"probe - problem registering spi master\n");
		goto err_spi_register;
	}
	dev_dbg(dev, "probe succeeded\n");

	/* let runtime pm put suspend */
	if (platform_info->autosuspend_delay > 0) {
		dev_info(&pdev->dev,
			"will use autosuspend for runtime pm, delay %dms\n",
			platform_info->autosuspend_delay);
		pm_runtime_set_autosuspend_delay(dev,
			platform_info->autosuspend_delay);
		pm_runtime_use_autosuspend(dev);
		pm_runtime_put_autosuspend(dev);
	} else {
		pm_runtime_put(dev);
	}
	return 0;

 err_spi_register:
	if (platform_info->enable_dma)
		lpc31xx_dma_remove(lpc31xx);

	free_irq(pdev->irq[0], lpc31xx);
 err_no_irq:
	clk_disable(lpc31xx->clk);
 err_no_clk_en:
	clk_unprepare(lpc31xx->clk);
 err_clk_prep:
	clk_put(lpc31xx->clk);
 err_no_clk:
	iounmap(lpc31xx->virtbase);
 err_no_ioremap:
	amba_release_regions(pdev);
 err_no_ioregion:
	spi_master_put(master);
 err_no_master:
 err_no_pdata:
	return status;
}

static int __devexit
lpc31xx_remove(struct platform_device *pdev)
{
	struct lpc31xx *lpc31xx = platform_get_drvdata(pdev);

	if (!lpc31xx)
		return 0;

	/*
	 * undo pm_runtime_put() in probe.  I assume that we're not
	 * accessing the primecell here.
	 */
	pm_runtime_get_noresume(&pdev->dev);

	load_spi_default_config(lpc31xx);
	if (lpc31xx->master_info->enable_dma)
		lpc31xx_dma_remove(lpc31xx);

	free_irq(pdev->irq[0], lpc31xx);
	clk_disable(lpc31xx->clk);
	clk_unprepare(lpc31xx->clk);
	clk_put(lpc31xx->clk);
	iounmap(lpc31xx->virtbase);
	amba_release_regions(pdev);
	tasklet_disable(&lpc31xx->pump_transfers);
	spi_unregister_master(lpc31xx->master);
	spi_master_put(lpc31xx->master);
	amba_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int lpc31xx_suspend(struct device *dev)
{
	struct lpc31xx *lpc31xx = dev_get_drvdata(dev);
	int ret;

	ret = spi_master_suspend(lpc31xx->master);
	if (ret) {
		dev_warn(dev, "cannot suspend master\n");
		return ret;
	}

	dev_dbg(dev, "suspended\n");
	return 0;
}

static int lpc31xx_resume(struct device *dev)
{
	struct lpc31xx *lpc31xx = dev_get_drvdata(dev);
	int ret;

	/* Start the queue running */
	ret = spi_master_resume(lpc31xx->master);
	if (ret)
		dev_err(dev, "problem starting queue (%d)\n", ret);
	else
		dev_dbg(dev, "resumed\n");

	return ret;
}
#endif	/* CONFIG_PM */

#ifdef CONFIG_PM_RUNTIME
static int lpc31xx_runtime_suspend(struct device *dev)
{
	struct lpc31xx *lpc31xx = dev_get_drvdata(dev);

	clk_disable(lpc31xx->clk);

	return 0;
}

static int lpc31xx_runtime_resume(struct device *dev)
{
	struct lpc31xx *lpc31xx = dev_get_drvdata(dev);

	clk_enable(lpc31xx->clk);

	return 0;
}
#endif

static const struct dev_pm_ops lpc31xx_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lpc31xx_suspend, lpc31xx_resume)
	SET_RUNTIME_PM_OPS(lpc31xx_runtime_suspend, lpc31xx_runtime_resume, NULL)
};


static struct platform_driver lpc31xx_spi_driver = {
	.probe		= lpc31xx_probe,
	.remove		= __devexit_p(lpc31xx_remove),
	.driver = {
		.name	= "spi-lpc31xx",
		.pm	= &lpc31xx_dev_pm_ops,
		.owner	= THIS_MODULE,
	},
};

static int __init lpc31xx_init(void)
{
	return platform_driver_register(&lpc31xx_spi_driver);
}
subsys_initcall(lpc31xx_init);

static void __exit lpc31xx_exit(void)
{
	platform_driver_unregister(&lpc31xx_spi_driver);
}
module_exit(lpc31xx_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC31xx SPI Controller Driver");
MODULE_LICENSE("GPL");
