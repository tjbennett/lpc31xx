/*
 * A driver for the LPC31xx SPI bus master.
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

//#define DEBUG

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
#include <linux/of_gpio.h>

//#define d_printk(args...) {printk(args);}
#define d_printk(args...) {}

/* Register access macros */
#define spi_readl(reg) _spi_readl(&SPI_##reg)
#define spi_writel(reg,value) _spi_writel(&SPI_##reg, (value))

static inline void
_spi_writel(volatile u32 *reg, uint32_t value)
{
       //printk("JDS - lpc31xx_spi_write %p value %x\n", reg, value);
       __raw_writel(value, reg);
}

static inline uint32_t
_spi_readl(volatile u32 *reg)
{
       uint32_t value;
       value = __raw_readl(reg);
       //printk("JDS - lpc31xx_spi_read %p value %x\n", reg, value);
       return value;
}



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
#define SPI_TXFF_FLUSH            _BIT(0)

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

/* SPI interrupt registers definitions ( SPI_INT_xxx) */
#define SPI_SMS_INT               _BIT(4)
#define SPI_TX_INT                _BIT(3)
#define SPI_RX_INT                _BIT(2)
#define SPI_TO_INT                _BIT(1)
#define SPI_OVR_INT               _BIT(0)
#define SPI_ALL_INTS              (SPI_SMS_INT | SPI_TX_INT | SPI_RX_INT | SPI_TO_INT | SPI_OVR_INT)

#define SPI_POLLING_TIMEOUT 8000
#define MAX_CHIP_SELECT 3

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
};

/**
 * The type of writing going on on this chip
 */
enum spi_writing {
	WRITING_NULL,
	WRITING_U8,
	WRITING_U16,
};

/**
 * enum ssp_rx_level_trig - receive FIFO watermark level which triggers
 * IT: Interrupt fires when _N_ or more elements in RX FIFO.
 */
enum spi_rx_level_trig {
	SSP_RX_1_OR_MORE_ELEM,
	SSP_RX_4_OR_MORE_ELEM,
	SSP_RX_8_OR_MORE_ELEM,
	SSP_RX_16_OR_MORE_ELEM,
	SSP_RX_32_OR_MORE_ELEM
};

/**
 * Transmit FIFO watermark level which triggers (IT Interrupt fires
 * when _N_ or more empty locations in TX FIFO)
 */
enum spi_tx_level_trig {
	SSP_TX_1_OR_MORE_EMPTY_LOC,
	SSP_TX_4_OR_MORE_EMPTY_LOC,
	SSP_TX_8_OR_MORE_EMPTY_LOC,
	SSP_TX_16_OR_MORE_EMPTY_LOC,
	SSP_TX_32_OR_MORE_EMPTY_LOC
};

/**
 * enum ssp_mode - SSP mode of operation (Communication modes)
 */
enum spi_mode {
	INTERRUPT_TRANSFER,
	POLLING_TRANSFER,
	DMA_TRANSFER
};

/**
 * enum spi_clock_params - clock parameters, to set SPI clock at a
 * desired freq
 */
struct spi_clock_params {
	uint8_t cpsdvsr; /* value from 2 to 254 (even only!) */
	uint8_t scr;	    /* value from 0 to 255 */
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
 * struct lpc31xx_spi - This is the private SSP driver data structure
 * @pdev: Platform device model hookup
 * @vendor: vendor data for the IP block
 * @phybase: the physical memory where the SSP device resides
 * @virtbase: the virtual memory where the SSP is mapped
 * @clk: outgoing clock "SPICLK" for the SPI bus
 * @master: SPI framework hookup
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
struct lpc31xx_spi {
	struct platform_device		*pdev;
	resource_size_t			phybase;
	void __iomem			*virtbase;
	struct clk			*clk;
	struct spi_master		*master;
	int				irq;
	/* Message per-transfer pump */
	struct tasklet_struct		pump_transfers;
	struct spi_message		*cur_msg;
	struct spi_transfer		*cur_transfer;
	struct lpc31xx_spi_chip		*cur_chip;
	bool				next_msg_cs_active;
	void				*tx;
	void				*tx_end;
	void				*rx;
	void				*rx_end;
	enum spi_reading		read;
	enum spi_writing		write;
	uint32_t				exp_fifo_level;
	enum spi_rx_level_trig		rx_lev_trig;
	enum spi_tx_level_trig		tx_lev_trig;
	int				gpio[MAX_CHIP_SELECT];
	int				alow[MAX_CHIP_SELECT];
	uint8_t 			current_bits_wd[MAX_CHIP_SELECT];
	int				current_speed_hz[MAX_CHIP_SELECT];

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
 * struct lpc31xx_spi_chip - To maintain runtime state of SSP for each client chip
 * @cr0: Value of control register CR0 of SSP - on later ST variants this
 *       register is 32 bits wide rather than just 16
 * @cr1: Value of control register CR1 of SSP
 * @dmacr: Value of DMA control Register of SSP
 * @cpsr: Value of Clock prescale register
 * @n_bytes: how many bytes(power of 2) reqd for a given data width of client
 * @enable_dma: Whether to enable DMA or not
 * @read: function ptr to be used to read when doing xfer for this chip
 * @write: function ptr to be used to write when doing xfer for this chip
 * @xfer_type: polling/interrupt/DMA
 *
 * Runtime state of the SSP controller, maintained per chip,
 * This would be set according to the current message that would be served
 */
struct lpc31xx_spi_chip {
	uint32_t cr0;
	uint16_t cr1;
	uint16_t dmacr;
	uint16_t cpsr;
	uint8_t n_bytes;
	bool enable_dma;
	enum spi_reading read;
	enum spi_writing write;
	int xfer_type;
	int chip_select;
};

/*
 * Clear a latched SPI interrupt
 */
static inline void lpc31xx_int_clr(struct lpc31xx_spi *espi, u32 ints)
{
	spi_writel(INT_CLRS_REG, ints);
}

/*
 * Disable a SPI interrupt
 */
static inline void lpc31xx_int_dis(struct lpc31xx_spi *espi, u32 ints)
{
	spi_writel(INT_CLRE_REG, ints);
}

/*
 * Enable a SPI interrupt
 */
static inline void lpc31xx_int_en(struct lpc31xx_spi *espi, u32 ints)
{
	spi_writel(INT_SETE_REG, ints);
}

/*
 * Set data width for the SPI chip select
 */
static void lpc31xx_set_cs_data_bits(struct lpc31xx_spi *espi, uint8_t cs, uint8_t data_width)
{
	if (espi->current_bits_wd[cs] != data_width)
	{
		u32 tmp = spi_readl(SLV_SET2_REG(0));
		tmp &= ~SPI_SLV2_WD_SZ(0x1F);
		tmp |= SPI_SLV2_WD_SZ((u32) (data_width - 1));
		spi_writel(SLV_SET2_REG(0), tmp);

		espi->current_bits_wd[cs] = data_width;
	}
}

/*
 * Set clock rate and delays for the SPI chip select
 */
static void lpc31xx_set_cs_clock(struct lpc31xx_spi *espi, uint8_t cs, u32 clockrate)
{
	u32 reg, div, ps, div1;

	if (clockrate != espi->current_speed_hz[cs])
	{
		reg = spi_readl(SLV_SET1_REG(0));
		reg &= ~0xFFFF;

		div = (clk_get_rate(espi->clk) + clockrate / 2) / clockrate;
		if (div > SPI_MAX_DIVIDER)
			div = SPI_MAX_DIVIDER;
		if (div < SPI_MIN_DIVIDER)
			div = SPI_MIN_DIVIDER;

		ps = (((div - 1) / 512) + 1) * 2;
		div1 = (((div + ps / 2) / ps) - 1);

		spi_writel(SLV_SET1_REG(0),
			(reg | SPI_SLV1_CLK_PS(ps) | SPI_SLV1_CLK_DIV1(div1)));

		espi->current_speed_hz[cs] = clockrate;
	}
}

/*
 * Flush the TX and RX FIFOs
 */
static int lpc31xx_fifo_flush(struct lpc31xx_spi *espi)
{
	unsigned long timeout;
	volatile uint32_t tmp;

	/* Clear TX FIFO first */
	spi_writel(TXF_FLUSH_REG, SPI_TXFF_FLUSH);

	/* Clear RX FIFO */
	timeout = jiffies + msecs_to_jiffies(SPI_POLLING_TIMEOUT);
	while (!(spi_readl(STS_REG) & SPI_ST_RX_EMPTY)) {
		if (time_after(jiffies, timeout)) {
			dev_warn(&espi->pdev->dev,
				 "timeout while flushing RX FIFO\n");
			return -ETIMEDOUT;
		}
		tmp = spi_readl(FIFO_DATA_REG);
	}
	return 0;
}


/*
 * Enable or disable the SPI clocks
 */
static void lpc31xx_spi_clks_enable(struct lpc31xx_spi *espi)
{
	struct clk *clk;
	int ret;

	clk = clk_get(NULL, "spi_pclk");
	ret = clk_enable(clk);
	clk_put(clk);
	espi->clk = clk;
	clk = clk_get(NULL, "spi_pclk_gated");
	ret = clk_enable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "spi_clk");
	ret = clk_enable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "spi_clk_gated");
	ret = clk_enable(clk);
	clk_put(clk);
}

static void lpc31xx_spi_clks_disable(void)
{
	struct clk *clk;

	clk = clk_get(NULL, "spi_pclk");
	clk_disable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "spi_pclk_gated");
	clk_disable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "spi_clk");
	clk_disable(clk);
	clk_put(clk);
	clk = clk_get(NULL, "spi_clk_gated");
	clk_disable(clk);
	clk_put(clk);
}

/**
 * null_cs_control - Dummy chip select function
 * @command: select/deselect the chip
 *
 * If no chip select function is provided by client this is used as dummy
 * chip select
 */
static void lpc31xx_cs_control(struct lpc31xx_spi *espi, uint32_t control)
{
	dev_dbg(&espi->pdev->dev, "cs_control CS=0x%x gpio %d control 0x%x\n",
			espi->cur_chip->chip_select, espi->gpio[espi->cur_chip->chip_select], control);
	gpio_set_value(espi->gpio[espi->cur_chip->chip_select], !control);
}

/**
 * giveback - current spi_message is over, schedule next message and call
 * callback of this message. Assumes that caller already
 * set message->status; dma and pio irqs are blocked
 * @espi: SPI driver private data structure
 */
static void giveback(struct lpc31xx_spi *espi)
{
	struct spi_transfer *last_transfer;
	espi->next_msg_cs_active = false;

	last_transfer = list_entry(espi->cur_msg->transfers.prev,
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
		next_msg = spi_get_next_queued_message(espi->master);

		/*
		 * see if the next and current messages point
		 * to the same spi device.
		 */

		if (next_msg && next_msg->spi != espi->cur_msg->spi)
			next_msg = NULL;
		if (!next_msg || espi->cur_msg->state == STATE_ERROR)
			lpc31xx_cs_control(espi, false);
		else
			espi->next_msg_cs_active = true;

	}

	espi->cur_msg = NULL;
	espi->cur_transfer = NULL;
	espi->cur_chip = NULL;
	spi_finalize_current_message(espi->master);
}

/**
 * flush - flush the FIFO to reach a clean state
 * @espi: SPI driver private data structure
 */
static int flush(struct lpc31xx_spi *espi)
{
	unsigned long limit = loops_per_jiffy << 1;

	dev_dbg(&espi->pdev->dev, "flush\n");
#ifdef JDS
	do {
		while (readw(SSP_SR(espi->virtbase)) & SSP_SR_MASK_RNE)
			readw(SSP_DR(espi->virtbase));
	} while ((readw(SSP_SR(espi->virtbase)) & SSP_SR_MASK_BSY) && limit--);
#endif
	espi->exp_fifo_level = 0;

	return limit;
}

/**
 * restore_state - Load configuration of current chip
 * @espi: SPI driver private data structure
 */
static void restore_state(struct lpc31xx_spi *espi)
{
	struct lpc31xx_spi_chip *chip = espi->cur_chip;

#ifdef JDS
	if (espi->vendor->extended_cr)
		writel(chip->cr0, SSP_CR0(espi->virtbase));
	else
		writew(chip->cr0, SSP_CR0(espi->virtbase));
	writew(chip->cr1, SSP_CR1(espi->virtbase));
	writew(chip->dmacr, SSP_DMACR(espi->virtbase));
	writew(chip->cpsr, SSP_CPSR(espi->virtbase));
	writew(DISABLE_ALL_INTERRUPTS, SSP_IMSC(espi->virtbase));
	writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(espi->virtbase));
#endif
}


/**
 * This will write to TX and read from RX according to the parameters
 * set in espi.
 */
static void readwriter(struct lpc31xx_spi *espi)
{

	/*
	 * The FIFO depth is different between primecell variants.
	 * I believe filling in too much in the FIFO might cause
	 * errors in 8bit wide transfers on ARM variants (just 8 words
	 * FIFO, means only 8x8 = 64 bits in FIFO) at least.
	 *
	 * To prevent this issue, the TX FIFO is only filled to the
	 * unused RX FIFO fill length, regardless of what the TX
	 * FIFO status flag indicates.
	 */
	dev_dbg(&espi->pdev->dev,
		"%s, rx: %p, rxend: %p, tx: %p, txend: %p bytes %d\n",
		__func__, espi->rx, espi->rx_end, espi->tx, espi->tx_end, espi->cur_chip->n_bytes);

	/* Set the FIFO trip level to the transfer size */
#ifdef JDS
	spi_writel(INT_TRSH_REG, (SPI_INT_TSHLD_TX(0) | SPI_INT_TSHLD_RX(t->len - 1)));
#endif
	spi_writel(DMA_SET_REG, 0);

	/* read as long as RX FIFO has frames in it */
	while ((!(spi_readl(STS_REG) & SPI_ST_RX_EMPTY)) && (espi->rx < espi->rx_end)) {
		switch (espi->read) {
		case READING_NULL:
			spi_readl(FIFO_DATA_REG);
			break;
		case READING_U8:
			*(uint8_t *)(espi->rx) = spi_readl(FIFO_DATA_REG) & 0xFFU;
			d_printk("%02x ", *(uint8_t *)(espi->rx));
			break;
		case READING_U16:
			*(uint16_t *)(espi->rx) = (uint16_t)spi_readl(FIFO_DATA_REG);;
			break;
		}
		espi->rx += (espi->cur_chip->n_bytes);
		espi->exp_fifo_level--;
	}

	/* write as long as TX FIFO has room */
	while ((espi->exp_fifo_level < SPI_FIFO_DEPTH) && (espi->tx < espi->tx_end)) {
		switch (espi->write) {
		case WRITING_NULL:
			spi_writel(FIFO_DATA_REG, -1);
			break;
		case WRITING_U8:
			d_printk("%02x ", *(uint8_t *)(espi->tx));
			spi_writel(FIFO_DATA_REG, *(uint8_t *) (espi->tx));
			break;
		case WRITING_U16:
			spi_writel(FIFO_DATA_REG, *(uint16_t *) (espi->tx));
			break;
		}
		espi->tx += (espi->cur_chip->n_bytes);
		espi->exp_fifo_level++;
		/* read as long as RX FIFO has frames in it */
		while ((!(spi_readl(STS_REG) & SPI_ST_RX_EMPTY)) && (espi->rx < espi->rx_end)) {
			switch (espi->read) {
			case READING_NULL:
				spi_readl(FIFO_DATA_REG);
				break;
			case READING_U8:
				*(uint8_t *)(espi->rx) = spi_readl(FIFO_DATA_REG) & 0xFFU;
				d_printk("%02x ", *(uint8_t *)(espi->rx));
				break;
			case READING_U16:
				*(uint16_t *)(espi->rx) = (uint16_t)spi_readl(FIFO_DATA_REG);;
				break;
			}
			espi->rx += (espi->cur_chip->n_bytes);
			espi->exp_fifo_level--;
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
static void *next_transfer(struct lpc31xx_spi *espi)
{
	struct spi_message *msg = espi->cur_msg;
	struct spi_transfer *trans = espi->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		espi->cur_transfer =
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
static void unmap_free_dma_scatter(struct lpc31xx_spi *espi)
{
	/* Unmap and free the SG tables */
	dma_unmap_sg(espi->dma_tx_channel->device->dev, espi->sgt_tx.sgl,
		     espi->sgt_tx.nents, DMA_TO_DEVICE);
	dma_unmap_sg(espi->dma_rx_channel->device->dev, espi->sgt_rx.sgl,
		     espi->sgt_rx.nents, DMA_FROM_DEVICE);
	sg_free_table(&espi->sgt_rx);
	sg_free_table(&espi->sgt_tx);
}

static void dma_callback(void *data)
{
	struct lpc31xx_spi *espi = data;
	struct spi_message *msg = espi->cur_msg;

	BUG_ON(!espi->sgt_rx.sgl);

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

		dma_sync_sg_for_cpu(&espi->pdev->dev,
				    espi->sgt_rx.sgl,
				    espi->sgt_rx.nents,
				    DMA_FROM_DEVICE);

		for_each_sg(espi->sgt_rx.sgl, sg, espi->sgt_rx.nents, i) {
			dev_dbg(&espi->pdev->dev, "SPI RX SG ENTRY: %d", i);
			print_hex_dump(KERN_ERR, "SPI RX: ",
				       DUMP_PREFIX_OFFSET,
				       16,
				       1,
				       sg_virt(sg),
				       sg_dma_len(sg),
				       1);
		}
		for_each_sg(espi->sgt_tx.sgl, sg, espi->sgt_tx.nents, i) {
			dev_dbg(&espi->pdev->dev, "SPI TX SG ENTRY: %d", i);
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

	unmap_free_dma_scatter(espi);

	/* Update total bytes transferred */
	msg->actual_length += espi->cur_transfer->len;
	if (espi->cur_transfer->cs_change)
		lpc31xx_cs_control(espi, false);

	/* Move to next transfer */
	msg->state = next_transfer(espi);
	tasklet_schedule(&espi->pump_transfers);
}

static void setup_dma_scatter(struct lpc31xx_spi *espi,
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
			dev_dbg(&espi->pdev->dev,
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
			sg_set_page(sg, virt_to_page(espi->dummypage),
				    mapbytes, 0);
			bytesleft -= mapbytes;
			dev_dbg(&espi->pdev->dev,
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
static int configure_dma(struct lpc31xx_spi *espi)
{
	struct dma_slave_config rx_conf = {
		.src_addr = SSP_DR(espi->phybase),
		.direction = DMA_DEV_TO_MEM,
		.device_fc = false,
	};
	struct dma_slave_config tx_conf = {
		.dst_addr = SSP_DR(espi->phybase),
		.direction = DMA_MEM_TO_DEV,
		.device_fc = false,
	};
	unsigned int pages;
	int ret;
	int rx_sglen, tx_sglen;
	struct dma_chan *rxchan = espi->dma_rx_channel;
	struct dma_chan *txchan = espi->dma_tx_channel;
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
	switch (espi->rx_lev_trig) {
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
		rx_conf.src_maxburst = espi->vendor->fifodepth >> 1;
		break;
	}

	switch (espi->tx_lev_trig) {
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
		tx_conf.dst_maxburst = espi->vendor->fifodepth >> 1;
		break;
	}

	switch (espi->read) {
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

	switch (espi->write) {
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
	pages = DIV_ROUND_UP(espi->cur_transfer->len, PAGE_SIZE);
	dev_dbg(&espi->pdev->dev, "using %d pages for transfer\n", pages);

	ret = sg_alloc_table(&espi->sgt_rx, pages, GFP_ATOMIC);
	if (ret)
		goto err_alloc_rx_sg;

	ret = sg_alloc_table(&espi->sgt_tx, pages, GFP_ATOMIC);
	if (ret)
		goto err_alloc_tx_sg;

	/* Fill in the scatterlists for the RX+TX buffers */
	setup_dma_scatter(espi, espi->rx,
			  espi->cur_transfer->len, &espi->sgt_rx);
	setup_dma_scatter(espi, espi->tx,
			  espi->cur_transfer->len, &espi->sgt_tx);

	/* Map DMA buffers */
	rx_sglen = dma_map_sg(rxchan->device->dev, espi->sgt_rx.sgl,
			   espi->sgt_rx.nents, DMA_FROM_DEVICE);
	if (!rx_sglen)
		goto err_rx_sgmap;

	tx_sglen = dma_map_sg(txchan->device->dev, espi->sgt_tx.sgl,
			   espi->sgt_tx.nents, DMA_TO_DEVICE);
	if (!tx_sglen)
		goto err_tx_sgmap;

	/* Send both scatter lists */
	rxdesc = dmaengine_prep_slave_sg(rxchan,
				      espi->sgt_rx.sgl,
				      rx_sglen,
				      DMA_DEV_TO_MEM,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc)
		goto err_rxdesc;

	txdesc = dmaengine_prep_slave_sg(txchan,
				      espi->sgt_tx.sgl,
				      tx_sglen,
				      DMA_MEM_TO_DEV,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc)
		goto err_txdesc;

	/* Put the callback on the RX transfer only, that should finish last */
	rxdesc->callback = dma_callback;
	rxdesc->callback_param = espi;

	/* Submit and fire RX and TX with TX last so we're ready to read! */
	dmaengine_submit(rxdesc);
	dmaengine_submit(txdesc);
	dma_async_issue_pending(rxchan);
	dma_async_issue_pending(txchan);
	espi->dma_running = true;

	return 0;

err_txdesc:
	dmaengine_terminate_all(txchan);
err_rxdesc:
	dmaengine_terminate_all(rxchan);
	dma_unmap_sg(txchan->device->dev, espi->sgt_tx.sgl,
		     espi->sgt_tx.nents, DMA_TO_DEVICE);
err_tx_sgmap:
	dma_unmap_sg(rxchan->device->dev, espi->sgt_rx.sgl,
		     espi->sgt_tx.nents, DMA_FROM_DEVICE);
err_rx_sgmap:
	sg_free_table(&espi->sgt_tx);
err_alloc_tx_sg:
	sg_free_table(&espi->sgt_rx);
err_alloc_rx_sg:
	return -ENOMEM;
}

static int __devinit lpc31xx_dma_probe(struct lpc31xx_spi *espi)
{
	dma_cap_mask_t mask;

	/* Try to acquire a generic DMA engine slave channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	/*
	 * We need both RX and TX channels to do DMA, else do none
	 * of them.
	 */
	espi->dma_rx_channel = dma_request_channel(mask,
					    espi->master_info->dma_filter,
					    espi->master_info->dma_rx_param);
	if (!espi->dma_rx_channel) {
		dev_dbg(&espi->pdev->dev, "no RX DMA channel!\n");
		goto err_no_rxchan;
	}

	espi->dma_tx_channel = dma_request_channel(mask,
					    espi->master_info->dma_filter,
					    espi->master_info->dma_tx_param);
	if (!espi->dma_tx_channel) {
		dev_dbg(&espi->pdev->dev, "no TX DMA channel!\n");
		goto err_no_txchan;
	}

	espi->dummypage = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!espi->dummypage) {
		dev_dbg(&espi->pdev->dev, "no DMA dummypage!\n");
		goto err_no_dummypage;
	}

	dev_info(&espi->pdev->dev, "setup for DMA on RX %s, TX %s\n",
		 dma_chan_name(espi->dma_rx_channel),
		 dma_chan_name(espi->dma_tx_channel));

	return 0;

err_no_dummypage:
	dma_release_channel(espi->dma_tx_channel);
err_no_txchan:
	dma_release_channel(espi->dma_rx_channel);
	espi->dma_rx_channel = NULL;
err_no_rxchan:
	dev_err(&espi->pdev->dev,
			"Failed to work in dma mode, work without dma!\n");
	return -ENODEV;
}

static void terminate_dma(struct lpc31xx_spi *espi)
{
	struct dma_chan *rxchan = espi->dma_rx_channel;
	struct dma_chan *txchan = espi->dma_tx_channel;

	dmaengine_terminate_all(rxchan);
	dmaengine_terminate_all(txchan);
	unmap_free_dma_scatter(espi);
	espi->dma_running = false;
}

static void lpc31xx_dma_remove(struct lpc31xx_spi *espi)
{
	if (espi->dma_running)
		terminate_dma(espi);
	if (espi->dma_tx_channel)
		dma_release_channel(espi->dma_tx_channel);
	if (espi->dma_rx_channel)
		dma_release_channel(espi->dma_rx_channel);
	kfree(espi->dummypage);
}

#else
static inline int configure_dma(struct lpc31xx_spi *espi)
{
	return -ENODEV;
}

static inline int lpc31xx_dma_probe(struct lpc31xx_spi *espi)
{
	return 0;
}

static inline void lpc31xx_dma_remove(struct lpc31xx_spi *espi)
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
	struct lpc31xx_spi *espi = dev_id;
	struct spi_message *msg = espi->cur_msg;
	uint16_t irq_status = 0;
	uint16_t flag = 0;

	dev_dbg(&espi->pdev->dev, "lpc31xx_interrupt_handler\n");

	if (unlikely(!msg)) {
		dev_err(&espi->pdev->dev,
			"bad message state in interrupt handler");
		/* Never fail */
		return IRQ_HANDLED;
	}
#ifdef JDS
	/* Read the Interrupt Status Register */
	irq_status = readw(SSP_MIS(espi->virtbase));

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
		dev_err(&espi->pdev->dev, "FIFO overrun\n");
		if (readw(SSP_SR(espi->virtbase)) & SSP_SR_MASK_RFF)
			dev_err(&espi->pdev->dev,
				"RXFIFO is full\n");
		if (readw(SSP_SR(espi->virtbase)) & SSP_SR_MASK_TNF)
			dev_err(&espi->pdev->dev,
				"TXFIFO is full\n");

		/*
		 * Disable and clear interrupts, disable SSP,
		 * mark message with bad status so it can be
		 * retried.
		 */
		writew(DISABLE_ALL_INTERRUPTS,
		       SSP_IMSC(espi->virtbase));
		writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(espi->virtbase));
		writew((readw(SSP_CR1(espi->virtbase)) &
			(~SSP_CR1_MASK_SSE)), SSP_CR1(espi->virtbase));
		msg->state = STATE_ERROR;

		/* Schedule message queue handler */
		tasklet_schedule(&espi->pump_transfers);
		return IRQ_HANDLED;
	}

	readwriter(espi);

	if ((espi->tx == espi->tx_end) && (flag == 0)) {
		flag = 1;
		/* Disable Transmit interrupt, enable receive interrupt */
		writew((readw(SSP_IMSC(espi->virtbase)) &
		       ~SSP_IMSC_MASK_TXIM) | SSP_IMSC_MASK_RXIM,
		       SSP_IMSC(espi->virtbase));
	}

	/*
	 * Since all transactions must write as much as shall be read,
	 * we can conclude the entire transaction once RX is complete.
	 * At this point, all TX will always be finished.
	 */
	if (espi->rx >= espi->rx_end) {
		writew(DISABLE_ALL_INTERRUPTS,
		       SSP_IMSC(espi->virtbase));
		writew(CLEAR_ALL_INTERRUPTS, SSP_ICR(espi->virtbase));
		if (unlikely(espi->rx > espi->rx_end)) {
			dev_warn(&espi->pdev->dev, "read %u surplus "
				 "bytes (did you request an odd "
				 "number of bytes on a 16bit bus?)\n",
				 (uint32_t) (espi->rx - espi->rx_end));
		}
		/* Update total bytes transferred */
		msg->actual_length += espi->cur_transfer->len;
		if (espi->cur_transfer->cs_change)
			lpc31xx_cs_control(espi, false);
		/* Move to next transfer */
		msg->state = next_transfer(espi);
		tasklet_schedule(&espi->pump_transfers);
		return IRQ_HANDLED;
	}
#endif
	return IRQ_HANDLED;
}

/**
 * This sets up the pointers to memory for the next message to
 * send out on the SPI bus.
 */
static int set_up_next_transfer(struct lpc31xx_spi *espi,
				struct spi_transfer *transfer)
{
	int residue;

	dev_dbg(&espi->pdev->dev, "set_up_next_transfer\n");

	/* Sanity check the message for this bus width */
	residue = espi->cur_transfer->len % espi->cur_chip->n_bytes;
	if (unlikely(residue != 0)) {
		dev_err(&espi->pdev->dev,
			"message of %u bytes to transmit but the current "
			"chip bus has a data width of %u bytes!\n",
			espi->cur_transfer->len,
			espi->cur_chip->n_bytes);
		dev_err(&espi->pdev->dev, "skipping this message\n");
		return -EIO;
	}
	espi->tx = (void *)transfer->tx_buf;
	espi->tx_end = espi->tx + espi->cur_transfer->len;
	espi->rx = (void *)transfer->rx_buf;
	espi->rx_end = espi->rx + espi->cur_transfer->len;
	espi->write =
	    espi->tx ? espi->cur_chip->write : WRITING_NULL;
	espi->read = espi->rx ? espi->cur_chip->read : READING_NULL;
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
	struct lpc31xx_spi *espi = (struct lpc31xx_spi *) data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;

	dev_dbg(&espi->pdev->dev, "pump_transfers\n");

	/* Get current state information */
	message = espi->cur_msg;
	transfer = espi->cur_transfer;

	/* Handle for abort */
	if (message->state == STATE_ERROR) {
		message->status = -EIO;
		giveback(espi);
		return;
	}

	/* Handle end of message */
	if (message->state == STATE_DONE) {
		message->status = 0;
		giveback(espi);
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
			lpc31xx_cs_control(espi, true);
	} else {
		/* STATE_START */
		message->state = STATE_RUNNING;
	}

	if (set_up_next_transfer(espi, transfer)) {
		message->state = STATE_ERROR;
		message->status = -EIO;
		giveback(espi);
		return;
	}
	/* Flush the FIFOs and let's go! */
	flush(espi);

	if (espi->cur_chip->enable_dma) {
		if (configure_dma(espi)) {
			dev_dbg(&espi->pdev->dev,
				"configuration of DMA failed, fall back to interrupt mode\n");
			goto err_config_dma;
		}
		return;
	}

err_config_dma:
#ifdef JDS
	/* enable all interrupts except RX */
	writew(ENABLE_ALL_INTERRUPTS & ~SSP_IMSC_MASK_RXIM, SSP_IMSC(espi->virtbase));
#endif
	return;
}

static void do_interrupt_dma_transfer(struct lpc31xx_spi *espi)
{
	dev_dbg(&espi->pdev->dev, "do_interrupt_dma_transfer\n");

#ifdef JDS
	/*
	 * Default is to enable all interrupts except RX -
	 * this will be enabled once TX is complete
	 */
	uint32_t irqflags = ENABLE_ALL_INTERRUPTS & ~SSP_IMSC_MASK_RXIM;

	/* Enable target chip, if not already active */
	if (!espi->next_msg_cs_active)
		lpc31xx_cs_control(espi, true);

	if (set_up_next_transfer(espi, espi->cur_transfer)) {
		/* Error path */
		espi->cur_msg->state = STATE_ERROR;
		espi->cur_msg->status = -EIO;
		giveback(espi);
		return;
	}
	/* If we're using DMA, set up DMA here */
	if (espi->cur_chip->enable_dma) {
		/* Configure DMA transfer */
		if (configure_dma(espi)) {
			dev_dbg(&espi->pdev->dev,
				"configuration of DMA failed, fall back to interrupt mode\n");
			goto err_config_dma;
		}
		/* Disable interrupts in DMA mode, IRQ from DMA controller */
		irqflags = DISABLE_ALL_INTERRUPTS;
	}
err_config_dma:
	/* Enable SSP, turn on interrupts */
	writew((readw(SSP_CR1(espi->virtbase)) | SSP_CR1_MASK_SSE),
	       SSP_CR1(espi->virtbase));
	writew(irqflags, SSP_IMSC(espi->virtbase));
#endif
}

static void do_polling_transfer(struct lpc31xx_spi *espi)
{
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct lpc31xx_spi_chip *chip;
	unsigned long time, timeout;
	uint32_t tmp;

	dev_dbg(&espi->pdev->dev, "do_polling_transfer\n");

	chip = espi->cur_chip;
	message = espi->cur_msg;

	while (message->state != STATE_DONE) {
		/* Handle for abort */
		if (message->state == STATE_ERROR)
			break;
		transfer = espi->cur_transfer;

		/* Setup timing and levels before initial chip select */
		tmp = spi_readl(SLV_SET2_REG(0)) & ~(SPI_SLV2_SPO | SPI_SLV2_SPH);
		/* Clock high between transfers */
#ifdef JDS
		tmp |= SPI_SLV2_SPO;
		/* Data captured on 2nd clock edge */
		tmp |= SPI_SLV2_SPH;
#endif
		spi_writel(SLV_SET2_REG(0), tmp);

		/* Delay if requested at end of transfer */
		if (message->state == STATE_RUNNING) {
			previous =
			    list_entry(transfer->transfer_list.prev,
				       struct spi_transfer, transfer_list);
			if (previous->delay_usecs)
				udelay(previous->delay_usecs);
			if (previous->cs_change)
				lpc31xx_cs_control(espi, true);
		} else {
			/* STATE_START */
			message->state = STATE_RUNNING;
			if (!espi->next_msg_cs_active)
				lpc31xx_cs_control(espi, true);
		}

		/* Configuration Changing Per Transfer */
		if (set_up_next_transfer(espi, transfer)) {
			/* Error path */
			message->state = STATE_ERROR;
			break;
		}
		/* Flush FIFOs and enable SSI */
		flush(espi);
		/* Make sure FIFO is flushed, clear pending interrupts, DMA
		   initially disabled, and then enable SPI interface */
		spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_ENABLE));

		dev_dbg(&espi->pdev->dev, "polling transfer ongoing ...\n");

		timeout = jiffies + msecs_to_jiffies(SPI_POLLING_TIMEOUT);
		while (espi->tx < espi->tx_end || espi->rx < espi->rx_end) {
			time = jiffies;
			readwriter(espi);
			if (time_after(time, timeout)) {
				dev_warn(&espi->pdev->dev,
				"%s: timeout!\n", __func__);
				message->state = STATE_ERROR;
				goto out;
			}
			cpu_relax();
		}

		/* Update total byte transferred */
		message->actual_length += espi->cur_transfer->len;
		if (espi->cur_transfer->cs_change)
			lpc31xx_cs_control(espi, false);

		/* Move to next transfer */
		message->state = next_transfer(espi);
	}
out:
	/* Handle end of message */
	if (message->state == STATE_DONE)
		message->status = 0;
	else
		message->status = -EIO;

	giveback(espi);
	return;
}

static void do_interrupt_transfer(struct lpc31xx_spi *espi)
{
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct lpc31xx_spi_chip *chip;
	unsigned long time, timeout;
	uint32_t tmp;

	dev_dbg(&espi->pdev->dev, "do_interrupt_transfer\n");

	chip = espi->cur_chip;
	message = espi->cur_msg;

	while (message->state != STATE_DONE) {
		/* Handle for abort */
		if (message->state == STATE_ERROR)
			break;
		transfer = espi->cur_transfer;

		/* Setup timing and levels before initial chip select */
		tmp = spi_readl(SLV_SET2_REG(0)) & ~(SPI_SLV2_SPO | SPI_SLV2_SPH);
		/* Clock high between transfers */
#ifdef JDS
		tmp |= SPI_SLV2_SPO;
		/* Data captured on 2nd clock edge */
		tmp |= SPI_SLV2_SPH;
#endif
		spi_writel(SLV_SET2_REG(0), tmp);

		/* Delay if requested at end of transfer */
		if (message->state == STATE_RUNNING) {
			previous =
			    list_entry(transfer->transfer_list.prev,
				       struct spi_transfer, transfer_list);
			if (previous->delay_usecs)
				udelay(previous->delay_usecs);
			if (previous->cs_change)
				lpc31xx_cs_control(espi, true);
		} else {
			/* STATE_START */
			message->state = STATE_RUNNING;
			if (!espi->next_msg_cs_active)
				lpc31xx_cs_control(espi, true);
		}

		/* Configuration Changing Per Transfer */
		if (set_up_next_transfer(espi, transfer)) {
			/* Error path */
			message->state = STATE_ERROR;
			break;
		}
		/* Flush FIFOs and enable SSI */
		flush(espi);
		/* Make sure FIFO is flushed, clear pending interrupts, DMA
		   initially disabled, and then enable SPI interface */
		spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_ENABLE));

		dev_dbg(&espi->pdev->dev, "polling transfer ongoing ...\n");

		timeout = jiffies + msecs_to_jiffies(SPI_POLLING_TIMEOUT);
		while (espi->tx < espi->tx_end || espi->rx < espi->rx_end) {
			time = jiffies;
			readwriter(espi);
			if (time_after(time, timeout)) {
				dev_warn(&espi->pdev->dev,
				"%s: timeout!\n", __func__);
				message->state = STATE_ERROR;
				goto out;
			}
			cpu_relax();
		}

		/* Update total byte transferred */
		message->actual_length += espi->cur_transfer->len;
		if (espi->cur_transfer->cs_change)
			lpc31xx_cs_control(espi, false);

		/* Move to next transfer */
		message->state = next_transfer(espi);
	}
out:
	/* Handle end of message */
	if (message->state == STATE_DONE)
		message->status = 0;
	else
		message->status = -EIO;

	giveback(espi);
	return;
}

static int lpc31xx_transfer_one_message(struct spi_master *master,
				      struct spi_message *msg)
{
	struct lpc31xx_spi *espi = spi_master_get_devdata(master);

	dev_dbg(&espi->pdev->dev, "lpc31xx_transfer_one_message\n");

	/* Initial message state */
	espi->cur_msg = msg;
	msg->state = STATE_START;

	espi->cur_transfer = list_entry(msg->transfers.next,
					 struct spi_transfer, transfer_list);

	/* Setup the SPI using the per chip configuration */
	espi->cur_chip = spi_get_ctldata(msg->spi);

	restore_state(espi);
	flush(espi);

	switch (espi->cur_chip->xfer_type) {
	case POLLING_TRANSFER:
		do_polling_transfer(espi);
		break;
	case INTERRUPT_TRANSFER:
		do_interrupt_transfer(espi);
		break;
	case DMA_TRANSFER:
		do_interrupt_dma_transfer(espi);
		break;
	}
	return 0;
}

static int lpc31xx_prepare_transfer_hardware(struct spi_master *master)
{
	struct lpc31xx_spi *espi = spi_master_get_devdata(master);

	dev_dbg(&espi->pdev->dev, "lpc31xx_prepare_transfer_hardware\n");
	/*
	 * Just make sure we have all we need to run the transfer by syncing
	 * with the runtime PM framework.
	 */
	pm_runtime_get_sync(&espi->pdev->dev);
	return 0;
}

static int lpc31xx_unprepare_transfer_hardware(struct spi_master *master)
{
	struct lpc31xx_spi *espi = spi_master_get_devdata(master);

	dev_dbg(&espi->pdev->dev, "lpc31xx_unprepare_transfer_hardware\n");
#ifdef JDS
	/* nothing more to do - disable spi/spi and power off */
	writew((readw(SSP_CR1(espi->virtbase)) &
		(~SSP_CR1_MASK_SSE)), SSP_CR1(espi->virtbase));

	if (espi->master_info->autosuspend_delay > 0) {
		pm_runtime_mark_last_busy(&espi->pdev->dev);
		pm_runtime_put_autosuspend(&espi->pdev->dev);
	} else {
		pm_runtime_put(&espi->pdev->dev);
	}
#endif
	return 0;
}

static int verify_controller_parameters(struct lpc31xx_spi *espi,
				struct lpc31xx_spi_config_chip const *chip_info)
{
#ifdef JDS
	if ((chip_info->iface < SSP_INTERFACE_MOTOROLA_SPI)
	    || (chip_info->iface > SSP_INTERFACE_UNIDIRECTIONAL)) {
		dev_err(&espi->pdev->dev,
			"interface is configured incorrectly\n");
		return -EINVAL;
	}
	if ((chip_info->iface == SSP_INTERFACE_UNIDIRECTIONAL) &&
	    (!espi->vendor->unidir)) {
		dev_err(&espi->pdev->dev,
			"unidirectional mode not supported in this "
			"hardware version\n");
		return -EINVAL;
	}
	if ((chip_info->hierarchy != SSP_MASTER)
	    && (chip_info->hierarchy != SSP_SLAVE)) {
		dev_err(&espi->pdev->dev,
			"hierarchy is configured incorrectly\n");
		return -EINVAL;
	}
	if ((chip_info->com_mode != INTERRUPT_TRANSFER)
	    && (chip_info->com_mode != DMA_TRANSFER)
	    && (chip_info->com_mode != POLLING_TRANSFER)) {
		dev_err(&espi->pdev->dev,
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
		if (espi->vendor->fifodepth < 16) {
			dev_err(&espi->pdev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	case SSP_RX_32_OR_MORE_ELEM:
		if (espi->vendor->fifodepth < 32) {
			dev_err(&espi->pdev->dev,
			"RX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(&espi->pdev->dev,
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
		if (espi->vendor->fifodepth < 16) {
			dev_err(&espi->pdev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	case SSP_TX_32_OR_MORE_EMPTY_LOC:
		if (espi->vendor->fifodepth < 32) {
			dev_err(&espi->pdev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
			return -EINVAL;
		}
		break;
	default:
		dev_err(&espi->pdev->dev,
			"TX FIFO Trigger Level is configured incorrectly\n");
		return -EINVAL;
		break;
	}
	if (chip_info->iface == SSP_INTERFACE_NATIONAL_MICROWIRE) {
		if ((chip_info->ctrl_len < SSP_BITS_4)
		    || (chip_info->ctrl_len > SSP_BITS_32)) {
			dev_err(&espi->pdev->dev,
				"CTRL LEN is configured incorrectly\n");
			return -EINVAL;
		}
		if ((chip_info->wait_state != SSP_MWIRE_WAIT_ZERO)
		    && (chip_info->wait_state != SSP_MWIRE_WAIT_ONE)) {
			dev_err(&espi->pdev->dev,
				"Wait State is configured incorrectly\n");
			return -EINVAL;
		}
		/* Half duplex is only available in the ST Micro version */
		if (espi->vendor->extended_cr) {
			if ((chip_info->duplex !=
			     SSP_MICROWIRE_CHANNEL_FULL_DUPLEX)
			    && (chip_info->duplex !=
				SSP_MICROWIRE_CHANNEL_HALF_DUPLEX)) {
				dev_err(&espi->pdev->dev,
					"Microwire duplex mode is configured incorrectly\n");
				return -EINVAL;
			}
		} else {
			if (chip_info->duplex != SSP_MICROWIRE_CHANNEL_FULL_DUPLEX)
				dev_err(&espi->pdev->dev,
					"Microwire half duplex mode requested,"
					" but this is only available in the"
					" ST version of LPC31xx\n");
			return -EINVAL;
		}
	}
#endif
	return 0;
}

static inline uint32_t spi_rate(uint32_t rate, uint16_t cpsdvsr, uint16_t scr)
{
	return rate / (cpsdvsr * (1 + scr));
}

static int calculate_effective_freq(struct lpc31xx_spi *espi, int freq, struct
				    spi_clock_params * clk_freq)
{
	unsigned long spi_clk_rate = clk_get_rate(espi->clk);
	int cpsr, scr, max_rate, min_rate;

	/*
	 * Make sure that max value is between values supported by the
	 * controller. Note that minimum value is already checked in
	 * lpc31xx_spi_transfer().
	 */
	max_rate = spi_clk_rate / 2;
	min_rate = spi_clk_rate / (254 * 256);
	freq = clamp(freq, min_rate, max_rate);

	/*
	 * Calculate divisors so that we can get speed according the
	 * following formula:
	 *	rate = spi_clock_rate / (cpsr * (1 + scr))
	 *
	 * cpsr must be even number and starts from 2, scr can be any number
	 * between 0 and 255.
	 */
	for (cpsr = 2; cpsr <= 254; cpsr += 2) {
		for (scr = 0; scr <= 255; scr++) {
			if ((spi_clk_rate / (cpsr * (scr + 1))) <= freq) {
				clk_freq->scr = (uint8_t)scr;
				clk_freq->cpsdvsr = (uint8_t)cpsr;
				dev_dbg(&espi->pdev->dev, "calculate_effective_freq %d actual %d\n", freq,
						spi_rate(spi_clk_rate, clk_freq->cpsdvsr, clk_freq->scr));
				return 0;
			}
		}
	}
	return -EINVAL;
}

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
	struct lpc31xx_spi_config_chip const *chip_info;
	struct lpc31xx_spi_chip *chip;
	struct spi_clock_params clk_freq = { .cpsdvsr = 0, .scr = 0};
	int status = 0;
	struct lpc31xx_spi *espi = spi_master_get_devdata(spi->master);
	unsigned int bits = spi->bits_per_word;
	uint32_t tmp;

	dev_dbg(&spi->dev, "lpc31xx_setup\n");

	if (!spi->max_speed_hz)
		return -EINVAL;

	/* Get controller_state if one is supplied */
	chip = spi_get_ctldata(spi);

	if (chip == NULL) {
		chip = kzalloc(sizeof(struct lpc31xx_spi_chip), GFP_KERNEL);
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
#ifdef JDS
		chip_info = &lpc31xx_default_chip_info;
#endif
		/* spi_board_info.controller_data not is supplied */
		dev_dbg(&spi->dev,
			"using default controller_data settings\n");
	} else
		dev_dbg(&spi->dev,
			"using user supplied controller_data settings\n");

	status = calculate_effective_freq(espi, spi->max_speed_hz, &clk_freq);
	if (status < 0)
		goto err_config_params;
#ifdef JDS

	status = verify_controller_parameters(espi, chip_info);
	if (status) {
		dev_err(&spi->dev, "controller data is incorrect");
		goto err_config_params;
	}

	espi->rx_lev_trig = chip_info->rx_lev_trig;
	espi->tx_lev_trig = chip_info->tx_lev_trig;

	/* Now set controller state based on controller data */
	chip->xfer_type = chip_info->com_mode;
#endif
	chip->xfer_type = INTERRUPT_TRANSFER;

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
		/* LPC31xx doesn't support more than 16-bits */
		status = -ENOTSUPP;
		goto err_config_params;
	}

#ifdef JDS
	/* Now Initialize all register settings required for this chip */
	chip->cr0 = 0;
	chip->cr1 = 0;
	chip->dmacr = 0;
	chip->cpsr = 0;
	if ((chip_info->com_mode == DMA_TRANSFER)
	    && ((espi->master_info)->enable_dma)) {
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
	if (espi->vendor->extended_cr) {
		uint32_t etx;

		if (espi->vendor->pl023) {
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
	if (espi->vendor->loopback) {
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
	chip->chip_select = spi->chip_select;

	/* Save controller_state */
	spi_set_ctldata(spi, chip);
	return status;
 err_config_params:
	spi_set_ctldata(spi, NULL);
	kfree(chip);
	return status;
}

/*
 * Setup the initial state of the SPI interface
 */
static void lpc31xx_spi_prep(struct lpc31xx_spi *espi)
{
	u32 tmp;

	/* Reset SPI block */
	spi_writel(CONFIG_REG, SPI_CFG_SW_RESET);

	/* Clear FIFOs */
	lpc31xx_fifo_flush(espi);

	/* Clear latched interrupts */
	lpc31xx_int_dis(espi, SPI_ALL_INTS);
	lpc31xx_int_clr(espi, SPI_ALL_INTS);

	/* Setup master mode, normal transmit mode, and interslave delay */
	spi_writel(CONFIG_REG, SPI_CFG_INTER_DLY(1));

	/* Make sure all 3 chip selects are initially disabled */
	spi_writel(SLV_ENAB_REG, 0);
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_UPDATE_EN));

	/* FIFO trip points at 50% */
	spi_writel(INT_TRSH_REG, (SPI_INT_TSHLD_TX(0x20) | SPI_INT_TSHLD_RX(0x20)));

	/* Only chip select 0 is used in this driver. However, the timings for this
	   chip select effect transfer speed and need to be adjusted for each GPO
	   based chip select. Use a default value to start with for now. */
	/* Inter-transfer delay is 0 (not used) */
	tmp = spi_readl(SLV_SET1_REG(0));
	tmp &= ~SPI_SLV1_INTER_TX_DLY(0xFF);
	spi_writel(SLV_SET1_REG(0), (tmp | SPI_SLV1_INTER_TX_DLY(0)));

	/* Configure enabled chip select slave setting 2 */
	tmp = SPI_SLV2_PPCS_DLY(0) | SPI_SLV2_CS_HIGH | SPI_SLV2_SPO;
	spi_writel(SLV_SET2_REG(0), tmp);

	/* Use a default of 8 data bits and a 100K clock for now */
	lpc31xx_set_cs_data_bits(espi, 0, 8);
	lpc31xx_set_cs_clock(espi, 0, 100000);

	/* We'll always use CS0 for this driver. Since the chip select is generated
	   by a GPO, it doesn't matter which one we use */
	spi_writel(SLV_ENAB_REG, SPI_SLV_EN(0));
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_UPDATE_EN));

	/* Controller stays disabled until a transfer occurs */
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
	struct lpc31xx_spi_chip *chip = spi_get_ctldata(spi);

	spi_set_ctldata(spi, NULL);
	kfree(chip);
}

static int __devinit
lpc31xx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_master *master;
	struct lpc31xx_spi *espi = NULL;	/*Data for this driver */
	int i, ngpios, status = 0;
	struct resource *res;

	/* Allocate master with space for data */
	master = spi_alloc_master(dev, sizeof(struct lpc31xx_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "probe - cannot alloc SPI master\n");
		status = -ENOMEM;
		goto err_no_master;
	}

	espi = spi_master_get_devdata(master);
	espi->master = master;
	espi->pdev = pdev;

	ngpios = of_gpio_count(pdev->dev.of_node);

	/*
	 * Bus Number Which has been Assigned to this SSP controller
	 * on this board
	 */
	master->bus_num = 0;
	master->num_chipselect = max(ngpios, 1);  /* always one even if no gpios */
	master->cleanup = lpc31xx_cleanup;
	master->setup = lpc31xx_setup;
	master->prepare_transfer_hardware = lpc31xx_prepare_transfer_hardware;
	master->transfer_one_message = lpc31xx_transfer_one_message;
	master->unprepare_transfer_hardware = lpc31xx_unprepare_transfer_hardware;
	master->rt = false;
	master->dev.of_node = of_node_get(pdev->dev.of_node);

	/*
	 * Supports mode 0-3, loopback, and active low CS. Transfers are
	 * always MS bit first on the original lpc31xx.
	 */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LOOP;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "unable to get iomem resource\n");
		status = -ENODEV;
		goto err_no_ioregion;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "unable to request iomem resources\n");
		status = -EBUSY;
		goto err_no_ioregion;
	}

	espi->phybase = res->start;
	espi->virtbase = ioremap(res->start, resource_size(res));
	if (espi->virtbase == NULL) {
		status = -ENOMEM;
		goto err_no_ioremap;
	}

	lpc31xx_spi_clks_enable(espi);

	/* Initialize transfer pump */
	tasklet_init(&espi->pump_transfers, pump_transfers,
		     (unsigned long)espi);

	/* Disable SPI */
	lpc31xx_spi_prep(espi);

	espi->irq = platform_get_irq(pdev, 4);
	if (espi->irq < 0) {
		status = espi->irq;
		dev_err(&pdev->dev, "failed to get irq resources\n");
		goto err_no_irq;
	}
	status = request_irq(espi->irq, lpc31xx_interrupt_handler, 0, pdev->name, espi);
	if (status < 0) {
		dev_err(&pdev->dev, "probe - cannot get IRQ (%d)\n", status);
		goto err_no_irq;
	}

	/* Get DMA channels */
#ifdef JDS
	if (platform_info->enable_dma) {
		status = lpc31xx_dma_probe(espi);
		if (status != 0)
			platform_info->enable_dma = 0;
	}
#endif
	for (i = 0; i < ngpios; i++) {
		int gpio;
		enum of_gpio_flags flags;

		gpio = of_get_gpio_flags(pdev->dev.of_node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(&pdev->dev, "invalid gpio #%d: %d\n", i, gpio);
			status = gpio;
			goto err_no_irq;
		}
		status = gpio_request(gpio, dev_name(&pdev->dev));
		if (status) {
			dev_err(&pdev->dev, "can't request gpio #%d: %d\n", i, status);
			goto err_no_irq;
		}
		espi->gpio[i] = gpio;
		espi->alow[i] = flags & OF_GPIO_ACTIVE_LOW;

		status = gpio_direction_output(gpio, espi->alow[i]);
		if (status) {
			dev_err(&pdev->dev, "can't set output direction for gpio #%d: %d\n", i, status);
			goto err_no_irq;
		}
		gpio_set_value(gpio, 1);
	}

	/* Register with the SPI framework */
	platform_set_drvdata(pdev, espi);
	status = spi_register_master(master);
	if (status != 0) {
		dev_err(&pdev->dev,
			"probe - problem registering spi master\n");
		goto err_spi_register;
	}
	dev_dbg(dev, "probe succeeded\n");

#ifdef JDS
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
#endif
	dev_info(&pdev->dev, "NXP LPC31xx SPI driver\n");

	return 0;

 err_spi_register:
#ifdef JDS
	if (platform_info->enable_dma)
		lpc31xx_dma_remove(espi);
#endif

	free_irq(espi->irq, espi);
err_no_irq:
	lpc31xx_spi_clks_disable();
	iounmap(espi->virtbase);
err_no_ioremap:
	release_mem_region(res->start, resource_size(res));
err_no_ioregion:
	spi_master_put(master);
err_no_master:
	return status;
}

static int __devexit
lpc31xx_remove(struct platform_device *pdev)
{
	struct lpc31xx_spi *espi = platform_get_drvdata(pdev);
	struct resource *res;

	if (!espi)
		return 0;

	/*
	 * undo pm_runtime_put() in probe.  I assume that we're not
	 * accessing the device here.
	 */
	pm_runtime_get_noresume(&pdev->dev);

	lpc31xx_spi_prep(espi);
#ifdef JDS
	if (espi->master_info->enable_dma)
		lpc31xx_dma_remove(espi);
#endif

	free_irq(espi->irq, espi);
	clk_disable(espi->clk);
	clk_unprepare(espi->clk);
	clk_put(espi->clk);
	iounmap(espi->virtbase);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	tasklet_disable(&espi->pump_transfers);
	spi_unregister_master(espi->master);
	spi_master_put(espi->master);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int lpc31xx_suspend(struct device *dev)
{
	struct lpc31xx_spi *espi = dev_get_drvdata(dev);
	int ret;

	ret = spi_master_suspend(espi->master);
	if (ret) {
		dev_warn(dev, "cannot suspend master\n");
		return ret;
	}

	dev_dbg(dev, "suspended\n");
	return 0;
}

static int lpc31xx_resume(struct device *dev)
{
	struct lpc31xx_spi *espi = dev_get_drvdata(dev);
	int ret;

	/* Start the queue running */
	ret = spi_master_resume(espi->master);
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
	struct lpc31xx_spi *espi = dev_get_drvdata(dev);

	clk_disable(espi->clk);

	return 0;
}

static int lpc31xx_runtime_resume(struct device *dev)
{
	struct lpc31xx_spi *espi = dev_get_drvdata(dev);

	clk_enable(espi->clk);

	return 0;
}
#endif

static const struct dev_pm_ops lpc31xx_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lpc31xx_suspend, lpc31xx_resume)
	SET_RUNTIME_PM_OPS(lpc31xx_runtime_suspend, lpc31xx_runtime_resume, NULL)
};

static const struct of_device_id lpc31xx_spi_of_match[] = {
	{ .compatible = "nxp,lpc31xx-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc31xx_spi_of_match);

static struct platform_driver lpc31xx_spi_driver = {
	.probe		= lpc31xx_probe,
	.remove		= __devexit_p(lpc31xx_remove),
	.driver = {
		.name	= "spi-lpc31xx",
		.pm	= &lpc31xx_dev_pm_ops,
		.owner	= THIS_MODULE,
		.of_match_table = lpc31xx_spi_of_match,
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
