/*
 * Driver for NXP LPC31xx SPI controller.
 *
 * Copyright (C) 2012 Jon Smirl <jonsmirl@gmail.com>
 *
 * Derived off from the lpc31xx SPI driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/scatterlist.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/of_gpio.h>

#include <mach/dma.h>

/***********************************************************************
 * SPI register definitions
 **********************************************************************/
#define SPI_CONFIG_REG    0x00
#define SPI_SLV_ENAB_REG  0x04
#define SPI_TXF_FLUSH_REG 0x08
#define SPI_FIFO_DATA_REG 0x0C
#define SPI_NHP_POP_REG   0x10
#define SPI_NHP_MODE_REG  0x14
#define SPI_DMA_SET_REG   0x18
#define SPI_STS_REG       0x1C
#define SPI_HWINFO_REG    0x20
#define SPI_SLV_SET1_REG(slv) (0x24 + (8 * slv))
#define SPI_SLV_SET2_REG(slv) (0x28 + (8 * slv))
#define SPI_INT_TRSH_REG  0xFD4
#define SPI_INT_CLRE_REG  0xFD8
#define SPI_INT_SETE_REG  0xFDC
#define SPI_INT_STS_REG   0xFE0
#define SPI_INT_ENAB_REG  0xFE4
#define SPI_INT_CLRS_REG  0xFE8
#define SPI_INT_SETS_REG  0xFEC
#define SPI_MOD_ID_REG    0xFFC

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

/* SPI intterrupt registers definitions ( SPI_INT_xxx) */
#define SPI_SMS_INT               _BIT(4)
#define SPI_TX_INT                _BIT(3)
#define SPI_RX_INT                _BIT(2)
#define SPI_TO_INT                _BIT(1)
#define SPI_OVR_INT               _BIT(0)
#define SPI_ALL_INTS              (SPI_SMS_INT | SPI_TX_INT | SPI_RX_INT | SPI_TO_INT | SPI_OVR_INT)

/* timeout in milliseconds */
#define SPI_TIMEOUT		5


/**
 * struct lpc31xx_spi - LPC31xx SPI controller structure
 * @lock: spinlock that protects concurrent accesses to fields @running,
 *        @current_msg and @msg_queue
 * @pdev: pointer to platform device
 * @clk: clock for the controller
 * @regs_base: pointer to ioremap()'d registers
 * @sspdr_phys: physical address of the SSPDR register
 * @irq: IRQ number used by the driver
 * @min_rate: minimum clock rate (in Hz) supported by the controller
 * @max_rate: maximum clock rate (in Hz) supported by the controller
 * @running: is the queue running
 * @wq: workqueue used by the driver
 * @msg_work: work that is queued for the driver
 * @wait: wait here until given transfer is completed
 * @msg_queue: queue for the messages
 * @current_msg: message that is currently processed (or %NULL if none)
 * @tx: current byte in transfer to transmit
 * @rx: current byte in transfer to receive
 * @fifo_level: how full is FIFO (%0..%SPI_FIFO_SIZE - %1). Receiving one
 *              frame decreases this level and sending one frame increases it.
 * @dma_rx: RX DMA channel
 * @dma_tx: TX DMA channel
 * @dma_rx_data: RX parameters passed to the DMA engine
 * @dma_tx_data: TX parameters passed to the DMA engine
 * @rx_sgt: sg table for RX transfers
 * @tx_sgt: sg table for TX transfers
 * @zeropage: dummy page used as RX buffer when only TX buffer is passed in by
 *            the client
 *
 * This structure holds LPC31xx SPI controller specific information. When
 * @running is %true, driver accepts transfer requests from protocol drivers.
 * @current_msg is used to hold pointer to the message that is currently
 * processed. If @current_msg is %NULL, it means that no processing is going
 * on.
 *
 * Most of the fields are only written once and they can be accessed without
 * taking the @lock. Fields that are accessed concurrently are: @current_msg,
 * @running, and @msg_queue.
 */
struct lpc31xx_spi {
	spinlock_t			lock;
	const struct platform_device	*pdev;
	struct clk			*clk;
	void __iomem			*regs_base;
	unsigned long			sspdr_phys;
	int				irq;
	unsigned long			min_rate;
	unsigned long			max_rate;
	bool				running;
	struct workqueue_struct		*wq;
	struct work_struct		msg_work;
	struct completion		wait;
	struct list_head		msg_queue;
	struct spi_message		*current_msg;
	size_t				tx;
	size_t				rx;
	size_t				fifo_level;
	struct dma_chan			*dma_rx;
	struct dma_chan			*dma_tx;
	struct lpc31xx_dma_data		dma_rx_data;
	struct lpc31xx_dma_data		dma_tx_data;
	struct sg_table			rx_sgt;
	struct sg_table			tx_sgt;
	void				*zeropage;

	uint32_t current_speed_hz [3]; /* Per CS */
	uint8_t current_bits_wd [3]; /* Per CS */
};

/**
 * struct lpc31xx_spi_chip - SPI device hardware settings
 * @spi: back pointer to the SPI device
 * @rate: max rate in hz this chip supports
 * @div_cpsr: cpsr (pre-scaler) divider
 * @div_scr: scr divider
 * @dss: bits per word (4 - 16 bits)
 * @ops: private chip operations
 *
 * This structure is used to store hardware register specific settings for each
 * SPI device. Settings are written to hardware by function
 * lpc31xx_spi_chip_setup().
 */
struct lpc31xx_spi_chip {
	const struct spi_device		*spi;
	unsigned long			rate;
	uint8_t				div_cpsr;
	uint8_t				div_scr;
	uint8_t				dss;
	int 				gpio;
	uint32_t			alow;
};

static inline void
lpc31xx_spi_write(const struct lpc31xx_spi *espi, uint32_t reg, uint32_t value)
{
	printk("JDS - lpc31xx_spi_write %p value %x\n", espi->regs_base + reg, value);
	__raw_writel(value, espi->regs_base + reg);
}

static inline uint32_t
lpc31xx_spi_read(const struct lpc31xx_spi *espi, uint32_t reg)
{
	uint32_t value;
	value = __raw_readl(espi->regs_base + reg);
	printk("JDS - lpc31xx_spi_read %p value %x\n", espi->regs_base + reg, value);
	return value;
}

/*
 * Clear a latched SPI interrupt
 */
static inline void lpc31xx_int_clr(const struct lpc31xx_spi *espi, uint32_t ints)
{
	lpc31xx_spi_write(espi, SPI_INT_CLRS_REG, ints);
}

/*
 * Disable a SPI interrupt
 */
static inline void lpc31xx_int_dis(const struct lpc31xx_spi *espi, uint32_t ints)
{
	lpc31xx_spi_write(espi, SPI_INT_CLRE_REG, ints);
}

/*
 * Set data width for the SPI chip select
 */
static void lpc31xx_set_cs_data_bits(struct lpc31xx_spi *espi, uint8_t cs, uint8_t data_width)
{
	printk("JDS - lpc31xx_set_cs_data_bits, width %x\n", data_width);
	if (espi->current_bits_wd[cs] != data_width)
	{
		uint32_t tmp = lpc31xx_spi_read(espi, SPI_SLV_SET2_REG(0));
		tmp &= ~SPI_SLV2_WD_SZ(0x1F);
		tmp |= SPI_SLV2_WD_SZ((uint32_t) (data_width - 1));
		lpc31xx_spi_write(espi, SPI_SLV_SET2_REG(0), tmp);

		espi->current_bits_wd[cs] = data_width;
	}
}

/*
 * Set clock rate and delays for the SPI chip select
 */
static void lpc31xx_set_cs_clock(struct lpc31xx_spi *espi, uint8_t cs, uint32_t clockrate)
{
	uint32_t reg, div, ps, div1;

	printk("JDS - lpc31xx_set_cs_clock\n");
	if (clockrate != espi->current_speed_hz[cs])
	{
		printk("setting clock - lpc31xx_set_cs_clock\n");
		reg = lpc31xx_spi_read(espi, SPI_SLV_SET1_REG(0));
		reg &= ~0xFFFF;

		div = ((espi->max_rate * 2)  + clockrate / 2) / clockrate;
		if (div > SPI_MAX_DIVIDER)
			div = SPI_MAX_DIVIDER;
		if (div < SPI_MIN_DIVIDER)
			div = SPI_MIN_DIVIDER;

		ps = (((div - 1) / 512) + 1) * 2;
		div1 = (((div + ps / 2) / ps) - 1);

		lpc31xx_spi_write(espi, SPI_SLV_SET1_REG(0),
			(reg | SPI_SLV1_CLK_PS(ps) | SPI_SLV1_CLK_DIV1(div1)));

		espi->current_speed_hz[cs] = clockrate;
	}
}

/*
 * Enable or disable the SPI clocks
 */
static void lpc31xx_spi_clks_enable(void)
{
	struct clk *clk;
	int ret;

	printk("----clocks on-----------\n");
	clk = clk_get(NULL, "spi_pclk");
	ret = clk_enable(clk);
	clk_put(clk);
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

	printk("----clocks off-----------\n");
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

static int lpc31xx_spi_enable(const struct lpc31xx_spi *espi)
{
	printk("JDS - lpc31xx_spi_enable\n");
	lpc31xx_spi_clks_enable();

	return 0;
}

static void lpc31xx_spi_disable(const struct lpc31xx_spi *espi)
{
	printk("JDS - lpc31xx_spi_disable\n");
	lpc31xx_spi_clks_disable();
}

static void lpc31xx_spi_enable_interrupts(const struct lpc31xx_spi *espi)
{
	printk("JDS - lpc31xx_spi_enable_interrupts\n");
	lpc31xx_spi_write(espi, SPI_INT_SETE_REG, (SPI_RX_INT | SPI_TO_INT | SPI_OVR_INT));
	enable_irq(espi->irq);


	printk("int status %x\n", lpc31xx_spi_read(espi, SPI_INT_STS_REG));
}

static void lpc31xx_spi_disable_interrupts(const struct lpc31xx_spi *espi)
{
	printk("JDS - lpc31xx_spi_disable_interrupts\n");
	disable_irq(espi->irq);
}

/**
 * lpc31xx_spi_calc_divisors() - calculates SPI clock divisors
 * @espi: lpc31xx SPI controller struct
 * @chip: divisors are calculated for this chip
 * @rate: desired SPI output clock rate
 *
 * Function calculates cpsr (clock pre-scaler) and scr divisors based on
 * given @rate and places them to @chip->div_cpsr and @chip->div_scr. If,
 * for some reason, divisors cannot be calculated nothing is stored and
 * %-EINVAL is returned.
 */
static int lpc31xx_spi_calc_divisors(const struct lpc31xx_spi *espi,
				    struct lpc31xx_spi_chip *chip,
				    unsigned long rate)
{
	unsigned long spi_clk_rate = clk_get_rate(espi->clk);
	int cpsr, scr;

	printk("JDS - lpc31xx_spi_calc_divisors min %ld max %ld req %ld\n", espi->min_rate, espi->max_rate, rate);
	/*
	 * Make sure that max value is between values supported by the
	 * controller. Note that minimum value is already checked in
	 * lpc31xx_spi_transfer().
	 */
	rate = clamp(rate, espi->min_rate, espi->max_rate);

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
			if ((spi_clk_rate / (cpsr * (scr + 1))) <= rate) {
				chip->div_scr = (uint8_t)scr;
				chip->div_cpsr = (uint8_t)cpsr;
				return 0;
			}
		}
	}
	return -EINVAL;
}

static void lpc31xx_spi_cs_control(struct spi_device *spi, bool control)
{
	struct lpc31xx_spi_chip *chip = spi_get_ctldata(spi);
	int value = (spi->mode & SPI_CS_HIGH) ? control : !control;

	if (!gpio_is_valid(chip->gpio))
		return;
	printk("JDS - lpc31xx_spi_cs_control %d value %d\n", chip->gpio, value);
	gpio_set_value(chip->gpio, value);
}

/**
 * lpc31xx_spi_setup() - setup an SPI device
 * @spi: SPI device to setup
 *
 * This function sets up SPI device mode, speed etc. Can be called multiple
 * times for a single device. Returns 0 in case of success, negative error in
 * case of failure. When this function returns success, the device is
 * deselected.
 */
static int lpc31xx_spi_setup(struct spi_device *spi)
{
	struct lpc31xx_spi *espi = spi_master_get_devdata(spi->master);
	struct lpc31xx_spi_chip *chip;
	enum of_gpio_flags flags;
	int ret, gpio;

	printk("JDS - lpc31xx_spi_setup\n");
	if (spi->bits_per_word < 4 || spi->bits_per_word > 16) {
		dev_err(&espi->pdev->dev, "invalid bits per word %d\n",
			spi->bits_per_word);
		return -EINVAL;
	}

	chip = spi_get_ctldata(spi);
	if (!chip) {
		dev_dbg(&espi->pdev->dev, "initial setup for %s\n",
			spi->modalias);

		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;

		chip->spi = spi;
		spi_set_ctldata(spi, chip);
	}
	if (spi->max_speed_hz != chip->rate) {
		int err;

		err = lpc31xx_spi_calc_divisors(espi, chip, spi->max_speed_hz);
		if (err != 0) {
error_out:
			spi_set_ctldata(spi, NULL);
			kfree(chip);
			return err;
		}
		chip->rate = spi->max_speed_hz;
	}

	chip->gpio = -EINVAL;
	gpio = of_get_named_gpio_flags(spi->dev.of_node, "gpio-cs", 0, &flags);
	if (!gpio_is_valid(gpio))
		return 0;

	ret = gpio_request(gpio, dev_name(&spi->dev));
	if (ret) {
		dev_err(&spi->dev, "can't request gpio-cs #%d: %d\n", gpio, ret);
		goto error_out;
	}
	chip->gpio = gpio;
	chip->alow = flags & OF_GPIO_ACTIVE_LOW;
	ret = gpio_direction_output(chip->gpio, chip->alow);
	if (ret) {
		dev_err(&spi->dev, "can't set output direction for gpio-cs #%d: %d\n", gpio, ret);
		goto error_out;
	}
	lpc31xx_spi_cs_control(spi, false);
	return 0;
}

/**
 * lpc31xx_spi_transfer() - queue message to be transferred
 * @spi: target SPI device
 * @msg: message to be transferred
 *
 * This function is called by SPI device drivers when they are going to transfer
 * a new message. It simply puts the message in the queue and schedules
 * workqueue to perform the actual transfer later on.
 *
 * Returns %0 on success and negative error in case of failure.
 */
static int lpc31xx_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct lpc31xx_spi *espi = spi_master_get_devdata(spi->master);
	struct spi_transfer *t;
	unsigned long flags;

	printk("JDS - lpc31xx_spi_transfer\n");
	if (!msg || !msg->complete)
		return -EINVAL;

	/* first validate each transfer */
	list_for_each_entry(t, &msg->transfers, transfer_list) {
		if (t->bits_per_word) {
			if (t->bits_per_word < 4 || t->bits_per_word > 16)
				return -EINVAL;
		}
		if (t->speed_hz && t->speed_hz < espi->min_rate)
				return -EINVAL;

		dev_dbg(&spi->dev,
			"  xfer %p: len %u tx %p/%08x rx %p/%08x DMAmapped=%d\n",
			t, t->len, t->tx_buf, t->tx_dma,
			t->rx_buf, t->rx_dma, msg->is_dma_mapped);
	}

	/*
	 * Now that we own the message, let's initialize it so that it is
	 * suitable for us. We use @msg->status to signal whether there was
	 * error in transfer and @msg->state is used to hold pointer to the
	 * current transfer (or %NULL if no active current transfer).
	 */
	msg->state = NULL;
	msg->status = 0;
	msg->actual_length = 0;

	spin_lock_irqsave(&espi->lock, flags);
	if (!espi->running) {
		spin_unlock_irqrestore(&espi->lock, flags);
		return -ESHUTDOWN;
	}
	list_add_tail(&msg->queue, &espi->msg_queue);
	queue_work(espi->wq, &espi->msg_work);
	spin_unlock_irqrestore(&espi->lock, flags);

	return 0;
}

/**
 * lpc31xx_spi_cleanup() - cleans up master controller specific state
 * @spi: SPI device to cleanup
 *
 * This function releases master controller specific state for given @spi
 * device.
 */
static void lpc31xx_spi_cleanup(struct spi_device *spi)
{
	struct lpc31xx_spi_chip *chip;
	printk("JDS - lpc31xx_spi_cleanup\n");
#if 0
	chip = spi_get_ctldata(spi);
	if (chip) {
		if (chip->ops && chip->ops->cleanup)
			chip->ops->cleanup(spi);
		spi_set_ctldata(spi, NULL);
		kfree(chip);
	}
#endif
}

/**
 * lpc31xx_spi_chip_setup() - configures hardware according to given @chip
 * @espi: lpc31xx SPI controller struct
 * @chip: chip specific settings
 *
 * This function sets up the actual hardware registers with settings given in
 * @chip. Note that no validation is done so make sure that callers validate
 * settings before calling this.
 */
static void lpc31xx_spi_chip_setup(const struct lpc31xx_spi *espi,
				  const struct lpc31xx_spi_chip *chip)
{
	printk("JDS - lpc31xx_spi_chip_setup\n");
#if 0
	u16 cr0;

	cr0 = chip->div_scr << SSPCR0_SCR_SHIFT;
	cr0 |= (chip->spi->mode & (SPI_CPHA|SPI_CPOL)) << SSPCR0_MODE_SHIFT;
	cr0 |= chip->dss;

	dev_dbg(&espi->pdev->dev, "setup: mode %d, cpsr %d, scr %d, dss %d\n",
		chip->spi->mode, chip->div_cpsr, chip->div_scr, chip->dss);
	dev_dbg(&espi->pdev->dev, "setup: cr0 %#x", cr0);

	lpc31xx_spi_write_uint8_t(espi, SSPCPSR, chip->div_cpsr);
	lpc31xx_spi_write_u16(espi, SSPCR0, cr0);
#endif
}

static inline int bits_per_word(const struct lpc31xx_spi *espi)
{
	struct spi_message *msg = espi->current_msg;
	struct spi_transfer *t = msg->state;

	return t->bits_per_word ? t->bits_per_word : msg->spi->bits_per_word;
}

static void lpc31xx_do_write(struct lpc31xx_spi *espi, struct spi_transfer *t)
{
	uint32_t data = 0x5555;

	if (bits_per_word(espi) > 8) {
		if (t->tx_buf)
			data = ((uint16_t *)t->tx_buf)[espi->tx];
		espi->tx += sizeof(uint16_t);
	} else {
		if (t->tx_buf)
			data = ((uint8_t *)t->tx_buf)[espi->tx];
		espi->tx += sizeof(uint8_t);
	}
	lpc31xx_spi_write(espi, SPI_FIFO_DATA_REG, data);
	printk("JDS - lpc31xx_do_write data %x\n", data);
}

static void lpc31xx_do_read(struct lpc31xx_spi *espi, struct spi_transfer *t)
{
	uint32_t data;


	data = lpc31xx_spi_read(espi, SPI_FIFO_DATA_REG);
	printk("JDS - lpc31xx_do_read data %x\n", data);
	/* The data can be tossed if there is no RX buffer */
	if (bits_per_word(espi) > 8) {
		if (t->rx_buf)
			((uint16_t *)t->rx_buf)[espi->rx] = data;
		espi->rx += sizeof(uint16_t);
	} else {
		if (t->rx_buf)
			((uint8_t *)t->rx_buf)[espi->rx] = data;
		espi->rx += sizeof(uint8_t);
	}
}

/**
 * lpc31xx_spi_read_write() - perform next RX/TX transfer
 * @espi: lpc31xx SPI controller struct
 *
 * This function transfers next bytes (or half-words) to/from RX/TX FIFOs. If
 * called several times, the whole transfer will be completed. Returns
 * %-EINPROGRESS when current transfer was not yet completed otherwise %0.
 *
 * When this function is finished, RX FIFO should be empty and TX FIFO should be
 * full.
 */
static int lpc31xx_spi_read_write(struct lpc31xx_spi *espi)
{
	struct spi_message *msg = espi->current_msg;
	struct spi_transfer *t = msg->state;

	printk("JDS - lpc31xx_spi_read_write, length %d\n", t->len);
	printk("JDS - lpc31xx_spi_read_write, rx %d tx %d\n", espi->rx, espi->tx);

	/* Set the FIFO trip level to the transfer size */
	lpc31xx_spi_write(espi, SPI_INT_TRSH_REG, (SPI_INT_TSHLD_TX(0) |
		SPI_INT_TSHLD_RX(t->len - 1)));
	lpc31xx_spi_write(espi, SPI_DMA_SET_REG, 0);

	/* read as long as RX FIFO has frames in it */
	while (!(lpc31xx_spi_read(espi, SPI_STS_REG) & SPI_ST_RX_EMPTY)) {
		lpc31xx_do_read(espi, t);
		espi->fifo_level--;
	}

	/* write as long as TX FIFO has room */
	while ((espi->fifo_level < SPI_FIFO_DEPTH) && (espi->tx < t->len)) {
		lpc31xx_do_write(espi, t);
		espi->fifo_level++;
	}

	printk("JDS - lpc31xx_spi_read_write, rx %d tx %d tlen %d\n", espi->rx, espi->tx, t->len);
	if (espi->rx == t->len)
		return 0;

	return -EINPROGRESS;
}

static void lpc31xx_spi_pio_transfer(struct lpc31xx_spi *espi)
{
	/*
	 * Now everything is set up for the current transfer. We prime the TX
	 * FIFO, enable interrupts, and wait for the transfer to complete.
	 */
	printk("JDS - lpc31xx_spi_pio_transfer\n");
	if (lpc31xx_spi_read_write(espi)) {
		lpc31xx_spi_enable_interrupts(espi);
		printk("JDS - lpc31xx_spi_pio_transfer - waiting\n");
		wait_for_completion(&espi->wait);
		printk("JDS - lpc31xx_spi_pio_transfer - waiting done\n");
	}
	printk("JDS - lpc31xx_spi_pio_transfer - exit\n");
}

/**
 * lpc31xx_spi_dma_prepare() - prepares a DMA transfer
 * @espi: lpc31xx SPI controller struct
 * @dir: DMA transfer direction
 *
 * Function configures the DMA, maps the buffer and prepares the DMA
 * descriptor. Returns a valid DMA descriptor in case of success and ERR_PTR
 * in case of failure.
 */
static struct dma_async_tx_descriptor *
lpc31xx_spi_dma_prepare(struct lpc31xx_spi *espi, enum dma_data_direction dir)
{
	struct spi_transfer *t = espi->current_msg->state;
	struct dma_async_tx_descriptor *txd;
	enum dma_slave_buswidth buswidth;
	struct dma_slave_config conf;
	enum dma_transfer_direction slave_dirn;
	struct scatterlist *sg;
	struct sg_table *sgt;
	struct dma_chan *chan;
	const void *buf, *pbuf;
	size_t len = t->len;
	int i, ret, nents;

	printk("JDS - lpc31xx_spi_dma_prepare\n");
	if (bits_per_word(espi) > 8)
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;

	memset(&conf, 0, sizeof(conf));
	conf.direction = dir;

	if (dir == DMA_FROM_DEVICE) {
		chan = espi->dma_rx;
		buf = t->rx_buf;
		sgt = &espi->rx_sgt;

		conf.src_addr = espi->sspdr_phys;
		conf.src_addr_width = buswidth;
		slave_dirn = DMA_DEV_TO_MEM;
	} else {
		chan = espi->dma_tx;
		buf = t->tx_buf;
		sgt = &espi->tx_sgt;

		conf.dst_addr = espi->sspdr_phys;
		conf.dst_addr_width = buswidth;
		slave_dirn = DMA_MEM_TO_DEV;
	}

	ret = dmaengine_slave_config(chan, &conf);
	if (ret)
		return ERR_PTR(ret);

	/*
	 * We need to split the transfer into PAGE_SIZE'd chunks. This is
	 * because we are using @espi->zeropage to provide a zero RX buffer
	 * for the TX transfers and we have only allocated one page for that.
	 *
	 * For performance reasons we allocate a new sg_table only when
	 * needed. Otherwise we will re-use the current one. Eventually the
	 * last sg_table is released in lpc31xx_spi_release_dma().
	 */

	nents = DIV_ROUND_UP(len, PAGE_SIZE);
	if (nents != sgt->nents) {
		sg_free_table(sgt);

		ret = sg_alloc_table(sgt, nents, GFP_KERNEL);
		if (ret)
			return ERR_PTR(ret);
	}

	pbuf = buf;
	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		size_t bytes = min_t(size_t, len, PAGE_SIZE);

		if (buf) {
			sg_set_page(sg, virt_to_page(pbuf), bytes,
				    offset_in_page(pbuf));
		} else {
			sg_set_page(sg, virt_to_page(espi->zeropage),
				    bytes, 0);
		}

		pbuf += bytes;
		len -= bytes;
	}

	if (WARN_ON(len)) {
		dev_warn(&espi->pdev->dev, "len = %d expected 0!", len);
		return ERR_PTR(-EINVAL);
	}

	nents = dma_map_sg(chan->device->dev, sgt->sgl, sgt->nents, dir);
	if (!nents)
		return ERR_PTR(-ENOMEM);

	txd = chan->device->device_prep_slave_sg(chan, sgt->sgl, nents,
						 slave_dirn, DMA_CTRL_ACK);
	if (!txd) {
		dma_unmap_sg(chan->device->dev, sgt->sgl, sgt->nents, dir);
		return ERR_PTR(-ENOMEM);
	}
	return txd;
}

/**
 * lpc31xx_spi_dma_finish() - finishes with a DMA transfer
 * @espi: lpc31xx SPI controller struct
 * @dir: DMA transfer direction
 *
 * Function finishes with the DMA transfer. After this, the DMA buffer is
 * unmapped.
 */
static void lpc31xx_spi_dma_finish(struct lpc31xx_spi *espi,
				  enum dma_transfer_direction dir)
{
	struct dma_chan *chan;
	struct sg_table *sgt;

	printk("JDS - lpc31xx_spi_dma_finish\n");
	if (dir == DMA_DEV_TO_MEM) {
		chan = espi->dma_rx;
		sgt = &espi->rx_sgt;
	} else {
		chan = espi->dma_tx;
		sgt = &espi->tx_sgt;
	}

	dma_unmap_sg(chan->device->dev, sgt->sgl, sgt->nents, dir);
}

static void lpc31xx_spi_dma_callback(void *callback_param)
{
	printk("JDS - lpc31xx_spi_dma_callback\n");
	complete(callback_param);
}

static void lpc31xx_spi_dma_transfer(struct lpc31xx_spi *espi)
{
	struct spi_message *msg = espi->current_msg;
	struct dma_async_tx_descriptor *rxd, *txd;

	printk("JDS - lpc31xx_spi_dma_transfer\n");
	rxd = lpc31xx_spi_dma_prepare(espi, DMA_DEV_TO_MEM);
	if (IS_ERR(rxd)) {
		dev_err(&espi->pdev->dev, "DMA RX failed: %ld\n", PTR_ERR(rxd));
		msg->status = PTR_ERR(rxd);
		return;
	}

	txd = lpc31xx_spi_dma_prepare(espi, DMA_MEM_TO_DEV);
	if (IS_ERR(txd)) {
		lpc31xx_spi_dma_finish(espi, DMA_MEM_TO_DEV);
		dev_err(&espi->pdev->dev, "DMA TX failed: %ld\n", PTR_ERR(rxd));
		msg->status = PTR_ERR(txd);
		return;
	}

	/* We are ready when RX is done */
	rxd->callback = lpc31xx_spi_dma_callback;
	rxd->callback_param = &espi->wait;

	/* Now submit both descriptors and wait while they finish */
	dmaengine_submit(rxd);
	dmaengine_submit(txd);

	dma_async_issue_pending(espi->dma_rx);
	dma_async_issue_pending(espi->dma_tx);

	wait_for_completion(&espi->wait);

	lpc31xx_spi_dma_finish(espi, DMA_MEM_TO_DEV);
	lpc31xx_spi_dma_finish(espi, DMA_DEV_TO_MEM);
}

/**
 * lpc31xx_spi_process_transfer() - processes one SPI transfer
 * @espi: lpc31xx SPI controller struct
 * @msg: current message
 * @t: transfer to process
 *
 * This function processes one SPI transfer given in @t. Function waits until
 * transfer is complete (may sleep) and updates @msg->status based on whether
 * transfer was successfully processed or not.
 */
static void lpc31xx_spi_process_transfer(struct lpc31xx_spi *espi,
					struct spi_message *msg,
					struct spi_transfer *t)
{
	uint32_t tmp;
	struct lpc31xx_spi_chip *chip = spi_get_ctldata(msg->spi);

	printk("JDS - lpc31xx_spi_process_transfer, bus width %d, %d\n", t->bits_per_word, msg->spi->bits_per_word);
	msg->state = t;

	/*
	 * Handle any transfer specific settings if needed. We use
	 * temporary chip settings here and restore original later when
	 * the transfer is finished.
	 */
	if (t->speed_hz || t->bits_per_word) {
		struct lpc31xx_spi_chip tmp_chip = *chip;

		if (t->speed_hz) {
			int err;

			err = lpc31xx_spi_calc_divisors(espi, &tmp_chip,
						       t->speed_hz);
			if (err) {
				dev_err(&espi->pdev->dev,
					"failed to adjust speed\n");
				msg->status = err;
				return;
			}
		}
		lpc31xx_set_cs_data_bits(espi, 0, t->bits_per_word);
		lpc31xx_set_cs_clock(espi, 0, t->speed_hz);

		/* Setup timing and levels before initial chip select */
		tmp = lpc31xx_spi_read(espi, SPI_SLV_SET2_REG(0)) & ~(SPI_SLV2_SPO | SPI_SLV2_SPH);
		/* Clock high between transfers */
#if 0
		tmp |= SPI_SLV2_SPO;
		/* Data captured on 2nd clock edge */
		tmp |= SPI_SLV2_SPH;
#endif
		lpc31xx_spi_write(espi, SPI_SLV_SET2_REG(0), tmp);

		lpc31xx_int_clr(espi, SPI_ALL_INTS);  /****fix from JPP*** */

#if 0
		/* Assert selected chip select */
		if (cs_change)
		{
			/* Force CS assertion */
			spi_force_cs(spidat, spi->chip_select, 0);
		}
		cs_change = t->cs_change;
#endif
		/*
		 * Set up temporary new hw settings for this transfer.
		 */
		lpc31xx_spi_chip_setup(espi, &tmp_chip);
	}
	/* Make sure FIFO is flushed, clear pending interrupts, DMA
	   initially disabled, and then enable SPI interface */
	lpc31xx_spi_write(espi, SPI_CONFIG_REG, (lpc31xx_spi_read(espi, SPI_CONFIG_REG) | SPI_CFG_ENABLE));

	espi->rx = 0;
	espi->tx = 0;

	/*
	 * There is no point of setting up DMA for the transfers which will
	 * fit into the FIFO and can be transferred with a single interrupt.
	 * So in these cases we will be using PIO and don't bother for DMA.
	 */
	if (espi->dma_rx && t->len > SPI_FIFO_DEPTH)
		lpc31xx_spi_dma_transfer(espi);
	else
		lpc31xx_spi_pio_transfer(espi);

	printk("x1\n");
	/*
	 * In case of error during transmit, we bail out from processing
	 * the message.
	 */
	if (msg->status)
		return;

	msg->actual_length += t->len;
	printk("x2\n");

	/*
	 * After this transfer is finished, perform any possible
	 * post-transfer actions requested by the protocol driver.
	 */
	if (t->delay_usecs) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(usecs_to_jiffies(t->delay_usecs));
	}
	if (t->cs_change) {
		if (!list_is_last(&t->transfer_list, &msg->transfers)) {
			/*
			 * In case protocol driver is asking us to drop the
			 * chip select briefly, we let the scheduler to handle
			 * any "delay" here.
			 */
			lpc31xx_spi_cs_control(msg->spi, false);
			cond_resched();
			lpc31xx_spi_cs_control(msg->spi, true);
		}
	}
	printk("x3\n");

	if (t->speed_hz || t->bits_per_word)
		lpc31xx_spi_chip_setup(espi, chip);
}

/*
 * Flush the TX and RX FIFOs
 */
static int lpc313x_fifo_flush(struct lpc31xx_spi *espi)
{
	unsigned long timeout;
	volatile uint32_t tmp;

	/* Clear TX FIFO first */
	lpc31xx_spi_write(espi, SPI_TXF_FLUSH_REG, SPI_TXFF_FLUSH);

	/* Clear RX FIFO */
	timeout = jiffies + msecs_to_jiffies(SPI_TIMEOUT);
	while (!(lpc31xx_spi_read(espi, SPI_STS_REG) & SPI_ST_RX_EMPTY)) {
		if (time_after(jiffies, timeout)) {
			dev_warn(&espi->pdev->dev,
				 "timeout while flushing RX FIFO\n");
			return -ETIMEDOUT;
		}
		tmp = lpc31xx_spi_read(espi, SPI_FIFO_DATA_REG);
	}
	return 0;
}

/*
 * lpc31xx_spi_process_message() - process one SPI message
 * @espi: lpc31xx SPI controller struct
 * @msg: message to process
 *
 * This function processes a single SPI message. We go through all transfers in
 * the message and pass them to lpc31xx_spi_process_transfer(). Chipselect is
 * asserted during the whole message (unless per transfer cs_change is set).
 *
 * @msg->status contains %0 in case of success or negative error code in case of
 * failure.
 */
static void lpc31xx_spi_process_message(struct lpc31xx_spi *espi,
				       struct spi_message *msg)
{
	struct spi_transfer *t;
	int err;

	printk("JDS - lpc31xx_spi_process_message\n");
	/*
	 * Enable the SPI controller and its clock.
	 */
	err = lpc31xx_spi_enable(espi);
	if (err) {
		dev_err(&espi->pdev->dev, "failed to enable SPI controller\n");
		msg->status = err;
		return;
	}
	err = lpc313x_fifo_flush(espi);
	if (err)
		return;
	/*
	 * We explicitly handle FIFO level. This way we don't have to check TX
	 * FIFO status using %SSPSR_TNF bit which may cause RX FIFO overruns.
	 */
	espi->fifo_level = 0;

	/*
	 * Update SPI controller registers according to SPI device and assert
	 * the chip select.
	 */
	lpc31xx_spi_chip_setup(espi, spi_get_ctldata(msg->spi));
	lpc31xx_spi_cs_control(msg->spi, true);

	list_for_each_entry(t, &msg->transfers, transfer_list) {
		lpc31xx_spi_process_transfer(espi, msg, t);
		if (msg->status)
			break;
	}

	/*
	 * Now the whole message is transferred (or failed for some reason). We
	 * deselect the device and disable the SPI controller.
	 */
	lpc31xx_spi_cs_control(msg->spi, false);
	lpc31xx_spi_disable(espi);
}

#define work_to_espi(work) (container_of((work), struct lpc31xx_spi, msg_work))

/**
 * lpc31xx_spi_work() - LPC31xx SPI workqueue worker function
 * @work: work struct
 *
 * Workqueue worker function. This function is called when there are new
 * SPI messages to be processed. Message is taken out from the queue and then
 * passed to lpc31xx_spi_process_message().
 *
 * After message is transferred, protocol driver is notified by calling
 * @msg->complete(). In case of error, @msg->status is set to negative error
 * number, otherwise it contains zero (and @msg->actual_length is updated).
 */
static void lpc31xx_spi_work(struct work_struct *work)
{
	struct lpc31xx_spi *espi = work_to_espi(work);
	struct spi_message *msg;

	printk("JDS - lpc31xx_spi_work\n");
	spin_lock_irq(&espi->lock);
	if (!espi->running || espi->current_msg ||
		list_empty(&espi->msg_queue)) {
		spin_unlock_irq(&espi->lock);
		return;
	}
	msg = list_first_entry(&espi->msg_queue, struct spi_message, queue);
	list_del_init(&msg->queue);
	espi->current_msg = msg;
	spin_unlock_irq(&espi->lock);

	lpc31xx_spi_process_message(espi, msg);

	/*
	 * Update the current message and re-schedule ourselves if there are
	 * more messages in the queue.
	 */
	spin_lock_irq(&espi->lock);
	espi->current_msg = NULL;
	if (espi->running && !list_empty(&espi->msg_queue))
		queue_work(espi->wq, &espi->msg_work);
	spin_unlock_irq(&espi->lock);

	/* notify the protocol driver that we are done with this message */
	msg->complete(msg->context);
}

static irqreturn_t lpc31xx_spi_interrupt(int irq, void *dev_id)
{
	struct lpc31xx_spi *espi = dev_id;
	printk("JDS - lpc31xx_spi_interrupt\n");
#if 0
	uint8_t irq_status = lpc31xx_spi_read_uint8_t(espi, SSPIIR);

	/*
	 * If we got ROR (receive overrun) interrupt we know that something is
	 * wrong. Just abort the message.
	 */
	if (unlikely(irq_status & SSPIIR_RORIS)) {
		/* clear the overrun interrupt */
		lpc31xx_spi_write_uint8_t(espi, SSPICR, 0);
		dev_warn(&espi->pdev->dev,
			 "receive overrun, aborting the message\n");
		espi->current_msg->status = -EIO;
	} else
#endif
	{
		/*
		 * Interrupt is either RX (RIS) or TX (TIS). For both cases we
		 * simply execute next data transfer.
		 */
		if (lpc31xx_spi_read_write(espi)) {
			/*
			 * In normal case, there still is some processing left
			 * for current transfer. Let's wait for the next
			 * interrupt then.
			 */
			return IRQ_HANDLED;
		}
	}

	/*
	 * Current transfer is finished, either with error or with success. In
	 * any case we disable interrupts and notify the worker to handle
	 * any post-processing of the message.
	 */
	printk("irq disable interrupt\n");
	lpc31xx_int_dis(espi, SPI_ALL_INTS);
	lpc31xx_int_clr(espi, SPI_ALL_INTS);
	printk("irq completing wait\n");
	complete(&espi->wait);
	return IRQ_HANDLED;
}

static bool lpc31xx_spi_dma_filter(struct dma_chan *chan, void *filter_param)
{
	chan->private = filter_param;
	return true;
}

static int lpc31xx_spi_setup_dma(struct lpc31xx_spi *espi)
{
	dma_cap_mask_t mask;
	int ret;

	printk("JDS - lpc31xx_spi_setup_dma\n");
	espi->zeropage = (void *)get_zeroed_page(GFP_KERNEL);
	if (!espi->zeropage)
		return -ENOMEM;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

#if 0
	espi->dma_rx_data.port = EP93XX_DMA_SSP;
#endif
	espi->dma_rx_data.direction = DMA_DEV_TO_MEM;
	espi->dma_rx_data.name = "lpc31xx-spi-rx";

	espi->dma_rx = dma_request_channel(mask, lpc31xx_spi_dma_filter,
					   &espi->dma_rx_data);
	if (!espi->dma_rx) {
		ret = -ENODEV;
		goto fail_free_page;
	}

#if 0
	espi->dma_tx_data.port = EP93XX_DMA_SSP;
#endif
	espi->dma_tx_data.direction = DMA_MEM_TO_DEV;
	espi->dma_tx_data.name = "lpc31xx-spi-tx";

	espi->dma_tx = dma_request_channel(mask, lpc31xx_spi_dma_filter,
					   &espi->dma_tx_data);
	if (!espi->dma_tx) {
		ret = -ENODEV;
		goto fail_release_rx;
	}

	return 0;

fail_release_rx:
	dma_release_channel(espi->dma_rx);
	espi->dma_rx = NULL;
fail_free_page:
	free_page((unsigned long)espi->zeropage);

	return ret;
}

static void lpc31xx_spi_release_dma(struct lpc31xx_spi *espi)
{
	printk("JDS - lpc31xx_spi_release_dma\n");
	if (espi->dma_rx) {
		dma_release_channel(espi->dma_rx);
		sg_free_table(&espi->rx_sgt);
	}
	if (espi->dma_tx) {
		dma_release_channel(espi->dma_tx);
		sg_free_table(&espi->tx_sgt);
	}

	if (espi->zeropage)
		free_page((unsigned long)espi->zeropage);
}

/*
 * Setup the initial state of the SPI interface
 */
static void lpc31xx_spi_prep(struct lpc31xx_spi *espi)
{
	uint32_t tmp;

	printk("JDS - lpc313x_spi_prep\n");
	/* Reset SPI block */
	lpc31xx_spi_write(espi, SPI_CONFIG_REG, SPI_CFG_SW_RESET);

	/* Clear FIFOs */
	lpc313x_fifo_flush(espi);

	/* Clear latched interrupts */
	lpc31xx_int_dis(espi, SPI_ALL_INTS);
	lpc31xx_int_clr(espi, SPI_ALL_INTS);

	/* Setup master mode, normal transmit mode, and interslave delay */
	lpc31xx_spi_write(espi, SPI_CONFIG_REG, SPI_CFG_INTER_DLY(1));

	/* Make sure all 3 chip selects are initially disabled */
	lpc31xx_spi_write(espi, SPI_SLV_ENAB_REG, 0);
	lpc31xx_spi_write(espi, SPI_CONFIG_REG, (lpc31xx_spi_read(espi, SPI_CONFIG_REG) | SPI_CFG_UPDATE_EN));

	/* FIFO trip points at 50% */
	lpc31xx_spi_write(espi, SPI_INT_TRSH_REG, (SPI_INT_TSHLD_TX(0x20) | SPI_INT_TSHLD_RX(0x20)));

	/* Only chip select 0 is used in this driver. However, the timings for this
	   chip select effect transfer speed and need to be adjusted for each GPIO
	   based chip select. Use a default value to start with for now. */
	/* Inter-transfer delay is 0 (not used) */
	tmp = lpc31xx_spi_read(espi, SPI_SLV_SET1_REG(0));
	tmp &= ~SPI_SLV1_INTER_TX_DLY(0xFF);
	lpc31xx_spi_write(espi, SPI_SLV_SET1_REG(0), (tmp | SPI_SLV1_INTER_TX_DLY(0)));

	/* Configure enabled chip select slave setting 2 */
	tmp = SPI_SLV2_PPCS_DLY(0) | SPI_SLV2_CS_HIGH | SPI_SLV2_SPO;
	lpc31xx_spi_write(espi, SPI_SLV_SET2_REG(0), tmp);

	/* Use a default of 8 data bits and a 100K clock for now */
	lpc31xx_set_cs_data_bits(espi, 0, 8);
	lpc31xx_set_cs_clock(espi, 0, 100000);

	/* We'll always use CS0 for this driver. Since the chip select is generated
	   by a GPIO, it doesn't matter which one we use */
	lpc31xx_spi_write(espi, SPI_SLV_ENAB_REG, SPI_SLV_EN(0));
	lpc31xx_spi_write(espi, SPI_CONFIG_REG, (lpc31xx_spi_read(espi, SPI_CONFIG_REG) | SPI_CFG_UPDATE_EN));

	/* Controller stays disabled until a transfer occurs */
}

static int __devinit lpc31xx_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct lpc31xx_spi *espi;
	struct resource *res;
	int error;

	printk("JDS - lpc31xx_spi_probe\n");

	master = spi_alloc_master(&pdev->dev, sizeof(*espi));
	if (!master) {
		dev_err(&pdev->dev, "failed to allocate spi master\n");
		return -ENOMEM;
	}
	master->setup = lpc31xx_spi_setup;
	master->transfer = lpc31xx_spi_transfer;
	master->cleanup = lpc31xx_spi_cleanup;
	master->bus_num = pdev->id;
	master->num_chipselect = SPI_NUM_SLAVES;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->dev.of_node = of_node_get(pdev->dev.of_node);

	platform_set_drvdata(pdev, master);

	espi = spi_master_get_devdata(master);

	espi->irq = platform_get_irq(pdev, 4);
	if (espi->irq < 0) {
		error = -EBUSY;
		dev_err(&pdev->dev, "failed to get irq resources\n");
		goto fail_put_clock;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "unable to get iomem resource\n");
		error = -ENODEV;
		goto fail_put_clock;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "unable to request iomem resources\n");
		error = -EBUSY;
		goto fail_put_clock;
	}

	espi->sspdr_phys = res->start;
	espi->regs_base = ioremap(res->start, resource_size(res));
	printk("JDS - base %p phys %x\n", espi->regs_base , res->start);
	if (!espi->regs_base) {
		dev_err(&pdev->dev, "failed to map resources\n");
		error = -ENODEV;
		goto fail_free_mem;
	}

	error = request_irq(espi->irq, lpc31xx_spi_interrupt, 0, "lpc31xx-spi", espi);
	if (error) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto fail_unmap_regs;
	}
	disable_irq(espi->irq);

	if (lpc31xx_spi_setup_dma(espi))
		dev_warn(&pdev->dev, "DMA setup failed. Falling back to PIO\n");

	espi->wq = create_singlethread_workqueue("lpc31xx_spid");
	if (!espi->wq) {
		dev_err(&pdev->dev, "unable to create workqueue\n");
		goto fail_free_dma;
	}
	INIT_WORK(&espi->msg_work, lpc31xx_spi_work);
	INIT_LIST_HEAD(&espi->msg_queue);
	espi->running = true;

	/* Enable clocks */
	lpc31xx_spi_clks_enable();
	cgu_soft_reset_module(SPI_PNRES_APB_SOFT);
	cgu_soft_reset_module(SPI_PNRES_IP_SOFT);

	espi->clk = clk_get(NULL, "spi_clk");
	if (IS_ERR(espi->clk)) {
		dev_err(&pdev->dev, "unable to get spi clock\n");
		error = PTR_ERR(espi->clk);
		goto fail_release_master;
	}
	/*
	 * Calculate maximum and minimum supported clock rates
	 * for the controller.
	 */
	espi->max_rate = clk_get_rate(espi->clk) / 2;
	espi->min_rate = clk_get_rate(espi->clk) / (254 * 256);
	espi->pdev = pdev;

	lpc31xx_spi_prep(espi);

	/* Keep SPI clocks off until a transfer is performed to save power */
	lpc31xx_spi_clks_disable();

	spin_lock_init(&espi->lock);
	init_completion(&espi->wait);

	error = spi_register_master(master);
	if (error) {
		dev_err(&pdev->dev, "failed to register SPI master\n");
		goto fail_free_queue;
	}

	dev_info(&pdev->dev, "LPC31xx SPI Controller at 0x%08lx irq %d\n",
		 (unsigned long)res->start, espi->irq);

	return 0;

fail_free_queue:
	destroy_workqueue(espi->wq);
fail_free_dma:
	lpc31xx_spi_release_dma(espi);
	free_irq(espi->irq, espi);
fail_unmap_regs:
	iounmap(espi->regs_base);
fail_free_mem:
	release_mem_region(res->start, resource_size(res));
fail_put_clock:
	clk_put(espi->clk);
fail_release_master:
	spi_master_put(master);
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int __devexit lpc31xx_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct lpc31xx_spi *espi = spi_master_get_devdata(master);
	struct resource *res;

	printk("JDS - lpc31xx_spi_remove\n");
	spin_lock_irq(&espi->lock);
	espi->running = false;
	spin_unlock_irq(&espi->lock);

	destroy_workqueue(espi->wq);

	/*
	 * Complete remaining messages with %-ESHUTDOWN status.
	 */
	spin_lock_irq(&espi->lock);
	while (!list_empty(&espi->msg_queue)) {
		struct spi_message *msg;

		msg = list_first_entry(&espi->msg_queue,
				       struct spi_message, queue);
		list_del_init(&msg->queue);
		msg->status = -ESHUTDOWN;
		spin_unlock_irq(&espi->lock);
		msg->complete(msg->context);
		spin_lock_irq(&espi->lock);
	}
	spin_unlock_irq(&espi->lock);

	lpc31xx_spi_release_dma(espi);
	free_irq(espi->irq, espi);
	iounmap(espi->regs_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	clk_put(espi->clk);
	platform_set_drvdata(pdev, NULL);

	spi_unregister_master(master);
	return 0;
}

/**
 * Suspend SPI by switching off the IP clocks
 **/
static int lpc31xx_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM
	struct spi_master *master = platform_get_drvdata(pdev);
	struct lpc31xx_spi *espi = spi_master_get_devdata(master);

	/* Check if SPI is idle before we pull off the clock */
	if (unlikely(!list_empty(&espi->msg_queue)))
		return 0;

	/* Pull the clocks off */
	lpc31xx_spi_clks_disable();
#endif
	return 0;
}

/**
 * Resume SPI by switching on the IP clocks
 **/
static int lpc31xx_spi_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	//struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	//struct lpc313xspi *spidat = spi_master_get_devdata(master);

	/* Switch on the clocks */
	lpc31xx_spi_clks_enable();
#endif
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id lpc313x_spi_of_match[] = {
	{ .compatible = "nxp,lpc31xx-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc313x_spi_of_match);
#endif

static struct platform_driver lpc31xx_spi_driver = {
	.probe		= lpc31xx_spi_probe,
	.remove		= __devexit_p(lpc31xx_spi_remove),
	.suspend	= lpc31xx_spi_suspend,
	.resume		= lpc31xx_spi_resume,
	.driver		= {
		.name	= "spi_lpc313x",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lpc313x_spi_of_match,
#endif
	},
};
module_platform_driver(lpc31xx_spi_driver);

MODULE_DESCRIPTION("LPC31xx SPI Controller driver");
MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lpc31xx-spi");

#ifdef OLD_SPI
/*
 * drivers/spi/spi_lpc313x.c
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
 * LPC313X SPI notes
 *
 * The LPC313X SPI Linux driver supports many chip selects using GPO based CS
 * (hardware chip selects are not supported due to timing constraints), clock
 * speeds up to 45MBps, data widths from 4 to 16 bits, DMA support, and full
 * power management.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <mach/registers.h>
#include <mach/dma.h>
#ifndef OLD_DMA
#include <linux/dmaengine.h>
#endif
#include <mach/board.h>
#include <mach/gpio.h>

/* Register access macros */
#define spi_readl(reg) _spi_readl(&SPI_##reg)
#define spi_writel(reg,value) _spi_writel((value),&SPI_##reg)

static inline void _spi_writel(uint16_t value, volatile void *reg)
{
	printk("spi_write %p value %x\n", reg, value);
	__raw_writew(value, reg);
}

static inline uint16_t _spi_readl(volatile void *reg)
{
	uint16_t value;
	value = __raw_readw(reg);
	printk("spi_read %p value %x\n", reg, value);
	return value;
}

struct lpc313xspi
{
	spinlock_t lock;
	struct platform_device *pdev;
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	struct list_head queue;
	wait_queue_head_t waitq;
	struct spi_master *master;
	int irq;
	int id;
	uint32_t spi_base_clock;
	struct lpc313x_spi_cfg *psppcfg;
	uint32_t current_speed_hz [3]; /* Per CS */
	uint8_t current_bits_wd [3]; /* Per CS */

	/* DMA allocated regions */
	uint32_t dma_base_v;
	dma_addr_t dma_base_p;

	/* DMA TX and RX physical and mapped spaces */
	uint32_t dma_tx_base_v, dma_rx_base_v, dma_tx_base_p, dma_rx_base_p;

	/* Allocated DMA channels */
#ifdef OLD_DMA
	int tx_dma_ch, rx_dma_ch;
#else
	struct dma_chan *tx_dma_ch, *rx_dma_ch;
	struct lpc31xx_dma_data tx_dma_data, rx_dma_data;
#endif

	/* DMA event flah */
	volatile int rxdmaevent;
};

/*
 * Enable or disable the SPI clocks
 */
static void lpc313x_spi_clks_enable(void)
{
	struct clk *clk;
	int ret;

	clk = clk_get(NULL, "spi_pclk");
	ret = clk_enable(clk);
	clk_put(clk);
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

static void lpc313x_spi_clks_disable(void)
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

/*
 * Flush the TX and RX FIFOs
 */
static void lpc313x_fifo_flush(struct lpc313xspi *spidat)
{
	volatile uint32_t tmp;

	/* Clear TX FIFO first */
	spi_writel(TXF_FLUSH_REG, SPI_TXFF_FLUSH);

	/* Clear RX FIFO */
	while (!(spi_readl(STS_REG) & SPI_ST_RX_EMPTY))
	{
		tmp = spi_readl(FIFO_DATA_REG);
	}
}

/*
 * Clear a latched SPI interrupt
 */
static inline void lpc313x_int_clr(struct lpc313xspi *spidat, uint32_t ints)
{
	spi_writel(INT_CLRS_REG, ints);
}

/*
 * Disable a SPI interrupt
 */
static inline void lpc313x_int_dis(struct lpc313xspi *spidat, uint32_t ints)
{
	spi_writel(INT_CLRE_REG, ints);
}

/*
 * Enable a SPI interrupt
 */
static inline void lpc313x_int_en(struct lpc313xspi *spidat, uint32_t ints)
{
	spi_writel(INT_SETE_REG, ints);
}

/*
 * Set a SPI chip select state
 */
static inline void spi_force_cs(struct lpc313xspi *spidat, uint8_t cs, uint cs_state)
{
	spidat->psppcfg->spics_cfg[cs].spi_cs_set((int) cs, (int) cs_state);
}

/*
 * Set data width for the SPI chip select
 */
static void lpc313x_set_cs_data_bits(struct lpc313xspi *spidat, uint8_t cs, uint8_t data_width)
{
	if (spidat->current_bits_wd[cs] != data_width)
	{
		uint32_t tmp = spi_readl(SLV_SET2_REG(0));
		tmp &= ~SPI_SLV2_WD_SZ(0x1F);
		tmp |= SPI_SLV2_WD_SZ((uint32_t) (data_width - 1));
		spi_writel(SLV_SET2_REG(0), tmp);

		spidat->current_bits_wd[cs] = data_width;
	}
}

/*
 * Set clock rate and delays for the SPI chip select
 */
static void lpc313x_set_cs_clock(struct lpc313xspi *spidat, uint8_t cs, uint32_t clockrate)
{
	uint32_t reg, div, ps, div1;

	if (clockrate != spidat->current_speed_hz[cs])
	{
		reg = spi_readl(SLV_SET1_REG(0));
		reg &= ~0xFFFF;

		div = (spidat->spi_base_clock + clockrate / 2) / clockrate;
		if (div > SPI_MAX_DIVIDER)
			div = SPI_MAX_DIVIDER;
		if (div < SPI_MIN_DIVIDER)
			div = SPI_MIN_DIVIDER;

		ps = (((div - 1) / 512) + 1) * 2;
		div1 = (((div + ps / 2) / ps) - 1);

		spi_writel(SLV_SET1_REG(0),
			(reg | SPI_SLV1_CLK_PS(ps) | SPI_SLV1_CLK_DIV1(div1)));

		spidat->current_speed_hz[cs] = clockrate;
	}
}

/*
 * Setup the initial state of the SPI interface
 */
static void lpc313x_spi_prep(struct lpc313xspi *spidat)
{
	uint32_t tmp;

	/* Reset SPI block */
	spi_writel(CONFIG_REG, SPI_CFG_SW_RESET);

	/* Clear FIFOs */
	lpc313x_fifo_flush(spidat);

	/* Clear latched interrupts */
	lpc313x_int_dis(spidat, SPI_ALL_INTS);
	lpc313x_int_clr(spidat, SPI_ALL_INTS);

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
	lpc313x_set_cs_data_bits(spidat, 0, 8);
	lpc313x_set_cs_clock(spidat, 0, 100000);

	/* We'll always use CS0 for this driver. Since the chip select is generated
	   by a GPO, it doesn't matter which one we use */
	spi_writel(SLV_ENAB_REG, SPI_SLV_EN(0));
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_UPDATE_EN));

	/* Controller stays disabled until a transfer occurs */
}

/*
 * Setup a SPI transfer
 */
static int lpc313x_spi_setup(struct spi_device *spi)
{
	unsigned int bits = spi->bits_per_word;

	/* There really isn't anything to do in this function, so verify the
	   parameters are correct for the transfer */
	if (spi->chip_select > spi->master->num_chipselect)
	{
		dev_dbg(&spi->dev,
			"setup: invalid chipselect %u (%u defined)\n",
			spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	if (bits == 0)
	{
		bits = 8;
	}
	if ((bits < 4) || (bits > 16))
	{
		dev_dbg(&spi->dev,
			"setup: invalid bits_per_word %u (8 to 16)\n", bits);
		return -EINVAL;
	}

	return 0;
}

/*
 * Handle the SPI interrupt
 */
static irqreturn_t lpc313x_spi_irq_handler(int irq, void *dev_id)
{
	struct lpc313xspi *spidat = dev_id;

	/* Disable interrupts for now, do not clear the interrupt states */
	lpc313x_int_dis(spidat, SPI_ALL_INTS);
	spidat->rxdmaevent = 1;

	wake_up(&spidat->waitq);

	return IRQ_HANDLED;
}

/*
 * SPI DMA TX callback
 */
static void lpc313x_dma_tx_spi_irq(int ch, dma_irq_type_t dtype, void *handle)
{
	/* Nothing really needs to be done with the DMA TX interrupt, all the work
	   is done with RX, so just return and let the DMA handler clear it. The
	   SPI will stall the clock if the TX FIFO becomes empty so there is no
	   chance of some type of underflow. */
}

/*
 * SPI DMA RX callback
 */
static void lpc313x_dma_rx_spi_irq(int ch, dma_irq_type_t dtype, void *handle)
{
	struct lpc313xspi *spidat = (struct lpc313xspi *) handle;

	if (dtype == DMA_IRQ_FINISHED)
	{
#ifdef OLD_DMA
		/* Disable interrupts for now */
		dma_set_irq_mask(spidat->rx_dma_ch, 1, 1);
#endif
		/* Flag event and wakeup */
		spidat->rxdmaevent = 1;
		wake_up(&spidat->waitq);
	}
	else if (dtype == DMA_IRQS_ABORT)
	{
		/* DMA data abort - this is global for the entire DMA
		   peripheral. Do nothing, as this might not be a
		   error for this channel. */
	}
}

/*
 * Handle a DMA transfer
 */
static int lpc313x_spi_dma_transfer(struct lpc313xspi *spidat, struct spi_transfer *t,
					uint8_t bits_per_word, int dmamapped)
{
	dma_setup_t dmarx, dmatx;
	int status = 0;
	uint32_t src, dest, srcmapped = 0, destmapped = 0;
	struct device *dev = &spidat->pdev->dev;
	static struct dma_async_tx_descriptor *txd;
	static struct dma_async_tx_descriptor *rxd;
	enum dma_slave_buswidth buswidth;
	struct dma_slave_config conf;

	printk("JDS -lpc313x_spi_dma_transfer\n");
	/* Set the FIFO trip level to the transfer size */
	spi_writel(INT_TRSH_REG, (SPI_INT_TSHLD_TX(16) |
		SPI_INT_TSHLD_RX(1)));
	spi_writel(DMA_SET_REG, (SPI_DMA_TX_EN | SPI_DMA_RX_EN));
	lpc313x_int_dis(spidat, SPI_ALL_INTS);
	lpc313x_int_en(spidat, SPI_OVR_INT);

#ifdef OLD_DMA
	/* Setup transfer */
	if (bits_per_word > 8) {
		dmarx.cfg = DMA_CFG_TX_HWORD | DMA_CFG_RD_SLV_NR(DMA_SLV_SPI_RX) | DMA_CFG_WR_SLV_NR(0);
		dmatx.cfg = DMA_CFG_TX_HWORD | DMA_CFG_RD_SLV_NR(0) | DMA_CFG_WR_SLV_NR(DMA_SLV_SPI_TX);
	} else {
		dmarx.cfg = DMA_CFG_TX_BYTE | DMA_CFG_RD_SLV_NR(DMA_SLV_SPI_RX) | DMA_CFG_WR_SLV_NR(0);
		dmatx.cfg = DMA_CFG_TX_BYTE | DMA_CFG_RD_SLV_NR(0) | DMA_CFG_WR_SLV_NR(DMA_SLV_SPI_TX);
	}
#else
	if (bits_per_word > 8)
		buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
#endif

	/* Determine the DMA source and destination addresses. If DMA buffers weren't
	   passed to this handler, they need to be mapped */
	if (dmamapped)
	{
		src = t->tx_dma;
		dest = t->rx_dma;
		if ((src == 0) && (dest == 0))
		{
			/* DMA mapped flag set, but not mapped */
			status = -ENOMEM;
			goto exit;
		}

		/* At least one of the DMA buffers are already mapped */
		if (src == 0)
		{
			/* Use temporary buffer for TX */
			src = spidat->dma_tx_base_p;
		}
		else
		{
			dest = spidat->dma_rx_base_p;
		}
	}
	else
	{
		/* Does TX buffer need to be DMA mapped */
		if (t->tx_buf == NULL)
		{
			/* Use the temporary buffer for this transfer */
			src = spidat->dma_tx_base_p;
		}
		else
		{
			/* Map DMA buffer */
			src = (uint32_t) dma_map_single(dev, (void *) t->tx_buf,
				t->len, DMA_TO_DEVICE);
			if (dma_mapping_error(dev, src))
			{
				status = -ENOMEM;
				goto exit;
			}

			srcmapped = src;
		}

		/* Does RX buffer need to be DMA mapped */
		if (t->rx_buf == NULL)
		{
			/* Use the temporary buffer for this transfer */
			dest = spidat->dma_rx_base_p;
		}
		else
		{
			/* Map DMA buffer */
			dest = (uint32_t) dma_map_single(dev, (void *) t->rx_buf,
				t->len, DMA_FROM_DEVICE);
			if (dma_mapping_error(dev, dest))
			{
				status = -ENOMEM;
				goto exit;
			}

			destmapped = dest;
		}
	}

#ifdef OLD_DMA
	/* Setup transfer data for DMA */
	dmarx.trans_length = (t->len - 1);
	dmarx.src_address = (SPI_PHYS + 0x0C);
	dmarx.dest_address = dest;
	dmatx.trans_length = (t->len - 1);
	dmatx.src_address = src;
	dmatx.dest_address = (SPI_PHYS + 0x0C);

	/* Setup the channels */
	dma_prog_channel(spidat->rx_dma_ch, &dmarx);
	dma_prog_channel(spidat->tx_dma_ch, &dmatx);

	/* Make sure the completion interrupt is enabled for RX, TX disabled */
	dma_set_irq_mask(spidat->rx_dma_ch, 1, 0);
	dma_set_irq_mask(spidat->tx_dma_ch, 1, 1);

	/* Start the transfer */
	spidat->rxdmaevent = 0;
	dma_start_channel(spidat->rx_dma_ch);
	dma_start_channel(spidat->tx_dma_ch);
#else
	memset(&conf, 0, sizeof(conf));
	conf.src_addr = (SPI_PHYS + 0x0C);
	conf.src_addr_width = buswidth;
	conf.dst_addr = dest;
	conf.dst_addr_width = buswidth;
	conf.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(spidat->rx_dma_ch, &conf);

	memset(&conf, 0, sizeof(conf));
	conf.src_addr = src;
	conf.src_addr_width = buswidth;
	conf.dst_addr = (SPI_PHYS + 0x0C);
	conf.dst_addr_width = buswidth;
	conf.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(spidat->tx_dma_ch, &conf);


	} else {
		chan = spidat->dma_tx_ch;
		buf = t->tx_buf;

		conf.dst_addr = dst;
		conf.dst_addr_width = buswidth;
		slave_dirn = DMA_MEM_TO_DEV;
	}

	ret = dmaengine_slave_config(chan, &conf);
	if (ret)
		return ERR_PTR(ret);


	if (WARN_ON(len)) {
		dev_warn(&spidat->pdev->dev, "len = %d expected 0!", len);
		return ERR_PTR(-EINVAL);
	}
	txd = chan->device->lpc31xx_dma_prep_dma_memcpy(chan, dest, src, len, DMA_CTRL_ACK);
#endif
	/* Wait for DMA to complete */
	wait_event_interruptible(spidat->waitq, spidat->rxdmaevent);

exit:
#ifdef OLD_DMA
	dma_stop_channel(spidat->tx_dma_ch);
	dma_stop_channel(spidat->rx_dma_ch);
#endif
	/* Unmap buffers */
	if (srcmapped != 0)
	{
		dma_unmap_single(dev, srcmapped, t->len, DMA_TO_DEVICE);
	}
	if (destmapped != 0)
	{
		dma_unmap_single(dev, destmapped, t->len, DMA_FROM_DEVICE);
	}

	return status;
}

/*
 * The bulk of the transfer work is done in this function. Based on the transfer
 * size, either FIFO (PIO) or DMA mode may be used.
 */
static void lpc313x_work_one(struct lpc313xspi *spidat, struct spi_message *m)
{
	struct spi_device *spi = m->spi;
	struct spi_transfer *t;
	unsigned int wsize, cs_change = 1;
	int status = 0;
	unsigned long flags;
	uint32_t tmp;

	/* Enable SPI clock and interrupts */
	spin_lock_irqsave(&spidat->lock, flags);
	lpc313x_spi_clks_enable();
	enable_irq(spidat->irq);

	/* Make sure FIFO is flushed and clear any pending interrupts */
	lpc313x_fifo_flush(spidat);
	/* lpc313x_int_clr(spidat, SPI_ALL_INTS); ***fix from JPP*** */

	/* Process each transfer in the message */
	list_for_each_entry (t, &m->transfers, transfer_list) {
		const void *txbuf = t->tx_buf;
		void *rxbuf = t->rx_buf;
		uint32_t data;
		unsigned int rlen, tlen = t->len;
		uint32_t speed_hz = t->speed_hz ? : spi->max_speed_hz;
		uint8_t bits_per_word = t->bits_per_word ? : spi->bits_per_word;

		/* Bits per word, data transfer size, and transfer counter */
		bits_per_word = bits_per_word ? : 8;
		wsize = bits_per_word >> 3;
		rlen = tlen;

		/* Setup the appropriate chip select */
		lpc313x_set_cs_clock(spidat, spi->chip_select, speed_hz);
		lpc313x_set_cs_data_bits(spidat, spi->chip_select, bits_per_word);
 
		/* Setup timing and levels before initial chip select */
		tmp = spi_readl(SLV_SET2_REG(0)) & ~(SPI_SLV2_SPO | SPI_SLV2_SPH);
		if (spidat->psppcfg->spics_cfg[spi->chip_select].spi_spo != 0)
		{
			/* Clock high between transfers */
			tmp |= SPI_SLV2_SPO;
		}
		if (spidat->psppcfg->spics_cfg[spi->chip_select].spi_sph != 0)
		{
			/* Data captured on 2nd clock edge */
			tmp |= SPI_SLV2_SPH;
		}
		spi_writel(SLV_SET2_REG(0), tmp);

		lpc313x_int_clr(spidat, SPI_ALL_INTS);  /****fix from JPP*** */

		/* Make sure FIFO is flushed, clear pending interrupts, DMA
		   initially disabled, and then enable SPI interface */
		spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_ENABLE));

		/* Assert selected chip select */
		if (cs_change)
		{
			/* Force CS assertion */
			spi_force_cs(spidat, spi->chip_select, 0);
		}
		cs_change = t->cs_change;

		/* The driver will pick the best transfer method based on the
		   current transfer size. For sizes smaller than the FIFO depth,
		   the FIFOs are used without DMA support. For transfers larger
		   than the FIFO depth, DMA is used. When dealing with fast SPI
		   clock rates and transfers larger than the FIFO depth, DMA is
		   required. The higher level SPI functions will limit transfer
		   sizes to 4Kbytes. */
		/***Fix from JPP ******/
		/*if (tlen < (SPI_FIFO_DEPTH * wsize)) {*/
		if (0) { //***MOD: Non-DMA SPI code broken -> possibly off-by-one bit error.
			if ((txbuf == NULL) && (rxbuf == NULL))
			{
				/* A PIO mode DMA transfer requires mapped
				   memory. Something is wrong. DMA transfer? */
				dev_err(&spidat->pdev->dev, "No mapped buffers\n");
				status = -EIO;
				goto exit;
			}

			/* Set the FIFO trip level to the transfer size */
			spi_writel(INT_TRSH_REG, (SPI_INT_TSHLD_TX(0) |
				SPI_INT_TSHLD_RX(tlen - 1)));
			spi_writel(DMA_SET_REG, 0);

			/* Fill TX FIFO */
			while ((!(spi_readl(STS_REG) & SPI_ST_TX_FF)) && (tlen > 0))
			{
				/* Fill FIFO */
				if (txbuf)
			{
					data = (wsize == 1)
						? *(const uint8_t *) txbuf
						: *(const u16 *) txbuf;
					spi_writel(FIFO_DATA_REG, data);
					txbuf += wsize;
				}
				else
				{
					/* Send dummy data */
					spi_writel(FIFO_DATA_REG, 0xFFFF);
				}

				tlen--;
			}

			/* Wait for data */
			lpc313x_int_en(spidat, (SPI_RX_INT | SPI_TO_INT | SPI_OVR_INT));
			spin_unlock_irqrestore(&spidat->lock, flags);
			wait_event_interruptible(spidat->waitq,
				(spi_readl(INT_STS_REG) & (SPI_RX_INT | SPI_OVR_INT)));
			spin_lock_irqsave(&spidat->lock, flags);

			/* Has an overflow occurred? */
			if (unlikely(spi_readl(INT_STS_REG) & SPI_OVR_INT))
			{
				/* RX FIFO overflow */
				dev_err(&spidat->pdev->dev, "RX FIFO overflow.\n");
				status = -EIO;
				goto exit;
			}

			/* Is there any data to read? */
			while (!(spi_readl(STS_REG) & SPI_ST_RX_EMPTY))
			{
				data = spi_readl(FIFO_DATA_REG);
				/* The data can be tossed if there is no RX buffer */
				if (rxbuf)
				{
					if (wsize == 1)
					{
						*(uint8_t *)rxbuf = (uint8_t) data;
					}
					else
					{
						*(u16 *)rxbuf = (u16) data;
					}

					rxbuf += wsize;
				}

				rlen--;
			}
		}
		else {
			/* DMA will be used for the transfer */
			spin_unlock_irqrestore(&spidat->lock, flags);
			status = lpc313x_spi_dma_transfer(spidat, t, bits_per_word,
				m->is_dma_mapped);
			spin_lock_irqsave(&spidat->lock, flags);
			if (status < 0)
				goto exit;
		}

		m->actual_length += t->len;
		if (t->delay_usecs)
		{
			udelay(t->delay_usecs);
		}

		if (!cs_change)
			continue;

		if (t->transfer_list.next == &m->transfers)
			break;
	}

exit:
	if (!(status == 0 && cs_change))
	{
		spi_force_cs(spidat, spi->chip_select, 1);
	}

	/* Disable SPI, stop SPI clock to save power */
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) & ~SPI_CFG_ENABLE));
	disable_irq(spidat->irq);
	lpc313x_spi_clks_disable();

	spin_unlock_irqrestore(&spidat->lock, flags);
	m->status = status;
	m->complete(m->context);
}

/*
 * Work queue function
 */
static void lpc313x_work(struct work_struct *work)
{
	struct lpc313xspi *spidat = container_of(work, struct lpc313xspi, work);
	unsigned long flags;

	spin_lock_irqsave(&spidat->lock, flags);

	while (!list_empty(&spidat->queue))
	{
		struct spi_message *m;

		m = container_of(spidat->queue.next, struct spi_message, queue);
		list_del_init(&m->queue);

		spin_unlock_irqrestore(&spidat->lock, flags);
		lpc313x_work_one(spidat, m);
		spin_lock_irqsave(&spidat->lock, flags);
	}

	spin_unlock_irqrestore(&spidat->lock, flags);
}

/*
 * Kick off a SPI transfer
 */
static int lpc313x_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_master *master = spi->master;
	struct lpc313xspi *spidat = spi_master_get_devdata(master);
	struct device *controller = spi->master->dev.parent;
	struct spi_transfer *t;
	unsigned long flags;

	m->actual_length = 0;

	/* check each transfer's parameters */
	list_for_each_entry (t, &m->transfers, transfer_list)
	{
		uint8_t bits_per_word = t->bits_per_word ? : spi->bits_per_word;

		bits_per_word = bits_per_word ? : 8;
		if ((!t->tx_buf) && (!t->rx_buf) && (t->len))
		{
			return -EINVAL;
		}
		if ((bits_per_word < 4) || (bits_per_word > 16))
		{
			return -EINVAL;
		}

		dev_dbg(controller,
			"  xfer %p: len %u tx %p/%08x rx %p/%08x DMAmapped=%d\n",
			t, t->len, t->tx_buf, t->tx_dma,
			t->rx_buf, t->rx_dma, m->is_dma_mapped);
	}

	spin_lock_irqsave(&spidat->lock, flags);
	list_add_tail(&m->queue, &spidat->queue);
	queue_work(spidat->workqueue, &spidat->work);
	spin_unlock_irqrestore(&spidat->lock, flags);

	return 0;
}

#ifdef CONFIG_OF
static void spi_set_cs0_state(int cs_num, int state)
{
	(void) cs_num;
	lpc313x_gpio_set_value(GPIO_SPI_CS_OUT0, state);
}

static void spi_set_cs1_state(int cs_num, int state)
{
	(void) cs_num;
printk("cs1 state %d\n", state);
	lpc313x_gpio_set_value(GPIO_MUART_CTS_N, state);
}

static void spi_set_cs2_state(int cs_num, int state)
{
printk("cs2 state %d\n", state);
	(void) cs_num;
	lpc313x_gpio_set_value(GPIO_MUART_RTS_N, state);
}

struct lpc313x_spics_cfg lpc313x_stdspics_cfg[] =
{
	/* SPI CS0 */
	[0] =
	{
		.spi_spo	= 0, /* Low clock between transfers */
		.spi_sph	= 0, /* Data capture on first clock edge (high edge with spi_spo=0) */
		.spi_cs_set	= spi_set_cs0_state,
	},
	[1] =
	{
		.spi_spo	= 0, /* Low clock between transfers */
		.spi_sph	= 0, /* Data capture on first clofck edge (high edge with spi_spo=0) */
		.spi_cs_set	= spi_set_cs1_state,
	},
	[2] =
	{
		.spi_spo	= 0, /* Low clock between transfers */
		.spi_sph	= 1, /* Data capture on first clock edge (high edge with spi_spo=0) */
		.spi_cs_set	= spi_set_cs2_state,
	},
};

struct lpc313x_spi_cfg lpc313x_spidata =
{
	.num_cs			= ARRAY_SIZE(lpc313x_stdspics_cfg),
	.spics_cfg		= lpc313x_stdspics_cfg,
};
#endif

static bool lpc313x_spi_dma_filter(struct dma_chan *chan, void *filter_param)
{
	chan->private = filter_param;
	return true;
}

static void spi_set_cs0_state(int cs_num, int state)
{
	(void) cs_num;
	lpc313x_gpio_set_value(GPIO_SPI_CS_OUT0, state);
}

static void spi_set_cs1_state(int cs_num, int state)
{
	(void) cs_num;
printk("cs1 state %d\n", state);
	lpc313x_gpio_set_value(GPIO_MUART_CTS_N, state);
}

static void spi_set_cs2_state(int cs_num, int state)
{
printk("cs2 state %d\n", state);
	(void) cs_num;
	lpc313x_gpio_set_value(GPIO_MUART_RTS_N, state);
}

struct lpc313x_spics_cfg lpc313x_stdspics_cfg[] =
{
	/* SPI CS0 */
	[0] =
	{
		.spi_spo	= 0, /* Low clock between transfers */
		.spi_sph	= 0, /* Data capture on first clock edge (high edge with spi_spo=0) */
		.spi_cs_set	= spi_set_cs0_state,
	},
	[1] =
	{
		.spi_spo	= 0, /* Low clock between transfers */
		.spi_sph	= 0, /* Data capture on first clock edge (high edge with spi_spo=0) */
		.spi_cs_set	= spi_set_cs1_state,
	},
	[2] =
	{
		.spi_spo	= 0, /* Low clock between transfers */
		.spi_sph	= 1, /* Data capture on first clock edge (high edge with spi_spo=0) */
		.spi_cs_set	= spi_set_cs2_state,
	},
};

struct lpc313x_spi_cfg lpc313x_spidata =
{
	.num_cs			= ARRAY_SIZE(lpc313x_stdspics_cfg),
	.spics_cfg		= lpc313x_stdspics_cfg,
};

/*
 * SPI driver probe
 */
static int __init lpc313x_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct lpc313xspi *spidat;
	struct clk *clk;
	int ret, irq, i;
	dma_addr_t dma_handle;
	dma_cap_mask_t mask;

	/* Get required resources, last IRQ is general one */
	irq = platform_get_irq(pdev, 4);
	if ((irq < 0) | (irq >= NR_IRQS)) {
		return -EBUSY;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct lpc313xspi));
	if (!master) {
		return -ENODEV;
	}
	spidat = spi_master_get_devdata(master);

	platform_set_drvdata(pdev, master);

	/* Is a board specific configuration available? */
	spidat->psppcfg = (struct lpc313x_spi_cfg *) pdev->dev.platform_data;
#ifdef CONFIG_OF
	spidat->psppcfg = &lpc313x_spidata;
#endif
	if (spidat->psppcfg == NULL) {
		/* No platform data, exit */
		ret = -ENODEV;
		goto errout;
	}
	if (spidat->psppcfg->num_cs < 1) {
		/* No chip selects supported in board structure, exit */
		ret = -ENODEV;
		goto errout;
	}
	for (i = 0; i < spidat->psppcfg->num_cs; i++) {
		if (spidat->psppcfg->spics_cfg[i].spi_cs_set == NULL) {
			/* Missing hardware CS control callback, exit */
			ret = -ENODEV;
			goto errout;
		}
	}

	/* Save ID for this device */
	spidat->pdev = pdev;
	spidat->id = pdev->id;
	spidat->irq = irq;
	spin_lock_init(&spidat->lock);

	INIT_WORK(&spidat->work, lpc313x_work);
	INIT_LIST_HEAD(&spidat->queue);
	init_waitqueue_head(&spidat->waitq);
	spidat->workqueue = create_singlethread_workqueue(dev_name(master->dev.parent));	//***MOD:Fix from JPP to compile to latest versions of Linux
	if (!spidat->workqueue) {
		ret = -ENOMEM;
		goto errout;
	}

	ret = request_irq(irq, lpc313x_spi_irq_handler, IRQF_DISABLED, "spi", spidat);
	if (ret) {
		ret = -EBUSY;
		goto errout2;
	}
	disable_irq(irq);

	master->bus_num = spidat->id;
	master->setup = lpc313x_spi_setup;
	master->transfer = lpc313x_spi_transfer;
	master->num_chipselect = spidat->psppcfg->num_cs;

	/* Setup several work DMA buffers for dummy TX and RX data. These buffers just
	   hold the temporary TX or RX data for the unused half of the transfer and have
	   a size of 4K (the maximum size of a transfer) */
	spidat->dma_base_v = (uint32_t) dma_alloc_coherent(&pdev->dev, (4096 << 1),
		&dma_handle, GFP_KERNEL);
	if (spidat->dma_base_v == (uint32_t) NULL) {
		dev_err(&pdev->dev, "error getting DMA region.\n");
		ret = -ENOMEM;
		goto errout3;
	}
	spidat->dma_base_p = dma_handle;;

	spidat->dma_tx_base_p = (uint32_t) spidat->dma_base_p;
	spidat->dma_tx_base_v = spidat->dma_base_v;
	spidat->dma_rx_base_p = (uint32_t) spidat->dma_base_p + 4096;
	spidat->dma_rx_base_v = spidat->dma_base_v + 4096;

	/* Fill dummy TX buffer with 0 */
	memset((void *) spidat->dma_tx_base_v, 0, 4096);

	/* Enable clocks */
	lpc313x_spi_clks_enable();
	cgu_soft_reset_module(SPI_PNRES_APB_SOFT);
	cgu_soft_reset_module(SPI_PNRES_IP_SOFT);

	/* Initial setup of SPI */
	clk = clk_get(NULL, "spi_clk");
	spidat->spi_base_clock = clk->get_rate(clk);
	clk_put(clk);
	lpc313x_spi_prep(spidat);

	/* Keep SPI clocks off until a transfer is performed to save power */
	lpc313x_spi_clks_disable();

#ifdef OLD_DMA
	/* Request RX and TX DMA channels */
	spidat->tx_dma_ch = spidat->rx_dma_ch = -1;
	spidat->tx_dma_ch = dma_request_channel_x("spi_tx", lpc313x_dma_tx_spi_irq, spidat);
	if (spidat->tx_dma_ch < 0) {
		dev_err(&pdev->dev, "error getting TX DMA channel.\n");
		ret = -EBUSY;
		goto errout4;
	}
	spidat->rx_dma_ch = dma_request_channel_x("spi_rx", lpc313x_dma_rx_spi_irq, spidat);
	if (spidat->rx_dma_ch < 0) {
		dev_err(&pdev->dev, "error getting RX DMA channel.\n");
		ret = -EBUSY;
		goto errout4;
	}
#else
	/* Try to acquire a generic DMA engine slave channel */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	spidat->tx_dma_ch = spidat->rx_dma_ch = NULL;

	spidat->tx_dma_data.port = DMA_CFG_RD_SLV_NR(DMA_SLV_SPI_TX);
	spidat->tx_dma_data.direction = DMA_MEM_TO_DEV;
	spidat->tx_dma_data.name = "spi-tx";
	spidat->tx_dma_ch = dma_request_channel(mask, lpc313x_spi_dma_filter,
								&spidat->tx_dma_data.port);
	if (!spidat->tx_dma_ch) {
		dev_err(&pdev->dev, "error getting TX DMA channel.\n");
		ret = -EBUSY;
		goto errout4;
	}
	spidat->rx_dma_data.port = DMA_CFG_WR_SLV_NR(DMA_SLV_SPI_RX);
	spidat->rx_dma_data.direction = DMA_DEV_TO_MEM;
	spidat->rx_dma_data.name = "spi-rx";
	spidat->rx_dma_ch = dma_request_channel(mask, lpc313x_spi_dma_filter,
								&spidat->rx_dma_data.port);
	if (!spidat->rx_dma_ch) {
		dev_err(&pdev->dev, "error getting RX DMA channel.\n");
		ret = -EBUSY;
		goto errout4;
	}
#endif

#ifdef CONFIG_OF
	master->dev.of_node = of_node_get(pdev->dev.of_node);
#endif

	ret = spi_register_master(master);
	if (ret)
		goto errout4;

	dev_info(&pdev->dev, "LPC313x SPI driver\n");

	return 0;

errout4:
#ifdef OLD_DMA
	if (spidat->tx_dma_ch != -1)
		dma_release_channel_x(spidat->tx_dma_ch);
	if (spidat->rx_dma_ch != -1)
		dma_release_channel_x(spidat->rx_dma_ch);
#else
	if (spidat->tx_dma_ch)
		dma_release_channel(spidat->tx_dma_ch);
	if (spidat->tx_dma_ch)
		dma_release_channel(spidat->rx_dma_ch);
#endif
	dma_free_coherent(&pdev->dev, (4096 << 1), (void *) spidat->dma_base_v,
		spidat->dma_base_p);
errout3:
	free_irq(spidat->irq, spidat);
errout2:
	destroy_workqueue(spidat->workqueue);
errout:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return ret;
}

/*
 * SPI driver removal
 */
static int __devexit lpc313x_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct lpc313xspi *spidat = spi_master_get_devdata(master);

	printk("JDS - lpc313x_spi_remove\n");
	/* Disable SPI interface */
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) & ~SPI_CFG_ENABLE));
	lpc313x_spi_clks_disable();

	spi_unregister_master(master);
	platform_set_drvdata(pdev, NULL);

#ifdef OLD_DMA
	if (spidat->tx_dma_ch != -1)
		dma_release_channel_x(spidat->tx_dma_ch);
	if (spidat->rx_dma_ch != -1)
		dma_release_channel_x(spidat->rx_dma_ch);
#else
	dma_release_channel(spidat->tx_dma_ch);
	dma_release_channel(spidat->rx_dma_ch);
#endif

	dma_free_coherent(&pdev->dev, (4096 << 1), (void *) spidat->dma_base_v,
		spidat->dma_base_p);

	/* Free resources */
	free_irq(spidat->irq, pdev);

	destroy_workqueue(spidat->workqueue);
	spi_master_put(master);

	return 0;
}

/**
 * Suspend SPI by switching off the IP clocks
 **/
static int lpc313x_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct lpc313xspi *spidat = spi_master_get_devdata(master);

	/* Check if SPI is idle before we pull off the clock */
	if (unlikely(!list_empty(&spidat->queue)))
		return 0;

	/* Pull the clocks off */
	lpc313x_spi_clks_disable();
#endif
	return 0;
}

/**
 * Resume SPI by switching on the IP clocks
 **/
static int lpc313x_spi_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	//struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	//struct lpc313xspi *spidat = spi_master_get_devdata(master);

	/* Switch on the clocks */
	lpc313x_spi_clks_enable();
#endif
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id lpc313x_spi_of_match[] = {
	{ .compatible = "nxp,lpc31xx-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc313x_spi_of_match);
#endif

static struct platform_driver lpc313x_spi_driver = {
	.probe		= lpc313x_spi_probe,
	.remove		= __devexit_p(lpc313x_spi_remove),
	.suspend    = lpc313x_spi_suspend,
	.resume     = lpc313x_spi_resume,
	.driver		= {
		.name	= "spi_lpc313x",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lpc313x_spi_of_match,
#endif
	},
};

static int __init lpc313x_spi_init(void)
{
	return platform_driver_register(&lpc313x_spi_driver);
}

static void __exit lpc313x_spi_exit(void)
{
	platform_driver_unregister(&lpc313x_spi_driver);
}

module_init(lpc313x_spi_init);
module_exit(lpc313x_spi_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC313X SPI Driver");
MODULE_LICENSE("GPL");
#endif
