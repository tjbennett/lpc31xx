/*
 * sound/soc/lpc31xx/lpc31xx-i2s.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/board.h>
#include <mach/registers.h>
#include <mach/hardware.h>

#include "lpc31xx-pcm.h"
#include "lpc31xx-i2s.h"

static struct resource *lpc31xx_mem = 0;
static void __iomem *lpc31xx_regs = 0;

static inline void
lpc31xx_i2s_write(void __iomem *regs, uint32_t reg, uint32_t value)
{
	__raw_writel(value, regs + reg);
}

static inline uint32_t
lpc31xx_i2s_read(void __iomem *regs, uint32_t reg)
{
	uint32_t value;
	value = __raw_readl(regs + reg);
	return value;
}

static void lpc31xx_i2s_shutdown(struct snd_pcm_substream *substream,
									struct snd_soc_dai *dai)
{
	struct lpc31xx_i2s_pair *pair = snd_soc_dai_get_drvdata(dai);
	struct lpc31xx_i2s_channel *chan;

	chan = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? &pair->tx : &pair->rx);

	if (chan->ch_on == 0) {
		/* This channel is not enabled! */
		pr_warning("%s: I2S channel is not on!\n", "fixme");
		return;
	}

	/* Channel specific shutdown */
	lpc31xx_chan_clk_enable(chan->chclk, 0, 0);
	chan->ch_on = 0;

	/* Can we shutdown I2S interface to save some power? */
	if (chan->ch_on != 0) {
		/* Other channel is busy, exit without shutting down main clock */
		return;
	}

	/* Safe to shut down */
	if (pair->initialized == 0) {
		/* Nothing is enabled! */
		pr_warning("I2S shutdown (%s) when nothing is enabled!\n",
				"fixme");
		return;
	}

	/* Disable I2S register access clock */
	cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 0);

	pair->initialized = 0;
}

static int lpc31xx_i2s_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct lpc31xx_i2s_pair *pair = snd_soc_dai_get_drvdata(dai);
	struct lpc31xx_i2s_channel *chan;

	chan = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? &pair->tx : &pair->rx);
	if (chan->ch_on != 0) {
		/* This channel already enabled! */
		pr_warning("%s: I2S channel is busy!\n", "fixme");
		return -EBUSY;
	}

	/* Initialize I2S interface */
	if (pair->initialized == 0) {
		/* Enable I2S register access clock */
		cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 1);
		cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 1);

		pair->initialized = 1;
	}

	/* Channel specific init, ok to leave the clocks off for now */
	chan->ch_on = 1;
	lpc31xx_chan_clk_enable(chan->chclk, 0, 0);

	/* Mask all interrupts for the I2S channel */
	//lpc31xx_i2s_write(info, I2S_CH_INT_MASK(channel->dir_info[dir].i2s_ch), I2S_FIFO_ALL_MASK);

	/* Tx/Rx DMA config */
	snd_soc_dai_set_dma_data(dai, substream, &chan->dma_params);
	return 0;
}

static int lpc31xx_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				      int clk_id, unsigned int freq, int dir)
{
	struct lpc31xx_i2s_pair *pair = snd_soc_dai_get_drvdata(cpu_dai);

	/* Will use in HW params later */
//	pair->dir_info[cpu_dai->id].ws_freq = freq;
	pair->tx.ws_freq = freq;
	pair->rx.ws_freq = freq;
	return 0;
}

static int lpc31xx_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				   unsigned int fmt)
{
	struct lpc31xx_i2s_pair *pair = snd_soc_dai_get_drvdata(cpu_dai);

	/* Will use in HW params later */
//	pair->dir_info[cpu_dai->id].daifmt = fmt;
	pair->tx.daifmt = fmt;
	pair->rx.daifmt = fmt;

	return 0;
}

static int lpc31xx_i2s_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				      int div_id, int div)
{
	/* This function isn't used */
	(void) cpu_dai;
	(void) div_id;
	(void) div;

	return 0;
}

static int lpc31xx_i2s_hw_params(struct snd_pcm_substream *substream,
			         struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai)
{
	struct lpc31xx_i2s_pair *pair = snd_soc_dai_get_drvdata(dai);
	struct lpc31xx_i2s_channel *chan;
	uint32_t tmp;

	chan = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? &pair->tx : &pair->rx);

	/* Setup the I2S data format */
	tmp = 0;
	switch (chan->daifmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			spin_lock_irq(&pair->lock);
			tmp = lpc31xx_i2s_read(lpc31xx_regs, I2S_FORMAT_SETTINGS) &
				~I2S_SET_FORMAT(chan->i2s_ch,
				I2S_FORMAT_MASK);
			lpc31xx_i2s_write(lpc31xx_regs, I2S_FORMAT_SETTINGS, tmp | I2S_SET_FORMAT(chan->i2s_ch,
				I2S_FORMAT_I2S));
			spin_unlock_irq(&pair->lock);
			break;

		default:
			pr_warning("%s: Unsupported audio data format\n", "fixme");
			return -EINVAL;
	}

#if defined (CONFIG_SND_DEBUG_VERBOSE)
	pr_info("Desired clock rate : %d\n", chan->ws_freq);
	pr_info("Channels           : %d\n", params_channels(params));
	pr_info("Data format        : %d\n", chan->daifmt);
#endif

	/* The playback and record rates are shared, so just set the CODEC clock
	   to the selected rate (will actually generate 256 * rate) */
	pair->freq = chan->ws_freq;
	if (lpc31xx_main_clk_rate(pair->freq) == 0)
	{
		pr_warning("Unsupported audio data rate (%d)\n",
			pair->freq);
		return -EINVAL;
	}

	/* Now setup the selected channel clocks (WS and BCK) */
	if (lpc31xx_chan_clk_enable(chan->chclk, pair->freq,
		(pair->freq * 32)) == 0)
	{
		pr_warning("Unsupported channel data rates (ws=%d, bck=%d)\n",
			pair->freq, (pair->freq * 32));
		return -EINVAL;
	}

	return 0;
}

static int lpc31xx_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_RESUME:
		break;

	default:
		pr_warning("lpc31xx_i2s_triggers: Unsupported cmd: %d\n",
				cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifdef CONFIG_PM
static int lpc31xx_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	struct lpc31xx_i2s_pair *pair = snd_soc_dai_get_drvdata(cpu_dai);

	/* Shutdown active clocks */
	if (pair->tx.ch_on != 0) {
		lpc31xx_chan_clk_enable(pair->tx.chclk, 0, 0);
	}
	if (pair->rx.ch_on != 0) {
		lpc31xx_chan_clk_enable(pair->rx.chclk, 0, 0);
	}

	/* Disable I2S register access clock */
	cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 0);
	/* shutdown main clocks */
	lpc31xx_main_clk_rate(0);

	return 0;
}

static int lpc31xx_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	struct lpc31xx_i2s_pair *pair = snd_soc_dai_get_drvdata(cpu_dai);

	/* resume main clocks */
	lpc31xx_main_clk_rate(pair->freq);
	/* Enable I2S register access clock */
	cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 1);

	/* resume active clocks */
	if (pair->tx.ch_on != 0) {
		lpc31xx_chan_clk_enable(pair->tx.chclk,
		pair->tx.ws_freq, (pair->freq * 32));
	}
	if (pair->rx.ch_on != 0) {
		lpc31xx_chan_clk_enable(pair->rx.chclk,
		pair->rx.ws_freq, (pair->freq * 32));
	}

	return 0;
}

#else
#define lpc31xx_i2s_suspend	NULL
#define lpc31xx_i2s_resume	NULL
#endif

static struct snd_soc_dai_ops lpc31xx_i2s_dai_ops = {
	.startup = lpc31xx_i2s_startup,
	.shutdown = lpc31xx_i2s_shutdown,
	.trigger = lpc31xx_i2s_trigger,
	.hw_params = lpc31xx_i2s_hw_params,
	.set_sysclk = lpc31xx_i2s_set_dai_sysclk,
	.set_fmt = lpc31xx_i2s_set_dai_fmt,
	.set_clkdiv = lpc31xx_i2s_set_dai_clkdiv,
};

static struct snd_soc_dai_driver dai_driver = {
	.name = "is2",
	.id = 1,
	.suspend = lpc31xx_i2s_suspend,
	.resume = lpc31xx_i2s_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = LPC31XX_I2S_RATES,
		.formats = LPC31XX_I2S_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = LPC31XX_I2S_RATES,
		.formats = LPC31XX_I2S_FORMATS,
	},
	.ops = &lpc31xx_i2s_dai_ops,
	.symmetric_rates = 1,
};

static struct device i2s[2];

static __devinit int lpc31xx_i2s_dev_probe(struct platform_device *pdev)
{
	struct lpc31xx_i2s_pair *pair;
	struct device_node *child;
	const __be32 *prop;
	struct resource *res;
	int i, err, len, tx, rx, slave;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENODEV;
		goto fail;
	}

	lpc31xx_mem = request_mem_region(res->start, resource_size(res), pdev->name);
	lpc31xx_regs = ioremap(res->start, resource_size(res));
	if (!lpc31xx_regs) {
		err = -ENXIO;
		goto fail_release_mem;
	}

	/* Workaround for ASOC not supporting two independent DAIs
	 * co-mingled onto a single device.
	 */
	i = 0;
	for_each_child_of_node(pdev->dev.of_node, child) {
		/* Device address */
		prop = of_get_property(child, "reg", &len);
		if (!prop || len < 2*sizeof(*prop)) {
			dev_err(&pdev->dev, "%s has no 'reg' property\n",
				child->full_name);
			continue;
		}
		tx = be32_to_cpu(*prop);
		rx = be32_to_cpu(*(prop+1));
		if (!(((tx == 0x80) && (rx == 0x180)) || ((tx == 0x100) && (rx == 0x200)))) {
			dev_err(&pdev->dev, "%s valid regs 0x80/0x180 or 0x100/0x200\n",
				child->full_name);
			continue;
		}
		prop = of_get_property(child, "receive-slave", NULL);
		slave = lpc31xx_i2s_read(lpc31xx_regs, I2S_CFG_MUX_SETTINGS);
		if (prop)
			slave &= ~(tx == 0x80 ? I2S_RXO_SELECT_MASTER : I2S_RX1_SELECT_MASTER);
		else
			slave |= (tx == 0x80 ? I2S_RXO_SELECT_MASTER : I2S_RX1_SELECT_MASTER);
		lpc31xx_i2s_write(lpc31xx_regs, I2S_CFG_MUX_SETTINGS, slave);

		pair = kzalloc(sizeof(*pair), GFP_KERNEL);
		if (pair == NULL) {
			err = ENOMEM;
			goto fail_unmap_mem;
		}
		spin_lock_init(&pair->lock);
		i2s[i] = pdev->dev;
		i2s[i].of_node = child;

		if (tx == 0x80) {
			pair->tx.cfg = res->start + 0x080;
			pair->tx.dma_addr = res->start + 0x0E0;
			pair->tx.dma_params.port = 0;
			pair->rx.cfg = res->start + 0x180;
			pair->rx.dma_addr = res->start + 0x1E0;
		} else {
			pair->tx.cfg = res->start + 0x100;
			pair->tx.dma_addr = res->start + 0x160;
			pair->rx.cfg = res->start + 0x200;
			pair->rx.dma_addr = res->start + 0x260;
		}
		dev_set_drvdata(&i2s[i], pair);

		err = snd_soc_register_dai(&i2s[i], &dai_driver);
		if (err)
			goto fail_unmap_mem;
		i++;
	}
	return 0;

fail_unmap_mem:
	iounmap(lpc31xx_regs);
fail_release_mem:
	if (lpc31xx_mem) {
		release_mem_region(lpc31xx_mem->start, resource_size(lpc31xx_mem));
		lpc31xx_mem = 0;
	}
fail:
	return err;
}

static __devexit int lpc31xx_i2s_dev_remove(struct platform_device *pdev)
{
	struct lpc31xx_i2s_pair *pair;

	snd_soc_unregister_dai(&i2s[0]);
	pair = dev_get_drvdata(&i2s[0]);
	kfree(pair);

	snd_soc_unregister_dai(&i2s[1]);
	pair = dev_get_drvdata(&i2s[1]);
	kfree(pair);

	iounmap(lpc31xx_regs);
	if (lpc31xx_mem)
		release_mem_region(lpc31xx_mem->start, resource_size(lpc31xx_mem));
	return 0;
}

static const struct of_device_id lpc31xx_i2s_of_match[] = {
	{ .compatible = "nxp,lpc31xx-i2s" },
	{},
};
MODULE_DEVICE_TABLE(of, lpc31xx_i2s_of_match);

static struct platform_driver lpc31xx_i2s_driver = {
	.probe  = lpc31xx_i2s_dev_probe,
	.remove = lpc31xx_i2s_dev_remove,
	.driver = {
		.name = "lpc31xx-i2s",
		.owner = THIS_MODULE,
		.of_match_table = lpc31xx_i2s_of_match,
	},
};

static int __init lpc31xx_i2s_init(void)
{
	return platform_driver_register(&lpc31xx_i2s_driver);
}
module_init(lpc31xx_i2s_init);

static void __exit lpc31xx_i2s_exit(void)
{
	platform_driver_unregister(&lpc31xx_i2s_driver);
}
module_exit(lpc31xx_i2s_exit);

/* Module information */
MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("ASoC LPC31XX I2S interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lpc31xx-i2s");

