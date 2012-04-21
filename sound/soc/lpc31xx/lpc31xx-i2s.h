/*
 * sound/soc/lpc31xx/lpc31xx-i2s.h
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

#ifndef __SOUND_SOC_LPC31XX_I2S_H
#define __SOUND_SOC_LPC31XX_I2S_H

#include <linux/types.h>

extern struct snd_soc_dai_driver lpc31xx_i2s_dai;

struct lpc31xx_i2s_res_info {
	struct resource			*mem;
	void __iomem			*regs;
};

/***********************************************************************
* Audio Subsystem (ADSS) register definitions
**********************************************************************/

/* I2S Controller Module Register Structure */
#define I2S_FORMAT_SETTINGS       0x00
#define I2S_CFG_MUX_SETTINGS      0x04
#define I2S_CLASSED_CFG           0x08
#define I2S_CLASSED_STS           0x0C
#define I2S_N_SOF_COUNTER         0x10

/* I2S channels */
#define I2S_CH_TX0 1
#define I2S_CH_TX1 2
#define I2S_CH_RX0 3
#define I2S_CH_RX1 4

/* I2S channel specific registers */
#define I2S_CH_LEFT_16BIT(n)      (((n) * 0x80) + 0x00)
#define I2S_CH_RIGHT_16BIT(n)     (((n) * 0x80) + 0x04)
#define I2S_CH_LEFT_24BIT(n)      (((n) * 0x80) + 0x08)
#define I2S_CH_RIGHT_24BIT(n)     (((n) * 0x80) + 0x0C)
#define I2S_CH_INT_STATUS(n)      (((n) * 0x80) + 0x10)
#define I2S_CH_INT_MASK(n)        (((n) * 0x80) + 0x14)
#define I2S_CH_LEFT32(n)          (((n) * 0x80) + 0x20)
#define I2S_CH_RIGHT32(n)         (((n) * 0x80) + 0x40)
#define I2S_CH_INTERLEAVED(n)     (((n) * 0x80) + 0x60)

/* I2S format settings register defines */
#define I2S_FORMAT_I2S            0x3
#define I2S_FORMAT_LSB16          0x4
#define I2S_FORMAT_LSB18          0x5
#define I2S_FORMAT_LSB20          0x6
#define I2S_FORMAT_LSB24          0x7
#define I2S_FORMAT_MASK           0x7
#define I2S_SET_FORMAT(n, s)      ((s) << (((n) - 1) * 3))

/* I2S Mux configuration setting defines */
#define I2S_RXO_SELECT_MASTER     _BIT(1)
#define I2S_RX1_SELECT_MASTER     _BIT(2)

/* I2S interrupt status and mask bits */
#define I2S_FIFO_RIGHT_UNDERRUN   _BIT(0)
#define I2S_FIFO_LEFT_UNDERRUN    _BIT(1)
#define I2S_FIFO_RIGHT_OVERRUN    _BIT(2)
#define I2S_FIFO_LEFT_OVERRUN     _BIT(3)
#define I2S_FIFO_LEFT_FULL        _BIT(4)
#define I2S_FIFO_LEFT_HALF_FULL   _BIT(5) /* RX only */
#define I2S_FIFO_LEFT_HALF_EMPTY  _BIT(5) /* TX only */
#define I2S_FIFO_LEFT_NOT_EMPTY   _BIT(6) /* RX only */
#define I2S_FIFO_LEFT_EMPTY       _BIT(6) /* TX only */
#define I2S_FIFO_RIGHT_FULL       _BIT(7)
#define I2S_FIFO_RIGHT_HALF_FULL  _BIT(8) /* RX only */
#define I2S_FIFO_RIGHT_HALF_EMPTY _BIT(8) /* TX only */
#define I2S_FIFO_RIGHT_NOT_EMPTY  _BIT(9) /* RX only */
#define I2S_FIFO_RIGHT_EMPTY      _BIT(9) /* TX only */
#define I2S_FIFO_ALL_MASK         0x3FF



#endif

