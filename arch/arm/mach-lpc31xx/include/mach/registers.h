/* linux/arch/arm/mach-lpc31xx/include/mach/registers.h
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Register defines for LPC31xx SoCs.
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

#ifndef __ASM_ARCH_REGISTERS_H
#define __ASM_ARCH_REGISTERS_H


/***********************************************************************
 * Interrupt controller register definitions
 **********************************************************************/
#define INTC_IRQ_PRI_MASK     __REG(INTC_PHYS + 0x000)
#define INTC_FIQ_PRI_MASK     __REG(INTC_PHYS + 0x004)
#define INTC_IRQ_VEC_BASE     __REG(INTC_PHYS + 0x100)
#define INTC_FIQ_VEC_BASE     __REG(INTC_PHYS + 0x104)
#define INTC_REQ_REG(irq)     __REG(INTC_PHYS + 0x400 + ((irq) << 2))

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

/***********************************************************************
 * Event router register definitions
 **********************************************************************/
#define EVRT_INT_PEND(bank)  __REG (EVTR_PHYS + 0xC00 + ((bank) << 2))
#define EVRT_INT_CLR(bank)   __REG (EVTR_PHYS + 0xC20 + ((bank) << 2))
#define EVRT_INT_SET(bank)   __REG (EVTR_PHYS + 0xC40 + ((bank) << 2))
#define EVRT_MASK(bank)      __REG (EVTR_PHYS + 0xC60 + ((bank) << 2))
#define EVRT_MASK_CLR(bank)  __REG (EVTR_PHYS + 0xC80 + ((bank) << 2))
#define EVRT_MASK_SET(bank)  __REG (EVTR_PHYS + 0xCA0 + ((bank) << 2))
#define EVRT_APR(bank)       __REG (EVTR_PHYS + 0xCC0 + ((bank) << 2))
#define EVRT_ATR(bank)       __REG (EVTR_PHYS + 0xCE0 + ((bank) << 2))
#define EVRT_RSR(bank)       __REG (EVTR_PHYS + 0xD20 + ((bank) << 2))
#define EVRT_OUT_PEND(vec,bank)     __REG (EVTR_PHYS + 0x1000 + ((vec) << 5) + ((bank) << 2))
#define EVRT_OUT_MASK(vec,bank)     __REG (EVTR_PHYS + 0x1400 + ((vec) << 5) + ((bank) << 2))
#define EVRT_OUT_MASK_CLR(vec,bank) __REG (EVTR_PHYS + 0x1800 + ((vec) << 5) + ((bank) << 2))
#define EVRT_OUT_MASK_SET(vec,bank) __REG (EVTR_PHYS + 0x1C00 + ((vec) << 5) + ((bank) << 2))

/***********************************************************************
 * WDT register definitions
 **********************************************************************/
#define WDT_IR       __REG (WDT_PHYS + 0x00)
#define WDT_TCR      __REG (WDT_PHYS + 0x04)
#define WDT_TC       __REG (WDT_PHYS + 0x08)
#define WDT_PR       __REG (WDT_PHYS + 0x0c)
#define WDT_MCR      __REG (WDT_PHYS + 0x14)
#define WDT_MR0      __REG (WDT_PHYS + 0x18)
#define WDT_MR1      __REG (WDT_PHYS + 0x1c)
#define WDT_EMR      __REG (WDT_PHYS + 0x3c)

#define WDT_IR_MR1        _BIT(1)
#define WDT_IR_MR0        _BIT(0)
#define WDT_TCR_CNT_RESET _BIT(1)
#define WDT_TCR_CNT_EN    _BIT(0)
#define WDT_MCR_STOP_MR1  _BIT(5)
#define WDT_MCR_RESET_MR1 _BIT(4)
#define WDT_MCR_INT_MR1   _BIT(3)
#define WDT_MCR_STOP_MR0  _BIT(2)
#define WDT_MCR_RESET_MR0 _BIT(1)
#define WDT_MCR_INT_MR0   _BIT(0)
#define WDT_EMR_CTRL0(n)  _SBF(4,((n) &0x3))
#define WDT_EMR_CTRL1(n)  _SBF(6,((n) &0x3))
#define WDT_EMR_M1        _BIT(1)
#define WDT_EMR_M0        _BIT(0)

/***********************************************************************
 * Timer register definitions
 **********************************************************************/
#define TIMER_LOAD(base)      __REG ((base) + 0x00)
#define TIMER_VALUE(base)     __REG ((base) + 0x04)
#define TIMER_CONTROL(base)   __REG ((base) + 0x08)
#define TIMER_CLEAR(base)     __REG ((base) + 0x0c)

#define TM_CTRL_ENABLE    _BIT(7)
#define TM_CTRL_MODE      _BIT(6)
#define TM_CTRL_PERIODIC  _BIT(6)
#define TM_CTRL_PS1       _SBF(2, 0)
#define TM_CTRL_PS16      _SBF(2, 1)
#define TM_CTRL_PS256     _SBF(2, 2)
#define TM_CTRL_PS_MASK   _SBF(2, 0x3)

/***********************************************************************
 * UART register definitions
 **********************************************************************/
#define UART_DLL_REG      __REG (UART_PHYS + 0x00)
#define UART_FIFO_REG     __REG (UART_PHYS + 0x00)
#define UART_IE_REG       __REG (UART_PHYS + 0x04)
#define UART_DLM_REG      __REG (UART_PHYS + 0x04)
#define UART_IIR_REG      __REG (UART_PHYS + 0x08)
#define UART_FCR_REG      __REG (UART_PHYS + 0x08)
#define UART_LCR_REG      __REG (UART_PHYS + 0x0c)
#define UART_MCR_REG      __REG (UART_PHYS + 0x10)
#define UART_LSR_REG      __REG (UART_PHYS + 0x14)
#define UART_MSR_REG      __REG (UART_PHYS + 0x18)
#define UART_SCR_REG      __REG (UART_PHYS + 0x1c)
#define UART_ACR_REG      __REG (UART_PHYS + 0x20)
#define UART_ICR_REG      __REG (UART_PHYS + 0x24)
#define UART_FDR_REG      __REG (UART_PHYS + 0x28)

#if 0
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
#endif

/***********************************************************************
 * ADC_REG register definitions
 **********************************************************************/
#define ADC_CON_REG            __REG (ADC_PHYS + 0x20)
#define ADC_R0_REG             __REG (ADC_PHYS + 0x00)
#define ADC_R1_REG             __REG (ADC_PHYS + 0x04)
#define ADC_R2_REG             __REG (ADC_PHYS + 0x08)
#define ADC_R3_REG             __REG (ADC_PHYS + 0x0C)
#define ADC_CSEL_REG           __REG (ADC_PHYS + 0x24)
#define ADC_INT_ENABLE_REG     __REG (ADC_PHYS + 0x28)
#define ADC_INT_STATUS_REG     __REG (ADC_PHYS + 0x2C)
#define ADC_INT_CLEAR_REG      __REG (ADC_PHYS + 0x30)

/***********************************************************************
 * SYS_REG register definitions
 **********************************************************************/
#define SYS_RNG_OSC_CFG        __REG (SYS_PHYS + 0x14)
#define SYS_ADC_PD             __REG (SYS_PHYS + 0x18)
#define SYS_SDMMC_DELAYMODES   __REG (SYS_PHYS + 0x2C)
#define SYS_USB_ATX_PLL_PD_REG __REG (SYS_PHYS + 0x30)
#define SYS_USB_OTG_CFG        __REG (SYS_PHYS + 0x34)
#define SYS_USB_OTG_LED_CTL    __REG (SYS_PHYS + 0x38)
#define SYS_USB_PLL_NDEC       __REG (SYS_PHYS + 0x40)
#define SYS_USB_PLL_MDEC       __REG (SYS_PHYS + 0x44)
#define SYS_USB_PLL_PDEC       __REG (SYS_PHYS + 0x48)
#define SYS_USB_PLL_SELR       __REG (SYS_PHYS + 0x4C)
#define SYS_USB_PLL_SELI       __REG (SYS_PHYS + 0x50)
#define SYS_USB_PLL_SELP       __REG (SYS_PHYS + 0x54)

#define SYS_MPMC_DELAY      __REG (SYS_PHYS + 0x68)
#define SYS_MPMC_WTD_DEL0   __REG (SYS_PHYS + 0x6C)
#define SYS_MPMC_WTD_DEL1   __REG (SYS_PHYS + 0x70)
#define SYS_MPMC_TESTMODE0  __REG (SYS_PHYS + 0x78)
#define SYS_REMAP_ADDR      __REG (SYS_PHYS + 0x84)
#define SYS_MUX_LCD_EBI     __REG (SYS_PHYS + 0x90)
#define SYS_MUX_GPIO_MCI    __REG (SYS_PHYS + 0x94)
#define SYS_MUX_NAND_MCI    __REG (SYS_PHYS + 0x98)

/***********************************************************************
 * GPIO register definitions
 **********************************************************************/
#if 0
#define GPIO_STATE(port)     __REG (GPIO_PHYS + (port) + 0x00)
#define GPIO_STATE_M0(port)  __REG (GPIO_PHYS + (port) + 0x10)
#define GPIO_M0_SET(port)    __REG (GPIO_PHYS + (port) + 0x14)
#define GPIO_M0_RESET(port)  __REG (GPIO_PHYS + (port) + 0x18)
#define GPIO_STATE_M1(port)  __REG (GPIO_PHYS + (port) + 0x20)
#define GPIO_M1_SET(port)    __REG (GPIO_PHYS + (port) + 0x24)
#define GPIO_M1_RESET(port)  __REG (GPIO_PHYS + (port) + 0x28)
#endif

#define GPIO_OUT_LOW(port, pin)  do { GPIO_M1_SET(port) = pin; GPIO_M0_RESET(port) = pin;} while(0)
#define GPIO_OUT_HIGH(port, pin) do { GPIO_M1_SET(port) = pin; GPIO_M0_SET(port) = pin;} while(0)
#define GPIO_IN(port, pin)       do { GPIO_M1_RESET(port) = pin; GPIO_M0_RESET(port) = pin;} while(0)
#define GPIO_DRV_IP(port, pin)   do { GPIO_M1_RESET(port) = pin; GPIO_M0_SET(port) = pin;} while(0)

#define IOCONF_EBI_MCI       (0x000)
#define IOCONF_EBI_I2STX_0   (0x040)
#define IOCONF_CGU           (0x080)
#define IOCONF_I2SRX_0       (0x0c0)
#define IOCONF_I2SRX_1       (0x100)
#define IOCONF_I2STX_1       (0x140)
#define IOCONF_EBI           (0x180)
#define IOCONF_GPIO          (0x1c0)
#define IOCONF_I2C1          (0x200)
#define IOCONF_SPI           (0x240)
#define IOCONF_NAND_CTRL     (0x280)
#define IOCONF_PWM           (0x2c0)
#define IOCONF_UART          (0x300)


/***********************************************************************
 * MPMC memory controller register definitions
 **********************************************************************/
#define MPMC_CTRL           __REG (MPMC_PHYS + 0x000)
#define MPMC_STATUS         __REG (MPMC_PHYS + 0x004)
#define MPMC_CONFIG         __REG (MPMC_PHYS + 0x008)
#define MPMC_DYNCTL         __REG (MPMC_PHYS + 0x020)
#define MPMC_DYNREF         __REG (MPMC_PHYS + 0x024)
#define MPMC_DYRDCFG        __REG (MPMC_PHYS + 0x028)
#define MPMC_DYTRP          __REG (MPMC_PHYS + 0x030)
#define MPMC_DYTRAS         __REG (MPMC_PHYS + 0x034)
#define MPMC_DYTSREX        __REG (MPMC_PHYS + 0x038)
#define MPMC_DYTAPR         __REG (MPMC_PHYS + 0x03C)
#define MPMC_DYTDAL         __REG (MPMC_PHYS + 0x040)
#define MPMC_DYTWR          __REG (MPMC_PHYS + 0x044)
#define MPMC_DYTRC          __REG (MPMC_PHYS + 0x048)
#define MPMC_DYTRFC         __REG (MPMC_PHYS + 0x04C)
#define MPMC_DYTXSR         __REG (MPMC_PHYS + 0x050)
#define MPMC_DYTRRD         __REG (MPMC_PHYS + 0x054)
#define MPMC_DYTMRD         __REG (MPMC_PHYS + 0x058)
#define MPMC_STEXDWT        __REG (MPMC_PHYS + 0x080)
#define MPMC_DYCONFIG       __REG (MPMC_PHYS + 0x100)
#define MPMC_DYRASCAS       __REG (MPMC_PHYS + 0x104)
#define MPMC_STCONFIG0      __REG (MPMC_PHYS + 0x200)
#define MPMC_STWTWEN0       __REG (MPMC_PHYS + 0x204)
#define MPMC_STWTOEN0       __REG (MPMC_PHYS + 0x208)
#define MPMC_STWTRD0        __REG (MPMC_PHYS + 0x20C)
#define MPMC_STWTPG0        __REG (MPMC_PHYS + 0x210)
#define MPMC_STWTWR0        __REG (MPMC_PHYS + 0x214)
#define MPMC_STWTTURN0      __REG (MPMC_PHYS + 0x218)
#define MPMC_STCONFIG1      __REG (MPMC_PHYS + 0x220)
#define MPMC_STWTWEN1       __REG (MPMC_PHYS + 0x224)
#define MPMC_STWTOEN1       __REG (MPMC_PHYS + 0x228)
#define MPMC_STWTRD1        __REG (MPMC_PHYS + 0x22C)
#define MPMC_STWTPG1        __REG (MPMC_PHYS + 0x230)
#define MPMC_STWTWR1        __REG (MPMC_PHYS + 0x234)
#define MPMC_STWTTURN1      __REG (MPMC_PHYS + 0x238)

#define NS_TO_MPMCCLK(ns, clk)	(((ns)*((clk + 500)/1000) + 500000)/1000000)
/* MPMC Controller Bit Field constants*/
#define MPMC_CTL_LOW_PWR               _BIT(2)
#define MPMC_CTL_ENABLE                _BIT(0)
/* MPMC status Bit Field constants*/
#define MPMC_STATUS_SA                 _BIT(2)
#define MPMC_STATUS_WR_BUF             _BIT(1)
#define MPMC_STATUS_BUSY               _BIT(0)
/* MPMC config Bit Field constants*/
#define MPMC_CFG_SDCCLK_1_2            _BIT(8)
#define MPMC_CFG_SDCCLK_1_1            (0)
/* SDRAM Controller Bit Field constants*/
#define MPMC_SDRAMC_CTL_DP             _BIT(13)
#define MPMC_SDRAMC_CTL_NORMAL_CMD     _SBF(7,0)
#define MPMC_SDRAMC_CTL_MODE_CMD       _SBF(7,1)
#define MPMC_SDRAMC_CTL_PALL_CMD       _SBF(7,2)
#define MPMC_SDRAMC_CTL_NOP_CMD        _SBF(7,3)
#define MPMC_SDRAMC_CTL_MCC            _BIT(5)
#define MPMC_SDRAMC_CTL_SR             _BIT(2)
#define MPMC_SDRAMC_CTL_CS             _BIT(1)
#define MPMC_SDRAMC_CTL_CE             _BIT(0)
/* SDRAM Config Bit Field constants*/
#define MPMC_SDRAMC_CFG_SDRAM_MD       _SBF(3,0)
#define MPMC_SDRAMC_CFG_LOW_PWR_MD     _SBF(3,1)
#define MPMC_SDRAMC_CFG_SYNC_FLASH     _SBF(3,2)
#define MPMC_SDRAMC_CFG_BUF_EN         _BIT(19)
#define MPMC_SDRAMC_CFG_WP             _BIT(20)


/* SDRAM Read Config Bit Field constants*/
#define MPMC_SDRAMC_RDCFG_CLKOUTDELAY_STG       _SBF(0,0)
#define MPMC_SDRAMC_RDCFG_CMDDELAY_STG          _SBF(0,1)
#define MPMC_SDRAMC_RDCFG_CMDDELAY_P1_STG       _SBF(0,2)
#define MPMC_SDRAMC_RDCFG_CMDDELAY_P2_STG       _SBF(0,3)
/* SDRAM RASCAS Bit Field constants*/
#define MPMC_SDRAMC_RASCAS_CAS0        _SBF(8,0)
#define MPMC_SDRAMC_RASCAS_CAS1        _SBF(8,1)
#define MPMC_SDRAMC_RASCAS_CAS2        _SBF(8,2)
#define MPMC_SDRAMC_RASCAS_CAS3        _SBF(8,3)
#define MPMC_SDRAMC_RASCAS_RAS0        _SBF(0,0)
#define MPMC_SDRAMC_RASCAS_RAS1        _SBF(0,1)
#define MPMC_SDRAMC_RASCAS_RAS2        _SBF(0,2)
#define MPMC_SDRAMC_RASCAS_RAS3        _SBF(0,3)

#endif  /* __ASM_ARCH_REGISTERS_H */
