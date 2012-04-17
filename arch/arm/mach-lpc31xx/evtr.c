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

#define IRQ_EVT_START   NR_IRQ_CPU

/* System specific IRQs */
/* event PIN or internal signal */
typedef enum _EVT_TYPE_
{
  EVT_ipint_int = 0,
  EVT_mLCD_DB_0,
  EVT_mLCD_DB_1,
  EVT_mLCD_DB_2,
  EVT_mLCD_DB_3,
  EVT_mLCD_DB_4,
  EVT_mLCD_DB_5,
  EVT_mLCD_DB_6,
  EVT_mLCD_DB_7,
  EVT_mLCD_DB_8,
  EVT_mLCD_DB_9,
  EVT_mLCD_DB_10,
  EVT_mLCD_DB_11,
  EVT_mLCD_DB_12,
  EVT_mLCD_DB_13,
  EVT_mLCD_DB_14,
  EVT_mLCD_DB_15,
  EVT_mLCD_RS,
  EVT_mLCD_CSB,
  EVT_mLCD_E_RD,
  EVT_mLCD_RW_WR,
  EVT_mNAND_RYBN0,
  EVT_mNAND_RYBN1,
  EVT_mNAND_RYBN2,
  EVT_mNAND_RYBN3,
  EVT_EBI_D_0,
  EVT_EBI_D_1,
  EVT_EBI_D_2,
  EVT_EBI_D_3,
  EVT_EBI_D_4,
  EVT_EBI_D_5,
  EVT_EBI_D_6,

  EVT_EBI_D_7,
  EVT_EBI_D_8,
  EVT_EBI_D_9,
  EVT_EBI_D_10,
  EVT_EBI_D_11,
  EVT_EBI_D_12,
  EVT_EBI_D_13,
  EVT_EBI_D_14,
  EVT_EBI_D_15,
  EVT_EBI_NWE,
  EVT_EBI_A_0_ALE,
  EVT_EBI_A_1_CLE,
  EVT_EBI_DQM_0_NOE,
  EVT_EBI_NCAS_BLOUT_0,
  EVT_EBI_NRAS_BLOUT_1,
  EVT_GPIO1,
  EVT_GPIO0,
  EVT_GPIO2,
  EVT_GPIO3,
  EVT_GPIO4,
  EVT_mGPIO5,
  EVT_mGPIO6,
  EVT_mGPIO7,
  EVT_mGPIO8,
  EVT_mGPIO9,
  EVT_mGPIO10,
  EVT_GPIO11,
  EVT_GPIO12,
  EVT_GPIO13,
  EVT_GPIO14,
  EVT_GPIO15,
  EVT_GPIO16,

  EVT_GPIO17,
  EVT_GPIO18,
  EVT_NAND_NCS_0,
  EVT_NAND_NCS_1,
  EVT_NAND_NCS_2,
  EVT_NAND_NCS_3,
  EVT_SPI_MISO,
  EVT_SPI_MOSI,
  EVT_SPI_CS_IN,
  EVT_SPI_SCK,
  EVT_SPI_CS_OUT0,
  EVT_UART_RXD,
  EVT_UART_TXD,
  EVT_mUART_CTS_N,
  EVT_mUART_RTS_N,
  EVT_mI2STX_CLK0,
  EVT_mI2STX_BCK0,
  EVT_mI2STX_DATA0,
  EVT_mI2STX_WS0,
  EVT_I2SRX_BCK0,
  EVT_I2SRX_DATA0,
  EVT_I2SRX_WS0,
  EVT_I2SRX_DATA1,
  EVT_I2SRX_BCK1,
  EVT_I2SRX_WS1,
  EVT_I2STX_DATA1,
  EVT_I2STX_BCK1,
  EVT_I2STX_WS1,
  EVT_CLK_256FS_O,
  EVT_I2C_SDA1,
  EVT_I2C_SCL1,
  EVT_PWM_DATA,

  EVT_AD_NINT_I,
  EVT_PLAY_DET_I,
  EVT_timer0_intct1,
  EVT_timer1_intct1,
  EVT_timer2_intct1,
  EVT_timer3_intct1,
  EVT_adc_int,
  EVT_wdog_m0,
  EVT_uart_rxd,
  EVT_i2c0_scl_n,
  EVT_i2c1_scl_n,
  EVT_arm926_nfiq,
  EVT_arm926_nirq,
  EVT_MCI_DAT_0,
  EVT_MCI_DAT_1,
  EVT_MCI_DAT_2,
  EVT_MCI_DAT_3,
  EVT_MCI_DAT_4,
  EVT_MCI_DAT_5,
  EVT_MCI_DAT_6,
  EVT_MCI_DAT_7,
  EVT_MCI_CMD,
  EVT_MCI_CLK,
  EVT_USB_VBUS,
  EVT_usb_otg_ahb_needclk,
  EVT_usb_atx_pll_lock,
  EVT_usb_otg_vbus_pwr_en,
  EVT_USB_ID,
  EVT_isram0_mrc_finished,
  EVT_isram1_mrc_finished,
  EVT_LAST
} EVENT_T;

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

/* structure to map board IRQ to event pin */
typedef struct {
	u32 irq;
	EVENT_T event_pin;
	EVENT_TYPE_T type;
} IRQ_EVENT_MAP_T;

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

/* Other chip IRQs routed through event router.
 * These IRQs should be treated as board IRQs but they are
 * common for all boards.
 */
#define IRQ_WDT        30  /* Watchdog interrupt */
#define IRQ_VBUS_EN    31  /* VBUS power enable */
#define IRQ_VBUS_OVRC  32  /* Detect VBUS over current - Host mode */
#define IRQ_USB_ID     33  /* Detect ID pin change - OTG */

#define _INTERNAL_IRQ_EVENT_MAP	\
	{IRQ_WDT, EVT_wdog_m0, EVT_RISING_EDGE}, \
	{IRQ_VBUS_EN, EVT_usb_otg_vbus_pwr_en, EVT_FALLING_EDGE}, \
	{IRQ_VBUS_OVRC, EVT_USB_VBUS, EVT_FALLING_EDGE}, \
	{IRQ_USB_ID, EVT_USB_ID, EVT_ACTIVE_LOW}, \

#if defined(CONFIG_LPC3152_AD)
/* For chips with analog die there are some more AD events routed
 * through event router.
 */
#define IRQ_RTC	        34
#define IRQ_PLAY        35
#define NR_IRQ_CHIP_EVT	6

#define AD_IRQ_EVENT_MAP	\
	{IRQ_RTC, EVT_AD_NINT_I, EVT_ACTIVE_LOW}, \
	{IRQ_PLAY, EVT_PLAY_DET_I, EVT_ACTIVE_HIGH}, \

#define CHIP_IRQ_EVENT_MAP   _INTERNAL_IRQ_EVENT_MAP \
	AD_IRQ_EVENT_MAP

#else
#define CHIP_IRQ_EVENT_MAP   _INTERNAL_IRQ_EVENT_MAP
#define NR_IRQ_CHIP_EVT	     4
#endif

/* now compute the board start IRQ number */
#define IRQ_BOARD_START   (NR_IRQ_CPU + NR_IRQ_CHIP_EVT)

/* Route all internal chip events to IRQ_EVT_ROUTER0 */
#define IRQ_EVTR0_START        IRQ_EVT_START
#define IRQ_EVTR0_END          (IRQ_BOARD_START - 1)


#if defined (CONFIG_MACH_VAL3153)

# define IRQ_CS8900_ETH_INT  IRQ_BOARD_START	/* Ethernet chip */
# define IRQ_SDMMC_CD0       (IRQ_BOARD_START + 1)	/* SD card detect */
# define IRQ_SDMMC_CD1       (IRQ_BOARD_START + 2)	/* SD card detect */
# define NR_IRQ_EBOARD        3

/* now define board irq to event pin map */
#define BOARD_IRQ_EVENT_MAP	{ \
	CHIP_IRQ_EVENT_MAP \
	{IRQ_CS8900_ETH_INT, EVT_GPIO16, EVT_ACTIVE_HIGH}, \
	{IRQ_SDMMC_CD0, EVT_GPIO12, EVT_ACTIVE_HIGH}, \
	{IRQ_SDMMC_CD1, EVT_GPIO13, EVT_ACTIVE_HIGH}, \
	}
/* Following defines group the board IRQs into 4 IRQ_EVNTR groups.
   IRQ_EVT_ROUTERx IRQ is generated when event in the corresponding
   group triggers.
*/
#define IRQ_EVTR1_START        IRQ_CS8900_ETH_INT
#define IRQ_EVTR1_END          IRQ_CS8900_ETH_INT
#define IRQ_EVTR2_START        IRQ_SDMMC_CD0
#define IRQ_EVTR2_END          IRQ_SDMMC_CD0
#define IRQ_EVTR3_START        IRQ_SDMMC_CD1
#define IRQ_EVTR3_END          IRQ_SDMMC_CD1


#elif defined (CONFIG_MACH_EA313X) || defined(CONFIG_MACH_EA3152)
# define IRQ_DM9000_ETH_INT   IRQ_BOARD_START	/* Ethernet chip */
# define IRQ_SDMMC_CD         (IRQ_BOARD_START + 1)	/* SD card detect */
# define IRQ_EA_VBUS_OVRC     (IRQ_BOARD_START + 2)	/* Over current indicator */
# define IRQ_PENDOWN	      (IRQ_BOARD_START + 3)	/* Pendown from touch screen */
# define NR_IRQ_EBOARD        4

/* now define board irq to event pin map */
#define BOARD_IRQ_EVENT_MAP	{ \
	CHIP_IRQ_EVENT_MAP \
	{IRQ_DM9000_ETH_INT, EVT_mNAND_RYBN3, EVT_ACTIVE_HIGH}, \
	{IRQ_SDMMC_CD, EVT_mI2STX_BCK0, EVT_ACTIVE_LOW}, \
	{IRQ_EA_VBUS_OVRC, EVT_I2SRX_WS0, EVT_ACTIVE_LOW}, \
	{IRQ_PENDOWN, EVT_GPIO4, EVT_ACTIVE_HIGH}, \
	}
/* Following defines group the board IRQs into 4 IRQ_EVNTR groups.
   IRQ_EVT_ROUTERx IRQ is generated when event in the corresponding
   group triggers.
*/
#define IRQ_EVTR1_START        IRQ_DM9000_ETH_INT
#define IRQ_EVTR1_END          IRQ_DM9000_ETH_INT
#define IRQ_EVTR2_START        IRQ_SDMMC_CD
#define IRQ_EVTR2_END          IRQ_SDMMC_CD
#define IRQ_EVTR3_START        IRQ_EA_VBUS_OVRC
#define IRQ_EVTR3_END          IRQ_PENDOWN

#elif defined (CONFIG_MACH_VAL3154)
# define IRQ_SDMMC_CD	 IRQ_BOARD_START 	/* SD card detect */
# define NR_IRQ_EBOARD	 1

/* now define board irq to event pin map */
#define BOARD_IRQ_EVENT_MAP	{ \
	CHIP_IRQ_EVENT_MAP \
	{IRQ_SDMMC_CD, EVT_mI2STX_BCK0, EVT_ACTIVE_LOW}, \
	}
/* Following defines group the board IRQs into 4 IRQ_EVNTR groups.
   IRQ_EVT_ROUTERx IRQ is generated when event in the corresponding
   group triggers.
*/
#define IRQ_EVTR1_START        IRQ_SDMMC_CD
#define IRQ_EVTR1_END          IRQ_SDMMC_CD
#define IRQ_EVTR2_START        0
#define IRQ_EVTR2_END          0
#define IRQ_EVTR3_START        0
#define IRQ_EVTR3_END          0

#else
# define NR_IRQ_EBOARD          0
#define IRQ_EVTR0_START        0
#define IRQ_EVTR0_END          0
#define IRQ_EVTR1_START        0
#define IRQ_EVTR1_END          0
#define IRQ_EVTR2_START        0
#define IRQ_EVTR2_END          0
#define IRQ_EVTR3_START        0
#define IRQ_EVTR3_END          0

#endif


static IRQ_EVENT_MAP_T irq_2_event[] = BOARD_IRQ_EVENT_MAP;

static void evt_mask_irq(struct irq_data *data)
{
	u32 bank = EVT_GET_BANK(irq_2_event[data->irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[data->irq - IRQ_EVT_START].event_pin & 0x1F;

	EVRT_MASK_CLR(bank) = _BIT(bit_pos);
}

static void evt_unmask_irq(struct irq_data *data)
{
	u32 bank = EVT_GET_BANK(irq_2_event[data->irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[data->irq - IRQ_EVT_START].event_pin & 0x1F;

	EVRT_MASK_SET(bank) = _BIT(bit_pos);
}

static void evt_ack_irq(struct irq_data *data)
{
	u32 bank = EVT_GET_BANK(irq_2_event[data->irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[data->irq - IRQ_EVT_START].event_pin & 0x1F;
	//EVRT_MASK_CLR(bank) = _BIT(bit_pos);
	EVRT_INT_CLR(bank) = _BIT(bit_pos);
}

static int evt_set_type(struct irq_data *data, unsigned int flow_type)
{
	u32 bank = EVT_GET_BANK(irq_2_event[data->irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[data->irq - IRQ_EVT_START].event_pin & 0x1F;

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
	u32 bank = EVT_GET_BANK(irq_2_event[data->irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[data->irq - IRQ_EVT_START].event_pin & 0x1F;

	if (on)
		/* enable routing to CGU_WAKEUP */
		EVRT_OUT_MASK_SET(4, bank) = _BIT(bit_pos);
	else
		/* disable routing to CGU_WAKEUP */
		EVRT_OUT_MASK_CLR(4, bank) = _BIT(bit_pos);

	return 0;
}

static int set_input(unsigned irq);

unsigned int evt_startup(struct irq_data *data)
{
	evt_unmask_irq(data);
	return set_input(data->irq);
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
		u32 status, bank, bit_pos; \
		if (IRQ_EVTR##n##_START == IRQ_EVTR##n##_END) { \
			/* translate IRQ number */ \
			irq = IRQ_EVTR##n##_START; \
			generic_handle_irq(irq); \
		} else { \
			for (irq = IRQ_EVTR##n##_START; irq <= IRQ_EVTR##n##_END; irq++) {  \
				/* compute bank & bit position for the event_pin */ \
				bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin); \
				bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F; \
				status = EVRT_OUT_PEND(n, bank); \
				if (status & _BIT(bit_pos)) { \
					generic_handle_irq(irq); \
				} \
			} \
		} \
	}


#if IRQ_EVTR0_END
ROUTER_HDLR(0)
#endif /* IRQ_EVTR0_END */

#if IRQ_EVTR1_END
ROUTER_HDLR(1)
#endif /* IRQ_EVTR1_END */

#if IRQ_EVTR2_END
ROUTER_HDLR(2)
#endif /* IRQ_EVTR2_END */

#if IRQ_EVTR3_END
ROUTER_HDLR(3)
#endif /* IRQ_EVTR3_END */

void __init lpc31xx_init_evtr(void)
{
	unsigned int irq;
	int i, j, v;
	u32 bank, bit_pos;

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

	/* Now configure external/board interrupts using event router */
	for (irq = IRQ_EVT_START; irq < IRQ_EVT_START + NR_IRQ_CHIP_EVT + NR_IRQ_EBOARD; irq++) {
		/* compute bank & bit position for the event_pin */
		bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin);
		bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F;

		irq_set_chip(irq, &lpc31xx_evtr_chip);
		set_irq_flags(irq, IRQF_VALID);
		/* configure the interrupt sensitivity */
		switch (irq_2_event[irq - IRQ_EVT_START].type) {
			case EVT_ACTIVE_LOW:
				EVRT_APR(bank) &= ~_BIT(bit_pos);
				EVRT_ATR(bank) &= ~_BIT(bit_pos);
				irq_set_handler(irq, handle_level_irq);
				break;
			case EVT_ACTIVE_HIGH:
				EVRT_APR(bank) |= _BIT(bit_pos);
				EVRT_ATR(bank) &= ~_BIT(bit_pos);
				irq_set_handler(irq, handle_level_irq);
				break;
			case EVT_FALLING_EDGE:
				EVRT_APR(bank) &= ~_BIT(bit_pos);
				EVRT_ATR(bank) |= _BIT(bit_pos);
				irq_set_handler(irq, handle_edge_irq);
				break;
			case EVT_RISING_EDGE:
				EVRT_APR(bank) |= _BIT(bit_pos);
				EVRT_ATR(bank) |= _BIT(bit_pos);
				irq_set_handler(irq, handle_edge_irq);
				break;
			case EVT_BOTH_EDGE:
				EVRT_ATR(bank) |= _BIT(bit_pos);
				irq_set_handler(irq, handle_edge_irq);
				break;
			default:
				printk("Invalid Event type.\r\n");
				break;
		}
		if ( (irq >= IRQ_EVTR0_START) && (irq <= IRQ_EVTR0_END) ) {
			/* enable routing to vector 0 */
			EVRT_OUT_MASK_SET(0, bank) = _BIT(bit_pos);
			v = 0;
		} else if ( (irq >= IRQ_EVTR1_START) && (irq <= IRQ_EVTR1_END) ) {
			/* enable routing to vector 1 */
			EVRT_OUT_MASK_SET(1, bank) = _BIT(bit_pos);
			v = 1;
		} else if ( (irq >= IRQ_EVTR2_START) && (irq <= IRQ_EVTR2_END) ) {
			/* enable routing to vector 2 */
			EVRT_OUT_MASK_SET(2, bank) = _BIT(bit_pos);
			v = 2;
		} else if ( (irq >= IRQ_EVTR3_START) && (irq <= IRQ_EVTR3_END) ) {
			/* enable routing to vector 3 */
			EVRT_OUT_MASK_SET(3, bank) = _BIT(bit_pos);
			v = 3;
		} else {
			v = -1;
			printk("Invalid Event router setup.\r\n");
		}
		printk("irq=%d Event=0x%02x bank:%d bit:%02d type:%d vector %d\n", irq,
			irq_2_event[irq - IRQ_EVT_START].event_pin, bank,
			bit_pos, irq_2_event[irq - IRQ_EVT_START].type, v);

	}
	/* for power management. Wake from internal irqs */
	EVRT_APR(3) &= ~_BIT(12);
	EVRT_ATR(3) &= ~_BIT(12);
	EVRT_MASK_SET(3) = _BIT(12);

	/* install IRQ_EVT_ROUTER0  chain handler */
#if IRQ_EVTR0_END
	/* install chain handler for IRQ_EVT_ROUTER0 */
	irq_set_chained_handler (IRQ_EVT_ROUTER0, router0_handler);
#endif

#if IRQ_EVTR1_END
	/* install chain handler for IRQ_EVT_ROUTER1 */
	irq_set_chained_handler (IRQ_EVT_ROUTER1, router1_handler);
#endif

#if IRQ_EVTR2_END
	/* install chain handler for IRQ_EVT_ROUTER2 */
	irq_set_chained_handler (IRQ_EVT_ROUTER2, router2_handler);
#endif

#if IRQ_EVTR3_END
	/* install chain handler for IRQ_EVT_ROUTER3 */
	irq_set_chained_handler (IRQ_EVT_ROUTER3, router3_handler);
#endif
}

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

struct event_data {
	int event;
	int group;
	int edge;
};

static struct event_data *events;
static int num_events;

extern int lpc3131_reg_to_gpio(unsigned index, unsigned gpio);

/* when a gpio pin is request as an interrupt source,
 * make sure it is input mode
 */
static int set_input(unsigned irq)
{
	int ret, reg, gpio, event;

	event = events[irq - 30].event;
	reg  = event_to_gpioreg[event];
	if (!reg) /* not a gpio pin */
		return 0;
	gpio = lpc3131_reg_to_gpio(reg >> 5, reg & 0x1F);
	printk("setting to input %d\n", gpio);
	ret = gpio_request(gpio, "IRQ");
	if (ret)
		return ret;
	return gpio_direction_input(gpio);
}

int event_to_irq(int event)
{
	int i;
	for (i = 0; i < num_events; i++) {
		if (events[i].event == event) {
			return i + 30; /* fixme */
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(event_to_irq);

static const struct of_device_id evtr_of_match[] __initconst = {
	{ .compatible = "nxp,lpc31xx-evtr", },
	{},
};

static int __devinit lpc313x_evtr_probe(struct platform_device *pdev)
{
	const __be32 *ip;
	struct device_node *np = pdev->dev.of_node;
	int cells, length, i;

	printk("###### Event router probe ######\n");

	irq_domain_generate_simple(evtr_of_match, 0x13000000, 30);

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
	return 0;
}

static int lpc313x_evtr_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static struct platform_driver lpc313x_evtr_driver = {
	.driver = {
		.name = "lpc31xx-evtr",
		.owner = THIS_MODULE,
		.of_match_table = evtr_of_match,
	},
	.probe = lpc313x_evtr_probe,
	.remove = lpc313x_evtr_remove,
};

static __init int lpc313x_evtr_init(void)
{
	if (platform_driver_register(&lpc313x_evtr_driver))
		printk(KERN_ERR "Unable to register Event Router driver\n");

	return 0;
}

/* Make sure we get initialised before anyone else tries to use us */
core_initcall(lpc313x_evtr_init);


