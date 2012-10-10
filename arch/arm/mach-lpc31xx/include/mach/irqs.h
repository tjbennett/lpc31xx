/* linux/arch/arm/mach-lpc31xx/include/mach/irqs.h
 *
 * Author:	Durgesh Pattamatta
 * Copyright (C) 2009 NXP semiconductors
 *
 * IRQ defines for LPC31xx SoCs.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */


#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#define IRQ_EVT_ROUTER0	1	/*interrupts from Event router 0*/
#define IRQ_EVT_ROUTER1	2	/*interrupts from Event router 1*/
#define IRQ_EVT_ROUTER2	3	/*interrupts from Event router 2*/
#define IRQ_EVT_ROUTER3	4	/*interrupts from Event router 3*/
#define IRQ_TIMER0	5	/*Timer 0 IRQ */
#define IRQ_TIMER1	6	/*Timer 1 IRQ */
#define IRQ_TIMER2	7	/*Timer 2 IRQ */
#define IRQ_TIMER3	8	/*Timer 3 IRQ */
#define IRQ_ADC		9	/*10bit ADC IRQ*/
#define IRQ_UART	10	/*UART IRQ */
#define IRQ_I2C0	11	/*I2C 0 IRQ */
#define IRQ_I2C1	12	/*I2C 1 IRQ */
#define IRQ_I2S0_OUT	13	/*I2S 0 out IRQ */
#define IRQ_I2S1_OUT	14	/*I2S 1 out IRQ */
#define IRQ_I2S0_IN	15	/*I2S 0 IN IRQ */
#define IRQ_I2S1_IN	16	/*I2S 1 IN IRQ */
#define IRQ_LCD		18	/*LCD IRQ */
#define IRQ_SPI_SMS	19	/*SPI SMS IRQ */
#define IRQ_SPI_TX	20	/*SPI Transmit IRQ */
#define IRQ_SPI_RX	21	/*SPI Receive IRQ */
#define IRQ_SPI_OVR	22	/*SPI overrun IRQ */
#define IRQ_SPI		23	/*SPI interrupt IRQ */
#define IRQ_DMA		24	/*DMA IRQ */
#define IRQ_NAND_FLASH	25	/*NAND flash IRQ */
#define IRQ_MCI		26	/*MCI IRQ */
#define IRQ_USB		27	/*USB IRQ */
#define IRQ_ISRAM0	28	/*ISRAM0 IRQ */
#define IRQ_ISRAM1	29	/*ISRAM1 IRQ */

#define NR_IRQ_CPU	30	/* IRQs directly recognized by CPU */

#define NR_IRQ_BOARD	34	/* Leave room for board specific IRQs */

#define NR_IRQS		(NR_IRQ_CPU + NR_IRQ_BOARD)

#endif
