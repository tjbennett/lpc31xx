/*  drivers/tty/serial/8250/8250_lpc31xx.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  Common code for machines with LPC31xx SoCs.
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
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/console.h>
#include <linux/serial_8250.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/errno.h>
#include <asm/pgtable.h>

#include <mach/hardware.h>

#include <mach/gpio.h>
#include <asm/mach/map.h>

#ifdef CONFIG_SERIAL_8250_LPC31xx_DMA
static void lpc31xx_dma_lock(struct uart_8250_port *up)
{
	mutex_lock(&dma_mutex);
}

static void lpc31xx_dma_unlock(struct uart_8250_port *up)
{
	mutex_unlock(&dma_mutex);
}

static void lpc31xx_uart_tx_dma_start(struct uart_8250_port *up);
static void lpc31xx_dma_tx_tasklet_func(unsigned long data)
{
	struct uart_8250_port *up = (struct uart_8250_port *) data;
	struct circ_buf *xmit = &up->port.state->xmit;

	if (dma_channel_enabled(up->dma_tx.dmach))
		return;

	lpc31xx_dma_lock(up);

	dma_stop_channel(up->dma_tx.dmach);

	xmit->tail = (xmit->tail + up->dma_tx.count) & (UART_XMIT_SIZE - 1);
	up->port.icount.tx += up->dma_tx.count;

	lpc31xx_uart_tx_dma_start(up);

	lpc31xx_dma_unlock(up);
}

static int lpc31xx_get_readl_rx_dma_count(struct uart_8250_port *up)
{
	int count;

	/* The DMA hardware returns the number of bytes currently
	   transferred by the hardware. It will return 0 when the
	   channel has stopped (full DMA transfer) or when nothing
	   has been transferred. To tell the difference between
	   empty 0 and full 0, we need to examine the DMA enable
	   status. */

	/* A race condition can exist where the DMA TCNT returns a
	   value right as the DMA is stopping. In this case, the
	   DMA is enabled during the check with a non-0 count
	   value. To get around this issue, the DMA count value
	   need to be verified again after disabling the DMA
	   channel. If it is 0, then the DMA completed and the
	   count is different. */
	dma_read_counter(up->dma_rx.dmach, &count);
	if ((!count) && (!dma_channel_enabled(up->dma_rx.dmach)))
		count = UART_XMIT_SIZE;

	return count;
}

static void serial8250_dma_rx_timer_check(unsigned long data)
{
	struct uart_8250_port *up = (struct uart_8250_port *) data;

	/* Emulate RX timeout when DMA buffer is not full */
	if ((lpc31xx_get_readl_rx_dma_count(up)) && (up->dma_rx.active))
		tasklet_schedule(&up->dma_rx.tasklet);
	else
		mod_timer(&up->dma_rx.timer, jiffies +
			msecs_to_jiffies(LPC31XX_UART_RX_TIMEOUT));
}

void lcp31xx_dma_rx_setup(struct uart_8250_port *up)
{
	dma_setup_t dmarx;

	up->buff_half_offs = UART_XMIT_SIZE - up->buff_half_offs;
	dmarx.trans_length = UART_XMIT_SIZE - 1;
	dmarx.src_address = (u32) up->port.mapbase;
	dmarx.dest_address = (u32) up->dma_rx.dma_buff_p;
	dmarx.dest_address += up->buff_half_offs;
	dmarx.cfg = DMA_CFG_TX_BYTE | DMA_CFG_RD_SLV_NR(DMA_SLV_UART_RX) |
		DMA_CFG_WR_SLV_NR(0);

	dma_prog_channel(up->dma_rx.dmach, &dmarx);
	dma_start_channel(up->dma_rx.dmach);
}

/*
 * DMA RX tasklet
 */
unsigned int serial8250_modem_status(struct uart_8250_port *up);
static void lpc31xx_dma_rx_tasklet_func(unsigned long data)
{
	unsigned int status, lsr;
	int count, count2, i, maxcount = 64, breakflush = 0;
	char ch, flag = TTY_NORMAL, *buf;
	struct uart_8250_port *up = (struct uart_8250_port *) data;
	int buffhalf = up->buff_half_offs;
	u32 pbuf;

	spin_lock(&up->port.lock);

	/*
	 * Per char stats don't work with DMA, so the status flags
	 * don't apply to a specific character. We'll take a best
	 * guess that the accumulated status only applies to the
	 * last character in the DMA buffer.
	 */
	status = serial_inp(up, UART_LSR);
	lsr = status | up->lsr_saved_flags;
	up->lsr_saved_flags = 0;

	if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
		/*
		 * For statistics only
		 */
		if (lsr & UART_LSR_BI) {
			lsr &= ~(UART_LSR_FE | UART_LSR_PE);
			up->port.icount.brk++;
			breakflush = 1;
			/*
			 * Breaks are trouble! Toss everything if
			 * one occurs.
			 */
			uart_handle_break(&up->port);
		} else if (lsr & UART_LSR_PE)
			up->port.icount.parity++;
		else if (lsr & UART_LSR_FE)
			up->port.icount.frame++;
		if (lsr & UART_LSR_OE)
			up->port.icount.overrun++;

		/*
		 * Mask off conditions which should be ignored.
		 */
		lsr &= up->port.read_status_mask;

		if (lsr & UART_LSR_BI) {
			DEBUG_INTR("handling break....");
			flag = TTY_BREAK;
		} else if (lsr & UART_LSR_PE)
			flag = TTY_PARITY;
		else if (lsr & UART_LSR_FE)
			flag = TTY_FRAME;
	}

	/* Disable DMA and get current DMA bytes transferred */
	count = lpc31xx_get_readl_rx_dma_count(up);
	dma_stop_channel(up->dma_rx.dmach);
	count2 = lpc31xx_get_readl_rx_dma_count(up);
	if (count != count2) {
		if (count2 == 0)
			count = UART_XMIT_SIZE;
		else
			count = count2;
	}
	dma_write_counter(up->dma_rx.dmach, 0);

	/* Setup DMA again using unused buffer half */
	lcp31xx_dma_rx_setup(up);
	pbuf = (u32) up->dma_rx.dma_buff_p;
	pbuf += buffhalf;
	buf = (char *) up->dma_rx.dma_buff_v;
	buf += buffhalf;

	if (breakflush) {
		/* Flush RX FIFO */
		while ((serial_inp(up, UART_LSR) & UART_LSR_DR) &&
			(maxcount-- > 0))
			ch = serial_inp(up, UART_RX);
	}
	else {
		for (i = 0; i < (count - 1); i++) {
			up->port.icount.rx++;
			if (uart_handle_sysrq_char(&up->port, buf[i]))
				continue;

			uart_insert_char(&up->port, lsr, UART_LSR_OE, buf[i], TTY_NORMAL);
		}

		up->port.icount.rx++;
		if (!uart_handle_sysrq_char(&up->port, buf[i]))
			uart_insert_char(&up->port, lsr, UART_LSR_OE, buf[i], flag);
	}

	serial8250_modem_status(up);

	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(up->port.state->port.tty);
	spin_lock(&up->port.lock);

	mod_timer(&up->dma_rx.timer, jiffies +
		msecs_to_jiffies(LPC31XX_UART_RX_TIMEOUT));

	/* Clear any pending RX error status and re-enable TX status interrupt */
	status = serial_inp(up, UART_LSR);
	serial_outp(up, UART_IER, up->ier);

	spin_unlock(&up->port.lock);
}

/*
 * DMA UART TX completion interrupt - this interrupt is more of a spotholder
 * as it is disabled and will never fire.
 */
static void lpc31xx_dma_tx_interrupt(int ch, dma_irq_type_t dtype, void *handle)
{
	struct uart_8250_port *up = handle;

	printk(KERN_INFO "serial DMA TX interrupt unexpected\n");
	tasklet_schedule(&up->dma_tx.tasklet);
}

/*
 * DMA UART RX completion interrupt - fires when the DMA RX transfer
 * is complete.
 */
static void lpc31xx_dma_rx_interrupt(int ch, dma_irq_type_t dtype, void *handle)
{
	struct uart_8250_port *up = handle;

	tasklet_schedule(&up->dma_rx.tasklet);
}

static void lpc31xx_uart_tx_dma_start(struct uart_8250_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	dma_setup_t dmatx;

	/* Start a DMA transfer, DMA is idle if this is called and
	   TX is enabled. */
	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(&up->port)) {
		dma_sync_single_for_device(up->port.dev,
					   up->dma_tx.dma_buff_p,
					   UART_XMIT_SIZE,
					   DMA_TO_DEVICE);

		up->dma_tx.count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
		if (up->dma_tx.count > 64)
			up->dma_tx.count = 64;

		/* Note TX buffer is loaned to the DMA so the TX bytes can't
		   be released until the DMA transfer is complete. */
		dmatx.trans_length = up->dma_tx.count - 1;
		dmatx.src_address = (u32) up->dma_tx.dma_buff_p;
		dmatx.src_address += xmit->tail;
		dmatx.dest_address = (u32) up->port.mapbase;
		dmatx.cfg = DMA_CFG_TX_BYTE | DMA_CFG_RD_SLV_NR(0) |
			DMA_CFG_WR_SLV_NR(DMA_SLV_UART_TX);

		dma_prog_channel(up->dma_tx.dmach, &dmatx);
		up->dma_tx.active = 1;
		dma_start_channel(up->dma_tx.dmach);

		/* Enable TX interrupt on TX FIFO empty */
		up->ier |= UART_IER_THRI;
	}
	else {
		up->dma_tx.active = 0;
		up->ier &= ~UART_IER_THRI;
	}

	serial_out(up, UART_IER, up->ier);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);
}
#endif


static struct of_device_id uart_ids[] = {
	{ .compatible = "nxp,lpc31xx-uart" },
	{ /* sentinel */ }
};

static void lpc31xx_uart_pm(struct uart_port * port, unsigned int state,
			      unsigned int oldstate)
{
	switch (state) {
	case 0:
		/* Free the pins so that UART IP will take control of it */
		if (oldstate != -1) {
			gpio_free(GPIO_UART_RXD);
			gpio_free(GPIO_UART_TXD);
		}
		/*
		 * Enable the peripheral clock for this serial port.
		 * This is called on uart_open() or a resume event.
		 */
		/* Enable UART base clock */
		cgu_endis_base_freq(CGU_SB_UARTCLK_BASE_ID, 1);

		/* Enable UART IP clock */
		cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 1);
		cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 1);
		break;
	case 1:
		/* we can wake the system in this state. So leave clocks on */
		printk(KERN_INFO "lpc31xx_uart_pm: UART can wake\n");
		break;
	case 3:
		/*
		 * Disable the peripheral clock for this serial port.
		 * This is called on uart_close() or a suspend event.
		 */
		/* Disable UART IP clock */
		cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 0);
		cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 0);

		/* Disable UART base clock */
		cgu_endis_base_freq(CGU_SB_UARTCLK_BASE_ID, 0);

		/* Free the pins and let GPIO handle it */
		gpio_request(GPIO_UART_RXD, "uart_rx");
		gpio_request(GPIO_UART_TXD, "uart_tx");

		gpio_direction_input(GPIO_UART_RXD);
		gpio_direction_output(GPIO_UART_TXD, 0);
		break;
	default:
		printk(KERN_ERR "lpc31xx_uart_pm: unknown pm %d\n", state);
	}

}

static struct plat_serial8250_port platform_serial_ports[] = {
	{
		.membase = (void *)io_p2v(UART_PHYS),
		.mapbase = (unsigned long)UART_PHYS,
		//.irq = IRQ_UART,
		.irq = 13,
		.uartclk = XTAL_CLOCK,
		.regshift = 2,
		.iotype = UPIO_MEM,
		.type	= PORT_NXP16750,
		.flags = UPF_BOOT_AUTOCONF | UPF_BUGGY_UART | UPF_SKIP_TEST,
		.pm = lpc31xx_uart_pm,
	},
	{
		.flags		= 0
	},
};

static struct platform_device serial_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = &platform_serial_ports,
	},
};

static struct platform_device *devices[] __initdata = {
	&serial_device,
};

static void __init lpc31xx_uart_init(void)
{
	int mul, div;

	struct device_node *node;
	int irq;

	node = of_find_matching_node(NULL, uart_ids);
	if (!node)
		return;

	/* Get the interrupts property */
	irq = irq_of_parse_and_map(node, 0);
	if (!irq) {
		pr_crit("LPC31xx: UART -  unable to get IRQ from DT\n");
		return;
	}
	of_node_put(node);

	platform_serial_ports[0].irq = irq;
	printk("JDS - Uart IRQ %d\n", irq);

	/* check what FDR bootloader is using */
	mul = (UART_FDR_REG >> 4) & 0xF;
	div = UART_FDR_REG & 0xF;
	if (div != 0)  {
		platform_serial_ports[0].uartclk = (XTAL_CLOCK * mul) / (mul + div);
	}
}

#if defined(CONFIG_SERIAL_8250_CONSOLE)
static int __init lpc31xx_init_console(void)
{
	static __initdata char serr[] =
		KERN_ERR "Serial port #%u setup failed\n";
	struct uart_port up;

	/* Switch on the UART clocks */
	cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 1);

 	/*
	 * Set up serial port #0. Do not use autodetection; the result is
	 * not what we want.
 	 */
	memset(&up, 0, sizeof(up));

	lpc31xx_uart_init();
	up.uartclk = platform_serial_ports[0].uartclk;
	up.irq = platform_serial_ports[0].irq;

	up.membase = (char *) io_p2v(UART_PHYS);
	up.mapbase = (unsigned long)UART_PHYS,
	up.regshift = 2;
	up.iotype = UPIO_MEM;
	up.type	= PORT_NXP16750;
	up.flags = UPF_BOOT_AUTOCONF | UPF_BUGGY_UART | UPF_SKIP_TEST;
	up.line	= 0;
	if (early_serial_setup(&up))
		printk(serr, up.line);

	return 0;
}
console_initcall(lpc31xx_init_console);

#endif /*CONFIG_SERIAL_8250_CONSOLE*/




