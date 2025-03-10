/*
 *  Freescale lpuart serial port driver
 *
 *  Copyright 2012-2014 Freescale Semiconductor, Inc.
 *  Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#if defined(CONFIG_SERIAL_FSL_LPUART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>

/* All registers are 8-bit width */
#define UARTBDH			0x00
#define UARTBDL			0x01
#define UARTCR1			0x02
#define UARTCR2			0x03
#define UARTSR1			0x04
#define UARTCR3			0x06
#define UARTDR			0x07
#define UARTCR4			0x0a
#define UARTCR5			0x0b
#define UARTMODEM		0x0d
#define UARTPFIFO		0x10
#define UARTCFIFO		0x11
#define UARTSFIFO		0x12
#define UARTTWFIFO		0x13
#define UARTTCFIFO		0x14
#define UARTRWFIFO		0x15

#define UARTBDH_LBKDIE		0x80
#define UARTBDH_RXEDGIE		0x40
#define UARTBDH_SBR_MASK	0x1f

#define UARTCR1_LOOPS		0x80
#define UARTCR1_RSRC		0x20
#define UARTCR1_M		0x10
#define UARTCR1_WAKE		0x08
#define UARTCR1_ILT		0x04
#define UARTCR1_PE		0x02
#define UARTCR1_PT		0x01

#define UARTCR2_TIE		0x80
#define UARTCR2_TCIE		0x40
#define UARTCR2_RIE		0x20
#define UARTCR2_ILIE		0x10
#define UARTCR2_TE		0x08
#define UARTCR2_RE		0x04
#define UARTCR2_RWU		0x02
#define UARTCR2_SBK		0x01

#define UARTSR1_TDRE		0x80
#define UARTSR1_TC		0x40
#define UARTSR1_RDRF		0x20
#define UARTSR1_IDLE		0x10
#define UARTSR1_OR		0x08
#define UARTSR1_NF		0x04
#define UARTSR1_FE		0x02
#define UARTSR1_PE		0x01

#define UARTCR3_R8		0x80
#define UARTCR3_T8		0x40
#define UARTCR3_TXDIR		0x20
#define UARTCR3_TXINV		0x10
#define UARTCR3_ORIE		0x08
#define UARTCR3_NEIE		0x04
#define UARTCR3_FEIE		0x02
#define UARTCR3_PEIE		0x01

#define UARTCR4_MAEN1		0x80
#define UARTCR4_MAEN2		0x40
#define UARTCR4_M10		0x20
#define UARTCR4_BRFA_MASK	0x1f
#define UARTCR4_BRFA_OFF	0

#define UARTCR5_TDMAS		0x80
#define UARTCR5_RDMAS		0x20

#define UARTMODEM_RXRTSE	0x08
#define UARTMODEM_TXRTSPOL	0x04
#define UARTMODEM_TXRTSE	0x02
#define UARTMODEM_TXCTSE	0x01

#define UARTPFIFO_TXFE		0x80
#define UARTPFIFO_FIFOSIZE_MASK	0x7
#define UARTPFIFO_TXSIZE_OFF	4
#define UARTPFIFO_RXFE		0x08
#define UARTPFIFO_RXSIZE_OFF	0

#define UARTCFIFO_TXFLUSH	0x80
#define UARTCFIFO_RXFLUSH	0x40
#define UARTCFIFO_RXOFE		0x04
#define UARTCFIFO_TXOFE		0x02
#define UARTCFIFO_RXUFE		0x01

#define UARTSFIFO_TXEMPT	0x80
#define UARTSFIFO_RXEMPT	0x40
#define UARTSFIFO_RXOF		0x04
#define UARTSFIFO_TXOF		0x02
#define UARTSFIFO_RXUF		0x01

/* 32-bit register definition */
#define UARTBAUD		0x00
#define UARTSTAT		0x04
#define UARTCTRL		0x08
#define UARTDATA		0x0C
#define UARTMATCH		0x10
#define UARTMODIR		0x14
#define UARTFIFO		0x18
#define UARTWATER		0x1c

/* 32-bit global registers only for i.MX7ulp/MX8x
 * The driver only use the reset feature to reset HW.
 */
#define UART_GLOBAL		0x8

#define UARTBAUD_MAEN1		0x80000000
#define UARTBAUD_MAEN2		0x40000000
#define UARTBAUD_M10		0x20000000
#define UARTBAUD_TDMAE		0x00800000
#define UARTBAUD_RDMAE		0x00200000
#define UARTBAUD_RIDMAE		0x00100000
#define UARTBAUD_MATCFG		0x00400000
#define UARTBAUD_BOTHEDGE	0x00020000
#define UARTBAUD_RESYNCDIS	0x00010000
#define UARTBAUD_LBKDIE		0x00008000
#define UARTBAUD_RXEDGIE	0x00004000
#define UARTBAUD_SBNS		0x00002000
#define UARTBAUD_SBR		0x00000000
#define UARTBAUD_SBR_MASK	0x1fff
#define UARTBAUD_OSR_MASK       0x1f
#define UARTBAUD_OSR_SHIFT      24

#define UARTSTAT_LBKDIF		0x80000000
#define UARTSTAT_RXEDGIF	0x40000000
#define UARTSTAT_MSBF		0x20000000
#define UARTSTAT_RXINV		0x10000000
#define UARTSTAT_RWUID		0x08000000
#define UARTSTAT_BRK13		0x04000000
#define UARTSTAT_LBKDE		0x02000000
#define UARTSTAT_RAF		0x01000000
#define UARTSTAT_TDRE		0x00800000
#define UARTSTAT_TC		0x00400000
#define UARTSTAT_RDRF		0x00200000
#define UARTSTAT_IDLE		0x00100000
#define UARTSTAT_OR		0x00080000
#define UARTSTAT_NF		0x00040000
#define UARTSTAT_FE		0x00020000
#define UARTSTAT_PE		0x00010000
#define UARTSTAT_MA1F		0x00008000
#define UARTSTAT_M21F		0x00004000

#define UARTCTRL_R8T9		0x80000000
#define UARTCTRL_R9T8		0x40000000
#define UARTCTRL_TXDIR		0x20000000
#define UARTCTRL_TXINV		0x10000000
#define UARTCTRL_ORIE		0x08000000
#define UARTCTRL_NEIE		0x04000000
#define UARTCTRL_FEIE		0x02000000
#define UARTCTRL_PEIE		0x01000000
#define UARTCTRL_TIE		0x00800000
#define UARTCTRL_TCIE		0x00400000
#define UARTCTRL_RIE		0x00200000
#define UARTCTRL_ILIE		0x00100000
#define UARTCTRL_TE		0x00080000
#define UARTCTRL_RE		0x00040000
#define UARTCTRL_RWU		0x00020000
#define UARTCTRL_SBK		0x00010000
#define UARTCTRL_MA1IE		0x00008000
#define UARTCTRL_MA2IE		0x00004000
#define UARTCTRL_IDLECFG_OFF	0x8
#define UARTCTRL_LOOPS		0x00000080
#define UARTCTRL_DOZEEN		0x00000040
#define UARTCTRL_RSRC		0x00000020
#define UARTCTRL_M		0x00000010
#define UARTCTRL_WAKE		0x00000008
#define UARTCTRL_ILT		0x00000004
#define UARTCTRL_PE		0x00000002
#define UARTCTRL_PT		0x00000001

#define UARTDATA_NOISY		0x00008000
#define UARTDATA_PARITYE	0x00004000
#define UARTDATA_FRETSC		0x00002000
#define UARTDATA_RXEMPT		0x00001000
#define UARTDATA_IDLINE		0x00000800
#define UARTDATA_INVALID	0x0000F000
#define UARTDATA_MASK		0x3ff

#define UARTMODIR_IREN		0x00020000
#define UARTMODIR_RTSWATER_S	0x8
#define UARTMODIR_RTSWATER_M	0x0000ff00
#define UARTMODIR_TXCTSSRC	0x00000020
#define UARTMODIR_TXCTSC	0x00000010
#define UARTMODIR_RXRTSE	0x00000008
#define UARTMODIR_TXRTSPOL	0x00000004
#define UARTMODIR_TXRTSE	0x00000002
#define UARTMODIR_TXCTSE	0x00000001

#define UARTFIFO_TXEMPT		0x00800000
#define UARTFIFO_RXEMPT		0x00400000
#define UARTFIFO_TXOF		0x00020000
#define UARTFIFO_RXUF		0x00010000
#define UARTFIFO_TXFLUSH	0x00008000
#define UARTFIFO_RXFLUSH	0x00004000
#define UARTFIFO_RXIDEN_MASK	0x7
#define UARTFIFO_RXIDEN_OFF	10
#define UARTFIFO_TXOFE		0x00000200
#define UARTFIFO_RXUFE		0x00000100
#define UARTFIFO_TXFE		0x00000080
#define UARTFIFO_FIFOSIZE_MASK	0x7
#define UARTFIFO_TXSIZE_OFF	4
#define UARTFIFO_RXFE		0x00000008
#define UARTFIFO_RXSIZE_OFF	0

#define UARTWATER_COUNT_MASK	0xff
#define UARTWATER_TXCNT_OFF	8
#define UARTWATER_RXCNT_OFF	24
#define UARTWATER_WATER_MASK	0xff
#define UARTWATER_TXWATER_OFF	0
#define UARTWATER_RXWATER_OFF	16

#define UART_GLOBAL_RST		0x2
#define RST_HW_MIN_US		20
#define RST_HW_MAX_US		40

#define UARTFIFO_RXIDEN_RDRF	0x3
#define UARTCTRL_IDLECFG	0x7
#define FSL_UART_RX_DMA_BUFFER_SIZE	128
#define UART_AUTOSUSPEND_TIMEOUT	3000

#define DRIVER_NAME	"fsl-lpuart"
#define DEV_NAME	"ttyLP"
#define UART_NR		6

/* IMX lpuart has four extra unused regs located at the beginning */
#define IMX_REG_OFF	0x10

struct lpuart_port {
	struct uart_port	port;
	void __iomem		*regbase;

	struct clk		*ipg_clk;
	struct clk		*per_clk;
	unsigned int		txfifo_size;
	unsigned int		rxfifo_size;
	unsigned int		txfifo_watermark;
	unsigned int		rxfifo_watermark;

	unsigned int		rts_watermark;
	bool			dma_eeop;

	bool			lpuart_dma_tx_use;
	bool			lpuart_dma_rx_use;
	bool			dma_rx_chan_active;
	struct dma_chan		*dma_tx_chan;
	struct dma_chan		*dma_rx_chan;
	struct dma_async_tx_descriptor  *dma_tx_desc;
	struct dma_async_tx_descriptor  *dma_rx_desc;
	dma_addr_t		dma_rx_buf_bus;
	dma_cookie_t		dma_tx_cookie;
	dma_cookie_t		dma_rx_cookie;
	unsigned char		*dma_rx_buf_virt;
	unsigned int		dma_tx_bytes;
	unsigned int		dma_rx_bytes;
	size_t			rxdma_len;
	bool			dma_tx_in_progress;
	bool			dma_rx_in_progress;
	unsigned int		dma_rx_timeout;
	struct timer_list	lpuart_timer;
	struct scatterlist	rx_sgl, tx_sgl[2];
	unsigned int		dma_tx_nents;
	wait_queue_head_t	dma_wait;
};

struct lpuart_soc_data {
	char	iotype;
	u8	reg_off;
};

static const struct lpuart_soc_data vf_data = {
	.iotype = UPIO_MEM,
};

static const struct lpuart_soc_data ls_data = {
	.iotype = UPIO_MEM32BE,
};

static struct lpuart_soc_data imx_data = {
	.iotype = UPIO_MEM32,
	.reg_off = IMX_REG_OFF,
};

static const struct of_device_id lpuart_dt_ids[] = {
	{ .compatible = "fsl,vf610-lpuart",	.data = &vf_data, },
	{ .compatible = "fsl,ls1021a-lpuart",	.data = &ls_data, },
	{ .compatible = "fsl,imx7ulp-lpuart",	.data = &imx_data, },
	{ .compatible = "fsl,imx8qm-lpuart",	.data = &imx_data, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lpuart_dt_ids);

static bool lpuart_is_32(struct lpuart_port *sport)
{
	return sport->port.iotype == UPIO_MEM32 ||
	       sport->port.iotype ==  UPIO_MEM32BE;
}

/* Forward declare this for the dma callbacks*/
static int lpuart_dma_rx(struct lpuart_port *sport);
static void lpuart_dma_tx_complete(void *arg);
static inline void lpuart_prepare_rx(struct lpuart_port *sport);

static inline u32 lpuart32_read(struct uart_port *port, u32 off)
{
	switch (port->iotype) {
	case UPIO_MEM32:
		return readl(port->membase + off);
	case UPIO_MEM32BE:
		return ioread32be(port->membase + off);
	default:
		return 0;
	}
}

static inline void lpuart32_write(struct uart_port *port, u32 val,
				  u32 off)
{
	switch (port->iotype) {
	case UPIO_MEM32:
		writel(val, port->membase + off);
		break;
	case UPIO_MEM32BE:
		iowrite32be(val, port->membase + off);
		break;
	}
}

static int lpuart_hw_reset(struct lpuart_port *sport)
{
	struct uart_port *port = &sport->port;
	struct device_node *np = sport->port.dev->of_node;
	int ret;

	if (uart_console(port))
		return 0;

	ret = clk_prepare_enable(sport->ipg_clk);
	if (ret) {
		dev_err(sport->port.dev, "failed to enable uart ipg clk: %d\n", ret);
		return ret;
	}

	if (np && (of_device_is_compatible(np, "fsl,imx7ulp-lpuart") ||
	    of_device_is_compatible(np, "fsl,imx8qm-lpuart"))) {
		writel(UART_GLOBAL_RST, sport->regbase + UART_GLOBAL);
		usleep_range(RST_HW_MIN_US, RST_HW_MAX_US);
		writel(0, sport->regbase + UART_GLOBAL);
		usleep_range(RST_HW_MIN_US, RST_HW_MAX_US);
	}

	clk_disable_unprepare(sport->ipg_clk);
	return 0;
}

static void lpuart_stop_tx(struct uart_port *port)
{
	unsigned char temp;

	temp = readb(port->membase + UARTCR2);
	temp &= ~(UARTCR2_TIE | UARTCR2_TCIE);
	writeb(temp, port->membase + UARTCR2);
}

static void lpuart32_stop_tx(struct uart_port *port)
{
	unsigned long temp;

	temp = lpuart32_read(port, UARTCTRL);
	temp &= ~(UARTCTRL_TIE | UARTCTRL_TCIE);
	lpuart32_write(port, temp, UARTCTRL);
}

static void lpuart_stop_rx(struct uart_port *port)
{
	unsigned char temp;

	temp = readb(port->membase + UARTCR2);
	writeb(temp & ~UARTCR2_RE, port->membase + UARTCR2);
}

static void lpuart32_stop_rx(struct uart_port *port)
{
	unsigned long temp;

	temp = lpuart32_read(port, UARTCTRL);
	lpuart32_write(port, temp & ~UARTCTRL_RE, UARTCTRL);
}

static void lpuart_recal_min_trans_size(struct lpuart_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	u32 txcount, rxcount;

	sport->dma_tx_bytes = uart_circ_chars_pending(xmit);

	/* lpuart32 and loopback mode re-calculate the trans size */
	if (!lpuart_is_32(sport) || !(sport->port.mctrl & TIOCM_LOOP))
		return;

	txcount = lpuart32_read(&sport->port, UARTWATER);
	txcount = txcount >> UARTWATER_TXCNT_OFF;
	txcount &= UARTWATER_COUNT_MASK;
	rxcount = lpuart32_read(&sport->port, UARTWATER);
	rxcount = rxcount >> UARTWATER_RXCNT_OFF;
	txcount = min_t(unsigned int, sport->txfifo_size - txcount,
			sport->rxfifo_size - rxcount);
	sport->dma_tx_bytes = min_t(unsigned int, txcount, sport->dma_tx_bytes);
}

static void lpuart_dma_tx(struct lpuart_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	struct scatterlist *sgl = sport->tx_sgl;
	struct device *dev = sport->port.dev;
	u32 toend_cnt;
	int ret;

	if (sport->dma_tx_in_progress)
		return;

	lpuart_recal_min_trans_size(sport);
	if (!sport->dma_tx_bytes)
		return;

	toend_cnt = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (xmit->tail < xmit->head || xmit->head == 0 ||
	   (sport->port.mctrl & TIOCM_LOOP && sport->dma_tx_bytes <= toend_cnt)) {
		sport->dma_tx_nents = 1;
		sg_init_one(sgl, xmit->buf + xmit->tail, sport->dma_tx_bytes);
	} else {
		sport->dma_tx_nents = 2;
		sg_init_table(sgl, 2);
		sg_set_buf(sgl, xmit->buf + xmit->tail,
				UART_XMIT_SIZE - xmit->tail);
		sg_set_buf(sgl + 1, xmit->buf, sport->dma_tx_bytes -
				(UART_XMIT_SIZE - xmit->tail));
	}

	ret = dma_map_sg(dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);
	if (!ret) {
		dev_err(dev, "DMA mapping error for TX.\n");
		return;
	}

	sport->dma_tx_desc = dmaengine_prep_slave_sg(sport->dma_tx_chan, sgl,
					ret, DMA_MEM_TO_DEV,
					DMA_PREP_INTERRUPT);
	if (!sport->dma_tx_desc) {
		dma_unmap_sg(dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);
		dev_err(dev, "Cannot prepare TX slave DMA!\n");
		return;
	}

	sport->dma_tx_desc->callback = lpuart_dma_tx_complete;
	sport->dma_tx_desc->callback_param = sport;
	sport->dma_tx_in_progress = true;
	sport->dma_tx_cookie = dmaengine_submit(sport->dma_tx_desc);
	dma_async_issue_pending(sport->dma_tx_chan);
}

static void lpuart_dma_tx_complete(void *arg)
{
	struct lpuart_port *sport = arg;
	struct scatterlist *sgl = &sport->tx_sgl[0];
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	if (!sport->dma_tx_in_progress) {
		spin_unlock_irqrestore(&sport->port.lock, flags);
		return;
	}

	dma_unmap_sg(sport->port.dev, sgl, sport->dma_tx_nents, DMA_TO_DEVICE);

	xmit->tail = (xmit->tail + sport->dma_tx_bytes) & (UART_XMIT_SIZE - 1);

	sport->port.icount.tx += sport->dma_tx_bytes;
	sport->dma_tx_in_progress = false;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (waitqueue_active(&sport->dma_wait)) {
		wake_up(&sport->dma_wait);
		spin_unlock_irqrestore(&sport->port.lock, flags);
		return;
	}

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(&sport->port))
		lpuart_dma_tx(sport);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int lpuart_dma_tx_request(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
					struct lpuart_port, port);
	struct dma_slave_config dma_tx_sconfig = {};
	int ret;

	if (lpuart_is_32(sport))
		dma_tx_sconfig.dst_addr = sport->port.mapbase + UARTDATA;
	else
		dma_tx_sconfig.dst_addr = sport->port.mapbase + UARTDR;
	dma_tx_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_tx_sconfig.dst_maxburst = 1;
	dma_tx_sconfig.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(sport->dma_tx_chan, &dma_tx_sconfig);

	if (ret < 0) {
		dev_err(sport->port.dev,
				"Dma slave config failed, err = %d\n", ret);
		return ret;
	}

	return 0;
}

static void lpuart_flush_buffer(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	u32 val;

	if (sport->lpuart_dma_tx_use) {
		if (sport->dma_tx_in_progress) {
			dma_unmap_sg(sport->port.dev, &sport->tx_sgl[0],
				sport->dma_tx_nents, DMA_TO_DEVICE);
			sport->dma_tx_in_progress = false;
		}
		dmaengine_terminate_all(sport->dma_tx_chan);
	}

	if (lpuart_is_32(sport)) {
		val = lpuart32_read(&sport->port, UARTFIFO);
		val |= UARTFIFO_TXFLUSH | UARTFIFO_RXFLUSH;
		lpuart32_write(&sport->port, val, UARTFIFO);
	} else {
		val = readb(sport->port.membase + UARTPFIFO);
		val |= UARTCFIFO_TXFLUSH | UARTCFIFO_RXFLUSH;
		writeb(val, sport->port.membase + UARTCFIFO);
	}
}

#if defined(CONFIG_CONSOLE_POLL)

static int lpuart_poll_init(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
					struct lpuart_port, port);
	unsigned long flags;
	unsigned char temp;

	sport->port.fifosize = 0;

	spin_lock_irqsave(&sport->port.lock, flags);
	/* Disable Rx & Tx */
	writeb(0, sport->port.membase + UARTCR2);

	temp = readb(sport->port.membase + UARTPFIFO);
	/* Enable Rx and Tx FIFO */
	writeb(temp | UARTPFIFO_RXFE | UARTPFIFO_TXFE,
			sport->port.membase + UARTPFIFO);

	/* flush Tx and Rx FIFO */
	writeb(UARTCFIFO_TXFLUSH | UARTCFIFO_RXFLUSH,
			sport->port.membase + UARTCFIFO);

	/* explicitly clear RDRF */
	if (readb(sport->port.membase + UARTSR1) & UARTSR1_RDRF) {
		readb(sport->port.membase + UARTDR);
		writeb(UARTSFIFO_RXUF, sport->port.membase + UARTSFIFO);
	}

	writeb(0, sport->port.membase + UARTTWFIFO);
	writeb(1, sport->port.membase + UARTRWFIFO);

	/* Enable Rx and Tx */
	writeb(UARTCR2_RE | UARTCR2_TE, sport->port.membase + UARTCR2);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	return 0;
}

static void lpuart_poll_put_char(struct uart_port *port, unsigned char c)
{
	/* drain */
	while (!(readb(port->membase + UARTSR1) & UARTSR1_TDRE))
		barrier();

	writeb(c, port->membase + UARTDR);
}

static int lpuart_poll_get_char(struct uart_port *port)
{
	if (!(readb(port->membase + UARTSR1) & UARTSR1_RDRF))
		return NO_POLL_CHAR;

	return readb(port->membase + UARTDR);
}

static int lpuart32_poll_init(struct uart_port *port)
{
	unsigned long flags;
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	u32 temp;

	sport->port.fifosize = 0;

	spin_lock_irqsave(&sport->port.lock, flags);

	/* Disable Rx & Tx */
	writel(0, sport->port.membase + UARTCTRL);

	temp = readl(sport->port.membase + UARTFIFO);

	/* Enable Rx and Tx FIFO */
	writel(temp | UARTFIFO_RXFE | UARTFIFO_TXFE,
		   sport->port.membase + UARTFIFO);

	/* flush Tx and Rx FIFO */
	writel(UARTFIFO_TXFLUSH | UARTFIFO_RXFLUSH,
			sport->port.membase + UARTFIFO);

	/* explicitly clear RDRF */
	if (readl(sport->port.membase + UARTSTAT) & UARTSTAT_RDRF) {
		readl(sport->port.membase + UARTDATA);
		writel(UARTFIFO_RXUF, sport->port.membase + UARTFIFO);
	}

	/* Enable Rx and Tx */
	writel(UARTCTRL_RE | UARTCTRL_TE, sport->port.membase + UARTCTRL);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	return 0;
}

static void lpuart32_poll_put_char(struct uart_port *port, unsigned char c)
{
	while (!(readl(port->membase + UARTSTAT) & UARTSTAT_TDRE))
		barrier();

	writel(c, port->membase + UARTDATA);
}

static int lpuart32_poll_get_char(struct uart_port *port)
{
	if (!(readl(port->membase + UARTSTAT) & UARTSTAT_RDRF))
		return NO_POLL_CHAR;

	return readl(port->membase + UARTDATA);
}
#endif

static inline void lpuart_transmit_buffer(struct lpuart_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;

	while (!uart_circ_empty(xmit) &&
		(readb(sport->port.membase + UARTTCFIFO) < sport->txfifo_size)) {
		writeb(xmit->buf[xmit->tail], sport->port.membase + UARTDR);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		lpuart_stop_tx(&sport->port);
}

static inline void lpuart32_transmit_buffer(struct lpuart_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long txcnt;

	txcnt = lpuart32_read(&sport->port, UARTWATER);
	txcnt = txcnt >> UARTWATER_TXCNT_OFF;
	txcnt &= UARTWATER_COUNT_MASK;
	while (!uart_circ_empty(xmit) && (txcnt < sport->txfifo_size)) {
		lpuart32_write(&sport->port, xmit->buf[xmit->tail], UARTDATA);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;
		txcnt = lpuart32_read(&sport->port, UARTWATER);
		txcnt = txcnt >> UARTWATER_TXCNT_OFF;
		txcnt &= UARTWATER_COUNT_MASK;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		lpuart32_stop_tx(&sport->port);
}

static void lpuart_start_tx(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
			struct lpuart_port, port);
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned char temp;

	if (sport->lpuart_dma_tx_use) {
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(port))
			lpuart_dma_tx(sport);
	} else {
		temp = readb(port->membase + UARTCR2);
		writeb(temp | UARTCR2_TIE, port->membase + UARTCR2);

		if (readb(port->membase + UARTSR1) & UARTSR1_TDRE)
			lpuart_transmit_buffer(sport);
	}
}

static void lpuart32_start_tx(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long temp;

	if (sport->lpuart_dma_tx_use) {
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(port))
			lpuart_dma_tx(sport);
	} else {
		temp = lpuart32_read(port, UARTCTRL);
		lpuart32_write(port, temp | UARTCTRL_TIE, UARTCTRL);
		if (lpuart32_read(&sport->port, UARTSTAT) & UARTSTAT_TDRE)
			lpuart32_transmit_buffer(sport);
	}
}

static void
lpuart_uart_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	switch (state) {
	case UART_PM_STATE_OFF:
		pm_runtime_mark_last_busy(port->dev);
		pm_runtime_put_autosuspend(port->dev);
		break;
	default:
		pm_runtime_get_sync(port->dev);
		break;
	}
}

/* return TIOCSER_TEMT when transmitter is not busy */
static unsigned int lpuart_tx_empty(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
			struct lpuart_port, port);
	unsigned char sr1 = readb(port->membase + UARTSR1);
	unsigned char sfifo = readb(port->membase + UARTSFIFO);

	if (sport->dma_tx_in_progress)
		return 0;

	if (sr1 & UARTSR1_TC && sfifo & UARTSFIFO_TXEMPT)
		return TIOCSER_TEMT;

	return 0;
}

static unsigned int lpuart32_tx_empty(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
			struct lpuart_port, port);
	unsigned int sr1 = lpuart32_read(port, UARTSTAT);
	unsigned int sfifo = lpuart32_read(port, UARTFIFO);

	if (sport->dma_tx_in_progress)
		return 0;

	if (sr1 & UARTSTAT_TC && sfifo & UARTFIFO_TXEMPT)
		return TIOCSER_TEMT;

	return 0;
}

static irqreturn_t lpuart_txint(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	if (sport->port.x_char) {
		if (lpuart_is_32(sport))
			lpuart32_write(&sport->port, sport->port.x_char, UARTDATA);
		else
			writeb(sport->port.x_char, sport->port.membase + UARTDR);
		goto out;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		if (lpuart_is_32(sport))
			lpuart32_stop_tx(&sport->port);
		else
			lpuart_stop_tx(&sport->port);
		goto out;
	}

	if (lpuart_is_32(sport))
		lpuart32_transmit_buffer(sport);
	else
		lpuart_transmit_buffer(sport);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t lpuart_rxint(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	unsigned int flg, ignored = 0;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags;
	unsigned char rx, sr;

	spin_lock_irqsave(&sport->port.lock, flags);

	while (!(readb(sport->port.membase + UARTSFIFO) & UARTSFIFO_RXEMPT)) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;
		/*
		 * to clear the FE, OR, NF, FE, PE flags,
		 * read SR1 then read DR
		 */
		sr = readb(sport->port.membase + UARTSR1);
		rx = readb(sport->port.membase + UARTDR);

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		if (sr & (UARTSR1_PE | UARTSR1_OR | UARTSR1_FE)) {
			if (sr & UARTSR1_PE)
				sport->port.icount.parity++;
			else if (sr & UARTSR1_FE)
				sport->port.icount.frame++;

			if (sr & UARTSR1_OR)
				sport->port.icount.overrun++;

			if (sr & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}

			sr &= sport->port.read_status_mask;

			if (sr & UARTSR1_PE)
				flg = TTY_PARITY;
			else if (sr & UARTSR1_FE)
				flg = TTY_FRAME;

			if (sr & UARTSR1_OR)
				flg = TTY_OVERRUN;

#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
#endif
		}

		tty_insert_flip_char(port, rx, flg);
	}

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);

	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}

static irqreturn_t lpuart32_rxint(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	unsigned int flg, ignored = 0;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags;
	unsigned long rx, sr;

	spin_lock_irqsave(&sport->port.lock, flags);

	while (!(lpuart32_read(&sport->port, UARTFIFO) & UARTFIFO_RXEMPT)) {
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		/*
		 * to clear the FE, OR, NF, FE, PE flags,
		 * read STAT then read DATA reg
		 */
		sr = lpuart32_read(&sport->port, UARTSTAT);
		rx = lpuart32_read(&sport->port, UARTDATA);

		if ((sr & UARTSTAT_FE) && (rx & UARTDATA_FRETSC) &&
			!(rx & UARTDATA_MASK)) {
			if (uart_handle_break(&sport->port))
				continue;
		}

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		if (sr & (UARTSTAT_PE | UARTSTAT_OR | UARTSTAT_FE)) {
			if (sr & UARTSTAT_PE)
				sport->port.icount.parity++;
			else if (sr & UARTSTAT_FE)
				sport->port.icount.frame++;

			if (sr & UARTSTAT_OR)
				sport->port.icount.overrun++;

			if (sr & sport->port.ignore_status_mask) {
				if (++ignored > 100)
					goto out;
				continue;
			}

			sr &= sport->port.read_status_mask;

			if (sr & UARTSTAT_PE)
				flg = TTY_PARITY;
			else if (sr & UARTSTAT_FE)
				flg = TTY_FRAME;

			if (sr & UARTSTAT_OR)
				flg = TTY_OVERRUN;

#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
#endif
			continue;
		}

		if (rx & UARTDATA_INVALID)
			continue;

		rx &= UARTDATA_MASK;
		tty_insert_flip_char(port, rx, flg);
	}

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);

	tty_flip_buffer_push(port);
	return IRQ_HANDLED;
}

static irqreturn_t lpuart_int(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	unsigned char sts, crdma;

	sts = readb(sport->port.membase + UARTSR1);
	crdma = readb(sport->port.membase + UARTCR5);

	if (sts & UARTSR1_RDRF && !(crdma & UARTCR5_RDMAS)) {
		if (sport->lpuart_dma_rx_use)
			lpuart_prepare_rx(sport);
		else
			lpuart_rxint(irq, dev_id);
	}

	if (sts & UARTSR1_TDRE && !sport->lpuart_dma_tx_use)
		lpuart_txint(irq, dev_id);

	return IRQ_HANDLED;
}

static irqreturn_t lpuart32_int(int irq, void *dev_id)
{
	struct lpuart_port *sport = dev_id;
	unsigned long sts, rxcount, crdma;

	sts = lpuart32_read(&sport->port, UARTSTAT);
	rxcount = lpuart32_read(&sport->port, UARTWATER);
	rxcount = rxcount >> UARTWATER_RXCNT_OFF;
	crdma = lpuart32_read(&sport->port,  UARTBAUD);

	if (!sts)
		return IRQ_NONE;

	if (!(crdma & UARTBAUD_RDMAE) && rxcount > 0) {
		if (!sport->lpuart_dma_rx_use ||
			(sts & (UARTSTAT_PE | UARTSTAT_NF | UARTSTAT_FE)))
			lpuart32_rxint(irq, dev_id);
		else if (sport->lpuart_dma_rx_use && sport->dma_rx_chan_active)
			lpuart_prepare_rx(sport);
	} else if (!(crdma & UARTBAUD_RDMAE) && (sts & UARTSTAT_IDLE) &&
			!(sport->lpuart_dma_rx_use && sport->dma_eeop &&
			rxcount > 0)) {
		lpuart32_write(&sport->port, UARTSTAT_IDLE, UARTSTAT);
	}

	if (sts & UARTSTAT_TDRE && !sport->lpuart_dma_tx_use)
		lpuart_txint(irq, dev_id);

	sts &= ~UARTSTAT_IDLE;
	lpuart32_write(&sport->port, sts, UARTSTAT);
	return IRQ_HANDLED;
}

static void lpuart_copy_rx_to_tty(struct lpuart_port *sport,
		struct tty_port *tty, int count)
{
	int copied;

	sport->port.icount.rx += count;

	if (!tty) {
		dev_err(sport->port.dev, "No tty port\n");
		return;
	}

	dma_sync_single_for_cpu(sport->port.dev, sport->dma_rx_buf_bus,
			sport->rxdma_len, DMA_FROM_DEVICE);
	copied = tty_insert_flip_string(tty,
			((unsigned char *)(sport->dma_rx_buf_virt)), count);

	if (copied != count)
		sport->port.icount.buf_overrun += count - copied;
	sport->port.icount.rx += copied;
}

static void lpuart_dma_stop(struct lpuart_port *sport, bool enable_pio)
{
	unsigned int temp;
	unsigned int crdma;

	if (lpuart_is_32(sport)) {
		lpuart32_write(&sport->port, UARTSTAT_IDLE, UARTSTAT);
		crdma = lpuart32_read(&sport->port, UARTBAUD);
		lpuart32_write(&sport->port, crdma & ~(UARTBAUD_RDMAE | UARTBAUD_RIDMAE),
				UARTBAUD);
		if (enable_pio) {
			temp = lpuart32_read(&sport->port, UARTCTRL);
			temp |= (UARTCTRL_RIE | UARTCTRL_ILIE);
			lpuart32_write(&sport->port, temp, UARTCTRL);
		}
	} else {
		temp = readb(sport->port.membase + UARTCR5);
		writeb(temp & ~UARTCR5_RDMAS, sport->port.membase + UARTCR5);
	}
}

static void lpuart_dma_rx_complete(void *arg)
{
	struct lpuart_port *sport = arg;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags;
	struct dma_tx_state state;
	int count, rxcount;

	if (!sport->dma_eeop)
		mod_timer(&sport->lpuart_timer,
			jiffies + sport->dma_rx_timeout);

	spin_lock_irqsave(&sport->port.lock, flags);
	sport->dma_rx_in_progress = false;
	dmaengine_tx_status(sport->dma_rx_chan, sport->dma_rx_cookie, &state);
	count = sport->rxdma_len - state.residue;
	spin_unlock_irqrestore(&sport->port.lock, flags);

	lpuart_copy_rx_to_tty(sport, port, count);
	tty_flip_buffer_push(port);

	spin_lock_irqsave(&sport->port.lock, flags);

	/* For end of packet, clear the idle flag to avoid to trigger
	 * the next transfer. Only i.MX8x lpuart support EEOP.
	 */
	if (sport->dma_eeop && lpuart_is_32(sport)) {
		rxcount = lpuart32_read(&sport->port, UARTWATER);
		rxcount = rxcount >> UARTWATER_RXCNT_OFF;
		if (!rxcount)
			lpuart32_write(&sport->port, UARTSTAT_IDLE, UARTSTAT);
	}

	if (!sport->dma_eeop && count < sport->rxfifo_watermark)
		lpuart_dma_stop(sport, true);
	else
		lpuart_dma_rx(sport);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void lpuart_timer_func(unsigned long data)
{
	struct lpuart_port *sport = (struct lpuart_port *)data;
	struct tty_port *port = &sport->port.state->port;
	struct dma_tx_state state;
	unsigned long flags;
	int count;

	del_timer(&sport->lpuart_timer);
	dmaengine_pause(sport->dma_rx_chan);
	dmaengine_tx_status(sport->dma_rx_chan, sport->dma_rx_cookie, &state);
	dmaengine_terminate_all(sport->dma_rx_chan);
	count = sport->rxdma_len - state.residue;

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->dma_rx_in_progress = false;
	lpuart_copy_rx_to_tty(sport, port, count);
	tty_flip_buffer_push(port);
	lpuart_dma_stop(sport, true);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int lpuart_dma_rx(struct lpuart_port *sport)
{
	dma_sync_single_for_device(sport->port.dev, sport->dma_rx_buf_bus,
			sport->rxdma_len, DMA_FROM_DEVICE);
	sport->dma_rx_desc = dmaengine_prep_slave_single(sport->dma_rx_chan,
			sport->dma_rx_buf_bus, sport->rxdma_len,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);

	if (!sport->dma_rx_desc) {
		dev_err(sport->port.dev, "Not able to get desc for rx\n");
		return -EIO;
	}

	sport->dma_rx_desc->callback = lpuart_dma_rx_complete;
	sport->dma_rx_desc->callback_param = sport;
	sport->dma_rx_in_progress = true;
	sport->dma_rx_cookie = dmaengine_submit(sport->dma_rx_desc);
	dma_async_issue_pending(sport->dma_rx_chan);

	return 0;
}

static void lpuart_dma_rx_free(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
					struct lpuart_port, port);

	sport->dma_rx_chan_active = false;
	dma_unmap_single(sport->port.dev, sport->dma_rx_buf_bus,
			sport->rxdma_len, DMA_FROM_DEVICE);

	devm_kfree(sport->port.dev, sport->dma_rx_buf_virt);
	sport->dma_rx_buf_bus = 0;
	sport->dma_rx_buf_virt = NULL;
}

static inline void lpuart_prepare_rx(struct lpuart_port *sport)
{
	unsigned long flags;
	unsigned int temp;
	unsigned int crdma;

	spin_lock_irqsave(&sport->port.lock, flags);

	if (!sport->dma_eeop) {
		sport->lpuart_timer.expires = jiffies + sport->dma_rx_timeout;
		add_timer(&sport->lpuart_timer);
	}

	lpuart_dma_rx(sport);
	if (lpuart_is_32(sport)) {
		temp = lpuart32_read(&sport->port, UARTCTRL);
		temp &= ~(UARTCTRL_RIE | UARTCTRL_ILIE);
		lpuart32_write(&sport->port, temp, UARTCTRL);
		crdma = lpuart32_read(&sport->port, UARTBAUD);
		if (sport->dma_eeop)
			crdma |= UARTBAUD_RIDMAE;
		lpuart32_write(&sport->port, crdma | UARTBAUD_RDMAE, UARTBAUD);
	} else {
		temp = readb(sport->port.membase + UARTCR5);
		writeb(temp | UARTCR5_RDMAS, sport->port.membase + UARTCR5);
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static int lpuart_config_rs485(struct uart_port *port,
			struct serial_rs485 *rs485)
{
	struct lpuart_port *sport = container_of(port,
			struct lpuart_port, port);

	u8 modem = readb(sport->port.membase + UARTMODEM) &
		~(UARTMODEM_TXRTSPOL | UARTMODEM_TXRTSE);
	writeb(modem, sport->port.membase + UARTMODEM);

	/* clear unsupported configurations */
	rs485->delay_rts_before_send = 0;
	rs485->delay_rts_after_send = 0;
	rs485->flags &= ~SER_RS485_RX_DURING_TX;

	if (rs485->flags & SER_RS485_ENABLED) {
		/* Enable auto RS-485 RTS mode */
		modem |= UARTMODEM_TXRTSE;

		/*
		 * RTS needs to be logic HIGH either during transer _or_ after
		 * transfer, other variants are not supported by the hardware.
		 */

		if (!(rs485->flags & (SER_RS485_RTS_ON_SEND |
				SER_RS485_RTS_AFTER_SEND)))
			rs485->flags |= SER_RS485_RTS_ON_SEND;

		if (rs485->flags & SER_RS485_RTS_ON_SEND &&
				rs485->flags & SER_RS485_RTS_AFTER_SEND)
			rs485->flags &= ~SER_RS485_RTS_AFTER_SEND;

		/*
		 * The hardware defaults to RTS logic HIGH while transfer.
		 * Switch polarity in case RTS shall be logic HIGH
		 * after transfer.
		 * Note: UART is assumed to be active high.
		 */
		if (rs485->flags & SER_RS485_RTS_ON_SEND)
			modem &= ~UARTMODEM_TXRTSPOL;
		else if (rs485->flags & SER_RS485_RTS_AFTER_SEND)
			modem |= UARTMODEM_TXRTSPOL;
	}

	/* Store the new configuration */
	sport->port.rs485 = *rs485;

	writeb(modem, sport->port.membase + UARTMODEM);
	return 0;
}

static unsigned int lpuart_get_mctrl(struct uart_port *port)
{
	unsigned int temp = 0;
	unsigned char reg;

	reg = readb(port->membase + UARTMODEM);
	if (reg & UARTMODEM_TXCTSE)
		temp |= TIOCM_CTS;

	if (reg & UARTMODEM_RXRTSE)
		temp |= TIOCM_RTS;

	return temp;
}

static unsigned int lpuart32_get_mctrl(struct uart_port *port)
{
	unsigned int temp = 0;
	unsigned long reg;

	reg = lpuart32_read(port, UARTMODIR);
	if (reg & UARTMODIR_TXCTSE)
		temp |= TIOCM_CTS;

	if (reg & UARTMODIR_RXRTSE)
		temp |= TIOCM_RTS;

	if (lpuart32_read(port, UARTCTRL) & UARTCTRL_LOOPS)
		temp |= TIOCM_LOOP;

	return temp;
}

static void lpuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* No flow control for user handle */
	unsigned char temp;
	struct lpuart_port *sport = container_of(port,
				struct lpuart_port, port);

	/* Make sure RXRTSE bit is not set when RS485 is enabled */
	if (!(sport->port.rs485.flags & SER_RS485_ENABLED)) {
		temp = readb(sport->port.membase + UARTMODEM) &
			~(UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);

		if (mctrl & TIOCM_RTS)
			temp |= UARTMODEM_RXRTSE;

		if (mctrl & TIOCM_CTS)
			temp |= UARTMODEM_TXCTSE;

		writeb(temp, port->membase + UARTMODEM);
	}
}

static void lpuart32_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned long temp;

	temp = lpuart32_read(port, UARTCTRL);
	if (mctrl & TIOCM_LOOP)
		temp |= UARTCTRL_LOOPS;
	else
		temp &= ~UARTCTRL_LOOPS;

	lpuart32_write(port, temp, UARTCTRL);
}

static void lpuart_break_ctl(struct uart_port *port, int break_state)
{
	unsigned char temp;

	temp = readb(port->membase + UARTCR2) & ~UARTCR2_SBK;

	if (break_state != 0)
		temp |= UARTCR2_SBK;

	writeb(temp, port->membase + UARTCR2);
}

static void lpuart32_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long temp;

	temp = lpuart32_read(port, UARTCTRL) & ~UARTCTRL_SBK;

	if (break_state != 0)
		temp |= UARTCTRL_SBK;

	lpuart32_write(port, temp, UARTCTRL);
}

static void lpuart_setup_watermark(struct lpuart_port *sport)
{
	unsigned char val, cr2;
	unsigned char cr2_saved;

	cr2 = readb(sport->port.membase + UARTCR2);
	cr2_saved = cr2;
	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_TE |
			UARTCR2_RIE | UARTCR2_RE);
	writeb(cr2, sport->port.membase + UARTCR2);

	val = readb(sport->port.membase + UARTPFIFO);
	writeb(val | UARTPFIFO_TXFE | UARTPFIFO_RXFE,
			sport->port.membase + UARTPFIFO);

	/* flush Tx and Rx FIFO */
	writeb(UARTCFIFO_TXFLUSH | UARTCFIFO_RXFLUSH,
			sport->port.membase + UARTCFIFO);

	/* explicitly clear RDRF */
	if (readb(sport->port.membase + UARTSR1) & UARTSR1_RDRF) {
		readb(sport->port.membase + UARTDR);
		writeb(UARTSFIFO_RXUF, sport->port.membase + UARTSFIFO);
	}

	writeb(0, sport->port.membase + UARTTWFIFO);
	writeb(1, sport->port.membase + UARTRWFIFO);

	/* Restore cr2 */
	writeb(cr2_saved, sport->port.membase + UARTCR2);
}

static void lpuart32_setup_watermark(struct lpuart_port *sport)
{
	unsigned long val, ctrl;
	unsigned long ctrl_saved;
	unsigned long rxiden_cnt = UARTFIFO_RXIDEN_RDRF;

	ctrl = lpuart32_read(&sport->port, UARTCTRL);
	ctrl_saved = ctrl;
	ctrl &= ~(UARTCTRL_TIE | UARTCTRL_TCIE | UARTCTRL_TE |
			UARTCTRL_RIE | UARTCTRL_RE);
	lpuart32_write(&sport->port, ctrl, UARTCTRL);

	/* enable FIFO mode */
	val = lpuart32_read(&sport->port, UARTFIFO);
	val |= UARTFIFO_TXFE | UARTFIFO_RXFE;
	val |= UARTFIFO_TXFLUSH | UARTFIFO_RXFLUSH;
	val &= ~(UARTFIFO_RXIDEN_MASK << UARTFIFO_RXIDEN_OFF);
	if (sport->dma_eeop)
		rxiden_cnt = 0;
	val |= ((rxiden_cnt & UARTFIFO_RXIDEN_MASK) <<
		UARTFIFO_RXIDEN_OFF);
	lpuart32_write(&sport->port, val, UARTFIFO);

	/* set the watermark */
	if (uart_console(&sport->port)) {
		val = (0x1 << UARTWATER_RXWATER_OFF) |
			(0x0 << UARTWATER_TXWATER_OFF);
	} else {
		val = lpuart32_read(&sport->port, UARTMODIR);
		val = sport->rts_watermark << UARTMODIR_RTSWATER_S;
		lpuart32_write(&sport->port, val, UARTMODIR);
		val = (sport->rxfifo_watermark << UARTWATER_RXWATER_OFF) |
			(sport->txfifo_watermark << UARTWATER_TXWATER_OFF);
	}
	lpuart32_write(&sport->port, val, UARTWATER);

	/* Restore cr2 */
	lpuart32_write(&sport->port, ctrl_saved, UARTCTRL);
}

static int lpuart_dma_rx_request(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port,
					struct lpuart_port, port);
	struct dma_slave_config dma_rx_sconfig;
	dma_addr_t dma_bus;
	unsigned char *dma_buf;
	int ret;

	dma_buf = devm_kzalloc(sport->port.dev,
			       sport->rxdma_len, GFP_KERNEL);

	if (!dma_buf) {
		dev_err(sport->port.dev, "Dma rx alloc failed\n");
		return -ENOMEM;
	}

	dma_bus = dma_map_single(sport->port.dev, dma_buf,
				 sport->rxdma_len, DMA_FROM_DEVICE);

	if (dma_mapping_error(sport->port.dev, dma_bus)) {
		dev_err(sport->port.dev, "dma_map_single rx failed\n");
		return -ENOMEM;
	}

	if (lpuart_is_32(sport))
		dma_rx_sconfig.src_addr = sport->port.mapbase + UARTDATA;
	else
		dma_rx_sconfig.src_addr = sport->port.mapbase + UARTDR;

	dma_rx_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_rx_sconfig.src_maxburst = 1;
	dma_rx_sconfig.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(sport->dma_rx_chan, &dma_rx_sconfig);

	if (ret < 0) {
		dev_err(sport->port.dev,
				"Dma slave config failed, err = %d\n", ret);
		return ret;
	}

	sport->dma_rx_buf_virt = dma_buf;
	sport->dma_rx_buf_bus = dma_bus;
	sport->dma_rx_in_progress = false;
	sport->dma_rx_chan_active = true;

	return 0;
}

static int lpuart_startup(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	unsigned long flags;
	unsigned char temp;

	/* determine FIFO size and enable FIFO mode */
	temp = readb(sport->port.membase + UARTPFIFO);

	sport->txfifo_size = 0x1 << (((temp >> UARTPFIFO_TXSIZE_OFF) &
		UARTPFIFO_FIFOSIZE_MASK) + 1);

	sport->port.fifosize = sport->txfifo_size;

	sport->rxfifo_size = 0x1 << (((temp >> UARTPFIFO_RXSIZE_OFF) &
		UARTPFIFO_FIFOSIZE_MASK) + 1);
	sport->rxdma_len = FSL_UART_RX_DMA_BUFFER_SIZE;

	if (sport->dma_rx_chan && !lpuart_dma_rx_request(port)) {
		sport->lpuart_dma_rx_use = true;
		setup_timer(&sport->lpuart_timer, lpuart_timer_func,
			    (unsigned long)sport);
	} else
		sport->lpuart_dma_rx_use = false;


	if (sport->dma_tx_chan && !lpuart_dma_tx_request(port)) {
		init_waitqueue_head(&sport->dma_wait);
		sport->lpuart_dma_tx_use = true;
		temp = readb(port->membase + UARTCR5);
		temp &= ~UARTCR5_RDMAS;
		writeb(temp | UARTCR5_TDMAS, port->membase + UARTCR5);
	} else
		sport->lpuart_dma_tx_use = false;

	spin_lock_irqsave(&sport->port.lock, flags);

	lpuart_setup_watermark(sport);

	temp = readb(sport->port.membase + UARTCR2);
	temp |= (UARTCR2_RIE | UARTCR2_RE | UARTCR2_TE);
	writeb(temp, sport->port.membase + UARTCR2);

	spin_unlock_irqrestore(&sport->port.lock, flags);
	return 0;
}

static int lpuart32_startup(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	struct tty_port *tty_port = &sport->port.state->port;
	unsigned long flags;
	unsigned long temp;
	int ret;

	/* some modem may need reset */
	if (!tty_port_suspended(tty_port)) {
		ret = device_reset(sport->port.dev);
		if (ret && ret != -ENOENT)
			return ret;
	}

	/* determine FIFO size */
	temp = lpuart32_read(&sport->port, UARTFIFO);

	sport->txfifo_size = 0x1 << (((temp >> UARTFIFO_TXSIZE_OFF) &
		UARTFIFO_FIFOSIZE_MASK) + 1);

	sport->port.fifosize = sport->txfifo_size;

	sport->rxfifo_size = 0x1 << (((temp >> UARTFIFO_RXSIZE_OFF) &
		UARTFIFO_FIFOSIZE_MASK) + 1);

	sport->txfifo_watermark = sport->txfifo_size >> 1;
	sport->rxfifo_watermark = 1;
	sport->rts_watermark = sport->rxfifo_size >> 1;
	sport->rxdma_len = FSL_UART_RX_DMA_BUFFER_SIZE;

	if (sport->dma_rx_chan && !lpuart_dma_rx_request(port)) {
		sport->lpuart_dma_rx_use = true;
		if (!sport->dma_eeop)
			setup_timer(&sport->lpuart_timer,
				    lpuart_timer_func,
				    (unsigned long)sport);
	} else
		sport->lpuart_dma_rx_use = false;


	if (sport->dma_tx_chan && !lpuart_dma_tx_request(port)) {
		init_waitqueue_head(&sport->dma_wait);
		sport->lpuart_dma_tx_use = true;
		temp = lpuart32_read(&sport->port, UARTBAUD);
		temp |= UARTBAUD_TDMAE;
		lpuart32_write(&sport->port, temp, UARTBAUD);
	} else
		sport->lpuart_dma_tx_use = false;

	spin_lock_irqsave(&sport->port.lock, flags);

	lpuart32_setup_watermark(sport);

	temp = lpuart32_read(&sport->port, UARTCTRL);
	temp |= (UARTCTRL_RIE | UARTCTRL_RE | UARTCTRL_TE);
	temp |= UARTCTRL_ILIE;
	temp |= UARTCTRL_IDLECFG << UARTCTRL_IDLECFG_OFF;
	lpuart32_write(&sport->port, temp, UARTCTRL);

	spin_unlock_irqrestore(&sport->port.lock, flags);
	return 0;
}

static void lpuart_shutdown(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	unsigned char temp;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&port->lock, flags);

	/* disable Rx/Tx and interrupts */
	temp = readb(port->membase + UARTCR2);
	temp &= ~(UARTCR2_TE | UARTCR2_RE |
			UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE);
	writeb(temp, port->membase + UARTCR2);

	spin_unlock_irqrestore(&port->lock, flags);

	if (sport->lpuart_dma_rx_use) {
		sport->dma_rx_in_progress = false;
		dmaengine_terminate_all(sport->dma_rx_chan);
		del_timer_sync(&sport->lpuart_timer);
		lpuart_dma_rx_free(&sport->port);
	}

	if (sport->lpuart_dma_tx_use) {
		ret = wait_event_interruptible_timeout(sport->dma_wait,
			!sport->dma_tx_in_progress, msecs_to_jiffies(1));
		if (ret <= 0) {
			sport->dma_tx_in_progress = false;
			dmaengine_terminate_all(sport->dma_tx_chan);
		}
	}
}

static void lpuart32_shutdown(struct uart_port *port)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	unsigned long temp;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&port->lock, flags);

	/* clear statue */
	temp = lpuart32_read(&sport->port, UARTSTAT);
	lpuart32_write(&sport->port, temp, UARTSTAT);

	/* disable Rx/Tx DMA */
	temp = lpuart32_read(port, UARTBAUD);
	temp &= ~(UARTBAUD_TDMAE | UARTBAUD_RDMAE | UARTBAUD_RIDMAE);
	lpuart32_write(port, temp, UARTBAUD);

	/* disable Rx/Tx and interrupts */
	temp = lpuart32_read(port, UARTCTRL);
	temp &= ~(UARTCTRL_TE | UARTCTRL_RE | UARTCTRL_TIE |
		UARTCTRL_TCIE | UARTCTRL_RIE | UARTCTRL_ILIE |
		UARTCTRL_LOOPS);
	lpuart32_write(port, temp, UARTCTRL);
	lpuart32_write(port, 0, UARTMODIR);

	spin_unlock_irqrestore(&port->lock, flags);

	if (sport->lpuart_dma_rx_use) {
		sport->dma_rx_in_progress = false;
		dmaengine_terminate_all(sport->dma_rx_chan);
		if (!sport->dma_eeop)
			del_timer_sync(&sport->lpuart_timer);
		lpuart_dma_rx_free(&sport->port);
	}

	if (sport->lpuart_dma_tx_use) {
		ret = wait_event_interruptible_timeout(sport->dma_wait,
			!sport->dma_tx_in_progress, msecs_to_jiffies(1));
		if (ret <= 0) {
			sport->dma_tx_in_progress = false;
			dmaengine_terminate_all(sport->dma_tx_chan);
		}
	}
}

static void
lpuart_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	unsigned long flags;
	unsigned char cr1, old_cr1, old_cr2, cr3, cr4, bdh, modem;
	unsigned int  baud;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	unsigned int sbr, brfa;

	cr1 = old_cr1 = readb(sport->port.membase + UARTCR1);
	old_cr2 = readb(sport->port.membase + UARTCR2);
	cr3 = readb(sport->port.membase + UARTCR3);
	cr4 = readb(sport->port.membase + UARTCR4);
	bdh = readb(sport->port.membase + UARTBDH);
	modem = readb(sport->port.membase + UARTMODEM);
	/*
	 * only support CS8 and CS7, and for CS7 must enable PE.
	 * supported mode:
	 *  - (7,e/o,1)
	 *  - (8,n,1)
	 *  - (8,m/s,1)
	 *  - (8,e/o,1)
	 */
	while ((termios->c_cflag & CSIZE) != CS8 &&
		(termios->c_cflag & CSIZE) != CS7) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8 ||
		(termios->c_cflag & CSIZE) == CS7)
		cr1 = old_cr1 & ~UARTCR1_M;

	if (termios->c_cflag & CMSPAR) {
		if ((termios->c_cflag & CSIZE) != CS8) {
			termios->c_cflag &= ~CSIZE;
			termios->c_cflag |= CS8;
		}
		cr1 |= UARTCR1_M;
	}

	/*
	 * When auto RS-485 RTS mode is enabled,
	 * hardware flow control need to be disabled.
	 */
	if (sport->port.rs485.flags & SER_RS485_ENABLED)
		termios->c_cflag &= ~CRTSCTS;

	if (termios->c_cflag & CRTSCTS) {
		modem |= (UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
	} else {
		termios->c_cflag &= ~CRTSCTS;
		modem &= ~(UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
	}

	if (termios->c_cflag & CSTOPB)
		termios->c_cflag &= ~CSTOPB;

	/* parity must be enabled when CS7 to match 8-bits format */
	if ((termios->c_cflag & CSIZE) == CS7)
		termios->c_cflag |= PARENB;

	if ((termios->c_cflag & PARENB)) {
		if (termios->c_cflag & CMSPAR) {
			cr1 &= ~UARTCR1_PE;
			if (termios->c_cflag & PARODD)
				cr3 |= UARTCR3_T8;
			else
				cr3 &= ~UARTCR3_T8;
		} else {
			cr1 |= UARTCR1_PE;
			if ((termios->c_cflag & CSIZE) == CS8)
				cr1 |= UARTCR1_M;
			if (termios->c_cflag & PARODD)
				cr1 |= UARTCR1_PT;
			else
				cr1 &= ~UARTCR1_PT;
		}
	}

	/* ask the core to calculate the divisor */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);

	/*
	 * Need to update the Ring buffer length according to the selected
	 * baud rate and restart Rx DMA path.
	 *
	 * Since timer function acqures sport->port.lock, need to stop before
	 * acquring same lock because otherwise del_timer_sync() can deadlock.
	 */
	if (old && sport->lpuart_dma_rx_use) {
		del_timer_sync(&sport->lpuart_timer);
		lpuart_dma_rx_free(&sport->port);
	}

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |=	(UARTSR1_FE | UARTSR1_PE);
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		sport->port.read_status_mask |= UARTSR1_FE;

	/* characters to ignore */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |= UARTSR1_PE;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= UARTSR1_FE;
		/*
		 * if we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= UARTSR1_OR;
	}

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	if (sport->lpuart_dma_rx_use) {
		/* Calculate delay for 1.5 DMA buffers */
		sport->dma_rx_timeout = (sport->port.timeout - HZ / 50) *
					sport->rxdma_len * 3 /
					sport->rxfifo_size / 2;
		dev_dbg(port->dev, "DMA Rx t-out %ums, tty t-out %u jiffies\n",
			sport->dma_rx_timeout * 1000 / HZ, sport->port.timeout);
		if (sport->dma_rx_timeout < msecs_to_jiffies(20))
			sport->dma_rx_timeout = msecs_to_jiffies(20);
	}

	/* wait transmit engin complete */
	while (!(readb(sport->port.membase + UARTSR1) & UARTSR1_TC))
		barrier();

	/* disable transmit and receive */
	writeb(old_cr2 & ~(UARTCR2_TE | UARTCR2_RE),
			sport->port.membase + UARTCR2);

	sbr = sport->port.uartclk / (16 * baud);
	brfa = ((sport->port.uartclk - (16 * sbr * baud)) * 2) / baud;
	bdh &= ~UARTBDH_SBR_MASK;
	bdh |= (sbr >> 8) & 0x1F;
	cr4 &= ~UARTCR4_BRFA_MASK;
	brfa &= UARTCR4_BRFA_MASK;
	writeb(cr4 | brfa, sport->port.membase + UARTCR4);
	writeb(bdh, sport->port.membase + UARTBDH);
	writeb(sbr & 0xFF, sport->port.membase + UARTBDL);
	writeb(cr3, sport->port.membase + UARTCR3);
	writeb(cr1, sport->port.membase + UARTCR1);
	writeb(modem, sport->port.membase + UARTMODEM);

	/* restore control register */
	writeb(old_cr2, sport->port.membase + UARTCR2);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void
lpuart32_serial_setbrg(struct lpuart_port *sport, unsigned int baudrate)
{
	u32 sbr, osr, baud_diff, tmp_osr, tmp_sbr, tmp_diff, tmp;
	u32 clk = sport->port.uartclk;

	/*
	 * The idea is to use the best OSR (over-sampling rate) possible.
	 * Note, OSR is typically hard-set to 16 in other LPUART instantiations.
	 * Loop to find the best OSR value possible, one that generates minimum
	 * baud_diff iterate through the rest of the supported values of OSR.
	 *
	 * Calculation Formula:
	 *  Baud Rate = baud clock / ((OSR+1) × SBR)
	 */
	baud_diff = baudrate;
	osr = 0;
	sbr = 0;

	for (tmp_osr = 4; tmp_osr <= 32; tmp_osr++) {
		/* calculate the temporary sbr value  */
		tmp_sbr = (clk / (baudrate * tmp_osr));
		if (tmp_sbr == 0)
			tmp_sbr = 1;

		/*
		 * calculate the baud rate difference based on the temporary
		 * osr and sbr values
		 */
		tmp_diff = clk / (tmp_osr * tmp_sbr) - baudrate;

		/* select best values between sbr and sbr+1 */
		tmp = clk / (tmp_osr * (tmp_sbr + 1));
		if (tmp_diff > (baudrate - tmp)) {
			tmp_diff = baudrate - tmp;
			tmp_sbr++;
		}

		if (tmp_diff <= baud_diff) {
			baud_diff = tmp_diff;
			osr = tmp_osr;
			sbr = tmp_sbr;

			if (!baud_diff)
				break;
		}
	}

	/* handle baudrate outside acceptable rate */
	if (baud_diff > ((baudrate / 100) * 3))
		dev_warn(sport->port.dev,
			 "unacceptable baud rate difference of more than 3%%\n");

	tmp = lpuart32_read(&sport->port, UARTBAUD);

	if ((osr > 3) && (osr < 8))
		tmp |= UARTBAUD_BOTHEDGE;

	tmp &= ~(UARTBAUD_OSR_MASK << UARTBAUD_OSR_SHIFT);
	tmp |= (((osr-1) & UARTBAUD_OSR_MASK) << UARTBAUD_OSR_SHIFT);

	tmp &= ~UARTBAUD_SBR_MASK;
	tmp |= sbr & UARTBAUD_SBR_MASK;

	lpuart32_write(&sport->port, tmp, UARTBAUD);
}

static void
lpuart32_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
	struct lpuart_port *sport = container_of(port, struct lpuart_port, port);
	unsigned long flags;
	unsigned long ctrl, old_ctrl, bd, modem;
	unsigned int  baud;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;

	ctrl = old_ctrl = lpuart32_read(&sport->port, UARTCTRL);
	bd = lpuart32_read(&sport->port, UARTBAUD);
	modem = lpuart32_read(&sport->port, UARTMODIR);
	/*
	 * only support CS8 and CS7, and for CS7 must enable PE.
	 * supported mode:
	 *  - (7,e/o,1)
	 *  - (8,n,1)
	 *  - (8,m/s,1)
	 *  - (8,e/o,1)
	 */
	while ((termios->c_cflag & CSIZE) != CS8 &&
		(termios->c_cflag & CSIZE) != CS7) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS8 ||
		(termios->c_cflag & CSIZE) == CS7)
		ctrl = old_ctrl & ~UARTCTRL_M;

	if (termios->c_cflag & CMSPAR) {
		if ((termios->c_cflag & CSIZE) != CS8) {
			termios->c_cflag &= ~CSIZE;
			termios->c_cflag |= CS8;
		}
		ctrl |= UARTCTRL_M;
	}

	if (termios->c_cflag & CRTSCTS) {
		modem |= (UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
	} else {
		termios->c_cflag &= ~CRTSCTS;
		modem &= ~(UARTMODEM_RXRTSE | UARTMODEM_TXCTSE);
	}

	if (termios->c_cflag & CSTOPB)
		bd |= UARTBAUD_SBNS;
	else
		bd &= ~UARTBAUD_SBNS;

	/* parity must be enabled when CS7 to match 8-bits format */
	if ((termios->c_cflag & CSIZE) == CS7)
		termios->c_cflag |= PARENB;

	if ((termios->c_cflag & PARENB)) {
		if (termios->c_cflag & CMSPAR) {
			ctrl &= ~UARTCTRL_PE;
			ctrl |= UARTCTRL_M;
		} else {
			ctrl |= UARTCR1_PE;
			if ((termios->c_cflag & CSIZE) == CS8)
				ctrl |= UARTCTRL_M;
			if (termios->c_cflag & PARODD)
				ctrl |= UARTCTRL_PT;
			else
				ctrl &= ~UARTCTRL_PT;
		}
	}

	/* ask the core to calculate the divisor */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 4);

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->port.read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |=	(UARTSTAT_FE | UARTSTAT_PE);
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		sport->port.read_status_mask |= UARTSTAT_FE;

	/* characters to ignore */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |= UARTSTAT_PE;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= UARTSTAT_FE;
		/*
		 * if we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= UARTSTAT_OR;
	}

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	if (sport->lpuart_dma_rx_use && !sport->dma_eeop) {
		/* Calculate delay for 1.5 DMA buffers */
		sport->dma_rx_timeout = (sport->port.timeout - HZ / 50) *
					sport->rxdma_len * 3 /
					sport->rxfifo_size / 2;
		dev_dbg(port->dev, "DMA Rx t-out %ums, tty t-out %u jiffies\n",
			sport->dma_rx_timeout * 1000 / HZ, sport->port.timeout);
		if (sport->dma_rx_timeout < msecs_to_jiffies(20))
			sport->dma_rx_timeout = msecs_to_jiffies(20);
	}

	/* wait transmit engin complete, there disable flow control */
	lpuart32_write(&sport->port, 0, UARTMODIR);
	while (!(lpuart32_read(&sport->port, UARTSTAT) & UARTSTAT_TC))
		barrier();

	/* disable transmit and receive */
	lpuart32_write(&sport->port, old_ctrl & ~(UARTCTRL_TE | UARTCTRL_RE),
			UARTCTRL);

	lpuart32_write(&sport->port, bd, UARTBAUD);
	lpuart32_serial_setbrg(sport, baud);
	lpuart32_write(&sport->port, modem, UARTMODIR);
	lpuart32_write(&sport->port, ctrl, UARTCTRL);

	spin_unlock_irqrestore(&sport->port.lock, flags);

	/* wait baud rate stable */
	usleep_range(1000, 2000);
}

static const char *lpuart_type(struct uart_port *port)
{
	return "FSL_LPUART";
}

static void lpuart_release_port(struct uart_port *port)
{
	/* nothing to do */
}

static int lpuart_request_port(struct uart_port *port)
{
	return  0;
}

/* configure/autoconfigure the port */
static void lpuart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_LPUART;
}

static int lpuart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_LPUART)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static const struct uart_ops lpuart_pops = {
	.tx_empty	= lpuart_tx_empty,
	.set_mctrl	= lpuart_set_mctrl,
	.get_mctrl	= lpuart_get_mctrl,
	.stop_tx	= lpuart_stop_tx,
	.start_tx	= lpuart_start_tx,
	.stop_rx	= lpuart_stop_rx,
	.break_ctl	= lpuart_break_ctl,
	.startup	= lpuart_startup,
	.shutdown	= lpuart_shutdown,
	.pm		= lpuart_uart_pm,
	.set_termios	= lpuart_set_termios,
	.type		= lpuart_type,
	.request_port	= lpuart_request_port,
	.release_port	= lpuart_release_port,
	.config_port	= lpuart_config_port,
	.verify_port	= lpuart_verify_port,
	.flush_buffer	= lpuart_flush_buffer,
#if defined(CONFIG_CONSOLE_POLL)
	.poll_init	= lpuart_poll_init,
	.poll_get_char	= lpuart_poll_get_char,
	.poll_put_char	= lpuart_poll_put_char,
#endif
};

static const struct uart_ops lpuart32_pops = {
	.tx_empty	= lpuart32_tx_empty,
	.set_mctrl	= lpuart32_set_mctrl,
	.get_mctrl	= lpuart32_get_mctrl,
	.stop_tx	= lpuart32_stop_tx,
	.start_tx	= lpuart32_start_tx,
	.stop_rx	= lpuart32_stop_rx,
	.break_ctl	= lpuart32_break_ctl,
	.startup	= lpuart32_startup,
	.shutdown	= lpuart32_shutdown,
	.pm		= lpuart_uart_pm,
	.set_termios	= lpuart32_set_termios,
	.type		= lpuart_type,
	.request_port	= lpuart_request_port,
	.release_port	= lpuart_release_port,
	.config_port	= lpuart_config_port,
	.verify_port	= lpuart_verify_port,
	.flush_buffer	= lpuart_flush_buffer,
#if defined(CONFIG_CONSOLE_POLL)
	.poll_init	= lpuart32_poll_init,
	.poll_get_char	= lpuart32_poll_get_char,
	.poll_put_char	= lpuart32_poll_put_char,
#endif
};

static struct lpuart_port *lpuart_ports[UART_NR];

#ifdef CONFIG_SERIAL_FSL_LPUART_CONSOLE
static void lpuart_console_putchar(struct uart_port *port, int ch)
{
	while (!(readb(port->membase + UARTSR1) & UARTSR1_TDRE))
		barrier();

	writeb(ch, port->membase + UARTDR);
}

static void lpuart32_console_putchar(struct uart_port *port, int ch)
{
	while (!(lpuart32_read(port, UARTSTAT) & UARTSTAT_TDRE))
		barrier();

	lpuart32_write(port, ch, UARTDATA);
}

static void
lpuart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct lpuart_port *sport = lpuart_ports[co->index];
	unsigned char  old_cr2, cr2;
	unsigned long flags;
	int locked = 1;

	if (sport->port.sysrq || oops_in_progress)
		locked = spin_trylock_irqsave(&sport->port.lock, flags);
	else
		spin_lock_irqsave(&sport->port.lock, flags);

	/* first save CR2 and then disable interrupts */
	cr2 = old_cr2 = readb(sport->port.membase + UARTCR2);
	cr2 |= (UARTCR2_TE |  UARTCR2_RE);
	cr2 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE);
	writeb(cr2, sport->port.membase + UARTCR2);

	uart_console_write(&sport->port, s, count, lpuart_console_putchar);

	/* wait for transmitter finish complete and restore CR2 */
	while (!(readb(sport->port.membase + UARTSR1) & UARTSR1_TC))
		barrier();

	writeb(old_cr2, sport->port.membase + UARTCR2);

	if (locked)
		spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void
lpuart32_console_write(struct console *co, const char *s, unsigned int count)
{
	struct lpuart_port *sport = lpuart_ports[co->index];
	unsigned long  old_cr, cr;
	unsigned long flags;
	int locked = 1;

	if (sport->port.sysrq || oops_in_progress)
		locked = spin_trylock_irqsave(&sport->port.lock, flags);
	else
		spin_lock_irqsave(&sport->port.lock, flags);

	/* first save CR2 and then disable interrupts */
	cr = old_cr = lpuart32_read(&sport->port, UARTCTRL);
	cr |= (UARTCTRL_TE |  UARTCTRL_RE);
	cr &= ~(UARTCTRL_TIE | UARTCTRL_TCIE | UARTCTRL_RIE);
	lpuart32_write(&sport->port, cr, UARTCTRL);

	uart_console_write(&sport->port, s, count, lpuart32_console_putchar);

	/* wait for transmitter finish complete and restore CR2 */
	while (!(lpuart32_read(&sport->port, UARTSTAT) & UARTSTAT_TC))
		barrier();

	lpuart32_write(&sport->port, old_cr, UARTCTRL);

	if (locked)
		spin_unlock_irqrestore(&sport->port.lock, flags);
}

/*
 * if the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
lpuart_console_get_options(struct lpuart_port *sport, int *baud,
			   int *parity, int *bits)
{
	unsigned char cr, bdh, bdl, brfa;
	unsigned int sbr, uartclk, baud_raw;

	cr = readb(sport->port.membase + UARTCR2);
	cr &= UARTCR2_TE | UARTCR2_RE;
	if (!cr)
		return;

	/* ok, the port was enabled */

	cr = readb(sport->port.membase + UARTCR1);

	*parity = 'n';
	if (cr & UARTCR1_PE) {
		if (cr & UARTCR1_PT)
			*parity = 'o';
		else
			*parity = 'e';
	}

	if (cr & UARTCR1_M)
		*bits = 9;
	else
		*bits = 8;

	bdh = readb(sport->port.membase + UARTBDH);
	bdh &= UARTBDH_SBR_MASK;
	bdl = readb(sport->port.membase + UARTBDL);
	sbr = bdh;
	sbr <<= 8;
	sbr |= bdl;
	brfa = readb(sport->port.membase + UARTCR4);
	brfa &= UARTCR4_BRFA_MASK;

	if (sport->per_clk)
		uartclk = clk_get_rate(sport->per_clk);
	else
		uartclk = clk_get_rate(sport->ipg_clk);
	/*
	 * baud = mod_clk/(16*(sbr[13]+(brfa)/32)
	 */
	baud_raw = uartclk / (16 * (sbr + brfa / 32));

	if (*baud != baud_raw)
		printk(KERN_INFO "Serial: Console lpuart rounded baud rate"
				"from %d to %d\n", baud_raw, *baud);
}

static void __init
lpuart32_console_get_options(struct lpuart_port *sport, int *baud,
			   int *parity, int *bits)
{
	unsigned long cr, bd;
	unsigned int sbr, osr, uartclk, baud_raw;

	cr = lpuart32_read(&sport->port, UARTCTRL);
	cr &= UARTCTRL_TE | UARTCTRL_RE;
	if (!cr)
		return;

	/* ok, the port was enabled */

	cr = lpuart32_read(&sport->port, UARTCTRL);

	*parity = 'n';
	if (cr & UARTCTRL_PE) {
		if (cr & UARTCTRL_PT)
			*parity = 'o';
		else
			*parity = 'e';
	}

	if (cr & UARTCTRL_M)
		*bits = 9;
	else
		*bits = 8;

	bd = lpuart32_read(&sport->port, UARTBAUD);
	bd &= UARTBAUD_SBR_MASK;
	sbr = bd;
	osr = (bd >> UARTBAUD_OSR_SHIFT) & UARTBAUD_OSR_MASK;
	if (sport->per_clk)
		uartclk = clk_get_rate(sport->per_clk);
	else
		uartclk = clk_get_rate(sport->ipg_clk);

	baud_raw = uartclk / ((osr + 1) * sbr);
	if (*baud != baud_raw)
		printk(KERN_INFO "Serial: Console lpuart rounded baud rate"
				"from %d to %d\n", baud_raw, *baud);
}

static int __init lpuart_console_setup(struct console *co, char *options)
{
	struct lpuart_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(lpuart_ports))
		co->index = 0;

	sport = lpuart_ports[co->index];
	if (sport == NULL)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		if (lpuart_is_32(sport))
			lpuart32_console_get_options(sport, &baud, &parity, &bits);
		else
			lpuart_console_get_options(sport, &baud, &parity, &bits);

	if (lpuart_is_32(sport))
		lpuart32_setup_watermark(sport);
	else
		lpuart_setup_watermark(sport);

	return uart_set_options(&sport->port, co, baud, parity, bits, flow);
}

static struct uart_driver lpuart_reg;
static struct console lpuart_console = {
	.name		= DEV_NAME,
	.write		= lpuart_console_write,
	.device		= uart_console_device,
	.setup		= lpuart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &lpuart_reg,
};

static struct console lpuart32_console = {
	.name		= DEV_NAME,
	.write		= lpuart32_console_write,
	.device		= uart_console_device,
	.setup		= lpuart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &lpuart_reg,
};

static void lpuart_early_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, lpuart_console_putchar);
}

static void lpuart32_early_write(struct console *con, const char *s, unsigned n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, lpuart32_console_putchar);
}

static int __init lpuart_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = lpuart_early_write;
	return 0;
}

static int __init lpuart32_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->port.iotype = UPIO_MEM32BE;
	device->con->write = lpuart32_early_write;
	return 0;
}

static int __init lpuart32_imx_early_console_setup(struct earlycon_device *device,
						   const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->port.iotype = UPIO_MEM32;
	device->port.membase += IMX_REG_OFF;
	device->con->write = lpuart32_early_write;

	return 0;
}
// make lpuart32 mean imx:
OF_EARLYCON_DECLARE(lpuart, "fsl,vf610-lpuart", lpuart_early_console_setup);
OF_EARLYCON_DECLARE(lpuart32ls, "fsl,ls1021a-lpuart", lpuart32_early_console_setup);
OF_EARLYCON_DECLARE(lpuart32, "fsl,imx7ulp-lpuart", lpuart32_imx_early_console_setup);
OF_EARLYCON_DECLARE(lpuart32, "fsl,imx8qm-lpuart", lpuart32_imx_early_console_setup);
EARLYCON_DECLARE(lpuart, lpuart_early_console_setup);
EARLYCON_DECLARE(lpuart32, lpuart32_imx_early_console_setup);

#define LPUART_CONSOLE	(&lpuart_console)
#define LPUART32_CONSOLE	(&lpuart32_console)
#else
#define LPUART_CONSOLE	NULL
#define LPUART32_CONSOLE	NULL
#endif

static struct uart_driver lpuart_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= DEV_NAME,
	.nr		= ARRAY_SIZE(lpuart_ports),
	.cons		= LPUART_CONSOLE,
};

static int lpuart_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id = of_match_device(lpuart_dt_ids,
							   &pdev->dev);
	const struct lpuart_soc_data *sdata = of_id->data;
	struct device_node *np = pdev->dev.of_node;
	struct lpuart_port *sport;
	struct resource *res;
	unsigned long cr_32;
	unsigned char cr_8;
	int ret;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}
	if (ret >= ARRAY_SIZE(lpuart_ports)) {
		dev_err(&pdev->dev, "serial%d out of range\n", ret);
		return -EINVAL;
	}
	sport->port.line = ret;
	sport->dma_eeop = of_device_is_compatible(np, "fsl,imx8qm-lpuart");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sport->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sport->port.membase))
		return PTR_ERR(sport->port.membase);

	sport->regbase = sport->port.membase;
	sport->port.membase += sdata->reg_off;
	sport->port.mapbase = res->start + sdata->reg_off;
	sport->port.dev = &pdev->dev;
	sport->port.type = PORT_LPUART;
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot obtain irq\n");
		return ret;
	}
	sport->port.irq = ret;
	sport->port.iotype = sdata->iotype;
	if (lpuart_is_32(sport))
		sport->port.ops = &lpuart32_pops;
	else
		sport->port.ops = &lpuart_pops;
	sport->port.flags = UPF_BOOT_AUTOCONF;

	if (!lpuart_is_32(sport))
		sport->port.rs485_config = lpuart_config_rs485;

	sport->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(sport->ipg_clk)) {
		ret = PTR_ERR(sport->ipg_clk);
		dev_err(&pdev->dev, "failed to get ipg clk: %d\n", ret);
		return ret;
	}
	sport->per_clk = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(sport->per_clk))
		sport->per_clk = NULL;

	ret = clk_prepare_enable(sport->ipg_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable uart ipg clk: %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(sport->per_clk);
	if (ret) {
		clk_disable_unprepare(sport->ipg_clk);
		dev_err(&pdev->dev, "failed to enable uart clk: %d\n", ret);
		return ret;
	}
	if (sport->per_clk)
		sport->port.uartclk = clk_get_rate(sport->per_clk);
	else
		sport->port.uartclk = clk_get_rate(sport->ipg_clk);

	lpuart_ports[sport->port.line] = sport;

	platform_set_drvdata(pdev, &sport->port);

	/* Disable interrupts before request irq */
	if (lpuart_is_32(sport)) {
		cr_32 = lpuart32_read(&sport->port, UARTCTRL);
		cr_32 &= ~(UARTCTRL_TIE | UARTCTRL_TCIE | UARTCTRL_RIE | UARTCTRL_ILIE);
		lpuart32_write(&sport->port, cr_32, UARTCTRL);
	} else {
		cr_8 = readb(sport->port.membase + UARTCR2);
		cr_8 &= ~(UARTCR2_TIE | UARTCR2_TCIE | UARTCR2_RIE | UARTCR2_ILIE);
		writeb(cr_8, sport->port.membase + UARTCR2);
	}

	if (lpuart_is_32(sport)) {
		lpuart_reg.cons = LPUART32_CONSOLE;
		ret = devm_request_irq(&pdev->dev, sport->port.irq, lpuart32_int, 0,
					DRIVER_NAME, sport);
	} else {
		lpuart_reg.cons = LPUART_CONSOLE;
		ret = devm_request_irq(&pdev->dev, sport->port.irq, lpuart_int, 0,
					DRIVER_NAME, sport);
	}

	if (ret)
		goto failed_irq_request;

	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, UART_AUTOSUSPEND_TIMEOUT);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = uart_add_one_port(&lpuart_reg, &sport->port);
	if (ret)
		goto failed_attach_port;

	ret = lpuart_hw_reset(sport);
	if (ret)
		goto failed_attach_port;

	sport->dma_tx_chan = dma_request_slave_channel(sport->port.dev, "tx");
	if (!sport->dma_tx_chan)
		dev_info(sport->port.dev, "NO DMA tx channel, run at cpu mode\n");

	sport->dma_rx_chan = dma_request_slave_channel(sport->port.dev, "rx");
	if (!sport->dma_rx_chan)
		dev_info(sport->port.dev, "NO DMA rx channel, run at cpu mode\n");

	if (!lpuart_is_32(sport) &&
		of_property_read_bool(np, "linux,rs485-enabled-at-boot-time")) {
		sport->port.rs485.flags |= SER_RS485_ENABLED;
		sport->port.rs485.flags |= SER_RS485_RTS_ON_SEND;
		writeb(UARTMODEM_TXRTSE, sport->port.membase + UARTMODEM);
	}

	return 0;

failed_attach_port:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
failed_irq_request:
	clk_disable_unprepare(sport->per_clk);
	clk_disable_unprepare(sport->ipg_clk);
	return ret;
}

static int lpuart_remove(struct platform_device *pdev)
{
	struct lpuart_port *sport = platform_get_drvdata(pdev);

	uart_remove_one_port(&lpuart_reg, &sport->port);

	clk_disable_unprepare(sport->per_clk);
	clk_disable_unprepare(sport->ipg_clk);

	if (sport->dma_tx_chan)
		dma_release_channel(sport->dma_tx_chan);

	if (sport->dma_rx_chan)
		dma_release_channel(sport->dma_rx_chan);

	clk_disable_unprepare(sport->per_clk);
	clk_disable_unprepare(sport->ipg_clk);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_dont_use_autosuspend(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lpuart_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpuart_port *sport = platform_get_drvdata(pdev);

	clk_disable_unprepare(sport->per_clk);
	clk_disable_unprepare(sport->ipg_clk);

	return 0;
};

static int lpuart_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpuart_port *sport = platform_get_drvdata(pdev);
	int ret;

	ret = clk_prepare_enable(sport->ipg_clk);
	if (ret)
		return ret;
	ret = clk_prepare_enable(sport->per_clk);
	if (ret) {
		clk_disable_unprepare(sport->ipg_clk);
		return ret;
	}

	return 0;
};

static void serial_lpuart_enable_wakeup(struct lpuart_port *sport, bool on)
{
	unsigned int val;

	if (lpuart_is_32(sport)) {
		val = lpuart32_read(&sport->port, UARTCTRL);
		if (on)
			val |= (UARTCTRL_RIE | UARTCTRL_ILIE);
		else
			val &= ~(UARTCTRL_RIE | UARTCTRL_ILIE);
		lpuart32_write(&sport->port, val, UARTCTRL);
	} else {
		val = readb(sport->port.membase + UARTCR2);
		if (on)
			val |= UARTCR2_RIE;
		else
			val &= ~UARTCR2_RIE;
		writeb(val, sport->port.membase + UARTCR2);
	}
}

static bool lpuart_uport_is_active(struct lpuart_port *sport)
{
	struct tty_port *port = &sport->port.state->port;
	struct tty_struct *tty;
	struct device *tty_dev;
	int may_wake = 0;

	tty = tty_port_tty_get(port);
	if (tty) {
		tty_dev = tty->dev;
		may_wake = device_may_wakeup(tty_dev);
		tty_kref_put(tty);
	}

	if ((tty_port_initialized(port) && may_wake) ||
	    (!console_suspend_enabled && uart_console(&sport->port)))
		return true;

	return false;
}

static int lpuart_suspend_noirq(struct device *dev)
{
	struct lpuart_port *sport = dev_get_drvdata(dev);
	bool irq_wake = irqd_is_wakeup_set(irq_get_irq_data(sport->port.irq));

	if (lpuart_uport_is_active(sport))
		serial_lpuart_enable_wakeup(sport, !!irq_wake);

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int lpuart_resume_noirq(struct device *dev)
{
	struct lpuart_port *sport = dev_get_drvdata(dev);
	unsigned int val;

	pinctrl_pm_select_default_state(dev);

	if (lpuart_uport_is_active(sport)) {
		serial_lpuart_enable_wakeup(sport, false);

		/* clear the wakeup flags */
		if (lpuart_is_32(sport)) {
			val = lpuart32_read(&sport->port, UARTSTAT);
			lpuart32_write(&sport->port, val, UARTSTAT);
		}
	}

	return 0;
}

static int lpuart_suspend(struct device *dev)
{
	struct lpuart_port *sport = dev_get_drvdata(dev);
	unsigned long temp;
	unsigned long flags;

	uart_suspend_port(&lpuart_reg, &sport->port);

	if (lpuart_uport_is_active(sport)) {
		spin_lock_irqsave(&sport->port.lock, flags);
		if (lpuart_is_32(sport)) {
			temp = lpuart32_read(&sport->port, UARTCTRL);
			temp &= ~(UARTCTRL_TE | UARTCTRL_TIE | UARTCTRL_TCIE);
			lpuart32_write(&sport->port, temp, UARTCTRL);
		} else {
			temp = readb(sport->port.membase + UARTCR2);
			temp &= ~(UARTCR2_TE | UARTCR2_TIE | UARTCR2_TCIE);
			writeb(temp, sport->port.membase + UARTCR2);
		}
		spin_unlock_irqrestore(&sport->port.lock, flags);

		if (sport->lpuart_dma_rx_use) {
			spin_lock_irqsave(&sport->port.lock, flags);
			lpuart_dma_stop(sport, false);
			spin_unlock_irqrestore(&sport->port.lock, flags);

			dmaengine_terminate_all(sport->dma_rx_chan);
			if (!sport->dma_eeop)
				del_timer_sync(&sport->lpuart_timer);
			lpuart_dma_rx_free(&sport->port);
		}

		if (sport->lpuart_dma_tx_use) {
			spin_lock_irqsave(&sport->port.lock, flags);
			if (lpuart_is_32(sport)) {
				temp = lpuart32_read(&sport->port, UARTBAUD);
				temp &= ~UARTBAUD_TDMAE;
				lpuart32_write(&sport->port, temp, UARTBAUD);
			} else {
				temp = readb(sport->port.membase + UARTCR5);
				temp &= ~UARTCR5_TDMAS;
				writeb(temp, sport->port.membase + UARTCR5);
			}
			spin_unlock_irqrestore(&sport->port.lock, flags);

			sport->dma_tx_in_progress = false;
			dmaengine_terminate_all(sport->dma_tx_chan);
		}
	} else if (pm_runtime_active(sport->port.dev)) {
		clk_disable_unprepare(sport->per_clk);
		clk_disable_unprepare(sport->ipg_clk);
		pm_runtime_disable(sport->port.dev);
		pm_runtime_set_suspended(sport->port.dev);
	}

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static void lpuart_console_fixup(struct lpuart_port *sport)
{
	struct tty_port *port = &sport->port.state->port;
	struct uart_port *uport = &sport->port;
	struct device_node *np = sport->port.dev->of_node;
	struct ktermios termios;

	if (!lpuart_is_32(sport) || !np)
		return;

	/* i.MX7ULP enter VLLS mode that lpuart module power off and registers
	 * all lost no matter the port is wakeup source.
	 * For console port, console baud rate setting lost and print messy
	 * log when enable the console port as wakeup source. To avoid the
	 * issue happen, user should not enable uart port as wakeup source
	 * in VLLS mode, or restore console setting here.
	 */
	if (of_device_is_compatible(np, "fsl,imx7ulp-lpuart") &&
	    lpuart_uport_is_active(sport) && console_suspend_enabled &&
	    uart_console(&sport->port)) {

		mutex_lock(&port->mutex);
		memset(&termios, 0, sizeof(struct ktermios));
		termios.c_cflag = uport->cons->cflag;
		if (port->tty && termios.c_cflag == 0)
			termios = port->tty->termios;
		uport->ops->set_termios(uport, &termios, NULL);
		mutex_unlock(&port->mutex);
	}
}

static inline void lpuart32_resume_init(struct lpuart_port *sport)
{
	unsigned long temp;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	lpuart32_setup_watermark(sport);

	temp = lpuart32_read(&sport->port, UARTCTRL);
	temp |= (UARTCTRL_RIE | UARTCTRL_TIE | UARTCTRL_RE |
		 UARTCTRL_TE | UARTCTRL_ILIE);

	if (sport->dma_rx_chan)
		temp &= ~(UARTCTRL_RIE | UARTCTRL_ILIE | UARTCTRL_RE);

	if (sport->dma_tx_chan)
		temp &= ~(UARTCTRL_TIE | UARTCTRL_TE);

	lpuart32_write(&sport->port, temp, UARTCTRL);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	if (sport->lpuart_dma_rx_use) {
		if (!lpuart_dma_rx_request(&sport->port)) {
			sport->lpuart_dma_rx_use = true;
			if (!sport->dma_eeop)
				setup_timer(&sport->lpuart_timer,
					    lpuart_timer_func,
					    (unsigned long)sport);
		} else {
			sport->lpuart_dma_rx_use = false;
		}

		spin_lock_irqsave(&sport->port.lock, flags);
		temp = lpuart32_read(&sport->port, UARTCTRL);
		temp |= (UARTCTRL_RIE | UARTCTRL_ILIE | UARTCTRL_RE);
		temp |= UARTCTRL_IDLECFG << UARTCTRL_IDLECFG_OFF;
		lpuart32_write(&sport->port, temp, UARTCTRL);
		spin_unlock_irqrestore(&sport->port.lock, flags);
	}

	if (sport->lpuart_dma_tx_use) {
		if (!lpuart_dma_tx_request(&sport->port)) {
			init_waitqueue_head(&sport->dma_wait);
			spin_lock_irqsave(&sport->port.lock, flags);
			temp = lpuart32_read(&sport->port, UARTBAUD);
			temp |= UARTBAUD_TDMAE;
			lpuart32_write(&sport->port, temp, UARTBAUD);
			spin_unlock_irqrestore(&sport->port.lock, flags);
		} else {
			sport->lpuart_dma_tx_use = false;
		}

		spin_lock_irqsave(&sport->port.lock, flags);
		temp = lpuart32_read(&sport->port, UARTCTRL);
		temp |= UARTCTRL_TE;
		lpuart32_write(&sport->port, temp, UARTCTRL);
		spin_unlock_irqrestore(&sport->port.lock, flags);
	}
}

static inline void lpuart_resume_init(struct lpuart_port *sport)
{
	unsigned char temp;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);
	lpuart_setup_watermark(sport);
	temp = readb(sport->port.membase + UARTCR2);
	temp |= (UARTCR2_RIE | UARTCR2_TIE | UARTCR2_RE | UARTCR2_TE);

	if (sport->dma_rx_chan)
		temp &= ~(UARTCR2_RIE | UARTCR2_RE);

	if (sport->dma_tx_chan)
		temp &= ~(UARTCR2_TIE | UARTCR2_TE);

	writeb(temp, sport->port.membase + UARTCR2);
	spin_unlock_irqrestore(&sport->port.lock, flags);

	if (sport->lpuart_dma_rx_use) {
		if (!lpuart_dma_rx_request(&sport->port)) {
			sport->lpuart_dma_rx_use = true;
			setup_timer(&sport->lpuart_timer,
				lpuart_timer_func,
				(unsigned long)sport);
		} else {
			sport->lpuart_dma_rx_use = false;
		}

		spin_lock_irqsave(&sport->port.lock, flags);
		temp = readb(sport->port.membase + UARTCR2);
		temp |= (UARTCR2_RIE | UARTCR2_RE);
		writeb(temp, sport->port.membase + UARTCR2);
		spin_unlock_irqrestore(&sport->port.lock, flags);
	}

	if (sport->lpuart_dma_tx_use) {
		if (!lpuart_dma_tx_request(&sport->port)) {
			init_waitqueue_head(&sport->dma_wait);
			spin_lock_irqsave(&sport->port.lock, flags);
			temp = readb(sport->port.membase + UARTCR5);
			temp |= UARTCR5_TDMAS;
			writeb(temp, sport->port.membase + UARTCR5);
			spin_unlock_irqrestore(&sport->port.lock, flags);
		} else {
			sport->lpuart_dma_tx_use = false;
		}

		spin_lock_irqsave(&sport->port.lock, flags);
		temp = readb(sport->port.membase + UARTCR2);
		temp |= UARTCR2_TE;
		writeb(temp, sport->port.membase + UARTCR2);
		spin_unlock_irqrestore(&sport->port.lock, flags);
	}
}

static int lpuart_resume(struct device *dev)
{
	struct lpuart_port *sport = dev_get_drvdata(dev);
	int ret;

	pinctrl_pm_select_default_state(dev);

	if (lpuart_uport_is_active(sport)) {
		if (lpuart_is_32(sport))
			lpuart32_resume_init(sport);
		else
			lpuart_resume_init(sport);
	} else if (pm_runtime_active(sport->port.dev)) {
		ret = clk_prepare_enable(sport->ipg_clk);
		if (ret)
			return ret;
		ret = clk_prepare_enable(sport->per_clk);
		if (ret) {
			clk_disable_unprepare(sport->ipg_clk);
			return ret;
		}
		pm_runtime_set_active(sport->port.dev);
		pm_runtime_enable(sport->port.dev);
	}

	lpuart_console_fixup(sport);
	uart_resume_port(&lpuart_reg, &sport->port);

	return 0;
}

static const struct dev_pm_ops lpuart_pm_ops = {
	SET_RUNTIME_PM_OPS(lpuart_runtime_suspend,
			   lpuart_runtime_resume, NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(lpuart_suspend_noirq,
				      lpuart_resume_noirq)
	SET_SYSTEM_SLEEP_PM_OPS(lpuart_suspend, lpuart_resume)
};
#define SERIAL_LPUART_PM_OPS	(&lpuart_pm_ops)

#else /* !CONFIG_PM_SLEEP */

#define SERIAL_LPUART_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver lpuart_driver = {
	.probe		= lpuart_probe,
	.remove		= lpuart_remove,
	.driver		= {
		.name	= "fsl-lpuart",
		.of_match_table = lpuart_dt_ids,
		.pm	= SERIAL_LPUART_PM_OPS,
	},
};

static int __init lpuart_serial_init(void)
{
	int ret = uart_register_driver(&lpuart_reg);

	if (ret)
		return ret;

	ret = platform_driver_register(&lpuart_driver);
	if (ret)
		uart_unregister_driver(&lpuart_reg);

	return ret;
}

static void __exit lpuart_serial_exit(void)
{
	platform_driver_unregister(&lpuart_driver);
	uart_unregister_driver(&lpuart_reg);
}

module_init(lpuart_serial_init);
module_exit(lpuart_serial_exit);

MODULE_DESCRIPTION("Freescale lpuart serial port driver");
MODULE_LICENSE("GPL v2");
