#include <linux/clk.h>
#include <linux/console.h>
#include <linux/ctype.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/ktime.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/serial.h>
#include <linux/serial_sci.h>
#include <linux/sh_dma.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysrq.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>


#include "serial_mctrl_gpio.h"
#include "sh-sci.h"

enum SCI_CLKS {
	SCI_FCK,		/* Functional Clock */
	SCI_SCK,		/* Optional External Clock */
	SCI_BRG_INT,		/* Optional BRG Internal Clock Source */
	SCI_SCIF_CLK,		/* Optional BRG External Clock Source */
	SCI_NUM_CLKS
};

#define SCSMR	0x00	/* Serial Mode Register */
#define SCBRR	0x04	/* Bit rate Register */
#define SCSCR	0x08	/* Serial Control Register */
#define SCFTDR	0x0C	/* Transmit FIFO data register */
#define SCFSR	0x10	/* Serial status register */
#define SCFRDR	0x14	/* Receive FIFO data register */
#define SCFCR	0x18	/* FIFO control register */
#define SCFDR	0x1C	/* FIFO data count register */
#define SCSPTR	0x20	/* Serial port register */
#define SCLSR	0x24	/* Line status register */
#define SCDL	0x30	/* Frequency division register */
#define SCCKS	0x34	/* Clock Select register */


/* SCSMR (Serial Mode Register) */
#define SCSMR_C_A	BIT(7)	/* Communication Mode */
#define SCSMR_CSYNC	BIT(7)	/*   - Clocked synchronous mode */
#define SCSMR_ASYNC	0	/*   - Asynchronous mode */
#define SCSMR_CHR	BIT(6)	/* 7-bit Character Length */
#define SCSMR_PE	BIT(5)	/* Parity Enable */
#define SCSMR_ODD	BIT(4)	/* Odd Parity */
#define SCSMR_STOP	BIT(3)	/* Stop Bit Length */
#define SCSMR_CKS	0x0003	/* Clock Select */

/* Serial Mode Register, SCIFA/SCIFB only bits */
#define SCSMR_CKEDG	BIT(12)	/* Transmit/Receive Clock Edge Select */
#define SCSMR_SRC_MASK	0x0700	/* Sampling Control */
#define SCSMR_SRC_16	0x0000	/* Sampling rate 1/16 */
#define SCSMR_SRC_5	0x0100	/* Sampling rate 1/5 */
#define SCSMR_SRC_7	0x0200	/* Sampling rate 1/7 */
#define SCSMR_SRC_11	0x0300	/* Sampling rate 1/11 */
#define SCSMR_SRC_13	0x0400	/* Sampling rate 1/13 */
#define SCSMR_SRC_17	0x0500	/* Sampling rate 1/17 */
#define SCSMR_SRC_19	0x0600	/* Sampling rate 1/19 */
#define SCSMR_SRC_27	0x0700	/* Sampling rate 1/27 */

/* Serial Control Register, SCIFA/SCIFB only bits */
#define SCSCR_TDRQE	BIT(15)	/* Tx Data Transfer Request Enable */
#define SCSCR_RDRQE	BIT(14)	/* Rx Data Transfer Request Enable */

/* Serial Control Register, HSCIF-only bits */
#define HSSCR_TOT_SHIFT	14

/* SCxSR (Serial Status Register) on SCI */
#define SCI_TDRE	BIT(7)	/* Transmit Data Register Empty */
#define SCI_RDRF	BIT(6)	/* Receive Data Register Full */
#define SCI_ORER	BIT(5)	/* Overrun Error */
#define SCI_FER		BIT(4)	/* Framing Error */
#define SCI_PER		BIT(3)	/* Parity Error */
#define SCI_TEND	BIT(2)	/* Transmit End */
#define SCI_RESERVED	0x03	/* All reserved bits */

#define SCI_DEFAULT_ERROR_MASK (SCI_PER | SCI_FER)

#define SCI_RDxF_CLEAR	(u32)(~(SCI_RESERVED | SCI_RDRF))
#define SCI_ERROR_CLEAR	(u32)(~(SCI_RESERVED | SCI_PER | SCI_FER | SCI_ORER))
#define SCI_TDxE_CLEAR	(u32)(~(SCI_RESERVED | SCI_TEND | SCI_TDRE))
#define SCI_BREAK_CLEAR	(u32)(~(SCI_RESERVED | SCI_PER | SCI_FER | SCI_ORER))

/* SCxSR (Serial Status Register) on SCIF, SCIFA, SCIFB, HSCIF */
#define SCIF_ER		BIT(7)	/* Receive Error */
#define SCIF_TEND	BIT(6)	/* Transmission End */
#define SCIF_TDFE	BIT(5)	/* Transmit FIFO Data Empty */
#define SCIF_BRK	BIT(4)	/* Break Detect */
#define SCIF_FER	BIT(3)	/* Framing Error */
#define SCIF_PER	BIT(2)	/* Parity Error */
#define SCIF_RDF	BIT(1)	/* Receive FIFO Data Full */
#define SCIF_DR		BIT(0)	/* Receive Data Ready */
/* SCIF only (optional) */
#define SCIF_PERC	0xf000	/* Number of Parity Errors */
#define SCIF_FERC	0x0f00	/* Number of Framing Errors */
/*SCIFA/SCIFB and SCIF on SH7705/SH7720/SH7721 only */
#define SCIFA_ORER	BIT(9)	/* Overrun Error */

#define SCIF_DEFAULT_ERROR_MASK (SCIF_PER | SCIF_FER | SCIF_BRK | SCIF_ER)

#define SCIF_RDxF_CLEAR		(u32)(~(SCIF_DR | SCIF_RDF))
#define SCIF_ERROR_CLEAR	(u32)(~(SCIF_PER | SCIF_FER | SCIF_ER))
#define SCIF_TDxE_CLEAR		(u32)(~(SCIF_TDFE))
#define SCIF_BREAK_CLEAR	(u32)(~(SCIF_PER | SCIF_FER | SCIF_BRK))

/* SCFCR (FIFO Control Register) */
#define SCFCR_RTRG1	BIT(7)	/* Receive FIFO Data Count Trigger */
#define SCFCR_RTRG0	BIT(6)
#define SCFCR_TTRG1	BIT(5)	/* Transmit FIFO Data Count Trigger */
#define SCFCR_TTRG0	BIT(4)
#define SCFCR_MCE	BIT(3)	/* Modem Control Enable */
#define SCFCR_TFRST	BIT(2)	/* Transmit FIFO Data Register Reset */
#define SCFCR_RFRST	BIT(1)	/* Receive FIFO Data Register Reset */
#define SCFCR_LOOP	BIT(0)	/* Loopback Test */

/* SCLSR (Line Status Register) on (H)SCIF */
#define SCLSR_TO	BIT(2)	/* Timeout */
#define SCLSR_ORER	BIT(0)	/* Overrun Error */

/* SCSPTR (Serial Port Register), optional */
#define SCSPTR_RTSIO	BIT(7)	/* Serial Port RTS# Pin Input/Output */
#define SCSPTR_RTSDT	BIT(6)	/* Serial Port RTS# Pin Data */
#define SCSPTR_CTSIO	BIT(5)	/* Serial Port CTS# Pin Input/Output */
#define SCSPTR_CTSDT	BIT(4)	/* Serial Port CTS# Pin Data */
#define SCSPTR_SCKIO	BIT(3)	/* Serial Port Clock Pin Input/Output */
#define SCSPTR_SCKDT	BIT(2)	/* Serial Port Clock Pin Data */
#define SCSPTR_SPB2IO	BIT(1)	/* Serial Port Break Input/Output */
#define SCSPTR_SPB2DT	BIT(0)	/* Serial Port Break Data */

/* HSSRR HSCIF */
#define HSCIF_SRE	BIT(15)	/* Sampling Rate Register Enable */
#define HSCIF_SRDE	BIT(14) /* Sampling Point Register Enable */

#define HSCIF_SRHP_SHIFT	8
#define HSCIF_SRHP_MASK		0x0f00

/* SCPCR (Serial Port Control Register), SCIFA/SCIFB only */
#define SCPCR_RTSC	BIT(4)	/* Serial Port RTS# Pin / Output Pin */
#define SCPCR_CTSC	BIT(3)	/* Serial Port CTS# Pin / Input Pin */
#define SCPCR_SCKC	BIT(2)	/* Serial Port SCK Pin / Output Pin */
#define SCPCR_RXDC	BIT(1)	/* Serial Port RXD Pin / Input Pin */
#define SCPCR_TXDC	BIT(0)	/* Serial Port TXD Pin / Output Pin */

/* SCPDR (Serial Port Data Register), SCIFA/SCIFB only */
#define SCPDR_RTSD	BIT(4)	/* Serial Port RTS# Output Pin Data */
#define SCPDR_CTSD	BIT(3)	/* Serial Port CTS# Input Pin Data */
#define SCPDR_SCKD	BIT(2)	/* Serial Port SCK Output Pin Data */
#define SCPDR_RXDD	BIT(1)	/* Serial Port RXD Input Pin Data */
#define SCPDR_TXDD	BIT(0)	/* Serial Port TXD Output Pin Data */

/*
 * BRG Clock Select Register (Some SCIF and HSCIF)
 * The Baud Rate Generator for external clock can provide a clock source for
 * the sampling clock. It outputs either its frequency divided clock, or the
 * (undivided) (H)SCK external clock.
 */
#define SCCKS_CKS	BIT(15)	/* Select (H)SCK (1) or divided SC_CLK (0) */
#define SCCKS_XIN	BIT(14)	/* SC_CLK uses bus clock (1) or SCIF_CLK (0) */

#define h_debug            printk("file %s func %s line %d", __FILE__, __FUNCTION__, __LINE__);

struct sci_port {

	struct uart_port port;
	int idx;

	struct clk		*clks;
	struct device *dev;

};

#define SCI_NPORTS CONFIG_SERIAL_SH_SCI_NR_UARTS
static struct sci_port sci_ports[SCI_NPORTS];
static struct uart_driver h_sh_sci_uart_driver;
#define SCI_OF_DATA(type, regtype)	(void *)((type) << 16 | (regtype))

static const struct of_device_id of_sci_match[] = {
	/* SoC-specific types */

	/* Family-specific types */
    {
		.compatible = "renesas,hoan-rcar-gen3-scif",
		.data = SCI_OF_DATA(PORT_SCIF, SCIx_SH4_SCIF_BRG_REGTYPE),
	},
	/* Generic types */
	{
		.compatible = "renesas,hoan-scif",
		.data = SCI_OF_DATA(PORT_SCIF, SCIx_SH4_SCIF_REGTYPE),
	},
};
MODULE_DEVICE_TABLE(of, of_sci_match);

struct plat_sci_reg {
	int reg;
	u8 size;
};

struct plat_sci_reg regs[12] = {
		{ SCSMR,  16 },
		{ SCBRR,  8 },
		{ SCSCR,  16 },
		{ SCFTDR,  8 },
		{ SCFSR,  16 },
		{ SCFRDR,  8 },
		{ SCFCR,  16 },
		{ SCFDR,  16 },
		{ SCSPTR,  16 },
		{ SCLSR,  16 },
		{ SCDL,  16 },
		{ SCCKS,  16 },
};

static int h_sci_get_size_regs(int reg)
{
	int size;
	unsigned int i;
	for (i = 0; i < 12; i++)
	{
		if (regs[i].reg == reg){
			size = regs[i].size;
			break;
		}

		if (i == 11){
			if (regs[i].reg != reg)
				size = 8; /*default*/
		}
	}

	return size;
}
static void h_sci_serial_out(struct uart_port* port, int reg , int val)
{

}

static unsigned int h_sci_serial_in(struct uart_port* port, int reg)
{
	unsigned int ret;

	return ret;
}

static unsigned int h_sh_sci_uart_tx_empty(struct uart_port* port)
{
	unsigned int ret = 0;

	h_debug;
	return ret;
}

static unsigned int h_sh_sci_uart_get_mctrl(struct uart_port* port)
{
	int ret = 0;

	h_debug;
	return ret;
}


static void h_sh_sci_uart_set_mctrl(struct uart_port* port, unsigned int mctrl)
{
	h_debug;
}

static void h_sh_sci_uart_start_tx(struct uart_port* port)
{
	h_debug;
	return ;
}


static void h_sh_sci_uart_stop_tx(struct uart_port* port)
{
	h_debug;
	return ;
}

static void h_sh_sci_uart_stop_rx(struct uart_port* port)
{
	h_debug;
	return ;
}


static void h_sh_sci_uart_break_ctl(struct uart_port* port, int ctl)
{
	h_debug;
	return;
}


static int h_sh_sci_uart_startup(struct uart_port* port)
{
	int ret = 0;

	h_debug;
	return ret;
}


static void h_sh_sci_uart_shutdown(struct uart_port* port)
{

	h_debug;
	return ;
}


static void h_sh_sci_uart_set_termios(struct uart_port *port, struct ktermios *kterm, struct ktermios *old)
{
	h_debug;
	return ;
}

static const char* h_sh_sci_uart_type(struct uart_port* port)
{
	char* ret = "OK";
	h_debug;
	return ret;
}

static void h_sh_sci_uart_release_port(struct uart_port* port)
{

	h_debug;
	return ;
}


static int h_sh_sci_uart_request_port(struct uart_port* port)
{
	int ret = 0;
	h_debug;
	return ret;
}

static void h_sh_sci_uart_config_port(struct uart_port* port, int val)
{
	h_debug;
	port->type = PORT_SCIF;
	return ;
}

static int h_sh_sci_uart_verify_port(struct uart_port* port, struct serial_struct *sstruct)
{
	int ret = 0;
	h_debug;
	return ret;
}


static const struct uart_ops h_sh_sci_uart_ops = {
	.tx_empty	= h_sh_sci_uart_tx_empty,
	.get_mctrl	= h_sh_sci_uart_get_mctrl,
	.set_mctrl	= h_sh_sci_uart_set_mctrl,
	.start_tx	= h_sh_sci_uart_start_tx,
	.stop_tx	= h_sh_sci_uart_stop_tx,
	.stop_rx	= h_sh_sci_uart_stop_rx,
	.break_ctl	= h_sh_sci_uart_break_ctl,
	.startup	= h_sh_sci_uart_startup,
	.shutdown	= h_sh_sci_uart_shutdown,
	.set_termios	= h_sh_sci_uart_set_termios,
	.type		= h_sh_sci_uart_type,
	.release_port	= h_sh_sci_uart_release_port,
	.request_port	= h_sh_sci_uart_request_port,
	.config_port	= h_sh_sci_uart_config_port,
	.verify_port	= h_sh_sci_uart_verify_port,
};



static DEFINE_MUTEX(sci_uart_registration_lock);
static struct uart_driver h_sh_sci_uart_driver = {
	.owner			= THIS_MODULE,
	.driver_name		= "h-na-sci",
	.dev_name		= "ttyHNASCI",
	.nr			= 18,
	//.cons			= NULL,
};


static int h_sh_sci_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct sci_port *sp;
	int uart_idx = 0;
	struct resource *res_mem;
	struct uart_port *port;
	int ret;
	u8 reg_val;


	h_debug;

	uart_idx = of_alias_get_id(np, "serial");
	if (uart_idx < 0 )
		return -EINVAL;

	printk("file %s func %s line %d uart_idx %d pdev->name %s", __FILE__, __FUNCTION__, __LINE__, uart_idx, pdev->name );

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -EINVAL;

	sp = devm_kzalloc(&pdev->dev, sizeof(*sp), GFP_KERNEL);
	if (!sp)
		return -ENOMEM;

	sp = &sci_ports[uart_idx];

	sp->idx = uart_idx;
	sp->dev = dev;
	sp->clks = devm_clk_get(dev, "scif_clk");
	printk("clk %s is %pC rate %lu\n", "scif_clk",
			sp->clks, clk_get_rate(sp->clks));

	port = &sp->port;
	platform_set_drvdata(pdev, sp);

	port->iotype	= UPIO_MEM;
	port->mapbase	= res_mem->start;
	port->ops	= &h_sh_sci_uart_ops;
	port->flags	= UPF_BOOT_AUTOCONF;
	port->dev	= &pdev->dev;
	port->fifosize	= 8;
	port->uartclk	= clk_get_rate(sp->clks);
	port->line	= uart_idx;


	mutex_lock(&sci_uart_registration_lock);
	if (!h_sh_sci_uart_driver.state) {
		ret = uart_register_driver(&h_sh_sci_uart_driver);
		if (ret) {
			mutex_unlock(&sci_uart_registration_lock);
			return ret;
		}
	}
	mutex_unlock(&sci_uart_registration_lock);

	ret = uart_add_one_port(&h_sh_sci_uart_driver, &sp->port);
	if (ret) {
		printk("file %s func %s line %d ret %d", __FILE__, __FUNCTION__, __LINE__, ret);
	}

	port->membase = devm_ioremap_nocache(port->dev, port->mapbase,
						resource_size(res_mem));

	reg_val = ioread8(port->membase + 0x04);
	printk("file %s func %s line %d reg_val %x", __FILE__, __FUNCTION__, __LINE__, reg_val);


	port->serial_in		= h_sci_serial_in;
	port->serial_out	= h_sci_serial_out;


	return 0;
}

static int h_sh_sci_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver sci_driver = {
	.probe		= h_sh_sci_probe,
	.remove		= h_sh_sci_remove,
	.driver		= {
		.name	= "hoan-sh-sci-2",
		.of_match_table = of_match_ptr(of_sci_match),
	},
};

static int __init sci_init(void)
{
	int ret;

	h_debug;

	return platform_driver_register(&sci_driver);
}

static void __exit sci_exit(void)
{
	int ret;

	return;
}


module_init(sci_init);
module_exit(sci_exit);


