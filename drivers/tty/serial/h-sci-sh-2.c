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


static unsigned int h_sh_sci_uart_tx_empty(struct uart_port* port)
{
	unsigned int ret = 0;

	return ret;
}

static unsigned int h_sh_sci_uart_get_mctrl(struct uart_port* port)
{
	int ret = 0;

	return ret;
}


static void h_sh_sci_uart_set_mctrl(struct uart_port* port, unsigned int mctrl)
{

}

static void h_sh_sci_uart_start_tx(struct uart_port* port)
{

	return ;
}


static void h_sh_sci_uart_stop_tx(struct uart_port* port)
{

	return ;
}

static void h_sh_sci_uart_stop_rx(struct uart_port* port)
{

	return ;
}


static void h_sh_sci_uart_break_ctl(struct uart_port* port, int ctl)
{
	return;
}


static int h_sh_sci_uart_startup(struct uart_port* port)
{
	int ret = 0;

	return ret;
}


static void h_sh_sci_uart_shutdown(struct uart_port* port)
{

	return ;
}


static void h_sh_sci_uart_set_termios(struct uart_port *port, struct ktermios *kterm, struct ktermios *old)
{

	return ;
}

static const char* h_sh_sci_uart_type(struct uart_port* port)
{
	char* ret = "OK";

	return ret;
}

static void h_sh_sci_uart_release_port(struct uart_port* port)
{


	return ;
}


static int h_sh_sci_uart_request_port(struct uart_port* port)
{
	int ret = 0;

	return ret;
}

static void h_sh_sci_uart_config_port(struct uart_port* port, int val)
{

	return ;
}

static int h_sh_sci_uart_verify_port(struct uart_port* port, struct serial_struct *sstruct)
{
	int ret = 0;

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


