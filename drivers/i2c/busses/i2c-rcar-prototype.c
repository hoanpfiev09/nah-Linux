/*
 * Driver for the Renesas R-Car I2C unit
 *
 * Copyright (C) 2014-15 Wolfram Sang <wsa@sang-engineering.com>
 * Copyright (C) 2011-2015 Renesas Electronics Corporation
 *
 * Copyright (C) 2012-14 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This file is based on the drivers/i2c/busses/i2c-sh7760.c
 * (c) 2005-2008 MSC Vertriebsges.m.b.H, Manuel Lauss <mlau@msc-ge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

/* register offsets */
#define ICSCR	0x00	/* slave ctrl */
#define ICMCR	0x04	/* master ctrl */
#define ICSSR	0x08	/* slave status */
#define ICMSR	0x0C	/* master status */
#define ICSIER	0x10	/* slave irq enable */
#define ICMIER	0x14	/* master irq enable */
#define ICCCR	0x18	/* clock dividers */
#define ICSAR	0x1C	/* slave address */
#define ICMAR	0x20	/* master address */
#define ICRXTX	0x24	/* data port */
#define ICDMAER	0x3c	/* DMA enable */
#define ICFBSCR	0x38	/* first bit setup cycle */

/* ICSCR */
#define SDBS	(1 << 3)	/* slave data buffer select */
#define SIE	(1 << 2)	/* slave interface enable */
#define GCAE	(1 << 1)	/* general call address enable */
#define FNA	(1 << 0)	/* forced non acknowledgment */

/* ICMCR */
#define MDBS	(1 << 7)	/* non-fifo mode switch */
#define FSCL	(1 << 6)	/* override SCL pin */
#define FSDA	(1 << 5)	/* override SDA pin */
#define OBPC	(1 << 4)	/* override pins */
#define MIE	(1 << 3)	/* master if enable */
#define TSBE	(1 << 2)
#define FSB	(1 << 1)	/* force stop bit */
#define ESG	(1 << 0)	/* en startbit gen */

/* ICSSR (also for ICSIER) */
#define GCAR	(1 << 6)	/* general call received */
#define STM	(1 << 5)	/* slave transmit mode */
#define SSR	(1 << 4)	/* stop received */
#define SDE	(1 << 3)	/* slave data empty */
#define SDT	(1 << 2)	/* slave data transmitted */
#define SDR	(1 << 1)	/* slave data received */
#define SAR	(1 << 0)	/* slave addr received */

/* ICMSR (also for ICMIE) */
#define MNR	(1 << 6)	/* nack received */
#define MAL	(1 << 5)	/* arbitration lost */
#define MST	(1 << 4)	/* sent a stop */
#define MDE	(1 << 3)
#define MDT	(1 << 2)
#define MDR	(1 << 1)
#define MAT	(1 << 0)	/* slave addr xfer done */

/* ICDMAER */
#define RSDMAE	(1 << 3)	/* DMA Slave Received Enable */
#define TSDMAE	(1 << 2)	/* DMA Slave Transmitted Enable */
#define RMDMAE	(1 << 1)	/* DMA Master Received Enable */
#define TMDMAE	(1 << 0)	/* DMA Master Transmitted Enable */

/* ICFBSCR */
#define TCYC06	0x04		/*  6*Tcyc delay 1st bit between SDA and SCL */
#define TCYC17	0x0f		/* 17*Tcyc delay 1st bit between SDA and SCL */


#define RCAR_BUS_PHASE_START	(MDBS | MIE | ESG)
#define RCAR_BUS_PHASE_DATA	(MDBS | MIE)
#define RCAR_BUS_MASK_DATA	(~(ESG | FSB) & 0xFF)
#define RCAR_BUS_PHASE_STOP	(MDBS | MIE | FSB)

#define RCAR_IRQ_SEND	(MNR | MAL | MST | MAT | MDE)
#define RCAR_IRQ_RECV	(MNR | MAL | MST | MAT | MDR)
#define RCAR_IRQ_STOP	(MST)

#define RCAR_IRQ_ACK_SEND	(~(MAT | MDE) & 0xFF)
#define RCAR_IRQ_ACK_RECV	(~(MAT | MDR) & 0xFF)

#define ID_LAST_MSG	(1 << 0)
#define ID_FIRST_MSG	(1 << 1)
#define ID_DONE		(1 << 2)
#define ID_ARBLOST	(1 << 3)
#define ID_NACK		(1 << 4)
/* persistent flags */
#define ID_P_PM_BLOCKED	(1 << 31)
#define ID_P_MASK	ID_P_PM_BLOCKED

enum rcar_i2c_type {
	I2C_RCAR_GEN1,
	I2C_RCAR_GEN2,
	I2C_RCAR_GEN3,
};

struct rcar_i2c_priv {
	void __iomem *io;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	int msgs_left;
	struct clk *clk;

	wait_queue_head_t wait;

	int pos;
	u32 icccr;
	u32 flags;
	enum rcar_i2c_type devtype;
	struct i2c_client *slave;

	struct resource *res;
	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;
	struct scatterlist sg;
	enum dma_data_direction dma_direction;
};

#define rcar_i2c_priv_to_dev(p)		((p)->adap.dev.parent)
#define rcar_i2c_is_recv(p)		((p)->msg->flags & I2C_M_RD)

#define LOOP_TIMEOUT	1024


static void rcar_i2c_write(struct rcar_i2c_priv *priv, int reg, u32 val)
{
	writel(val, priv->io + reg);
}

static u32 rcar_i2c_read(struct rcar_i2c_priv *priv, int reg)
{
	return readl(priv->io + reg);
}

static void rcar_i2c_init(struct rcar_i2c_priv *priv)
{
	/* reset master mode */

	/* start clock */

}

static int rcar_i2c_bus_barrier(struct rcar_i2c_priv *priv)
{
	int i;


	return -EBUSY;
}

static int rcar_i2c_clock_calculate(struct rcar_i2c_priv *priv, struct i2c_timings *t)
{


	return 0;
}

static void rcar_i2c_prepare_msg(struct rcar_i2c_priv *priv)
{

}

static void rcar_i2c_next_msg(struct rcar_i2c_priv *priv)
{

}

/*
 *		interrupt functions
 */
static void rcar_i2c_dma_unmap(struct rcar_i2c_priv *priv)
{

}

static void rcar_i2c_cleanup_dma(struct rcar_i2c_priv *priv)
{

}

static void rcar_i2c_dma_callback(void *data)
{
}

static void rcar_i2c_dma(struct rcar_i2c_priv *priv)
{
}

static void rcar_i2c_irq_send(struct rcar_i2c_priv *priv, u32 msr)
{

}

static void rcar_i2c_irq_recv(struct rcar_i2c_priv *priv, u32 msr)
{
}

static bool rcar_i2c_slave_irq(struct rcar_i2c_priv *priv)
{
	u32 ssr_raw, ssr_filtered;
	u8 value;

	return true;
}

static irqreturn_t rcar_i2c_irq(int irq, void *ptr)
{
	return IRQ_HANDLED;
}

static struct dma_chan *rcar_i2c_request_dma_chan(struct device *dev,
					enum dma_transfer_direction dir,
					dma_addr_t port_addr)
{
	struct dma_chan *chan;
	return chan;
}

static void rcar_i2c_request_dma(struct rcar_i2c_priv *priv,
				 struct i2c_msg *msg)
{

}

static void rcar_i2c_release_dma(struct rcar_i2c_priv *priv)
{
}

static int rcar_i2c_master_xfer(struct i2c_adapter *adap,
				struct i2c_msg *msgs,
				int num)
{
	int ret;
	return ret;
}

static int rcar_reg_slave(struct i2c_client *slave)
{
	return 0;
}

static int rcar_unreg_slave(struct i2c_client *slave)
{
	return 0;
}

static u32 rcar_i2c_func(struct i2c_adapter *adap)
{
	/*
	 * This HW can't do:
	 * I2C_SMBUS_QUICK (setting FSB during START didn't work)
	 * I2C_M_NOSTART (automatically sends address after START)
	 * I2C_M_IGNORE_NAK (automatically sends STOP after NAK)
	 */
	u32 ret;
	return ret;
}

static const struct i2c_algorithm rcar_i2c_algo = {
	.master_xfer	= rcar_i2c_master_xfer,
	.functionality	= rcar_i2c_func,
	.reg_slave	= rcar_reg_slave,
	.unreg_slave	= rcar_unreg_slave,
};

static const struct of_device_id rcar_i2c_dt_ids[] = {
	{ .compatible = "renesas,i2c-r8a7778", .data = (void *)I2C_RCAR_GEN1 },
	{ .compatible = "renesas,i2c-r8a7779", .data = (void *)I2C_RCAR_GEN1 },
	{ .compatible = "renesas,i2c-r8a7790", .data = (void *)I2C_RCAR_GEN2 },
	{ .compatible = "renesas,i2c-r8a7791", .data = (void *)I2C_RCAR_GEN2 },
	{ .compatible = "renesas,i2c-r8a7792", .data = (void *)I2C_RCAR_GEN2 },
	{ .compatible = "renesas,i2c-r8a7793", .data = (void *)I2C_RCAR_GEN2 },
	{ .compatible = "renesas,i2c-r8a7794", .data = (void *)I2C_RCAR_GEN2 },
	{ .compatible = "renesas,i2c-r8a7795", .data = (void *)I2C_RCAR_GEN3 },
	{ .compatible = "renesas,i2c-r8a7796", .data = (void *)I2C_RCAR_GEN3 },
	{ .compatible = "renesas,i2c-rcar", .data = (void *)I2C_RCAR_GEN1 },	/* Deprecated */
	{ .compatible = "renesas,rcar-gen1-i2c", .data = (void *)I2C_RCAR_GEN1 },
	{ .compatible = "renesas,rcar-gen2-i2c", .data = (void *)I2C_RCAR_GEN2 },
	{ .compatible = "renesas,rcar-gen3-i2c", .data = (void *)I2C_RCAR_GEN3 },
	{},
};
MODULE_DEVICE_TABLE(of, rcar_i2c_dt_ids);

static int rcar_i2c_probe(struct platform_device *pdev)
{

	int ret = 0;
	return ret;
}

static int rcar_i2c_remove(struct platform_device *pdev)
{


	return 0;
}

static struct platform_driver rcar_i2c_driver = {
	.driver	= {
		.name	= "i2c-rcar",
		.of_match_table = rcar_i2c_dt_ids,
	},
	.probe		= rcar_i2c_probe,
	.remove		= rcar_i2c_remove,
};

module_platform_driver(rcar_i2c_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car I2C bus driver");
MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
