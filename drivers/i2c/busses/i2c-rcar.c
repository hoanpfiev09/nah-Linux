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

//#define RCAR_IRQ_SEND	(MNR | MAL | MST | MAT | MDE)
//#define RCAR_IRQ_RECV	(MNR | MAL | MST | MAT | MDR)
#define RCAR_IRQ_SEND	(MNR | MAL | MST | MDE)
#define RCAR_IRQ_RECV	(MNR | MAL | MST | MDR)

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
	printk("Hoan_rcar_i2c_init\n");
	/* reset master mode */
	rcar_i2c_write(priv, ICMIER, 0);
	rcar_i2c_write(priv, ICMCR, MDBS);
	rcar_i2c_write(priv, ICMSR, 0);
	/* start clock */
	rcar_i2c_write(priv, ICCCR, priv->icccr);
}

static int rcar_i2c_bus_barrier(struct rcar_i2c_priv *priv)
{
	int i;
	printk("Hoan_rcar_i2c_bus_barrier\n");

	return 0;
}

static int rcar_i2c_clock_calculate(struct rcar_i2c_priv *priv, struct i2c_timings *t)
{
	printk("Hoan_rcar_i2c_clock_calculate\n");

	u32 scgd, cdf, round, ick, sum, scl, cdf_width;
		unsigned long rate;
		struct device *dev = rcar_i2c_priv_to_dev(priv);

		/* Fall back to previously used values if not supplied */
		t->bus_freq_hz = t->bus_freq_hz ?: 100000;
		t->scl_fall_ns = t->scl_fall_ns ?: 35;
		t->scl_rise_ns = t->scl_rise_ns ?: 200;
		t->scl_int_delay_ns = t->scl_int_delay_ns ?: 50;

		switch (priv->devtype) {
		case I2C_RCAR_GEN1:
			cdf_width = 2;
			break;
		case I2C_RCAR_GEN2:
		case I2C_RCAR_GEN3:
			cdf_width = 3;
			break;
		default:
			dev_err(dev, "device type error\n");
			return -EIO;
		}

		/*
		 * calculate SCL clock
		 * see
		 *	ICCCR
		 *
		 * ick	= clkp / (1 + CDF)
		 * SCL	= ick / (20 + SCGD * 8 + F[(ticf + tr + intd) * ick])
		 *
		 * ick  : I2C internal clock < 20 MHz
		 * ticf : I2C SCL falling time
		 * tr   : I2C SCL rising  time
		 * intd : LSI internal delay
		 * clkp : peripheral_clk
		 * F[]  : integer up-valuation
		 */
		rate = clk_get_rate(priv->clk);
		cdf = rate / 20000000;
		if (cdf >= 1U << cdf_width) {
			dev_err(dev, "Input clock %lu too high\n", rate);
			return -EIO;
		}
		ick = rate / (cdf + 1);

		/*
		 * it is impossible to calculate large scale
		 * number on u32. separate it
		 *
		 * F[(ticf + tr + intd) * ick] with sum = (ticf + tr + intd)
		 *  = F[sum * ick / 1000000000]
		 *  = F[(ick / 1000000) * sum / 1000]
		 */
		sum = t->scl_fall_ns + t->scl_rise_ns + t->scl_int_delay_ns;
		round = (ick + 500000) / 1000000 * sum;
		round = (round + 500) / 1000;

		/*
		 * SCL	= ick / (20 + SCGD * 8 + F[(ticf + tr + intd) * ick])
		 *
		 * Calculation result (= SCL) should be less than
		 * bus_speed for hardware safety
		 *
		 * We could use something along the lines of
		 *	div = ick / (bus_speed + 1) + 1;
		 *	scgd = (div - 20 - round + 7) / 8;
		 *	scl = ick / (20 + (scgd * 8) + round);
		 * (not fully verified) but that would get pretty involved
		 */
		for (scgd = 0; scgd < 0x40; scgd++) {
			scl = ick / (20 + (scgd * 8) + round);
			if (scl <= t->bus_freq_hz)
				goto scgd_find;
		}
		dev_err(dev, "it is impossible to calculate best SCL\n");
		return -EIO;

	scgd_find:
		dev_dbg(dev, "clk %d/%d(%lu), round %u, CDF:0x%x, SCGD: 0x%x\n",
			scl, t->bus_freq_hz, clk_get_rate(priv->clk), round, cdf, scgd);

		/* keep icccr value */
		priv->icccr = scgd << cdf_width | cdf;

		return 0;
}

static void rcar_i2c_prepare_msg(struct rcar_i2c_priv *priv)
{
	printk("Hoan_rcar_i2c_prepare_msg ICMSR = %x\n", rcar_i2c_read(priv, ICMSR)& 0xFF);
	int read = !!rcar_i2c_is_recv(priv);

	priv->pos = 0;
	if (priv->msgs_left == 1)
		priv->flags |= ID_LAST_MSG;

	rcar_i2c_write(priv, ICMAR, (priv->msg->addr << 1) | read);
	//rcar_i2c_write(priv, ICMAR, (0x10 << 1) | read);
	printk("read =  %d, priv->pos= %d\n priv->msgs_left =%d\n", read, priv->pos, priv->msgs_left);
	printk("slave addr %x\n", priv->msg->addr);
	printk("value register %x\n ICMSR = %x\n", (priv->msg->addr << 1) | read, rcar_i2c_read(priv, ICMSR)& 0xFF);
	/*
	 * We don't have a testcase but the HW engineers say that the write order
	 * of ICMSR and ICMCR depends on whether we issue START or REP_START. Since
	 * it didn't cause a drawback for me, let's rather be safe than sorry.
	 */
	if (priv->flags & ID_FIRST_MSG) {
		printk("Hoan_rcar_write_out_before\n");
		rcar_i2c_write(priv, ICMSR, 0);
		rcar_i2c_write(priv, ICMCR, RCAR_BUS_PHASE_START);
	} else {
		printk("Hoan_rcar_write_out_after\n");
		rcar_i2c_write(priv, ICMCR, RCAR_BUS_PHASE_START);
		rcar_i2c_write(priv, ICMSR, 0);
	}
	rcar_i2c_write(priv, ICMIER, read ? RCAR_IRQ_RECV : RCAR_IRQ_SEND);
	printk("Hoan_rcar_i2c_prepare_msg_end ICMSR = %x\n",rcar_i2c_read(priv, ICMSR)& 0xFF );
}

static void rcar_i2c_next_msg(struct rcar_i2c_priv *priv)
{
	printk("Hoan_rcar_i2c_next_msg\n");
	priv->msg++;
	priv->msgs_left--;
	priv->flags &= ID_P_MASK;
	rcar_i2c_prepare_msg(priv);
}

/*
 *		interrupt functions
 */

static irqreturn_t rcar_i2c_irq(int irq, void *ptr)
{
	struct rcar_i2c_priv *priv = ptr;
	u32 msr, val;

	/* Clear START or STOP as soon as we can */
	//printk("Hoan_rcar_i2c_irq\n");
	val = rcar_i2c_read(priv, ICMCR);
	rcar_i2c_write(priv, ICMCR, val & RCAR_BUS_MASK_DATA);
	//printk("Hoan_rcar_i2c_irq is_receiver_interrupt =%d\n",rcar_i2c_is_recv(priv));
	u32 test = 0xFF;
    //printk("Hoan_before%x", test );
    test &= RCAR_BUS_MASK_DATA;
    //printk("Hoan_after %x", test );

	msr = rcar_i2c_read(priv, ICMSR);
	printk("Hoan_rcar_i2c_irq rcar_i2c_read(priv, ICMSR) ICMSR =%x", rcar_i2c_read(priv, ICMSR));
	/* Only handle interrupts that are currently enabled */
	msr &= rcar_i2c_read(priv, ICMIER);

	//printk("Hoan_rcar_i2c_irq ICMSR =%x\n",rcar_i2c_read(priv, ICMSR));
	printk("Hoan_rcar_i2c_irq RCAR_IRQ_ACK_SEND =%x ICMIER =%x ICMSR =%x priv->pos= %d\n",RCAR_IRQ_ACK_SEND, rcar_i2c_read(priv, ICMIER) & 0xff, rcar_i2c_read(priv, ICMSR)& 0xff, priv->pos);
	if (!msr) {
		printk("Hoan_rcar_i2c_read(priv, ICMIER)");

		return IRQ_NONE;
	}

	/* Arbitration lost */
	if (msr & MAL) {
		printk("Hoan_(msr & MAL)");
		priv->flags |= ID_DONE | ID_ARBLOST;
		goto out;
	}

	/* Nack */
	if (msr & MNR) {
		printk("Hoan_(msr & MNR)");
		/* HW automatically sends STOP after received NACK */
		rcar_i2c_write(priv, ICMIER, RCAR_IRQ_STOP);
		priv->flags |= ID_NACK;
		goto out;
	}

	/* Stop */
	if (msr & MST) {
		printk("Hoan_(msr & MST)");
		priv->msgs_left--; /* The last message also made it */
		priv->flags |= ID_DONE;
		goto out;
	}


out:
	printk("Hoan_rcar_i2c_irq_OUT");
	if (priv->flags & ID_DONE) {
		printk("Hoan_rcar_i2c_irq_OUT priv->flags & ID_DONE");
		rcar_i2c_write(priv, ICMIER, 0);
		rcar_i2c_write(priv, ICMSR, 0);
		wake_up(&priv->wait);
	}
	//printk("Hoan_rcar_i2c_irq_OUT");
	return IRQ_HANDLED;
}


static int Hoan_i2c_master_xfer( struct i2c_adapter *adap,
		struct i2c_msg *msgs,
		int num)
{
	struct rcar_i2c_priv *priv = i2c_get_adapdata(adap);
	struct device *dev = rcar_i2c_priv_to_dev(priv);
	int i, ret;

	long time_left;

	pm_runtime_get_sync(dev);

//********************************
	rcar_i2c_write(priv, ICMCR, 0);
//********************************

	rcar_i2c_init(priv);
	ret = rcar_i2c_bus_barrier(priv);
		if (ret < 0)
			goto out;

	for (i = 0; i < num; i++) {
			/* This HW can't send STOP after address phase */
		if (msgs[i].len == 0) {
			ret = -EOPNOTSUPP;
			goto out;
		}
	//		rcar_i2c_request_dma(priv, msgs + i);
	}

	priv->flags = (priv->flags & ID_P_MASK) | ID_FIRST_MSG;
	priv->msgs_left = num;
	priv ->msg = msgs;

	i = 0;


	printk("Hoan_rcar_i2c_prepare_msg ICMSR = %x\n", rcar_i2c_read(priv, ICMSR)& 0xFF);
	//int read = !!rcar_i2c_is_recv(priv);

	int read = 0;

	priv->pos = 0;
	if (priv->msgs_left == 1)
		priv->flags |= ID_LAST_MSG;

	printk("value register ICMSR = %x  ICMCR = %x ICMIER =%x ICMAR =%x\n",  rcar_i2c_read(priv, ICMSR)& 0xFF, rcar_i2c_read(priv, ICMCR)& 0xFF, rcar_i2c_read(priv, ICMIER)& 0xFF, rcar_i2c_read(priv, ICMAR)& 0xFF);

	rcar_i2c_write(priv, ICMAR, (priv->msg->addr << 1) | read);
		//rcar_i2c_write(priv, ICMAR, (0x10 << 1) | read);
	printk("read =  %d, priv->pos= %d\n priv->msgs_left =%d\n", read, priv->pos, priv->msgs_left);
	printk("slave addr %x\n", priv->msg->addr);
	printk("value register %x\n ICMSR = %x\n", (priv->msg->addr << 1) | read, rcar_i2c_read(priv, ICMSR)& 0xFF);
	/*
	 * We don't have a testcase but the HW engineers say that the write order
	 * of ICMSR and ICMCR depends on whether we issue START or REP_START. Since
	 * it didn't cause a drawback for me, let's rather be safe than sorry.
	 */
	//if (priv->flags & ID_FIRST_MSG) {
	//	printk("Hoan_rcar_write_out_before\n");



	rcar_i2c_write(priv, ICMSR, 0);
	rcar_i2c_write(priv, ICMCR, RCAR_BUS_PHASE_START);

	printk("ICMSR1 = %x\n", rcar_i2c_read(priv, ICMSR)& 0xFF);

	rcar_i2c_write(priv, ICRXTX, 0x70);
	//udelay(100);
	printk("ICMSR2 = %x\n", rcar_i2c_read(priv, ICMSR)& 0xFF);
#define RCAR_SEND (~(1<<0) & 0xFF)
#define RCAR_ADDR_SEND (~(1<<3) & 0xFF)
	//rcar_i2c_write(priv, ICMSR, RCAR_IRQ_ACK_SEND);
	//rcar_i2c_write(priv, ICMSR, RCAR_SEND);
	rcar_i2c_write(priv, ICMSR, RCAR_ADDR_SEND);
	udelay(100);
	rcar_i2c_write(priv, ICRXTX, 0xef);
	udelay(100);
	rcar_i2c_write(priv, ICMSR, RCAR_ADDR_SEND);
	udelay(100);

	//rcar_i2c_write(priv, ICRXTX, 0xdf);
	//udelay(100);
	//printk("ICMSR3 = %x\n", rcar_i2c_read(priv, ICMSR)& 0xFF);
	//rcar_i2c_write(priv, ICMSR, RCAR_IRQ_ACK_SEND);

	//udelay(100);
	//printk("ICMSR4 = %x\n", rcar_i2c_read(priv, ICMSR)& 0xFF);
	//rcar_i2c_write(priv, ICMSR, RCAR_IRQ_ACK_SEND);
	//udelay(100);



	//rcar_i2c_write(priv, ICRXTX, 0x70);
	//rcar_i2c_write(priv, ICMSR, RCAR_IRQ_ACK_SEND);

	//udelay(100);

	//rcar_i2c_write(priv, ICRXTX, 0xef);
	//rcar_i2c_write(priv, ICMSR, RCAR_IRQ_ACK_SEND);

	//udelay(100);



	//rcar_i2c_write(priv, ICMCR, RCAR_BUS_PHASE_STOP);
	//rcar_i2c_write(priv, ICMSR, RCAR_IRQ_ACK_SEND);


	//udelay(100);


	/* reset master mode */
	rcar_i2c_write(priv, ICMIER, 0);
	rcar_i2c_write(priv, ICMCR, MDBS);
	rcar_i2c_write(priv, ICMSR, 0);


	time_left = wait_event_timeout(priv->wait, priv->flags & ID_DONE, adap->timeout);

	out:
		pm_runtime_put(dev);

		if (ret < 0 && ret != -ENXIO)
			dev_err(dev, "error %d : %x\n", ret, priv->flags);

		return ret;
}



static u32 rcar_i2c_func(struct i2c_adapter *adap)
{
	/*
	 * This HW can't do:
	 * I2C_SMBUS_QUICK (setting FSB during START didn't work)
	 * I2C_M_NOSTART (automatically sends address after START)
	 * I2C_M_IGNORE_NAK (automatically sends STOP after NAK)
	 */
	return I2C_FUNC_I2C | I2C_FUNC_SLAVE |
		(I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm rcar_i2c_algo = {
	//.master_xfer	= rcar_i2c_master_xfer,
	.master_xfer	= Hoan_i2c_master_xfer,
	.functionality	= rcar_i2c_func,
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


static int count = 0;
static int rcar_i2c_probe(struct platform_device *pdev)
{
	count ++;
	printk("Hoan_rcar_i2c_probe lan thu i= %d\n", count);
	struct rcar_i2c_priv *priv;
	struct i2c_adapter *adap;
	struct device *dev = &pdev->dev;
	struct i2c_timings i2c_t;
	int irq, ret;


	priv = devm_kzalloc(dev, sizeof(struct rcar_i2c_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "cannot get clock\n");
		return PTR_ERR(priv->clk);
	}

	priv->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	priv->io = devm_ioremap_resource(dev, priv->res);
	if (IS_ERR(priv->io))
		return PTR_ERR(priv->io);

	priv->devtype = (enum rcar_i2c_type)of_device_get_match_data(dev);
	init_waitqueue_head(&priv->wait);

	printk("Hoan_rcar_rcar_i2c_probe &priv->adap->test = %d\n", priv->adap.test);
	adap = &priv->adap;
	adap->test = 1000;
	printk("Hoan_rcar_rcar_i2c_probe &priv->adap->test = %d\n", priv->adap.test);
	adap->nr = pdev->id;
	adap->algo = &rcar_i2c_algo;
	adap->class = I2C_CLASS_DEPRECATED;
	adap->retries = 3;
	adap->dev.parent = dev;
	adap->dev.of_node = dev->of_node;
	i2c_set_adapdata(adap, priv);
	strlcpy(adap->name, pdev->name, sizeof(adap->name));

	i2c_parse_fw_timings(dev, &i2c_t, false);

	/* Init DMA */
	sg_init_table(&priv->sg, 1);
	priv->dma_direction = DMA_NONE;
	priv->dma_rx = priv->dma_tx = ERR_PTR(-EPROBE_DEFER);

	/* Activate device for clock calculation */
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
	ret = rcar_i2c_clock_calculate(priv, &i2c_t);
	if (ret < 0)
		goto out_pm_put;

	/* Stay always active when multi-master to keep arbitration working */
	if (of_property_read_bool(dev->of_node, "multi-master"))
		priv->flags |= ID_P_PM_BLOCKED;
	else
		pm_runtime_put(dev);


	//irq = platform_get_irq(pdev, 0);
	//ret = devm_request_irq(dev, irq, rcar_i2c_irq, 0, dev_name(dev), priv);
	//if (ret < 0) {
	//	dev_err(dev, "cannot get irq %d\n", irq);
	//	goto out_pm_disable;
	//}

	platform_set_drvdata(pdev, priv);

	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0)
		goto out_pm_disable;

	dev_info(dev, "probed\n");

	return 0;

 out_pm_put:
	pm_runtime_put(dev);
 out_pm_disable:
	pm_runtime_disable(dev);
	printk("Hoan_pm_runtime_disable\n");
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
