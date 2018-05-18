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







static void Hoan_i2c_write(struct rcar_i2c_priv *priv, int reg, u32 val)
{
	writel(val, priv->io + reg);
}

static u32 Hoan_i2c_read(struct rcar_i2c_priv *priv, int reg)
{
	return readl(priv->io + reg);
}

static int Hoan_i2c_probe(struct platform_device *dev)
{
	return 0;
}
static int Hoan_i2c_remove(struct platform_device *dev)
{
	return 0;
}
static int Hoan_calc_CRC(struct rcar_i2c_priv* priv, struct i2c_timings* t)
{
	return 0;
}

static u32 Hoan_i2c_func(struct i2c_adapter *adap)
{
	return 0;
}

static int Hoan_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	return 0;
}

static int Hoan_i2c_busy_check(struct rcar_i2c_priv* priv)
{
	return 0;
}

static void Hoan_i2c_msend(struct rcar_i2c_priv* priv)
{
}

static void Hoan_i2c_mrecv(struct rcar_i2c_priv* priv)
{
}

static irqreturn_t Hoan_i2c_irq(int irq, void *ptr)
{
	return IRQ_HANDLED;
}

static const struct i2c_algorithm Hoan_i2c_algo =
{
	.master_xfer	= Hoan_i2c_master_xfer,
		.functionality	= Hoan_i2c_func,
};

