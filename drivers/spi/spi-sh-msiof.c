// SPDX-License-Identifier: GPL-2.0
/*
 * SuperH MSIOF SPI Master Interface
 *
 * Copyright (c) 2009 Magnus Damm
 * Copyright (C) 2014 Renesas Electronics Corporation
 * Copyright (C) 2014-2017 Glider bvba
 */

#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <asm/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sh_dma.h>

#include <linux/spi/sh_msiof.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>

static struct dentry *spi_gpio_debug_dir;
//static inline void spi_gpio_fault_injector_init(struct platform_device *pdev) {}

#define h_debug printk("file %s func %s line %d \n", __FILE__, __FUNCTION__, __LINE__);

struct sh_msiof_chipdata {
	u16 tx_fifo_size;
	u16 rx_fifo_size;
	u16 master_flags;
	u16 min_div_pow;
};

struct rcar_sh_msiof_priv {
	void __iomem *mapbase;
	struct clk *clk;
	struct spi_master	*master;
	struct platform_device *pdev;
	struct sh_msiof_spi_info *info;
	unsigned int tx_fifo_size;
	unsigned int rx_fifo_size;

	struct completion done;

	struct dentry *debug_dir;
};

struct sh_msiof_spi_priv {
	struct spi_master *master;
	void __iomem *mapbase;
	struct clk *clk;
	struct platform_device *pdev;
	struct sh_msiof_spi_info *info;
	struct completion done;
	struct completion done_txdma;
	unsigned int tx_fifo_size;
	unsigned int rx_fifo_size;
	unsigned int min_div_pow;
	void *tx_dma_page;
	void *rx_dma_page;
	dma_addr_t tx_dma_addr;
	dma_addr_t rx_dma_addr;
	unsigned short unused_ss;
	bool native_cs_inited;
	bool native_cs_high;
	bool slave_aborted;

	struct dentry *debug_dir;
};

#define MAX_SS	3	/* Maximum number of native chip selects */

#define TMDR1	0x00	/* Transmit Mode Register 1 */
#define TMDR2	0x04	/* Transmit Mode Register 2 */
#define TMDR3	0x08	/* Transmit Mode Register 3 */
#define RMDR1	0x10	/* Receive Mode Register 1 */
#define RMDR2	0x14	/* Receive Mode Register 2 */
#define RMDR3	0x18	/* Receive Mode Register 3 */
#define TSCR	0x20	/* Transmit Clock Select Register */
#define RSCR	0x22	/* Receive Clock Select Register (SH, A1, APE6) */
#define CTR	0x28	/* Control Register */
#define FCTR	0x30	/* FIFO Control Register */
#define STR	0x40	/* Status Register */
#define IER	0x44	/* Interrupt Enable Register */
#define TDR1	0x48	/* Transmit Control Data Register 1 (SH, A1) */
#define TDR2	0x4c	/* Transmit Control Data Register 2 (SH, A1) */
#define TFDR	0x50	/* Transmit FIFO Data Register */
#define RDR1	0x58	/* Receive Control Data Register 1 (SH, A1) */
#define RDR2	0x5c	/* Receive Control Data Register 2 (SH, A1) */
#define RFDR	0x60	/* Receive FIFO Data Register */

/* TMDR1 and RMDR1 */
#define MDR1_TRMD	 0x80000000 /* Transfer Mode (1 = Master mode) */
#define MDR1_SYNCMD_MASK 0x30000000 /* SYNC Mode */
#define MDR1_SYNCMD_SPI	 0x20000000 /*   Level mode/SPI */
#define MDR1_SYNCMD_LR	 0x30000000 /*   L/R mode */
#define MDR1_SYNCAC_SHIFT	 25 /* Sync Polarity (1 = Active-low) */
#define MDR1_BITLSB_SHIFT	 24 /* MSB/LSB First (1 = LSB first) */
#define MDR1_DTDL_SHIFT		 20 /* Data Pin Bit Delay for MSIOF_SYNC */
#define MDR1_SYNCDL_SHIFT	 16 /* Frame Sync Signal Timing Delay */
#define MDR1_FLD_MASK	 0x0000000c /* Frame Sync Signal Interval (0-3) */
#define MDR1_FLD_SHIFT		  2
#define MDR1_XXSTP	 0x00000001 /* Transmission/Reception Stop on FIFO */
/* TMDR1 */
#define TMDR1_PCON	 0x40000000 /* Transfer Signal Connection */
#define TMDR1_SYNCCH_MASK 0xc000000 /* Synchronization Signal Channel Select */
#define TMDR1_SYNCCH_SHIFT	 26 /* 0=MSIOF_SYNC, 1=MSIOF_SS1, 2=MSIOF_SS2 */

/* TMDR2 and RMDR2 */
#define MDR2_BITLEN1(i)	(((i) - 1) << 24) /* Data Size (8-32 bits) */
#define MDR2_WDLEN1(i)	(((i) - 1) << 16) /* Word Count (1-64/256 (SH, A1))) */
#define MDR2_GRPMASK1	0x00000001 /* Group Output Mask 1 (SH, A1) */

/* TSCR and RSCR */
#define SCR_BRPS_MASK	    0x1f00 /* Prescaler Setting (1-32) */
#define SCR_BRPS(i)	(((i) - 1) << 8)
#define SCR_BRDV_MASK	    0x0007 /* Baud Rate Generator's Division Ratio */
#define SCR_BRDV_DIV_2	    0x0000
#define SCR_BRDV_DIV_4	    0x0001
#define SCR_BRDV_DIV_8	    0x0002
#define SCR_BRDV_DIV_16	    0x0003
#define SCR_BRDV_DIV_32	    0x0004
#define SCR_BRDV_DIV_1	    0x0007

/* CTR */
#define CTR_TSCKIZ_MASK	0xc0000000 /* Transmit Clock I/O Polarity Select */
#define CTR_TSCKIZ_SCK	0x80000000 /*   Disable SCK when TX disabled */
#define CTR_TSCKIZ_POL_SHIFT	30 /*   Transmit Clock Polarity */
#define CTR_RSCKIZ_MASK	0x30000000 /* Receive Clock Polarity Select */
#define CTR_RSCKIZ_SCK	0x20000000 /*   Must match CTR_TSCKIZ_SCK */
#define CTR_RSCKIZ_POL_SHIFT	28 /*   Receive Clock Polarity */
#define CTR_TEDG_SHIFT		27 /* Transmit Timing (1 = falling edge) */
#define CTR_REDG_SHIFT		26 /* Receive Timing (1 = falling edge) */
#define CTR_TXDIZ_MASK	0x00c00000 /* Pin Output When TX is Disabled */
#define CTR_TXDIZ_LOW	0x00000000 /*   0 */
#define CTR_TXDIZ_HIGH	0x00400000 /*   1 */
#define CTR_TXDIZ_HIZ	0x00800000 /*   High-impedance */
#define CTR_TSCKE	0x00008000 /* Transmit Serial Clock Output Enable */
#define CTR_TFSE	0x00004000 /* Transmit Frame Sync Signal Output Enable */
#define CTR_TXE		0x00000200 /* Transmit Enable */
#define CTR_RXE		0x00000100 /* Receive Enable */

/* FCTR */
#define FCTR_TFWM_MASK	0xe0000000 /* Transmit FIFO Watermark */
#define FCTR_TFWM_64	0x00000000 /*  Transfer Request when 64 empty stages */
#define FCTR_TFWM_32	0x20000000 /*  Transfer Request when 32 empty stages */
#define FCTR_TFWM_24	0x40000000 /*  Transfer Request when 24 empty stages */
#define FCTR_TFWM_16	0x60000000 /*  Transfer Request when 16 empty stages */
#define FCTR_TFWM_12	0x80000000 /*  Transfer Request when 12 empty stages */
#define FCTR_TFWM_8	0xa0000000 /*  Transfer Request when 8 empty stages */
#define FCTR_TFWM_4	0xc0000000 /*  Transfer Request when 4 empty stages */
#define FCTR_TFWM_1	0xe0000000 /*  Transfer Request when 1 empty stage */
#define FCTR_TFUA_MASK	0x07f00000 /* Transmit FIFO Usable Area */
#define FCTR_TFUA_SHIFT		20
#define FCTR_TFUA(i)	((i) << FCTR_TFUA_SHIFT)
#define FCTR_RFWM_MASK	0x0000e000 /* Receive FIFO Watermark */
#define FCTR_RFWM_1	0x00000000 /*  Transfer Request when 1 valid stages */
#define FCTR_RFWM_4	0x00002000 /*  Transfer Request when 4 valid stages */
#define FCTR_RFWM_8	0x00004000 /*  Transfer Request when 8 valid stages */
#define FCTR_RFWM_16	0x00006000 /*  Transfer Request when 16 valid stages */
#define FCTR_RFWM_32	0x00008000 /*  Transfer Request when 32 valid stages */
#define FCTR_RFWM_64	0x0000a000 /*  Transfer Request when 64 valid stages */
#define FCTR_RFWM_128	0x0000c000 /*  Transfer Request when 128 valid stages */
#define FCTR_RFWM_256	0x0000e000 /*  Transfer Request when 256 valid stages */
#define FCTR_RFUA_MASK	0x00001ff0 /* Receive FIFO Usable Area (0x40 = full) */
#define FCTR_RFUA_SHIFT		 4
#define FCTR_RFUA(i)	((i) << FCTR_RFUA_SHIFT)

/* STR */
#define STR_TFEMP	0x20000000 /* Transmit FIFO Empty */
#define STR_TDREQ	0x10000000 /* Transmit Data Transfer Request */
#define STR_TEOF	0x00800000 /* Frame Transmission End */
#define STR_TFSERR	0x00200000 /* Transmit Frame Synchronization Error */
#define STR_TFOVF	0x00100000 /* Transmit FIFO Overflow */
#define STR_TFUDF	0x00080000 /* Transmit FIFO Underflow */
#define STR_RFFUL	0x00002000 /* Receive FIFO Full */
#define STR_RDREQ	0x00001000 /* Receive Data Transfer Request */
#define STR_REOF	0x00000080 /* Frame Reception End */
#define STR_RFSERR	0x00000020 /* Receive Frame Synchronization Error */
#define STR_RFUDF	0x00000010 /* Receive FIFO Underflow */
#define STR_RFOVF	0x00000008 /* Receive FIFO Overflow */

/* IER */
#define IER_TDMAE	0x80000000 /* Transmit Data DMA Transfer Req. Enable */
#define IER_TFEMPE	0x20000000 /* Transmit FIFO Empty Enable */
#define IER_TDREQE	0x10000000 /* Transmit Data Transfer Request Enable */
#define IER_TEOFE	0x00800000 /* Frame Transmission End Enable */
#define IER_TFSERRE	0x00200000 /* Transmit Frame Sync Error Enable */
#define IER_TFOVFE	0x00100000 /* Transmit FIFO Overflow Enable */
#define IER_TFUDFE	0x00080000 /* Transmit FIFO Underflow Enable */
#define IER_RDMAE	0x00008000 /* Receive Data DMA Transfer Req. Enable */
#define IER_RFFULE	0x00002000 /* Receive FIFO Full Enable */
#define IER_RDREQE	0x00001000 /* Receive Data Transfer Request Enable */
#define IER_REOFE	0x00000080 /* Frame Reception End Enable */
#define IER_RFSERRE	0x00000020 /* Receive Frame Sync Error Enable */
#define IER_RFUDFE	0x00000010 /* Receive FIFO Underflow Enable */
#define IER_RFOVFE	0x00000008 /* Receive FIFO Overflow Enable */

static u32 h_sh_msiof_read(struct rcar_sh_msiof_priv *p, int reg_offs)
{
	//printk("file %s func %s line %d reg_offs 0x%x", __FILE__, __FUNCTION__, __LINE__, reg_offs);
	switch (reg_offs) {

	case TSCR:
	case RSCR:
		return ioread16(p->mapbase + reg_offs);
	default:
		return ioread32(p->mapbase + reg_offs);
	}
}

static void h_sh_msiof_write(struct rcar_sh_msiof_priv *p, int reg_offs,
			   u32 value)
{
	//printk("file %s func %s line %d reg_offs 0x%x value 0x%lx", __FILE__, __FUNCTION__, __LINE__, reg_offs, value);
	switch (reg_offs) {

	case TSCR:
	case RSCR:
		iowrite16(value, p->mapbase + reg_offs);
		break;
	default:
		iowrite32(value, p->mapbase + reg_offs);
		break;
	}
}

static void h_sh_msiof_read_reg_inf(struct rcar_sh_msiof_priv *p)
{
	printk("TMDR1 %x TMDR2 %x TMDR3 %x RMDR1 %x RMDR2 %x RMDR3 %x CTR %x TSCR %x TFDR %x RFDR %x STR %x IER %x FCTR %x",
			h_sh_msiof_read(p, TMDR1), h_sh_msiof_read(p, TMDR2), h_sh_msiof_read(p, TMDR3), h_sh_msiof_read(p, RMDR1), h_sh_msiof_read(p, RMDR2)
			, h_sh_msiof_read(p, RMDR3), h_sh_msiof_read(p, CTR), h_sh_msiof_read(p, TSCR), h_sh_msiof_read(p, TFDR), h_sh_msiof_read(p, RFDR)
			, h_sh_msiof_read(p, STR), h_sh_msiof_read(p, IER), h_sh_msiof_read(p, FCTR));
}

static irqreturn_t sh_msiof_spi_irq(int irq, void *data)
{
	//struct sh_msiof_spi_priv *p = data;

	h_debug;
	/* just disable the interrupt and wake up */
	//sh_msiof_write(p, IER, 0);
	//complete(&p->done);

	return IRQ_HANDLED;
}

static irqreturn_t h_sh_msiof_spi_irq(int irq, void *data)
{
	struct sh_msiof_spi_priv *p = data;
	u32 val_reg;

	h_sh_msiof_write(p, STR, h_sh_msiof_read(p, STR) &
			(STR_TEOF | STR_REOF));

	return IRQ_HANDLED;
}


static int h_sh_msiof_spi_setup(struct spi_device *spi)
{
	struct device_node	*np = spi->master->dev.of_node;
	struct rcar_sh_msiof_priv *p = spi_master_get_devdata(spi->master);
	int ret = 0;

	if (!np) {
		/*
		 * Use spi->controller_data for CS (same strategy as spi_gpio),
		 * if any. otherwise let HW control CS
		 */
		spi->cs_gpio = (uintptr_t)spi->controller_data;
	}

	if (!gpio_is_valid(spi->cs_gpio)) {
		dev_err(&spi->dev, "%d is not a valid gpio\n",
				spi->cs_gpio);
		return -EINVAL;
	}

	if (gpio_is_valid(spi->cs_gpio)) {
		ret = gpio_direction_output(spi->cs_gpio, !(spi->mode & SPI_CS_HIGH));
		h_sh_msiof_read_reg_inf(p);
		printk("file %s func %s line %d ret %d spi->cs_gpio %d", __FILE__, __FUNCTION__, __LINE__, ret, spi->cs_gpio);
		return 0;
	}

	h_sh_msiof_read_reg_inf(p);
	if(!ret)
		h_debug;

	return ret;
}


static int h_sh_msiof_prepare_message(struct spi_master *master,
				    struct spi_message *msg)
{
	struct rcar_sh_msiof_priv *p = spi_master_get_devdata(master);
	struct spi_device *spi = msg->spi;
	//printk("file %s func %s line %d spi->mode %x", __FILE__, __FUNCTION__, __LINE__, spi->mode);
	u32 val_reg = 0;

	/* Master mode on TMDR1 and RMDR1 */
	h_sh_msiof_write(p, TMDR1, MDR1_TRMD | TMDR1_PCON | MDR1_SYNCMD_SPI | MDR1_XXSTP | (1 << MDR1_FLD_SHIFT));
	h_sh_msiof_write(p, RMDR1, MDR1_SYNCMD_SPI | MDR1_XXSTP | (1 << MDR1_FLD_SHIFT));
	printk("file %s func %s line %d CTR %x \n", __FILE__, __FUNCTION__, __LINE__, h_sh_msiof_read(p, CTR));
	/* MSIOF CTR This is depend on CPOL and CPHA */

	/* TSCR */
	h_sh_msiof_write(p, TSCR, 0x1004);
	/* IER */
	h_sh_msiof_write(p, IER, STR_TEOF |  STR_REOF);

	/* SPI_CPOL  SPI_CPHA  SPI_LSB_FIRST    =>   MSIOF CTR */
	val_reg = h_sh_msiof_read(p, CTR);
	val_reg &= ~ CTR_TSCKIZ_MASK;
	val_reg &= ~ CTR_RSCKIZ_MASK;
	if(spi->mode & SPI_CPOL) {
		val_reg |= (3 << CTR_TSCKIZ_POL_SHIFT);
		val_reg |= (3 << CTR_RSCKIZ_POL_SHIFT);
	}
	else {
		val_reg |= (2 << CTR_TSCKIZ_POL_SHIFT);
		val_reg |= (2 << CTR_RSCKIZ_POL_SHIFT);
	}
	h_sh_msiof_write(p, CTR, val_reg);
	printk("file %s func %s line %d CTR %x \n", __FILE__, __FUNCTION__, __LINE__, h_sh_msiof_read(p, CTR));
	val_reg = h_sh_msiof_read(p, CTR);
	val_reg &= ~ (1 << CTR_TEDG_SHIFT);
	val_reg &= ~ (1 << CTR_REDG_SHIFT);
	if(!(spi->mode & SPI_CPHA)) {
		val_reg |= (1 << CTR_TEDG_SHIFT);
		val_reg |= (1 << CTR_REDG_SHIFT);
	}

	h_sh_msiof_write(p, CTR, val_reg);
	printk("file %s func %s line %d CTR %x \n", __FILE__, __FUNCTION__, __LINE__, h_sh_msiof_read(p, CTR));
	val_reg = h_sh_msiof_read(p, TMDR1);
	val_reg &= ~ (1 << MDR1_BITLSB_SHIFT);
	h_sh_msiof_write(p, TMDR1, val_reg);

	val_reg = h_sh_msiof_read(p, RMDR1);
	val_reg &= ~ (1 << MDR1_BITLSB_SHIFT);
	h_sh_msiof_write(p, RMDR1, val_reg);

	if (spi->mode & SPI_LSB_FIRST) {
		val_reg = h_sh_msiof_read(p, TMDR1);
		val_reg &= ~ (1 << MDR1_BITLSB_SHIFT);
		val_reg |= (1 << MDR1_BITLSB_SHIFT);
		h_sh_msiof_write(p, TMDR1, val_reg);

		val_reg = h_sh_msiof_read(p, RMDR1);
		val_reg &= ~ (1 << MDR1_BITLSB_SHIFT);
		val_reg |= (1 << MDR1_BITLSB_SHIFT);
		h_sh_msiof_write(p, RMDR1, val_reg);
	}

	printk("file %s func %s line %d CTR %x \n", __FILE__, __FUNCTION__, __LINE__, h_sh_msiof_read(p, CTR));
	//h_sh_msiof_read_reg_inf(p);
	return 0;
}


static int i = 0;

static int h_pm_test;

static int h_sh_msiof_transfer_word(struct rcar_sh_msiof_priv *p, u8 *data, unsigned nbytes)
{
	unsigned int i = 0;
	u32 d_wrfifo = 0;
	u32 tmp_TMDR2 = 0;

	printk("file %s func %s line %d nbytes %d data[0] %x", __FILE__, __FUNCTION__, __LINE__, nbytes, data[0]);
	for(i = 0; i < nbytes; i++)
	{
		d_wrfifo |= data[i] << (8 * (4 - (i + 1)));
		printk("file %s func %s line %d i %d data[%d] %x d_wrfifo %x", __FILE__, __FUNCTION__, __LINE__, i, i, data[i], d_wrfifo);
	}

	tmp_TMDR2 = h_sh_msiof_read(p, TMDR2);
	printk("file %s func %s line %d tmp_TMDR2 %x", __FILE__, __FUNCTION__, __LINE__, tmp_TMDR2);

	//tmp_TMDR2 &= ~(31 << 24);
	tmp_TMDR2 |= ((nbytes * 8 - 1) << 24) ;
	printk("file %s func %s line %d tmp_TMDR2 %x", __FILE__, __FUNCTION__, __LINE__, tmp_TMDR2);
	h_sh_msiof_write(p, TMDR2, tmp_TMDR2);

	h_sh_msiof_write(p, IER, 0x00);
	h_sh_msiof_write(p, FCTR, 0x3f00000);

	/*Config TFDR*/
	h_sh_msiof_write(p, TFDR , d_wrfifo);
	mdelay(100);
	h_sh_msiof_write(p, CTR , 0xac00c200);
	mdelay(1000);

	mdelay(5000);

	return 0;
}

static int h_sh_msiof_transfer_bytes(struct rcar_sh_msiof_priv *p, u8 *data)
{
	u32 tmp_TMDR2 = 0;
	u32 d_wrfifo = 0;

	d_wrfifo |= data[i] << 24;
	h_sh_msiof_write(p, TFDR , d_wrfifo);
	return 0;
}
static int n_transfer_one;

static int h_sh_msiof_write_to_fifo8(struct rcar_sh_msiof_priv *p, u8 *data, int bytes)
{
	unsigned int i;
	u32 d_wrfifo = 0;

}

static int h_sh_msiof_transfer_once(struct rcar_sh_msiof_priv *p, u8 *tx_data, u8 *rx_data, int bytes)
{
	u32 tmp_TMDR2 = 0;
	u32 tmp_RMDR2 = 0;
	u32 d_wrfifo = 0;
	u32 tmp_STR;
	unsigned int i;

	printk("file %s func %s line %d bytes %d", __FILE__, __FUNCTION__, __LINE__, bytes);
	tmp_TMDR2 = h_sh_msiof_read(p, TMDR2);
	/* Clear  WDLEN1[7:0] and resetting these bits*/
	tmp_TMDR2 &= ~(255 << 16);
	tmp_TMDR2 |= ((bytes - 1) << 16);
	h_sh_msiof_write(p, TMDR2, tmp_TMDR2);

	tmp_RMDR2 = h_sh_msiof_read(p, RMDR2);
	tmp_RMDR2 &= ~(255 << 16);
	tmp_RMDR2 |= ((bytes - 1) << 16);
	h_sh_msiof_write(p, RMDR2, tmp_RMDR2);

	h_sh_msiof_write(p, IER, IER_TEOFE | IER_REOFE);

	for (i = 0; i < bytes; i++)
	{
		d_wrfifo = tx_data[i] << 24;
		h_sh_msiof_write(p, TFDR , d_wrfifo);
	}

	printk("file %s func %s line %d CTR %x \n", __FILE__, __FUNCTION__, __LINE__, h_sh_msiof_read(p, CTR));
	if(rx_data)
		h_sh_msiof_write(p, CTR , 0xac00c300);
	else
		h_sh_msiof_write(p, CTR , 0xac00c200);

	udelay(1000);
	tmp_STR = h_sh_msiof_read(p, STR);
	/* wait for complete */
	for (i = 100; i > 0; i--) {
		tmp_STR = h_sh_msiof_read(p, STR);
		if(tmp_STR & STR_TEOF)
			break;
		udelay(10);
	}

	if(!(i > 0))
		printk("file %s func %s line %d err timeout", __FILE__, __FUNCTION__, __LINE__);

	if(rx_data) {
		for (i = 0; i < bytes; i++)
			rx_data[i] = ((u32)h_sh_msiof_read(p, RFDR)) >> 24;
	}

	h_sh_msiof_read_reg_inf(p);
	/*Clear status*/
	h_sh_msiof_write(p, STR , tmp_STR);
	h_sh_msiof_write(p, CTR , 0xac000000);
	h_sh_msiof_write(p, CTR , 0xac000003);

	return 0;
}

static int h_sh_msiof_transfer_one(struct spi_master *master,
				 struct spi_device *spi,
				 struct spi_transfer *t)
{

	unsigned int i = 0, n_words = 0;
	unsigned int nbytes;
	struct rcar_sh_msiof_priv *p = spi_master_get_devdata(master);
	unsigned int len = t->len;
	const void *tx_buf = t->tx_buf;
    void *rx_buf = t->rx_buf;
	unsigned int bits = t->bits_per_word;
	unsigned int sz_tx = sizeof(*tx_buf);
	u32 tmp_TMDR2 = 0;
	u32 tmp_RMDR2 = 0;

	printk("file %s func %s line %d t->len %d *tx_buf %x *rx_buf %x \n",
			__FILE__, __FUNCTION__, __LINE__, t->len, tx_buf, rx_buf);

	u8 *tx_data = t->tx_buf;
	u8 *rx_data = t->rx_buf;

	/* Configure Clock */
	h_sh_msiof_write(p, TSCR, 0x1004);

	n_words = len;
	nbytes = 1;
	tmp_TMDR2 |= ((nbytes * 8 - 1) << 24) ;
	h_sh_msiof_write(p, TMDR2, tmp_TMDR2);
	tmp_RMDR2 |= ((nbytes * 8 - 1) << 24);
	h_sh_msiof_write(p, RMDR2, tmp_RMDR2);
	for (i = 0; i < n_words / 64; i++)
	{
		h_sh_msiof_transfer_once(p, tx_data, rx_data, 64);
		tx_data += 64;

		if(rx_data)
			rx_data += 64;
	}

	if (n_words % 64)
		h_sh_msiof_transfer_once(p, tx_data, rx_data, n_words % 64);

	return 0;
}


static const struct sh_msiof_chipdata sh_data = {
	.tx_fifo_size = 64,
	.rx_fifo_size = 64,
	.master_flags = 0,
	.min_div_pow = 0,
};

static const struct sh_msiof_chipdata rcar_gen2_data = {
	.tx_fifo_size = 64,
	.rx_fifo_size = 64,
	.master_flags = SPI_MASTER_MUST_TX,
	.min_div_pow = 0,
};

static const struct sh_msiof_chipdata rcar_gen3_data = {
	.tx_fifo_size = 64,
	.rx_fifo_size = 64,
	.master_flags = SPI_MASTER_MUST_TX,
	.min_div_pow = 1,
};

static const struct of_device_id sh_msiof_match[] = {
	{ .compatible = "renesas,sh-mobile-msiof", .data = &sh_data },
	{ .compatible = "renesas,msiof-r8a7743",   .data = &rcar_gen2_data },
	{ .compatible = "renesas,msiof-r8a7745",   .data = &rcar_gen2_data },
	{ .compatible = "renesas,msiof-r8a7790",   .data = &rcar_gen2_data },
	{ .compatible = "renesas,msiof-r8a7791",   .data = &rcar_gen2_data },
	{ .compatible = "renesas,msiof-r8a7792",   .data = &rcar_gen2_data },
	{ .compatible = "renesas,msiof-r8a7793",   .data = &rcar_gen2_data },
	{ .compatible = "renesas,msiof-r8a7794",   .data = &rcar_gen2_data },
	{ .compatible = "renesas,rcar-gen2-msiof", .data = &rcar_gen2_data },
	{ .compatible = "renesas,msiof-r8a7796",   .data = &rcar_gen3_data },
	{ .compatible = "renesas,rcar-gen3-msiof", .data = &rcar_gen3_data },
	{ .compatible = "renesas,sh-msiof",        .data = &sh_data }, /* Deprecated */
	{},
};
MODULE_DEVICE_TABLE(of, sh_msiof_match);

#ifdef CONFIG_OF
static struct sh_msiof_spi_info *sh_msiof_spi_parse_dt(struct device *dev)
{
	struct sh_msiof_spi_info *info;
	struct device_node *np = dev->of_node;
	u32 num_cs = 1;

	info = devm_kzalloc(dev, sizeof(struct sh_msiof_spi_info), GFP_KERNEL);
	if (!info)
		return NULL;

	info->mode = of_property_read_bool(np, "spi-slave") ? MSIOF_SPI_SLAVE
							    : MSIOF_SPI_MASTER;

	/* Parse the MSIOF properties */
	if (info->mode == MSIOF_SPI_MASTER)
		of_property_read_u32(np, "num-cs", &num_cs);
	of_property_read_u32(np, "renesas,tx-fifo-size",
					&info->tx_fifo_override);
	of_property_read_u32(np, "renesas,rx-fifo-size",
					&info->rx_fifo_override);
	of_property_read_u32(np, "renesas,dtdl", &info->dtdl);
	of_property_read_u32(np, "renesas,syncdl", &info->syncdl);

	info->num_chipselect = num_cs;

	printk("file %s func %s line %d num_cs %d",
				__FILE__, __FUNCTION__, __LINE__, num_cs);
	return info;
}
#else
static struct sh_msiof_spi_info *sh_msiof_spi_parse_dt(struct device *dev)
{
	return NULL;
}
#endif


static int h_sh_msiof_get_cs_gpios(struct rcar_sh_msiof_priv *p)
{
	struct device *dev = &p->pdev->dev;
	unsigned int used_ss_mask = 0;
	unsigned int cs_gpios = 0;
	unsigned int num_cs, i;
	int ret;

	ret = gpiod_count(dev, "cs");
	if (ret <= 0)
		return 0;

	num_cs = max_t(unsigned int, ret, p->master->num_chipselect);

	for (i = 0; i < num_cs; i++) {
		struct gpio_desc *gpiod;

		gpiod = devm_gpiod_get_index(dev, "cs", i, GPIOD_ASIS);
		if (!IS_ERR(gpiod)) {
			cs_gpios++;
			continue;
		}
	}

	return 0;
}

static void h_sh_msiof_spi_cleanup(struct spi_device *spi)
{
	h_debug;
	/* de-activate cs-gpio */
	gpio_direction_output(spi->cs_gpio, !(spi->mode & SPI_CS_HIGH));
}

static int h_sh_msiof_spi_prepare_hardware(struct spi_master *master)
{
	h_debug;
	return 0;
}


static int h_sh_msiof_spi_unprepare_hardware(struct spi_master *master)
{
	h_debug;
	return 0;
}

static int h_sh_msiof_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct rcar_sh_msiof_priv *priv;
	struct device *dev = &pdev->dev;
	struct resource *res;
	const struct sh_msiof_chipdata *chipdata;
	struct sh_msiof_spi_info *info;
	int i;
	int ret;

	h_debug;

	chipdata = of_device_get_match_data(&pdev->dev);
	if (chipdata) {
		info = sh_msiof_spi_parse_dt(&pdev->dev);
	}


	master = spi_alloc_master(&pdev->dev, sizeof(*priv));
	if (!master)
		return -ENOMEM;

	priv = spi_master_get_devdata(master);
	priv->master = master;
	platform_set_drvdata(pdev, priv);
	pm_runtime_enable(dev);
	//pm_runtime_get_sync(dev);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->mapbase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->mapbase))
		return PTR_ERR(priv->mapbase);

	priv->pdev = pdev;
	priv->info = info;

	priv->tx_fifo_size = chipdata->tx_fifo_size;
	priv->rx_fifo_size = chipdata->rx_fifo_size;

	/* Setup GPIO chip selects */
	//master->num_chipselect = priv->info->num_chipselect;
	master->num_chipselect	= 1; /* single chip-select */
	ret = h_sh_msiof_get_cs_gpios(priv);
	if (ret)
		printk("file %s func %s line %d CS error", __FILE__, __FUNCTION__, __LINE__);

	i = platform_get_irq(pdev, 0);
	if (i < 0) {
		dev_err(&pdev->dev, "cannot get IRQ\n");
		ret = i;
		goto err1;
	}

	ret = devm_request_irq(&pdev->dev, i, h_sh_msiof_spi_irq, 0,
			       dev_name(&pdev->dev), priv);

	if (ret) {
		dev_err(&pdev->dev, "unable to request irq\n");
		goto err1;
	}

	/* get clock */
	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "clk not found\n");
		ret = PTR_ERR(priv->clk);
		goto err1;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto err1;

	/* init master code */
	master->dev.of_node	= pdev->dev.of_node;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->mode_bits |= SPI_LSB_FIRST | SPI_3WIRE;
	//	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	/*
	master->num_chipselect = p->info->num_chipselect;
	ret = sh_msiof_get_cs_gpios(p);
	if (ret)
		goto err1;
	 *
	  */
	master->max_speed_hz	= clk_get_rate(priv->clk);
	master->setup		= h_sh_msiof_spi_setup;
	master->cleanup		= h_sh_msiof_spi_cleanup;
	master->flags		= SPI_MASTER_MUST_TX | SPI_MASTER_MUST_RX;
	//master->flags = chipdata->master_flags;
	master->bits_per_word_mask	= SPI_BPW_MASK(8) | SPI_BPW_MASK(16) |
					  SPI_BPW_MASK(32);
	//SPI_BPW_RANGE_MASK(8, 32);
	master->transfer_one		= h_sh_msiof_transfer_one;
	master->prepare_message		= h_sh_msiof_prepare_message;
	//master->unprepare_message	= pic32_spi_unprepare_message;
	master->prepare_transfer_hardware	= h_sh_msiof_spi_prepare_hardware;
	master->unprepare_transfer_hardware	= h_sh_msiof_spi_unprepare_hardware;

	priv->pdev = pdev;
	init_completion(&priv->done);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret < 0) {
		dev_err(&pdev->dev, "spi_register_master error.\n");
		goto err1;
	}

	return ret;

	err1:
	printk("file %s func %s err1 line %d ", __FILE__, __FUNCTION__, __LINE__);
	return 0;

}

static int sh_msiof_spi_remove(struct platform_device *pdev)
{
	struct sh_msiof_spi_priv *p = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	return 0;
}

static const struct platform_device_id spi_driver_ids[] = {
	{ "spi_sh_msiof",	(kernel_ulong_t)&sh_data },
	{},
};
MODULE_DEVICE_TABLE(platform, spi_driver_ids);

#ifdef CONFIG_PM_SLEEP
static int sh_msiof_spi_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sh_msiof_spi_priv *p = platform_get_drvdata(pdev);

	return spi_master_suspend(p->master);
}

static int sh_msiof_spi_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sh_msiof_spi_priv *p = platform_get_drvdata(pdev);

	return spi_master_resume(p->master);
}

static SIMPLE_DEV_PM_OPS(sh_msiof_spi_pm_ops, sh_msiof_spi_suspend,
			 sh_msiof_spi_resume);
#define DEV_PM_OPS	&sh_msiof_spi_pm_ops
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver sh_msiof_spi_drv = {
	.probe		= h_sh_msiof_spi_probe,
	.remove		= sh_msiof_spi_remove,
	.id_table	= spi_driver_ids,
	.driver		= {
		.name		= "spi_sh_msiof",
		.pm		= DEV_PM_OPS,
		.of_match_table = of_match_ptr(sh_msiof_match),
	},
};
module_platform_driver(sh_msiof_spi_drv);

MODULE_DESCRIPTION("SuperH MSIOF SPI Master Interface Driver");
MODULE_AUTHOR("Magnus Damm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:spi_sh_msiof");
