// SPDX-License-Identifier: GPL-2.0
/*
 *  R-Car Gen3 THS thermal sensor driver
 *  Based on rcar_thermal.c and work from Hien Dang and Khiem Nguyen.
 *
 * Copyright (C) 2016 Renesas Electronics Corporation.
 * Copyright (C) 2016 Sang Engineering
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/sys_soc.h>
#include <linux/thermal.h>

#include "thermal_core.h"

/* Register offsets */
#define REG_GEN3_IRQSTR		0x04
#define REG_GEN3_IRQMSK		0x08
#define REG_GEN3_IRQCTL		0x0C
#define REG_GEN3_IRQEN		0x10
#define REG_GEN3_IRQTEMP1	0x14
#define REG_GEN3_IRQTEMP2	0x18
#define REG_GEN3_IRQTEMP3	0x1C
#define REG_GEN3_CTSR		0x20
#define REG_GEN3_THCTR		0x20
#define REG_GEN3_TEMP		0x28
#define REG_GEN3_THCODE1	0x50
#define REG_GEN3_THCODE2	0x54
#define REG_GEN3_THCODE3	0x58

/* IRQ{STR,MSK,EN} bits */
#define IRQ_TEMP1		BIT(0)
#define IRQ_TEMP2		BIT(1)
#define IRQ_TEMP3		BIT(2)
#define IRQ_TEMPD1		BIT(3)
#define IRQ_TEMPD2		BIT(4)
#define IRQ_TEMPD3		BIT(5)

/* CTSR bits */
#define CTSR_PONM	BIT(8)
#define CTSR_AOUT	BIT(7)
#define CTSR_THBGR	BIT(5)
#define CTSR_VMEN	BIT(4)
#define CTSR_VMST	BIT(1)
#define CTSR_THSST	BIT(0)

/* THCTR bits */
#define THCTR_PONM	BIT(6)
#define THCTR_THSST	BIT(0)

#define CTEMP_MASK	0xFFF

#define MCELSIUS(temp)	((temp) * 1000)
#define GEN3_FUSE_MASK	0xFFF

#define TSC_MAX_NUM	3

/* Structure for thermal temperature calculation */
struct equation_coefs {
	int a1;
	int b1;
	int a2;
	int b2;
};

struct rcar_gen3_thermal_tsc {
	void __iomem *base;
	struct thermal_zone_device *zone;
	struct equation_coefs coef;
	int low;
	int high;
};

struct rcar_gen3_thermal_priv {
	struct rcar_gen3_thermal_tsc *tscs[TSC_MAX_NUM];
	unsigned int num_tscs;
	spinlock_t lock; /* Protect interrupts on and off */
	void (*thermal_init)(struct rcar_gen3_thermal_tsc *tsc);
};

static inline u32 rcar_gen3_thermal_read(struct rcar_gen3_thermal_tsc *tsc,
					 u32 reg)
{
	return ioread32(tsc->base + reg);
}

static inline void rcar_gen3_thermal_write(struct rcar_gen3_thermal_tsc *tsc,
					   u32 reg, u32 data)
{
	iowrite32(data, tsc->base + reg);
}

/*
 * Linear approximation for temperature
 *
 * [reg] = [temp] * a + b => [temp] = ([reg] - b) / a
 *
 * The constants a and b are calculated using two triplets of int values PTAT
 * and THCODE. PTAT and THCODE can either be read from hardware or use hard
 * coded values from driver. The formula to calculate a and b are taken from
 * BSP and sparsely documented and understood.
 *
 * Examining the linear formula and the formula used to calculate constants a
 * and b while knowing that the span for PTAT and THCODE values are between
 * 0x000 and 0xfff the largest integer possible is 0xfff * 0xfff == 0xffe001.
 * Integer also needs to be signed so that leaves 7 bits for binary
 * fixed point scaling.
 */

#define FIXPT_SHIFT 7
#define FIXPT_INT(_x) ((_x) << FIXPT_SHIFT)
#define INT_FIXPT(_x) ((_x) >> FIXPT_SHIFT)
#define FIXPT_DIV(_a, _b) DIV_ROUND_CLOSEST(((_a) << FIXPT_SHIFT), (_b))
#define FIXPT_TO_MCELSIUS(_x) ((_x) * 1000 >> FIXPT_SHIFT)

#define RCAR3_THERMAL_GRAN 500 /* mili Celsius */

/* no idea where these constants come from */
#define TJ_1 116
#define TJ_3 -41

static void rcar_gen3_thermal_calc_coefs(struct equation_coefs *coef,
					 int *ptat, int *thcode)
{
	int tj_2;

	/* TODO: Find documentation and document constant calculation formula */

	/*
	 * Division is not scaled in BSP and if scaled it might overflow
	 * the dividend (4095 * 4095 << 14 > INT_MAX) so keep it unscaled
	 */
	tj_2 = (FIXPT_INT((ptat[1] - ptat[2]) * 157)
		/ (ptat[0] - ptat[2])) - FIXPT_INT(41);

	coef->a1 = FIXPT_DIV(FIXPT_INT(thcode[1] - thcode[2]),
			     tj_2 - FIXPT_INT(TJ_3));
	coef->b1 = FIXPT_INT(thcode[2]) - coef->a1 * TJ_3;

	coef->a2 = FIXPT_DIV(FIXPT_INT(thcode[1] - thcode[0]),
			     tj_2 - FIXPT_INT(TJ_1));
	coef->b2 = FIXPT_INT(thcode[0]) - coef->a2 * TJ_1;
}

static int rcar_gen3_thermal_round(int temp)
{
	int result, round_offs;

	round_offs = temp >= 0 ? RCAR3_THERMAL_GRAN / 2 :
		-RCAR3_THERMAL_GRAN / 2;
	result = (temp + round_offs) / RCAR3_THERMAL_GRAN;
	return result * RCAR3_THERMAL_GRAN;
}

static int rcar_gen3_thermal_convert_temp(struct rcar_gen3_thermal_tsc *tsc)
{
	int mcelsius, val1, val2;
	u32 reg;

	/* Read register and convert to mili Celsius */
	reg = rcar_gen3_thermal_read(tsc, REG_GEN3_TEMP) & CTEMP_MASK;

	val1 = FIXPT_DIV(FIXPT_INT(reg) - tsc->coef.b1, tsc->coef.a1);
	val2 = FIXPT_DIV(FIXPT_INT(reg) - tsc->coef.b2, tsc->coef.a2);
	mcelsius = rcar_gen3_thermal_round(FIXPT_TO_MCELSIUS(
						(val1 + val2) / 2));

	return mcelsius;
}

static int rcar_gen3_thermal_get_temp(void *devdata, int *temp)
{
	struct rcar_gen3_thermal_tsc *tsc = devdata;
	int mcelsius, val1, val2;
	u32 reg;

	/* Read register and convert to mili Celsius */
	reg = rcar_gen3_thermal_read(tsc, REG_GEN3_TEMP) & CTEMP_MASK;

	val1 = FIXPT_DIV(FIXPT_INT(reg) - tsc->coef.b1, tsc->coef.a1);
	val2 = FIXPT_DIV(FIXPT_INT(reg) - tsc->coef.b2, tsc->coef.a2);
	mcelsius = FIXPT_TO_MCELSIUS((val1 + val2) / 2);

	/* Make sure we are inside specifications */
	if ((mcelsius < MCELSIUS(-40)) || (mcelsius > MCELSIUS(125)))
		return -EIO;

	/* Round value to device granularity setting */
	*temp = rcar_gen3_thermal_round(mcelsius);

	return 0;
}


static int rcar_gen3_thermal_mcelsius_to_temp(struct rcar_gen3_thermal_tsc *tsc,
					      int mcelsius)
{

	int val1, val2;

	val1 = (mcelsius * tsc->coef.a1)/1000 + tsc->coef.b1;
	val2 = (mcelsius * tsc->coef.a2)/1000 + tsc->coef.b2;

	return INT_FIXPT((val1 + val2) / 2);
}

static int rcar_gen3_thermal_update_temp(struct rcar_gen3_thermal_tsc *tsc)
{
	u32 ctemp;
	int temp_code;
	int mcelsius, val1, val2;

	ctemp = rcar_gen3_thermal_read(tsc, REG_GEN3_TEMP) & CTEMP_MASK;
	val1 = FIXPT_DIV(FIXPT_INT(ctemp) - tsc->coef.b1, tsc->coef.a1);
	val2 = FIXPT_DIV(FIXPT_INT(ctemp) - tsc->coef.b2, tsc->coef.a2);
	mcelsius = FIXPT_TO_MCELSIUS((val1 + val2) / 2);

	/* set the interrupts to exceed the temperature */
	temp_code = rcar_gen3_thermal_mcelsius_to_temp(tsc,
			mcelsius + MCELSIUS(1));
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQTEMP1, temp_code);
	/* set the interrupts to fall below the temperature */
	temp_code = rcar_gen3_thermal_mcelsius_to_temp(tsc,
			mcelsius - MCELSIUS(1));
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQTEMP2, temp_code);

	return 0;
}

static int rcar_gen3_thermal_set_trips(void *devdata, int low, int high)
{
	struct rcar_gen3_thermal_tsc *tsc = devdata;

	low = clamp_val(low, -40000, 120000);
	high = clamp_val(high, -40000, 120000);

	printk("file %s func %s line %d low %d high %d",__FILE__, __FUNCTION__, __LINE__, low, high);

	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQTEMP1,
				rcar_gen3_thermal_mcelsius_to_temp(tsc, low));

	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQTEMP2,
				rcar_gen3_thermal_mcelsius_to_temp(tsc, high));

	tsc->low = low;
	tsc->high = high;

	return 0;
}

static const struct thermal_zone_of_device_ops rcar_gen3_tz_of_ops = {
	.get_temp	= rcar_gen3_thermal_get_temp,
	//.set_trips	= rcar_gen3_thermal_set_trips,
};

static void rcar_thermal_irq_set(struct rcar_gen3_thermal_priv *priv, bool on)
{
	unsigned int i;
	u32 val = on ? IRQ_TEMPD1 | IRQ_TEMP2 : 0;

	for (i = 0; i < priv->num_tscs; i++)
		rcar_gen3_thermal_write(priv->tscs[i], REG_GEN3_IRQMSK, val);
}

static irqreturn_t rcar_gen3_thermal_irq(int irq, void *data)
{
	struct rcar_gen3_thermal_priv *priv = data;
	u32 status;
	int i, ret = IRQ_HANDLED;

	spin_lock(&priv->lock);
	for (i = 0; i < priv->num_tscs; i++) {
		status = rcar_gen3_thermal_read(priv->tscs[i], REG_GEN3_IRQSTR);
		rcar_gen3_thermal_write(priv->tscs[i], REG_GEN3_IRQSTR, 0);
		if (status)
			ret = IRQ_WAKE_THREAD;
	}

	if (ret == IRQ_WAKE_THREAD)
		rcar_thermal_irq_set(priv, false);

	spin_unlock(&priv->lock);

	return ret;
}

static irqreturn_t rcar_gen3_thermal_irq_thread(int irq, void *data)
{
	struct rcar_gen3_thermal_priv *priv = data;
	unsigned long flags;
	int i;

	for (i = 0; i < priv->num_tscs; i++)
	{
		rcar_gen3_thermal_update_temp(priv->tscs[i]);
		thermal_zone_device_update(priv->tscs[i]->zone,
					   THERMAL_EVENT_UNSPECIFIED);
	}
	spin_lock_irqsave(&priv->lock, flags);
	rcar_thermal_irq_set(priv, true);
	spin_unlock_irqrestore(&priv->lock, flags);

	return IRQ_HANDLED;
}

static const struct soc_device_attribute r8a7795es1[] = {
	{ .soc_id = "r8a7795", .revision = "ES1.*" },
	{ /* sentinel */ }
};

static void rcar_gen3_thermal_init_r8a7795es1(struct rcar_gen3_thermal_tsc *tsc)
{
	rcar_gen3_thermal_write(tsc, REG_GEN3_CTSR,  CTSR_THBGR);
	rcar_gen3_thermal_write(tsc, REG_GEN3_CTSR,  0x0);

	usleep_range(1000, 2000);

	rcar_gen3_thermal_write(tsc, REG_GEN3_CTSR, CTSR_PONM);

	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQCTL, 0x3f);
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQMSK, 0);
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQEN, IRQ_TEMPD1 | IRQ_TEMP2);

	rcar_gen3_thermal_write(tsc, REG_GEN3_CTSR,
				CTSR_PONM | CTSR_AOUT | CTSR_THBGR | CTSR_VMEN);

	usleep_range(100, 200);

	rcar_gen3_thermal_write(tsc, REG_GEN3_CTSR,
				CTSR_PONM | CTSR_AOUT | CTSR_THBGR | CTSR_VMEN |
				CTSR_VMST | CTSR_THSST);

	usleep_range(1000, 2000);
}

static void rcar_gen3_thermal_init(struct rcar_gen3_thermal_tsc *tsc)
{
	u32 reg_val;

	reg_val = rcar_gen3_thermal_read(tsc, REG_GEN3_THCTR);
	reg_val &= ~THCTR_PONM;
	rcar_gen3_thermal_write(tsc, REG_GEN3_THCTR, reg_val);

	usleep_range(1000, 2000);

	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQCTL, 0x3f);
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQMSK, 0);
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQEN, IRQ_TEMPD1 | IRQ_TEMP2);

	reg_val = rcar_gen3_thermal_read(tsc, REG_GEN3_THCTR);
	reg_val |= THCTR_THSST;
	rcar_gen3_thermal_write(tsc, REG_GEN3_THCTR, reg_val);

	usleep_range(1000, 2000);
}

static const struct of_device_id rcar_gen3_thermal_dt_ids[] = {
	{ .compatible = "renesas,r8a7795-thermal", },
	{ .compatible = "renesas,r8a7796-thermal", },
	{ .compatible = "renesas,r8a77965-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, rcar_gen3_thermal_dt_ids);

static int rcar_gen3_thermal_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return 0;
}

static int rcar_gen3_thermal_probe(struct platform_device *pdev)
{
	struct rcar_gen3_thermal_priv *priv;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct thermal_zone_device *zone;
	struct rcar_gen3_thermal_tsc* tsc;
	int ret, irq, i;
	char *irqname;

	tsc = devm_kzalloc(dev, sizeof(*tsc), GFP_KERNEL);
		if (!tsc)
			return -ENOMEM;


	platform_set_drvdata(pdev, tsc);

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	printk("file %s func %s line %d pdev->name %s",__FILE__, __FUNCTION__, __LINE__,  pdev->name);


	res = platform_get_resource(pdev, IORESOURCE_MEM,
							    0);
	tsc->base = devm_ioremap_resource(dev, res);

	if (IS_ERR(tsc->base))
		return PTR_ERR(tsc->base);

	printk("file %s func %s line %d tsc->base %x",__FILE__, __FUNCTION__, __LINE__,  tsc->base);


	//void __iomem* io = ioremap(0xe61a0000, 4);
	//pr_info("Hoan _test io %x\n", io);
	//pr_info("Hoan _test io %x\n", ioread32(io));
	//pr_info("Hoan _test tsc->base %x\n", ioread32(tsc->base));

	//void __iomem* io2 = ioremap(0xe61a8000, 4);
	//pr_info("Hoan _test io2 %x\n", io2);
	//pr_info("Hoan _test io2 %x\n", ioread32(io2));


	/*
	 * Start -> setting in nomal mode -> normal mode
	 *
	 */
	/*
	 * Setting in normal mode
	 */

	//Write B'0 to PONW in THCTR register
	u32 reg_thctr = rcar_gen3_thermal_read(tsc,REG_GEN3_THCTR);
	rcar_gen3_thermal_write(tsc, REG_GEN3_THCTR, reg_thctr & (~BIT(6)));
	printk("file %s func %s line %d reg_thctr %x",__FILE__, __FUNCTION__, __LINE__,  rcar_gen3_thermal_read(tsc,REG_GEN3_THCTR));

	//Wait 1 [ms]
	usleep_range(1000, 2000);
	//Set IRQCTL register
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQCTL, 0);
	//Set IRQMSK register
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQMSK, BIT(3));
	printk("file %s func %s line %d reg_IRQMSK %x",__FILE__, __FUNCTION__, __LINE__,  rcar_gen3_thermal_read(tsc,REG_GEN3_IRQMSK));

	//Set IRQTEMP# register
	//rcar_gen3_thermal_write(tsc, REG_GEN3_IRQTEMP1, 0);
	//rcar_gen3_thermal_write(tsc, REG_GEN3_IRQTEMP2, 0);
	//rcar_gen3_thermal_write(tsc, REG_GEN3_IRQTEMP3, 0);

	//Set IRQEN register
	rcar_gen3_thermal_write(tsc, REG_GEN3_IRQEN, BIT(3));

	//Write B'1 to THSST in THCTR register
	rcar_gen3_thermal_write(tsc, REG_GEN3_THCTR, rcar_gen3_thermal_read(tsc,REG_GEN3_THCTR) | BIT(0));

	//*******************************************//


	int Tj_T = (1509 - 435)*157/(2631 - 435) -41;
	printk("file %s func %s line %d Tj_T %d",__FILE__, __FUNCTION__, __LINE__,  Tj_T);

	int THCODE1, THCODE2, THCODE3, PLAT1, PLAT2, PLAT3;

	THCODE1 = 3397; THCODE2 = 2800; THCODE3 = 2221; PLAT1 = 2631; PLAT2 = 1509; PLAT3 = 435;
	u32 temp_code;
	int k;

	for (k= 0; k< 30; k++)
	{

	temp_code = rcar_gen3_thermal_read (tsc, REG_GEN3_TEMP);

	//printk("file %s func %s line %d temp_code %x temp_code_11 %d",__FILE__, __FUNCTION__, __LINE__,  temp_code, temp_code);

	int temp;
	if(temp_code < THCODE2)
	{
		temp = 157 * (PLAT3 - PLAT2)*  (THCODE3 - temp_code) / ((PLAT3 - PLAT1)* (THCODE3 - THCODE2)) - 41;
	}
	else
	{
		temp = 157 * (PLAT1 - PLAT2)*  (THCODE1 - temp_code) / ((PLAT1 - PLAT3)* (THCODE2 - THCODE1)) + 116;
	}

	printk("file %s func %s line %d temp_code %d temp %d",__FILE__, __FUNCTION__, __LINE__, temp_code, temp);
	usleep_range(1000000, 2000000);

	}

	return ret;
}

static int __maybe_unused rcar_gen3_thermal_suspend(struct device *dev)
{
	struct rcar_gen3_thermal_priv *priv = dev_get_drvdata(dev);

	rcar_thermal_irq_set(priv, false);

	return 0;
}

static int __maybe_unused rcar_gen3_thermal_resume(struct device *dev)
{
	struct rcar_gen3_thermal_priv *priv = dev_get_drvdata(dev);
	unsigned int i;

	for (i = 0; i < priv->num_tscs; i++) {
		struct rcar_gen3_thermal_tsc *tsc = priv->tscs[i];

		priv->thermal_init(tsc);
		rcar_gen3_thermal_update_temp(tsc);
	}

	rcar_thermal_irq_set(priv, true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(rcar_gen3_thermal_pm_ops, rcar_gen3_thermal_suspend,
			 rcar_gen3_thermal_resume);

static struct platform_driver rcar_gen3_thermal_driver = {
	.driver	= {
		.name	= "rcar_gen3_thermal",
		.pm = &rcar_gen3_thermal_pm_ops,
		.of_match_table = rcar_gen3_thermal_dt_ids,
	},
	.probe		= rcar_gen3_thermal_probe,
	.remove		= rcar_gen3_thermal_remove,
};
module_platform_driver(rcar_gen3_thermal_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("R-Car Gen3 THS thermal sensor driver");
MODULE_AUTHOR("Wolfram Sang <wsa+renesas@sang-engineering.com>");
