/*
 * drivers/input/touchscreen/lpc32xx_tsc.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2010 NXP Semiconductors
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
 */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>

/*
 * Touchscreen controller register offsets
 */
#define LPC32XX_TSC_STAT(x)			((x) + 0x00)
#define LPC32XX_TSC_SEL(x)			((x) + 0x04)
#define LPC32XX_TSC_CON(x)			((x) + 0x08)
#define LPC32XX_TSC_FIFO(x)			((x) + 0x0C)
#define LPC32XX_TSC_DTR(x)			((x) + 0x10)
#define LPC32XX_TSC_RTR(x)			((x) + 0x14)
#define LPC32XX_TSC_UTR(x)			((x) + 0x18)
#define LPC32XX_TSC_TTR(x)			((x) + 0x1C)
#define LPC32XX_TSC_DXP(x)			((x) + 0x20)
#define LPC32XX_TSC_MIN_X(x)			((x) + 0x24)
#define LPC32XX_TSC_MAX_X(x)			((x) + 0x28)
#define LPC32XX_TSC_MIN_Y(x)			((x) + 0x2C)
#define LPC32XX_TSC_MAX_Y(x)			((x) + 0x30)
#define LPC32XX_TSC_AUX_UTR(x)			((x) + 0x34)
#define LPC32XX_TSC_AUX_MIN(x)			((x) + 0x38)
#define LPC32XX_TSC_AUX_MAX(x)			((x) + 0x3C)

#define LPC32XX_TSC_STAT_FIFO_OVRRN		(1 << 8)
#define LPC32XX_TSC_STAT_FIFO_EMPTY		(1 << 7)

#define LPC32XX_TSC_ADCCON_IRQ_TO_FIFO_4	(0x1 << 11)
#define LPC32XX_TSC_ADCCON_X_SAMPLE_SIZE(s)	((10 - s) << 7)
#define LPC32XX_TSC_ADCCON_Y_SAMPLE_SIZE(s)	((10 - s) << 4)
#define LPC32XX_TSC_ADCCON_POWER_UP		(1 << 2)
#define LPC32XX_TSC_ADCCON_AUTO_EN		(1 << 0)

#define LPC32XX_TSC_FIFO_TS_P_LEVEL		(1 << 31)
#define LPC32XX_TSC_FIFO_NORMALIZE_X_VAL(x)	(((x) & 0x03FF0000) >> 16)
#define LPC32XX_TSC_FIFO_NORMALIZE_Y_VAL(y)	((y) & 0x000003FF)

#define LPC32XX_TSC_ADCDAT_VALUE_MASK		0x000003FF

#define MOD_NAME "lpc32xx-ts"

struct lpc32xx_tsc_t {
	struct input_dev *dev;
	void __iomem *tsc_base;
	int irq;
	struct clk *clk;
	int suspended;
};

static void lpc32xx_fifo_clear(struct lpc32xx_tsc_t *lpc32xx_tsc_dat)
{
	while (!(__raw_readl(LPC32XX_TSC_STAT(lpc32xx_tsc_dat->tsc_base)) &
		LPC32XX_TSC_STAT_FIFO_EMPTY))
		__raw_readl(LPC32XX_TSC_FIFO(lpc32xx_tsc_dat->tsc_base));
}

static irqreturn_t lpc32xx_ts_interrupt(int irq, void *dev_id)
{
	u32 tmp, rv[4], xs[4], ys[4];
	int idx;
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat =
		(struct lpc32xx_tsc_t *) dev_id;
	struct input_dev *input = lpc32xx_tsc_dat->dev;

	tmp = __raw_readl(LPC32XX_TSC_STAT(lpc32xx_tsc_dat->tsc_base));

	if (tmp & LPC32XX_TSC_STAT_FIFO_OVRRN) {
		/* FIFO overflow - throw away samples */
		lpc32xx_fifo_clear(lpc32xx_tsc_dat);
		return IRQ_HANDLED;
	}

	idx = 0;
	while ((idx < 4) &&
		(!(__raw_readl(LPC32XX_TSC_STAT(lpc32xx_tsc_dat->tsc_base)) &
		LPC32XX_TSC_STAT_FIFO_EMPTY))) {
		tmp = __raw_readl(LPC32XX_TSC_FIFO(lpc32xx_tsc_dat->tsc_base));
		xs[idx] = LPC32XX_TSC_ADCDAT_VALUE_MASK -
			LPC32XX_TSC_FIFO_NORMALIZE_X_VAL(tmp);
		ys[idx] = LPC32XX_TSC_ADCDAT_VALUE_MASK -
			LPC32XX_TSC_FIFO_NORMALIZE_Y_VAL(tmp);
		rv[idx] = tmp;
		idx++;
	}

	/* Data is only valid if pen is still down */
	if ((!(rv[3] & LPC32XX_TSC_FIFO_TS_P_LEVEL)) && (idx == 4)) {
		input_report_abs(input, ABS_X, ((xs[1] + xs[2]) / 2));
		input_report_abs(input, ABS_Y, ((ys[1] + ys[2]) / 2));
		input_report_abs(input, ABS_PRESSURE, 1);
		input_report_key(input, BTN_TOUCH, 1);
	} else {
		input_report_abs(input, ABS_PRESSURE, 0);
		input_report_key(input, BTN_TOUCH, 0);
	}

	input_sync(input);

	return IRQ_HANDLED;
}

static void stop_tsc(struct lpc32xx_tsc_t *lpc32xx_tsc_dat)
{
	u32 tmp;

	/* Disable auto mode */
	tmp = __raw_readl(LPC32XX_TSC_CON(lpc32xx_tsc_dat->tsc_base));
	tmp &= ~LPC32XX_TSC_ADCCON_AUTO_EN;
	__raw_writel(tmp, LPC32XX_TSC_CON(lpc32xx_tsc_dat->tsc_base));
}

static void setup_tsc(struct lpc32xx_tsc_t *lpc32xx_tsc_dat)
{
	u32 tmp;

	tmp = __raw_readl(LPC32XX_TSC_CON(lpc32xx_tsc_dat->tsc_base));
	tmp &= ~LPC32XX_TSC_ADCCON_POWER_UP;
	__raw_writel(tmp, LPC32XX_TSC_CON(lpc32xx_tsc_dat->tsc_base));

	/* Set the TSC FIFO depth to 4 samples @ 10-bits sample */
	tmp = (LPC32XX_TSC_ADCCON_IRQ_TO_FIFO_4 |
		LPC32XX_TSC_ADCCON_X_SAMPLE_SIZE(10) |
		LPC32XX_TSC_ADCCON_Y_SAMPLE_SIZE(10));
	__raw_writel(tmp, LPC32XX_TSC_CON(lpc32xx_tsc_dat->tsc_base));

	__raw_writel(0x0284, LPC32XX_TSC_SEL(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, LPC32XX_TSC_MIN_X(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x03FF, LPC32XX_TSC_MAX_X(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, LPC32XX_TSC_MIN_Y(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x03FF, LPC32XX_TSC_MAX_Y(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, LPC32XX_TSC_AUX_UTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, LPC32XX_TSC_AUX_MIN(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x0000, LPC32XX_TSC_AUX_MAX(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x2, LPC32XX_TSC_RTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(446, LPC32XX_TSC_UTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x2, LPC32XX_TSC_DTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x10, LPC32XX_TSC_TTR(lpc32xx_tsc_dat->tsc_base));
	__raw_writel(0x4, LPC32XX_TSC_DXP(lpc32xx_tsc_dat->tsc_base));

	/*
	 * Set sample rate to about 60Hz, this rate is based on the
	 * RTC clock, which should be a stable 32768Hz
	 */
	__raw_writel(88, LPC32XX_TSC_UTR(lpc32xx_tsc_dat->tsc_base));

	lpc32xx_fifo_clear(lpc32xx_tsc_dat);

	tmp = __raw_readl(LPC32XX_TSC_CON(lpc32xx_tsc_dat->tsc_base));
	tmp |= LPC32XX_TSC_ADCCON_AUTO_EN;
	__raw_writel(tmp, LPC32XX_TSC_CON(lpc32xx_tsc_dat->tsc_base));
}

static int __devinit lpc32xx_ts_probe(struct platform_device *pdev)
{
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat = NULL;
	struct resource *res;
	int retval = -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform I/O memory\n");
		retval = -EBUSY;
		goto errout;
	}

	lpc32xx_tsc_dat = kzalloc(sizeof(struct lpc32xx_tsc_t), GFP_KERNEL);
	if (unlikely(!lpc32xx_tsc_dat)) {
		dev_err(&pdev->dev, "failed allocating memory\n");
		retval = -ENOMEM;
		goto errout;
	}

	lpc32xx_tsc_dat->tsc_base = ioremap(res->start,
		res->end - res->start + 1);
	if (!lpc32xx_tsc_dat->tsc_base) {
		dev_err(&pdev->dev, "failed mapping memory\n");
		retval = -EBUSY;
		goto errout;
	}

	lpc32xx_tsc_dat->dev = input_allocate_device();
	if (!lpc32xx_tsc_dat->dev) {
		dev_err(&pdev->dev, "failed allocating input device\n");
		retval = -ENOMEM;
		goto errout;
	}

	lpc32xx_tsc_dat->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(lpc32xx_tsc_dat->clk)) {
		dev_err(&pdev->dev, "failed getting clock\n");
		goto errout;
	}
	clk_enable(lpc32xx_tsc_dat->clk);

	setup_tsc(lpc32xx_tsc_dat);

	lpc32xx_tsc_dat->irq = platform_get_irq(pdev, 0);
	if ((lpc32xx_tsc_dat->irq < 0) || (lpc32xx_tsc_dat->irq >= NR_IRQS)) {
		dev_err(&pdev->dev, "failed getting interrupt resource\n");
		retval = -EINVAL;
		goto errout;
	}

	retval = request_irq(lpc32xx_tsc_dat->irq, lpc32xx_ts_interrupt,
		0, MOD_NAME, lpc32xx_tsc_dat);
	if (retval < 0) {
		dev_err(&pdev->dev, "failed requesting interrupt\n");
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, lpc32xx_tsc_dat);

	lpc32xx_tsc_dat->dev->name = MOD_NAME;
	lpc32xx_tsc_dat->dev->phys = "lpc32xx/input0";
	lpc32xx_tsc_dat->dev->id.bustype = BUS_HOST;
	lpc32xx_tsc_dat->dev->id.vendor = 0x0001;
	lpc32xx_tsc_dat->dev->id.product = 0x0002;
	lpc32xx_tsc_dat->dev->id.version = 0x0100;
	lpc32xx_tsc_dat->dev->dev.parent = &pdev->dev;

	lpc32xx_tsc_dat->dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	lpc32xx_tsc_dat->dev->keybit[BIT_WORD(BTN_TOUCH)] =
		BIT_MASK(BTN_TOUCH);
	input_set_abs_params(lpc32xx_tsc_dat->dev, ABS_X, 0, 1023, 0, 0);
	input_set_abs_params(lpc32xx_tsc_dat->dev, ABS_Y, 0, 1023, 0, 0);
	input_set_abs_params(lpc32xx_tsc_dat->dev, ABS_PRESSURE, 0, 1, 0, 0);

	retval = input_register_device(lpc32xx_tsc_dat->dev);
	if (retval) {
		dev_err(&pdev->dev, "failed registering input device\n");
		goto err_free_irq;
	}

	device_init_wakeup(&pdev->dev, 1);

	return 0;

err_free_irq:
	stop_tsc(lpc32xx_tsc_dat);
	platform_set_drvdata(pdev, NULL);
	free_irq(lpc32xx_tsc_dat->irq, lpc32xx_tsc_dat->dev);

errout:
	if (lpc32xx_tsc_dat) {
		if (lpc32xx_tsc_dat->clk) {
			clk_disable(lpc32xx_tsc_dat->clk);
			clk_put(lpc32xx_tsc_dat->clk);
		}

		if (lpc32xx_tsc_dat->dev)
			input_free_device(lpc32xx_tsc_dat->dev);

		if (lpc32xx_tsc_dat->tsc_base)
			iounmap(lpc32xx_tsc_dat->tsc_base);

		kfree(lpc32xx_tsc_dat);
	}

	return retval;
}

static int __devexit lpc32xx_ts_remove(struct platform_device *pdev)
{
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat = platform_get_drvdata(pdev);

	stop_tsc(lpc32xx_tsc_dat);
	free_irq(lpc32xx_tsc_dat->irq, lpc32xx_tsc_dat->dev);
	platform_set_drvdata(pdev, NULL);
	input_unregister_device(lpc32xx_tsc_dat->dev);

	if (lpc32xx_tsc_dat->clk) {
		clk_disable(lpc32xx_tsc_dat->clk);
		clk_put(lpc32xx_tsc_dat->clk);
	}

	if (lpc32xx_tsc_dat->tsc_base)
		iounmap(lpc32xx_tsc_dat->tsc_base);

	kfree(lpc32xx_tsc_dat);

	return 0;
}

#if defined (CONFIG_PM)
static int lpc32xx_ts_suspend(struct device *dev)
{
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(lpc32xx_tsc_dat->irq);
	else {
		lpc32xx_tsc_dat->suspended = 1;
		stop_tsc(lpc32xx_tsc_dat);
		clk_disable(lpc32xx_tsc_dat->clk);
	}

	return 0;
}

static int lpc32xx_ts_resume(struct device *dev)
{
	struct lpc32xx_tsc_t *lpc32xx_tsc_dat = dev_get_drvdata(dev);

	if (lpc32xx_tsc_dat->suspended) {
		clk_enable(lpc32xx_tsc_dat->clk);
		setup_tsc(lpc32xx_tsc_dat);
		lpc32xx_tsc_dat->suspended = 0;
	} else
		disable_irq_wake(lpc32xx_tsc_dat->irq);

	return 0;
}

static const struct dev_pm_ops lpc32xx_ts_pm_ops = {
	.suspend	= lpc32xx_ts_suspend,
	.resume		= lpc32xx_ts_resume,
};
#define LPC32XX_TS_PM_OPS (&lpc32xx_ts_pm_ops)
#else
#define LPC32XX_TS_PM_OPS NULL
#endif

static struct platform_driver lpc32xx_ts_driver = {
	.probe		= lpc32xx_ts_probe,
	.remove		= __devexit_p(lpc32xx_ts_remove),
	.driver		= {
		.name	= MOD_NAME,
		.owner	= THIS_MODULE,
		.pm	= LPC32XX_TS_PM_OPS,
	},
};

static int __init lpc32xx_ts_init(void)
{
	return platform_driver_register(&lpc32xx_ts_driver);
}

static void __exit lpc32xx_ts_exit(void)
{
	platform_driver_unregister(&lpc32xx_ts_driver);
}

module_init(lpc32xx_ts_init);
module_exit(lpc32xx_ts_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC32XX TSC Driver");
MODULE_LICENSE("GPL");
