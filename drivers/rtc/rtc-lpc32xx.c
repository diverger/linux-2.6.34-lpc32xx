/*
 * drivers/rtc/rtc-lpc32xx.c
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

#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/slab.h>

/*
 * Clock and Power control register offsets
 */
#define RTC_UCOUNT(x)		((x) + 0x00)
#define RTC_DCOUNT(x)		((x) + 0x04)
#define RTC_MATCH0(x)		((x) + 0x08)
#define RTC_MATCH1(x)		((x) + 0x0C)
#define RTC_CTRL(x)		((x) + 0x10)
#define RTC_INTSTAT(x)		((x) + 0x14)
#define RTC_KEY(x)		((x) + 0x18)
#define RTC_SRAM(x, y)		((x) + 0x80 + ((y) * 4))

#define RTC_MATCH0_EN		(1 << 0)
#define RTC_MATCH1_EN		(1 << 1)
#define RTC_ONSW_MATCH0_EN	(1 << 2)
#define RTC_ONSW_MATCH1_EN	(1 << 3)
#define RTC_SW_RESET		(1 << 4)
#define RTC_CNTR_DIS		(1 << 6)
#define RTC_ONSW_FORCE_HIGH	(1 << 7)

#define RTC_MATCH0_INT_STS	(1 << 0)
#define RTC_MATCH1_INT_STS	(1 << 1)
#define RTC_ONSW_INT_STS	(1 << 2)

#define RTC_KEY_ONSW_LOADVAL	0xB5C13F27

static const char rtc_name[] = "rtc-lpc32xx";

struct lpc32xx_rtc_priv {
	void __iomem *rtc_base;
	int irq;
	int alarm_enabled;
	struct rtc_device *rtc;
	spinlock_t lock;
};

static inline void write_seconds(u32 iobase, unsigned long secs)
{
	u32 tmp;

	tmp = readl(RTC_CTRL(iobase));
	tmp |= RTC_CNTR_DIS;
	writel(tmp, RTC_CTRL(iobase));

	writel(secs, RTC_UCOUNT(iobase));
	writel((0xFFFFFFFF - secs), RTC_DCOUNT(iobase));

	tmp &= ~RTC_CNTR_DIS;
	writel(tmp, RTC_CTRL(iobase));
}

static int lpc32xx_rtc_read_time(struct device *dev, struct rtc_time *time)
{
	unsigned long elapsed_sec;
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = dev_get_drvdata(dev);

	elapsed_sec = readl(RTC_UCOUNT(lpc32xx_rtc_dat->rtc_base));

	rtc_time_to_tm(elapsed_sec, time);

	return 0;
}

static int lpc32xx_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = dev_get_drvdata(dev);

	spin_lock_irq(&lpc32xx_rtc_dat->lock);
	write_seconds((u32) lpc32xx_rtc_dat->rtc_base, secs);
	spin_unlock_irq(&lpc32xx_rtc_dat->lock);

	return 0;
}

static int lpc32xx_rtc_read_alarm(struct device *dev,
	struct rtc_wkalrm *wkalrm)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = dev_get_drvdata(dev);
	unsigned long alarmsecs;

	alarmsecs = readl(RTC_MATCH0(lpc32xx_rtc_dat->rtc_base));
	wkalrm->enabled = lpc32xx_rtc_dat->alarm_enabled;

	rtc_time_to_tm(alarmsecs, &wkalrm->time);

	return 0;
}

static int lpc32xx_rtc_set_alarm(struct device *dev,
	struct rtc_wkalrm *wkalrm)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = dev_get_drvdata(dev);
	unsigned long alarmsecs;
	int ret;

	ret = rtc_tm_to_time(&wkalrm->time, &alarmsecs);
	if (ret < 0) {
		dev_err(dev, "Failed to convert time: %d\n", ret);
		return ret;
	}

	spin_lock_irq(&lpc32xx_rtc_dat->lock);

	if (lpc32xx_rtc_dat->alarm_enabled)
		writel(~RTC_MATCH0_EN &
			readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)),
			RTC_CTRL(lpc32xx_rtc_dat->rtc_base));

	writel(alarmsecs, RTC_MATCH0(lpc32xx_rtc_dat->rtc_base));
	lpc32xx_rtc_dat->alarm_enabled = wkalrm->enabled;

	if (wkalrm->enabled) {
		writel(RTC_MATCH0_INT_STS,
			RTC_INTSTAT(lpc32xx_rtc_dat->rtc_base));

		writel(RTC_MATCH0_EN |
			readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)),
			RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	}

	spin_unlock_irq(&lpc32xx_rtc_dat->lock);

	return 0;
}

static int lpc32xx_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = dev_get_drvdata(dev);
	u32 tmp;

	tmp = readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	seq_printf(seq, "Alarm_IRQ\t: %s\n",
		   (tmp & RTC_MATCH0_EN) ? "yes" : "no");

	return 0;
}

static int lpc32xx_rtc_alarm_irq_enable(struct device *dev,
	unsigned int enabled)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = dev_get_drvdata(dev);

	if (enabled) {
		if (!lpc32xx_rtc_dat->alarm_enabled) {
			spin_lock_irq(&lpc32xx_rtc_dat->lock);
			lpc32xx_rtc_dat->alarm_enabled = 1;
			writel(RTC_MATCH0_EN |
				readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)),
				RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
			spin_unlock_irq(&lpc32xx_rtc_dat->lock);
		}
	} else if (lpc32xx_rtc_dat->alarm_enabled) {
		spin_lock_irq(&lpc32xx_rtc_dat->lock);
		lpc32xx_rtc_dat->alarm_enabled = 0;
		writel(~RTC_MATCH0_EN &
			readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)),
			RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
		spin_unlock_irq(&lpc32xx_rtc_dat->lock);
	}

	return 0;
}

static irqreturn_t lpc32xx_rtc_alarm_interrupt(int irq, void *dev)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat =
		(struct lpc32xx_rtc_priv *) dev;

	spin_lock(&lpc32xx_rtc_dat->lock);

	/* If the alarm isn't disabled, the match will keep occuring, so
		disable it now */
	writel(readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)) &
		~RTC_MATCH0_EN, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));

	/* Write a large value to the match value so the RTC won't
	   keep firing the match status */
	writel(0xFFFFFFFF, RTC_MATCH0(lpc32xx_rtc_dat->rtc_base));
	writel(RTC_MATCH0_INT_STS,
		RTC_INTSTAT(lpc32xx_rtc_dat->rtc_base));

	rtc_update_irq(lpc32xx_rtc_dat->rtc, 1, RTC_IRQF | RTC_AF);

	spin_unlock(&lpc32xx_rtc_dat->lock);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops lpc32xx_rtc_ops = {
	.read_time		= lpc32xx_rtc_read_time,
	.set_mmss		= lpc32xx_rtc_set_mmss,
	.read_alarm		= lpc32xx_rtc_read_alarm,
	.set_alarm		= lpc32xx_rtc_set_alarm,
	.proc			= lpc32xx_rtc_proc,
	.alarm_irq_enable	= lpc32xx_rtc_alarm_irq_enable,
};

static int __devinit lpc32xx_rtc_probe(struct platform_device *pdev)
{
	struct resource *res, *mem = NULL;
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat = NULL;
	int rtcirq, retval;
	u32 tmp;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Can't get memory resource\n");
		return -ENOENT;
	}

	rtcirq = platform_get_irq(pdev, 0);
	if ((rtcirq < 0) || (rtcirq >= NR_IRQS)) {
		dev_err(&pdev->dev, "Can't get interrupt resource\n");
		return -ENOENT;
	}

	lpc32xx_rtc_dat = kzalloc(sizeof(struct lpc32xx_rtc_priv),
		GFP_KERNEL);
	if (unlikely(!lpc32xx_rtc_dat)) {
		dev_err(&pdev->dev, "Can't allocate memory\n");
		return -ENOMEM;
	}
	lpc32xx_rtc_dat->irq = rtcirq;

	mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!mem) {
		dev_err(&pdev->dev, "RTC registers are not free\n");
		retval = -EBUSY;
		goto err_reqmem;
	}

	lpc32xx_rtc_dat->rtc_base = ioremap(res->start,
		res->end - res->start + 1);
	if (!lpc32xx_rtc_dat->rtc_base) {
		dev_err(&pdev->dev, "Can't map memory\n");
		retval = -EIO;
		goto err_noremap;
	}

	spin_lock_init(&lpc32xx_rtc_dat->lock);

	/* If RTC has never been previously setup, then set it up
	   now */
	if (readl(RTC_KEY(lpc32xx_rtc_dat->rtc_base)) !=
		RTC_KEY_ONSW_LOADVAL) {
		spin_lock_irq(&lpc32xx_rtc_dat->lock);
		tmp = readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
		tmp &= ~(RTC_SW_RESET | RTC_CNTR_DIS | RTC_MATCH0_EN |
			RTC_MATCH1_EN |	RTC_ONSW_MATCH0_EN |
			RTC_ONSW_MATCH1_EN | RTC_ONSW_FORCE_HIGH);
		writel(tmp, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));

		/* Clear latched interrupt states */
		writel(0xFFFFFFFF, RTC_MATCH0(lpc32xx_rtc_dat->rtc_base));
		writel(RTC_MATCH0_INT_STS | RTC_MATCH1_INT_STS |
			RTC_ONSW_INT_STS,
			RTC_INTSTAT(lpc32xx_rtc_dat->rtc_base));

		/* Write key value to RTC so it won't reload on reset */
		writel(RTC_KEY_ONSW_LOADVAL,
			RTC_KEY(lpc32xx_rtc_dat->rtc_base));

		spin_unlock_irq(&lpc32xx_rtc_dat->lock);
	} else if (readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)) & RTC_MATCH0_EN)
		lpc32xx_rtc_dat->alarm_enabled = 1;

	platform_set_drvdata(pdev, lpc32xx_rtc_dat);

	device_init_wakeup(&pdev->dev, 1);
	lpc32xx_rtc_dat->rtc = rtc_device_register(rtc_name, &pdev->dev,
		&lpc32xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(lpc32xx_rtc_dat->rtc)) {
		dev_err(&pdev->dev, "Can't get RTC\n");
		retval = PTR_ERR(lpc32xx_rtc_dat->rtc);
		goto err_noreg;
	}

	retval = request_irq(lpc32xx_rtc_dat->irq, lpc32xx_rtc_alarm_interrupt,
		0, "rtcalarm", lpc32xx_rtc_dat);
	if (retval < 0) {
		dev_err(&pdev->dev, "Can't request interrupt\n");
		goto err_free_irq;
	}

	return 0;

err_free_irq:
	rtc_device_unregister(lpc32xx_rtc_dat->rtc);
err_noreg:
	iounmap(lpc32xx_rtc_dat->rtc_base);
err_noremap:
	release_resource(mem);
err_reqmem:
	kfree(lpc32xx_rtc_dat);

	return retval;
}

static int __devexit lpc32xx_rtc_remove(struct platform_device *pdev)
{
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat =
		platform_get_drvdata(pdev);

	free_irq(lpc32xx_rtc_dat->irq, pdev);
	rtc_device_unregister(lpc32xx_rtc_dat->rtc);
	iounmap(lpc32xx_rtc_dat->rtc_base);
	release_resource(dev_get_drvdata(&lpc32xx_rtc_dat->rtc->dev));
	kfree(lpc32xx_rtc_dat);

	return 0;
}

#ifdef CONFIG_PM
/* Turn off the alarm if it should not be a wake source. */
static int lpc32xx_rtc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat =
		platform_get_drvdata(pdev);
	u32 tmp;

	spin_lock_irq(&lpc32xx_rtc_dat->lock);
	tmp = readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base));

	if (device_may_wakeup(&pdev->dev))
		tmp |= RTC_MATCH0_EN;
	else
		tmp &= ~RTC_MATCH0_EN;

	writel(tmp, RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
	spin_unlock_irq(&lpc32xx_rtc_dat->lock);

	return 0;
}

/* Enable the alarm if it should be enabled (in case it was disabled to
 * prevent use as a wake source).
 */
static int lpc32xx_rtc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat =
		platform_get_drvdata(pdev);

	if (lpc32xx_rtc_dat->alarm_enabled) {
		spin_lock_irq(&lpc32xx_rtc_dat->lock);

		writel(RTC_MATCH0_EN |
			readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)),
			RTC_CTRL(lpc32xx_rtc_dat->rtc_base));
		spin_unlock_irq(&lpc32xx_rtc_dat->lock);
	}

	return 0;
}

/* Unconditionally disable the alarm */
static int lpc32xx_rtc_freeze(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc32xx_rtc_priv *lpc32xx_rtc_dat =
		platform_get_drvdata(pdev);

	spin_lock_irq(&lpc32xx_rtc_dat->lock);

	writel(~RTC_MATCH0_EN &
		readl(RTC_CTRL(lpc32xx_rtc_dat->rtc_base)),
		RTC_CTRL(lpc32xx_rtc_dat->rtc_base));

	return 0;
}
#else
#define lpc32xx_rtc_suspend NULL
#define lpc32xx_rtc_resume NULL
#define lpc32xx_rtc_freeze NULL
#endif

static const struct dev_pm_ops lpc32xx_rtc_pm_ops = {
	.suspend = lpc32xx_rtc_suspend,
	.resume = lpc32xx_rtc_resume,
	.freeze = lpc32xx_rtc_freeze,
	.thaw = lpc32xx_rtc_resume,
	.restore = lpc32xx_rtc_resume
};

static struct platform_driver lpc32xx_rtc_driver = {
	.probe		= lpc32xx_rtc_probe,
	.remove		= __devexit_p(lpc32xx_rtc_remove),
	.driver = {
		.name = rtc_name,
		.pm = &lpc32xx_rtc_pm_ops,
	},
};

static int __init lpc32xx_rtc_init(void)
{
	return platform_driver_register(&lpc32xx_rtc_driver);
}

static void __exit lpc32xx_rtc_exit(void)
{
	platform_driver_unregister(&lpc32xx_rtc_driver);
}

module_init(lpc32xx_rtc_init);
module_exit(lpc32xx_rtc_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC32XX RTC Driver");
MODULE_LICENSE("GPL");
