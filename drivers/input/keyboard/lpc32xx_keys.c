/*
 * linux/drivers/input/keyboard/lpc32xx-keys.c
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/board.h>

/*
 * Key scanner register offsets
 */
#define LPC32XX_KS_DEB(x)			((x) + 0x00)
#define LPC32XX_KS_STATE_COND(x)		((x) + 0x04)
#define LPC32XX_KS_IRQ(x)			((x) + 0x08)
#define LPC32XX_KS_SCAN_CTL(x)			((x) + 0x0C)
#define LPC32XX_KS_FAST_TST(x)			((x) + 0x10)
#define LPC32XX_KS_MATRIX_DIM(x)		((x) + 0x14)
#define LPC32XX_KS_DATA(x, y)			((x) + 0x40 + (y  << 2))

#define LPC32XX_KSCAN_DEB_NUM_DEB_PASS(n)	((n) & 0xFF)

#define LPC32XX_KSCAN_SCOND_IN_IDLE		0x0
#define LPC32XX_KSCAN_SCOND_IN_SCANONCE		0x1
#define LPC32XX_KSCAN_SCOND_IN_IRQGEN		0x2
#define LPC32XX_KSCAN_SCOND_IN_SCAN_MATRIX	0x3

#define LPC32XX_KSCAN_IRQ_PENDING_CLR		0x1

#define LPC32XX_KSCAN_SCTRL_SCAN_DELAY(n)	((n) & 0xFF)

#define LPC32XX_KSCAN_FTST_FORCESCANONCE	0x1
#define LPC32XX_KSCAN_FTST_USE32K_CLK		0x2

#define LPC32XX_KSCAN_MSEL_SELECT(n)		((n) & 0xF)

struct lpc32xx_kscan_drv {
	struct input_dev *input;
	struct lpc32XX_kscan_cfg *kscancfg;
	struct clk *clk;
	void __iomem *kscan_base;
	int irq;
	u8 lastkeystates[8];
};

static void lpc32xx_mod_states(struct lpc32xx_kscan_drv *kscandat, int off)
{
	u8 st, key;
	int j, scancode, keycode;

	key = (u8) __raw_readl(LPC32XX_KS_DATA(kscandat->kscan_base, off));
	if (key != kscandat->lastkeystates[off]) {
		for (j = 0; j < kscandat->kscancfg->matrix_sz; j++) {
			st = key & (1 << j);
			if (st != (kscandat->lastkeystates[off] & (1 << j))) {
				/* Key state changed, signal an event */
				scancode = (int) (j *
					kscandat->kscancfg->matrix_sz) + off;
				keycode =
					kscandat->kscancfg->keymap[scancode];
				input_report_key(kscandat->input, keycode,
					(st != 0));
			}
		}

		kscandat->lastkeystates[off] = key;
	}
}

static irqreturn_t lpc32xx_kscan_irq(int irq, void *dev_id)
{
	int i;
	struct lpc32xx_kscan_drv *kscandat =
		(struct lpc32xx_kscan_drv *) dev_id;

	for (i = 0; i < kscandat->kscancfg->matrix_sz; i++)
		lpc32xx_mod_states(kscandat, i);

	__raw_writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));

	input_sync(kscandat->input);

	return IRQ_HANDLED;
}

static int __devinit lpc32xx_kscan_probe(struct platform_device *pdev)
{
	struct lpc32xx_kscan_drv *kscandat;
	struct resource *res;
	int retval, i;

	kscandat = kzalloc(sizeof(struct lpc32xx_kscan_drv), GFP_KERNEL);
	if (unlikely(!kscandat)) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get platform I/O memory\n");
		retval = -EBUSY;
		goto err_nores;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		retval = -EBUSY;
		goto err_nores;
	}

	kscandat->kscan_base = ioremap(res->start, res->end - res->start + 1);
	if (kscandat->kscan_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		retval = -EBUSY;
		goto err_noremap;
	}

	/* Get the key scanner clock */
	kscandat->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(kscandat->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		retval = -ENODEV;
		goto err_noclk;
	}
	clk_enable(kscandat->clk);

	kscandat->irq = platform_get_irq(pdev, 0);
	if ((kscandat->irq < 0) || (kscandat->irq >= NR_IRQS)) {
		dev_err(&pdev->dev, "failed to get platform irq\n");
		retval = -EINVAL;
		goto err_noirq;
	}
	retval = request_irq(kscandat->irq, lpc32xx_kscan_irq,
		0, pdev->name, kscandat);
	if (retval) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto err_noirq;
	}

	kscandat->input = input_allocate_device();
	if (kscandat->input == NULL) {
		dev_err(&pdev->dev, "failed to allocate device\n");
		retval = -ENOMEM;
		goto err_noalloc;
	}

	kscandat->kscancfg = (struct lpc32XX_kscan_cfg *)
		pdev->dev.platform_data;
	platform_set_drvdata(pdev, kscandat);

	/* Setup key input */
	kscandat->input->evbit[0]	= BIT_MASK(EV_KEY);
	kscandat->input->name		= pdev->name;
	kscandat->input->phys		= "matrix-keys/input0";
	kscandat->input->dev.parent	=  &pdev->dev;
	kscandat->input->id.vendor	= 0x0001;
	kscandat->input->id.product	= 0x0001;
	kscandat->input->id.version	= 0x0100;
	for (i = 0; i < kscandat->kscancfg->matrix_sz; i++)
		__set_bit(kscandat->kscancfg->keymap[i],
			kscandat->input->keybit);

	input_set_capability(kscandat->input, EV_MSC, MSC_SCAN);

	retval = input_register_device(kscandat->input);
	if (retval) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err_notregistered;
	}

	/* Configure the key scanner */
	__raw_writel(kscandat->kscancfg->deb_clks,
		LPC32XX_KS_DEB(kscandat->kscan_base));
	__raw_writel(kscandat->kscancfg->scan_delay,
		LPC32XX_KS_SCAN_CTL(kscandat->kscan_base));
	__raw_writel(LPC32XX_KSCAN_FTST_USE32K_CLK,
		LPC32XX_KS_FAST_TST(kscandat->kscan_base));
	__raw_writel(kscandat->kscancfg->matrix_sz,
		LPC32XX_KS_MATRIX_DIM(kscandat->kscan_base));
	__raw_writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));

	return 0;

err_notregistered:
	input_free_device(kscandat->input);
err_noalloc:
	free_irq(kscandat->irq, pdev);
err_noirq:
	clk_put(kscandat->clk);
err_noclk:
	iounmap(kscandat->kscan_base);
err_noremap:
	release_mem_region(res->start, resource_size(res));
err_nores:
	kfree(kscandat);

	return retval;
}

static int __devexit lpc32xx_kscan_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	free_irq(kscandat->irq, pdev);
	input_unregister_device(kscandat->input);
	clk_put(kscandat->clk);
	iounmap(kscandat->kscan_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(kscandat);

	return 0;
}

#ifdef CONFIG_PM
static int lpc32xx_kscan_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	/* Clear IRQ and disable clock */
	__raw_writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));
	clk_disable(kscandat->clk);

	return 0;
}

static int lpc32xx_kscan_resume(struct platform_device *pdev)
{
	struct lpc32xx_kscan_drv *kscandat = platform_get_drvdata(pdev);

	/* Enable clock and clear IRQ */
	clk_enable(kscandat->clk);
	__raw_writel(1, LPC32XX_KS_IRQ(kscandat->kscan_base));

	return 0;
}
#else
#define lpc32xx_kscan_suspend	NULL
#define lpc32xx_kscan_resume	NULL
#endif

static struct platform_driver lpc32xx_kscan_driver = {
	.probe		= lpc32xx_kscan_probe,
	.remove		= __devexit_p(lpc32xx_kscan_remove),
	.suspend	= lpc32xx_kscan_suspend,
	.resume		= lpc32xx_kscan_resume,
	.driver		= {
		.name	= "lpc32xx_keys",
	}
};

static int __init lpc32xx_kscan_init(void)
{
	return platform_driver_register(&lpc32xx_kscan_driver);
}

static void __exit lpc32xx_kscan_exit(void)
{
	platform_driver_unregister(&lpc32xx_kscan_driver);
}

module_init(lpc32xx_kscan_init);
module_exit(lpc32xx_kscan_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("Key scanner driver for LPC32XX devices");
