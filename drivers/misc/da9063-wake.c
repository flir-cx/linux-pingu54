/*
 * Wake device driver for DA9063
 * Copyright (C) 2016 FLIR Systems
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>

struct da9063_wake {
	struct da9063 *hw;
	struct device *dev;
	int irq;
};


static irqreturn_t da9063_wake_irq_handler(int irq, void *data)
{
	struct da9063_wake *wake = data;
	pm_wakeup_event(wake->dev,0);

	return IRQ_HANDLED;
}


static int da9063_wake_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_wake *wake;
	int error;
	int irq;

	wake = devm_kzalloc(&pdev->dev, sizeof(struct da9063_wake),
			     GFP_KERNEL);
	if (!wake) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	wake->dev = &pdev->dev;
	wake->hw = da9063;

	irq = platform_get_irq_byname(pdev, "WAKE");
	if (irq < 0) {
		error = irq;
		dev_err(&pdev->dev, "Failed to get platform IRQ: %d\n", error);
		return error;
	}
	wake->irq =irq;

	error = devm_request_threaded_irq(&pdev->dev, irq,
					  NULL, da9063_wake_irq_handler,
					  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  "WAKE", wake);
	if (error) {
		dev_err(&pdev->dev,
			"Failed to request IRQ %d: %d\n", irq, error);
		return error;
	}

	device_init_wakeup(&pdev->dev, 1);

	platform_set_drvdata(pdev, wake);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int da9063_wake_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9063_wake *wake = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(wake->irq);

	return 0;
}

static int da9063_wake_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct da9063_wake *wake = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(wake->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(da9063_wake_pm_ops, da9063_wake_suspend, da9063_wake_resume);

static struct platform_driver da9063_wake_driver = {
	.probe	= da9063_wake_probe,
	.driver	= {
		.name	= DA9063_DRVNAME_WAKE,
		.pm	= &da9063_wake_pm_ops,
	},
};
module_platform_driver(da9063_wake_driver);

MODULE_AUTHOR("Felix Hammarstrand");
MODULE_DESCRIPTION("charge wake device driver for Dialog DA9063");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DA9063_DRVNAME_WAKE);
