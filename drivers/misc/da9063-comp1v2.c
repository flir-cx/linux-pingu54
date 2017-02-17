/*
 * Comparator 1V2 device driver for DA9063
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
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>

struct da9063_comp1v2 {
	struct da9063 *hw;
	struct device *dev;
	int irq;
};


static irqreturn_t da9063_comp1v2_irq_handler(int irq, void *data)
{
	struct da9063_comp1v2 *comp = data;
	int val, error;

	error = regmap_read(comp->hw->regmap, DA9063_REG_STATUS_A, &val);

	if(!error && !(val & DA9063_COMP_1V2) )
	{
		dev_err(comp->dev, "Battery removed, powering off!\n");
		regmap_update_bits(comp->hw->regmap, DA9063_REG_CONTROL_A,
					 DA9063_SYSTEM_EN, 0);
		mdelay(2000);
	}
	return IRQ_HANDLED;
}


static int da9063_comp1v2_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_comp1v2 *comp;
	int error;
	int irq;

	comp = devm_kzalloc(&pdev->dev, sizeof(struct da9063_comp1v2),
			     GFP_KERNEL);
	if (!comp) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	comp->dev = &pdev->dev;
	comp->hw = da9063;

	irq = platform_get_irq_byname(pdev, "COMP_1V2");
	if (irq < 0) {
		error = irq;
		dev_err(&pdev->dev, "Failed to get platform IRQ: %d\n", error);
		return error;
	}
	comp->irq =irq;

	error = devm_request_threaded_irq(&pdev->dev, irq,
					  NULL, da9063_comp1v2_irq_handler,
					  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  "COMP_1V2", comp);
	if (error) {
		dev_err(&pdev->dev,
			"Failed to request IRQ %d: %d\n", irq, error);
		return error;
	}
	error = regmap_update_bits(comp->hw->regmap, DA9063_REG_ADC_CONT,
				 DA9063_COMP1V2_EN, DA9063_COMP1V2_EN);

	platform_set_drvdata(pdev, comp);
	return 0;
}



static void da9063_comp1v2_shutdown(struct platform_device *pdev)
{
	struct da9063_comp1v2 *comp = platform_get_drvdata(pdev);

	regmap_update_bits(comp->hw->regmap, DA9063_REG_ADC_CONT,
				 DA9063_COMP1V2_EN, 0);
}


static struct platform_driver da9063_comp1v2_driver = {
	.probe	= da9063_comp1v2_probe,
	.driver	= {
		.name	= DA9063_DRVNAME_COMP1V2,
	},
	.shutdown = da9063_comp1v2_shutdown,
};
module_platform_driver(da9063_comp1v2_driver);

MODULE_AUTHOR("Felix Hammarstrand");
MODULE_DESCRIPTION("Comparator driver for Dialog DA9063");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DA9063_DRVNAME_COMP1V2);
