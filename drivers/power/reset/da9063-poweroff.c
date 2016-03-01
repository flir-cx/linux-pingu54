/* poweroff-da9063.c - Poweroff driver for DA9063
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/mfd/da9063/registers.h>
#include <linux/mfd/da9063/core.h>

/*
 * Hold configuration here, cannot be more than one instance of the driver
 * since pm_power_off itself is global.
 */
struct da9063 *da9063;

static void da9063_poweroff_do_poweroff(void)
{
	regmap_update_bits(da9063->regmap, DA9063_REG_CONTROL_A,
				 DA9063_SYSTEM_EN, 0);

	mdelay(2000);
	WARN_ON(1);
}

static int da9063_poweroff_probe(struct platform_device *pdev)
{
	da9063 = dev_get_drvdata(pdev->dev.parent);
	/* If a pm_power_off function has already been added, leave it alone */
	if (pm_power_off != NULL) {
		pr_err("%s: pm_power_off function already registered",
		       __func__);
		return -EBUSY;
	}
 	pm_power_off = &da9063_poweroff_do_poweroff; 
	return 0;
}

static int da9063_poweroff_remove(struct platform_device *pdev)
{
	if (pm_power_off == &da9063_poweroff_do_poweroff)
		pm_power_off = NULL;

	return 0;
}

static struct platform_driver da9063_poweroff_driver = {
	.probe		= da9063_poweroff_probe,
	.remove         = da9063_poweroff_remove,
	.driver		= {
		.name	= DA9063_DRVNAME_POWEROFF,
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(da9063_poweroff_driver);

MODULE_AUTHOR("Bo Svangård <bobo@larven.se>");
MODULE_DESCRIPTION("Poweroff driver for Dialog DA9063");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_POWEROFF);
