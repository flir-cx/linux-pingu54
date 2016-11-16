/*
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
#include <asm/system_misc.h>


static struct da9063 *da9063;

static void da9063_reset_do_reset(enum reboot_mode reboot_mode, const char *cmd)
{
	regmap_update_bits(da9063->regmap, DA9063_REG_CONTROL_F,
				 DA9063_SHUTDOWN, DA9063_SHUTDOWN);
	mdelay(1000);
}

static int da9063_reset_probe(struct platform_device *pdev)
{
	da9063 = dev_get_drvdata(pdev->dev.parent);

 	arm_pm_restart = da9063_reset_do_reset;
	return 0;
}

static int da9063_reset_remove(struct platform_device *pdev)
{
	arm_pm_restart = NULL;
	return 0;
}

static struct platform_driver da9063_reset_driver = {
	.probe		= da9063_reset_probe,
	.remove		= da9063_reset_remove,
	.driver		= {
		.name	= DA9063_DRVNAME_RESET,
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(da9063_reset_driver);

MODULE_AUTHOR("Felix Hammarstrand");
MODULE_DESCRIPTION("reset driver for Dialog DA9063");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_RESET);
