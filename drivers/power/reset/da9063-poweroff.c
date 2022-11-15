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
#include <linux/gpio/consumer.h>

/* This driver uses either a GPIO based shutdown or by writing to a control register 
 on the da9063 chip. If the optional GPIO-pin is assigned in DT it will use it.*/

struct da9063_poweroff {
	struct da9063 *hw;
	struct device *dev;
	struct gpio_desc *pwroff_gpio;
};

/*
 * Hold configuration here, cannot be more than one instance of the driver
 * since pm_power_off itself is global.
 */
struct da9063_poweroff *pwroff_data;

static void da9063_poweroff_do_reg_poweroff(void)
{
	regmap_update_bits(pwroff_data->hw->regmap, DA9063_REG_CONTROL_A,
				 DA9063_SYSTEM_EN, 0);

	mdelay(2000);
	WARN_ON(1);
}

static void da9063_poweroff_do_gpio_poweroff(void)
{
	int ret = gpiod_direction_output(pwroff_data->pwroff_gpio, 1);
	if (ret) {
		dev_err(pwroff_data->dev, "Could not set pwroff gpio (%d)\n",
			ret);
	}
	mdelay(2000);
	WARN_ON(1);
}

static int da9063_poweroff_probe(struct platform_device *pdev)
{

	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;

	pwroff_data = devm_kzalloc(&pdev->dev, sizeof(struct da9063_poweroff),
			     GFP_KERNEL);
	if (!pwroff_data) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	pwroff_data->dev = dev;
	pwroff_data->hw = da9063;

	/* If a pm_power_off function has already been added, leave it alone */
	if (pm_power_off != NULL) {
		pr_err("%s: pm_power_off function already registered",
		       __func__);
		return -EBUSY;
	}

	pwroff_data->pwroff_gpio = devm_gpiod_get_optional(dev, "pwroff", GPIOD_OUT_LOW);

	if (pwroff_data->pwroff_gpio) {
		// Use gpio based shutdown. 
		pm_power_off = &da9063_poweroff_do_gpio_poweroff; 
	} else {
		// If not set we do shutdown by writing a register. 
		pm_power_off = &da9063_poweroff_do_reg_poweroff; 
	}

	return 0;
}

static int da9063_poweroff_remove(struct platform_device *pdev)
{
	if (pm_power_off == &da9063_poweroff_do_reg_poweroff || 
		pm_power_off == &da9063_poweroff_do_gpio_poweroff)
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
