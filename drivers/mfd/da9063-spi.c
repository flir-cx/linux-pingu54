/*
 * SPI access for Dialog DA9063 PMICs.
 *
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *
 * Author: David Dajun Chen <dchen@diasemi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/mfd/core.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/regmap.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>

static int da9063_spi_probe(struct spi_device *spi)
{
	int ret;
	struct da9063 *da9063;

	da9063 = devm_kzalloc(&spi->dev, sizeof(struct da9063), GFP_KERNEL);
	if (!da9063)
		return -ENOMEM;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	da9063->dev = &spi->dev;
	da9063->chip_irq = spi->irq;

        /* FLIR FIXME: Hard coded type and variant should be taken from
        *  registers or possibly device tree.
        */
	da9063->type = PMIC_TYPE_DA9063;
	da9063->variant_code = PMIC_DA9063_CA;

        
	spi_set_drvdata(spi, da9063);

	da9063_regmap_config.read_flag_mask = 1;
	da9063_regmap_config.write_flag_mask = 0;
	da9063_regmap_config.reg_bits = 7;
	da9063_regmap_config.pad_bits = 1;
	da9063_regmap_config.use_single_read = 1;
	da9063_regmap_config.use_single_write = 1;
	da9063_range_cfg->selector_mask = 0x7;
	da9063_range_cfg->selector_shift = 0;
	da9063_range_cfg->window_len = 128;

	ret = da9063_register_regmap(da9063);
	if (ret) {
		dev_err(da9063->dev, "Failed to register regmap: %d\n", ret);
		return ret;
	}
	da9063->regmap = devm_regmap_init_spi(spi, &da9063_regmap_config);
	if (IS_ERR(da9063->regmap)) {
		ret = PTR_ERR(da9063->regmap);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	return da9063_device_init(da9063, spi->irq);
}

static struct spi_device_id da9063_spi_id[] = {
	{ "da9063", PMIC_TYPE_DA9063 },
	{ "da9063l", PMIC_TYPE_DA9063L },
	{}
};

static struct spi_driver da9063_spi_driver = {
	.probe = da9063_spi_probe,
	.id_table = da9063_spi_id,
	.driver = {
		.name = "da9063",
		.owner = THIS_MODULE,
	},
};

static int __init da9063_spi_init(void)
{
	int ret;

	ret = spi_register_driver(&da9063_spi_driver);
	if (ret != 0) {
		pr_err("Failed to register DA9063 SPI driver, %d\n", ret);
		return ret;
	}

	return 0;
}
subsys_initcall(da9063_spi_init);

static void __exit da9063_spi_exit(void)
{
	spi_unregister_driver(&da9063_spi_driver);
}
module_exit(da9063_spi_exit);

MODULE_AUTHOR("David Dajun Chen <dchen@diasemi.com>");
MODULE_DESCRIPTION("SPI driver for Dialog DA9063 PMIC");
MODULE_LICENSE("GPL");
