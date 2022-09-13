/* Texas Instruments TMP114 SMBus temperature sensor driver
 *
 * Copyright (C) 2010 Steven King <sfking@fdwdc.com>
 * Copyright (C) 2022 Mathias Båge <mathias.bage@teledyneflir.com>
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

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/regmap.h>
#include <linux/of.h>

#define	DRIVER_NAME "tmp114"

#define	TMP114_TEMP_REG			0x00

struct tmp114 {
	struct regmap *regmap;
	u16 config_orig;
	unsigned long ready_time;
};

/* convert 16-bit TMP114 register value to milliCelsius */
static inline int tmp114_reg_to_mC(s16 val)
{
	return (val * 1000) / 128;
}

static int tmp114_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *temp)
{
	struct tmp114 *tmp114 = dev_get_drvdata(dev);
	unsigned int regval;
	int err, reg;

	switch (attr) {
	case hwmon_temp_input:
		// continuous conversion => always ready
		reg = TMP114_TEMP_REG;
		break;
	default:
		return -EOPNOTSUPP;
	}

	err = regmap_read(tmp114->regmap, reg, &regval);
	if (err < 0)
		return err;
	*temp = tmp114_reg_to_mC(regval);

	return 0;
}

static umode_t tmp114_is_visible(const void *data, enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return S_IRUGO;
	default:
		return 0;
	}
}

static u32 tmp114_chip_config[] = {
	HWMON_C_REGISTER_TZ,
	0
};

static const struct hwmon_channel_info tmp114_chip = {
	.type = hwmon_chip,
	.config = tmp114_chip_config,
};

static u32 tmp114_temp_config[] = {
	HWMON_T_INPUT, 0
};

static const struct hwmon_channel_info tmp114_temp = {
	.type = hwmon_temp,
	.config = tmp114_temp_config,
};

static const struct hwmon_channel_info *tmp114_info[] = {
	&tmp114_chip,
	&tmp114_temp,
	NULL
};

static const struct hwmon_ops tmp114_hwmon_ops = {
	.is_visible = tmp114_is_visible,
	.read = tmp114_read,
};

static const struct hwmon_chip_info tmp114_chip_info = {
	.ops = &tmp114_hwmon_ops,
	.info = tmp114_info,
};

static const struct regmap_config tmp114_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = TMP114_TEMP_REG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
	.use_single_rw = true,
};

static int tmp114_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct tmp114 *tmp114;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(dev,
			"adapter doesn't support SMBus word transactions\n");
		return -ENODEV;
	}

	tmp114 = devm_kzalloc(dev, sizeof(*tmp114), GFP_KERNEL);
	if (!tmp114)
		return -ENOMEM;

	i2c_set_clientdata(client, tmp114);

	tmp114->regmap = devm_regmap_init_i2c(client, &tmp114_regmap_config);
	if (IS_ERR(tmp114->regmap))
		return PTR_ERR(tmp114->regmap);

	// we always use TMP114's power-on reset defaults (continuous conversion mode)
	// so no need to set or save configuration register bits

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 tmp114,
							 &tmp114_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev)) {
		dev_dbg(dev, "unable to register hwmon device\n");
		return PTR_ERR(hwmon_dev);
	}

	return 0;
}

static const struct i2c_device_id tmp114_id[] = {
	{ "tmp114", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tmp114_id);

static const struct of_device_id tmp114_of_match[] = {
	{ .compatible = "ti,tmp114" },
	{ },
};
MODULE_DEVICE_TABLE(of, tmp114_of_match);

static struct i2c_driver tmp114_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.of_match_table = of_match_ptr(tmp114_of_match),
	.probe		= tmp114_probe,
	.id_table	= tmp114_id,
};

module_i2c_driver(tmp114_driver);

MODULE_AUTHOR("Mathias Båge <mathias.bage@teledyneflir>");
MODULE_DESCRIPTION("Texas Instruments TMP114 temperature sensor driver");
MODULE_LICENSE("GPL");
