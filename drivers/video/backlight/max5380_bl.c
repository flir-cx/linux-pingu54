// SPDX-License-Identifier: GPL-2.0-only
/*
* Simple driver for Texas Instruments max5380 Backlight driver chip
* Copyright (C) 2012 Texas Instruments
*/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#define FLIR_MAX5380_COMPATIBLE_STR "flir,max5380"
#define MAX5380_NAME "backlight_vf"

/**
 * @brief Structure holding driver data
 *
 */
struct max5380_data {
	struct i2c_client *client;
	struct backlight_device *bdev;

	/* Taken from device tree */
	u32 default_brightness;
	u32 max_brightness;
};

static const struct of_device_id max5380_match_table[] = {
	{
		.compatible = FLIR_MAX5380_COMPATIBLE_STR,
	},
	{},
};

MODULE_DEVICE_TABLE(of, max5380_match_table);
#ifdef CONFIG_OF
/**
 * @brief Returns a u32 from device tree or default value if not found.
 *
 * @param dev The device containing the tree
 * @param prop The name of the property containing the value
 * @param default_val Return this value as default if prop not found
 * @return u32
 */
static u32 max5380_read_of_u32(struct device *dev, const char *prop,
			       u32 default_val)
{
	struct device_node *np = dev->of_node;
	u32 v;
	if (of_property_read_u32(np, prop, &v)) {
		dev_dbg(dev, "%s: %s defaults to %u\n", __func__, prop,
			default_val);
		return default_val;
	} else {
		dev_dbg(dev, "%s: %s read to %u\n", __func__, prop, v);
		return v;
	}
}

/**
 * @brief Parses device tree for max5380_data
 *
 * @param dev Device holding device tree data
 * @param bld backlight data structure
 * @return int 	0 on success.
 * 		-EINVAL if the device tree entry is not found.
 */
static int max5380_parse_dt(struct device *dev, struct max5380_data *bld)
{
	const struct of_device_id *match;

	dev_dbg(dev, "%s\n", __func__);
	match = of_match_device(max5380_match_table, dev);
	if (!match) {
		dev_err(dev, "no device tree match for platform device!\n");
		return -EINVAL;
	}

	bld->default_brightness = max5380_read_of_u32(dev, "default-brightness", 55);
	bld->max_brightness = max5380_read_of_u32(dev, "max-brightness", 253);
	return 0;
}
#else
static inline void max5380_parse_dt(struct device *dev, struct max5380_data *bm)
{
	return 0;
}
#endif

/**
 * @brief called from linux
 *
 * @param backlight Backlight device.
 * @return 0 on success
 */
static int max5380_update_status(struct backlight_device *backlight)
{
	u8 b;
	int ret;
	struct max5380_data *bld = bl_get_data(backlight);
	int brightness = backlight_get_brightness(backlight);

	b = brightness;
	if (brightness > 0) {
		/* scale into 0 - 255 */
		b = (u8)(((u32)brightness * 255) /
			 ((u32)backlight->props.max_brightness));
		dev_err(&bld->client->dev,
			"setting backlight to %u scaled from %u\n", b,
			brightness);
	} else {
		b = 0;
	}

	ret = i2c_smbus_write_byte(bld->client, b);

	if (ret < 0) {
		dev_err(&bld->client->dev, "i2c write failed\n");
		return ret;
	}
	return 0;
}

static const struct backlight_ops max5380_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = max5380_update_status,
};

/**
 * @brief Probe function, called by linux.
 * 
 * @param client i2c client
 * @param id	 i2c device id 
 * @return 0 on success
 */
static int max5380_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct backlight_properties props;
	struct max5380_data *bld;
	int ret;
	dev_dbg(&client->dev, "%s: probing\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check\n");
		return -EOPNOTSUPP;
	}

	bld = devm_kzalloc(&client->dev, sizeof(struct max5380_data),
			   GFP_KERNEL);
	if (!bld)
		return -ENOMEM;

	max5380_parse_dt(&client->dev, bld);

	bld->client = client;

	/* backlight register */
	props.type = BACKLIGHT_RAW;
	props.max_brightness = bld->max_brightness;
	props.brightness =
		clamp_t(u32, bld->default_brightness, 0, props.max_brightness);
	bld->bdev =
		devm_backlight_device_register(&bld->client->dev, MAX5380_NAME,
					       &bld->client->dev, bld,
					       &max5380_bl_ops, &props);
	if (IS_ERR(bld->bdev)) {
		dev_err(&bld->client->dev,
			"failed to register backlight device\n");
		return PTR_ERR(bld->bdev);
	}

	i2c_set_clientdata(client, bld->bdev);
	if (ret < 0) {
		dev_err(&client->dev, "fail : backlight register.\n");
		return ret;
	}
	dev_info(&client->dev, "max5380 backlight register OK.\n");

	return 0;
}

/**
 * @brief Remove, called by linux
 * 
 * @param client i2c client
 * @return 0 on success
 */
static int max5380_remove(struct i2c_client *client)
{
	struct backlight_device *backlight = i2c_get_clientdata(client);

	backlight->props.brightness = 0;
	backlight_update_status(backlight);

	dev_err(&client->dev, "%s: remove\n", __func__);
	return 0;
}

static const struct i2c_device_id max5380_id[] = { { MAX5380_NAME, 0 }, {} };

MODULE_DEVICE_TABLE(i2c, max5380_id);

static struct i2c_driver max5380_i2c_driver = {
	.driver = {
			.name = MAX5380_NAME,
			.of_match_table = max5380_match_table,
		},
	.probe = max5380_probe,
	.remove = max5380_remove,
	.id_table = max5380_id,
};

module_i2c_driver(max5380_i2c_driver);

MODULE_DESCRIPTION("FLIR Backlight driver for max5380");
MODULE_AUTHOR("Jonas Rydow <jonas.rydow@teledyneflir.com>");
MODULE_LICENSE("GPL v2");
