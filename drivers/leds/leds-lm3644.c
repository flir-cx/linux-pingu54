/*
 * lm3644.c - Led flash driver
 *
 * Version:
 * 2020:  : adapted driver for LM3644
 * 2017-12-05: v0.12 : adapted driver for AS3648
 * 2012-09-04: v0.11 : changed rounding, prefere ROUND_UP
 * 2012-07-23: v0.10 : incorporate many review comments
 * 2012-07-10: v0.9 : added automatic shutdown, some review comments
 * 2012-06-11: v0.8 : incorporated reviews, better diag pulse handling
 * 2012-05-04: v0.7 : first version which is adapted to LM3644
 * code originating from AS3648 driver
 * code originating from AS3649 driver
 *
 * Copyright (C) 2020 FLIR
 * Copyright (C) 2012 Ulrich Herrmann <ulrich.herrmann@ams.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/leds-lm3644.h>
#include <linux/leds.h>
#include <linux/regmap.h>

#define LM3644_REG_ENABLE			(0x1)
#define LM3644_REG_IVFM_MODE			(0x2)
#define LM3644_REG_LED1_FLASH_BRIGHTNESS	(0x3)
#define LM3644_REG_LED2_FLASH_BRIGHTNESS	(0x4)
#define LM3644_REG_LED1_TORCH_BRIGHTNESS	(0x5)
#define LM3644_REG_LED2_TORCH_BRIGHTNESS	(0x6)
#define LM3644_REG_BOOST_CONFIG			(0x7)
#define LM3644_REG_TIMING_CONFIG		(0x8)
#define LM3644_REG_TEMP				(0x9)
#define LM3644_REG_FLAGS1			(0xA)
#define LM3644_REG_FLAGS2			(0xB)
#define LM3644_REG_DEVICEID			(0xC)
#define LM3644_REG_LAST_FLASH			(0xD)
#define LM3644_REG_MAX				(0xD)

#define LM3644_REG_DEVICEID_MATCH1  0x02
#define LM3644_REG_DEVICEID_MATCH2  0x04

#define LM3644_ATTR(_name)						\
	__ATTR(_name, 0660, lm3644_##_name##_show, lm3644_##_name##_store)

#define LM3644_RO_ATTR(_name)  \
	__ATTR(_name, 0440, lm3644_##_name##_show, NULL)

#define LM3644_WO_ATTR(_name)  \
	__ATTR(_name, 0220, NULL, lm3644_##_name##_store)
#define attr_name(_attr) ((_attr).attr.name)

static void lm3644_dummy_enable(struct device *dev, bool on)
{
}

struct lm3644_data {
	void (*enable)(struct device *dev, bool on);
	struct lm3644_platform_data *pdata;
	struct i2c_client *client;
	struct mutex update_lock;
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	unsigned int id;
	unsigned int flags1; /* shadow of fault reg for later on read out */
	unsigned int flags2; /* shadow of fault reg for later on read out */
	struct regmap *regmap;
};

static const struct lm3644_data lm3644_default_data = {
	.client = NULL,
};

static int device_add_attributes(struct device *dev,
				 struct device_attribute *attrs)
{
	int error = 0;
	int i;

	if (attrs) {
		for (i = 0; attr_name(attrs[i]); i++) {
			error = device_create_file(dev, &attrs[i]);
			if (error) {
				dev_err(dev, "Failed creating %s",
						attrs[i].attr.name);
				break;
			}
		}
		if (error)
			while (--i >= 0)
				device_remove_file(dev, &attrs[i]);
	}
	return error;
}

static void device_remove_attributes(struct device *dev,
				     struct device_attribute *attrs)
{
	int i;

	if (attrs)
		for (i = 0; attr_name(attrs[i]); i++)
			device_remove_file(dev, &attrs[i]);
}

static bool is_write_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case LM3644_REG_ENABLE:
	case LM3644_REG_IVFM_MODE:
	case LM3644_REG_LED1_FLASH_BRIGHTNESS:
	case LM3644_REG_LED2_FLASH_BRIGHTNESS:
	case LM3644_REG_LED1_TORCH_BRIGHTNESS:
	case LM3644_REG_LED2_TORCH_BRIGHTNESS:
	case LM3644_REG_BOOST_CONFIG:
	case LM3644_REG_TIMING_CONFIG:
	case LM3644_REG_TEMP:
	case LM3644_REG_FLAGS1:
	case LM3644_REG_FLAGS2:
	case LM3644_REG_DEVICEID:
	case LM3644_REG_LAST_FLASH:
		return true;
	default:
		return false;
	}
}

void lm3644_flags_update(struct device *dev)
{
	struct lm3644_data *data = dev_get_drvdata(dev);
	int err;
	unsigned int flags1;
	unsigned int flags2;
	/* flags are cleared when they are read */
	err = regmap_read(data->regmap, LM3644_REG_FLAGS1, &flags1);
	err = regmap_read(data->regmap, LM3644_REG_FLAGS2, &flags2);
	data->flags1 |= flags1;
	data->flags2 |= flags2;
}

static ssize_t lm3644_flags_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lm3644_data *data = dev_get_drvdata(dev);
	unsigned int flags1;
	unsigned int flags2;

	lm3644_flags_update(dev);
	flags1 = data->flags1;
	flags2 = data->flags2;

	data->flags1 = 0;
	data->flags2 = 0;

	return scnprintf(buf, PAGE_SIZE, "FLAGS1: 0x%02x\nFLAGS2: 0x%02x\n",
			 flags1, flags2);
}


static struct device_attribute lm3644_attributes[] = {
	LM3644_RO_ATTR(flags),
	__ATTR_NULL
};


static bool is_read_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		/* /\* case 0x08 ... 0x0F: *\/ */
		/* return false; */
	default:
		return true;
	}
}
static bool is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	/* case ..._REG_...: */
	/*	return true; */
	default:
		return false;
	}
}


#ifdef CONFIG_OF
static struct lm3644_platform_data *lm3644_setup_dt(struct i2c_client *client)
{
	struct lm3644_platform_data *pdata;
	/* struct device_node *np = client->dev.of_node; */

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	return pdata;
}
#else
static int lm3644_setup_dt(struct i2c_client *client)
{
	return -1;
}
#endif

static int lm3644_configure(struct i2c_client *client,
		struct lm3644_data *data, struct lm3644_platform_data *pdata)
{
	int ret = 0;

	regmap_write(data->regmap, LM3644_REG_ENABLE, 0x0);
	regmap_write(data->regmap, LM3644_REG_LED1_TORCH_BRIGHTNESS, 0xce);
	regmap_write(data->regmap, LM3644_REG_LED2_TORCH_BRIGHTNESS, 0xce);
	regmap_write(data->regmap, LM3644_REG_LED1_FLASH_BRIGHTNESS, 0x0);
	regmap_write(data->regmap, LM3644_REG_LED2_FLASH_BRIGHTNESS, 0x0);
	regmap_write(data->regmap, LM3644_REG_LED2_FLASH_BRIGHTNESS, 0x0);
	regmap_write(data->regmap, LM3644_REG_TEMP, 0x09);

	ret = device_add_attributes(&client->dev, lm3644_attributes);

	if (ret < 0)
		goto out;
	return 0;
out:
	device_remove_attributes(&client->dev, lm3644_attributes);
	return ret;
}

/* torch */
static void lm3644_torch_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct lm3644_data *data =
		container_of(cdev, struct lm3644_data, cdev_torch);
	struct device *dev = &data->client->dev;
	lm3644_flags_update(&data->client->dev);
	if (data->flags1 || data->flags2) {
		if ((data->flags1 >> 2) && 1)
			dev_warn(dev, "Thermal shutdown fault detected");
		if ((data->flags1 >> 4) && 1)
			dev_warn(dev, "VLED2 Short fault");
		if ((data->flags1 >> 5) && 1)
			dev_warn(dev, "VLED1 Short fault");
		if ((data->flags1 >> 6) && 1)
			dev_warn(dev, "Vout Short fault");

		if ((data->flags2 >> 0) & 1)
			dev_warn(dev, "TEMP Trip fault detected");
		if ((data->flags2 >> 3) & 1)
			dev_warn(dev, "NTC Open fault detected");
		if ((data->flags2 >> 4) & 1)
			dev_warn(dev, "NTC Short fault detected");
		dev_warn(dev, "Flags was 0x%02x 0x%02x\n",
			 data->flags1, data->flags2);
		data->flags1 = data->flags2 = 0;
	}

	regmap_write(data->regmap, LM3644_REG_ENABLE, brightness ? 0x1b:0);

}

static void lm3644_flash_brightness_set(struct led_classdev *cdev,
					 enum led_brightness brightness)
{
	struct lm3644_data *data =
		container_of(cdev, struct lm3644_data, cdev_flash);

	dev_err(&data->client->dev, "Flash function not implemented\n");
}


static int lm3644_setup_cdev(struct i2c_client *client,
			     struct lm3644_data *data)
{
	int err;

	data->cdev_flash.name = "flash";
	data->cdev_flash.max_brightness = 100;
	data->cdev_flash.brightness_set = lm3644_flash_brightness_set;
	data->cdev_flash.default_trigger = "flash";
	err = led_classdev_register((struct device *)
				    &client->dev, &data->cdev_flash);
	if (err < 0)
		goto err_out;

	data->cdev_torch.name = "torch";
	data->cdev_torch.max_brightness = 100;
	data->cdev_torch.brightness_set = lm3644_torch_brightness_set;
	data->cdev_torch.default_trigger = "torch";
	err = led_classdev_register((struct device *)
				    &client->dev, &data->cdev_torch);
	if (err < 0)
		goto err_create_torch_file;

	data->cdev_torch.flags |= LED_CORE_SUSPENDRESUME;
	return 0;

err_create_torch_file:
	led_classdev_unregister(&data->cdev_flash);
err_out:
	return err;
}


static const struct regmap_config lm3644_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LM3644_REG_MAX,
	.writeable_reg  = is_write_reg,
	.readable_reg   = is_read_reg,
	.volatile_reg   = is_volatile_reg,
};

static int lm3644_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lm3644_data *data;
	struct lm3644_platform_data *lm3644_pdata = client->dev.platform_data;
	int err = 0;
	if (client->dev.of_node) {
		lm3644_pdata = lm3644_setup_dt(client);
		if (IS_ERR(lm3644_pdata)) {
			err = PTR_ERR(lm3644_pdata);
			dev_err(&client->dev, "%s: No platform data\n",
				__func__);
			return err;
		}
	}

	dev_info(&client->dev, "Probing LM3644 device\n");

	if (!lm3644_pdata)
		return -EIO;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_USER);
	if (!data)
		return -ENOMEM;

	/* initialize with meaningful data (register names, etc.) */
	*data = lm3644_default_data;

	dev_set_drvdata(&client->dev, data);

	data->client = client;

	data->regmap = devm_regmap_init_i2c(client, &lm3644_regmap);
	if (IS_ERR(data->regmap)) {
		err = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			err);
		return err;
	}

	mutex_init(&data->update_lock);

	data->enable = lm3644_dummy_enable;
	if (lm3644_pdata->enable)
		data->enable = lm3644_pdata->enable;
	if (lm3644_pdata->init)
		lm3644_pdata->init(&client->dev, true);
	data->enable(&client->dev, true);

	err = regmap_read(data->regmap, LM3644_REG_DEVICEID, &data->id);
	if (err < 0) {
		data->enable(&client->dev, false);
		dev_err(&client->dev, "Failed to read REG_FLAG Register (%d)\n", err);
		goto exit;
	}

	if (data->id != LM3644_REG_DEVICEID_MATCH1 &&
	    data->id != LM3644_REG_DEVICEID_MATCH2) {
		err = -ENXIO;
		dev_err(&client->dev, "wrong chip detected,ids 0x%02x",
			data->id);
		data->enable(&client->dev, false);
		goto exit;
	}

	dev_info(&client->dev, "LM3644 driver detected %s id=0x%02x",
		 "LM3644", data->id);

	i2c_set_clientdata(client, data);

	err = lm3644_configure(client, data, lm3644_pdata);

	data->enable(&client->dev, false);

	if (err < 0)
		goto exit;

	/* pm_runtime_enable(&client->dev); */
	/* pm_suspend_ignore_children(&client->dev, true); */
	/* err = pm_runtime_get_sync(&data->client->dev); */
	/* if (err < 0) */
	/*	goto exit; */
	/* pm_runtime_set_autosuspend_delay(&client->dev, */
	/*           lm3644_pdata->autosuspend_delay_ms); */
	/* pm_runtime_use_autosuspend(&client->dev); */
	/* pm_runtime_mark_last_busy(&data->client->dev); */
	/* pm_runtime_put_autosuspend(&data->client->dev); */

	err = lm3644_setup_cdev(client, data);

exit:
	if (err < 0) {
		if (lm3644_pdata->init)
			lm3644_pdata->init(&client->dev, false);
		dev_err(&client->dev, "could not configure %d", err);
		i2c_set_clientdata(client, NULL);
	}

	return err;
}


static int lm3644_remove(struct i2c_client *client)
{
	struct lm3644_data *data = i2c_get_clientdata(client);

	dev_info(&client->dev, "Removing LM3644 device\n");

	led_classdev_unregister(&data->cdev_flash);
	led_classdev_unregister(&data->cdev_torch);
	regmap_write(data->regmap, LM3644_REG_ENABLE, 0x0);

	device_remove_attributes(&client->dev, lm3644_attributes);
	/* pm_runtime_get_sync(&data->client->dev); */
	/* lm3644_set_leds(data, 3, LM3644_REG_Control_mode_etorch, 0); */
	/* pm_runtime_put_sync(&data->client->dev); */
	/* pm_runtime_suspend(&client->dev); */
	/* pm_runtime_disable(&client->dev); */
	/* if (data->pdata->init) */
	/*	data->pdata->init(&client->dev, false); */
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int lm3644_suspend(struct device *dev)
{
	struct lm3644_data *data = dev_get_drvdata(dev);
	int err;
	err = regmap_read(data->regmap, LM3644_REG_FLAGS1, &data->flags1);
	err |= regmap_read(data->regmap, LM3644_REG_FLAGS2, &data->flags2);
	data->enable(dev, false);
	regmap_write(data->regmap, LM3644_REG_ENABLE, 0x0);
	return err;
}

static int lm3644_resume(struct device *dev)
{
	struct lm3644_data *data = dev_get_drvdata(dev);

	data->enable(dev, true);
	return 0;
}
#endif

#if CONFIG_PM
static const struct dev_pm_ops lm3644_pm = {
	SET_RUNTIME_PM_OPS(lm3644_suspend, lm3644_resume, NULL)
};
#endif
static void lm3644_shutdown(struct i2c_client *client)
{
	struct lm3644_data *data = i2c_get_clientdata(client);

	dev_info(&client->dev, "Shutting down LM3644 device\n");

	data->enable(&client->dev, false);
	regmap_write(data->regmap, LM3644_REG_ENABLE, 0x0);
}

static const struct i2c_device_id lm3644_id[] = {
	{ "lm3644", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lm3644_id);

#ifdef CONFIG_OF
static const struct of_device_id lm3644_of_match[] = {
	{ .compatible = "ti,lm3644", },
	{},
};

MODULE_DEVICE_TABLE(of, lm3644_of_match);
#endif

static struct i2c_driver lm3644_driver = {
	.driver = {
		.name   = "lm3644",
#if CONFIG_PM
		.pm = &lm3644_pm,
#endif
	},
	.probe  = lm3644_probe,
	.remove = lm3644_remove,
	.id_table = lm3644_id,
	.shutdown = lm3644_shutdown,
};

static int __init lm3644_init(void)
{
	return i2c_add_driver(&lm3644_driver);
}

static void __exit lm3644_exit(void)
{
	i2c_del_driver(&lm3644_driver);
}

MODULE_AUTHOR("Bo Svangård <bo.svangard@flir.se>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LM3644 LED flash light");

module_init(lm3644_init);
module_exit(lm3644_exit);
