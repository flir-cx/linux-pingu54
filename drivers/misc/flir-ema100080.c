// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 TeledyneFlir
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include "flir-ema100080.h"

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>

struct flir_ema100080_i2c_cmd {
	u8 reg;
	u8 cmd;
};

/**
 * Holds the data read from device tree and other housekeeping data.
 */
struct flir_ema100080_data {
	struct device *dev;

	struct i2c_client *client;

	/* Commands to run on startup, to init emagin display */
	int i2c_config_cmds_size;
	struct flir_ema100080_i2c_cmd i2c_config_cmds[FLIR_EMA100080_MAX_I2C_CMDS];

	/* GPIO for enabling psave on dac */
	struct gpio_desc *fvm_psave_gpiod;

	struct regulator *supply;

	/* ioctl */
	struct miscdevice miscdev;
};

/* Forwards for device */
static int flir_ema100080_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int flir_ema100080_remove(struct i2c_client *client);

/* Forwards for ioctl */
static long flir_ema100080_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
static int flir_ema100080_dev_open(struct inode *inode, struct file *filp);
static int flir_ema100080_do_ioctl(struct flir_ema100080_data *vf, u32 cmd, u8 *buf);

/* Forwards internal */
static int
flir_ema100080_read_dt_i2c_cmds(struct device *dev, struct flir_ema100080_i2c_cmd *i2c_cmds);
static int flir_ema100080_read_dt_driver_data(struct device *dev, struct flir_ema100080_data *vf);

/* Module loading/unloading responses */
static int flir_ema100080_on_probe(struct flir_ema100080_data *vf);
static int flir_ema100080_on_remove(struct flir_ema100080_data *vf);

/* ioctl responses */
static int flir_ema100080_set_pwr_on(struct flir_ema100080_data *vf);
static int flir_ema100080_set_pwr_off(struct flir_ema100080_data *vf);
static int flir_ema100080_get_pwr_on(struct flir_ema100080_data *vf);

static const struct file_operations miscdev_fops = {
	.owner = THIS_MODULE,
	.open = flir_ema100080_dev_open,
	.unlocked_ioctl = flir_ema100080_ioctl,
};
/**
 * Handle reads on
 * /sys/devices/platform/soc/2100000.bus/21a0000.i2c/i2c-0/0-0032/pwr_on
 * or similar
 *
 * @param dev device
 * @param attr device attributes
 * @param buf the resulting value as a string
 * @return ssize_t number of bytes put in buf
 */
static ssize_t pwr_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct flir_ema100080_data *vf = (struct flir_ema100080_data *)dev_get_drvdata(dev);

	ret = flir_ema100080_get_pwr_on(vf);
	if (ret < 0) {
		dev_err(dev, "power on failed\n");
		return 0;
	}
	return sysfs_emit(buf, "%d\n", ret);
}

/**
 * Handles writes on
 * /sys/devices/platform/soc/2100000.bus/21a0000.i2c/i2c-0/0-0032/pwr_on
 * or similar
 *
 * @param dev device
 * @param attr device attributes
 * @param buf the input written to the file
 * @param count number of bytes in buf
 * @return ssize_t
 */
static ssize_t pwr_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool on;
	int ret;

	struct flir_ema100080_data *vf = (struct flir_ema100080_data *)dev_get_drvdata(dev);

	if (kstrtobool(buf, &on) < 0)
		return -EINVAL;
	if (on)
		ret = flir_ema100080_set_pwr_on(vf);
	else
		ret = flir_ema100080_set_pwr_off(vf);

	return (ret < 0) ? ret : count;
}

static DEVICE_ATTR_RW(pwr_on);
static struct attribute *flirvf_attrs[] = { &dev_attr_pwr_on.attr, NULL };
static const struct attribute_group flirvf_attr_groups = {
	.attrs = flirvf_attrs,
};

static const struct of_device_id flir_ema100080_of_match[] = {
	{
		.compatible = FLIR_EMA100080_COMPATIBLE_STR,
	},
	{}
};
MODULE_DEVICE_TABLE(of, flir_ema100080_of_match);

/**
 * Probe, called on platform device match
 *
 * @param pdev	platform device as found in device tree
 * @return	0 - On success
 *		negative - on error
 */
static int flir_ema100080_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct flir_ema100080_data *vf;

	const struct of_device_id *match;
	struct device *dev = &client->dev;

	dev_dbg(dev, "probe IOCTL_FLIR_VF_PWR_ON_GET IOCTL_R=%u\n", IOCTL_FLIR_VF_PWR_ON_GET);
	dev_dbg(dev, "probe IOCTL_FLIR_VF_PWR_ON IOCTL_W=%u\n", IOCTL_FLIR_VF_PWR_ON);
	dev_dbg(dev, "probe IOCTL_FLIR_VF_PWR_OFF IOCTL_W=%u\n", IOCTL_FLIR_VF_PWR_OFF);

	// sanity check
	match = of_match_device(flir_ema100080_of_match, &client->dev);
	if (!match) {
		dev_err(dev, "no device tree match for platform device!\n");
		return -EINVAL;
	}

	/* Read driver data from device-tree */
	vf = devm_kzalloc(dev, sizeof(struct flir_ema100080_data), GFP_KERNEL);
	if (!vf)
		return -ENOMEM;

	ret = flir_ema100080_read_dt_driver_data(&client->dev, vf);
	if (ret < 0)
		return ret;

	/* Setup ioctl access */
	vf->miscdev.minor = MISC_DYNAMIC_MINOR;
	vf->miscdev.name = FLIR_EMA100080_NAME;
	vf->miscdev.fops = &miscdev_fops;
	vf->dev = &client->dev;
	vf->client = client;
	i2c_set_clientdata(client, vf);

	vf->supply = devm_regulator_get(dev, "vin");
	if (IS_ERR(vf->supply)) {
		ret = PTR_ERR(vf->supply);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "can't get regulator vin-supply (%i)", ret);
		else
			dev_err(dev, "Deferred probe");
		dev_err(dev, "Got error %i\n", ret);
		vf->supply = NULL;
		return ret;
	}

	ret = misc_register(&vf->miscdev);
	if (ret) {
		dev_err(dev, "Failed to register miscdev\n");
		return ret;
	}

	ret = sysfs_create_group(&client->dev.kobj, &flirvf_attr_groups);
	if (ret) {
		dev_err(dev, "Failed to add sys fs entry\n");
		goto err_misc_deregister;
	}
	/* Call the probe implementation */
	ret = flir_ema100080_on_probe(vf);
	if (ret < 0) {
		dev_err(dev, "Probe vf callback failed %d\n", ret);
		goto err_remove_group;
	}
	ret = flir_ema100080_on_remove(vf);
	dev_info(dev, "Viewfinder flir ema100080 found!\n");
	return ret;

err_remove_group:
	dev_dbg(&client->dev, "remove sysfs group\n");
	sysfs_remove_group(&client->dev.kobj, &flirvf_attr_groups);
err_misc_deregister:
	dev_dbg(&client->dev, "deregister misc dev\n");
	misc_deregister(&vf->miscdev);

	dev_err(dev, "Probing for ema100080 failed!\n");
	return ret;
}

/**
 * Remove, called on platform device removal
 *
 * @param pdev Pointer to platform device being removed.
 * @return 0 on success
 */
static int flir_ema100080_remove(struct i2c_client *client)
{
	int ret;
	struct flir_ema100080_data *vf = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	sysfs_remove_group(&client->dev.kobj, &flirvf_attr_groups);
	dev_dbg(&client->dev, "remove sysfs group\n");

	misc_deregister(&vf->miscdev);
	dev_dbg(&client->dev, "deregister misc dev\n");

	ret = flir_ema100080_on_remove(vf);
	vf->dev = 0;
	if (ret < 0)
		dev_err(&client->dev, "%s: remove vf callback failed\n", __func__);

	dev_dbg(&client->dev, "%s: exit\n", __func__);

	return 0;
}

static int flir_ema100080_dev_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static long flir_ema100080_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *mdev = (struct miscdevice *)file->private_data;
	struct flir_ema100080_data *vf;
	long ret = 0;
	char *tmp;
	struct device *dev;

	vf = container_of(mdev, struct flir_ema100080_data, miscdev);

	if (!vf) {
		pr_err("%s: flir_ema100080_data struct is NULL.", __func__);
		return -EFAULT;
	}

	dev = vf->dev;
	tmp = kzalloc(_IOC_SIZE(cmd), GFP_KERNEL);

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = copy_from_user(tmp, (void *)arg, _IOC_SIZE(cmd));
		if (ret)
			dev_err(dev, "Copy from user failed: %ld\n", ret);
	}

	if (!ret) {
		ret = flir_ema100080_do_ioctl(vf, cmd, tmp);
		if (ret)
			dev_err(dev, "ioctl 0x%X failed: %ld\n", cmd, ret);
	}

	if ((!ret) && (_IOC_DIR(cmd) & _IOC_READ)) {
		ret = copy_to_user((void *)arg, tmp, _IOC_SIZE(cmd));
		if (ret)
			dev_err(dev, "Copy to user failed: %ld\n", ret);
	}
	kfree(tmp);

	return ret;
}

static int flir_ema100080_do_ioctl(struct flir_ema100080_data *vf, u32 cmd, u8 *buf)
{
	int ret = -EINVAL;

	switch (cmd) {
	case IOCTL_FLIR_VF_PWR_ON_GET:
		ret = flir_ema100080_get_pwr_on(vf);
		if (ret < 0)
			return ret;

		memcpy(buf, &ret, sizeof(ret));
		ret = 0;
		break;
	case IOCTL_FLIR_VF_PWR_OFF:
		ret = flir_ema100080_set_pwr_off(vf);
		break;
	case IOCTL_FLIR_VF_PWR_ON:
		ret = flir_ema100080_set_pwr_on(vf);
		break;
	default:
		dev_err(vf->dev, "ioctl %u not supported\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * Read an initialization array of tuples <8u u8> into i2c_commands
 *
 * @param dev		Device in which the device tree node lives.
 * @param i2c_cmds	The command array to fill
 * @return		Ńumber of tuples read from dt, negative on failure.
 */
static int
flir_ema100080_read_dt_i2c_cmds(struct device *dev, struct flir_ema100080_i2c_cmd *i2c_cmds)
{
	static const char propname[] = "i2c-config-cmds";
	int num_args = 2; // tuples of 2
	int num_cmds;
	int arr_len;
	int i;
	struct device_node *np = dev->of_node;

	if (!of_get_property(np, propname, &arr_len)) {
		dev_err(dev, "%s missing or invalid to be compatible with %s\n", propname, FLIR_EMA100080_COMPATIBLE_STR);
		return -EINVAL;
	}

	num_cmds = arr_len / (sizeof(u32) * num_args);
	if (num_cmds > FLIR_EMA100080_MAX_I2C_CMDS) {
		dev_err(dev, "%s contains more than max (%d) commands\n", propname, FLIR_EMA100080_MAX_I2C_CMDS);
		return -EOVERFLOW;
	}

	for (i = 0; i < num_cmds; i++) {
		u32 reg;
		u32 cmd;
		int offset = i * num_args;

		if (of_property_read_u32_index(np, propname, offset, &reg))
			goto fail_prop_read;

		if (of_property_read_u32_index(np, propname, offset + 1, &cmd))
			goto fail_prop_read;
		if (reg > U8_MAX) {
			dev_err(dev, "%s[%i][0] = 0x%x does not fit in a u8\n", propname, i, reg);
			goto fail_prop_read;
		}
		if (cmd > U8_MAX) {
			dev_err(dev, "%s[%i][2] = 0x%x does not fit in a u8\n", propname, i, cmd);
			goto fail_prop_read;
		}
		i2c_cmds[i].cmd = cmd;
		i2c_cmds[i].reg = reg;

		dev_dbg(dev, "%s[%i] reg=0x%02hhx cmd 0x%02hhx\n", propname, i, i2c_cmds[i].reg, i2c_cmds[i].cmd);
	}
	return num_cmds;
fail_prop_read:
	return -EINVAL;
}
/**
 * Reads device tree to fill driver data.
 *
 * @param dev		device
 * @param vf		where to place the data read from device-tree,
 *			memory needs to be allocated by caller.
 * @return int		On success, returns 0 and the vf structure is
 *			filled.
 *			In case of error an negative error code is returned.
 */
static int flir_ema100080_read_dt_driver_data(struct device *dev, struct flir_ema100080_data *vf)
{
	int ret;
	int retval = 0;
	//struct device_node *np = dev->of_node;

	vf->fvm_psave_gpiod = devm_gpiod_get(dev, "eoco-fvm-psave", GPIOD_ASIS);
	if (IS_ERR(vf->fvm_psave_gpiod)) {
		dev_err(dev, "unable to get eoco-fvm-psave gpio from dt\n");
		ret = PTR_ERR(vf->fvm_psave_gpiod);
	}

	ret = flir_ema100080_read_dt_i2c_cmds(dev, vf->i2c_config_cmds);
	if (ret < 0) {
		dev_err(dev, "unable to get i2c-bus\n");
		retval = (retval == 0 ? ret : 0);
	} else {
		vf->i2c_config_cmds_size = ret;
	}

	return retval;
}

/**
 * @brief Initializes the emagin display
 *
 * @param vf viewfinder data struct, holds the device data
 * @return int non negative value on success
 */
static int init_emagin_lcd(struct flir_ema100080_data *vf)
{
	int i;
	int ret = 0;

	dev_dbg(vf->dev, "Enter %s", __func__);

	for (i = 0; i < vf->i2c_config_cmds_size; i++) {
		u8 reg = vf->i2c_config_cmds[i].reg;
		u8 cmd = vf->i2c_config_cmds[i].cmd;

		dev_dbg(vf->dev, "i2c write[%i] reg=0x%02hhx cmd 0x%02hhx\n", i, reg, cmd);
		ret = i2c_smbus_write_byte_data(vf->client, reg, cmd);

		if (ret < 0) {
			dev_err(vf->dev, "i2c write failed, is the eMagin lcd connected?\n");
			return ret;
		}
	}
	dev_dbg(vf->dev, "i2c write complete\n");
	return 0;
}

/**
 * @brief Set power on
 *
 * @param vf viewfinder data struct, holds the device data
 * @return int non negative value on success
 */
static int flir_ema100080_set_pwr_on(struct flir_ema100080_data *vf)
{
	int ret = 0;

	dev_dbg(vf->dev, "Set psave high\n");
	if (gpiod_get_direction(vf->fvm_psave_gpiod)) {
		ret = gpiod_direction_output(vf->fvm_psave_gpiod, 1);
		if (ret) {
			dev_err(vf->dev, "Could not set psave gpio (%d)\n", ret);
			return ret;
		}
	}

	ret = regulator_enable(vf->supply);

	// Need to wait for chip to power up
	mdelay(100);

	return init_emagin_lcd(vf);
}

/**
 * @brief Called when device is probed
 *
 * @param vf viewfinder data struct, holds the device data
 * @return int non negative value on success
 */
static int flir_ema100080_on_probe(struct flir_ema100080_data *vf)
{
	int ret = flir_ema100080_set_pwr_on(vf);

	if (ret < 0) {
		dev_err(vf->dev, "Failed to initialize emagin lcd\n");
		return ret;
	}

	return 0;
}

/**
 * @brief Set power off
 *
 * @param vf viewfinder data struct, holds the device data
 * @return int non negative value on success
 */
static int flir_ema100080_set_pwr_off(struct flir_ema100080_data *vf)
{
	int ret = 0;

	dev_dbg(vf->dev, "Set psave to input\n");
	if (!gpiod_get_direction(vf->fvm_psave_gpiod)) {
		if (gpiod_direction_input(vf->fvm_psave_gpiod)) {
			dev_err(vf->dev, "Could not set psave gpio to input\n");
			return -EFAULT;
		}
	}

	if (regulator_is_enabled(vf->supply))
		ret = regulator_disable(vf->supply);

	return 0;
}

/**
 * @brief Called when device is removed
 *
 * @param vf viewfinder data struct, holds the device data
 * @return int non negative value on success
 */
static int flir_ema100080_on_remove(struct flir_ema100080_data *vf)
{
	return flir_ema100080_set_pwr_off(vf);
}

/**
 * @brief Get the power satatus of device
 *
 * @param vf viewfinder data struct, holds the device data
 * @return int non negative value on success
 */
static int flir_ema100080_get_pwr_on(struct flir_ema100080_data *vf)
{
	return gpiod_get_value_cansleep(vf->fvm_psave_gpiod);
}

//static SIMPLE_DEV_PM_OPS(flir_ema100080_pm_ops, flir_ema100080_suspend, flir_ema100080_resume);

static const struct i2c_device_id flir_ema100080_id[] = {
	{ FLIR_EMA100080_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, flir_ema100080_id);

static struct i2c_driver flir_ema100080_driver = {
	.driver = {
		.name	= FLIR_EMA100080_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = flir_ema100080_of_match,
//		.pm	= &flir_ema100080_pm_ops,
	},
	.probe		= flir_ema100080_probe,
	.remove		= flir_ema100080_remove,
	.id_table	= flir_ema100080_id,
};

module_i2c_driver(flir_ema100080_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FLIR Viewfinder driver, companion of mxc_lcd_if");
MODULE_AUTHOR("Jonas Rydow");

//******************************************************************
