// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input/kxtj9.h>
#include <linux/input-polldev.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#define NAME			"kxtj9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
#define SELF_TEST		0x3A
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
#define STPOL			(1 << 1) /* Self test polarity */
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* DATA CONTROL REGISTER BITS */
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F		3
#define ODR200F		4
#define ODR400F		5
#define ODR800F		6
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL		(1 << 3)
#define KXTJ9_IEA		(1 << 4)
#define KXTJ9_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

/* IOCTL defines */
#define	SENSOR_IOCTL_BASE		'S'
#define	SENSOR_GET_MODEL_NAME		_IOR(SENSOR_IOCTL_BASE, 0, char *)
#define	SENSOR_GET_POWER_STATUS		_IOR(SENSOR_IOCTL_BASE, 2, int)
#define	SENSOR_SET_POWER_STATUS		_IOR(SENSOR_IOCTL_BASE, 3, int)
#define	SENSOR_GET_DELAY_TIME		_IOR(SENSOR_IOCTL_BASE, 4, int)
#define	SENSOR_SET_DELAY_TIME		_IOR(SENSOR_IOCTL_BASE, 5, int)
#define	SENSOR_GET_RAW_DATA			_IOR(SENSOR_IOCTL_BASE, 6, short[3])
#define SENSOR_ACC_ENABLE_SELFTEST 	_IOW(SENSOR_IOCTL_BASE,13, int)
#define SENSOR_ACC_SELFTEST 		_IOR(SENSOR_IOCTL_BASE,14, int)


struct kxtj9_acc_data {
	s16 x;
	s16 y;
	s16 z;
};

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtj9_odr_table[] = {
	{ 3,	ODR800F },
	{ 5,	ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0,	ODR12_5F},
};

struct kxtj9_data {
	struct i2c_client *client;
	struct kxtj9_platform_data pdata;
	struct input_dev *input_dev;
	unsigned int last_poll_interval;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;

	struct miscdevice miscdev;
	char name[10];
};

static int kxtj9_i2c_read(struct kxtj9_data *tj9, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static int kxtj9_read_acceleration_data(struct kxtj9_data *tj9, struct kxtj9_acc_data *data)
{
	s16 acc_data[3]; /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;

	err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
	if (err < 0) {
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");
		return err;
	}

	x = le16_to_cpu(acc_data[tj9->pdata.axis_map_x]);
	y = le16_to_cpu(acc_data[tj9->pdata.axis_map_y]);
	z = le16_to_cpu(acc_data[tj9->pdata.axis_map_z]);

	x >>= tj9->shift;
	y >>= tj9->shift;
	z >>= tj9->shift;

	data->x = tj9->pdata.negate_x ? -x : x;
	data->y = tj9->pdata.negate_y ? -y : y;
	data->z = tj9->pdata.negate_z ? -z : z;

	return 0;
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9)
{
	struct kxtj9_acc_data data;
	int err;

	err = kxtj9_read_acceleration_data(tj9, &data);
	if (err < 0)
		return;

	input_report_abs(tj9->input_dev, ABS_X, data.x);
	input_report_abs(tj9->input_dev, ABS_Y, data.y);
	input_report_abs(tj9->input_dev, ABS_Z, data.z);
	input_sync(tj9->input_dev);
}

static irqreturn_t kxtj9_isr(int irq, void *dev)
{
	struct kxtj9_data *tj9 = dev;
	int err;

	/* data ready is the only possible interrupt type */
	kxtj9_report_acceleration_data(tj9);

	err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
	if (err < 0)
		dev_err(&tj9->client->dev,
			"error clearing interrupt status: %d\n", err);

	return IRQ_HANDLED;
}

static int kxtj9_update_g_range(struct kxtj9_data *tj9, u8 new_g_range)
{
	switch (new_g_range) {
	case KXTJ9_G_2G:
		tj9->shift = 4;
		break;
	case KXTJ9_G_4G:
		tj9->shift = 3;
		break;
	case KXTJ9_G_8G:
		tj9->shift = 2;
		break;
	default:
		return -EINVAL;
	}

	tj9->ctrl_reg1 &= 0xe7;
	tj9->ctrl_reg1 |= new_g_range;

	return 0;
}

static int kxtj9_update_odr(struct kxtj9_data *tj9, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtj9_odr_table); i++) {
		tj9->data_ctrl = kxtj9_odr_table[i].mask;
		if (poll_interval < kxtj9_odr_table[i].cutoff)
			break;
	}

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

static int kxtj9_device_power_on(struct kxtj9_data *tj9)
{
	if (tj9->pdata.power_on)
		return tj9->pdata.power_on();

	return 0;
}

static void kxtj9_device_power_off(struct kxtj9_data *tj9)
{
	int err;

	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		dev_dbg(&tj9->client->dev, "soft power off failed\n");

	if (tj9->pdata.power_off)
		tj9->pdata.power_off();
}

static int kxtj9_enable(struct kxtj9_data *tj9)
{
	int err;

	err = kxtj9_device_power_on(tj9);
	if (err < 0)
		return err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (tj9->client->irq) {
		err = i2c_smbus_write_byte_data(tj9->client,
						INT_CTRL1, tj9->int_ctrl);
		if (err < 0)
			return err;
	}

	err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);
	if (err < 0)
		return err;

	/* turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	err = kxtj9_update_odr(tj9, tj9->last_poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
	if (tj9->client->irq) {
		err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
		if (err < 0) {
			dev_err(&tj9->client->dev,
				"error clearing interrupt: %d\n", err);
			goto fail;
		}
	}

	return 0;

fail:
	kxtj9_device_power_off(tj9);
	return err;
}

static void kxtj9_disable(struct kxtj9_data *tj9)
{
	kxtj9_device_power_off(tj9);
}

static int kxtj9_input_open(struct input_dev *input)
{
	struct kxtj9_data *tj9 = input_get_drvdata(input);

	return kxtj9_enable(tj9);
}

static void kxtj9_input_close(struct input_dev *dev)
{
	struct kxtj9_data *tj9 = input_get_drvdata(dev);

	kxtj9_disable(tj9);
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj9_get_poll(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", tj9->last_poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj9_set_poll(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	disable_irq(client->irq);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	tj9->last_poll_interval = max(interval, tj9->pdata.min_interval);

	kxtj9_update_odr(tj9, tj9->last_poll_interval);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, kxtj9_get_poll, kxtj9_set_poll);

static struct attribute *kxtj9_attributes[] = {
	&dev_attr_poll.attr,
	NULL
};

static struct attribute_group kxtj9_attribute_group = {
	.attrs = kxtj9_attributes
};

static void kxtj9_poll(struct input_dev *input)
{
	struct kxtj9_data *tj9 = input_get_drvdata(input);
	unsigned int poll_interval = input_get_poll_interval(input);

	kxtj9_report_acceleration_data(tj9);

	if (poll_interval != tj9->last_poll_interval) {
		kxtj9_update_odr(tj9, poll_interval);
		tj9->last_poll_interval = poll_interval;
	}
}

static void kxtj9_platform_exit(void *data)
{
	struct kxtj9_data *tj9 = data;

	if (tj9->pdata.exit)
		tj9->pdata.exit();
}

static int kxtj9_verify(struct kxtj9_data *tj9)
{
	int retval;

	retval = kxtj9_device_power_on(tj9);
	if (retval < 0)
		return retval;

	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	if (retval < 0) {
		dev_dbg(&tj9->client->dev, "read err int source (%d)\n", retval);
		goto out;
	}
	dev_dbg(&tj9->client->dev, "who am i 0x%x\n", retval);

	retval = (retval != 0x07 && retval != 0x08 && retval != 0x35) ? -EIO : 0;

out:
	kxtj9_device_power_off(tj9);
	return retval;
}

static int kxtj9_set_selftest(struct kxtj9_data *tj9, int enable)
{
	struct i2c_client *client = tj9->client;
	int ret;

	/* Ensure that PC1 is cleared before updating control registers */
	ret = i2c_smbus_write_byte_data(client, CTRL_REG1, 0);
	if (ret < 0)
		return ret;

	if (enable) {
		/* Set STPOL - self test polarity and enable self test*/
		tj9->int_ctrl |= STPOL;
		ret = i2c_smbus_write_byte_data(client, INT_CTRL1, tj9->int_ctrl);
		if (ret < 0)
			return ret;

		ret = i2c_smbus_write_byte_data(client, SELF_TEST, 0xCA);
		if (ret < 0)
			return ret;
	}
	else {
		/* Disable self test and reset STPOL*/
		ret = i2c_smbus_write_byte_data(client, SELF_TEST, 0x00);
		if (ret < 0)
			return ret;

		tj9->int_ctrl &= ~STPOL;
		ret = i2c_smbus_write_byte_data(client, INT_CTRL1, tj9->int_ctrl);
		if (ret < 0)
			return ret;
	}

	/* Turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	ret = i2c_smbus_write_byte_data(client, CTRL_REG1, tj9->ctrl_reg1);
	if (ret < 0)
		return ret;

	return 0;
}

static int kxtj9_selftest(struct kxtj9_data *tj9)
{
	int ret;
	struct kxtj9_acc_data selftestdata;
	struct kxtj9_acc_data normaldata;
	struct kxtj9_acc_data testdiff;
	unsigned int half_g, max_diff, min_diff;

	switch (tj9->pdata.g_range) {
		case KXTJ9_G_2G:
			half_g = 512;
			break;
		case KXTJ9_G_4G:
			half_g = 256;
			break;
		case KXTJ9_G_8G:
		default:
			half_g = 128;
			break;
	}

	ret = kxtj9_read_acceleration_data(tj9, &normaldata);
	if (ret < 0) {
		dev_err(&tj9->client->dev, "failed to read test acceleration data (%d)\n", ret);
		return ret;
	}

	ret = kxtj9_set_selftest(tj9, 1);
	if (ret < 0) {
		dev_err(&tj9->client->dev, "failed to activate selftest (%d)\n", ret);
		return ret;
	}
	msleep(tj9->last_poll_interval);

	ret = kxtj9_read_acceleration_data(tj9, &selftestdata);
	if (ret < 0) {
		dev_err(&tj9->client->dev, "failed to read test acceleration data (%d)\n", ret);
		return ret;
	}

	ret = kxtj9_set_selftest(tj9, 0);
	if (ret < 0) {
		dev_err(&tj9->client->dev, "failed to disable selftest (%d)\n", ret);
		return ret;
	}

	/* Use absolute values since any axis may be negated */
	testdiff.x = abs(selftestdata.x-normaldata.x);
	testdiff.y = abs(selftestdata.y-normaldata.y);
	testdiff.z = abs(selftestdata.z-normaldata.z);

	dev_dbg(&tj9->client->dev, "Self test: test acceleration data x=%d, y=%d, z=%d\n",
					selftestdata.x, selftestdata.y, selftestdata.z);
	dev_dbg(&tj9->client->dev, "Self test: norm acceleration data x=%d, y=%d, z=%d\n",
					normaldata.x, normaldata.y, normaldata.z);
	dev_dbg(&tj9->client->dev, "Self test:              diff data x=%d, y=%d, z=%d\n",
				testdiff.x, testdiff.y, testdiff.z);

	/* 
	* The data sheet is not very clear on what output interval that is OK
	* during the self test. It should be about 0.5g higher than normal
	* measurements. Practical measurements have shown that an acceptance 
	* interval [g/2-g/8, g/2+g/8] works.
	*/
	max_diff = half_g + half_g/4;
	min_diff = half_g - half_g/4;
	if (testdiff.x > max_diff || testdiff.x < min_diff || \
		testdiff.y > max_diff || testdiff.y < min_diff || \
		testdiff.z > max_diff || testdiff.z < min_diff) {
		dev_err(&tj9->client->dev, "failed selftest (%d,%d,%d), 0.5g=%d\n", \
			testdiff.x, testdiff.y, testdiff.z, half_g);
		return -EINVAL;
	}
	return 0;
}

static long kxtj9_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *mdev = (struct miscdevice *)file->private_data;
	struct kxtj9_data *tj9;
	void __user *argp = (void __user *)arg;
	struct kxtj9_acc_data data;
	long ret = 0;
	short sdata[3];
	int enable;

	tj9 = (struct kxtj9_data *) container_of(mdev, struct kxtj9_data, miscdev);
	if (!tj9) {
		printk(KERN_ERR "kxtj9_data struct is NULL.");
		return -EFAULT;
	}

	switch (cmd) {
	case SENSOR_GET_MODEL_NAME:
		if (copy_to_user(argp, "KIONIX ACC", strlen("KIONIX ACC") + 1)) {
			printk(KERN_ERR "SENSOR_GET_MODEL_NAME copy_to_user failed.");
			ret = -EFAULT;
		}
		break;
	case SENSOR_SET_POWER_STATUS:
		if (copy_from_user(&enable, argp, sizeof(int))) {
			printk(KERN_ERR "SENSOR_SET_POWER_STATUS copy_to_user failed.");
			ret = -EFAULT;
			break;
		}
		if (enable) {
			ret = kxtj9_enable(tj9);
			if (ret < 0)
				printk(KERN_ERR "SENSOR_SET_POWER_STATUS kxtj9_enable failed.");
		}
		else {
			kxtj9_disable(tj9);
		}
		break;
	case SENSOR_ACC_SELFTEST:
		ret = kxtj9_selftest(tj9);
		if (copy_to_user(argp, &ret, sizeof(ret))) {
			printk(KERN_ERR "SENSOR_ACC_SELFTEST copy_to_user failed.");
			ret = -EFAULT;
		}
		break;
	case SENSOR_GET_RAW_DATA:
		ret = kxtj9_read_acceleration_data(tj9, &data);
		if (!ret) {
			sdata[0] = data.x;
			sdata[1] = data.y;
			sdata[2] = data.z;
			if (copy_to_user(argp, sdata, sizeof(sdata))) {
				printk(KERN_ERR "SENSOR_GET_RAW_DATA copy_to_user failed.");
				ret = -EFAULT;
			}
		}
		break;
	default:
		ret = -1;
	}
	return ret;
}

static int kxtj9_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int kxtj9_release(struct inode *inode, struct file *file)
{
	/* FIXME */
	return 0;
}

static const struct file_operations kxtj9_fops = {
	.owner = THIS_MODULE,
	.open = kxtj9_open,
	.release = kxtj9_release,
	.unlocked_ioctl = kxtj9_ioctl,
};


#ifdef CONFIG_OF
static const struct of_device_id kxtj9_of_match[] = {
	{ .compatible = "kionix,kxtj9", },
	{ .compatible = "kionix,kxtj3", },
	{ }
};
MODULE_DEVICE_TABLE(of, kxtj9_of_match);

static struct kxtj9_platform_data *kxtj9_parse_dt(struct device *dev)
{
	const struct of_device_id *of_id =
				of_match_device(kxtj9_of_match, dev);
	struct device_node *np = dev->of_node;
	struct kxtj9_platform_data *pdata;
	u32 mapx = 0;
	u32 mapy = 1;
	u32 mapz = 2;

	if (!of_id || !np)
		return NULL;

	pdata = kzalloc(sizeof(struct kxtj9_platform_data),
			GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	of_property_read_u32(np, "min-interval", &pdata->min_interval);
	of_property_read_u32(np, "init-interval", &pdata->init_interval);

	of_property_read_u32(np, "axis-map-x", &mapx);
	pdata->axis_map_x = (u8) mapx;
	of_property_read_u32(np, "axis-map-y", &mapy);
	pdata->axis_map_y = (u8) mapy;
	of_property_read_u32(np, "axis-map-z", &mapz);
	pdata->axis_map_z = (u8) mapz;

	pdata->negate_x = of_property_read_bool(np, "negate-x");
	pdata->negate_y = of_property_read_bool(np, "negate-y");
	pdata->negate_z = of_property_read_bool(np, "negate-z");

	pdata->is_sensor_accel = of_property_read_bool(np, "is-sensor-accel");

	pdata->res_12bit = RES_12BIT;
	pdata->g_range = KXTJ9_G_2G;

	dev_dbg(dev, "axis-map-x is set to %d\n", pdata->axis_map_x);
	dev_dbg(dev, "axis-map-y is set to %d\n", pdata->axis_map_y);
	dev_dbg(dev, "axis-map-z is set to %d\n", pdata->axis_map_z);
	dev_dbg(dev, "negate_x is %s\n", pdata->negate_x ? "set" : "not set");
	dev_dbg(dev, "negate_y is %s\n", pdata->negate_y ? "set" : "not set");
	dev_dbg(dev, "negate_z is %s\n", pdata->negate_z ? "set" : "not set");
	dev_dbg(dev, "is_sensor_accel is %s\n", pdata->is_sensor_accel ? "set" : "not set");

	return pdata;
}
#else
static inline struct kxtj9_platform_data *
kxtj9_parse_dt(struct device *dev)
{
	return NULL;
}
#endif


static int kxtj9_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	const struct kxtj9_platform_data *pdata = dev_get_platdata(&client->dev);
	struct kxtj9_data *tj9;
	struct input_dev *input_dev;
	int err;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (!pdata) {
		pdata = kxtj9_parse_dt(&client->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);

		if (!pdata) {
			dev_err(&client->dev, "missing platform data\n");
			return -EINVAL;
		}
	}

	tj9 = devm_kzalloc(&client->dev, sizeof(*tj9), GFP_KERNEL);
	if (!tj9) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	tj9->client = client;
	tj9->pdata = *pdata;
	strcpy(tj9->name, NAME);

	if (pdata->init) {
		err = pdata->init();
		if (err < 0)
			return err;
	}

	err = devm_add_action_or_reset(&client->dev, kxtj9_platform_exit, tj9);
	if (err)
		return err;

	err = kxtj9_verify(tj9);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		return err;
	}

	if (pdata->is_sensor_accel)
		tj9->miscdev.name = "KionixSensorAccel";
	else
		tj9->miscdev.name = "KionixLCDAccel";

	tj9->miscdev.minor = MISC_DYNAMIC_MINOR;
	tj9->miscdev.fops = &kxtj9_fops;

	err = misc_register(&tj9->miscdev);
	if (err != 0) {
		dev_err(&client->dev, "register miscdevice error");
		return err;
	}
	i2c_set_clientdata(client, tj9);

	tj9->ctrl_reg1 = tj9->pdata.res_12bit | tj9->pdata.g_range;
	tj9->last_poll_interval = tj9->pdata.init_interval;

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev) {
		dev_err(&client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	input_set_drvdata(input_dev, tj9);
	tj9->input_dev = input_dev;

	if (pdata->is_sensor_accel)
		input_dev->name = "KionixSensorAccel";
	else
		input_dev->name = "KionixLCDAccel";

	input_dev->id.bustype = BUS_I2C;

	input_dev->open = kxtj9_input_open;
	input_dev->close = kxtj9_input_close;

	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	if (client->irq <= 0) {
		err = input_setup_polling(input_dev, kxtj9_poll);
		if (err)
			return err;
	}

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"unable to register input polled device %s: %d\n",
			input_dev->name, err);
		return err;
	}

	if (client->irq) {
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tj9->int_ctrl |= KXTJ9_IEN | KXTJ9_IEA | KXTJ9_IEL;
		tj9->ctrl_reg1 |= DRDYE;

		err = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, kxtj9_isr,
						IRQF_TRIGGER_RISING |
							IRQF_ONESHOT,
						"kxtj9-irq", tj9);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			return err;
		}

		err = devm_device_add_group(&client->dev,
					    &kxtj9_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			return err;
		}
	}

	return 0;
}

static int __maybe_unused kxtj9_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		kxtj9_disable(tj9);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int __maybe_unused kxtj9_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		kxtj9_enable(tj9);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);

static const struct i2c_device_id kxtj9_id[] = {
	{ NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, kxtj9_id);

static struct i2c_driver kxtj9_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
		.of_match_table = kxtj9_of_match,
		.pm	= &kxtj9_pm_ops,
	},
	.probe		= kxtj9_probe,
	.id_table	= kxtj9_id,
};

module_i2c_driver(kxtj9_driver);

MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
