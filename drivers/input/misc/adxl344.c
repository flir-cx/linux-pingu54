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

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/input/adxl344.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define ADXL344_NAME "adxl344"

/* IOCTL defines */
#define SENSOR_IOCTL_BASE 'S'
#define SENSOR_GET_MODEL_NAME _IOR(SENSOR_IOCTL_BASE, 0, char *)
#define SENSOR_GET_RAW_DATA _IOR(SENSOR_IOCTL_BASE, 6, short[3])
#define SENSOR_ACC_SELFTEST _IOR(SENSOR_IOCTL_BASE, 14, int)

/* ADXL345/6 Register Map */
#define DEVID 0x00 /* R   Device ID */
#define OFSX 0x1E /* R/W X-axis offset */
#define OFSY 0x1F /* R/W Y-axis offset */
#define OFSZ 0x20 /* R/W Z-axis offset */
#define ACT_INACT_CTL 0x27 /* R/W Axis enable control for activity and */
/* inactivity detection */
#define BW_RATE 0x2C /* R/W Data rate and power mode control */
#define POWER_CTL 0x2D /* R/W Power saving features control */
#define INT_ENABLE 0x2E /* R/W Interrupt enable control */
#define INT_MAP 0x2F /* R/W Interrupt mapping control */
#define DATA_FORMAT 0x31 /* R/W Data format control */
#define DATAX0 0x32 /* R   X-Axis Data 0 */
#define DATAX1 0x33 /* R   X-Axis Data 1 */
#define DATAY0 0x34 /* R   Y-Axis Data 0 */
#define DATAY1 0x35 /* R   Y-Axis Data 1 */
#define DATAZ0 0x36 /* R   Z-Axis Data 0 */
#define DATAZ1 0x37 /* R   Z-Axis Data 1 */
#define FIFO_CTL 0x38 /* R/W FIFO control */

/* DEVIDs */
#define ID_ADXL345 0xE5
#define ID_ADXL346 0xE6

/* POWER_CTL Bits */
#define PCTL_LINK (1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE (1 << 3)
#define PCTL_SLEEP (1 << 2)
#define PCTL_WAKEUP(x) ((x)&0x3)

/* DATA_FORMAT Bits */
#define SELF_TEST (1 << 7)
#define SPI (1 << 6)
#define INT_INVERT (1 << 5)
#define FULL_RES (1 << 3)
#define JUSTIFY (1 << 2)
#define RANGE(x) ((x)&0x3)
#define RANGE_PM_2g 0
#define RANGE_PM_4g 1
#define RANGE_PM_8g 2
#define RANGE_PM_16g 3

#define ADXL_FIFO_BYPASS 0

struct adxl344_acc_data {
	s16 x;
	s16 y;
	s16 z;
};

struct adxl344_bus_ops {
	u16 bustype;
	int (*read)(struct device *, unsigned char);
	int (*read_block)(struct device *, unsigned char, int, void *);
	int (*write)(struct device *, unsigned char, unsigned char);
};

struct adxl344_data {
	struct i2c_client *client;
	struct adxl344_platform_data pdata;
	struct miscdevice miscdev;
	unsigned model;
	char name[10];
};

static int adxl344_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);

static const struct of_device_id adxl344_of_match[] = {
	{
		.compatible = "flir,adxl344",
	},
	{}
};
MODULE_DEVICE_TABLE(of, adxl344_of_match);

static struct adxl344_platform_data *adxl344_parse_dt(struct device *dev)
{
	const struct of_device_id *of_id =
		of_match_device(adxl344_of_match, dev);
	struct device_node *np = dev->of_node;
	struct adxl344_platform_data *pdata;
	u32 mapx = 0;
	u32 mapy = 1;
	u32 mapz = 2;

	if (!of_id || !np)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(struct adxl344_platform_data),
			     GFP_KERNEL);

	if (!pdata)
		return ERR_PTR(-ENOMEM);

	of_property_read_u32(np, "axis-map-x", &mapx);
	pdata->axis_map_x = (u8)mapx;
	of_property_read_u32(np, "axis-map-y", &mapy);
	pdata->axis_map_y = (u8)mapy;
	of_property_read_u32(np, "axis-map-z", &mapz);
	pdata->axis_map_z = (u8)mapz;

	// Check if mapping is correct
	if (!((mapx * mapy * mapz) == 0 && (mapx + mapy + mapz) == 3)) {
		dev_err(dev, "x,y,z, mapping incorrect.");
		return ERR_PTR(-EINVAL);
	}

	pdata->negate_x = of_property_read_bool(np, "negate-x");
	pdata->negate_y = of_property_read_bool(np, "negate-y");
	pdata->negate_z = of_property_read_bool(np, "negate-z");

	pdata->is_sensor_accel = of_property_read_bool(np, "is-sensor-accel");

	dev_dbg(dev, "axis-map-x is set to %d\n", pdata->axis_map_x);
	dev_dbg(dev, "axis-map-y is set to %d\n", pdata->axis_map_y);
	dev_dbg(dev, "axis-map-z is set to %d\n", pdata->axis_map_z);
	dev_dbg(dev, "negate_x is %s\n", pdata->negate_x ? "set" : "not set");
	dev_dbg(dev, "negate_y is %s\n", pdata->negate_y ? "set" : "not set");
	dev_dbg(dev, "negate_z is %s\n", pdata->negate_z ? "set" : "not set");
	dev_dbg(dev, "is_sensor_accel is %s\n",
		pdata->is_sensor_accel ? "set" : "not set");

	return pdata;
}

static int adxl344_enable(struct i2c_client *client)
{
	return i2c_smbus_write_byte_data(client, POWER_CTL, PCTL_MEASURE);
}

static int adxl344_disable(struct i2c_client *client)
{
	return i2c_smbus_write_byte_data(client, POWER_CTL, 0);
}

static int adxl344_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int adxl344_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int adxl344_get_acc(struct adxl344_data *adxl344,
			   struct adxl344_acc_data *axis)
{
	s16 buf[3];
	s16 x, y, z;
	int ret = i2c_smbus_read_i2c_block_data(adxl344->client, DATAX0,
						DATAZ1 - DATAX0 + 1, (u8 *)buf);

	// Does axis remapping
	x = le16_to_cpu(buf[adxl344->pdata.axis_map_x]);
	y = le16_to_cpu(buf[adxl344->pdata.axis_map_y]);
	z = le16_to_cpu(buf[adxl344->pdata.axis_map_z]);

	// Does axis invert
	axis->x = adxl344->pdata.negate_x ? -x : x;
	axis->y = adxl344->pdata.negate_y ? -y : y;
	axis->z = adxl344->pdata.negate_z ? -z : z;

	dev_dbg(&adxl344->client->dev, " %d %d %d ", axis->x, axis->y, axis->z);

	return ret;
}

static int adxl344_selftest(struct adxl344_data *adxl344)
{
	int ret;
	struct adxl344_acc_data selftestdata;
	struct adxl344_acc_data normaldata;
	struct adxl344_acc_data testdiff;
	unsigned int half_g, max_diff, min_diff;

	// Get referencedata
	ret = adxl344_get_acc(adxl344, &normaldata);
	if (ret < 0) {
		dev_err(&adxl344->client->dev,
			"failed to read test acceleration data (%d)\n", ret);
		return ret;
	}

	// Enable self-test.
	i2c_smbus_write_byte_data(adxl344->client, DATA_FORMAT,
				  RANGE_PM_2g | SELF_TEST);

	// Minimum sleep is 4 times the sampling rate. Assuming 100hz this is 40ms
	msleep(40);

	// Get selftest-data
	ret = adxl344_get_acc(adxl344, &selftestdata);
	if (ret < 0) {
		dev_err(&adxl344->client->dev,
			"failed to read test acceleration data (%d)\n", ret);
		return ret;
	}

	// Disable selftest
	i2c_smbus_write_byte_data(adxl344->client, DATA_FORMAT, RANGE_PM_2g);

	/* Use absolute values since any axis may be negated */
	testdiff.x = abs(selftestdata.x - normaldata.x);
	testdiff.y = abs(selftestdata.y - normaldata.y);
	testdiff.z = abs(selftestdata.z - normaldata.z);

	dev_dbg(&adxl344->client->dev,
		"Self test: test acceleration data x=%d, y=%d, z=%d\n",
		selftestdata.x, selftestdata.y, selftestdata.z);
	dev_dbg(&adxl344->client->dev,
		"Self test: norm acceleration data x=%d, y=%d, z=%d\n",
		normaldata.x, normaldata.y, normaldata.z);
	dev_dbg(&adxl344->client->dev,
		"Self test:              diff data x=%d, y=%d, z=%d\n",
		testdiff.x, testdiff.y, testdiff.z);

	/* 
	* These values are picked from ADXL344A data shett table 15 page 19
	*/
	max_diff = 400; // Assuming 2g measurement range and 10 bit resolution.
	min_diff = 70;
	if (testdiff.x > max_diff || testdiff.x < min_diff ||
	    testdiff.y > max_diff || testdiff.y < min_diff ||
	    testdiff.z > max_diff || testdiff.z < min_diff) {
		dev_err(&adxl344->client->dev,
			"failed selftest (%d,%d,%d), 0.5g=%d\n", testdiff.x,
			testdiff.y, testdiff.z, half_g);
		return -EINVAL;
	}
	return 0;
}

static long adxl344_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct miscdevice *mdev = (struct miscdevice *)file->private_data;
	struct adxl344_data *adxl344;
	void __user *argp = (void __user *)arg;
	struct adxl344_acc_data data;
	long ret = 0;
	short sdata[3];

	adxl344 = (struct adxl344_data *)container_of(mdev, struct adxl344_data,
						      miscdev);
	if (!adxl344) {
		printk(KERN_ERR "adxl344 struct is NULL.");
		return -EFAULT;
	}

	switch (cmd) {
	case SENSOR_GET_MODEL_NAME:
		if (copy_to_user(argp, ADXL344_NAME,
				 strlen(ADXL344_NAME) + 1)) {
			dev_err(&adxl344->client->dev,
				"SENSOR_GET_MODEL_NAME copy_to_user failed.");
			ret = -EFAULT;
		}
		break;
	case SENSOR_ACC_SELFTEST: // 0x8004530e
		ret = adxl344_selftest(adxl344);
		if (copy_to_user(argp, &ret, sizeof(ret))) {
			dev_err(&adxl344->client->dev,
				"SENSOR_ACC_SELFTEST copy_to_user failed.");
			ret = -EFAULT;
		}
		break;
	case SENSOR_GET_RAW_DATA: // 0x80065306
		ret = adxl344_get_acc(adxl344, &data);
		if (ret) {
			sdata[0] = data.x;
			sdata[1] = data.y;
			sdata[2] = data.z;
			if (copy_to_user(argp, sdata, sizeof(sdata))) {
				dev_err(&adxl344->client->dev,
					"SENSOR_GET_RAW_DATA copy_to_user failed.");
				ret = -EFAULT;
			}
		}
		break;
	default:
		ret = -1;
	}
	return 0;
}

static const struct file_operations adxl344_fops = {
	.owner = THIS_MODULE,
	.open = adxl344_open,
	.release = adxl344_release,
	.unlocked_ioctl = adxl344_ioctl,
};

static int adxl344_setup(struct i2c_client *client)
{
	// Disable until configured
	i2c_smbus_write_byte_data(client, POWER_CTL, 0);

	// Clear offset calibration
	i2c_smbus_write_byte_data(client, OFSX, 0);
	i2c_smbus_write_byte_data(client, OFSY, 0);
	i2c_smbus_write_byte_data(client, OFSZ, 0);

	// Disable Inactivations
	i2c_smbus_write_byte_data(client, ACT_INACT_CTL, 0);

	// Set samplerate to 100Hz
	i2c_smbus_write_byte_data(client, BW_RATE, 0x0A);

	// Set range and resolution
	i2c_smbus_write_byte_data(client, DATA_FORMAT, RANGE_PM_2g);

	// Bypass fifo
	i2c_smbus_write_byte_data(client, FIFO_CTL, ADXL_FIFO_BYPASS);

	// Disable interrupts
	i2c_smbus_write_byte_data(client, INT_ENABLE, 0);
	i2c_smbus_write_byte_data(client, INT_MAP, 0);

	i2c_smbus_write_byte_data(client, POWER_CTL, PCTL_MEASURE);
	return 0;
}

static int adxl344_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	const struct adxl344_platform_data *pdata =
		dev_get_platdata(&client->dev);
	struct adxl344_data *adxl344;
	int err;
	s32 revid;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (!pdata) {
		pdata = adxl344_parse_dt(&client->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);

		if (!pdata) {
			dev_err(&client->dev, "missing platform data\n");
			return -EINVAL;
		}
	}

	adxl344 = devm_kzalloc(&client->dev, sizeof(*adxl344), GFP_KERNEL);
	if (!adxl344) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	adxl344->client = client;
	adxl344->pdata = *pdata;
	strcpy(adxl344->name, ADXL344_NAME);

	if (pdata->is_sensor_accel)
		adxl344->miscdev.name = "adxl344SensorAccel";
	else
		adxl344->miscdev.name = "adxl344LCDAccel";

	adxl344->miscdev.minor = MISC_DYNAMIC_MINOR;
	adxl344->miscdev.fops = &adxl344_fops;

	err = misc_register(&adxl344->miscdev);
	if (err != 0) {
		dev_err(&client->dev, "register miscdevice error");
		return err;
	}
	i2c_set_clientdata(client, adxl344);

	revid = i2c_smbus_read_byte_data(client, DEVID);

	switch (revid) {
	case ID_ADXL345:
		adxl344->model = 345;
		break;
	case ID_ADXL346:
		adxl344->model = 346;
		break;
	default:
		dev_err(&client->dev, "Failed to probe %s\n",
			adxl344->miscdev.name);
		goto deregister_misc;
	}

	adxl344_setup(client);
	return err;

deregister_misc:
	misc_deregister(&adxl344->miscdev);
	return -ENODEV;
}

static int adxl344_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return adxl344_disable(client);
}

static int adxl344_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	return adxl344_enable(client);
}

static int adxl344_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id adxl344_id[] = {
	{ ADXL344_NAME, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, adxl344_id);

static SIMPLE_DEV_PM_OPS(adxl344_pm_ops, adxl344_suspend, adxl344_resume);

static struct i2c_driver adxl344_driver = {
	.driver = {
		.name	= ADXL344_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = adxl344_of_match,
		.pm	= &adxl344_pm_ops,
	},
	.probe		= adxl344_probe,
	.remove		= adxl344_remove,
	.id_table	= adxl344_id,
};

module_i2c_driver(adxl344_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FLIR Adxl344 accelerometer driver");
MODULE_AUTHOR("Olov Lejondahl");

//******************************************************************