// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011 Kionix, Inc.
 * Copyright (C) 2021 TeledyneFlir
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input-polldev.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#define NAME			"bm1422"
/* OUTPUT REGISTERS */
#define WHO_AM_I		0x0F
/* INPUT_ABS CONSTANTS */
#define FUZZ			0	//10
#define FLAT			0	//3
#define MAG_MAX         8000

#define BM1422AGMV_DEVICE_ADDRESS_0E   (0x0E)	// 7bit Addrss
#define BM1422AGMV_DEVICE_ADDRESS_0F   (0x0F)	// 7bit Address
#define BM1422AGMV_WIA_VAL             (0x41)

#define BM1422AGMV_WIA                 (0x0F)
#define BM1422AGMV_DATAX               (0x10)
#define BM1422AGMV_DATAY               (0x12)
#define BM1422AGMV_DATAZ               (0x14)
#define BM1422AGMV_STA1                (0x18)
#define BM1422AGMV_CNTL1               (0x1B)
#define BM1422AGMV_CNTL2               (0x1C)
#define BM1422AGMV_CNTL3               (0x1D)
#define BM1422AGMV_AVE_A               (0x40)
#define BM1422AGMV_CNTL4               (0x5C)
#define BM1422AGMV_OFF_X               (0x6C)
#define BM1422AGMV_OFF_Y               (0x72)
#define BM1422AGMV_OFF_Z               (0x78)

#define BM1422AGMV_STA1_RD_DRDY        (1 << 6)

#define BM1422AGMV_CNTL1_FS1           (1 << 1)
#define BM1422AGMV_CNTL1_ODR_10Hz      (0 << 3)
#define BM1422AGMV_CNTL1_ODR_20Hz      (0x10)
#define BM1422AGMV_CNTL1_ODR_100Hz     (0x08)
#define BM1422AGMV_CNTL1_ODR_1kHz      (0x18)
#define BM1422AGMV_CNTL1_RST_LV        (1 << 5)
#define BM1422AGMV_CNTL1_OUT_BIT       (1 << 6)
#define BM1422AGMV_CNTL1_PC1           (1 << 7)

#define BM1422AGMV_CNTL2_DRP           (1 << 2)
#define BM1422AGMV_CNTL2_DREN          (1 << 3)

#define BM1422AGMV_CNTL3_FORCE         (1 << 6)

#define BM1422AGMV_AVE_A_AVE1          (1 << 2)
#define BM1422AGMV_AVE_A_AVE2          (2 << 2)
#define BM1422AGMV_AVE_A_AVE4          (0 << 2)
#define BM1422AGMV_AVE_A_AVE8          (3 << 2)
#define BM1422AGMV_AVE_A_AVE16         (4 << 2)

#define BM1422AGMV_14BIT_SENS          (24)
#define BM1422AGMV_12BIT_SENS          (6)

#define	SENSOR_IOCTL_BASE		'S'
#define	SENSOR_GET_MODEL_NAME		_IOR(SENSOR_IOCTL_BASE, 0, char *)
#define	SENSOR_GET_POWER_STATUS		_IOR(SENSOR_IOCTL_BASE, 2, int)
#define	SENSOR_SET_POWER_STATUS		_IOR(SENSOR_IOCTL_BASE, 3, int)
#define	SENSOR_GET_DELAY_TIME		_IOR(SENSOR_IOCTL_BASE, 4, int)
#define	SENSOR_SET_DELAY_TIME		_IOR(SENSOR_IOCTL_BASE, 5, int)
#define	SENSOR_GET_RAW_DATA		_IOR(SENSOR_IOCTL_BASE, 6, short[3])
#define SENSOR_ACC_ENABLE_SELFTEST	_IOW(SENSOR_IOCTL_BASE, 13, int)
#define SENSOR_ACC_SELFTEST		_IOR(SENSOR_IOCTL_BASE, 14, int)

struct bm1422_mag_data {
	s16 x;
	s16 y;
	s16 z;
};

struct bm1422_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_polled_dev *poll_dev;

	unsigned int axis_map_x;
	unsigned int axis_map_y;
	unsigned int axis_map_z;

	bool negate_x;
	bool negate_y;
	bool negate_z;

	unsigned int off_x;
	unsigned int off_y;
	unsigned int off_z;
	bool offsets_set;

	struct miscdevice miscdev;
	char name[20];
	int users;
	bool power;
};

static DEFINE_SPINLOCK(users_lock);

static int bm1422_power_on_and_start(struct bm1422_data *bm);
static int bm1422_power_off(struct bm1422_data *bm);

static int bm1422_i2c_read(struct bm1422_data *bm, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = bm->client->addr,
		 .flags = bm->client->flags,
		 .len = 1,
		 .buf = &addr,
		  },
		{
		 .addr = bm->client->addr,
		 .flags = bm->client->flags | I2C_M_RD,
		 .len = len,
		 .buf = data,
		  },
	};

	return i2c_transfer(bm->client->adapter, msgs, 2);
}

static int bm1422_read_magnetometer_data(struct bm1422_data *bm,
					 struct bm1422_mag_data *data)
{
	s16 mag_data[3];	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	s16 x, y, z;
	int err;
	u8 stat;

	/* Wait for RD_DRDY */
	stat = i2c_smbus_read_byte_data(bm->client, BM1422AGMV_STA1);
	while (stat == 0) {
		usleep_range(50, 70);
		stat = i2c_smbus_read_byte_data(bm->client, BM1422AGMV_STA1);
	}

	err = bm1422_i2c_read(bm, BM1422AGMV_DATAX, (u8 *) mag_data, 6);
	if (err < 0) {
		dev_err(&bm->client->dev, "magnetometer data read failed\n");
		return err;
	}

	x = le16_to_cpu(mag_data[bm->axis_map_x]);
	y = le16_to_cpu(mag_data[bm->axis_map_y]);
	z = le16_to_cpu(mag_data[bm->axis_map_z]);

	data->x = bm->negate_x ? -x : x;
	data->y = bm->negate_y ? -y : y;
	data->z = bm->negate_z ? -z : z;

	return 0;
}

static void bm1422_report_magnetometer_data(struct bm1422_data *bm)
{
	struct bm1422_mag_data data;
	int err;

	err = bm1422_read_magnetometer_data(bm, &data);
	if (err < 0)
		return;

	input_report_abs(bm->input_dev, ABS_X, data.x);
	input_report_abs(bm->input_dev, ABS_Y, data.y);
	input_report_abs(bm->input_dev, ABS_Z, data.z);
	input_sync(bm->input_dev);
}

static void bm1422_poll(struct input_polled_dev *dev)
{
	struct bm1422_data *bm = dev->private;

	bm1422_report_magnetometer_data(bm);
}

static void bm1422_init_input_device(struct bm1422_data *bm,
				     struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -MAG_MAX, MAG_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -MAG_MAX, MAG_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -MAG_MAX, MAG_MAX, FUZZ, FLAT);

	input_dev->name = "bm1422_mag";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &bm->client->dev;
}

static void bm1422_polled_input_open(struct input_polled_dev *dev)
{
	struct bm1422_data *bm = dev->private;
	spin_lock(&users_lock);
	bm->users++;
	spin_unlock(&users_lock);
}

static void bm1422_polled_input_close(struct input_polled_dev *dev)
{
	struct bm1422_data *bm = dev->private;
	spin_lock(&users_lock);
	bm->users--;
	spin_unlock(&users_lock);
}

static int bm1422_setup_polled_device(struct bm1422_data *bm)
{
	int err;
	struct input_polled_dev *poll_dev;

	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		dev_err(&bm->client->dev, "Failed to allocate polled device\n");
		return -ENOMEM;
	}

	bm->poll_dev = poll_dev;
	bm->input_dev = poll_dev->input;

	poll_dev->private = bm;
	poll_dev->poll = bm1422_poll;
	poll_dev->open = bm1422_polled_input_open;
	poll_dev->close = bm1422_polled_input_close;

	bm1422_init_input_device(bm, poll_dev->input);

	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&bm->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}

	return 0;
}

static void bm1422_teardown_polled_device(struct bm1422_data *bm)
{
	input_unregister_polled_device(bm->poll_dev);
	input_free_polled_device(bm->poll_dev);
}

static int bm1422_verify(struct bm1422_data *bm)
{
	int retval;

	retval = i2c_smbus_read_byte_data(bm->client, WHO_AM_I);
	if (retval < 0) {
		dev_dbg(&bm->client->dev, "read err WHO_AM_I (%d)\n", retval);
		return retval;
	}

	retval = (retval != BM1422AGMV_WIA_VAL) ? -EIO : 0;
	return retval;
}

static long bm1422_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *mdev = (struct miscdevice *)file->private_data;
	struct bm1422_data *bm;
	void __user *argp = (void __user *)arg;
	struct bm1422_mag_data data;
	long ret = 0;
	short sdata[3];
	int enable;

	bm = container_of(mdev, struct bm1422_data, miscdev);

	switch (cmd) {
	case SENSOR_GET_MODEL_NAME:
		if (copy_to_user(argp, bm->name, strlen(bm->name) + 1)) {
			dev_err(&bm->client->dev, "SENSOR_GET_MODEL_NAME copy_to_user failed.");
			ret = -EFAULT;
		}
		break;
	case SENSOR_SET_POWER_STATUS:
		if (copy_from_user(&enable, argp, sizeof(int))) {
			dev_err(&bm->client->dev, "SENSOR_SET_POWER_STATUS copy_to_user failed.");
			ret = -EFAULT;
		}

		if (enable) {
			if (!bm->power) {
				ret = bm1422_power_on_and_start(bm);
				bm->power = true;
			}
		} else {
			if (bm->power) {
				if (bm->users == 0) {
					ret = bm1422_power_off(bm);
					bm->power = false;
				} else if (bm->users < 0) {
					dev_err(&bm->client->dev, "bm1422 mismatch in SET_POWER_STATUS toggling, number of users less than zero...!!");
					ret = -EFAULT;
				}
			}
		}
		break;
	case SENSOR_GET_RAW_DATA:
		ret = bm1422_read_magnetometer_data(bm, &data);
		if (!ret) {
			sdata[0] = data.x;
			sdata[1] = data.y;
			sdata[2] = data.z;
			if (copy_to_user(argp, sdata, sizeof(sdata))) {
				dev_err(&bm->client->dev, "SENSOR_GET_RAW_DATA copy_to_user failed.");
				ret = -EFAULT;
			}
		}
		break;
	default:
		dev_err(&bm->client->dev, "Undefined ioctl call called for bm1422 driver");
		ret = -ENOIOCTLCMD;
	}
	return ret;
}

static int bm1422_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = (struct miscdevice *)file->private_data;
	struct bm1422_data *bm = container_of(mdev, struct bm1422_data, miscdev);

	spin_lock(&users_lock);
	bm->users++;
	spin_unlock(&users_lock);
	return nonseekable_open(inode, file);
}

static int bm1422_release(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = (struct miscdevice *)file->private_data;
	struct bm1422_data *bm = container_of(mdev, struct bm1422_data, miscdev);

	spin_lock(&users_lock);
	bm->users--;
	spin_unlock(&users_lock);
	return 0;
}

static const struct file_operations bm1422_fops = {
	.owner = THIS_MODULE,
	.open = bm1422_open,
	.release = bm1422_release,
	.unlocked_ioctl = bm1422_ioctl,
};

#ifdef CONFIG_OF
static const struct of_device_id bm1422_of_match[] = {
	{.compatible = "rohm,bm1422", },
	{ }
};

MODULE_DEVICE_TABLE(of, bm1422_of_match);

static int bm1422_parse_dt(struct device *dev, struct bm1422_data *bm)
{
	const struct of_device_id *of_id =
	    of_match_device(bm1422_of_match, dev);
	struct device_node *np = dev->of_node;
	unsigned int mapx = 0;
	unsigned int mapy = 1;
	unsigned int mapz = 2;

	if (!of_id || !np)
		return -EIO;

	of_property_read_u32(np, "axis-map-x", &mapx);
	bm->axis_map_x = (u8) mapx;
	of_property_read_u32(np, "axis-map-y", &mapy);
	bm->axis_map_y = (u8) mapy;
	of_property_read_u32(np, "axis-map-z", &mapz);
	bm->axis_map_z = (u8) mapz;

	bm->negate_x = of_property_read_bool(np, "negate-x");
	bm->negate_y = of_property_read_bool(np, "negate-y");
	bm->negate_z = of_property_read_bool(np, "negate-z");

	dev_dbg(dev, "axis-map-x is set to %d\n", bm->axis_map_x);
	dev_dbg(dev, "axis-map-y is set to %d\n", bm->axis_map_y);
	dev_dbg(dev, "axis-map-z is set to %d\n", bm->axis_map_z);
	dev_dbg(dev, "negate_x is %s\n", bm->negate_x ? "set" : "not set");
	dev_dbg(dev, "negate_y is %s\n", bm->negate_y ? "set" : "not set");
	dev_dbg(dev, "negate_z is %s\n", bm->negate_z ? "set" : "not set");

	return 0;
}
#else
static inline void bm1422_parse_dt(struct device *dev, struct bm1422_data *bm)
{
	return 0;
}
#endif

static int bm1422_axis_offset(struct bm1422_data *bm,
			      u8 data_reg, u8 offset_reg, unsigned int *offset)
{
	int retval = 0;
	s16 wk_dat = 1;
	u16 diff_axis = 9999;
	u8 reg;

	/* Step 0 - Power off reset with default CNTL1 setting 0x22 */
	reg = 0;
	reg |= BM1422AGMV_CNTL1_RST_LV;
	reg |= BM1422AGMV_CNTL1_FS1;
	i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL1, reg);
	dev_dbg(&bm->client->dev, "Step 0: Wrote 0x%x to CNTL1\n", reg);
	usleep_range(1000, 1200);

	/* Step 1 */
	reg = 0;
	reg |= BM1422AGMV_CNTL1_PC1;	// Power control
	reg |= BM1422AGMV_CNTL1_OUT_BIT;	// 14 bit resolution
	reg |= BM1422AGMV_CNTL1_FS1;	// Single mode measurement
	// BM1422AGMV_CNTL1_RST_LV=0 -> Logic reset control, reset release
	retval = i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL1, reg);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing CNTL1\n");
		return retval;
	}
	dev_dbg(&bm->client->dev, "Step 1: Wrote 0x%x to CNTL1\n", reg);

	/* Delay here since that is what they do in example code */
	usleep_range(1000, 1200);

	/* Write to RSTB_LV. Just write anything to RSTB_LV.
	 * I have tried 0/1/0x0101 with the same results.
	 */
	retval =
	    i2c_smbus_write_word_data(bm->client, BM1422AGMV_CNTL4,
				      cpu_to_le16((s16) 0));
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing CNTL4\n");
		return retval;
	}

	usleep_range(1000, 1200);

	/* Step 2 */
	retval = i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL2,
					   BM1422AGMV_CNTL2_DREN);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing DREN to CNTL2\n");
		return retval;
	}

	while (wk_dat < 96) {
		s16 mag_data;
		s16 data, abs_data;
		s32 stat;

		/* Step 3 */
		retval =
		    i2c_smbus_write_word_data(bm->client, offset_reg,
					      cpu_to_le16(wk_dat));
		if (retval < 0) {
			dev_err(&bm->client->dev, "Error writing offset reg\n");
			return retval;
		}

		/* Step 4 */
		retval =
		    i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL3,
					      BM1422AGMV_CNTL3_FORCE);
		if (retval < 0) {
			dev_err(&bm->client->dev, "Error writing CNTL3 reg\n");
			return retval;
		}

		/* Wait for DRDY in STA1 */
		stat = i2c_smbus_read_byte_data(bm->client, BM1422AGMV_STA1);
		if (stat < 0) {
			dev_err(&bm->client->dev, "Error %d reading STA1 reg\n",
				stat);
			return stat;
		}
		while (stat == 0) {
			usleep_range(50, 200);
			stat =
			    i2c_smbus_read_byte_data(bm->client,
						     BM1422AGMV_STA1);
			if (stat < 0) {
				dev_err(&bm->client->dev,
					"Error %d reading STA1 reg\n", stat);
				return stat;
			}
		}

		/* Step 5 */
		retval = bm1422_i2c_read(bm, data_reg, (u8 *) &mag_data, 2);
		data = le16_to_cpu(mag_data);

		abs_data = (data > 0 ? data : -data);
		if (diff_axis > abs_data) {
			*offset = wk_dat;
			diff_axis = abs_data;
		}

		wk_dat++;
	}

	i2c_smbus_write_word_data(bm->client, offset_reg, cpu_to_le16(*offset));

	return 0;
}

static int bm1422_compute_offsets(struct bm1422_data *bm)
{
	int retval = 0;

	/* Skip offset calculation for now */
	return retval;

	retval = bm1422_axis_offset(bm, BM1422AGMV_DATAX,
				    BM1422AGMV_OFF_X, &bm->off_x);
	if (retval < 0) {
		dev_err(&bm->client->dev,
			"Error computing offset for X axis\n");
		return retval;
	}

	dev_dbg(&bm->client->dev, "New x offset is %d (%x)\n", bm->off_x, bm->off_x);

	retval = bm1422_axis_offset(bm, BM1422AGMV_DATAY,
				    BM1422AGMV_OFF_Y, &bm->off_y);
	if (retval < 0) {
		dev_err(&bm->client->dev,
			"Error computing offset for Y axis\n");
		return retval;
	}

	dev_dbg(&bm->client->dev, "New y offset is %d (%x)\n", bm->off_y, bm->off_y);

	retval = bm1422_axis_offset(bm, BM1422AGMV_DATAZ,
				    BM1422AGMV_OFF_Z, &bm->off_z);
	if (retval < 0) {
		dev_err(&bm->client->dev,
			"Error computing offset for Z axis\n");
		return retval;
	}

	dev_dbg(&bm->client->dev, "New z offset is %d (%x)\n", bm->off_z, bm->off_z);

	bm->offsets_set = true;
	return 0;
}

//TODO: static int bm1422_power_set(struct bm1422_data *bm, int enable)
static int bm1422_power_on_and_start(struct bm1422_data *bm)
{
	int retval = 0;

	/* Step 1 */
	/* 14 bit output, Power control, Cont mode with FS1=0 */
	retval = i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL1,
					   BM1422AGMV_CNTL1_OUT_BIT |
					   BM1422AGMV_CNTL1_PC1 |
					   BM1422AGMV_CNTL1_ODR_20Hz);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing CNTL1\n");
		return retval;
	}

	/* Delay here since that is what they do in example code */
	usleep_range(1000, 1200);

	/* Set RSTB_LV = 0 */
	retval = i2c_smbus_write_word_data(bm->client, BM1422AGMV_CNTL4, 0);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing CNTL4\n");
		return retval;
	}

	/* Step 2 */
	retval = i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL2,
					   BM1422AGMV_CNTL2_DREN);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing CNTL2\n");
		return retval;
	}

	/* Average X times */
	retval = i2c_smbus_write_byte_data(bm->client, BM1422AGMV_AVE_A,
					   BM1422AGMV_AVE_A_AVE4);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing AVE_A\n");
		return retval;
	}

	/* Step 3 */
	/* Set offsets */
	if (bm->offsets_set) {
		retval = i2c_smbus_write_word_data(bm->client, BM1422AGMV_OFF_X,
						   cpu_to_le16(bm->off_x));
		if (retval < 0) {
			dev_err(&bm->client->dev, "Error writing OFF_X\n");
			return retval;
		}

		retval = i2c_smbus_write_word_data(bm->client, BM1422AGMV_OFF_Y,
						   cpu_to_le16(bm->off_y));
		if (retval < 0) {
			dev_err(&bm->client->dev, "Error writing OFF_Y\n");
			return retval;
		}

		retval = i2c_smbus_write_word_data(bm->client, BM1422AGMV_OFF_Z,
						   cpu_to_le16(bm->off_z));
		if (retval < 0) {
			dev_err(&bm->client->dev, "Error writing OFF_Z\n");
			return retval;
		}
	}

	/* Step 4 */
	retval = i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL3,
					   BM1422AGMV_CNTL3_FORCE);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error writing CNTL3\n");
		return retval;
	}

	return 0;
}

static int bm1422_power_off(struct bm1422_data *bm)
{
	int retval;
	u8 reg;

	/* Power off reset with default CNTL1 setting 0x22 */
	reg = BM1422AGMV_CNTL1_RST_LV | BM1422AGMV_CNTL1_FS1;
	retval = i2c_smbus_write_byte_data(bm->client, BM1422AGMV_CNTL1, reg);

	return retval;
}

static int bm1422_init(struct bm1422_data *bm)
{
	int retval;

	bm->off_x = 0;
	bm->off_y = 0;
	bm->off_z = 0;
	bm->offsets_set = false;

	/* Compute offsets for all axes */
	retval = bm1422_compute_offsets(bm);
	if (retval < 0)
		return retval;

	/* Step 0 - Power off first to ensure we are in a known state
	 * (after offset calculations)
	 */
	retval = bm1422_power_off(bm);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error powering off\n");
		return retval;
	}
	/* END of step 0 */

	usleep_range(1000, 1200);

	retval = bm1422_power_on_and_start(bm);
	if (retval < 0) {
		dev_err(&bm->client->dev, "Error powering on\n");
		return retval;
	}

	return 0;
}

static int bm1422_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	/*const struct bm1422_platform_data *pdata = dev_get_platdata(&client->dev); */
	struct bm1422_data *bm;
	int err;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	bm = kzalloc(sizeof(*bm), GFP_KERNEL);
	if (!bm)
		return -ENOMEM;

	err = bm1422_parse_dt(&client->dev, bm);
	if (err < 0) {
		dev_err(&client->dev, "parse dt failed %d\n", err);
		goto err_free_mem;
	}

	bm->client = client;
	strcpy(bm->name, "BM1422");

	err = bm1422_verify(bm);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_free_mem;
	}

	bm->miscdev.minor = MISC_DYNAMIC_MINOR;
	bm->miscdev.name = "RohmMagnetometer";
	bm->miscdev.fops = &bm1422_fops;

	err = misc_register(&bm->miscdev);
	if (err != 0) {
		dev_err(&client->dev, "register miscdevice error");
		goto err_free_mem;
	}

	i2c_set_clientdata(client, bm);

	bm1422_init(bm);

	err = bm1422_setup_polled_device(bm);
	if (err)
		goto err_free_mem;
	/* err = bm1422_create_sysfs_attributes(dev); */
	/* if (err) */
	/*	goto err_teardown_polled_device; */
	return err;

/* err_teardown_polled_device: */
/*	bm1422_teardown_polled_device(bm); */

err_free_mem:
	kfree(bm);
	return err;
}

static int bm1422_remove(struct i2c_client *client)
{
	struct bm1422_data *bm = i2c_get_clientdata(client);

	bm1422_teardown_polled_device(bm);

	kfree(bm);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bm1422_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bm1422_data *bm = i2c_get_clientdata(client);
	struct input_dev *input_dev = bm->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		bm1422_disable(bm);

	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int bm1422_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bm1422_data *bm = i2c_get_clientdata(client);
	struct input_dev *input_dev = bm->input_dev;
	int retval = 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		bm1422_enable(bm);

	mutex_unlock(&input_dev->mutex);
	return retval;
}
#endif

static SIMPLE_DEV_PM_OPS(bm1422_pm_ops, bm1422_suspend, bm1422_resume);

static const struct i2c_device_id bm1422_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, bm1422_id);

static struct i2c_driver bm1422_driver = {
	.driver = {
		   .name = NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = bm1422_of_match,
		   .pm = &bm1422_pm_ops,
		    },
	.probe = bm1422_probe,
	.remove = bm1422_remove,
	.id_table = bm1422_id,
};

module_i2c_driver(bm1422_driver);

MODULE_DESCRIPTION("BM1422AGMV magnetometer driver");
MODULE_AUTHOR("Ola Redell <ola.redell@teledyneflir.com>");
MODULE_LICENSE("GPL");
