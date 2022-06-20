/*
 * Copyright (c) 2011 Bosch Sensortec GmbH
 * Copyright (c) 2011 Unixphere
 *
 * This driver adds support for Ublxo i2c gps
 *
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/max7.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/kfifo.h>
#include <asm/ioctls.h>
#include <linux/pm_runtime.h>
#include <linux/atomic.h>
#include <linux/kthread.h>

#define FIFO_SIZE 512

static DECLARE_KFIFO(i2cfifo, unsigned char, FIFO_SIZE);

#define MAX7_TIMEOUT 6000 /* Timeout in 6 seconds */
static DECLARE_WAIT_QUEUE_HEAD(wq);

/*Global Variable Declaration*/
static int max7_device_Open = 0; /* Is device open?  Used to prevent multiple
                                       access to the device */
static int max7_read_msg_stream(struct i2c_client *client, void *kbuf,
				int streamlen);
static int max7_read_msg_stream_len(struct i2c_client *client);
static int max7_runtime_resume(struct device *dev);
static int max7_runtime_suspend(struct device *dev);
static int max7_initialise(struct i2c_client *client);
static int max7_disable(bool runtime);
static int max7_enable(bool runtime);

struct max7_data *max7;
struct task_struct *thread;
/*ublox receivers DDC address*/
#define UBX_DDC_ADDR 0x42
#define UBX_CFG_ACK 0x01
#define UBX_CFG_NAK 0x00

/*ublox msg sync character*/
#define UBX_MSG_SYNC_CHAR_1 0xB5
#define UBX_MSG_SYNC_CHAR_2 0x62

/*NMEA msg class*/
#define NMEA_CLASS_STD_MSG 0XF0

/*NMEA msg ID*/
#define NMEA_CLASS_ID_GGA 0X00
#define NMEA_CLASS_ID_GLL 0X01
#define NMEA_CLASS_ID_GSA 0X02
#define NMEA_CLASS_ID_GSV 0X03
#define NMEA_CLASS_ID_RMC 0X04
#define NMEA_CLASS_ID_VTG 0X05
#define NMEA_CLASS_ID_ZDA 0x08

/*ublox msg class*/
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_INF 0x04
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A
#define UBX_CLASS_AID 0x0B
#define UBX_CLASS_TIM 0x0D
#define UBX_CLASS_LOG 0x21

/*ublox msg ID*/
#define UBX_MSG_ID_CFG_MSG 0X01
#define UBX_MSG_ID_CFG_INF 0X02
#define UBX_MSG_ID_CFG_RST 0X04
#define UBX_MSG_ID_CFG_CFG 0X09
#define UBX_MSG_ID_CFG_NAV5 0X24

/*Register Definition*/
#define UBX_MSG_STREAM_LEN_HIGH 0xFD
#define UBX_MSG_STREAM_LEN_LOW 0xFE
#define UBX_MSG_STREAM_START 0xFF

/*IOCTL Command list to share with user space*/
#define READ_RESPONSE 0x01
#define GPS_SOFT_RESET 0X02
#define GPS_HARD_RESET 0X03
#define READ_MSG_STREAM_LEN 0x04

#define GPSSIZE 2048

static irqreturn_t max7_isr(int irq, void *dev_id)
{
	struct i2c_client *client = max7->client;
	int streamlen = 0;
	int i;
	dev_dbg(&client->dev, "%s\n", __func__);
	streamlen = max7_read_msg_stream_len(client);
	if (streamlen > GPSSIZE) {
		streamlen = GPSSIZE;
	}

	if (streamlen > 0) {
		memset(max7->rdkbuf, 0, GPSSIZE);

		if (max7_read_msg_stream(client, max7->rdkbuf, streamlen) < 0) {
			dev_err(&client->dev,
				"Fail reading message stream..\n");
		}
		i = kfifo_in(&i2cfifo, max7->rdkbuf, strlen(max7->rdkbuf));
		if (i != strlen(max7->rdkbuf)) {
			dev_err(&client->dev,
				"failed to write to fifo, fifo full\n");
		}
	}

	return IRQ_HANDLED;
}

static int calculate_checksum(char *msg, int msg_len)
{
	u16 i;
	char *msg_ptr = NULL;
	u8 a = 0;
	u8 b = 0;

	u16 len = (4 + ((struct ubx_msg *)msg)->payload_len);
	// msg_class = 1 byt + msg_id = 1 byt + payload_len = 2 byt + payload

	msg_ptr = &((struct ubx_msg *)msg)->msg_class;
	for (i = 0; i < len; i++) {
		a = a + msg_ptr[i];
		b = b + a;
	}
	msg[msg_len - 2] = a;
	msg[msg_len - 1] = b;

	return 0;
}
static char *frame_ubx_msg(u8 class, u8 id, u16 len, void *payload, int msg_len)
{
	char *msg;
	char *ptr = NULL;

	msg = kzalloc(msg_len, GFP_KERNEL);
	if (!msg) {
		return 0;
	}
	memset(msg, 0, msg_len);
	((struct ubx_msg *)msg)->sync1 = UBX_MSG_SYNC_CHAR_1;
	((struct ubx_msg *)msg)->sync2 = UBX_MSG_SYNC_CHAR_2;
	((struct ubx_msg *)msg)->msg_class = class;
	((struct ubx_msg *)msg)->msg_id = id;
	((struct ubx_msg *)msg)->payload_len = len;
	//memcpy(&((struct ubx_msg *)msg)->payload, (char *)payload, len);
	memcpy((msg + 6), (char *)payload,
	       len); // sync1 = 1 byt + sync2 = 1 byt + msg_class = 1 byt + msg_id = 1 byt + payload_len = 2 byt
	calculate_checksum(msg, msg_len);

	ptr = (char *)msg;
	return msg;
}

static int max7_write(struct i2c_client *client, void *msg, int len)
{
	int ret = 0;
	u8 cmd = UBX_MSG_STREAM_START;
	u8 ubx_buf[500] = "";
	u8 i;
	u8 *pmsg = (u8 *)msg;

	//flush the stream before read
	ret = max7_read_msg_stream_len(client);
	if (ret > 500) {
		max7_read_msg_stream(client, ubx_buf, 500);
	} else if (ret > 0) {
		max7_read_msg_stream(client, ubx_buf, ret);
	}

	//flush the stream before read
	memset(ubx_buf, '\0', sizeof(ubx_buf));

	ubx_buf[0] = cmd;
	for (i = 0; i < len; i++) {
		ubx_buf[i + 1] = *pmsg++;
	}

	ret = i2c_master_send(client, ubx_buf, (len + 1));
	if (ret != (len + 1)) {
		dev_err(&client->dev, "Couldn't send I2C command. (%i)\n", ret);
		return ret;
	}

	return len;
}

static int max7_transfer(struct i2c_client *client, u8 cmd, void *buf, u16 len)
{
	struct i2c_msg msgs[] = { {
					  .addr = client->addr,
					  .flags = 0,
					  .len = 1,
					  .buf = &cmd,
				  },
				  {
					  .addr = client->addr,
					  .flags = I2C_M_RD,
					  .len = len,
					  .buf = buf,
				  } };

	if (i2c_transfer(client->adapter, msgs, 2) == 2) {
		return 0;
	}

	return -EIO;
}

static int max7_read_msg_stream(struct i2c_client *client, void *kbuf,
				int streamlen)
{
	u8 cmd = UBX_MSG_STREAM_START;

	if (max7_transfer(client, cmd, kbuf, streamlen) != 0) {
		dev_err(&client->dev, "%s: max7_transfer error\n", __func__);
		return -1;
	}
	return 0;
}

static int max7_read_msg_stream_len(struct i2c_client *client)
{
	u8 cmd = UBX_MSG_STREAM_LEN_HIGH;
	u8 buf[2];
	int streamlen = 0, retry = 0;

	while ((streamlen == 0) && (retry < 5)) {
		if (max7_transfer(client, cmd, buf, 2) != 0) {
			return -1;
		}
		streamlen = ((buf[0] << 8) | buf[1]);
		retry++;
	}
	return streamlen;
}

/**
 * FAD_IOControl
 *
 * @param filep
 * @param cmd
 * @param arg
 *
 * @return
 */
static long max7_i2c_ioctl(struct file *filep, unsigned int cmd,
			   unsigned long arg)
{
	int ret = 0;
	int len;

	if (!access_ok((void *)arg, _IOC_SIZE(cmd)))
		return -EFAULT;

	switch (cmd) {
	case FIONREAD:
		len = kfifo_len(&i2cfifo);
		ret = __put_user(len, (int __user *)arg);
		break;

	default:
		return -ENOTTY;
	}

	return ret;
}

static int send_ublox_request(struct i2c_client *client, u8 msg_class,
			      u16 msg_ID, void *config_data, u16 len)
{
	char *msg = NULL;
	int msg_len = (8 + len);
	int ret;

	msg = frame_ubx_msg(msg_class, msg_ID, len, config_data, msg_len);

	ret = max7_write(client, msg, msg_len);
	if (ret < 0) {
		dev_info(&client->dev, "failed to send ublox request\n");
		kfree(msg);
		return ret;
	}
	kfree(msg);
	return 1;
}

static int process_ublox_ack(struct i2c_client *client, u8 msg_class,
			     u16 msg_ID)
{
	struct ubx_ack_nak ubx_res;
	int streamlen = 0;

	streamlen = max7_read_msg_stream_len(client);
	if (streamlen <= 0) {
		return -1;
	}
	if (streamlen > sizeof(ubx_res))
		streamlen = sizeof(ubx_res);
	if (max7_read_msg_stream(client, &ubx_res, streamlen) < 0) {
		dev_err(&client->dev, "%s: max7_read_msg_stream error\n",
			__func__);
		return -1;
	}

	if ((ubx_res.msg_class == UBX_CLASS_ACK) &&
	    (ubx_res.clsID == msg_class) && (ubx_res.msgID == msg_ID)) {
		if (ubx_res.ack_nak == UBX_CFG_ACK) {
			return UBX_CFG_ACK;
		} else if (ubx_res.ack_nak == UBX_CFG_NAK) {
			return UBX_CFG_NAK;
		} else
			return -1;
	} else {
		dev_err(&client->dev, "Invalid Response from ublox\n");
		return -1;
	}
}

static int max7_i2c_open(struct inode *inode, struct file *file)
{
	struct i2c_client *client = max7->client;
	pm_runtime_get(&client->dev);
	if (max7_device_Open) {
		dev_err(&client->dev, "%s:Device Busy \n", __func__);
		return -EBUSY;
	} else {
		max7_device_Open++;
	}
	max7_runtime_resume(&client->dev);
	max7_initialise(client);
	enable_irq(gpio_to_irq(max7->irqpin));
	return 0;
}

static int max7_i2c_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = max7->client;
	if (max7_device_Open) {
		max7_device_Open--;
	}
	disable_irq(gpio_to_irq(max7->irqpin));
	wake_up_interruptible(&wq);
	pm_runtime_put(&client->dev);
	pm_runtime_suspend(&client->dev);
	return 0;
}

ssize_t max7_i2c_write(struct file *file, const char __user *buf, size_t count,
		       loff_t *p_off)
{
	struct i2c_client *client = max7->client;
	unsigned char *kbuf = NULL;
	int ret;

	kbuf = kzalloc(count, GFP_KERNEL);
	if (!kbuf) {
		ret = -ENOMEM;
		goto out1;
	}

	if (copy_from_user(kbuf, (void __user *)buf, count)) {
		dev_err(&client->dev, "can't copy %zd bytes to kbuf\n", count);
		ret = -EFAULT;
		goto out2;
	}

	/* //Only allow UBX messages */
	/* if(kbuf[0] != 0xb5) */
	/* { */
	/*   ret = -EPERM; */
	/*   goto out2; */
	/* } */

	ret = max7_write(client, kbuf, count);

out2:
	kfree(kbuf);
out1:
	return ret;
}

static int max7_read_buffer(void *length)
{
	size_t *len = (size_t *)length;

	while (*len == 0 && !kthread_should_stop()) {
		usleep_range(10, 20);
		*len = kfifo_len(&i2cfifo);
	}
	wake_up(&wq);
	return 0;
}

ssize_t max7_i2c_read(struct file *file, char __user *buf, size_t count,
		      loff_t *p_off)
{
	size_t len = 0;
	unsigned int copied;
	long ret;
	struct i2c_client *client = max7->client;

	thread = kthread_run(max7_read_buffer, &len, "max7");
	if (IS_ERR(thread)) {
		dev_err(&client->dev, "Could not start the kernel thread.\n");
		return 0;
	}

	ret = wait_event_interruptible_timeout(wq, len != 0,
					       msecs_to_jiffies(MAX7_TIMEOUT));
	if (ret == -ERESTARTSYS) {
		dev_dbg(&client->dev, "GPS read interrupted\n");
		kthread_stop(thread);
		return ret;
	} else if (ret == 0) {
		dev_err(&client->dev, "GPS read timeout\n");
		kthread_stop(thread);
		return ret;
	} else if (ret < 0) {
		dev_err(&client->dev, "GPS read returned %ld\n", ret);
		kthread_stop(thread);
		return ret;
	}

	if (count < len)
		len = count;

	if (kfifo_to_user(&i2cfifo, buf, len, &copied)) {
		dev_err(&client->dev, "%s: can't copy %d bytes to buf\n",
			__func__, len);
		return -EFAULT;
	}
	return len;
}

static const struct file_operations max7_fops = {
	.owner = THIS_MODULE,
	.open = max7_i2c_open,
	.release = max7_i2c_release,
	.read = max7_i2c_read,
	.write = max7_i2c_write,
	.unlocked_ioctl = max7_i2c_ioctl,
};

static struct miscdevice max7_gps_miscdevice = { .minor = MISC_DYNAMIC_MINOR,
						 .name = "ttySC1",
						 .fops = &max7_fops };

static int max7_initialise(struct i2c_client *client)
{
	int ret;
	//Disable uart port
	char data_disableuart[] = { 0x01, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				    0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	char data_enablei2cirq[] = { 0x00, 0x00, 0x1b, 0x00, 0x84, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
				     0x07, 0x00, 0x02, 0x00, 0x00, 0x00 };
	//Enable enhanced timeout, (does not shutdown DDC port)

	ret = send_ublox_request(client, UBX_CLASS_CFG, 0, &data_disableuart,
				 sizeof(data_disableuart));
	if (ret < 0) {
		dev_info(&client->dev, "Failed to send disableuart request\n");
		goto out;
	}

	usleep_range(100, 200);
	/*Process the Ublox ACK / NAck message and print it*/
	ret = process_ublox_ack(client, UBX_CLASS_CFG, 0);
	if (ret < 0) {
		dev_info(&client->dev,
			 "Ublox Navigation Setting (UBX-CFG-...) Failed \n");
		goto out;
	}

	ret = send_ublox_request(client, UBX_CLASS_CFG, 0, &data_enablei2cirq,
				 sizeof(data_enablei2cirq));
	if (ret < 0) {
		dev_info(&client->dev, "Failed to send enablei2cirq request\n");
		goto out;
	}

	usleep_range(100, 200);
	/*Process the Ublox ACK / NAck message and print it*/
	ret = process_ublox_ack(client, UBX_CLASS_CFG, 0);
	if (ret < 0) {
		dev_info(&client->dev,
			 "Ublox Navigation Setting (UBX-CFG-...) Failed \n");
		goto out;
	}

out:
	return ret;
}

static int max7_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct device *dev = &client->dev;
	struct device_node *node;

	node = dev->of_node;
	if (!node) {
		return -ENODEV;
	}

	max7 = kzalloc(sizeof(struct max7_data), GFP_KERNEL);
	if (!max7) {
		return -ENOMEM;
	}

	max7->supply = devm_regulator_get(dev, "vin");
	if (IS_ERR(max7->supply)) {
		ret = PTR_ERR(max7->supply);
		if (ret != -EPROBE_DEFER) {
			dev_err(dev, "can't get regulator vin-supply (%i)",
				ret);
		}
		max7->supply = NULL;
		return ret;
	}

	ret = regulator_enable(max7->supply);

	max7->resetpin = of_get_named_gpio_flags(node, "reset-pin", 0, NULL);
	if (max7->resetpin >= 0) {
		if (gpio_is_valid(max7->resetpin)) {
			ret = devm_gpio_request_one(&client->dev,
						    max7->resetpin,
						    GPIOF_OUT_INIT_LOW,
						    "Ublox reset");

			if (ret) {
				dev_err(&client->dev,
					"Failed to request GPIO %d, error %d\n",
					max7->resetpin, ret);
				ret = -EINVAL;
				goto err_001;
			}
			gpio_direction_input(max7->resetpin);
		} else {
			dev_err(&client->dev, "Reset pin not valid\n");
			ret = -EINVAL;
			goto err_001;
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c_check_functionality error\n");
		goto err_free_pdata;
	}

	max7->irqpin = of_get_named_gpio_flags(node, "interrupt-pin", 0, NULL);

	if (max7->irqpin < 0) {
		dev_err(dev, "Failed reading interrupt pin\n");
		ret = -EINVAL;
		goto err_free_pdata;
	};

	if (gpio_is_valid(max7->irqpin)) {
		ret = devm_gpio_request_one(&client->dev, max7->irqpin,
					    GPIOF_IN, "Ublox IRQ");
		if (ret) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				max7->irqpin, ret);
			goto err_01;
		}
	}

	//regulator has been enabled, sleep a while
	// for gps to become ready
	msleep(250);

	max7->client = client;
	ret = max7_initialise(client);
	if (ret < 0) {
		ret = -EPROBE_DEFER;
		goto err_0;
	}
	i2c_set_clientdata(client, max7);
	ret = misc_register(&max7_gps_miscdevice);
	if (ret < 0) {
		dev_err(dev, "Miscdevice register failed\n");
		goto err_0;
	}

	INIT_KFIFO(i2cfifo);

	max7->rdkbuf = kzalloc(GPSSIZE, GFP_KERNEL);
	if (!max7->rdkbuf) {
		dev_err(dev, "Failed allocating memory...\n");
		ret = -ENOMEM;
		goto err_1;
	}
	ret = devm_request_threaded_irq(&client->dev, gpio_to_irq(max7->irqpin),
					NULL, max7_isr,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					client->name, max7);
	if (ret) {
		dev_err(dev, "Unable to request IRQ.\n");
		goto err_2;
	}
	disable_irq(gpio_to_irq(max7->irqpin));

	dev_info(dev, "Enabling pm_runtime support\n");
	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, 2000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_suspend(&client->dev);

	//For some reason, the gps is not suspended on cold boots
	//despite pm_runtime_suspend being called above...
	//so let's use max7_runtime_susupend directly
	max7_runtime_suspend(&client->dev);
	goto err_out;

	devm_free_irq(&client->dev, gpio_to_irq(max7->irqpin), max7);

err_2:
	kfree(max7->rdkbuf);
	max7->rdkbuf = 0;
err_1:
	misc_deregister(&max7_gps_miscdevice);
err_0:
	devm_gpio_free(&client->dev, max7->irqpin);
err_01:
err_free_pdata:
	if (max7->resetpin >= 0) {
		devm_gpio_free(&client->dev, max7->resetpin);
	}
err_001:
	regulator_disable(max7->supply);
	kfree(max7);
	max7 = 0;
err_out:

	return ret;
}

static int max7_remove(struct i2c_client *client)
{
	devm_free_irq(&client->dev, gpio_to_irq(max7->irqpin), max7);
	devm_gpio_free(&client->dev, max7->irqpin);
	if (max7->resetpin >= 0) {
		gpio_direction_output(max7->resetpin, 0);
		devm_gpio_free(&client->dev, max7->resetpin);
	}
	kthread_stop(thread);
	kfree(max7->rdkbuf);
	max7->rdkbuf = 0;
	misc_deregister(&max7_gps_miscdevice);
	regulator_disable(max7->supply);
	kfree(max7);
	max7 = 0;
	return 0;
}

static const struct i2c_device_id max7_id[] = { { "max7", 0 }, {} };

static int max7_suspend(struct device *dev)
{
	return max7_disable(false);
}

static void max7_shutdown(struct i2c_client *client)
{
	max7_disable(false);
}

static int max7_runtime_suspend(struct device *dev)
{
	return max7_disable(true);
}

static int max7_disable(bool runtime)
{
	int ret = 0;
	if (atomic_read(&(max7->runtimesuspend)) == 0) {
		if (runtime) {
			atomic_set(&(max7->runtimesuspend), 1);
		}
		ret = regulator_disable(max7->supply);
		usleep_range(1000, 5000);
		if (max7->resetpin >= 0) {
			gpio_direction_output(max7->resetpin, 0);
		}
	}
	return ret;
}

static int max7_resume(struct device *dev)
{
	return max7_enable(false);
}

static int max7_runtime_resume(struct device *dev)
{
	return max7_enable(true);
}

static int max7_enable(bool runtime)
{
	int ret = 0;
	if (runtime) {
		if (atomic_read(&(max7->runtimesuspend)) == 1) {
			atomic_set(&(max7->runtimesuspend), 0);
			if (max7->resetpin >= 0) {
				gpio_direction_input(max7->resetpin);
			}
			ret = regulator_enable(max7->supply);
			usleep_range(200000, 300000);
		}
	}
	else {
		if(atomic_read(&(max7->runtimesuspend)) == 0){
			if (max7->resetpin >= 0) {
				gpio_direction_input(max7->resetpin);
			}
			ret = regulator_enable(max7->supply);
			usleep_range(200000, 300000);
		}
	}
	return ret;
}

static const struct dev_pm_ops max7_pmops = {
	.suspend = max7_suspend,
	.resume = max7_resume,
	.runtime_suspend = max7_runtime_suspend,
};

MODULE_DEVICE_TABLE(i2c, max7_id);
#ifdef CONFIG_OF
static const struct of_device_id ublox_of_match[] = {
	{
		.compatible = "ublox,m8",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ublox_of_match);
#else
static const struct of_device_id ublox_of_match[] = {
	{},
};
#endif

static struct i2c_driver max7_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = "max7",
		.of_match_table	= of_match_ptr(ublox_of_match),
		.pm	= &max7_pmops,
        },
        .id_table       = max7_id,
        .probe          = max7_probe,
        .remove         = max7_remove,
	.shutdown = max7_shutdown,
};

module_i2c_driver(max7_driver);

MODULE_AUTHOR("HCL Technologies");
MODULE_AUTHOR("Bo Svangård <bobo@larven.se>");
MODULE_DESCRIPTION("Max7 Driver");
MODULE_LICENSE("GPL");
