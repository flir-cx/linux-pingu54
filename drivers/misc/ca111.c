/*
 * Driver for the CA111 Laser range finder
 *
 * Author: Bo Svangård <bobo@larven.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/ca111.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/kthread.h>

struct ca111_dev_info {
	struct i2c_client		*client;
	const struct i2c_device_id     *id;
	struct device			*dev;
	unsigned int			gpio_int;
	unsigned int			irq;
	struct mutex			mutex;
	int laserstatus;
	int checklaserstatus;
	struct work_struct isr_wq;
	struct work_struct input_wq;
	struct delayed_work check_laser_status_wq;
	int i2ccommand;
	unsigned long prvcmd_time;
};
struct device *dev;
struct input_dev *button_dev;
struct semaphore mr_sem;
//SYSFS interface
static DEVICE_ATTR(singlemeasurefast, S_IRUSR | S_IRGRP | S_IROTH, ca111_sysfs_singlemeasure_fast, NULL);
static DEVICE_ATTR(continousmeasurefast, S_IRUSR | S_IRGRP | S_IROTH, ca111_sysfs_continousmeasure_fast, NULL);
static DEVICE_ATTR(singlemeasurehigh, S_IRUSR | S_IRGRP | S_IROTH, ca111_sysfs_singlemeasure_high, NULL);
static DEVICE_ATTR(continousmeasurehigh, S_IRUSR | S_IRGRP | S_IROTH, ca111_sysfs_continousmeasure_high, NULL);
static DEVICE_ATTR(stopmeasure, S_IRUSR | S_IRGRP | S_IROTH, ca111_sysfs_stopmeasure, NULL);
static DEVICE_ATTR(status, S_IRUSR | S_IRGRP | S_IROTH, ca111_sysfs_status, NULL);
static DEVICE_ATTR(laser, S_IRUSR | S_IRGRP | S_IROTH, ca111_sysfs_laseron, NULL);


static struct attribute *ca111_sysfs_attrs[] = {
	&dev_attr_stopmeasure.attr,
	&dev_attr_status.attr,
	&dev_attr_singlemeasurefast.attr,
	&dev_attr_continousmeasurefast.attr,
	&dev_attr_singlemeasurehigh.attr,
	&dev_attr_continousmeasurehigh.attr,
	&dev_attr_laser.attr,
	NULL
};

static struct attribute_group ca111_sysfs_attr_grp = {
	.name = "control",
	.attrs = ca111_sysfs_attrs,
};

/*
 * This function services the GPIO interupt from the CA111
 */
irqreturn_t ca111_irq_handler(int irq, void *dev_id)
{
	struct ca111_dev_info *ca111_devinfo = dev_get_drvdata(dev);
	schedule_work(&ca111_devinfo->isr_wq);
	return IRQ_HANDLED;
}


void ca111_isr_wq (unsigned long unused){
	int ret;
	int length;
	int status;
	int laseron;
	struct ca111_dev_info *ca111_devinfo = dev_get_drvdata(dev);
	mutex_lock(&ca111_devinfo->mutex);
	ret = ca111_read_measurment(ca111_devinfo->client, &length, &status);
	if(length >= 50){
		input_report_abs(button_dev, ABS_DISTANCE, length);
	}

	laseron = ((status == CA111_LASERLIT) |
		   (status == CA111_ERROR_OUT_OF_RANGE) |
		   (status == CA111_ERROR_WEAK_SIGNAL)) ? 1:0;

	input_report_key(button_dev, KEY_ESC, laseron);
	ca111_devinfo->laserstatus = laseron;
	input_sync(button_dev);
	mutex_unlock(&ca111_devinfo->mutex);
}


void ca111_input_wq (struct work_struct *work){
	struct ca111_dev_info *ca111_devinfo = container_of(work, struct ca111_dev_info, input_wq);
	int length, status;
	int laseron;
	static int toggle=0;
	int i2ccommand= ca111_devinfo->i2ccommand;
	/* if ((ca111_devinfo->i2ccommand == CA111_REG_LASER_ON) ||  */
	/*     (ca111_devinfo->i2ccommand == CA111_REG_LASER_OFF)) { */
	/*   ca111_i2c_send_command(ca111_devinfo->client, CA111_REG_LASER_STATUS); */
	/*   ca111_read_measurment(ca111_devinfo->client, &length, &status); */
	/* } */

	mutex_lock(&ca111_devinfo->mutex);
	if(toggle == 0){
		if ((i2ccommand == CA111_REG_LASER_ON) ||
		    (i2ccommand == CA111_REG_LASER_OFF)) {
			toggle = 1;
		}

	} else {
		//toggle is 1...
		if ((i2ccommand == CA111_REG_LASER_ON) ||
		    (i2ccommand == CA111_REG_LASER_OFF)) {
			i2ccommand = CA111_REG_LASER_STATUS;
		}
		toggle = 0;
	} 

		ca111_i2c_send_command(ca111_devinfo->client, i2ccommand );
		ca111_read_measurment(ca111_devinfo->client, &length, &status);
			
		//report laser status through KEY_ESC
		laseron = ((status == CA111_LASERLIT) |
			   (status == CA111_ERROR_OUT_OF_RANGE) |
			   (status == CA111_ERROR_WEAK_SIGNAL)) ? 1:0;

		input_report_key(button_dev, KEY_ESC, laseron);
		ca111_devinfo->laserstatus = laseron;
		input_sync(button_dev);
		mutex_unlock(&ca111_devinfo->mutex);
}

void ca111_check_laser_status_wq (struct work_struct *work){
	struct ca111_dev_info *ca111_devinfo = dev_get_drvdata(dev);
	//dev_info(dev, "WQ: Checking laser status...\n");
	if(! ca111_devinfo->checklaserstatus){
		//dev_info(dev, "Expecting laser to be off\n");
		if(ca111_devinfo->laserstatus){
		  //dev_info(dev, "laser was on... turning it off...\n");
		  ca111_devinfo->i2ccommand = CA111_REG_LASER_OFF;
		  schedule_work(&ca111_devinfo->input_wq);
		}
	}
}

int button_event(struct input_dev *idev, unsigned int type, unsigned int code, int value)
{
	struct ca111_dev_info *ca111_devinfo = dev_get_drvdata(dev);

	//Button events
	// Use MSC_RAW button for LASER ON (value = 1) /
	//                           LASER OFF (value = 0)
	// Use MSC_PULSELED for High quality distance measurments
	// Use MSC_GESTURE for Low quality distance measurments
	// Laser off (value = 0), Single measurment (value = 1),
	// Continous measurments (value = 2)

	if(type == EV_MSC){
		dev_dbg(dev, "EV_MSC key detected, code is %i, value is %i\n", code,value);
		if(code == MSC_RAW){
			if(value == 1){
				dev_dbg(dev,"Laser On\n");

				ca111_devinfo->checklaserstatus = true;
				ca111_devinfo->i2ccommand = CA111_REG_LASER_ON;
			} else {
				dev_dbg(dev,"Laser Off\n");
				ca111_devinfo->checklaserstatus = false;
				ca111_devinfo->i2ccommand = CA111_REG_LASER_OFF;
			}
		} else if(code == MSC_PULSELED){
			if(value == 1){
				dev_dbg(dev, "Single measurment High quality\n");
				ca111_devinfo->checklaserstatus = true;
				ca111_devinfo->i2ccommand = CA111_REG_TRIGGER_SINGLE_HIGH;
			} else if(value){
				dev_dbg(dev, "Continous measurment high quality\n");
				ca111_devinfo->checklaserstatus = true;
				ca111_devinfo->i2ccommand = CA111_REG_TRIGGER_CONTINUOS_HIGH;
			} else {
				dev_dbg(dev, "Laser Off\n");
				ca111_devinfo->checklaserstatus = false;
				ca111_devinfo->i2ccommand = CA111_REG_LASER_OFF;
			}
		} else if(code == MSC_GESTURE){
			if(value == 1){
				dev_dbg(dev,"Single measurment Low quality\n");
				ca111_devinfo->checklaserstatus = true;
				ca111_devinfo->i2ccommand = CA111_REG_TRIGGER_SINGLE_LOW;
			} else if(value){
				dev_dbg(dev,"Continous measurment Low quality\n");
				ca111_devinfo->checklaserstatus = true;
				ca111_devinfo->i2ccommand = CA111_REG_TRIGGER_CONTINUOS_LOW;
			} else {
				dev_dbg(dev,"Laser Off\n");
				ca111_devinfo->checklaserstatus = false;
				ca111_devinfo->i2ccommand = CA111_REG_LASER_OFF;
			}
		} else {
			dev_err(dev, "Unknown input evdev code %i\n", code);
			return -1;
		}
		schedule_work(&ca111_devinfo->input_wq);

		//cancel previously defined wq, start new wq to check if laser is still lit
		if(ca111_devinfo->i2ccommand == CA111_REG_LASER_OFF){
			cancel_delayed_work(&ca111_devinfo->check_laser_status_wq);
			schedule_delayed_work(&ca111_devinfo->check_laser_status_wq, 5*HZ/10 );
		}
		return 0;
	} else {
		dev_err(dev, "Unknown event type %i\n", type);
	}
	return -1;
}
struct input_dev* ca111_get_input_dev(void)
{
    return button_dev;
}
EXPORT_SYMBOL(ca111_get_input_dev);

static ssize_t ca111_sysfs_continousmeasure_high(struct device* dev, struct device_attribute* attr, char* buf)
{
	input_event(button_dev, EV_MSC, MSC_PULSELED, 2);
	return 0;
}

static ssize_t ca111_sysfs_singlemeasure_high(struct device* dev, struct device_attribute* attr, char* buf)
{
	input_event(button_dev, EV_MSC, MSC_PULSELED, 1);
	return 0;
}


static ssize_t ca111_sysfs_continousmeasure_fast(struct device* dev, struct device_attribute* attr, char* buf)
{
	input_event(button_dev, EV_MSC, MSC_GESTURE, 2);
	return 0;
}



static ssize_t ca111_sysfs_singlemeasure_fast(struct device* dev, struct device_attribute* attr, char* buf)
{
	input_event(button_dev, EV_MSC, MSC_GESTURE, 1);
	return 0;
}

static ssize_t ca111_sysfs_stopmeasure(struct device* dev, struct device_attribute* attr, char* buf)
{
	input_event(button_dev, EV_MSC, MSC_RAW, 0);
	return 0;
}


/**
 * ca111_sysfs_status
 *
 * @param dev device driver structure
 * @param attr
 * @param buf
 *
 * @return 0
 * prints out (to pr_err) the laser status....
 */
static ssize_t ca111_sysfs_status(struct device* dev, struct device_attribute* attr, char* buf)
{
  char msg[10];
  int ret;
  int retval;
  struct ca111_dev_info *ca111_devinfo = dev_get_drvdata(dev);

  ca111_i2c_send_command(ca111_devinfo->client, CA111_REG_LASER_STATUS);
  ret = i2c_master_recv(ca111_devinfo->client, msg, 3);
  retval = msg[2];
   if(retval == 0xA){
     ret = sprintf(buf, "Laser is turned on\n");
  } else if (retval == 0xB){
     ret = sprintf(buf, "Laser is turned off\n");
  } else {
     ret = sprintf(buf, "Laser is in unknown state.. (retval %i)\n", retval);
  }
  return ret;
}


/**
 * ca111_sysfs_status
 *
 * @param dev device driver structure
 * @param attr
 * @param buf
 *
 * @return 1 laser is on
 *         0 laser is off
 *         -1  unknown
 *         -ENXIO failure, no laser
 */
int ca111_get_laserstatus()
{
  int retval=0;
  struct ca111_dev_info *ca111_devinfo = dev_get_drvdata(dev);

    if(ca111_devinfo){
	    mutex_lock(&ca111_devinfo->mutex);
	    dev_dbg(dev, "Laserstatus requested, sent %i\n",
		    ca111_devinfo->laserstatus);
	    retval = ca111_devinfo->laserstatus;
    mutex_unlock(&ca111_devinfo->mutex);
  }
  return retval;
}
EXPORT_SYMBOL(ca111_get_laserstatus);

static ssize_t ca111_sysfs_laseron(struct device* dev, struct device_attribute* attr, char* buf)
{
	input_event(button_dev, EV_MSC, MSC_RAW, 1);
	return 0;
}

static int ca111_i2c_send_command(struct i2c_client *client, char command)
{
	char msg[]={command};
	int tmp;

	//Limit command sending to module
	if (down_interruptible(&mr_sem)){
		dev_err(dev, "Error: could not get semaphore lock...\n");
		return -1;
	}

	tmp = i2c_master_send(client, msg, 1);

	if(command == CA111_REG_LASER_OFF){
	  //Make sure that laser is turned off!!
	  msleep(1000);
	  tmp = i2c_master_send(client, msg, 1);
	  msleep(400);
	  msg[0] = CA111_REG_LASER_STATUS;
	  tmp = i2c_master_send(client, msg, 1);
	  msleep(400);
	}

	up(&mr_sem);
	return tmp;
};

static int ca111_laser_off(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct ca111_dev_info *ca111_devinfo = dev_get_drvdata(dev);
	mutex_lock(&ca111_devinfo->mutex);
	ca111_devinfo->laserstatus = 0;
	ret = ca111_i2c_send_command(client, CA111_REG_LASER_OFF);
	mutex_unlock(&ca111_devinfo->mutex);
	return ret;

};

/**
 * ca111_read_measurment
 * Read three bytes from i2c bus
 *
 * @param client
 * @param id
 *
 * @return If unsuccesful read of i2c bus return negative error
 *         If succesful read of i2c bus, return error code from Ca111 read (byte 2)
 */
int ca111_read_measurment(struct i2c_client *client, int *length, int *status)
{
	char msg[10];
	int ret;
	int retval;
	ret = i2c_master_recv(client, msg, 3);
	if(ret != 3){
		retval = -1;
	} else{
		retval = msg[2];
	}
	*length = (msg[0] << 8) + msg[1];
	*status = msg[2];
	return retval;
};

struct regulator *regulator;

struct probe_data_struct {
	struct i2c_client *client;
	const struct i2c_device_id *id;
};

static int really_probe(void *void_data) {
	struct probe_data_struct *data = void_data;
	struct i2c_client *client = data->client;
	const struct i2c_device_id *id = data->id;
	struct ca111_dev_info *ca111_devinfo;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	usleep_range(2000,10000);

	dev_dbg(dev, "%s: Start\n", __func__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	sema_init(&mr_sem, 1);      /* usage count is 1 */

	ca111_devinfo = devm_kzalloc(dev, sizeof(*ca111_devinfo), GFP_KERNEL);
	if (!ca111_devinfo) {
		dev_err(dev, "Can't alloc ca111_devinfo struct\n");
		return -ENOMEM;
	}

	ca111_devinfo->laserstatus = 0;
	mutex_init(&ca111_devinfo->mutex);
	ca111_devinfo->id = id;
	ca111_devinfo->client = client;
	i2c_set_clientdata(client,ca111_devinfo);

	ret = sysfs_create_group(&dev->kobj, &ca111_sysfs_attr_grp);
	if(ret){
		dev_err(dev, "CA111 Error creating sysfs\n");
	}

	ret = ca111_laser_off(client,id);
	if(ret < 0){
		dev_err(dev, "laser off returned %i\n", ret);
		ret =  -EPROBE_DEFER;
		goto OUT1;
	};

	INIT_WORK(&ca111_devinfo->isr_wq, (void (*)(struct work_struct *)) ca111_isr_wq);
	INIT_WORK(&ca111_devinfo->input_wq, (void (*)(struct work_struct *)) ca111_input_wq);
	INIT_DELAYED_WORK(&ca111_devinfo->check_laser_status_wq, (void (*)(struct work_struct *)) ca111_check_laser_status_wq);
	ret = request_irq(gpio_to_irq(107),	//FIXME
			  ca111_irq_handler,	/* our handler */
		    IRQF_SHARED, "ca111_irq_handler",
		    (void *)(ca111_irq_handler));
	if(ret != 0){
		dev_err(dev, "%s: cannot register irq\n", __func__);
		goto OUT2;
	}

	button_dev = input_allocate_device();
	button_dev->name = "Laser distance meter";
	if (!button_dev) {
		dev_err(dev, "%s: Not enough memory\n", __func__);
		ret = -ENOMEM;
		goto OUT2;
	}

	button_dev->evbit[0] = (BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY) | BIT_MASK(EV_MSC));
	button_dev->keybit[BIT_WORD(ABS_DISTANCE)] = BIT_MASK(ABS_DISTANCE);
	button_dev->keybit[BIT_WORD(KEY_ESC)] = BIT_MASK(KEY_ESC);
	button_dev->mscbit[0] = BIT_MASK(MSC_RAW)|BIT_MASK(MSC_PULSELED)|BIT_MASK(MSC_GESTURE);
	input_set_abs_params(button_dev, ABS_DISTANCE, 0, 40000, 0, 0);
	button_dev->event = button_event;

	ret = input_register_device(button_dev);
	if (ret) {
		dev_err(dev, "Failed to register device\n");
		goto err_free_dev;
	}

	pr_info("ca111_probe success\n");

	return 0;
err_free_dev:
	input_free_device(button_dev);
OUT2:
	free_irq(gpio_to_irq(107), (void *)(ca111_irq_handler)); //FIXME


OUT1:
	sysfs_remove_group(&dev->kobj, &ca111_sysfs_attr_grp);
	ret |= regulator_disable(regulator);
	dev->driver = NULL;
	dev_set_drvdata(dev, NULL);

	pr_info("ca111_probe fail %d\n", ret);

	return 0;
}

static int ca111_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct probe_data_struct *data;
	struct task_struct *probe_task;
	int ret;

	dev = &client->dev;

	regulator = devm_regulator_get(&client->dev, "vin");
	if(IS_ERR(regulator)){
		if (PTR_ERR(regulator) != -EPROBE_DEFER)
			dev_err(dev, "Error getting regulator...\n");
		ret = PTR_ERR(regulator);
		regulator=NULL;
		return ret;
	}

	ret = regulator_enable(regulator);
	if(ret != 0){
		dev_err(dev, "Error enabling regulator... %i\n", ret);
		return ret;
	}

	data = devm_kzalloc (dev, sizeof(*data), GFP_KERNEL);
	data->client = client;
	data->id = id;

	probe_task = kthread_run(really_probe, data, "probe_ca111");
	if (IS_ERR(probe_task))
	    ret = PTR_ERR(probe_task);

	return ret;
}

static int ca111_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ca111_dev_info *ca111_devinfo = i2c_get_clientdata(client);
	regulator_disable(regulator);
	regulator_put(regulator);
	input_unregister_device(button_dev);
	ca111_laser_off(ca111_devinfo->client,ca111_devinfo->id);
	sysfs_remove_group(&dev->kobj, &ca111_sysfs_attr_grp);
	free_irq(gpio_to_irq(107), (void *)(ca111_irq_handler)); //FIXME
	return 0;
}

static const struct i2c_device_id ca111_i2c_ids[] = {
	{ "ca111", 0 },
	{ },
};

#ifdef CONFIG_OF
static const struct of_device_id ca111_of_match[] = {
	{ .compatible = "precaster,ca111", },
	{ },
};
MODULE_DEVICE_TABLE(of, ca111_of_match);
#else
static const struct of_device_id ca111_of_match[] = {
	{ },
};
#endif

static int ca111_pm_resume(struct device *dev)
{
	int ret;
	ret = regulator_enable(regulator);
	return ret;
}

static int ca111_pm_suspend(struct device *dev)
{
	int ret;
	ret = regulator_disable(regulator);
	return ret;
}

static const struct dev_pm_ops ca111_pm_ops = {
	.resume = ca111_pm_resume,
	.suspend = ca111_pm_suspend,
};

static void ca111_shutdown(struct i2c_client *client)
{
	regulator_disable(regulator);
}

static struct i2c_driver ca111_driver = {
	.probe		= ca111_probe,
	.remove		= ca111_remove,
	.id_table	= ca111_i2c_ids,
	.driver = {
		.name		= "ca111",
		.owner		= THIS_MODULE,
		.pm		= &ca111_pm_ops,
		.of_match_table	= of_match_ptr(ca111_of_match),
	},
	.shutdown = ca111_shutdown,
};
module_i2c_driver(ca111_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bo Svangård <bobo@larven.se>");
MODULE_ALIAS("i2c:ca111");
MODULE_DESCRIPTION("CA111 Laser Rangefinder");
