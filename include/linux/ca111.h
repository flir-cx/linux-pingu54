/*
 * ca111 Laser Distance Mesure device driver
 *
 * Bo Svangård <bo.svangard@larven.se>
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef CA111_LDM_H
#define CA111_LDM_H

/*
 * This is platform data for bq2415x chip. It contains default board
 * voltages and currents which can be also later configured via sysfs. If
 * value is -1 then default chip value (specified in datasheet) will be
 * used.
 *
 */

#define CA111_REG_LASER_ON 0x6c
#define CA111_REG_LASER_OFF 0x73
#define CA111_REG_LASER_STATUS 0x72
#define CA111_REG_TRIGGER_SINGLE_HIGH 0x6d
#define CA111_REG_TRIGGER_CONTINUOS_HIGH 0x63
#define CA111_REG_TRIGGER_SINGLE_LOW 0x4d
#define CA111_REG_TRIGGER_CONTINUOS_LOW 0x43
#define CA111_REG_STOP_MEASURMENT 0x73


#define CA111_ERROR_NORMAL 0x00
#define CA111_ERROR_OUT_OF_RANGE 0x01
#define CA111_ERROR_WEAK_SIGNAL 0x02
#define CA111_ERROR_OPERATING_VOLTAGE 0x05
#define CA111_ERROR_TEMPERATURE 0x06
#define CA111_ERROR_LASERPOWER 0x09
#define CA111_LASERLIT 0x0a
#define CA111_LASERNOTLIT 0x0b

enum ca111_mode {
	CA111_MODE_OFF,		/* Disabled */
	CA111_MODE_FOCUSING,    /* Mode 1 */
	CA111_MODE_RANGEFIND,	/* Mode 2 */
};


enum ca111_measuring_speed {
	CA111_MEASURESPEED_NORMAL,
	CA111_MEASURESPEED_HIGH,
};

enum ca111_measuring_accuracy {
	CA111_ACCURACY_LOW,
	CA111_ACCURACY_HIGH,
};

struct ca111_platform_data {  
	enum ca111_mode mode;
	enum ca111_measuring_speed speed;
	enum ca111_measuring_accuracy accuracy;
	const char *notify_device;	/* name */
};

static ssize_t ca111_sysfs_continousmeasure_fast(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t ca111_sysfs_singlemeasure_fast(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t ca111_sysfs_continousmeasure_high(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t ca111_sysfs_singlemeasure_high(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t ca111_sysfs_stopmeasure(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t ca111_sysfs_status(struct device* dev, struct device_attribute* attr, char* buf);
static ssize_t ca111_sysfs_laseron(struct device* dev, struct device_attribute* attr, char* buf);
static int ca111_i2c_send_command(struct i2c_client *client, char command);
/* static int ca111_laser_on(struct i2c_client *client, const struct i2c_device_id *id); */
static int ca111_laser_off(struct i2c_client *client, const struct i2c_device_id *id);
/* static int ca111_measurment_single(struct i2c_client *client, const struct i2c_device_id *id); */
/* static int ca111_measurment_continous(struct i2c_client *client, const struct i2c_device_id *id); */
static int ca111_read_measurment(struct i2c_client *client, int *length, int *status);
extern struct input_dev* ca111_get_input_dev(void);
extern int ca111_get_laserstatus(void);

#endif
