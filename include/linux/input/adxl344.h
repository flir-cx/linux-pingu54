/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * include/linux/input/adxl344.h
 *
 * Digital Accelerometer characteristics are highly application specific
 * and may vary between boards and models. The platform_data for the
 * device's "struct device" holds this information.
 *
 * Copyright 2009 Analog Devices Inc.
 */

#ifndef __ADXL344_H__
#define __ADXL344_H__

struct adxl344_platform_data {

	/*
	 * By default, x is axis 0, y is axis 1, z is axis 2; these can be
	 * changed to account for sensor orientation within the host device.
	 */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	/*
	 * Each axis can be negated to account for sensor orientation within
	 * the host device.
	 */
	bool negate_x;
	bool negate_y;
	bool negate_z;

	/* Distinguish between sensor and LCD accelerometers in FLIR cameras*/
	bool is_sensor_accel;
};
#endif  /* __KXTJ9_H__ */
