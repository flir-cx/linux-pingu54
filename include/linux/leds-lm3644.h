/*
 * as3649.h - platform data structure for as3649 led controller
 *
 * Copyright (C) 2012 Ulrich Herrmann <ulrich.herrmann@austriamicrosystems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */

#ifndef __LINUX_LM3644_H
#define __LINUX_LM3644_H

#include <linux/device.h>

struct lm3644_platform_data {
	void (*init)(struct device *dev, bool on); /* callback to set
					LM3644 enable pin high/low*/
	void (*enable)(struct device *dev, bool on); /* callback to get/
					release resources for LM3644*/
	int autosuspend_delay_ms; /* For pm_runtime_set_autosuspend_delay*/
	int reset_gpio;
	u16 max_torch_current;
};
#endif /* __LINUX_LM3644_H */
