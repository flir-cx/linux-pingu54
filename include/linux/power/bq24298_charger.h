/*
 * Platform data for the TI bq24298 battery charger driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BQ24298_CHARGER_H_
#define _BQ24298_CHARGER_H_

struct bq24298_platform_data {
	unsigned int	gpio_int;	/* GPIO pin that's connected to INT# */
};

extern int bq24298_enable_otg(void);
extern int bq24298_disable_otg(void);
extern int bq24298_get_otg(void);
extern int bq24298_set_iinlim(s16 currentlim);
int bq24298_set_iinlim_helper(s16 currentlim,u8 source);


#endif
