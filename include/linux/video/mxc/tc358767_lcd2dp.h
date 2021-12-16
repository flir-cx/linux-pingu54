/*
 * API definitions for the Toshiba TC358767A DSI to DisplayPort converter
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _TC358767_LCD2DP_H_
#define _TC358767_LCD2DP_H_

void *lcd2dp_get_handle(const char *idstr);
void lcd2dp_set_hotplug(void *handle, bool active);
void lcd2dp_signal_hotplug_irq(void *handle);

#endif
