/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Support for hardware backed boot-time.
 *
 * Copyright (C) 2022 FLIR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

typedef u64 imx_boottime_t;

extern imx_boottime_t board_get_time(void);
