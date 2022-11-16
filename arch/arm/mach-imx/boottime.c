// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Boot-time support using EPIT
 *
 * Copyright (C) 2022 FLIR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <asm/mach/time.h>
#include "common.h"
#include "hardware.h"

#define EPIT1_BASE_ADDR (0x020D0000)

struct epit {
	u32 cr;
	u32 sr;
	u32 lr;
	u32 cmpr;
	u32 cnr;
};

// Read boot timer (us since init)
u32 board_get_time(void)
{
	static struct epit *epit_regs;

	if (epit_regs == NULL)
		epit_regs = (struct epit *) ioremap(EPIT1_BASE_ADDR, 0x1000);

	return 0xFFFFFFFF - epit_regs->cnr;
}
