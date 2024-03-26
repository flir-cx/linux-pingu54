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

#include <linux/time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/timekeeping.h>
#include <boottime.h>
#include "common.h"
#include "hardware.h"
#include <linux/math64.h>

static struct kobject *boottime_kobject;

#if defined(CONFIG_MXC_EPIT_BOOTTIME)
#define EPIT1_BASE_ADDR (0x020D0000)

struct epit {
	u32 cr;
	u32 sr;
	u32 lr;
	u32 cmpr;
	u32 cnr;
};

// Read boot timer (us since init)
imx_boottime_t board_get_time(void)
{
	static struct epit *epit_regs;

	if (epit_regs == NULL)
		epit_regs = (struct epit *) ioremap(EPIT1_BASE_ADDR, 0x1000);

	return (u64) (0xFFFFFFFF - epit_regs->cnr);
}
#elif defined(CONFIG_IMX7ULP_BOOTTIME)

#define TSMRB_LOW 0x410A3C08
#define TSMRB_HIGH 0x410A3C0C

imx_boottime_t board_get_time(void)
{
	static void *timer_low;
	static void *timer_high;
	u64 res;
	u32 low;
	u32 high;

	if (timer_low == 0)
		timer_low = ioremap(TSMRB_LOW, 4);
	if (timer_high == 0)
		timer_high = ioremap(TSMRB_HIGH, 4);

	low = readl(timer_low);
	high = readl(timer_high);

	res = (u64) high << 32 | low;

	return (imx_boottime_t)res;
}

#else

imx_boottime_t board_get_time(void)
{
	return div_u64(ktime_get_ns(), NSEC_PER_USEC);
}

#endif

static ssize_t boottime_show(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%llu\n", board_get_time());
}

static ssize_t boottime_store(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	char a[50];
	char *b;

	//limit message length
	strncpy(a, buf, 49);
	a[49] = 0;

	b = strchr(a, '\n');
	if (b)
		*b = '\0';

	pr_info("%s boottime %llu\n", a, board_get_time());
	return count;
}

static struct kobj_attribute boottime_attribute =
	__ATTR(stamp, 0660, boottime_show, boottime_store);

static int __init boottime_init(void)
{
	int error = 0;

	pr_info("boottime_init boottime %llu\n", board_get_time());
	pr_debug("Module boottime initialized successfully\n");

	boottime_kobject = kobject_create_and_add("boottime",
						  kernel_kobj);
	if (!boottime_kobject)
		return -ENOMEM;

	error = sysfs_create_file(boottime_kobject, &boottime_attribute.attr);
	if (error)
		pr_err("failed to create /sys/kernel/boottime.\n");

	return error;
}

static void __exit boottime_exit(void)
{
	sysfs_remove_file(boottime_kobject, &boottime_attribute.attr);
	kobject_put(boottime_kobject);
	pr_debug("Module boottime uninitialized successfully.\n");
}

module_init(boottime_init);
module_exit(boottime_exit);
