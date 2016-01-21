/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file fake.c
 *
 * @brief This file contains a Fake LCD driver device interface and fops
 * functions.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/mod_devicetable.h>
#include <linux/vmalloc.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include "mxc_dispdrv.h"

#define DISPDRV_FAKE	"fake"

struct fsl_mxc_fake_display_data {
	const char *disp_name;
	const struct display_timings *timings;
	struct fb_videomode native_mode;
};

struct fake_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_fake;
	bool inited;
	int ipu_id;
	int disp_id;
	struct notifier_block nb;
	struct fsl_mxc_fake_display_data display;
};

#ifndef MODULE
static int __init fake_setup(char *options)
{
	return 1;
}
__setup("fake=", fake_setup);
#endif


static int fake_disp_setup(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	return 0;
}

int fake_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	return 0;
}

static int fake_set_videomode(struct device *dev,
		struct fake_data *fake,
		struct mxc_dispdrv_setting *setting,
		int index)
{
	int ret = -ENOENT;
	int i;
	struct fb_videomode *modedb = NULL;
	struct fsl_mxc_fake_display_data *display;

	if (index == 0)
		display = &fake->display;
	else {
		dev_info(dev, " Invalid display index %d\n", index);
		return -ENOENT;
	}

	if (!display->timings->num_timings)
		return -ENOENT;

	modedb = vzalloc(sizeof(*modedb) *
			display->timings->num_timings);
	if (!modedb)
		return -ENOMEM;

	for (i = 0; i < display->timings->num_timings; i++) {
		struct videomode vm;

		videomode_from_timing(display->timings->timings[i], &vm);
		ret = fb_videomode_from_videomode(&vm, &modedb[i]);
		if (ret)
			goto out;
	}

	ret = fb_find_mode(&setting->fbi->var, setting->fbi,
			setting->dft_mode_str, modedb,
			display->timings->num_timings, NULL,
			setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var,
				&display->native_mode);
		dev_info(dev, "Using native mode for %s\n",
				display->disp_name);
	} else {
		dev_info(dev, "Using specified mode for %s\n",
				setting->dft_mode_str);
	}

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < display->timings->num_timings; i++) {
		struct fb_videomode m;

		fb_var_to_videomode(&m, &setting->fbi->var);
		if (fb_mode_is_equal(&m, &modedb[i])) {
			ret = fb_add_videomode(&modedb[i],
				&setting->fbi->modelist);
			break;
		}
	}
out:
	vfree(modedb);
	return ret;
}

static int fake_disp_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret;
	struct fake_data *fake = mxc_dispdrv_getdata(disp);
	struct device *dev = &fake->pdev->dev;
	uint32_t setting_idx = 0;

//	setting->if_fmt = IPU_PIX_FMT_ABGR32;

	ret = ipu_di_to_crtc(dev, fake->ipu_id,
			fake->disp_id, &setting->crtc);
	if (ret < 0)
		return ret;

	/* must use spec video mode defined by driver */
	fake_set_videomode(&fake->pdev->dev, fake, setting, setting_idx);

	if (!fake->inited) {
		fake->nb.notifier_call = fake_fb_event;
		ret = fb_register_client(&fake->nb);
		fake->inited = true;
	}

	return ret;
}

static void fake_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct fake_data *fake = mxc_dispdrv_getdata(disp);

	fb_unregister_client(&fake->nb);
}

static struct mxc_dispdrv_driver fake_drv = {
	.name 	= DISPDRV_FAKE,
	.init 	= fake_disp_init,
	.deinit	= fake_disp_deinit,
	.setup = fake_disp_setup,
};

static int fake_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int fake_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device_id imx_fake_devtype[] = {
	{
		.name = "fake-imx6",
		.driver_data = 0,
	}, {
		/* sentinel */
	}
};

static const struct of_device_id imx_fake_dt_ids[] = {
	{ .compatible = "fsl,imx6q-fake", .data = &imx_fake_devtype[0],},
	{ /* sentinel */ }
};

/*!
 * This function is called by the driver framework to initialize the Fake
 * device.
 *
 * @param	dev	The device structure for the Fake passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int fake_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct fake_data *fake;
	struct device_node *np = pdev->dev.of_node;
 	struct device_node *disp_np = NULL;
	u32 ipu_id, disp_id;
	const struct of_device_id *of_id =
			of_match_device(imx_fake_dt_ids, &pdev->dev);

	fake = devm_kzalloc(&pdev->dev, sizeof(*fake), GFP_KERNEL);
	if (!fake)
		return -ENOMEM;


	ret = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (ret) {
		dev_err(&pdev->dev, "failed to read of property ipu_id\n");
		return ret;
	}

	ret = of_property_read_u32(np, "disp_id", &disp_id);
	if (ret) {
		dev_err(&pdev->dev, "failed to read of property disp_id\n");
		return ret;
	}

	fake->ipu_id = ipu_id;
	fake->disp_id = disp_id;

	/* Primary display timings */
	disp_np = of_parse_phandle(np, "display" , 0);
	if (!disp_np) {
		dev_err(&pdev->dev, "failed to get primary display.\n");
		return -ENOENT;
	}
	fake->display.disp_name = disp_np->name;
	fake->display.timings = of_get_display_timings(disp_np);

	if (!fake->display.timings) {
		dev_err(&pdev->dev, "failed to get primary display timings\n");
		return -ENOENT;
	}

	fake->pdev = pdev;
	fake->disp_fake = mxc_dispdrv_register(&fake_drv);
	mxc_dispdrv_setdata(fake->disp_fake, fake);

	dev_set_drvdata(&pdev->dev, fake);

	dev_dbg(&pdev->dev, "%s exit\n", __func__);
	return ret;
}

static int fake_remove(struct platform_device *pdev)
{
	struct fake_data *fake = dev_get_drvdata(&pdev->dev);

	if (!fake->inited)
		return 0;
	mxc_dispdrv_puthandle(fake->disp_fake);
	mxc_dispdrv_unregister(fake->disp_fake);

	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static struct platform_driver mxcfake_driver = {
	.driver = {
		.name = "mxc_fake",
		.of_match_table	= imx_fake_dt_ids,
	},
	.probe = fake_probe,
	.remove = fake_remove,
	.suspend = fake_suspend,
	.resume = fake_resume,
};

static int __init fake_init(void)
{
	pr_err("fake_init \n");
	return platform_driver_register(&mxcfake_driver);
}

static void __exit fake_uninit(void)
{
	platform_driver_unregister(&mxcfake_driver);
}

module_init(fake_init);
module_exit(fake_uninit);

MODULE_AUTHOR("Peter Fitger, FLIR Systems AB");
MODULE_DESCRIPTION("MXC FAKE driver");
MODULE_LICENSE("GPL");
