/*
 * Copyright 2005-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file flir_csi.c
 *
 * @brief flir FPGA/CSI camera driver functions
 *
 * @ingroup Camera
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define MIN_FPS 9
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define FLIR_CSI_XCLK_MIN 6000000
#define FLIR_CSI_XCLK_MAX 27000000

enum flir_csi_mode {
	flir_csi_mode_MIN = 0,
	flir_csi_mode_VGA_640_480 = 0,
	flir_csi_mode_QVGA_320_240 = 1,
	flir_csi_mode_XVGA_1024_768 = 2,
	flir_csi_mode_UVGA_1280_1024 = 3,
	flir_csi_mode_HDMI_960_720 = 4,
	flir_csi_mode_UVGA_1280_960 = 5,
	flir_csi_mode_MAX = 5
};

enum flir_csi_frame_rate {
	flir_csi_9_fps,
	flir_csi_15_fps,
	flir_csi_30_fps,
	flir_num_frame_rates
};

static const int flir_framerates[] = {
	[flir_csi_9_fps] = 9,
	[flir_csi_15_fps] = 15,
	[flir_csi_30_fps] = 30,
};

struct flir_csi_mode_info {
	enum flir_csi_mode mode;
	u32 width;
	u32 height;
};

struct flir_csi_dt_data{
	int csi_id;
	int mclk;
	int capturemode;
	u32 format;
};

/*!
 * Maintains the information on the current state of the sensor.
 */
static const struct flir_csi_mode_info flir_csi_mode_info_data[flir_num_frame_rates][flir_csi_mode_MAX + 1] = {
	{
		{flir_csi_mode_VGA_640_480,     640,  480},
		{flir_csi_mode_QVGA_320_240,    320,  240},
		{flir_csi_mode_XVGA_1024_768,  1024,  768},
		{flir_csi_mode_UVGA_1280_1024, 1280,  1024},
		{flir_csi_mode_HDMI_960_720,    960,  720},
		{flir_csi_mode_UVGA_1280_960,  1280,  960},
	},
	{
		{flir_csi_mode_VGA_640_480,     640,  480},
		{flir_csi_mode_QVGA_320_240,    320,  240},
		{flir_csi_mode_XVGA_1024_768,  1024,  768},
		{flir_csi_mode_UVGA_1280_1024, 1280,  1024},
		{flir_csi_mode_HDMI_960_720,    960,  720},
		{flir_csi_mode_UVGA_1280_960,  1280,  960},
	},
	{
		{flir_csi_mode_VGA_640_480,     640,  480},
		{flir_csi_mode_QVGA_320_240,    320,  240},
		{flir_csi_mode_XVGA_1024_768,  1024,  768},
		{flir_csi_mode_UVGA_1280_1024, 1280,  1024},
		{flir_csi_mode_HDMI_960_720,    960,  720},
		{flir_csi_mode_UVGA_1280_960,  1280,  960},
	},
};

static int flir_csi_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int flir_csi_remove(struct i2c_client *client);

static const struct i2c_device_id flir_csi_id[] = {
		{"flir_csi", 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, flir_csi_id);

static struct i2c_driver flir_csi_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "flir_csi",
		  },
	.probe  = flir_csi_probe,
	.remove = flir_csi_remove,
	.id_table = flir_csi_id,
};

static int flir_csi_init_mode(struct sensor_data *sensor,
		enum flir_csi_frame_rate frame_rate,
		enum flir_csi_mode mode)
{
	int retval = 0;

	pr_debug("flir_csi_init_mode\n");

	if (mode > flir_csi_mode_MAX || mode < flir_csi_mode_MIN) {
		pr_err("Wrong flir_csi mode detected!\n");
	}

	sensor->pix.width = flir_csi_mode_info_data[frame_rate][mode].width;
	sensor->pix.height = flir_csi_mode_info_data[frame_rate][mode].height;

	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct sensor_data *sensor = s->priv;

	pr_debug("ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = sensor->mclk;
	pr_debug("   clock_curr=mclk=%d\n", sensor->mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = FLIR_CSI_XCLK_MIN;
	p->u.bt656.clock_max = FLIR_CSI_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
	p->u.bt656.latch_clk_inv = 1;

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {
	} else if (!on && sensor->on) {
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	pr_debug("ioctl_g_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is %d instead of V4L2_BUF_TYPE_VIDEO_CAPTURE\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum flir_csi_frame_rate frame_rate;
	int ret = 0;

	pr_debug("ioctl_s_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			frame_rate = flir_csi_15_fps;
		else if (tgt_fps == 30)
			frame_rate = flir_csi_30_fps;
		else if (tgt_fps == 9)
			frame_rate = flir_csi_9_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;

		ret = flir_csi_init_mode(sensor, frame_rate,
				sensor->streamcap.capturemode);
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	pr_debug("ioctl_g_fmt_cap\n");

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	struct sensor_data *sensor = s->priv;

	pr_debug("ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = sensor->brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = sensor->hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = sensor->contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = sensor->saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = sensor->red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = sensor->blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = sensor->ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;

	pr_debug("In flir_csi:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct sensor_data *sensor = s->priv;

	pr_debug("ioctl_enum_framesizes (%d %d)\n", fsize->index, flir_csi_mode_MAX);

	if (fsize->index > flir_csi_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = sensor->pix.pixelformat;
	fsize->discrete.width = flir_csi_mode_info_data[0][fsize->index].width;
	fsize->discrete.height = flir_csi_mode_info_data[0][fsize->index].height;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
				     struct v4l2_frmivalenum *fival)
{
	int i, j, count;
	struct sensor_data *sensor = s->priv;

	if (fival->index < 0 || fival->index > flir_csi_mode_MAX) {
		pr_err("Index %d out of range\n", fival->index);
		return -EINVAL;
	}

	if (fival->width == 0 || fival->height == 0 ||
	    fival->pixel_format == 0) {
		pr_warn("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(flir_csi_mode_info_data); i++) {
		for (j = 0; j < (flir_csi_mode_MAX + 1); j++) {
			if (fival->pixel_format == sensor->pix.pixelformat
			 && fival->width == flir_csi_mode_info_data[i][j].width
			 && fival->height == flir_csi_mode_info_data[i][j].height) {
				count++;
			}
			if (fival->index == (count - 1)) {
				fival->discrete.denominator =
						flir_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;

	pr_debug("ioctl_g_chip_ident\n");

	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ov_flir_csi_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	pr_debug("ioctl_init\n");

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	struct sensor_data *sensor = s->priv;

	if (fmt->index > flir_csi_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = sensor->pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum flir_csi_frame_rate frame_rate;

	sensor->on = true;

	/* mclk */
	tgt_xclk = sensor->mclk;
	tgt_xclk = min(tgt_xclk, (u32)FLIR_CSI_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)FLIR_CSI_XCLK_MIN);
	sensor->mclk = tgt_xclk;

	pr_debug("ioctl_dev_init\n");

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = flir_csi_15_fps;
	else if (tgt_fps == 30)
		frame_rate = flir_csi_30_fps;
	else if (tgt_fps == 9)
		frame_rate = flir_csi_9_fps;
	else
		return -EINVAL; /* Only support 9fps, 15fps or 30fps now. */

	return flir_csi_init_mode(sensor, frame_rate,
				sensor->streamcap.capturemode);
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc flir_csi_ioctl_desc[] = {
		{ vidioc_int_dev_init_num,
		  (v4l2_int_ioctl_func *)ioctl_dev_init },
		{ vidioc_int_dev_exit_num,
		  ioctl_dev_exit},
		{ vidioc_int_s_power_num,
		  (v4l2_int_ioctl_func *)ioctl_s_power },
		{ vidioc_int_g_ifparm_num,
		  (v4l2_int_ioctl_func *)ioctl_g_ifparm },
		{ vidioc_int_init_num,
		  (v4l2_int_ioctl_func *)ioctl_init },
		{ vidioc_int_enum_fmt_cap_num,
		  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
		{ vidioc_int_g_fmt_cap_num,
		  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
		{ vidioc_int_g_parm_num,
		  (v4l2_int_ioctl_func *)ioctl_g_parm },
		{ vidioc_int_s_parm_num,
		  (v4l2_int_ioctl_func *)ioctl_s_parm },
		{ vidioc_int_g_ctrl_num,
		  (v4l2_int_ioctl_func *)ioctl_g_ctrl },
		{ vidioc_int_s_ctrl_num,
		  (v4l2_int_ioctl_func *)ioctl_s_ctrl },
		{ vidioc_int_enum_framesizes_num,
		  (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
		{ vidioc_int_enum_frameintervals_num,
		  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
		{ vidioc_int_g_chip_ident_num,
		  (v4l2_int_ioctl_func *)ioctl_g_chip_ident },
};

static struct v4l2_int_slave flir_csi_slave[2] = {
	{
		.ioctls = flir_csi_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(flir_csi_ioctl_desc),
	},
	{
		.ioctls = flir_csi_ioctl_desc,
		.num_ioctls = ARRAY_SIZE(flir_csi_ioctl_desc),
	}
};

static struct v4l2_int_device flir_csi_int_device[2] = {
	{
		.module = THIS_MODULE,
		.name = "flir_csi_0",
		.type = v4l2_int_type_slave,
		.u = {
				.slave = &flir_csi_slave[0],
		},
	},
	{
		.module = THIS_MODULE,
		.name = "flir_csi_1",
		.type = v4l2_int_type_slave,
		.u = {
				.slave = &flir_csi_slave[1],
		},
	},
};

/*!
 * flir_csi I2C probe function
 *
 * @param adapter	struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int flir_csi_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct flir_csi_dt_data *dtdata;
	struct sensor_data *sensor;
	struct device_node *np = client->dev.of_node;
	u32 val;
	const char *format;

	dtdata = devm_kzalloc(&client->dev, sizeof(*dtdata), GFP_KERNEL);

	if (!dtdata) {
		dev_err(&client->dev, "failed to alloc dtdata\n");
		return -ENOMEM;
	}

	if (of_property_read_u32(np, "csi_id", &val) == 0) {
		dtdata->csi_id = val;
        } else {
          dev_err(&client->dev, "Did not get csi_id from OF data\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "mclk", &val) == 0) {
		dtdata->mclk = val;
	} else {
		dev_err(&client->dev, "Did not get mclk value from OF data\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "capturemode", &val) == 0) {
		dtdata->capturemode = val;
	} else {
		dev_err(&client->dev, "Did not get capturemode value from OF data\n");
		return -EINVAL;
	}
	if (of_property_read_string(np, "format", &format) == 0) {
		memcpy(&dtdata->format, format, 4);
	} else {
		dev_err(&client->dev, "Did not get pixel format value from OF data\n");
		return -EINVAL;
	}

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor) {
		dev_err(&client->dev, "failed to alloc sensor\n");
		return -ENOMEM;
	}

	dev_dbg(&client->dev, "CSI %d\n", dtdata->csi_id);

	/* Set initial values for the sensor struct. */
	sensor->mclk = dtdata->mclk;
	sensor->csi = dtdata->csi_id;

	if (sensor->csi >= 2) {
		dev_err(&client->dev, "Erroneous CSI ID\n");
		return -EINVAL;
	}

	sensor->i2c_client = client;
	sensor->pix.pixelformat = dtdata->format;
	sensor->pix.width = flir_csi_mode_info_data[0][dtdata->capturemode].width;
	sensor->pix.height = flir_csi_mode_info_data[0][dtdata->capturemode].height;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
				       V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = dtdata->capturemode;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	flir_csi_int_device[sensor->csi].priv = sensor;
	retval = v4l2_int_device_register(&flir_csi_int_device[sensor->csi]);

	dev_dbg(&client->dev, "%s returning %d", __func__, retval);

	return retval;
}

/*!
 * flir_csi I2C detach function
 *
 * @param client	struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int flir_csi_remove(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s\n", __func__);

	v4l2_int_device_unregister(&flir_csi_int_device[0]);

	return 0;
}

module_i2c_driver(flir_csi_i2c_driver);

MODULE_AUTHOR("Peter Fitger, FLIR Systems AB");
MODULE_AUTHOR("Bo Svangård, FLIR Systems AB");
MODULE_DESCRIPTION("FLIR CSI Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
