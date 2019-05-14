/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
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

/* This file is based on rpmsg_v2.c */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/rpmsg.h>
#include <linux/dma-mapping.h>

#define MIN_FPS 9
#define MAX_FPS 9
#define DEFAULT_FPS 9

enum ovRpmsg_mode {
	ovRpmsg_mode_MIN = 0,
	ovRpmsg_mode_QQVGA_160_120 = 0,
	ovRpmsg_mode_MAX = 0
};

enum ovRpmsg_frame_rate {
	ovRpmsg_9_fps,
};

struct ovRpmsg_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct ovRpmsg_mode_info {
	enum ovRpmsg_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

// static struct sensor_data ovRpmsg_data;

enum rxtypes {
	RX_LUT,
	RX_BUF_0,
	RX_BUF_1,
	RX_BUF_2,
	RX_BUF_3,
};

struct csi_rx_msg {
	uint32_t type;
	uint32_t addr;
};

struct ovRpmsg {
	struct v4l2_subdev		subdev;
	struct v4l2_pix_format pix;
	const struct ovRpmsg_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	struct rpmsg_device *rpmsg_dev;
	void (*callback)(uint32_t addr, uint32_t buf_num, void *ptr);
	void *callback_dev;
	bool on;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct ovRpmsg ovRpmsg_data;

static const struct ovRpmsg_datafmt ovRpmsg_colour_fmts[] = {
	{MEDIA_BUS_FMT_YUYV8_1X16, V4L2_COLORSPACE_JPEG},
};

/* Find a data format by a pixel code in an array */
static const struct ovRpmsg_datafmt
			*rpmsg_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ovRpmsg_colour_fmts); i++)
		if (ovRpmsg_colour_fmts[i].code == code)
			return ovRpmsg_colour_fmts + i;

	return NULL;
}

/* change to or back to subsampling mode set the mode directly
 * image size below 1280 * 960 is subsampling mode */
static int rpmsg_change_mode_direct(enum ovRpmsg_frame_rate frame_rate,
			    enum ovRpmsg_mode mode)
{
	pr_info("%s: width = %d, height = %d\n",
		__func__, ovRpmsg_data.pix.width, ovRpmsg_data.pix.height);

	pr_info("%s: mode = %d, frame_rate = %d bp2\n", __func__, mode, frame_rate);


	if (mode != ovRpmsg_mode_QQVGA_160_120 || frame_rate != ovRpmsg_9_fps) {
		return -EINVAL;
	}

	return 0;
}

static int rpmsg_change_mode(enum ovRpmsg_frame_rate frame_rate,
			    enum ovRpmsg_mode mode)
{
	int retval = 0;

	if (mode > ovRpmsg_mode_MAX || mode < ovRpmsg_mode_MIN) {
		pr_err("Wrong rpmsg mode detected!\n");
		return -1;
	}

	/* change back to subsampling modem download firmware directly
	 * image size below 1280 * 960 is subsampling mode */
	retval = rpmsg_change_mode_direct(frame_rate, mode);

	return retval;
}

/*!
 * rpmsg_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int rpmsg_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = 0;
		cparm->timeperframe.denominator = DEFAULT_FPS;
		cparm->timeperframe.numerator = 1;
		cparm->capturemode = MEDIA_BUS_FMT_YUYV8_1X16;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ov5460_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int rpmsg_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ovRpmsg_frame_rate frame_rate;
	int ret = 0;

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

		if (tgt_fps == 9)
			frame_rate = ovRpmsg_9_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			goto error;
		}

		ret = rpmsg_change_mode(frame_rate,
				a->parm.capture.capturemode);
		if (ret < 0)
			goto error;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

error:
	return ret;
}

static int rpmsg_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct ovRpmsg_datafmt *fmt = rpmsg_find_datafmt(mf->code);

	if (format->pad)
		return -EINVAL;

	if (!fmt) {
		mf->code	= ovRpmsg_colour_fmts[0].code;
		mf->colorspace	= ovRpmsg_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	return 0;
}

static int rpmsg_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	if (format->pad)
		return -EINVAL;

	mf->code	= MEDIA_BUS_FMT_YUYV8_1X16;
	mf->colorspace	= V4L2_COLORSPACE_JPEG;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int rpmsg_enum_code(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ovRpmsg_colour_fmts))
		return -EINVAL;

	code->code = ovRpmsg_colour_fmts[code->index].code;
	return 0;
}

/*!
 * rpmsg_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int rpmsg_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > ovRpmsg_mode_MAX)
		return -EINVAL;

	fse->max_width = 160;
	fse->min_width = fse->max_width;
	fse->max_height = 120;
	fse->min_height = fse->max_height;
	return 0;
}

/*!
 * rpmsg_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int rpmsg_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index != 0)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warning("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	if (fie->width == 160 && fie->height == 120) {
		fie->interval.denominator = 9;
		return 0;
	}

	return -EINVAL;
}

static struct v4l2_subdev_video_ops rpmsg_subdev_video_ops = {
	.g_parm = rpmsg_g_parm,
	.s_parm = rpmsg_s_parm,
};

static const struct v4l2_subdev_pad_ops rpmsg_subdev_pad_ops = {
	.enum_frame_size       = rpmsg_enum_framesizes,
	.enum_frame_interval   = rpmsg_enum_frameintervals,
	.enum_mbus_code        = rpmsg_enum_code,
	.set_fmt               = rpmsg_set_fmt,
	.get_fmt               = rpmsg_get_fmt,
};

static struct v4l2_subdev_ops ovRpsg_subdev_ops = {
	.video	= &rpmsg_subdev_video_ops,
	.pad	= &rpmsg_subdev_pad_ops,
};

/*
 * rpmsg_send_buffer
 *
 * Tell M4 to generate colorized buffer
 */

int rpmsg_send_buffer(dma_addr_t eba)
{
	int err = 0;
	struct csi_rx_msg msg;
	static int buffer_num;

	msg.type = RX_BUF_0 + buffer_num;
	msg.addr = eba;

	err = rpmsg_send(ovRpmsg_data.rpmsg_dev->ept, &msg, sizeof(msg));

	buffer_num = (buffer_num + 1) & 3;

	return err;
}
EXPORT_SYMBOL(rpmsg_send_buffer);

/*
 * rpmsg_lepton_callback
 *
 * Callback from RPMSG when buffer is received from M4
 */

static int rpmsg_lepton_callback(struct rpmsg_device *dev, void *data, int len,
		void *priv, u32 src)
{
	struct {
		uint32_t addr;
		uint32_t buf_num;
	} *msg = data;
//	print_hex_dump(KERN_INFO, "incoming message:", DUMP_PREFIX_NONE, 16, 1, data, len, true);

//	pr_info("Got %X %d\n", msg->addr, msg->buf_num);

	if (ovRpmsg_data.callback) {
		ovRpmsg_data.callback(msg->addr, msg->buf_num, ovRpmsg_data.callback_dev);
	}

	return 0;
}

int rpmsg_setup_callback(void (*func)(uint32_t addr, uint32_t buf_num, void *ptr), void *callback_dev)
{
	ovRpmsg_data.callback = func;
	ovRpmsg_data.callback_dev = callback_dev;
	return 0;
}
EXPORT_SYMBOL(rpmsg_setup_callback);


static int rpmsg_lepton_probe(struct rpmsg_device *dev)
{
	int err;

	dev_info(&dev->dev, "new channel: 0x%x -> 0x%x!\n", dev->src, dev->dst);

	/* Set initial values for the sensor struct. */
	memset(&ovRpmsg_data, 0, sizeof(ovRpmsg_data));

	ovRpmsg_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ovRpmsg_data.pix.width = 160;
	ovRpmsg_data.pix.height = 120;
	ovRpmsg_data.streamcap.capturemode = 0;
	ovRpmsg_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ovRpmsg_data.streamcap.timeperframe.numerator = 1;
	ovRpmsg_data.rpmsg_dev = dev;

	v4l2_subdev_init(&ovRpmsg_data.subdev, &ovRpsg_subdev_ops);
	ovRpmsg_data.subdev.owner = dev->dev.driver->owner;
	ovRpmsg_data.subdev.dev = &dev->dev;
	ovRpmsg_data.subdev.dev->of_node = of_find_node_by_name(of_root, "m4_rpmsg");
	v4l2_set_subdevdata(&ovRpmsg_data.subdev, dev);
	strlcpy(ovRpmsg_data.subdev.name, dev->dev.driver->name, sizeof(ovRpmsg_data.subdev.name));

	err = v4l2_async_register_subdev(&ovRpmsg_data.subdev);
	if (err < 0)
		dev_err(&dev->dev, "%s--Async register failed, ret=%d\n", __func__, err);

	pr_info("camera ovRpmsg is found\n");

	return err;
}

static void rpmsg_lepton_remove(struct rpmsg_device *dev)
{
	dev_info(&dev->dev, "rpmsg lepton-csi client driver is removed\n");

	v4l2_async_unregister_subdev(&ovRpmsg_data.subdev);
}

static struct rpmsg_device_id rpmsg_driver_lepton_id_table[] = {
	{ .name	= "rpmsg-client-lepton-csi" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_lepton_id_table);

static struct rpmsg_driver rpmsg_lepton_client = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= rpmsg_driver_lepton_id_table,
	.probe		= rpmsg_lepton_probe,
	.callback	= rpmsg_lepton_callback,
	.remove		= rpmsg_lepton_remove,
};

module_rpmsg_driver(rpmsg_lepton_client);

MODULE_AUTHOR("Peter Fitger, FLIR Systems AB");
MODULE_DESCRIPTION("OVRPMSG Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");


