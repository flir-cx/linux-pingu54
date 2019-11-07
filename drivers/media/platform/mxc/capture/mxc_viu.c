/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fwnode.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#define IMX_VIU_DRV_NAME	"imx-viu-v4l2"
#define IMX_VIU_MAX_SUBDEV_NUM	4

#define notifier_to_viu_dev(n) container_of(n, struct imx_viu_device, notifier)
#define v4l2_dev_to_viu_dev(d) container_of(d, struct imx_viu_device, v4l2_dev)

/* VIU registers */
#define VIU_SCR					0x00
#define VIU_DINVSZ				0x08
#define VIU_DINVFL				0x0c
#define VIU_DMA_ADDR				0x14
#define VIU_DMA_INC				0x18
#define VIU_INVSZ				0x1c
#define VIU_HPRALRM				0x20
#define VIU_ALPHA				0x24
#define VIU_HFACTOR				0x28
#define VIU_VFACTOR				0x2c
#define VIU_VID_SIZE				0X30
#define VIU_LUT_ADDR				0x34
#define VIU_LUT_DATA				0x38
#define VIU_EXT_CONFIG				0x3c
#define VIU_ACT_ORG				0x4c
#define VIU_ACT_SIZE				0x50

/* reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

/* register bit fields */
#define SCR_MODE32BIT				BIT(31)
#define SCR_ROUND_ON				BIT(30)
#define SCR_DITHER_ON				BIT(29)
#define SCR_GET_FIELD_NO(x)			REG_GET(x, 28, 28)
#define SCR_DMA_ACT				BIT(27)
#define SCR_SCALER_EN				BIT(26)
#define SCR_YUV2RGB_EN				BIT(25)
#define SCR_BC_EN				BIT(24)
#define SCR_MODE444				BIT(23)
#define SCR_ERROR_IRQ				BIT(21)
#define SCR_DMA_END_IRQ				BIT(20)
#define SCR_VSTART_IRQ				BIT(19)
#define SCR_HSYNC_IRQ				BIT(18)
#define SCR_VSYNC_IRQ				BIT(17)
#define SCR_FIELD_IRQ				BIT(16)
#define SCR_ECC_EN				BIT(14)
#define SCR_ERROR_EN				BIT(13)
#define SCR_DMA_END_EN				BIT(12)
#define SCR_VSTART_EN				BIT(11)
#define SCR_HSYNC_EN				BIT(10)
#define SCR_VSYNC_EN				BIT(9)
#define SCR_FIELD_EN				BIT(8)
#define SCR_GET_ERROR_CODE(x)			REG_GET(x, 7, 4)
#define SCR_SET_FORMAT_CTRL(x)			REG_PUT(x, 3, 1)
#define SCR_SOFT_RESET				BIT(0)
#define SCR_IRQ_MASK				(SCR_ERROR_IRQ   |	\
						 SCR_DMA_END_IRQ |	\
						 SCR_VSTART_IRQ  |	\
						 SCR_HSYNC_IRQ   |	\
						 SCR_VSYNC_IRQ   |	\
						 SCR_FIELD_IRQ)
#define SCR_IRQ_EN_MASK				(SCR_ECC_EN	 |	\
						 SCR_ERROR_EN	 |	\
						 SCR_DMA_END_EN	 |	\
						 SCR_VSTART_EN	 |	\
						 SCR_HSYNC_EN	 |	\
						 SCR_VSYNC_EN	 |	\
						 SCR_FIELD_EN)

#define DINVSZ_GET_DETECTED_LINEC(x)		REG_GET(x, 31, 16)
#define DINVSZ_GET_DETECTED_PIXELC(x)		REG_GET(x, 15, 0)

#define DINVFL_GET_DETECTED_FRAME_HEIGHT(x)	REG_GET(x, 31, 16)
#define DINVFL_GET_DETECTED_FRAME_WIDTH(x)	REG_GET(x, 15, 0)

#define INVSZ_GET_LINEC(x)			REG_GET(x, 31, 16)
#define INVSZ_SET_LINEC(x)			REG_PUT(x, 31, 16)
#define INVSZ_GET_PIXELC(x)			REG_GET(x, 15, 0)
#define INVSZ_SET_PIXELC(x)			REG_PUT(x, 15, 0)

#define HPRALRM_SET_ALARM(x)			REG_PUT(x, 15, 0)

#define HFACTOR_SET_FACTOR_INTEGER(x)		REG_PUT(x, 10, 8)
#define HFACTOR_SET_FACTOR_FRACTIONAL(x)	REG_PUT(x, 7,  0)

#define VFACTOR_SET_FACTOR_INTEGER(x)		REG_PUT(x, 10, 8)
#define VFACTOR_SET_FACTOR_FRACTIONAL(x)	REG_PUT(x, 7,  0)

#define VID_SIZE_SET_LINEC(x)			REG_PUT(x, 31, 16)
#define VID_SIZE_SET_PIXELC(x)			REG_PUT(x, 15, 0)

#define EXT_CONFIG_MONO_LSB			BIT(14)
#define EXT_CONFIG_MODE_8BIT			BIT(13)
#define EXT_CONFIG_CS_EN			BIT(12)
#define EXT_CONFIG_LENDIAN			BIT(11)
#define EXT_CONFIG_RGB2YUV_EN			BIT(9)
#define EXT_CONFIG_DE_VALID			BIT(8)
#define EXT_CONFIG_SET_INP_FORMAT(x)		REG_PUT(x, 7, 5)
#define EXT_CONFIG_PCLK_POL_ACTIVE_LOW		BIT(4)
#define EXT_CONFIG_VSYNC_POL_ACTIVE_LOW		BIT(3)
#define EXT_CONFIG_HSYNC_POL_ACTIVE_LOW		BIT(2)
#define EXT_CONFIG_DE_POL_ACTIVE_LOW		BIT(1)
#define EXT_CONFIG_HMIRROR_EN			BIT(0)

#define ACT_ORG_SET_ACT_ORG_Y(x)		REG_PUT(x, 31, 16)
#define ACT_ORG_SET_ACT_ORG_X(x)		REG_PUT(x, 15, 0)

#define ACT_SIZE_SET_ACT_IMG_HEIGHT		REG_PUT(x, 31, 16)
#define ACT_SIZE_SET_ACT_IMG_WIDTH		REG_PUT(x, 15, 0)

/* All VIU IRQs index */
#define ECC_IRQ					1
#define ERROR_IRQ				2
#define DMA_END_IRQ				3
#define VSTART_IRQ				4
#define HSYNC_IRQ				5
#define VSYNC_IRQ				6
#define FIELD_IRQ				7

/* TODO: output format */
#define OUTPUT_IS_RGB				0
#define OUTPUT_IS_32BIT				0

struct imx_viu_input {
	unsigned int index;
	const char *name;
	unsigned int type;
	unsigned int caps;
};

struct imx_viu_fh {
	struct v4l2_fh fh;

	/* TODO other fields added later */
};

struct imx_viu_fmt {
	char name[32];
	u32  pix_fmt;
	u32  fourcc;
	u32  mbus_code;
	int  bpp;
};

struct viu_buf_internal {
	struct list_head queue;
	bool discard;
};

struct imx_viu_buffer {
	struct vb2_v4l2_buffer vb;
	struct viu_buf_internal internal;
	uint32_t field;
};

struct imx_viu_reg_stack {
	unsigned int viu_scr;
	unsigned int viu_dinvsz;
	unsigned int viu_dinvfl;
	unsigned int viu_dma_addr;
	unsigned int viu_dma_inc;
	unsigned int viu_invsz;
	unsigned int viu_hpralrm;
	unsigned int viu_alpha;
	unsigned int viu_hfactor;
	unsigned int viu_vfactor;
	unsigned int viu_vid_size;
	unsigned int viu_lut_addr;
	unsigned int viu_lut_data;
	unsigned int viu_ext_config;
	unsigned int viu_act_org;
	unsigned int viu_act_size;
};

struct imx_viu_device {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device *vdev;
	struct vb2_queue queue;
	struct v4l2_subdev *v4l2_sd;

	int irq;
	void __iomem *base;
	struct completion field;
	struct completion dma_done;
	struct completion vsync;

	spinlock_t slock;
	struct list_head active_queue;
	struct list_head discard;
	uint32_t frame_count;

	void *discard_buffer;
	dma_addr_t discard_buffer_dma;
	size_t discard_size;
	struct viu_buf_internal	buf_discard;
	struct clk *ipg_clk;
	struct mutex lock;
	atomic_t use_count;
	unsigned int curr_input;

	struct imx_viu_reg_stack reset;
	struct imx_viu_fmt viu_fmt;
	struct v4l2_pix_format v4l2_pix_fmt;
	u32 buf_type;
	u32 mbus_code;

	/* async subdevs */
	struct v4l2_async_subdev subdevs[IMX_VIU_MAX_SUBDEV_NUM];
	struct v4l2_async_subdev *async_subdevs[IMX_VIU_MAX_SUBDEV_NUM];
	struct v4l2_async_notifier notifier;
};

static const struct imx_viu_fmt viu_fmts[] = {
	{
		.name		= "UYVY-16",
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.pix_fmt	= V4L2_PIX_FMT_UYVY,
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.bpp		= 16,
	}, {
		.name		= "YUYV-16",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.pix_fmt	= V4L2_PIX_FMT_YUYV,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.bpp		= 16,
	}, {
		.name		= "BGRA-32",
		.fourcc		= V4L2_PIX_FMT_ABGR32,
		.pix_fmt	= V4L2_PIX_FMT_ABGR32,
		.mbus_code	= MEDIA_BUS_FMT_ARGB8888_1X32,
		.bpp		= 32,
	},
};

static const struct imx_viu_input inputs[] = {
	{
		.index = 0,
		.name  = "Camera BT656",
		.type  = V4L2_INPUT_TYPE_CAMERA,
		.caps  = 0,
	},
};

static const struct imx_viu_fmt *viu_fmt_by_mbus(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(viu_fmts); i++) {
		if (viu_fmts[i].mbus_code == code)
			return viu_fmts + i;
	}

	pr_err("unknown mbus:0x%x\n", code);

	return NULL;
}

static const struct imx_viu_fmt *viu_fmt_by_pix_fmt(u32 pixelformat)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(viu_fmts); i++) {
		if (viu_fmts[i].pix_fmt == pixelformat)
			return viu_fmts + i;
	}

	pr_err("unknown pix_fmt:0x%x\n", pixelformat);

	return NULL;
}

static void imx_viu_save_reg_stack(struct imx_viu_device *viu_dev,
				   struct imx_viu_reg_stack *stack)
{
	stack->viu_scr		= readl(viu_dev->base + VIU_SCR);
	stack->viu_invsz	= readl(viu_dev->base + VIU_INVSZ);
	stack->viu_vid_size	= readl(viu_dev->base + VIU_VID_SIZE);
	stack->viu_ext_config	= readl(viu_dev->base + VIU_EXT_CONFIG);
}

static void imx_viu_restore_reg_stack(struct imx_viu_device *viu_dev,
				      struct imx_viu_reg_stack *stack)
{
	writel(stack->viu_scr,        viu_dev->base + VIU_SCR);
	writel(stack->viu_invsz,      viu_dev->base + VIU_INVSZ);
	writel(stack->viu_vid_size,   viu_dev->base + VIU_VID_SIZE);
	writel(stack->viu_ext_config, viu_dev->base + VIU_EXT_CONFIG);
}

static struct imx_viu_buffer *viu_ibuf_to_buf(struct viu_buf_internal *int_buf)
{
	return container_of(int_buf, struct imx_viu_buffer, internal);
}

/* VIU config helper functions */
static void imx_viu_irq_enable(struct imx_viu_device *viu_dev,
			       int irq_idx)
{
	uint32_t scr;

	scr = readl(viu_dev->base + VIU_SCR);

	switch (irq_idx) {
	case ECC_IRQ:
		scr |= SCR_ECC_EN;
		break;
	case ERROR_IRQ:
		scr |= SCR_ERROR_EN;
		break;
	case DMA_END_IRQ:
		scr |= SCR_DMA_END_EN;
		break;
	case VSTART_IRQ:
		scr |= SCR_VSTART_EN;
		break;
	case HSYNC_IRQ:
		scr |= SCR_HSYNC_EN;
		break;
	case VSYNC_IRQ:
		scr |= SCR_VSYNC_EN;
		break;
	case FIELD_IRQ:
		scr |= SCR_FIELD_EN;
		break;
	default:
		/* unsupported irq */
		return;
	}

	writel(scr, viu_dev->base + VIU_SCR);
}

static void imx_viu_irq_disable(struct imx_viu_device *viu_dev,
				int irq_idx)
{
	uint32_t scr;

	scr = readl(viu_dev->base + VIU_SCR);

	switch (irq_idx) {
	case ECC_IRQ:
		scr &= ~SCR_ECC_EN;
		break;
	case ERROR_IRQ:
		scr &= ~SCR_ERROR_EN;
		break;
	case DMA_END_IRQ:
		scr &= ~SCR_DMA_END_EN;
		break;
	case VSTART_IRQ:
		scr &= ~SCR_VSTART_EN;
		break;
	case HSYNC_IRQ:
		scr &= ~SCR_HSYNC_EN;
		break;
	case VSYNC_IRQ:
		scr &= ~SCR_VSYNC_EN;
		break;
	case FIELD_IRQ:
		scr &= ~SCR_FIELD_EN;
		break;
	default:
		/* unsupported irq */
		return;
	}

	writel(scr, viu_dev->base + VIU_SCR);
}

static void imx_viu_irq_init(struct imx_viu_device *viu_dev)
{
	uint32_t scr = 0;

	scr = readl(viu_dev->base + VIU_SCR);

	/* disable all VIU IRQs */
	scr &= ~SCR_IRQ_EN_MASK;

	writel(scr, viu_dev->base + VIU_SCR);
}

static int imx_viu_wait_for_vsync(struct imx_viu_device *viu_dev)
{
	unsigned long ret;

	imx_viu_irq_enable(viu_dev, VSYNC_IRQ);

	ret = wait_for_completion_timeout(&viu_dev->vsync, HZ * 10);
	if (!ret) {
		dev_err(viu_dev->dev, "wait for vsync timeout, scr = %#x\n",
			readl(viu_dev->base + VIU_SCR));
		imx_viu_irq_disable(viu_dev, VSYNC_IRQ);

		return -EBUSY;
	}

	imx_viu_irq_disable(viu_dev, VSYNC_IRQ);

	return 0;
}

static int imx_viu_wait_for_field(struct imx_viu_device *viu_dev,
				  unsigned int field)
{
	int ret;
	unsigned int scr;
	struct v4l2_pix_format *fmt = &viu_dev->v4l2_pix_fmt;

	switch (fmt->field) {
	case V4L2_FIELD_INTERLACED:
		if (field > 1)
			return -EINVAL;

		imx_viu_irq_enable(viu_dev, FIELD_IRQ);
next_field:
		ret = wait_for_completion_timeout(&viu_dev->field, HZ / 10);
		if (!ret) {
			dev_err(viu_dev->dev, "wait for field timeout\n");
			imx_viu_irq_disable(viu_dev, FIELD_IRQ);
			return -EBUSY;
		}
		scr = readl(viu_dev->base + VIU_SCR);
		if (SCR_GET_FIELD_NO(scr) != field)
			goto next_field;

		imx_viu_irq_disable(viu_dev, FIELD_IRQ);
		break;
	case V4L2_FIELD_NONE:
		if (field != 0)
			return -EINVAL;

		return imx_viu_wait_for_vsync(viu_dev);
	default:
		return -EINVAL;
	}

	return 0;
}

/* reset don't change register values */
static void imx_viu_soft_reset(struct imx_viu_device *viu_dev)
{
	unsigned int scr;

	return; 

	scr = readl(viu_dev->base + VIU_SCR);

	/* set reset */
	scr |= SCR_SOFT_RESET;
	writel(scr, viu_dev->base + VIU_SCR);

	mdelay(1);

	/* out of reset */
	scr &= ~SCR_SOFT_RESET;
	writel(scr, viu_dev->base + VIU_SCR);
}

static int imx_viu_set_fmt(struct imx_viu_device *viu_dev)
{
	uint32_t scr, invsz, vid_size, ext_config;
	struct v4l2_pix_format *fmt = &viu_dev->v4l2_pix_fmt;
	struct imx_viu_fmt *viu_fmt = &viu_dev->viu_fmt;

	scr = readl(viu_dev->base + VIU_SCR);
	scr &= ~SCR_SET_FORMAT_CTRL(0x7);
	scr &= ~(SCR_MODE444 | SCR_YUV2RGB_EN);

	ext_config = readl(viu_dev->base + VIU_EXT_CONFIG);
	ext_config &= ~EXT_CONFIG_SET_INP_FORMAT(0x7);
	ext_config |= EXT_CONFIG_LENDIAN;

	/* TODO: only support BT.656 format */
	switch (viu_fmt->mbus_code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
#if OUTPUT_IS_32BIT
		/* RGB32 or YUV444 */
		scr |= SCR_MODE32BIT;
#else
		/* RGB565 or YUV422 */
		scr &= ~SCR_MODE32BIT;
#endif
#if OUTPUT_IS_RGB
		scr |= SCR_SET_FORMAT_CTRL(0x3);
#else
		scr |= SCR_SET_FORMAT_CTRL(0x0);
#endif
		invsz = INVSZ_SET_LINEC(fmt->height) |
			INVSZ_SET_PIXELC(fmt->width);
		vid_size = VID_SIZE_SET_LINEC(fmt->height) |
			   VID_SIZE_SET_PIXELC(fmt->width);
		/* ITU-656 stream */
		ext_config |= EXT_CONFIG_SET_INP_FORMAT(0x0);
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		/* YUV422 */
		scr &= ~SCR_MODE32BIT;
		scr |= SCR_SET_FORMAT_CTRL(0x1);
		invsz = INVSZ_SET_LINEC(fmt->height) |
			INVSZ_SET_PIXELC(fmt->width);
		vid_size = VID_SIZE_SET_LINEC(fmt->height) |
			   VID_SIZE_SET_PIXELC(fmt->width);
		/* ITU-656 stream */
		ext_config |= EXT_CONFIG_SET_INP_FORMAT(0x0);
		break;
	case MEDIA_BUS_FMT_ARGB8888_1X32:
		/* BGRA8888 */
		scr |= SCR_MODE32BIT | SCR_YUV2RGB_EN;
		scr |= SCR_SET_FORMAT_CTRL(0x3);
		invsz = INVSZ_SET_LINEC(fmt->height) |
			INVSZ_SET_PIXELC(fmt->width);
		vid_size = VID_SIZE_SET_LINEC(fmt->height) |
			   VID_SIZE_SET_PIXELC(fmt->width);
		/* ITU-656 stream */
		ext_config |= EXT_CONFIG_SET_INP_FORMAT(0x0);
		break;
	default:
		dev_err(viu_dev->dev,
			"%s: unsupported mbus fmt: %#x\n",
			__func__, viu_fmt->mbus_code);
		return -EINVAL;
	}

	writel(ext_config, viu_dev->base + VIU_EXT_CONFIG);
	writel(invsz, viu_dev->base + VIU_INVSZ);
	writel(vid_size, viu_dev->base + VIU_VID_SIZE);
	writel(scr, viu_dev->base + VIU_SCR);

	return 0;
}

static int imx_viu_config_dma(struct imx_viu_device *viu_dev,
			      dma_addr_t dma_addr,
			      unsigned int field)
{
	unsigned int scr, dma_inc = 0, field_addr;
	struct v4l2_pix_format *fmt = &viu_dev->v4l2_pix_fmt;

	if (field > 1)
		return -EINVAL;

	/* 'dma_inc' shall only be configured when DMA
	 * is inactive, during vertical blanking
	 */
	scr = readl(viu_dev->base + VIU_SCR);
	if (WARN_ON(scr & SCR_DMA_ACT))
		return -EBUSY;

	switch (fmt->field) {
	case V4L2_FIELD_INTERLACED:
		dma_inc    = fmt->bytesperline;
		field_addr = dma_addr + field * fmt->bytesperline;
		break;
	case V4L2_FIELD_NONE:
		dma_inc = 0;
		field_addr = dma_addr;
		break;
	default:
		dev_err(viu_dev->dev,
			"%s: unsupported field type: %d\n",
			__func__, fmt->field);
		return -EINVAL;
	}

	writel(dma_inc,    viu_dev->base + VIU_DMA_INC);
	writel(field_addr, viu_dev->base + VIU_DMA_ADDR);

	/* activate DMA only durning vertial blank*/
	scr |= SCR_DMA_ACT;
	writel(scr, viu_dev->base + VIU_SCR);

	return 0;
}

static int imx_viu_vidioc_querycap(struct file *file, void *fh,
				   struct v4l2_capability *cap)
{
	struct imx_viu_device *viu_dev = video_drvdata(file);

	strlcpy(cap->driver, IMX_VIU_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->card, "V4L2 i.MX VIU", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", dev_name(viu_dev->dev));

	return 0;
}

static int imx_viu_vidioc_enum_input(struct file *filp, void *fh,
				     struct v4l2_input *inp)
{
	const struct imx_viu_input *input;

	if (inp->index >= ARRAY_SIZE(inputs))
		return -EINVAL;

	input = &inputs[inp->index];
	WARN_ON(input->index != inp->index);

	inp->type = input->type;
	inp->capabilities = input->caps;
	strlcpy(inp->name, input->name, sizeof(inp->name));
	inp->std = (input->caps & V4L2_IN_CAP_STD) ?
		   (V4L2_STD_NTSC_M | V4L2_STD_PAL) : 0;

	return 0;
}

static int imx_viu_vidioc_g_input(struct file *filp, void *fh, unsigned int *i)
{
	struct imx_viu_device *viu_dev = video_drvdata(filp);

	*i = viu_dev->curr_input;

	return 0;
}

static int imx_viu_vidioc_s_input(struct file *filp, void *fh, unsigned int i)
{
	struct imx_viu_device *viu_dev = video_drvdata(filp);

	if (i >= ARRAY_SIZE(inputs))
		return -EINVAL;

	/* 's_input' need to be called at the beginning */
	if (vb2_is_busy(&viu_dev->queue))
		return -EBUSY;

	viu_dev->curr_input = i;

	return 0;
}

static int imx_viu_vidioc_enum_fmt_vid_cap(struct file *filp, void *fh,
					   struct v4l2_fmtdesc *f)
{
	int ret;
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_subdev *v4l2_sd = viu_dev->v4l2_sd;
	const struct imx_viu_fmt *viu_fmt;
	struct v4l2_subdev_mbus_code_enum  code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		/* format index during enumeration */
		.index = f->index,
	};

	ret = v4l2_subdev_call(v4l2_sd, pad, enum_mbus_code, NULL, &code);
	if (ret)
		return -EINVAL;

	viu_fmt = viu_fmt_by_mbus(code.code);
	if (!viu_fmt) {
		dev_err(viu_dev->dev, "mbus code %#x invalid\n", code.code);
		return -EINVAL;
	}

	strlcpy(f->description, viu_fmt->name, sizeof(f->description));
	f->pixelformat = viu_fmt->pix_fmt;

	return 0;
}

static int imx_viu_vidioc_try_fmt_vid_cap(struct file *filp, void *fh,
					  struct v4l2_format *f)
{
	int ret;
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_subdev *v4l2_sd = viu_dev->v4l2_sd;
	struct v4l2_pix_format *v4l2_pix_fmt = &f->fmt.pix;
	const struct imx_viu_fmt *viu_fmt;
	struct v4l2_subdev_format v4l2_sd_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	viu_fmt = viu_fmt_by_pix_fmt(v4l2_pix_fmt->pixelformat);
	if (!viu_fmt)
		return -EINVAL;

	if (!v4l2_pix_fmt->width || !v4l2_pix_fmt->height) {
		dev_err(viu_dev->dev, "invalid with or height\n");
		return -EINVAL;
	}

	v4l2_fill_mbus_format(&v4l2_sd_fmt.format,
			      v4l2_pix_fmt,
			      viu_fmt->mbus_code);

	ret = v4l2_subdev_call(v4l2_sd, pad, set_fmt, NULL, &v4l2_sd_fmt);
	if (ret)
		return -EINVAL;

	v4l2_fill_pix_format(v4l2_pix_fmt, &v4l2_sd_fmt.format);

	/* TODO: set field none */
	v4l2_pix_fmt->field = V4L2_FIELD_NONE;

#if OUTPUT_IS_32BIT
	v4l2_pix_fmt->sizeimage = (32 >> 3) *
				  v4l2_pix_fmt->height *
				  v4l2_pix_fmt->width;
	v4l2_pix_fmt->bytesperline = (32 >> 3) * v4l2_pix_fmt->width;
#else
	v4l2_pix_fmt->sizeimage = (viu_fmt->bpp >> 3) *
				  v4l2_pix_fmt->height *
				  v4l2_pix_fmt->width;
	v4l2_pix_fmt->bytesperline = (viu_fmt->bpp >> 3) * v4l2_pix_fmt->width;
#endif

	return 0;
}

static int imx_viu_vidioc_g_fmt_vid_cap(struct file *filp, void *fh,
					struct v4l2_format *f)
{
	struct imx_viu_device *viu_dev = video_drvdata(filp);

	f->fmt.pix = viu_dev->v4l2_pix_fmt;

	return 0;
}

static int imx_viu_vidioc_s_fmt_vid_cap(struct file *filp, void *fh,
					struct v4l2_format *f)
{
	int ret;
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_pix_format *v4l2_pix_fmt = &f->fmt.pix;
	const struct imx_viu_fmt *viu_fmt;

	ret = imx_viu_vidioc_try_fmt_vid_cap(filp, viu_dev, f);
	if (ret < 0)
		return ret;

	viu_fmt = viu_fmt_by_pix_fmt(v4l2_pix_fmt->pixelformat);
	viu_dev->viu_fmt   = *viu_fmt;
	viu_dev->mbus_code = viu_fmt->mbus_code;
	viu_dev->v4l2_pix_fmt.pixelformat = v4l2_pix_fmt->pixelformat;
	viu_dev->v4l2_pix_fmt.width	= v4l2_pix_fmt->width;
	viu_dev->v4l2_pix_fmt.height	= v4l2_pix_fmt->height;
	viu_dev->v4l2_pix_fmt.sizeimage	= v4l2_pix_fmt->sizeimage;
	viu_dev->v4l2_pix_fmt.bytesperline = v4l2_pix_fmt->bytesperline;
	viu_dev->v4l2_pix_fmt.field	= v4l2_pix_fmt->field;
	viu_dev->buf_type		= f->type;

	return 0;
}

static int imx_viu_vidioc_g_parm(struct file *filp, void *fh,
				 struct v4l2_streamparm *a)
{
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_subdev *v4l2_sd = viu_dev->v4l2_sd;

	return v4l2_subdev_call(v4l2_sd, video, g_parm, a);
}

static int imx_viu_vidioc_s_parm(struct file *filp, void *fh,
				 struct v4l2_streamparm *a)
{
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_subdev *v4l2_sd = viu_dev->v4l2_sd;

	return v4l2_subdev_call(v4l2_sd, video, s_parm, a);
}

static int imx_viu_vidioc_enum_framesizes(struct file *filp, void *fh,
					  struct v4l2_frmsizeenum *fsize)
{
	int ret;
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_subdev *v4l2_sd = viu_dev->v4l2_sd;
	const struct imx_viu_fmt *viu_fmt;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	viu_fmt  = viu_fmt_by_pix_fmt(fsize->pixel_format);
	fse.code = viu_fmt->mbus_code;

	ret = v4l2_subdev_call(v4l2_sd, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width  = fse.max_width;
	fsize->discrete.height = fse.max_height;

	return 0;
}

static int imx_viu_vidioc_enum_frameintervals(struct file *filp, void *fh,
					      struct v4l2_frmivalenum *fival)
{
	int ret;
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_subdev *v4l2_sd = viu_dev->v4l2_sd;
	const struct imx_viu_fmt *viu_fmt;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index  = fival->index,
		.width  = fival->width,
		.height = fival->height,
		.which  = V4L2_SUBDEV_FORMAT_ACTIVE,
	};

	viu_fmt  = viu_fmt_by_pix_fmt(fival->pixel_format);
	fie.code = viu_fmt->mbus_code;

	ret = v4l2_subdev_call(v4l2_sd, pad, enum_frame_interval, NULL, &fie);
	if (ret)
		return ret;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = fie.interval;

	return 0;
}

static const struct v4l2_ioctl_ops imx_viu_ioctl_ops = {
	.vidioc_querycap		= imx_viu_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap	= imx_viu_vidioc_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= imx_viu_vidioc_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= imx_viu_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= imx_viu_vidioc_s_fmt_vid_cap,

	.vidioc_enum_input		= imx_viu_vidioc_enum_input,
	.vidioc_g_input			= imx_viu_vidioc_g_input,
	.vidioc_s_input			= imx_viu_vidioc_s_input,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_g_parm			= imx_viu_vidioc_g_parm,
	.vidioc_s_parm			= imx_viu_vidioc_s_parm,
	.vidioc_enum_framesizes		= imx_viu_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals	= imx_viu_vidioc_enum_frameintervals,
};

static int imx_viu_open(struct file *filp)
{
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct video_device *vdev = video_devdata(filp);
	struct imx_viu_fh *viu_fh;

	if (mutex_lock_interruptible(&viu_dev->lock))
		return -ERESTARTSYS;

	viu_fh = kzalloc(sizeof(*viu_fh), GFP_KERNEL);
	if (!viu_fh) {
		dev_err(viu_dev->dev, "Cannot allocate 'viu_fh' struct\n");
		mutex_unlock(&viu_dev->lock);
		return -ENOMEM;
	}

	/* first open: VIU should be clean */
	if (atomic_inc_return(&viu_dev->use_count) == 1) {
		pm_runtime_get_sync(viu_dev->dev);
		viu_dev->frame_count = 0;
	}

	v4l2_fh_init(&viu_fh->fh, vdev);
	filp->private_data = &viu_fh->fh;
	v4l2_fh_add(&viu_fh->fh);

	mutex_unlock(&viu_dev->lock);

	return 0;
}

static int imx_viu_release(struct file *filp)
{
	struct imx_viu_device *viu_dev = video_drvdata(filp);
	struct v4l2_fh *fh = filp->private_data;
	struct imx_viu_fh *viu_fh;

	if (!fh) {
		WARN_ON(1);
		return 0;
	}

	mutex_lock(&viu_dev->lock);

	viu_fh = container_of(fh, struct imx_viu_fh, fh);

	if (atomic_dec_and_test(&viu_dev->use_count)) {
		pm_runtime_put(viu_dev->dev);
		mutex_unlock(&viu_dev->lock);
		vb2_fop_release(filp);
	} else
		mutex_unlock(&viu_dev->lock);

	return 0;
}

static const struct v4l2_file_operations imx_viu_fops = {
	.owner	= THIS_MODULE,
	.open	= imx_viu_open,
	.release = imx_viu_release,
	.unlocked_ioctl = video_ioctl2,
	.read	= vb2_fop_read,
	.mmap	= vb2_fop_mmap,
	.poll	= vb2_fop_poll,
};

static int imx_viu_queue_setup(struct vb2_queue *q,
			       unsigned int *num_buffers,
			       unsigned int *num_planes,
			       unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct imx_viu_device *viu_dev = vb2_get_drv_priv(q);
	struct v4l2_pix_format *fmt = &viu_dev->v4l2_pix_fmt;

	/* TODO: don't support multiple plane format */

	WARN_ON(*num_buffers < 3);

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_UYVY:
		if (!*num_planes || *num_planes > 1)
			*num_planes = 1;
		sizes[0] = fmt->sizeimage;
		break;
	case V4L2_PIX_FMT_YUYV:
		if (!*num_planes || *num_planes > 1)
			*num_planes = 1;
		sizes[0] = fmt->sizeimage;
		break;
	case V4L2_PIX_FMT_ABGR32:
		if (!*num_planes || *num_planes > 1)
			*num_planes = 1;
		sizes[0] = fmt->sizeimage;
		break;
	default:
		/* unsupported format */
		return -EINVAL;
	}

	return 0;
}

#if 0
static void imx_viu_wait_prepare(struct vb2_queue *q)
{
}

static void imx_viu_wait_finish(struct vb2_queue *q)
{
}
#endif

static int imx_viu_buf_prepare(struct vb2_buffer *vb)
{
	struct imx_viu_device *viu_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format *fmt = &viu_dev->v4l2_pix_fmt;
	unsigned int plane_no = 0;

	if (WARN_ON(vb->num_planes != 1))
		return -EINVAL;

	if (vb2_plane_size(vb, plane_no) < fmt->sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, plane_no, fmt->sizeimage);

	return 0;
}

static int imx_viu_start_streaming(struct vb2_queue *q,
				   unsigned int count)
{
	int ret = 0;
	unsigned long flags;
	struct imx_viu_device *viu_dev = vb2_get_drv_priv(q);
	struct imx_viu_buffer *viu_buf;
	struct vb2_buffer *vb;
	dma_addr_t dma_addr;

	if (WARN_ON(count < 3))
		return -ENOBUFS;

	/* discard buffers for no buffer available */
	viu_dev->discard_size = viu_dev->v4l2_pix_fmt.sizeimage;
	viu_dev->discard_buffer = dma_alloc_coherent(viu_dev->v4l2_dev.dev,
					PAGE_ALIGN(viu_dev->discard_size),
					&viu_dev->discard_buffer_dma,
					GFP_DMA | GFP_KERNEL);
	if (!viu_dev->discard_buffer)
		return -ENOMEM;

	spin_lock_irqsave(&viu_dev->slock, flags);

	/* queue the discard buffer first */
	viu_dev->buf_discard.discard = true;
	list_add_tail(&viu_dev->buf_discard.queue,
		      &viu_dev->discard);

	if (unlikely(list_empty(&viu_dev->active_queue))) {
		WARN_ON(1);
		spin_unlock_irqrestore(&viu_dev->slock, flags);
		return -ENOBUFS;
	}

	viu_buf = list_first_entry(&viu_dev->active_queue,
				   struct imx_viu_buffer,
				   internal.queue);

	vb = &viu_buf->vb.vb2_buf;
	dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	viu_buf->field = 0;

	spin_unlock_irqrestore(&viu_dev->slock, flags);

	ret = imx_viu_set_fmt(viu_dev);
	if (ret)
		return ret;

	/* wait for the first field of a frame */
	ret = imx_viu_wait_for_field(viu_dev, 0);
	if (ret)
		return ret;

	/* enable ERROR interrupt here */
	imx_viu_irq_enable(viu_dev, ERROR_IRQ);
	/* enable dma transfer */
	imx_viu_irq_enable(viu_dev, DMA_END_IRQ);
	ret = imx_viu_config_dma(viu_dev, dma_addr, 0);
	if (ret)
		return ret;

	return ret;
}

static void imx_viu_stop_streaming(struct vb2_queue *q)
{
	int ret;
	unsigned long flags;
	struct imx_viu_device *viu_dev = vb2_get_drv_priv(q);
	struct imx_viu_buffer *pos, *tmp;
	struct vb2_buffer *vb;
	void *tmpbuf;

	/* It shall only be configured when DMA is
	 * inactive, during vertical blanking.
	 */
	ret = wait_for_completion_timeout(&viu_dev->dma_done, HZ / 10);
	if (!ret) {
		dev_err(viu_dev->dev, "wait for dma done timeout\n");
	}
	/* disable ERROR irq to make release smooth */
	imx_viu_irq_disable(viu_dev, ERROR_IRQ);
	imx_viu_irq_disable(viu_dev, DMA_END_IRQ);

	spin_lock_irqsave(&viu_dev->slock, flags);

	if (list_empty(&viu_dev->active_queue))
		WARN_ON(1);

	list_for_each_entry_safe(pos, tmp,
			&viu_dev->active_queue, internal.queue) {

		list_del_init(&pos->internal.queue);
		vb = &pos->vb.vb2_buf;
		if (vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&viu_dev->active_queue);
	INIT_LIST_HEAD(&viu_dev->discard);

	tmpbuf = viu_dev->discard_buffer;
	viu_dev->discard_buffer = NULL;

	spin_unlock_irqrestore(&viu_dev->slock, flags);

	dma_free_coherent(viu_dev->v4l2_dev.dev,
				viu_dev->discard_size, tmpbuf,
				viu_dev->discard_buffer_dma);
}

static void imx_viu_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct imx_viu_device *viu_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imx_viu_buffer *viu_buf = container_of(vbuf,
						struct imx_viu_buffer,
						vb);
	unsigned long flags;

	spin_lock_irqsave(&viu_dev->slock, flags);

	/* the 'vb' is in active state, so add this
	 * buffer to 'active_queue' to be fetched
	 * when streamon called
	 */
	list_add_tail(&viu_buf->internal.queue, &viu_dev->active_queue);

	spin_unlock_irqrestore(&viu_dev->slock, flags);
}

static struct vb2_ops imx_viu_vb2_ops = {
	.queue_setup		= imx_viu_queue_setup,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_prepare		= imx_viu_buf_prepare,
	.start_streaming	= imx_viu_start_streaming,
	.stop_streaming		= imx_viu_stop_streaming,
	.buf_queue		= imx_viu_buf_queue,
};

static int imx_viu_dma_done_handle(struct imx_viu_device *viu_dev)
{
	int ret;
	uint32_t field = 0;
	unsigned long flags;
	struct imx_viu_buffer *buf;
	struct viu_buf_internal *ibuf;
	struct vb2_buffer *vb;
	dma_addr_t dma_addr;

	spin_lock_irqsave(&viu_dev->slock, flags);

	if (unlikely(list_empty(&viu_dev->active_queue))) {
		WARN_ON(1);
		spin_unlock_irqrestore(&viu_dev->slock, flags);
		return -ENOBUFS;
	}

	ibuf = list_first_entry(&viu_dev->active_queue, struct viu_buf_internal,
				queue);

	if (ibuf->discard) {

		/* discard buffer just return to discard queue */
		/* not dqbuf to user */
		list_move_tail(viu_dev->active_queue.next, &viu_dev->discard);
	} else {
		/* make vb2 know about the buffer done, can return to user */
		buf = viu_ibuf_to_buf(ibuf);
		vb = &buf->vb.vb2_buf;

		/* delete current buffer from queue */
		list_del_init(&buf->internal.queue);
		to_vb2_v4l2_buffer(vb)->sequence = viu_dev->frame_count;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}

	viu_dev->frame_count++;

	if (list_empty(&viu_dev->active_queue)) {

		if (list_empty(&viu_dev->discard)) {
			spin_unlock_irqrestore(&viu_dev->slock, flags);
			dev_err(viu_dev->dev,
				"trying to access empty discard list\n");
			return 0;
		}

		list_move_tail(viu_dev->discard.next, &viu_dev->active_queue);

		imx_viu_config_dma(viu_dev, viu_dev->discard_buffer_dma, 0);
		spin_unlock_irqrestore(&viu_dev->slock, flags);

		return 0;
	}

	buf = list_first_entry(&viu_dev->active_queue, struct imx_viu_buffer,
				internal.queue);

	vb = &buf->vb.vb2_buf;
	vb->state = VB2_BUF_STATE_ACTIVE;

	dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	ret = imx_viu_config_dma(viu_dev, dma_addr, field);
	spin_unlock_irqrestore(&viu_dev->slock, flags);
	return ret;
}

static irqreturn_t imx_viu_irq_handler(int irq, void *dev_id)
{
	unsigned int scr;
	static bool recover = false;
	struct imx_viu_device *viu_dev = dev_id;
	int errcode;

	scr = readl(viu_dev->base + VIU_SCR);

	if (WARN_ON(!(scr & SCR_IRQ_MASK))) {
		dev_err(viu_dev->dev, "interrupt is not from VIU\n");
		return IRQ_NONE;
	}

	/* write 1 clear irqs */
	writel(scr, viu_dev->base + VIU_SCR);

	if (scr & SCR_ERROR_IRQ) {
		dev_warn(viu_dev->dev, "ERROR IRQ %#x, reset VIU\n",
				      SCR_GET_ERROR_CODE(scr));
		imx_viu_soft_reset(viu_dev);
		errcode = SCR_GET_ERROR_CODE(scr);
		if (0x1 == errcode || 0x2 == errcode) {
			/* DMA_ACT is set during vertical active, recover */
			recover = true;
			imx_viu_irq_enable(viu_dev, VSYNC_IRQ);

			if (0x2 == errcode) {
				writel(scr & ~SCR_DMA_ACT, viu_dev->base + VIU_SCR);
			}
		}
		return IRQ_HANDLED;
	}

	if (scr & SCR_DMA_END_IRQ) {
		imx_viu_dma_done_handle(viu_dev);
		complete(&viu_dev->dma_done);
		return IRQ_HANDLED;
	}
#if 0
	if (scr & SCR_VSTART_IRQ) {
		dev_dbg(viu_dev->dev, "vstart irq happens\n");
		return IRQ_HANDLED;
	}
#endif
#if 0
	if (scr & SCR_HSYNC_IRQ) {
		dev_dbg(viu_dev->dev, "hsync irq happens\n");
	}
#endif

	if (scr & SCR_VSYNC_IRQ) {
		if (recover) {
			scr |= SCR_DMA_ACT;
			scr &= ~SCR_VSYNC_EN;
			writel(scr, viu_dev->base + VIU_SCR);
			recover = false;
		}
		complete(&viu_dev->vsync);
		return IRQ_HANDLED;
	}

	if (scr & SCR_FIELD_IRQ) {
		complete(&viu_dev->field);
	}

	return IRQ_HANDLED;
}

static int imx_viu_subdev_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct imx_viu_device *viu_dev = notifier_to_viu_dev(notifier);

	/* TODO support multiple subdevs */
	viu_dev->v4l2_sd = subdev;

	dev_info(viu_dev->dev, "subdev %s bound success\n", subdev->name);

	return 0;
}

static void imx_viu_subdev_unbind(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_subdev *asd)
{
	struct imx_viu_device *viu_dev = notifier_to_viu_dev(notifier);

	viu_dev->v4l2_sd = NULL;

	dev_info(viu_dev->dev, "subdev %s unbind\n", subdev->name);
}

static int imx_viu_async_subdevs_register(struct imx_viu_device *viu_dev)
{
	int ret;
	unsigned int num_subdevs = 0;
	struct fwnode_handle *endpoint = NULL;
	struct fwnode_handle *np = dev_fwnode(viu_dev->dev);
	struct fwnode_handle *remote = NULL;
	struct v4l2_async_notifier *notifier = &viu_dev->notifier;
	struct v4l2_async_subdev *asd;

	while (num_subdevs < IMX_VIU_MAX_SUBDEV_NUM) {
		endpoint = fwnode_graph_get_next_endpoint(np, endpoint);
		if (!endpoint)
			break;

		remote = fwnode_graph_get_remote_port_parent(endpoint);

		fwnode_handle_put(remote);
		if (!remote)
			continue;

		asd = &viu_dev->subdevs[num_subdevs];
		asd->match_type = V4L2_ASYNC_MATCH_FWNODE;
		asd->match.fwnode.fwnode = remote;
		viu_dev->async_subdevs[num_subdevs] = asd;

		num_subdevs++;
	}

	if (unlikely(endpoint))
		fwnode_handle_put(endpoint);

	if (!num_subdevs) {
		dev_err(viu_dev->dev, "no subdev found for viu\n");
		return -ENODEV;
	}

	notifier->subdevs = viu_dev->async_subdevs;
	notifier->num_subdevs = num_subdevs;
	notifier->bound  = imx_viu_subdev_bound;
	notifier->unbind = imx_viu_subdev_unbind;

	ret = v4l2_async_notifier_register(&viu_dev->v4l2_dev, notifier);
	if (ret) {
		dev_err(viu_dev->dev, "register async notifier failed\n");
		return -EINVAL;
	}

	return 0;
}

static int imx_viu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct imx_viu_device *viu_dev;
	struct resource *res;
	struct vb2_queue *vb2q;

	viu_dev = devm_kzalloc(&pdev->dev, sizeof(*viu_dev), GFP_KERNEL);
	if (!viu_dev) {
		dev_err(&pdev->dev, "Can't allocate 'viu_dev' struct\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	viu_dev->irq = platform_get_irq(pdev, 0);
	if (viu_dev->irq < 0)
		return -ENODEV;

	ret = devm_request_irq(&pdev->dev, viu_dev->irq,
			       imx_viu_irq_handler, IRQF_SHARED | IRQF_NO_THREAD,
			       "imx-viu", viu_dev);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
		return ret;
	}

	viu_dev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(viu_dev->base))
		return PTR_ERR(viu_dev->base);

	viu_dev->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(viu_dev->ipg_clk))
		ret = PTR_ERR(viu_dev->ipg_clk);

	viu_dev->dev = &pdev->dev;

	mutex_init(&viu_dev->lock);

	/* Register 'v4l2_device' */
	snprintf(viu_dev->v4l2_dev.name,
		 sizeof(viu_dev->v4l2_dev.name), "IMX-VIU");
#if defined(CONFIG_MEDIA_CONTROLLER)
	/* TODO Add media controller support */
#endif
	ret = v4l2_device_register(viu_dev->dev, &viu_dev->v4l2_dev);

	/* Initialize the vb2 queue */
	vb2q = &viu_dev->queue;
	vb2q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vb2q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	vb2q->dev = viu_dev->dev;
	vb2q->drv_priv = viu_dev;
	vb2q->buf_struct_size = sizeof(struct imx_viu_buffer);
	vb2q->min_buffers_needed = 3;
	vb2q->ops = &imx_viu_vb2_ops;
	vb2q->mem_ops = &vb2_dma_contig_memops;
	vb2q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vb2q->lock = &viu_dev->lock;

	/* TODO other possible configurable fields:
	 *	'buf_struct_size';
	 *	'gfp_flags';
	 *	'min_buffers_needed';
	 *	'alloc_devs'
	 */ 
	ret = vb2_queue_init(vb2q);
	if (ret)
		goto unregister_v4l2_dev;

	/* Register 'video_device' */
	viu_dev->vdev = video_device_alloc();
	if (!viu_dev->vdev) {
		ret = -ENOMEM;
		goto unregister_v4l2_dev;
	}

	strlcpy(viu_dev->vdev->name, "IMX viu", sizeof(viu_dev->vdev->name));

	viu_dev->vdev->release	 = video_device_release;
	viu_dev->vdev->v4l2_dev	 = &viu_dev->v4l2_dev;
	viu_dev->vdev->vfl_dir	 = VFL_DIR_RX;
	viu_dev->vdev->fops	 = &imx_viu_fops;
	viu_dev->vdev->ioctl_ops = &imx_viu_ioctl_ops;
	viu_dev->vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE |
				     V4L2_CAP_STREAMING |
				     V4L2_CAP_READWRITE;
	viu_dev->vdev->lock	 = &viu_dev->lock;
	viu_dev->vdev->queue	 = vb2q;

	video_set_drvdata(viu_dev->vdev, viu_dev);

	ret = video_register_device(viu_dev->vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto release_video_device;

	/* Register async subdevs */
	ret = imx_viu_async_subdevs_register(viu_dev);
	if (ret) {
		dev_err(viu_dev->dev, "register async subdevs failed\n");
		goto release_video_device;
	}

	pm_runtime_enable(viu_dev->dev);
	init_completion(&viu_dev->field);
	init_completion(&viu_dev->dma_done);
	init_completion(&viu_dev->vsync);
	spin_lock_init(&viu_dev->slock);

	/* save all the VIU registers reset value
	 * One time save, any times restore
	 */
	pm_runtime_get_sync(viu_dev->dev);

	imx_viu_irq_init(viu_dev);
	INIT_LIST_HEAD(&viu_dev->active_queue);
	INIT_LIST_HEAD(&viu_dev->discard);

	dev_info(viu_dev->dev, "IMX VIU Driver probe success\n");

	return 0;

release_video_device:
	video_device_release(viu_dev->vdev);
unregister_v4l2_dev:
	v4l2_device_unregister(&viu_dev->v4l2_dev);

	return ret;
}

static int imx_viu_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = dev_get_drvdata(&pdev->dev);
	struct imx_viu_device *viu_dev = v4l2_dev_to_viu_dev(v4l2_dev);

	video_device_release(viu_dev->vdev);
	v4l2_device_unregister(&viu_dev->v4l2_dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP 
static int imx_viu_suspend(struct device *dev)
{
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct imx_viu_device *viu_dev = v4l2_dev_to_viu_dev(v4l2_dev);
	int ret;

	pr_info("imx_viu_suspend\n");

	if (!vb2_is_streaming(&viu_dev->queue))
		return 0;

	ret = wait_for_completion_timeout(&viu_dev->dma_done, HZ / 10);
	if (!ret) {
		dev_err(dev, "wait for dma done timeout\n");
	}
	/* disable ERROR irq to make release smooth */
	imx_viu_irq_disable(viu_dev, ERROR_IRQ);
	imx_viu_irq_disable(viu_dev, DMA_END_IRQ);

	pinctrl_pm_select_sleep_state(dev);

	/* save registers for resume */
	imx_viu_save_reg_stack(viu_dev, &viu_dev->reset);

	clk_disable_unprepare(viu_dev->ipg_clk);
	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int imx_viu_resume(struct device *dev)
{
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct imx_viu_device *viu_dev = v4l2_dev_to_viu_dev(v4l2_dev);

	pr_info("imx_viu_resume\n");

	if (!vb2_is_streaming(&viu_dev->queue))
		return 0;

	request_bus_freq(BUS_FREQ_HIGH);
	clk_prepare_enable(viu_dev->ipg_clk);

	/* resume registers' value */
	imx_viu_restore_reg_stack(viu_dev, &viu_dev->reset);
	imx_viu_irq_enable(viu_dev, DMA_END_IRQ);
	imx_viu_irq_enable(viu_dev, ERROR_IRQ);

	pinctrl_pm_select_default_state(dev);

	return 0;
}
#else
#define imx_viu_suspend		NULL
#define imx_viu_resume		NULL
#endif

#ifdef CONFIG_PM
static int imx_viu_runtime_suspend(struct device *dev)
{
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct imx_viu_device *viu_dev = v4l2_dev_to_viu_dev(v4l2_dev);

	clk_disable_unprepare(viu_dev->ipg_clk);

	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int imx_viu_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct imx_viu_device *viu_dev = v4l2_dev_to_viu_dev(v4l2_dev);

	request_bus_freq(BUS_FREQ_HIGH);

	ret = clk_prepare_enable(viu_dev->ipg_clk);

	if (ret)
		release_bus_freq(BUS_FREQ_HIGH);

	return ret;
}
#else
#define imx_viu_runtime_suspend	NULL
#define imx_viu_runtime_resume	NULL
#endif

static const struct dev_pm_ops imx_viu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx_viu_suspend, imx_viu_resume)
	SET_RUNTIME_PM_OPS(imx_viu_runtime_suspend, imx_viu_runtime_resume, NULL)
};

static const struct of_device_id imx_viu_of_match[] = {
	{ .compatible = "fsl,imx7ulp-viu", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_viu_of_match);

static struct platform_driver imx_viu_driver = {
	.probe    = imx_viu_probe,
	.remove   = imx_viu_remove,
	.driver   = {
		.name = IMX_VIU_DRV_NAME,
		.of_match_table = of_match_ptr(imx_viu_of_match),
		.pm = &imx_viu_pm_ops,
	},
};

module_platform_driver(imx_viu_driver);

MODULE_DESCRIPTION("NXP i.MX VIU driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
