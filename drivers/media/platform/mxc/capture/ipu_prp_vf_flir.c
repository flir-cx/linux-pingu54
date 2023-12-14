// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2004-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/* * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_prp_vf_flir.c
 *
 * @brief IPU Use case for PRP-VF
 *
 * @ingroup IPU
 */

#include <linux/dma-mapping.h>
#include <linux/console.h>
#include <linux/ipu.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/mipi_csi2.h>
#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"

extern void ipu_indicate_new_buffer(void);
extern int32_t ipu_is_channel_active(struct ipu_soc *ipu, ipu_channel_t channel);

static int lcd_buffer_num;
static int dp_buffer_num;
static struct ipu_soc *disp_ipu;
static u32 paddr[3];

#if defined(CONFIG_MXC_CAMERA_FLIR)
#define IPU_DEF_FORMAT	IPU_PIX_FMT_BGR32
#define IPU_PIX_SIZE	4
#else
#define IPU_DEF_FORMAT	IPU_PIX_FMT_UYVY
#define IPU_PIX_SIZE	2
#endif

static void get_disp_ipu(cam_data *cam)
{
	if (cam->output > 2)
		disp_ipu = ipu_get_soc(1); /* using DISP4 */
	else
		disp_ipu = ipu_get_soc(0);
}

static irqreturn_t prpvf_rot_eof_callback(int irq, void *dev_id)
{
	cam_data *cam = dev_id;
	uint32_t cur;
	uint32_t nbuf;
	static uint32_t repair;
	static uint32_t lastActive;
	uint32_t nowActive;
#ifdef CONFIG_MXC_EPIT_BOOTTIME
	extern u32 board_get_time(void);
	static int init;

	if (!init) {
		pr_info("first_csi_frame boottime %d\n", board_get_time());
		init = 1;
	}
#endif

	/* if ((cam->supervisor_ctr || repair) && (irq > 0)) { */
	/*	// Back in buissiness after error, reset buffer usage which might get stuck */
	/*	ipu_clear_buffer_ready(cam->ipu, CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 0); */
	/*	ipu_clear_buffer_ready(cam->ipu, CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 1); */
	/*	ipu_clear_buffer_ready(cam->ipu, CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, 2); */
	/*	ipu_clear_buffer_ready(cam->ipu, CSI_PRP_VF_MEM, IPU_GRAPH_IN_BUFFER, 0); */
	/*	ipu_clear_buffer_ready(cam->ipu, CSI_PRP_VF_MEM, IPU_GRAPH_IN_BUFFER, 1); */
	/*	ipu_clear_buffer_ready(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER, 0); */
	/*	ipu_clear_buffer_ready(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER, 1); */
	/*	ipu_clear_buffer_ready(disp_ipu, MEM_DC_SYNC, IPU_INPUT_BUFFER, 0); */
	/*	ipu_clear_buffer_ready(disp_ipu, MEM_DC_SYNC, IPU_INPUT_BUFFER, 1); */
	/*	ipu_clear_buffer_ready(disp_ipu, MEM_DC_SYNC, IPU_INPUT_BUFFER, 2); */
	/*	repair = 0; */
	/* } else  */
	if (irq <= 0) {
		// Error has occurred, indicate reset of buffers needed
		repair = 1;
	}

	// Update CSI output buffer (tripple buffered)
	cur = ipu_get_cur_buffer_idx(cam->ipu, CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER);
	nbuf = (cur + 1) % 3;
	ipu_update_channel_buffer(cam->ipu, CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, nbuf, paddr[nbuf]);
	ipu_select_buffer(cam->ipu, CSI_PRP_VF_MEM, IPU_OUTPUT_BUFFER, nbuf);

	if (irq > 0) {
		// Update main (LCD/ViewFinder) output buffer
		lcd_buffer_num = !lcd_buffer_num;
		ipu_update_channel_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER,
					  lcd_buffer_num, paddr[cur]);
		ipu_select_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER, lcd_buffer_num);

		// Update secondary (DisplayPort) output buffer
		nowActive = ipu_is_channel_active(disp_ipu, MEM_DC_SYNC);
		if (nowActive != lastActive) {
			if (nowActive)
				repair = 1;
			lastActive = nowActive;
		}
		if (nowActive) {
			if (++dp_buffer_num >= 3)
				dp_buffer_num = 0;
			ipu_update_channel_buffer(disp_ipu, MEM_DC_SYNC, IPU_INPUT_BUFFER,
						  dp_buffer_num, paddr[cur]);
			ipu_select_buffer(disp_ipu, MEM_DC_SYNC, IPU_INPUT_BUFFER, dp_buffer_num);
		}

		ipu_indicate_new_buffer();
	}

	/* if (cam->supervisor_ctr) { */
	/*	cam->supervisor_major = 0; */
	/*	if (irq > 0) { */
	/*		if (--cam->supervisor_ctr == 0) */
	/*			cancel_delayed_work(&cam->supervise_start_wq); */
	/*	} else { */
	/*		cam->supervisor_ctr = 3; */
	/*	} */
	/* } */
	return IRQ_HANDLED;
}
/*
 * Function definitions
 */

/*!
 * prpvf_start - start the vf task
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 */
static int prpvf_start(void *private)
{
	struct fb_var_screeninfo fbvar_out, fbvar_ovl;
	struct fb_info *fbi_out = NULL;
	struct fb_info *fbi_ovl = NULL;
	struct fb_info *fbi_dp = NULL;
	cam_data *cam = (cam_data *) private;
	ipu_channel_params_t vf;
	u32 vf_out_format = 0;
	u32 size = IPU_PIX_SIZE;
	int err = 0;
	int i = 0;
	short color;
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;
	int ipu_id;
	int csi_id;
#endif

	if (!cam) {
		dev_err(cam->dev, "private is NULL\n");
		return -EIO;
	}

	if (cam->overlay_active == true) {
		pr_debug("already started.\n");
		return 0;
	}

	get_disp_ipu(cam);

	for (i = 0; i < num_registered_fb; i++) {
		char *idstr = registered_fb[i]->fix.id;

		if ((fbi_out == NULL) &&
		    (((strcmp(idstr, "DISP3 FG") == 0) && (cam->output < 3)) ||
		     ((strcmp(idstr, "DISP4 FG") == 0) && (cam->output >= 3)))) {
			fbi_out = registered_fb[i];
		} else if ((fbi_ovl == NULL) &&
			 (((strcmp(idstr, "DISP3 XX") == 0) && (cam->output < 3)) ||
			  ((strcmp(idstr, "DISP4 XX") == 0) && (cam->output >= 3)) ||
			  ((strcmp(idstr, "simple") == 0)))) {
			fbi_ovl = registered_fb[i];
		} else if ((fbi_dp == NULL) &&
		    (((strcmp(idstr, "DISP3 BG") == 0) && (cam->output < 3)) ||
		     ((strcmp(idstr, "DISP4 BG") == 0) && (cam->output >= 3)))) {
			fbi_dp = registered_fb[i];
		}
	}

	if (fbi_out == NULL) {
		dev_err(cam->dev, "DISP FG fb not found\n");
		return -EPERM;
	}
	if (fbi_ovl == NULL) {
		dev_err(cam->dev, "DISP XX fb not found\n");
		/* return -EPERM; */
	}

	fbvar_out = fbi_out->var;

	/* Store the overlay frame buffer's original std */
	cam->fb_origin_std = fbvar_out.nonstd;

	if (cam->devtype == IMX5_V4L2 || cam->devtype == IMX6_V4L2) {
		/* Use DP to do CSC so that we can get better performance */
		vf_out_format = IPU_DEF_FORMAT;
		fbvar_out.nonstd = vf_out_format;
		color = 0x80;
	} else {
		vf_out_format = IPU_PIX_FMT_RGB565;
		fbvar_out.nonstd = 0;
		color = 0x0;
	}

	fbvar_out.bits_per_pixel = IPU_PIX_SIZE * 8;
	fbvar_out.xres = fbvar_out.xres_virtual = cam->win.w.width;
	fbvar_out.yres = cam->win.w.height;
	fbvar_out.yres_virtual = cam->win.w.height * 3;
	fbvar_out.yoffset = 0;
	fbvar_out.accel_flags = FB_ACCEL_DOUBLE_FLAG;
	fbvar_out.activate |= FB_ACTIVATE_FORCE;
	fb_set_var(fbi_out, &fbvar_out);

	if (fbi_ovl) {
		/* Set up graphics frame buffer to be double buffered */
		fbvar_ovl = fbi_ovl->var;
		fbvar_ovl.xres = fbvar_ovl.xres_virtual = cam->win.w.width;
		fbvar_ovl.yres = cam->win.w.height;
		fbvar_ovl.yres_virtual = cam->win.w.height * 3;
		fbvar_ovl.yoffset = 0;
		fbvar_ovl.accel_flags = FB_ACCEL_DOUBLE_FLAG;
		fbvar_ovl.activate |= FB_ACTIVATE_FORCE;
		fb_set_var(fbi_ovl, &fbvar_ovl);
	}

	if (fbi_dp) {
		struct fb_var_screeninfo fbvar_dp;

		int nowActive = ipu_is_channel_active(disp_ipu, MEM_DC_SYNC);

		if (nowActive)
			fb_blank(fbi_dp, FB_BLANK_NORMAL);

		/* Set up displayport buffer to proper pixel format */
		fbvar_dp = fbi_dp->var;
		fbvar_dp.activate |= FB_ACTIVATE_FORCE;
		fbvar_dp.nonstd = vf_out_format;
		fb_set_var(fbi_dp, &fbvar_dp);

		if (nowActive)
			fb_blank(fbi_dp, FB_BLANK_UNBLANK);
	}

	ipu_disp_set_window_pos(disp_ipu, MEM_FG_SYNC, cam->win.w.left,
			cam->win.w.top);

	console_lock();
	fb_blank(fbi_out, FB_BLANK_UNBLANK);
	console_unlock();

	/* correct display ch buffer address */
	paddr[0] = fbi_out->fix.smem_start + (fbi_out->fix.line_length * fbi_out->var.yres);
	paddr[1] = fbi_out->fix.smem_start;
	paddr[2] = fbi_out->fix.smem_start + 2 * (fbi_out->fix.line_length * fbi_out->var.yres);

	ipu_update_channel_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER, 0, paddr[0]);
	ipu_update_channel_buffer(disp_ipu, MEM_FG_SYNC, IPU_INPUT_BUFFER, 1, paddr[1]);

	memset(&vf, 0, sizeof(ipu_channel_params_t));
	ipu_csi_get_window_size(cam->ipu, &vf.csi_prp_vf_mem.in_width,
				&vf.csi_prp_vf_mem.in_height, cam->csi);
	vf.csi_prp_vf_mem.in_pixel_fmt = IPU_DEF_FORMAT;
	vf.csi_prp_vf_mem.out_width = cam->win.w.width;
	vf.csi_prp_vf_mem.out_height = cam->win.w.height;
	vf.csi_prp_vf_mem.csi = cam->csi;
	vf.csi_prp_vf_mem.out_pixel_fmt = vf_out_format;
	size = cam->win.w.width * cam->win.w.height * size;

	if (fbi_ovl)
		vf.csi_prp_vf_mem.graphics_combine_en = 1;
	else
		vf.csi_prp_vf_mem.graphics_combine_en = 0;

	vf.csi_prp_vf_mem.global_alpha_en = 0;
	vf.csi_prp_vf_mem.key_color_en = 0;
	vf.csi_prp_vf_mem.in_g_pixel_fmt = IPU_PIX_FMT_BGRA32;
	vf.csi_prp_vf_mem.alpha_chan_en = 0;

#ifdef CONFIG_MXC_MIPI_CSI2
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info) {
		if (mipi_csi2_get_status(mipi_csi2_info)) {
			ipu_id = mipi_csi2_get_bind_ipu(mipi_csi2_info);
			csi_id = mipi_csi2_get_bind_csi(mipi_csi2_info);

			if (cam->ipu == ipu_get_soc(ipu_id)
				&& cam->csi == csi_id) {
				vf.csi_prp_vf_mem.mipi_en = true;
				vf.csi_prp_vf_mem.mipi_vc =
				mipi_csi2_get_virtual_channel(mipi_csi2_info);
				vf.csi_prp_vf_mem.mipi_id =
				mipi_csi2_get_datatype(mipi_csi2_info);

				mipi_csi2_pixelclk_enable(mipi_csi2_info);
			} else {
				vf.csi_prp_vf_mem.mipi_en = false;
				vf.csi_prp_vf_mem.mipi_vc = 0;
				vf.csi_prp_vf_mem.mipi_id = 0;
			}
		} else {
			vf.csi_prp_vf_mem.mipi_en = false;
			vf.csi_prp_vf_mem.mipi_vc = 0;
			vf.csi_prp_vf_mem.mipi_id = 0;
		}
	}
#endif

	err = ipu_init_channel(cam->ipu, CSI_PRP_VF_MEM, &vf);
	if (err != 0) {
		dev_err(cam->dev, "Error initializing CSI_PRP_VF_MEM channel\n");
		goto out_5;
	}

	if (fbi_ovl) {
		err = ipu_init_channel_buffer(cam->ipu, CSI_PRP_VF_MEM,
			IPU_GRAPH_IN_BUFFER,
			vf.csi_prp_vf_mem.in_g_pixel_fmt,
			cam->win.w.width,
			cam->win.w.height,
			cam->win.w.width,
			cam->vf_rotation,
			fbi_ovl->fix.smem_start,
			fbi_ovl->fix.smem_start,
			0, 0, 0);
	}

	if (err != 0) {
		dev_err(cam->dev, "Error initializing CSI_PRP_VF_MEM overlay buffer\n");
		goto out_4;
	}

	err = ipu_init_channel_buffer(cam->ipu, CSI_PRP_VF_MEM,
				      IPU_OUTPUT_BUFFER,
				      vf_out_format, cam->win.w.width,
				      cam->win.w.height,
				      cam->win.w.width,
				      cam->vf_rotation,
				      paddr[0],
				      paddr[1],
				      paddr[2], 0, 0);
	if (err != 0) {
		dev_err(cam->dev, "Error initializing CSI_PRP_VF_MEM video buffer\n");
		goto out_4;
	}

	ipu_clear_irq(cam->ipu, IPU_IRQ_PRP_VF_OUT_EOF);
	err = ipu_request_irq(cam->ipu, IPU_IRQ_PRP_VF_OUT_EOF,
			      prpvf_rot_eof_callback,
		      0, "Mxc Camera", cam);
	if (err < 0) {
		dev_err(cam->dev, "Error request irq:IPU_IRQ_PRP_VF_OUT_EOF\n");
		goto out_4;
	}

	ipu_enable_channel(cam->ipu, CSI_PRP_VF_MEM);

	ipu_select_buffer(cam->ipu, CSI_PRP_VF_MEM,
			  IPU_OUTPUT_BUFFER, 0);

	cam->overlay_active = true;
	return err;

out_4:
	ipu_uninit_channel(cam->ipu, CSI_PRP_VF_MEM);
out_5:
	return err;
}

/*!
 * prpvf_stop - stop the vf task
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 */
static int prpvf_stop(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0, i = 0;
	struct fb_info *fbi = NULL;
	struct fb_var_screeninfo fbvar;
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;
	int ipu_id;
	int csi_id;
#endif

	if (cam->overlay_active == false)
		return 0;

	for (i = 0; i < num_registered_fb; i++) {
		char *idstr = registered_fb[i]->fix.id;

		if (((strcmp(idstr, "DISP3 FG") == 0) && (cam->output < 3)) ||
		    ((strcmp(idstr, "DISP4 FG") == 0) && (cam->output >= 3))) {
			fbi = registered_fb[i];
			break;
		}
	}

	if (fbi == NULL) {
		dev_err(cam->dev, "DISP FG fb not found\n");
		return -EPERM;
	}

	ipu_disable_channel(cam->ipu, CSI_PRP_VF_MEM, true);

	ipu_uninit_channel(cam->ipu, CSI_PRP_VF_MEM);

	console_lock();
	fb_blank(fbi, FB_BLANK_POWERDOWN);
	console_unlock();

	/* Set the overlay frame buffer std to what it is used to be */
	fbvar = fbi->var;
	fbvar.accel_flags = FB_ACCEL_TRIPLE_FLAG;
	fbvar.nonstd = cam->fb_origin_std;
	fbvar.activate |= FB_ACTIVATE_FORCE;
	fb_set_var(fbi, &fbvar);

#ifdef CONFIG_MXC_MIPI_CSI2
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info) {
		if (mipi_csi2_get_status(mipi_csi2_info)) {
			ipu_id = mipi_csi2_get_bind_ipu(mipi_csi2_info);
			csi_id = mipi_csi2_get_bind_csi(mipi_csi2_info);

			if (cam->ipu == ipu_get_soc(ipu_id)
				&& cam->csi == csi_id)
				mipi_csi2_pixelclk_disable(mipi_csi2_info);
		}
	}
#endif

	cam->overlay_active = false;
	return err;
}

/*!
 * Enable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int prp_vf_enable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;

	return ipu_enable_csi(cam->ipu, cam->csi);
}

/*!
 * Disable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int prp_vf_disable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;

	/* free csi eof irq firstly.
	 * when disable csi, wait for idmac eof.
	 * it requests eof irq again
	 */
	ipu_free_irq(cam->ipu, IPU_IRQ_PRP_VF_OUT_EOF, cam);

	return ipu_disable_csi(cam->ipu, cam->csi);
}

/*!
 * function to select PRP-VF as the working path
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 * @return  status
 */
int prp_vf_flir_select(void *private)
{
	cam_data *cam;
	int err = 0;

	if (private) {
		cam = (cam_data *) private;
		cam->vf_start_sdc = prpvf_start;
		cam->vf_stop_sdc = prpvf_stop;
		cam->vf_enable_csi = prp_vf_enable_csi;
		cam->vf_disable_csi = prp_vf_disable_csi;
		cam->overlay_active = false;
	} else
		err = -EIO;

	return err;
}
EXPORT_SYMBOL(prp_vf_flir_select);

/*!
 * function to de-select PRP-VF as the working path
 *
 * @param private    cam_data * mxc v4l2 main structure
 *
 * @return  int
 */
int prp_vf_flir_deselect(void *private)
{
	cam_data *cam;

	if (private) {
		cam = (cam_data *) private;
		cam->vf_start_sdc = NULL;
		cam->vf_stop_sdc = NULL;
		cam->vf_enable_csi = NULL;
		cam->vf_disable_csi = NULL;
	}
	return 0;
}
EXPORT_SYMBOL(prp_vf_flir_deselect);

/*!
 * Init viewfinder task.
 *
 * @return  Error code indicating success or failure
 */
__init int prp_vf_flir_init(void)
{
	return 0;
}

/*!
 * Deinit viewfinder task.
 *
 * @return  Error code indicating success or failure
 */
void __exit prp_vf_flir_exit(void)
{
}

module_init(prp_vf_flir_init);
module_exit(prp_vf_flir_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IPU PRP VF SDC Driver");
MODULE_LICENSE("GPL");
