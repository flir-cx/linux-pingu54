/* Copyright 2020 NXP */
/*
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/ipu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "mxc_dispdrv.h"


#define BOE_VX039_MAX_BL_IF_BRIGHTNESS	255
#define BOE_VX039_MAX_BL_REG_LOW	0xFF
#define BOE_VX039_MAX_BL_REG_HIGH	0x01
#define BOE_VX039_MAX_BL_REG		0x01FF
#define BOE_VX039_REG_WRDISBV		0x51
#define BOE_VX039_SUPPLY_NAME		"power"


//#define PLANTUML_DEBUG
#ifdef PLANTUML_DEBUG
#define LOG_ENTER()  pr_err("plantuml:\"%ps\" -> \"%s\"\n", (void *)_RET_IP_, __func__)
#define LOG_EXIT()   pr_err("plantuml:\"%s\" --> \"%ps\"\n", __func__, (void *)_RET_IP_)
#define LOG_EXITR(x) pr_err("plantuml:\"%s\" --> \"%ps\" : %d\n", __func__, (void *)_RET_IP_, x)
#else
#define LOG_ENTER()
#define LOG_EXIT()
#define LOG_EXITR(x)
#endif

//#define DEBUG_BOE_REG_SETUP

struct boe_lcd_i2c_platform_data {
	u32 default_ifmt;
	u32 ipu_id;
	u32 disp_id;
	u8 i2c_bus;
	u8 i2c_addr;
	struct regulator *pwr_regulator;
	struct pinctrl *pinctrl;
};

struct boe_lcdif_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_lcdif;
	struct backlight_device *bl_dev;
};

#define DISPDRV_LCD_I2C	"lcd-i2c"

static struct fb_videomode lcdif_modedb[] = {
	{
		// 1024x768 @ 60 Hz , pixel clk @ 50,1MHz //
		.name = "BOE-XGA",
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.pixclock = 19921,
		.left_margin = 8,
		.right_margin = 4,
		.upper_margin = 35,
		.lower_margin = 3,
		.hsync_len = 4,
		.vsync_len = 6,
		.sync = 0, // We don't have drdy FB_SYNC_OE_LOW_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0,
	},
};

struct setup_entry_4b {
	// high part of reg addr
	u8 reg_h;
	u8 reg_l;
	u8 val_h;
	u8 val_l;
};

static struct setup_entry_4b vf_setup_4byte[] = {
	{0x26, 0x00, 0x00, 0x20}, // Not described!
	{0x53, 0x00, 0x00, 0x20}, // Not described!
	{0x51, 0x00, 0x00, 0xFF}, // WRDISBV DBV[7:0]
	{0x51, 0x01, 0x00, 0x03}, // WRDISBV DBV[9:8] 0x3FF Full brightness?
	{0x80, 0x00, 0x00, 0x00}, // RESCTRL1 D0 => 45 MHz
	{0x80, 0x01, 0x00, 0x00}, // RESCTRL1 NC[7:0] => x-axis resolution 0*4
	{0x80, 0x02, 0x00, 0xC0}, // RESCTRL1 NL[7:0] => y-axis resolution 192*4=768 (NL[8]=0 below)
	{0x80, 0x03, 0x00, 0x10}, // RESCTRL1 NC[8] = 1, NL[8] = 0, x-axis resolution 0x100*4=1024
	{0x80, 0x04, 0x00, 0x00}, // RESCTRL1 NC_DEC[7:0] => 0
	{0x80, 0x05, 0x00, 0x04}, // RESCTRL1 NC_DEC[10:8] => NC_DEC=0x0400 => 1024
	{0x81, 0x00, 0x00, 0x01}, // RESCTRL2 T1A[9:8] = 1
	{0x81, 0x01, 0x00, 0xD1}, // RESCTRL2 T1A[7:0] = 0xD1 => T1A=0x1D1 => 466 hsync dpclk
	{0x81, 0x02, 0x00, 0x00}, // RESCTRL2 VBPDA[9:8] = 0
	{0x81, 0x03, 0x00, 0x23}, // RESCTRL2 VBPDA[7:0] = 0x23 => 35
	{0x81, 0x04, 0x00, 0x00}, // RESCTRL2 VBFDA[9:8] = 0
	{0x81, 0x05, 0x00, 0x03}, // RESCTRL2 VBFDA[7:0] = 0x3 => 3
	{0x81, 0x06, 0x00, 0x01}, // RESCTRL2 PSELA[2:0] = 1, (0h=>1 VBP line, 7h=>10 VBP line) 1h?
	{0x82, 0x00, 0x00, 0x01}, // Not described!
	{0x82, 0x01, 0x00, 0xD1}, // Not described!
	{0x82, 0x02, 0x00, 0x00}, // Not described!
	{0x82, 0x03, 0x00, 0x23}, // Not described!
	{0x82, 0x04, 0x00, 0x00}, // Not described!
	{0x82, 0x05, 0x00, 0x03}, // Not described!
	{0x82, 0x06, 0x00, 0x03}, // Not described!
	{0x83, 0x00, 0x00, 0x80}, // Set RGB_DE_OPT=1 for RGB video mode 2, no data enable used
	{0x83, 0x01, 0x00, 0x0A}, // Set RGB_HBP, according to mail
	{0x35, 0x00, 0x00, 0x00}, // Not described!
	{0xFF, 0x00, 0x00, 0x5A}, // Not described!
	{0xFF, 0x01, 0x00, 0x81}, // Not described!
	{0xF9, 0x0D, 0x00, 0x40}, // Not described!
	{0xF9, 0x0E, 0x00, 0x47}, // Not described!
	{0xF9, 0x0F, 0x00, 0x4E}, // Not described!
	{0xF9, 0x10, 0x00, 0x55}, // Not described!
	{0xF9, 0x11, 0x00, 0x5C}, // Not described!
	{0xF9, 0x12, 0x00, 0x5E}, // Not described!
	{0xF9, 0x13, 0x00, 0x61}, // Not described!
	{0xF9, 0x14, 0x00, 0x64}, // Not described!
	{0xF9, 0x15, 0x00, 0x67}, // Not described!
	{0xF9, 0x16, 0x00, 0x6A}, // Not described!
	{0xF9, 0x17, 0x00, 0x6C}, // Not described!
	{0xF9, 0x18, 0x00, 0x6F}, // Not described!
	{0xF9, 0x19, 0x00, 0x72}, // Not described!
	{0xF9, 0x1A, 0x00, 0x75}, // Not described!
	{0xF9, 0x1B, 0x00, 0x78}, // Not described!
	{0xF9, 0x1C, 0x00, 0x7A}, // Not described!
	{0xF9, 0x1D, 0x00, 0x7D}, // Not described!
	{0xF9, 0x1E, 0x00, 0x80}, // Not described!
	{0xF9, 0x1F, 0x00, 0x83}, // Not described!
	{0xF9, 0x20, 0x00, 0x86}, // Not described!
	{0xF9, 0x21, 0x00, 0x88}, // Not described!
	{0xF9, 0x22, 0x00, 0x8B}, // Not described!
	{0xF9, 0x23, 0x00, 0x8E}, // Not described!
	{0xF9, 0x24, 0x00, 0x91}, // Not described!
	{0xF9, 0x25, 0x00, 0x94}, // Not described!
	{0xF9, 0x26, 0x00, 0x96}, // Not described!
	{0xF9, 0x27, 0x00, 0x99}, // Not described!
	{0xF9, 0x28, 0x00, 0x9C}, // Not described!
	{0xF9, 0x29, 0x00, 0x9F}, // Not described!
	{0xF9, 0x2A, 0x00, 0xA2}, // Not described!
	{0xF9, 0x2B, 0x00, 0xA4}, // Not described!
	{0xF9, 0x2C, 0x00, 0xA7}, // Not described!
	{0xF9, 0x2D, 0x00, 0xAA}, // Not described!
	{0xF9, 0x2E, 0x00, 0xAD}, // Not described!
	{0xF9, 0x2F, 0x00, 0xB0}, // Not described!
	{0xF4, 0x13, 0x00, 0x42}, // Not described!
	{0xF2, 0x07, 0x00, 0x11}  // Not described!
};
struct setup_entry_2b {
	// high part of reg addr
	u8 reg_h;
	u8 reg_l;
};
static struct setup_entry_2b vf_setup_2byte[] = {
	{0x11, 0x00}, // Sleep out
	{0x29, 0x00} // Display on
};

#ifdef DEBUG_BOE_VF_TESTMODE
// This code can be used to start the test mode of the display
static struct setup_entry_4b vf_setup_4byte_test[] = {
	{0xF0, 0x00, 0x00, 0xAA, 0x0000}, // Default
	{0xF0, 0x01, 0x00, 0x11, 0x0000}, // skip to bist page that below code will available
	{0xC4, 0x00, 0x00, 0xAA, 0x0000}, //BIST ON
	{0xC4, 0x01, 0x00, 0x55, 0x0000}, //BIST ON
	{0xC4, 0x02, 0x00, 0x01, 0x0000}, //BIST picture run
	{0xC4, 0x03, 0x00, 0x80, 0x0000}, //BIST picture run
	{0xC5, 0x00, 0x00, 0x09, 0x0000}, //Picture cycle time
	{0xC5, 0x01, 0x00, 0xFF, 0x0000}, // Picture choose
};
#endif

/* write a register to the display */
static int i2c_reg_write(struct device *dev, u8 *data_out, int len)
{
	int ret;
	struct boe_lcd_i2c_platform_data *plat_data = dev->platform_data;
	struct i2c_msg msg = {
		.addr	= plat_data->i2c_addr,
		.flags	= 0,
		.len	= len,
		.buf	= data_out,
	};
	struct i2c_adapter *i2c_adap = i2c_get_adapter(plat_data->i2c_bus);

	if (IS_ERR(i2c_adap)) {
		pr_err("%s: i2c_adap is null\n", __func__);
		return PTR_ERR(i2c_adap);
	}
	ret = i2c_transfer(i2c_adap, &msg, 1);
	// According to meeting with boe the controller needs 100us
	// to process the data
	udelay(100);

	if (len == 2)
		dev_dbg(dev, "Write register 0x%02x%02x! %d\n",
			data_out[0], data_out[1], ret);
	else if (len == 4)
		dev_dbg(dev, "Write register 0x%02x%02x%02x%02x! %d\n",
			data_out[0], data_out[1], data_out[2], data_out[3], ret);

	if (ret < 0)
		dev_err(dev, "Failed writing register 0x%02x%02x! %d\n",
			data_out[0], data_out[1], ret);

	// i2c_transfer returns messages sent, we want to return 0 on success
	if (ret == 1)
		ret = 0;
	i2c_put_adapter(i2c_adap);

	return ret;
}


/* read a register from the display */
static int i2c_reg_read(struct device *dev, u8 *data_out, u8 *data_in)
{
	int ret;
	struct boe_lcd_i2c_platform_data *plat_data = dev->platform_data;
	struct i2c_msg msgs[2] = {
		{
		.addr	= plat_data->i2c_addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data_out,
		},
		{
		.addr	= plat_data->i2c_addr,
		.flags	= I2C_M_RD,
		.len	= 2,
		.buf	= data_in,
		}
	};
	struct i2c_adapter *i2c_adap = i2c_get_adapter(plat_data->i2c_bus);

	if (IS_ERR(i2c_adap)) {
		pr_err("%s: i2c_adap is null\n", __func__);
		return PTR_ERR(i2c_adap);
	}

	ret = i2c_transfer(i2c_adap, msgs, 2);
	udelay(100);

	if (ret < 2)
		dev_err(dev, "Failed reading register 0x%02x%02x! %d\n",
			data_out[0], data_out[1], ret);
	else
		dev_info(dev, "Read back register 0x%02x%02x = 0x%02x%02x %d\n",
			data_out[0], data_out[1], data_in[0], data_in[1], ret);

	// i2c_transfer returns messages sent, we want to return 0 on success
	if (ret == 2)
		ret = 0;
	i2c_put_adapter(i2c_adap);

	return ret;
}

/**
 * @brief reads brightness register from display
 *
 * @param dev i2c device
 * @param val Where to put the value on success
 * @return int 0 0n success
 */
static int boe_disp_read_brightness(struct device *dev, u16 *val)
{
	int ret;
	u8 data_in[2];
	u8 data_out[2] = {
		BOE_VX039_REG_WRDISBV, 0x00
	};

	LOG_ENTER();
	ret = i2c_reg_read(dev, data_out, data_in);
	if (ret)
		return ret;
	*val = data_in[1];

	data_out[1] = 0x01;
	ret = i2c_reg_read(dev, data_out, data_in);

	*val |= data_in[1] << 8;

	dev_info(dev, "boe_read brightness:%d.\n", *val);
	LOG_EXITR(ret);
	return ret;
};


/**
 * @brief Writes a 9 bit value as brightness
 *
 * @param dev device
 * @param val 0x00 - 0x1FF
 * @return int zero on success
 */
static int boe_disp_write_brightness(struct device *dev, u16 val)
{
	u8 data_out[4] = {
		BOE_VX039_REG_WRDISBV,
		0x00,
		0x00,
		val & BOE_VX039_MAX_BL_REG_LOW
	};
	int ret;

	LOG_ENTER();
	// Write DBV[7:0]
	ret = i2c_reg_write(dev, data_out, 4);

	if (!ret) {
		// Write DBV[8:8]
		data_out[3] = (val >> 8) & BOE_VX039_MAX_BL_REG_HIGH;
		data_out[1] = 0x01;
	}
	ret = i2c_reg_write(dev, data_out, 4);
	LOG_EXITR(ret);
	return ret;
}

static int boe_disp_i2c_init(struct device *dev)
{
	int i;
	int ret = 0;
	u8 data_out[4];
#ifdef DEBUG_BOE_REG_SETUP
	u8 data_in[2];
#endif

	LOG_ENTER();

	for (i = 0; ret >= 0 && i < ARRAY_SIZE(vf_setup_4byte); ++i) {
		data_out[0] = vf_setup_4byte[i].reg_h;
		data_out[1] = vf_setup_4byte[i].reg_l;
		data_out[2] = vf_setup_4byte[i].val_h;
		data_out[3] = vf_setup_4byte[i].val_l;
		ret = i2c_reg_write(dev, data_out, 4);
#ifdef DEBUG_BOE_REG_SETUP
		i2c_reg_read(dev, data_out, data_in);
#endif
	}

	for (i = 0; ret >= 0 && i < ARRAY_SIZE(vf_setup_2byte); ++i) {
		data_out[0] = vf_setup_2byte[i].reg_h;
		data_out[1] = vf_setup_2byte[i].reg_l;
		ret = i2c_reg_write(dev, data_out, 2);
	}

	LOG_EXITR(ret);
	return ret;
}

static int boe_disp_bl_set_brightness(struct boe_lcdif_data *lcdif, int brightness)
{
	struct backlight_device *bl = lcdif->bl_dev;
	// The register seems to be 9 bits wide, hence 0x1FF as max
	u16 boe_brightness;
	int ret;

	LOG_ENTER();
	boe_brightness = ((brightness * BOE_VX039_MAX_BL_REG) / bl->props.max_brightness);
	dev_info(&bl->dev, "boe_brightness:%d.\n", boe_brightness);
	ret = boe_disp_write_brightness(&lcdif->pdev->dev, boe_brightness);

	LOG_EXITR(ret);
	return ret;
}

static int boe_disp_bl_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	struct boe_lcdif_data *lcdif = bl_get_data(bl);

	LOG_ENTER();
	if (bl->props.power != FB_BLANK_UNBLANK ||
		bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;
	boe_disp_bl_set_brightness(lcdif, brightness);

	dev_dbg(&bl->dev, "boe_disp backlight brightness:%d.\n", brightness);

	LOG_EXITR(0);
	return 0;
}


static int boe_disp_bl_get_brightness(struct backlight_device *bl)
{
	u16 val;
	int ret;
	struct boe_lcdif_data *lcdif = bl_get_data(bl);

	LOG_ENTER();
	ret = boe_disp_read_brightness(&lcdif->pdev->dev, &val);
	if (ret) {
		dev_err(&lcdif->pdev->dev,
			"Failed to read boe brightness: %d\n", ret);
		return ret;
	}
	ret = (val * bl->props.max_brightness) / BOE_VX039_MAX_BL_REG;
	LOG_EXITR(ret);
	return ret;
}

static int boe_disp_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	LOG_ENTER();
	LOG_EXIT();
	return 0;
}
static const struct backlight_ops boe_disp_lcd_bl_ops = {
	.update_status = boe_disp_bl_update_status,
	.get_brightness = boe_disp_bl_get_brightness,
	.check_fb = boe_disp_bl_check_fb,
};

static int boe_disp_init_backlight(struct boe_lcdif_data *lcdif)
{
	struct backlight_properties props;
	struct backlight_device	*bl;
	struct device *dev = &lcdif->pdev->dev;

	LOG_ENTER();
	if (lcdif->bl_dev) {
		pr_debug("boe_disp backlight already init!\n");
		return 0;
	}
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = BOE_VX039_MAX_BL_IF_BRIGHTNESS;
	props.type = BACKLIGHT_RAW;
	bl = devm_backlight_device_register(dev, "mxcfb_boe", dev, lcdif,
					    &boe_disp_lcd_bl_ops, &props);
	if (IS_ERR(bl)) {
		pr_err("error %ld on backlight register\n", PTR_ERR(bl));
		return PTR_ERR(bl);
	}

	lcdif->bl_dev = bl;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.brightness = BOE_VX039_MAX_BL_IF_BRIGHTNESS;

	boe_disp_bl_update_status(bl);
	LOG_EXIT();

	return 0;
}

// This would be called from mxcfb_probe -> mxcfb_dispdrv_init ->
// mxc_dispdrv_gethandle. This is run after boe_lcdif_init and boe_lcdif_probe
static int boe_dispdrv_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret, i;
	struct boe_lcdif_data *lcdif = mxc_dispdrv_getdata(disp);
	struct device *dev = &lcdif->pdev->dev;
	struct boe_lcd_i2c_platform_data *plat_data = dev->platform_data;
	struct fb_videomode *modedb = lcdif_modedb;

	int modedb_sz = ARRAY_SIZE(lcdif_modedb);

	LOG_ENTER();

	/* use platform defined ipu/di */
	ret = ipu_di_to_crtc(dev, plat_data->ipu_id,
			     plat_data->disp_id, &setting->crtc);
	if (ret < 0)
		return ret;

	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				modedb, modedb_sz, NULL, setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);
		setting->if_fmt = plat_data->default_ifmt;
	}

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < modedb_sz; i++) {
		struct fb_videomode m;

		fb_var_to_videomode(&m, &setting->fbi->var);
		if (fb_mode_is_equal(&m, &modedb[i])) {
			fb_add_videomode(&modedb[i],
					&setting->fbi->modelist);
			break;
		}
	}

	ret = boe_disp_i2c_init(dev);

	if (ret) {
		dev_err(dev, "Failed initalizing lcd %d\n", ret);
		return ret;
	}

	ret = boe_disp_init_backlight(lcdif);

	LOG_EXITR(ret);
	return ret;
}

// The ones below are defined just to get a feel for how they are used.
static void boe_dispdrv_deinit(struct mxc_dispdrv_handle *disp)
{
	struct boe_lcdif_data *lcdif = mxc_dispdrv_getdata(disp);
	LOG_ENTER();
	devm_backlight_device_unregister(&lcdif->pdev->dev, lcdif->bl_dev);
	LOG_EXIT();
}

static int boe_dispdrv_enable(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	LOG_ENTER();
	LOG_EXIT();
	return 0;
}

static void boe_dispdrv_disable(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	LOG_ENTER();
	LOG_EXIT();
}

static int boe_dispdrv_setup(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	LOG_ENTER();
	LOG_EXIT();

	return 0;
}

static int boe_dispdrv_swap_panel(struct mxc_dispdrv_handle *disp, struct fb_info *fbi, int active)
{
	LOG_ENTER();
	LOG_EXIT();

	return 0;
}

static int boe_dispdrv_get_active_panel(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	LOG_ENTER();
	LOG_EXIT();
	return 0;
}
static struct mxc_dispdrv_driver boe_disp_drv = {
	.name	= DISPDRV_LCD_I2C,
	.init	= boe_dispdrv_init,
	.deinit	= boe_dispdrv_deinit,
	.enable	= boe_dispdrv_enable,

	/* display driver disable function, called at early part of fb_blank */
	.disable = boe_dispdrv_disable,

	/* display driver setup function, called at early part of fb_set_par */
	.setup = boe_dispdrv_setup,

	/* display driver swap panel function */
	.swap_panel = boe_dispdrv_swap_panel,

	/* display driver get active panel function */
	.get_active_panel = boe_dispdrv_get_active_panel
};


/**
 * @brief Reads 32 bit value from device_node, logs failure as error
 *
 * @param np device node
 * @param propname property name
 * @param out_value value read
 * @return int 0 on success, -ative otherwise
 */
static int of_property_read_u32_with_log(const struct device_node *np,
					 const char *propname,
					 u32 *out_value,
					 struct device *dev)
{
	int err;

	err = of_property_read_u32(np, propname, out_value);
	if (err) {
		dev_err(dev, "get of property %s failed\n", propname);
		return err;
	}
	dev_dbg(dev, "get of property %s=0x%08x\n", propname, *out_value);
	return err;
}

/**
 * @brief Reads 8 bit value from device_node, logs failure as error
 *
 * @param np device node
 * @param propname property name
 * @param out_value value read
 * @return int 0 on success, -ative otherwise
 */
static int of_property_read_u8_with_log(const struct device_node *np,
					const char *propname,
					u8 *out_value,
					struct device *dev)
{
	int err;
	u32 tmp_val;
	// of_property_read_u8 is broken, it always returns 0!
	err = of_property_read_u32(np, propname, &tmp_val);
	if (err) {
		dev_err(dev, "get of property %s failed\n", propname);
		return err;
	}
	*out_value = tmp_val & 0xFF;
	dev_dbg(dev, "get of property %s=0x%02x\n", propname, tmp_val);
	return err;
}

static int lcd_get_of_properties(struct platform_device *pdev,
				struct boe_lcd_i2c_platform_data *plat_data)
{
	struct device_node *np = pdev->dev.of_node;
	int err;
	const char *default_ifmt;

	err = of_property_read_u32_with_log(np, "ipu_id", &plat_data->ipu_id, &pdev->dev);
	if (err)
		return err;
	err = of_property_read_u32_with_log(np, "disp_id", &plat_data->disp_id, &pdev->dev);
	if (err)
		return err;
	err = of_property_read_u8_with_log(np, "i2c_bus", &plat_data->i2c_bus, &pdev->dev);
	if (err)
		return err;
	err = of_property_read_u8_with_log(np, "i2c_addr", &plat_data->i2c_addr, &pdev->dev);
	if (err)
		return err;
	dev_dbg(&pdev->dev, "ipu_id=0x%x, disp_id=0x%x, bus=0x%x, addr=0x%x",
		plat_data->ipu_id,
		plat_data->disp_id,
		plat_data->i2c_bus,
		plat_data->i2c_addr);
	err = of_property_read_string(np, "default_ifmt", &default_ifmt);
	if (err) {
		dev_dbg(&pdev->dev, "get of property default_ifmt fail\n");
		return err;
	}
	dev_dbg(&pdev->dev, "default_ifmt=%s", default_ifmt);
	if (!strncmp(default_ifmt, "RGB24", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_RGB24;
	else if (!strncmp(default_ifmt, "BGR24", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_BGR24;
	else if (!strncmp(default_ifmt, "GBR24", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_GBR24;
	else if (!strncmp(default_ifmt, "RGB565", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_RGB565;
	else if (!strncmp(default_ifmt, "RGB666", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_RGB666;
	else if (!strncmp(default_ifmt, "YUV444", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_YUV444;
	else if (!strncmp(default_ifmt, "LVDS666", 7))
		plat_data->default_ifmt = IPU_PIX_FMT_LVDS666;
	else if (!strncmp(default_ifmt, "YUYV16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_YUYV;
	else if (!strncmp(default_ifmt, "UYVY16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_UYVY;
	else if (!strncmp(default_ifmt, "YVYU16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_YVYU;
	else if (!strncmp(default_ifmt, "VYUY16", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_VYUY;
	else {
		dev_err(&pdev->dev, "err default_ifmt!\n");
		return -ENOENT;
	}

	return err;
}

static int boe_lcdif_probe(struct platform_device *pdev)
{
	int ret;
	struct boe_lcdif_data *lcdif;
	struct boe_lcd_i2c_platform_data *plat_data;
	struct regulator *pwr_reg;

	LOG_ENTER();

	// The regulator is not up so this probe will be defered
	// the first few times, this is why this is done before
	// any allocations. Since it is on from u-boot there is
	// no settling time after enable.
	pwr_reg = devm_regulator_get_optional(&pdev->dev, BOE_VX039_SUPPLY_NAME);
	if (PTR_ERR(pwr_reg) != -ENODEV) {
		if (IS_ERR(pwr_reg))
			return PTR_ERR(pwr_reg);
		ret = regulator_enable(pwr_reg);
	}

	plat_data = devm_kzalloc(&pdev->dev,
				sizeof(struct boe_lcd_i2c_platform_data),
				GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;
	pdev->dev.platform_data = plat_data;
	plat_data->pwr_regulator = pwr_reg;

	ret = lcd_get_of_properties(pdev, plat_data);
	if (ret < 0) {
		if (ret == -EPROBE_DEFER)
			dev_err(&pdev->dev, "probe DEFERED\n");
		else
			dev_err(&pdev->dev, "get lcd of property fail\n");
		return ret;
	}

	plat_data->pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(plat_data->pinctrl)) {
		dev_err(&pdev->dev, "can't get/select default pinctrl\n");
		return PTR_ERR(plat_data->pinctrl);
	}

	lcdif = devm_kzalloc(&pdev->dev, sizeof(struct boe_lcdif_data),
				GFP_KERNEL);
	if (!lcdif)
		return -ENOMEM;

	lcdif->pdev = pdev;
	lcdif->disp_lcdif = mxc_dispdrv_register(&boe_disp_drv);
	mxc_dispdrv_setdata(lcdif->disp_lcdif, lcdif);

	dev_set_drvdata(&pdev->dev, lcdif);

	LOG_EXITR(ret);
	return ret;
}

static int boe_lcdif_remove(struct platform_device *pdev)
{
	struct boe_lcdif_data *lcdif = dev_get_drvdata(&pdev->dev);

	LOG_ENTER();

	mxc_dispdrv_puthandle(lcdif->disp_lcdif);
	mxc_dispdrv_unregister(lcdif->disp_lcdif);

	LOG_EXIT();
	return 0;
}

/**
 * @brief Enable the display interface bus
 *
 * @param pdev
 * @return 0 on success -errno on fail
 */
static int enable_di_bus(struct platform_device *pdev)
{
	int ret = 0;
	struct boe_lcd_i2c_platform_data *plat_data = pdev->dev.platform_data;
	struct pinctrl_state *pin_state;

	pin_state = pinctrl_lookup_state(plat_data->pinctrl, "default");
	ret = pinctrl_select_state(plat_data->pinctrl, pin_state);
	if (ret)
		dev_err(&pdev->dev, "Failed to enable di bus\n");
	else
		dev_info(&pdev->dev, "Enabled di bus\n");

	return ret;
}
/**
 * @brief Disable the display interface bus
 *
 * @param pdev
 * @return 0 on success -errno on fail
 */
static int disable_di_bus(struct platform_device *pdev)
{
	int ret = 0;
	struct boe_lcd_i2c_platform_data *plat_data = pdev->dev.platform_data;
	struct pinctrl_state *pin_state;

	pin_state = pinctrl_lookup_state(plat_data->pinctrl, "disabled");
	ret = pinctrl_select_state(plat_data->pinctrl, pin_state);
	if (ret)
		dev_err(&pdev->dev, "Failed to disable di bus\n");
	else
		dev_info(&pdev->dev, "Disabled di bus\n");

	return ret;
}

/**
 * @brief Disable power supply regulator, if it exists
 *
 * @param pdev
 * @return 0 on success or if no regulator exists, else errno
 */
static int boe_lcd_disp_pwr_disable(struct platform_device *pdev)
{
	int ret = 0;
	struct boe_lcd_i2c_platform_data *plat_data = pdev->dev.platform_data;

	LOG_ENTER();

	if (!IS_ERR(plat_data->pwr_regulator)) {
		ret = regulator_disable(plat_data->pwr_regulator);
		if (ret)
			dev_err(&pdev->dev, "failed to disable %s regulator\n",
				BOE_VX039_SUPPLY_NAME);
		else
			dev_info(&pdev->dev, "%s regulator disabled\n",
				 BOE_VX039_SUPPLY_NAME);
	} else {
		dev_info(&pdev->dev, "%s regulator not found, disable passed\n",
			 BOE_VX039_SUPPLY_NAME);
	}

	LOG_EXITR(ret);
	return ret;
}
/**
 * @brief Enable power supply regulator, if it exists
 *
 * @param pdev
 * @return 0 on success or if no regulator exists, else errno
 */
static int boe_lcd_disp_pwr_enable(struct platform_device *pdev)
{
	int ret = 0;
	struct boe_lcd_i2c_platform_data *plat_data = pdev->dev.platform_data;

	LOG_ENTER();

	if (!IS_ERR(plat_data->pwr_regulator)) {
		ret = regulator_enable(plat_data->pwr_regulator);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable %s regulator\n",
				BOE_VX039_SUPPLY_NAME);
			return ret;
		}
		dev_info(&pdev->dev, "%s regulator enabled\n",
				BOE_VX039_SUPPLY_NAME);

		// From VX039X0M-NH0_Ver1.0...pdf, 10.1 Power on sequence, T1+T2
		msleep(21);
	} else {
		dev_info(&pdev->dev, "%s regulator not found, enable passed\n",
			 BOE_VX039_SUPPLY_NAME);
	}

	LOG_EXITR(ret);
	return ret;
}

/**
 * @brief Entry point for suspend cb
 *
 * @param pdev
 * @param state
 * @return int
 */
int boe_lcdif_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;

	LOG_ENTER();
	ret = boe_lcd_disp_pwr_disable(pdev);
	if (ret)
		dev_err(&pdev->dev, "failed to disable display power\n");

	ret = disable_di_bus(pdev);

	LOG_EXITR(ret);
	return ret;
}

/**
 * @brief Entry point for resume cb
 *
 * @param pdev
 * @return int
 */
int boe_lcdif_resume(struct platform_device *pdev)
{
	int ret;
	struct boe_lcdif_data *lcdif = dev_get_drvdata(&pdev->dev);

	LOG_ENTER();
	ret = boe_lcd_disp_pwr_enable(pdev);
	if (ret)
		return ret;

	ret = enable_di_bus(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to reenable di bus %d, bailing out.\n", ret);
		return ret;
	}

	ret = boe_disp_i2c_init(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize lcd %d\n", ret);
		return ret;
	}

	ret = boe_disp_bl_update_status(lcdif->bl_dev);

	LOG_EXITR(ret);
	return ret;

}
static const struct of_device_id boe_lcd_dt_ids[] = {
	{ .compatible = "boe,vx039x0m-nh0"},
	{ /* sentinel */ }
};
static struct platform_driver mxc_lcdif_driver = {
	.driver = {
		.name = "mxcfb_boe",
		.of_match_table	= boe_lcd_dt_ids,
	},
	.probe = boe_lcdif_probe,
	.remove = boe_lcdif_remove,
	.suspend = boe_lcdif_suspend,
	.resume	= boe_lcdif_resume,
};

/**
 * @brief Entry point when loaded as a module
 *
 * @return int
 */
static int __init boe_lcdif_init(void)
{
	LOG_ENTER();
	LOG_EXIT();
	// This will trigger boe_dispdrv_init from the nxp framework
	return platform_driver_register(&mxc_lcdif_driver);
}

/**
 * @brief exit point as module
 *
 */
static void __exit boe_lcdif_exit(void)
{
	LOG_ENTER();
	LOG_EXIT();
	platform_driver_unregister(&mxc_lcdif_driver);
}

module_init(boe_lcdif_init);
module_exit(boe_lcdif_exit);

MODULE_AUTHOR("Jonas Rydow");
MODULE_DESCRIPTION("i.MX ipuv3 LCD extern port driver with i2c setup");
MODULE_LICENSE("GPL");
