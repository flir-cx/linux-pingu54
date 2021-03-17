/*
 * Copyright (C) 2016 FLIR Systems.
 *
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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>
#include <linux/i2c.h>
#include "mipi_dsi.h"
#include <linux/workqueue.h>

#define KCDA914BL_MAX_BRIGHT					255
#define KCDA914BL_DEF_BRIGHT					255

#define KCDA914_MAX_DPHY_CLK					(300)
#define KCDA914_ONE_DATA_LANE					(0x1)
#define KCDA914_TWO_DATA_LANE					(0x2)

#define KCDA914_I2C_ADDRESS						(0x68>>1)
#define KCDA914_I2C_BUS							(2)

#define KCDA914_REG_POWER_DOWN_CONTROL			(0x0F)
#define KCDA914_REG_PWM1						(0x76)
#define KCDA914_REG_PWM2						(0x77)
static void brightness_work_handler(struct work_struct *w);
static struct workqueue_struct *wq = 0;
struct mipi_dsi_info *dsi;
static DECLARE_DELAYED_WORK(brightness_work, brightness_work_handler);


struct reg_value {
    u8 reg;
    u8 val;
};

static struct reg_value vf_setup[] =
{
{0x00, 0x1},
{0x01, 0x4},
{0x02, 0x2D},
{0x03, 0xA0},
{0x04, 0x0},
{0x05, 0xE0},
{0x06, 0x1},
{0x07, 0x80},
{0x08, 0x2},
{0x09, 0x80},
{0x0A, 0x80},
{0x0B, 0x0},
{0x0C, 0x0},
{0x0D, 0x88},
{0x0E, 0x21},
{0x0F, 0x0},
{0x10, 0x0},
{0x11, 0x0},
{0x12, 0x0},
{0x13, 0x0},
{0x14, 0x0},
{0x15, 0x0},
{0x16, 0x0},
{0x17, 0x0},
{0x18, 0x0},
{0x19, 0x0},
{0x1A, 0x0},
{0x1B, 0x0},
{0x1C, 0x0},
{0x1D, 0x0},
{0x1E, 0x0},
{0x1F, 0x0},
{0x20, 0x0},
{0x21, 0x0},
{0x22, 0x0},
{0x23, 0x0},
{0x24, 0x0},
{0x25, 0x0},
{0x26, 0x0},
{0x27, 0x0},
{0x28, 0x0},
{0x29, 0x10},
{0x2A, 0x10},
{0x2B, 0x4},
{0x2C, 0x4},
{0x2D, 0xE0},
{0x2E, 0x10},
{0x2F, 0x0},
{0x30, 0x0},
{0x31, 0x0},
{0x32, 0x40},
{0x33, 0x0},
{0x34, 0x80},
{0x35, 0x0},
{0x36, 0xAA},
{0x37, 0x0},
{0x38, 0xD1},
{0x39, 0x0},
{0x3A, 0xED},
{0x3B, 0x0},
{0x3C, 0x0D},
{0x3D, 0x1},
{0x3E, 0x1D},
{0x3F, 0x1},
{0x40, 0x2B},
{0x41, 0x1},
{0x42, 0x3B},
{0x43, 0x1},
{0x44, 0x47},
{0x45, 0x1},
{0x46, 0x57},
{0x47, 0x1},
{0x48, 0x63},
{0x49, 0x1},
{0x4A, 0x71},
{0x4B, 0x1},
{0x4C, 0x81},
{0x4D, 0x1},
{0x4E, 0x95},
{0x4F, 0x1},
{0x50, 0xAD},
{0x51, 0x1},
{0x52, 0xFF},
{0x53, 0x1},
{0x54, 0xBF},
{0x55, 0x1},
{0x56, 0x7E},
{0x57, 0x1},
{0x58, 0x55},
{0x59, 0x1},
{0x5A, 0x2E},
{0x5B, 0x1},
{0x5C, 0x12},
{0x5D, 0x1},
{0x5E, 0xF2},
{0x5F, 0x0},
{0x60, 0xE2},
{0x61, 0x0},
{0x62, 0xD4},
{0x63, 0x0},
{0x64, 0xC4},
{0x65, 0x0},
{0x66, 0xB8},
{0x67, 0x0},
{0x68, 0xA7},
{0x69, 0x0},
{0x6A, 0x9B},
{0x6B, 0x0},
{0x6C, 0x8D},
{0x6D, 0x0},
{0x6E, 0x7D},
{0x6F, 0x0},
{0x70, 0x69},
{0x71, 0x0},
{0x72, 0x51},
{0x73, 0x0},
{0x74, 0x0},
{0x75, 0x0},
{0x76, 0x0},
{0x77, 0xC0}, //disable backlight when powering up viewfinder
{0x78, 0x0},
{0x79, 0x0},
{0x7A, 0x0},
{0x7B, 0x0},
{0x7C, 0x39},
{0x7D, 0x1},
{0x7E, 0x0},
{0x7F, 0x0},
{0x80, 0x10},
{0x81, 0x0},
{0x82, 0x3C},
{0x83, 0x0},
{0x84, 0x5A},
{0x85, 0x0},
{0x86, 0x40},
{0x87, 0x0},
{0x88, 0x20},
{0x89, 0x3},
{0x8A, 0x58},
{0x8B, 0x0},
{0x8C, 0x0},
{0x8D, 0x0},
{0x8E, 0x0},
{0x8F, 0x0},
{0x90, 0x0},
{0x91, 0x0},
{0x92, 0x11},
{0x93, 0x0},
{0x94, 0x0},
{0x95, 0x0},
{0x96, 0xBF},
{0x97, 0x1},
{0x98, 0x40},
{0x99, 0x0},
{0x9A, 0x0},
{0x9B, 0x0},
{0x9C, 0x0},
{0x9D, 0x0},
{0x9E, 0x0},
{0x9F, 0x0},
{0xA0, 0x1}, //clk_sel
{0xA1, 0x0},
{0xA2, 0x7},
{0xA3, 0x7},
{0xA4, 0x7},
{0xA5, 0x0},
{0xA6, 0x0},
{0xA7, 0x0},
{0xA8, 0x1}, //num lanes
{0xA9, 0x50},// color_coding
{0xAA, 0x80},
{0xAB, 0x2},
{0xAC, 0x20},
{0xAD, 0x0},
{0xAE, 0x20},
{0xAF, 0x0},
{0xB0, 0x3},
{0xB1, 0x0},
{0xB2, 0x0},
{0xB3, 0x0},
{0xB4, 0x0},
{0xB5, 0x0},
{0xB6, 0x0},
{0xB7, 0x0},
{0xB8, 0x0},
{0xB9, 0x0},
{0xBA, 0x0},
{0xBB, 0x0},
{0xBC, 0x0},
{0xBD, 0x0},
{0xBE, 0x0},
{0xBF, 0x0},
{0xC0, 0x0},
{0xC1, 0x0},
{0xC2, 0x0},
{0xC3, 0x0},
{0xC4, 0x0},
{0xC5, 0x0},
{0xC6, 0x0},
{0xC7, 0x0},
{0xC8, 0x0},
{0xC9, 0x0},
{0xCA, 0x0},
{0xCB, 0x0},
{0xCC, 0x0},
{0xCD, 0x0},
{0xCE, 0x0},
{0xCF, 0x0},
{0xD0, 0x0},
{0xD1, 0x0},
{0xD2, 0x0},
{0xD3, 0x0},
{0xD4, 0x0},
{0xD5, 0x0},
{0xD6, 0x0},
{0xD7, 0x0},
{0xD8, 0x0},
{0xD9, 0x0},
{0xDA, 0x0},
{0xDB, 0x0},
{0xDC, 0x0},
{0xDD, 0x0},
{0xDE, 0x0},
{0xDF, 0x0},
{0xE0, 0x0},
{0xE1, 0x0},
{0xE2, 0x0},
{0xE3, 0x0},
{0xE4, 0x0},
{0xE5, 0x0},
{0xE6, 0x0},
{0xE7, 0x0},
{0xE8, 0x0},
{0xE9, 0x0},
{0xEA, 0x0},
{0xEB, 0x0},
{0xEC, 0x0},
{0xED, 0x0},
{0xEE, 0x0},
{0xEF, 0x0},
{0xF0, 0x80},
{0xF1, 0x0},
{0xF2, 0x0},
{0xF3, 0x0},
{0xF4, 0x0},
{0xF5, 0x0},
{0xF6, 0x0},
{0xF7, 0x0},
{0xF8, 0x0},
{0xF9, 0x0},
{0xFA, 0x0},
{0xFB, 0x0},
{0xFC, 0x0},
{0xFD, 0x0},
{0xFE, 0x0},
{0xFF, 0x0}
};

static int kcda914bl_brightness;
static int mipid_init_backlight(struct mipi_dsi_info *mipi_dsi);
static void mipid_bl_set_brightness(struct mipi_dsi_info *mipi_dsi, int brightness);

static void brightness_work_handler(struct work_struct *w)
{
	mipid_bl_set_brightness(dsi, kcda914bl_brightness);
}


static struct fb_videomode kopin_lcd_modedb[] = {
	{
	 "KOPIN-VGA", 60, 640, 480, 37000,
	 100, 100,
	 31, 10,
	 96,4,
	 FB_SYNC_OE_LOW_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = KCDA914_TWO_DATA_LANE,
	.max_phy_clk    = KCDA914_MAX_DPHY_CLK,
	.dpi_fmt		= MIPI_RGB888,
};
void mipid_kcda914_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &kopin_lcd_modedb[0];
	*size = ARRAY_SIZE(kopin_lcd_modedb);
	*data = &lcd_config;
}


/* write a register to the display */
static int kcda914_i2c_reg_write(struct mipi_dsi_info *mipi_dsi,u8 reg, u8 val)
{
	int ret;
	unsigned char data[2] = { reg, val };
	struct i2c_adapter 	*hI2C = i2c_get_adapter(KCDA914_I2C_BUS);
	struct i2c_msg msg = {
		.addr	= KCDA914_I2C_ADDRESS,
		.flags	= 0,
		.len	= 2,
		.buf	= data,
	};

	ret = i2c_transfer(hI2C, &msg, 1);
	udelay(100);

	if (ret < 0) {
		dev_err(&mipi_dsi->pdev->dev, "Failed writing register 0x%02x! %d \n", reg,ret);
	}
	i2c_put_adapter(hI2C);
	return ret;
}



int mipid_kcda914_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	int err;
	int i;

	if(mipi_dsi->vf_rst_gpio)
		gpio_set_value_cansleep(mipi_dsi->vf_rst_gpio, 0);

	if(mipi_dsi->lcd_mipi_sel_gpio)
		gpio_set_value_cansleep(mipi_dsi->lcd_mipi_sel_gpio, 0);

	if(mipi_dsi->vf_1v8_en_gpio)
		gpio_set_value_cansleep(mipi_dsi->vf_1v8_en_gpio, 1);
	udelay(700);

	if(mipi_dsi->vf_4v5_en_gpio)
		gpio_set_value_cansleep(mipi_dsi->vf_4v5_en_gpio, 1);
	udelay(1000);

	if(mipi_dsi->vf_rst_gpio)
		gpio_set_value_cansleep(mipi_dsi->vf_rst_gpio, 1);
	udelay(100);

	for(i=0; i<ARRAY_SIZE(vf_setup);i++)
	{
		kcda914_i2c_reg_write(mipi_dsi,vf_setup[i].reg,vf_setup[i].val);
	}

	err = mipid_init_backlight(mipi_dsi);
	return err;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	struct mipi_dsi_info *mipi_dsi = bl_get_data(bl);

	if (bl->props.power != FB_BLANK_UNBLANK ||
		bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;
	mipid_bl_set_brightness(mipi_dsi, brightness);

	dev_dbg(&bl->dev, "mipid backlight brightness:%d.\n", brightness);
	return 0;
}


static void mipid_bl_set_brightness(struct mipi_dsi_info *mipi_dsi, int brightness)
{
	int temp;
	temp = brightness << 4 ;
	kcda914_i2c_reg_write(mipi_dsi, KCDA914_REG_PWM1 ,temp & 0xff);
	kcda914_i2c_reg_write(mipi_dsi, KCDA914_REG_PWM2 ,0xc0 | ((temp >> 8) & 0xf)) ;
	kcda914bl_brightness = brightness & KCDA914BL_MAX_BRIGHT;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return kcda914bl_brightness;
}

static int mipi_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	return 0;
}

static const struct backlight_ops mipid_lcd_bl_ops = {
	.update_status = mipid_bl_update_status,
	.get_brightness = mipid_bl_get_brightness,
	.check_fb = mipi_bl_check_fb,
};

static int mipid_init_backlight(struct mipi_dsi_info *mipi_dsi)
{
	struct backlight_properties props;
	struct backlight_device	*bl;

	if (mipi_dsi->bl) {
		pr_debug("mipid backlight already init!\n");
		return 0;
	}
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = KCDA914BL_MAX_BRIGHT;
	props.type = BACKLIGHT_RAW;
	bl = backlight_device_register("backlight_vf", &mipi_dsi->pdev->dev,
		mipi_dsi, &mipid_lcd_bl_ops, &props);
	if (IS_ERR(bl)) {
		pr_err("error %ld on backlight register\n", PTR_ERR(bl));
		return PTR_ERR(bl);
	}
	mipi_dsi->bl = bl;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.brightness = KCDA914BL_DEF_BRIGHT;

	mipid_bl_update_status(bl);
	return 0;
}

int mipid_kcda914_lcd_power_set(struct mipi_dsi_info *mipi_dsi, int state)
{

	if(state) {
		mipid_kcda914_lcd_setup(mipi_dsi);
		dsi=mipi_dsi;
		if (!wq)
			wq = create_freezable_workqueue("brightness_kcda914");
		queue_delayed_work(wq, &brightness_work, msecs_to_jiffies(250));

		if(mipi_dsi->lcd_mipi_sel_gpio)
			gpio_set_value_cansleep(mipi_dsi->lcd_mipi_sel_gpio, 0);

	} else {
		if(mipi_dsi->vf_rst_gpio)
			gpio_set_value_cansleep(mipi_dsi->vf_rst_gpio, 0);

		if(mipi_dsi->vf_4v5_en_gpio)
			gpio_set_value_cansleep(mipi_dsi->vf_4v5_en_gpio, 0);

		if(mipi_dsi->vf_1v8_en_gpio)
			gpio_set_value_cansleep(mipi_dsi->vf_1v8_en_gpio, 0);
	}
	return 0;
}

int mipid_kcda914_lcd_power_get(struct mipi_dsi_info *mipi_dsi)
{
	struct device *dev = &mipi_dsi->pdev->dev;
	int power = 0;
	int vf_rst_n, vf_4v5_en, vf_1v8_en;
	if (mipi_dsi->vf_rst_gpio)
		vf_rst_n = gpio_get_value_cansleep(mipi_dsi->vf_rst_gpio);

	if (mipi_dsi->vf_4v5_en_gpio)
		vf_4v5_en = gpio_get_value_cansleep(mipi_dsi->vf_4v5_en_gpio);

	if (mipi_dsi->vf_1v8_en_gpio)
		vf_1v8_en = gpio_get_value_cansleep(mipi_dsi->vf_1v8_en_gpio);
	
	if (vf_rst_n < 0 ||
	    vf_4v5_en < 0 ||
	    vf_1v8_en < 0) {
		dev_err(dev, "Failed to get gpio in %s\n", __func__);
		power = -EIO;
	}

	if (vf_rst_n == 1 &&
	    vf_4v5_en == 1 &&
	    vf_1v8_en == 1)
		power = 1;

	return power;
}
