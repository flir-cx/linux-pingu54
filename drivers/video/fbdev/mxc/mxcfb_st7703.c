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
#include <video/mipi_display.h>
#include "mipi_dsi.h"

#define ST7703_MAX_DPHY_CLK					(360)
#define ST7703_ONE_DATA_LANE					(0x1)
#define ST7703_TWO_DATA_LANE					(0x2)
//#define ST7703_CMD_SWRESET					(0x1)
//#define ST7703_CMD_SLPOUT						(0x11)
//#define ST7703_CMD_DISPON						(0x29)
//#define ST7703_REG_MADCTL						(0x36)
#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x04)

#define ST7703_CMD_SWRESET				(0x01)
#define ST7703_CMD_RDDIDIF				(0x04)
#define ST7703_CMD_RDDPM				(0x0A)
#define ST7703_CMD_RDDMADCTL			(0x0B)
#define ST7703_CMD_RDDCOLMOD			(0x0C)
#define ST7703_CMD_RDDIM				(0x0D)
#define ST7703_CMD_RDDSM				(0x0E)
#define ST7703_CMD_RDDSDR				(0x0F)
#define ST7703_CMD_SLPIN				(0x10)
#define ST7703_CMD_SLPOUT				(0x11)
#define ST7703_CMD_NORON				(0x13)
#define ST7703_CMD_INVOFF				(0x20)
#define ST7703_CMD_INVON				(0x21)
#define ST7703_CMD_ALLPOFF				(0x22)
#define ST7703_CMD_ALLPON				(0x23)
#define ST7703_CMD_DISPOFF				(0x28)
#define ST7703_CMD_DISPON				(0x29)
#define ST7703_CMD_MADCTL				(0x36)
#define ST7703_CMD_IDMOFF				(0x38)
#define ST7703_CMD_IDMON				(0x39)
#define ST7703_CMD_WRDISBV				(0x51)
#define ST7703_CMD_RDRDISBV				(0x52)
#define ST7703_CMD_WRCTRLD				(0x53)
#define ST7703_CMD_RDCTRLD				(0x54)
#define ST7703_CMD_WRCABC				(0x55)
#define ST7703_CMD_RDCABC				(0x56)
#define ST7703_CMD_RDABCSDR				(0x68)
#define ST7703_CMD_RDBWLB				(0x70)
#define ST7703_CMD_RDBkx				(0x71)
#define ST7703_CMD_RDBky				(0x72)
#define ST7703_CMD_RDWx					(0x73)
#define ST7703_CMD_RDWy					(0x74)
#define ST7703_CMD_RDRGLB				(0x75)
#define ST7703_CMD_RDRx					(0x76)
#define ST7703_CMD_RDRy					(0x77)
#define ST7703_CMD_RDGx					(0x78)
#define ST7703_CMD_RDGy					(0x79)
#define ST7703_CMD_RDBALB				(0x7A)
#define ST7703_CMD_RDBx					(0x7B)
#define ST7703_CMD_RDBy					(0x7C)
#define ST7703_CMD_RDAx					(0x7D)
#define ST7703_CMD_RDAy					(0x7E)
#define ST7703_CMD_Read_DDB_start		(0xA1)
#define ST7703_CMD_Read_DDB_continue	(0xA8)
#define ST7703_CMD_RDID1				(0xDA)
#define ST7703_CMD_RDID2				(0xDB)
#define ST7703_CMD_RDID3				(0xDC)
#define ST7703_CMD_SETEXTC				(0xB9)
#define ST7703_CMD_SETDISP				(0xB2)
#define ST7703_CMD_SETRGBIF				(0xB3)
#define ST7703_CMD_SETCYC				(0xB4)
#define ST7703_CMD_SETBGP				(0xB5)
#define ST7703_CMD_SETCOM				(0xB6)
#define ST7703_CMD_SETOTP				(0xB7)
#define ST7703_CMD_SETPOWER_EXT			(0xB8)
#define ST7703_CMD_SETMIPI				(0xBA)
#define ST7703_CMD_SET_VDC				(0xBC)
#define ST7703_CMD_SETSCR				(0xC0)
#define ST7703_CMD_SETPOWER				(0xC1)
#define ST7703_CMD_SETID				(0xC3)
#define ST7703_CMD_SETDDB				(0xC4)
#define ST7703_CMD_SETIO				(0xC7)
#define ST7703_CMD_SETCABC				(0xC8)
#define ST7703_CMD_SETPANEL				(0xCC)
#define ST7703_CMD_DGC_R				(0xCD)
#define ST7703_CMD_DGC_G				(0xCE)
#define ST7703_CMD_DGC_B				(0xCF)
#define ST7703_CMD_SETGAMMA				(0xE0)
#define ST7703_CMD_SETEQ				(0xE3)
#define ST7703_CMD_SET_ROI				(0xE4)
#define ST7703_CMD_SETCOLOR_EN			(0xE5)
#define ST7703_CMD_SETGIP1				(0xE9)
#define ST7703_CMD_SETGIP2				(0xEA)
#define ST7703_CMD_SETCOLOR				(0xEB)
#define ST7703_CMD_TEMP_SENSOR			(0xF1)
#define ST7703_CMD_TEMP_VOLTAGE			(0xF2)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

struct reg_value {
	u8  dsi_data_type;
	u16 delay_after_write;
	u32 buf_size;
	u8  buf[72];
};

static struct reg_value lcd_setup[] =
{
// Enable user command
{0x39, 0, 4 ,{0XB9,0xf1,0x12,0x83}},
// Setdisp Gate number = 480, BLK_CON = 01 => VSSD, RESO_SEL=3 => 640RGB
// WHITE_GND_EN = 0 => Source voltage = Lowest, WHITE_FRAME = 7 => 7 frames
// ISC = 0 => Source output refresh ctl = 0 frames.
{0x39, 0, 4 ,{0XB2,0x00,0x13,0x70}},
// SETRGBIF Vertical back porch VBP_RGB_GEN = 0x10 => 16
// Vertical front porch VFP_RGB_GEN = 0x10 => 16
// Horizontal back porch blank frame period DE_BP_RGB_GEN = 0x28 => 40
// Horizontal front porch blank frame period DE_BP_RGB_GEN = 0x28 => 40
// The rest of the parameters are not documented.
{0x39, 0, 11,{0xB3,0x10,0x10,0x28,0x28,0x03,0xFF,0x00,0x00, 0x00,0x00}},
// SETCYC ZINV_S2401_EN = 1 => S2401 for use, Even row, disable zig-zag
{0x39, 0, 2,{0xB4,0x80}},
// SETBGP VREF_SEL=0x14 => 5.45V
{0x39, 0, 3,{0xB5,0x14,0x14}},
// VCOMDC_F=0x3A => -0.81, VCOMDC_B=0x3A => -0.81
{0x39, 0, 3,{0xB6,0x3A,0x3A}},
// SETPOWER_EXT
{0x39, 0, 2,{0xB8,0x26}}, //2-POWER MODE
// SETMIPI
// 0x31, first nibble undocumented, VCMAIN=0 => virtual channel 0
// Lane = 1 => 2 MIPI lanes
// 0x81 LDO=4 => 1.7 volt, RTERM = 1 => 90 Ohm
// 0x05 IHSRX = 5 => x6
// 0xF9 Tx_clk fDSICLK/8, rest is undocumented
// 0x0E HFP = 14, 0x0E => HBP = 14, What is this
{0x39, 0, 28,{0xBA,0x31,0x81,0x05,0xF9,0x0E,0x0E,0x20,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x25,
0x00,0x90,0x0A,0x00,0x00,0x01,0x4F,0x01,
0x00,0x00,0x37}}, //2-LANES MIPI
// SETVDC, NVDD = -1.8 V, VDD = 1.9
{0x39, 0, 2,{0xBC,0x46}},
// Undocumented
{0x39, 0, 4,{0xBF,0x02,0x11,0x00}},
// SETSCR
{0x39, 0, 10,{0xC0,0x73,0x73,0x50,0x50,0x00,0x00,0x12,0x70,0x00}},
// SETPOWER, missing 3 parameters, according to spec
{0x39, 0, 13,{0xC1,0x43,0x00,0x32,0x32,0x77,0xC1,0xFF,0xFF,
0xEE,0xEE,0xEE,0xEE}},
// 0xC6 Undocumented
{0x39, 0, 7,{0xC6,0x02,0x00,0xFF,0xFF,0x00,0xFF}},
// SETPANEL BGR_PANEL=1 => BGR, REV_PANEL=1 => Normally black
// GS_PANEL=0 => normal dir, SS_PANEL=1 => reverse source direction
{0x39, 0, 2,{0xCC,0x0B}},
// SETGAMMA
{0x39, 0, 35,{0xE0,0x02,0x16,0x25,0x2A,0x2A,0x3F,0x53,0x44,
0x07,0x0D,0x0D,0x12,0x14,0x12,0x13,0x15,
0x1A,0x00,0x16,0x25,0x2A,0x2A,0x3F,0x53,
0x44,0x07,0x0D,0x0D,0x12,0x14,0x12,0x13,
0x15,0x1A}},
// SETEQ
{0x39, 0, 15,{0xE3,0x03,0x03,0x03,0x03,0x00,0x03,0x00,0x00,
0x00,0x00,0xFF,0x80,0xC0,0x10}},

// MADCTR, flip both x and y
{0x39, 0, 2,{0x36,0xC0}},

//set GIP 1
{0x39, 0, 64,{0xE9,0xC8,0x10,0x06,0x01,0xE1,0xDB,0x91,0x12,
0x31,0x23,0x4F,0x82,0xDB,0x91,0x47,0x08,
0x00,0x20,0x02,0x00,0x00,0x00,0x00,0x20,
0x02,0x00,0x00,0x00,0x84,0xF8,0x86,0x64,
0x42,0x20,0x00,0x88,0x88,0x88,0x88,0x85,
0xF8,0x87,0x75,0x53,0x31,0x11,0x88,0x88,
0x88,0x88,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
//set GIP 2
{0x39, 0, 62,{0xEA,0x00,0x1A,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x00,0x00,0x00,0xF1,0x88,0x81,0x13,
0x35,0x57,0x75,0x88,0x88,0x88,0x88,0xF0,
0x88,0x80,0x02,0x24,0x46,0x64,0x88,0x88,
0x88,0x88,0x23,0x10,0x00,0x02,0xC7,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x40,0xDB,
0x91,0x00,0x00,0x00,0x00}},
// SETPANEL again, other value.
// 0x07 => SS_PANEL = 0 => Normal scan dir, GS_PANEL =1 Reverse vertical scan dir
// REV_PANEL = 1 => reverse polarity, BGR_PANEL=1 => BGR
{0x39, 0, 02,{0xCC,0x07}},
// SETIO 0x10 => PWM_OE disabled, INVPWM Original CABC PWM, VOUT_OE VOUT pin output frame sync signal = 1
// HOUT_OE
{0x39, 0, 02,{0xC7,0x10}},
// SLPOUT sleep out, wait 250ms
{0x39, 250,01,{0x11}},
// DISPON display on, wait 50ms
{0x39, 50, 01,{0x29}},
};

static struct fb_videomode otm_lcd_modedb[] = {
	{
		.name = "TRULY-VGA",
		.refresh = 60,		/* refresh */
		.xres = 640,
		.yres = 480,
		.pixclock = 33000,
		.left_margin = 150,
		.right_margin = 100,
		.upper_margin = 16,
		.lower_margin = 16,
		.hsync_len = 90,
		.vsync_len = 4,
		.sync = FB_SYNC_OE_LOW_ACT,
		.vmode =FB_VMODE_NONINTERLACED,
		.flag = 0
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num	= ST7703_TWO_DATA_LANE,
	.max_phy_clk	= ST7703_MAX_DPHY_CLK,
	.dpi_fmt		= MIPI_RGB888,
};
void mipid_st7703_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	pr_err("mipid_st7703_get_lcd_videomode\n");
	*mode = &otm_lcd_modedb[0];
	*size = ARRAY_SIZE(otm_lcd_modedb);
	*data = &lcd_config;
}

#if 0 /* Left for use when reading panel later */
static int st7703_write_reg(struct mipi_dsi_info *mipi_dsi, u32 reg, u32 data)
{
	int err;
	int buf = reg | (data <<8);

	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		&buf, 0);
	if (err)
		dev_err(&mipi_dsi->pdev->dev,"st7703_write_reg err:%d \n",err);
	return err;
}

static int st7703_write_cmd(struct mipi_dsi_info *mipi_dsi, u32 cmd)
{
	int err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE,
		&cmd, 0);
	msleep(1);
	if (err)
		dev_err(&mipi_dsi->pdev->dev,"st7703_write_cmd err:%d \n",err);
	return err;
}
#endif

static int st7703_write_reg(struct mipi_dsi_info *mipi_dsi, struct reg_value *reg)
{
	int ret;
	dev_dbg(&mipi_dsi->pdev->dev, "st7703_lcd_setup write type=0x%02X cmd=0x%02X size %u \n",
				reg->dsi_data_type, reg->buf[0] , reg->buf_size);

	ret = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, reg->dsi_data_type,
					(u32 *)&reg->buf, reg->buf_size);
	if (reg->delay_after_write)
		msleep(reg->delay_after_write);
	if (ret)
		dev_err(&mipi_dsi->pdev->dev, "st7703_lcd_setup Error writing %d\n", ret);
	else
		dev_dbg(&mipi_dsi->pdev->dev, "st7703_lcd_setup wrote OK type=0x%02X cmd=0x%02X size %u \n",
				reg->dsi_data_type, reg->buf[0] , reg->buf_size);
	return ret;
}

int mipid_st7703_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	int i;
	//u8 buf[MIPI_DSI_MAX_RET_PACK_SIZE * 4];
	dev_err(&mipi_dsi->pdev->dev, "mipid_st7703_lcd_setup enter\n");
	if (mipi_dsi->lcd_mipi_sel_gpio)
		gpio_set_value_cansleep(mipi_dsi->lcd_mipi_sel_gpio, 1);
	msleep(20);
#if 0
	// Try to read from the display
	//u8 buf[MIPI_DSI_MAX_RET_PACK_SIZE * 4];
	//ret = st7703_write_cmd(&mipi_dsi->pdev->dev, RDDIDIF);
	//if (ret)
	//	dev_err(&mipi_dsi->pdev->dev, "Error writing packet RDDIDIF\n");
	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	ret = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
				(u32*)buf, 0);
	CHECK_RETCODE(ret);
	buf[0] = ST7703_CMD_RDDIDIF;
	ret = mipi_dsi->mipi_dsi_pkt_read(mipi_dsi, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, (u32*) buf, MIPI_DSI_MAX_RET_PACK_SIZE);
	if (ret)
		dev_err(&mipi_dsi->pdev->dev, "Error writing packet RDDIDIF %d\n", ret);
	else{
		dev_err(&mipi_dsi->pdev->dev, "Got RDDIDIF packet {0x%02X,0x%02X,0x%02X,0x%02X,0x%02X}\n",
			buf[0], buf[1],buf[2],buf[3],buf[4]);
	}
#endif
	for (i = 0; i < ARRAY_SIZE(lcd_setup); i++)
	{
		// Check if we should apply rotation
		if (lcd_setup[i].buf[0] == ST7703_CMD_MADCTL){
			lcd_setup[i].buf[1] = mipi_dsi->rotate_primary_180 ? 0xC0 : 0x0;
		}

		st7703_write_reg(mipi_dsi, &lcd_setup[i]);
	}
	return 0;
}

int mipid_st7703_lcd_power_set(struct mipi_dsi_info *mipi_dsi, int state)
{
	struct device *dev = &mipi_dsi->pdev->dev;

	dev_dbg(&mipi_dsi->pdev->dev, "mipid_st7703_lcd_power_set %i\n", state);

	if (!mipi_dsi->lcd_power_gpio){
		dev_err(dev, "No power interface\n");
		return -ENXIO;
	}

	if (state) {
		dev_dbg(dev, "Power on LCD\n");
		if (mipi_dsi->lcd_power_gpio){
			gpio_set_value_cansleep(mipi_dsi->lcd_power_gpio, 1);
		}
		gpio_set_value_cansleep(mipi_dsi->lcd_mipi_sel_gpio, 1);
		mipi_dsi->mipi_dsi_power_on(mipi_dsi->disp_mipi);
		mipi_dsi->mipi_dsi_set_mode(mipi_dsi, 1);
		msleep((1000 / mipi_dsi->mode->refresh + 1) << 1);
		mipid_st7703_lcd_setup(mipi_dsi);
		mipi_dsi->mipi_dsi_set_mode(mipi_dsi, 0);
		msleep((1000 / mipi_dsi->mode->refresh + 1) << 1);
	} else {
		dev_dbg(dev, "Power off LCD\n");
		if (mipi_dsi->lcd_power_gpio){
			gpio_set_value_cansleep(mipi_dsi->lcd_power_gpio, 0);
		}
		if (mipi_dsi->lcd_mipi_sel_gpio){
		  gpio_set_value_cansleep(mipi_dsi->lcd_mipi_sel_gpio, 0);
		}

	}
	return 0;
}

int mipid_st7703_lcd_power_get(struct mipi_dsi_info *mipi_dsi)
{
	struct device *dev = &mipi_dsi->pdev->dev;
	int power = 0;
	int lcd_mipi_sel, lcd_power;
	dev_dbg(&mipi_dsi->pdev->dev, "mipid_st7703_lcd_power_get\n");
	if (mipi_dsi->lcd_mipi_sel_gpio)
		lcd_mipi_sel = gpio_get_value_cansleep(mipi_dsi->lcd_mipi_sel_gpio);
	if (mipi_dsi->lcd_power_gpio)
		lcd_power = gpio_get_value_cansleep(mipi_dsi->lcd_power_gpio);

	if (lcd_mipi_sel < 0 || lcd_power < 0) {
		dev_err(dev, "failed to get gpio in %s\n", __func__);
		return -ENXIO;
	}

	power = lcd_mipi_sel && lcd_power;
	return power;
}

struct reg_value *st7703_find_reg(int cmd)
{
	int i;
	struct reg_value *reg;
	for (i = 0; i < ARRAY_SIZE(lcd_setup); i++) {
		if (lcd_setup[i].buf[0] == cmd)
			reg = &lcd_setup[i];
	}
	return reg;
}

int mipid_st7703_lcd_rotation_set(struct mipi_dsi_info *mipi_dsi, int state)
{
	struct reg_value config =
		state ? (struct reg_value){ 0x39, 0, 2, { 0x36, 0xC0 } } :
			(struct reg_value){ 0x39, 0, 2, { 0x36, 0x00 } };

	dev_dbg(&mipi_dsi->pdev->dev,
		"mipid_st7703_lcd_rotation_set  writing %u\n", state);

	msleep(20);
	st7703_write_reg(mipi_dsi, &config);
	st7703_write_reg(mipi_dsi, st7703_find_reg(ST7703_CMD_SLPOUT));
	st7703_write_reg(mipi_dsi, st7703_find_reg(ST7703_CMD_DISPON));
	return 0;
}