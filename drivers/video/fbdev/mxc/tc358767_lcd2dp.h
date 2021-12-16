/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DRIVERS_VIDEO_TC358767_DSI2EDP_H
#define __DRIVERS_VIDEO_TC358767_DSI2EDP_H

#define TC358767_RATES	(SNDRV_PCM_RATE_32000|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000 \
                        |SNDRV_PCM_RATE_88200|SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_176400 \
                        |SNDRV_PCM_RATE_192000)

#define TC358767_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE \
                         | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_3LE)


enum lcd2dp_init_state {
	LCD2DP_IDLE,		// DP output is off
	LCD2DP_SETUP_OK,	// DP setup is OK, DP link active
	LCD2DP_NO_SUPPORT,	// Attached display lacks required support
	LCD2DP_EDID_OK,		// Attached display has required support
	LCD2DP_ACTIVE,		// Video (and possibly audio) output to display
	LCD2DP_SLEEP,		// DP link active but a/v flow disabled
	LCD2DP_ERROR,		// Something went wrong
	LCD2DP_SUSPENDED,	// Driver is PM suspended
};

enum lcd2dp_edid_state {
	LCD2DP_E_NOTREAD = 0,
	LCD2DP_E_OK = 1,
	LCD2DP_E_NO_SUPP = 2,
};

enum lcd2dp_video_attach_state {
	LCD2DP_V_UNATTACHED = 0,
	LCD2DP_V_ATTACHED = 1,
};

enum lcd2dp_audio_attach_state {
	LCD2DP_A_UNATTACHED = 0,
	LCD2DP_A_ATTACHED = 1,
};

enum lcd2dp_sync_state {
	LCD2DP_S_OFF = 0,
	LCD2DP_S_VIDEO = 1,
	LCD2DP_S_VIDEO_SWITCH = 2,
	LCD2DP_S_PLAYBACK = 3,
};

struct mxc_lcd2dp {
	struct snd_soc_component *component;
	struct i2c_client *client_i2c;
	struct device *dev;
	struct regmap *regmap;
	struct task_struct *thread;
	bool i2c_shutdown;
	struct mutex lock;
	int reset_gpio;
	int shdn_gpio;
	int irq;
	unsigned int sysclk;
	u32 i2scfg_reg;
	int audio_mute;
	int sleep;
	union {
		unsigned int word;
		struct {
			unsigned int init:4;
			unsigned int edid:2;
			unsigned int video_attach:1;
			unsigned int audio_attach:1;
			unsigned int sync:2;
			unsigned int sync_req:2;
		};
	} state;
	union {
		unsigned int word;
		struct {
			unsigned int disable:1;
			unsigned int enable:1;
			unsigned int hpd_irq:1;
			unsigned int sync:1;
		};
	} dp_req;
	union {
		unsigned int word;
		struct {
			unsigned int audio_channels:2;
			unsigned int audio_rate:16;
			unsigned int v640x480x60:1;
		};
	} sink_cap;
	unsigned int setup_retry;
	unsigned int edid_retry;
};

// DSI
#define DSI_INTSTATUS		0x0220

// DPI - Parallel input
#define DPIP_PXL_FMT		0x0440

// Video Path0
#define VIDEO_PATH0_CTRL	0x0450
#define H_TIMING_CTRL01		0x0454
#define H_TIMING_CTRL02		0x0458
#define V_TIMING_CTRL01		0x045c
#define V_TIMING_CTRL02		0x0460
#define VIDEO_FRAME_TIMING_UPLOAD_ENB0	0x0464

// System
#define DP_IDREG		0x0500
#define SYS_BOOT		0x0504
#define SYS_STAT		0x0508
#define SYS_RSTENB		0x050C
#define SYS_CTRL		0x0510

// GPIO
#define GPIO_INPUT		0x054c

// Interrupt
#define INTCTL_G		0x0560
#define INTSTS_G		0x0564
#define INT_GP0_LCNT		0x0584

// DP Control
#define DP0_CTRL		0x0600

// DP Clock
#define VIDEO_VID_FM		0x0610
#define VIDEO_VID_N		0x0614
#define VIDEO_VID_M		0x0618
#define AUDIO_VID_FM		0x0628
#define AUDIO_VID_N		0x062c
#define AUDIO_VID_M		0x0630

// DP0 Main
#define DP0_SEC_SAMPLE		0x0640
#define DP0_VIDEO_SYNC_DELAY	0x0644
#define DP0_TOT_VAL		0x0648
#define DP0_START_VAL		0x064c
#define DP0_ACTIVE_VAL		0x0650
#define DP0_SYNC_VAL		0x0654
#define DP0_MISC		0x0658

// DP0 AUX
#define DP0_AUX_CFG0		0x0660
#define DP0_AUX_CFG1		0x0664
#define DP0_AUX_ADDR		0x0668
#define DP0_AUX_WDATA0		0x066c
#define DP0_AUX_RDATA0		0x067c
#define DP0_AUX_RDATA1		0x0680
#define DP0_AUX_RDATA2		0x0684
#define DP0_AUX_RDATA3		0x0688
#define DP0_AUX_STAT		0x068c

// DP0 Link
#define DP0_SRC_CTRL		0x06a0
#define DP0_LTSTAT		0x06d0
#define DP0_LTLOOP_CTRL		0x06d8
#define DP0_SNK_LTCTRL		0x06e4

// DP Audio Config Register
#define AUDCFG0			0x0700
#define AUDCFG1			0x0704
#define AUDIFDATA0		0x0708
#define AUDIFDATA1		0x070C
#define AUDIFDATA2		0x0710
#define AUDIFDATA3		0x0714
#define AUDIFDATA4		0x0718
#define AUDIFDATA5		0x071C
#define AUDIFDATA6		0x0720

// DP1 Link
#define DP1_SRC_CTRL		0x07A0

// DP PHY
#define DP_PHY_CTRL		0x0800

// I2S Audio Config Register
#define I2SCFG			0x0880
#define I2SCH0STAT0		0x0888
#define I2SCH0STAT1		0x088c
#define I2SCH0STAT2		0x0890
#define I2SCH0STAT3		0x0894
#define I2SCH0STAT4		0x0898
#define I2SCH0STAT5		0x089c
#define I2SCH1STAT0		0x08a0
#define I2SCH1STAT1		0x08a4
#define I2SCH1STAT2		0x08a8
#define I2SCH1STAT3		0x08ac
#define I2SCH1STAT4		0x08b0
#define I2SCH1STAT5		0x08b4

// PLL
#define DP0_PLL_CTRL		0x0900
#define DP1_PLL_CTRL		0x0904
#define PXL_PLL_CTRL		0x0908
#define PXL_PLL_PARAM		0x0914
#define SYS_PLL_PARAM		0x0918

// Debug
#define TEST_CTL		0x0A00

/* Bits, masks and shifts */

/* I2SCFG */
#define TC358767_SAMPLE_LEN_16BITS		0x00
#define TC358767_SAMPLE_LEN_18BITS		0x02
#define TC358767_SAMPLE_LEN_20BITS		0x04
#define TC358767_SAMPLE_LEN_24BITS		0x08

#define TC358767_I2SCFG_SAMPLELEN_MASK		0x00F0
#define TC358767_I2SCFG_SAMPLELEN_SHIFT	(4)

#define TC358767_I2SCFG_ALIGN_MASK		0x0300
#define TC358767_I2SCFG_ALIGN_SHIFT		(8)
#define TC358767_I2S_MODE			0x00
#define TC358767_LEFT_JUSTIFIED_MODE		0x01
#define TC358767_RIGHT_JUSTIFIED_MODE		0x02

/* AUDCFG0 */
#define TC358767_MUTE_MASK			0x01000000

#endif
