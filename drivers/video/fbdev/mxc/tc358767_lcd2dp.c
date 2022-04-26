/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2016, FLIR Systems AB.  All rights reserved.
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

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/swab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/sysfs.h>
#include <linux/mxcfb.h>
#include <linux/video/mxc/tc358767_lcd2dp.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tc358767_lcd2dp.h"


#define DP_RETRIES 5
#define DP_RETRY_MDELAY 500
#define EDID_RETRIES 2
#define EDID_RETRY_MDELAY 300
#define SYNC_NOTIFY_MDELAY 100

/* Set to 1 to replace DPI image with internally generated color bars */
#define COLOR_BAR 0


static inline int lcd2dp_poll_timeout(struct mxc_lcd2dp *lcd2dp, unsigned int addr,
				  unsigned int cond_mask,
				  unsigned int cond_value,
				  unsigned long sleep_us, u64 timeout_us)
{
	unsigned int val;

	return regmap_read_poll_timeout(lcd2dp->regmap, addr, val,
					(val & cond_mask) == cond_value,
					sleep_us, timeout_us);
}

static int lcd2dp_aux_wait_busy(struct mxc_lcd2dp *lcd2dp)
{
	return lcd2dp_poll_timeout(lcd2dp, DP0_AUX_STAT, AUX_BUSY, 0, 100, 100000);
}

static inline void lcd2dp_reg_write(struct mxc_lcd2dp *lcd2dp,
					unsigned int addr,
					unsigned int val)
{
	int ret;

	BUG_ON(!lcd2dp);
	if (lcd2dp->i2c_shutdown) {
		dev_err(&lcd2dp->client_i2c->dev, "i2c shutdown\n");
		return;
	}

	dev_dbg(&lcd2dp->client_i2c->dev, "Write to %X val = %X\n", addr, val);
	/*
	 * TC358767 requires register address in big endian
	 * and register value in little endian.
	 * Regmap currently sends it all in big endian.
	 */
	ret = regmap_write(lcd2dp->regmap, addr, val); //__swab32(val));
	if (ret)
		dev_err(&lcd2dp->client_i2c->dev, "regmap_write addr %X ret %d\n", addr, ret);
}

static inline void lcd2dp_reg_read(struct mxc_lcd2dp *lcd2dp,
					unsigned int addr,
					unsigned int *val)
{
	int ret;

	BUG_ON(!lcd2dp);
	if (lcd2dp->i2c_shutdown) {
		dev_err(&lcd2dp->client_i2c->dev, "i2c shutdown\n");
		return;
	}

	ret = regmap_read(lcd2dp->regmap, addr, val);
	if (ret)
		dev_err(&lcd2dp->client_i2c->dev, "regmap_read addr %X ret %d\n", addr, ret);
	/*
	 * TC358767 returns register value in little endian.
	 * Covert it to big endian.
	 */
	//*val = __swab32(*val);

	dev_dbg(&lcd2dp->client_i2c->dev, "Read from %X val = %X\n", addr, *val);
}

static void tc358767_clk_on(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "codec clock -> on\n");
}

static void tc358767_clk_off(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "codec clock -> off\n");
}

static void tc358767_reinit(struct snd_soc_component *component)
{
        /* Set interface control */
#if 0
	snd_soc_update_bits(codec, I2SCFG,
			    TC358767_I2SCFG_ALIGN_MASK,
			    lcd2dp->i2scfg_reg);
#endif
}

static int tc358767_power_on(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "codec power -> on\n");
	tc358767_reinit(component);
	return 0;
}

static int tc358767_power_off(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "codec power -> off\n");
	return 0;
}

static int tc358767_set_bias_level(struct snd_soc_component *component,
				   enum snd_soc_bias_level level)
{
	dev_dbg(component->dev, "## %s: %d -> %d\n", __func__,
		snd_soc_component_get_bias_level(component), level);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_STANDBY)
			tc358767_clk_on(component);
		break;
	case SND_SOC_BIAS_STANDBY:
		switch (snd_soc_component_get_bias_level(component)) {
		case SND_SOC_BIAS_OFF:
			tc358767_power_on(component);
			break;
		case SND_SOC_BIAS_PREPARE:
			tc358767_clk_off(component);
			break;
		default:
			BUG();
		}
		break;
	case SND_SOC_BIAS_OFF:
		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_STANDBY)
			tc358767_power_off(component);
		break;
	}
	component->dapm.bias_level = level;

	return 0;
}

static int tc358767_codec_probe(struct snd_soc_component *component)
{
	struct mxc_lcd2dp *lcd2dp = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "codec probe\n");
	if (lcd2dp)
		lcd2dp->component = component;

	return 0;
}

static void tc358767_codec_remove(struct snd_soc_component *component)
{
	/* power down chip */
	dev_dbg(component->dev, "codec remove\n");
	tc358767_set_bias_level(component, SND_SOC_BIAS_OFF);
}

static unsigned int tc358767_codec_read(struct snd_soc_component *component, unsigned int reg)
{
	struct mxc_lcd2dp *lcd2dp = snd_soc_component_get_drvdata(component);
    u32 value = 0;

	mutex_lock(&lcd2dp->lock);
	lcd2dp_reg_read(lcd2dp, reg, &value);
	mutex_unlock(&lcd2dp->lock);

	return value;
}

static int tc358767_codec_write(struct snd_soc_component *component, 
                                 unsigned int reg,
                                 unsigned int value)
{
	struct mxc_lcd2dp *lcd2dp = snd_soc_component_get_drvdata(component);

	mutex_lock(&lcd2dp->lock);
	lcd2dp_reg_write(lcd2dp, reg, value);
	mutex_unlock(&lcd2dp->lock);
	return 0;
}

static int tc358767_suspend(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "codec suspend\n");
	tc358767_set_bias_level(component, SND_SOC_BIAS_OFF);
	return 0;
}

static int tc358767_resume(struct snd_soc_component *component)
{
	dev_dbg(component->dev, "codec resume\n");
	tc358767_set_bias_level(component, SND_SOC_BIAS_STANDBY);
	return 0;
}

static const struct snd_soc_component_driver soc_codec_driver_tc358767 = {
	.probe			= tc358767_codec_probe,
	.remove			= tc358767_codec_remove,
	.set_bias_level		= tc358767_set_bias_level,
    .read               = tc358767_codec_read,
    .write              = tc358767_codec_write,
    .resume             = tc358767_resume,
    .suspend            = tc358767_suspend,
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};


//static struct snd_soc_codec_driver soc_codec_driver_tc358767 = {
//	.probe			= tc358767_codec_probe,
//	.remove			= tc358767_codec_remove,
//    .read                   = tc358767_codec_read,
//    .write                  = tc358767_codec_write,
//	.suspend		= tc358767_suspend,
//	.resume			= tc358767_resume,
//	.set_bias_level		= tc358767_set_bias_level,
//};

static int tc358767_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct mxc_lcd2dp *lcd2dp = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "## %s: clk_id = %d, freq = %d, dir = %d\n",
		__func__, clk_id, freq, dir);

	lcd2dp->sysclk = freq;
	return 0;
}


static int tc358767_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
//struct snd_soc_codec *codec = dai->codecs;
//	u32 data = 0;

	dev_dbg(component->dev, "## %s: format %d width %d rate %d\n",
		__func__, params_format(params), params_width(params),
		params_rate(params));

#if 0
	switch (params_width(params)) {
	case 16:
		data = (TC358767_SAMPLE_LEN_16BITS <<
			TC358767_I2SCFG_SAMPLELEN_SHIFT);
		break;
	case 18:
		data = (TC358767_SAMPLE_LEN_18BITS <<
			TC358767_I2SCFG_SAMPLELEN_SHIFT);
		break;
	case 20:
		data = (TC358767_SAMPLE_LEN_20BITS <<
			TC358767_I2SCFG_SAMPLELEN_SHIFT);
		break;
	case 24:
		data = (TC358767_SAMPLE_LEN_24BITS <<
			TC358767_I2SCFG_SAMPLELEN_SHIFT);
		break;
	default:
		dev_err(codec->dev, "%s: Unsupported format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	snd_soc_update_bits(codec, I2SCFG,
			    TC358767_I2SCFG_SAMPLELEN_MASK,
			    data);
#endif
	return 0;
}

static int tc358767_set_dai_fmt(struct snd_soc_dai *codec_dai,
			        unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	//struct snd_soc_codec *codec = codec_dai->codecs;
//	struct mxc_lcd2dp *lcd2dp = snd_soc_codec_get_drvdata(codec);
//	u32 i2scfg_reg = 0;

	dev_dbg(component->dev, "## %s: fmt = 0x%x\n", __func__, fmt);

#if 0
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
        case SND_SOC_DAIFMT_CBS_CFS:
                /* Codec is clk & FRM slave */
		break;
	default:
		dev_alert(codec->dev, "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2scfg_reg |= (TC358767_RIGHT_JUSTIFIED_MODE <<
			       TC358767_I2SCFG_ALIGN_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2scfg_reg |= (TC358767_LEFT_JUSTIFIED_MODE <<
			       TC358767_I2SCFG_ALIGN_SHIFT);
		break;
	default:
		dev_err(codec->dev, "Invalid DAI interface format\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, I2SCFG,
			    TC358767_I2SCFG_ALIGN_MASK,
			    i2scfg_reg);

	lcd2dp->i2scfg_reg = i2scfg_reg;
#endif

	return 0;
}

static int tc358767_dac_mute(struct snd_soc_dai *codec_dai, int mute, int direction)
{
	//struct snd_soc_codec *codec = codec_dai->codecs;
	//struct mxc_lcd2dp *lcd2dp = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_component *component = codec_dai->component;
	struct mxc_lcd2dp *lcd2dp = snd_soc_component_get_drvdata(component);
	unsigned int val;

	dev_dbg(component->dev, "codec %s\n", mute ? "mute" : "un-mute");

	mutex_lock(&lcd2dp->lock);
	if (mute != lcd2dp->audio_mute) {
		lcd2dp->audio_mute = mute;

		if (lcd2dp->state.init == LCD2DP_ACTIVE) {
			lcd2dp_reg_read(lcd2dp, AUDCFG0, &val);
			if (mute) {
				if (!(val & TC358767_MUTE_MASK)) {
					val |= TC358767_MUTE_MASK;
					lcd2dp_reg_write(lcd2dp, AUDCFG0, val);
				}
			} else {
				if (val & TC358767_MUTE_MASK) {
					val &= ~TC358767_MUTE_MASK;
					lcd2dp_reg_write(lcd2dp, AUDCFG0, val);
				}
			}
		}
	}
	mutex_unlock(&lcd2dp->lock);

	return 0;
}

static struct snd_soc_dai_ops tc358767_dai_ops = {
	.hw_params	= tc358767_hw_params,
	.set_sysclk	= tc358767_set_dai_sysclk,
	.set_fmt	= tc358767_set_dai_fmt,
	.mute_stream	= tc358767_dac_mute,
};

static struct snd_soc_dai_driver tc358767_dai_driver[] = {
	{
		.name = "tc358767",
		.playback = {
			.stream_name	 = "Playback",
			.channels_min	 = 1,
			.channels_max	 = 2,
			.rates		 = SNDRV_PCM_RATE_32000,
			.formats	 = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tc358767_dai_ops,
		.symmetric_rates = 1,
	}
};

static bool tc_readable_reg(struct device *dev, unsigned int reg)
{
	return reg != SYS_CTRL;
}

static bool tc_writeable_reg(struct device *dev, unsigned int reg)
{
	return (reg != DP_IDREG) &&
	       (reg != DP0_LTSTAT) &&
	       (reg != DP0_SNKLTCHGREQ);
}


static const struct regmap_range tc_volatile_ranges[] = {
	regmap_reg_range(DP0_AUX_WDATA0, DP0_AUX_STAT),
	regmap_reg_range(DP0_LTSTAT, DP0_SNKLTCHGREQ),
	regmap_reg_range(DP_PHY_CTRL, DP_PHY_CTRL),
	regmap_reg_range(DP0_PLL_CTRL, PXL_PLL_CTRL),
	regmap_reg_range(VIDEO_FRAME_TIMING_UPLOAD_ENB0, VIDEO_FRAME_TIMING_UPLOAD_ENB0),
	regmap_reg_range(INTSTS_G, INTSTS_G),
	regmap_reg_range(GPIO_INPUT, GPIO_INPUT),
};

static const struct regmap_access_table tc_volatile_table = {
	.yes_ranges = tc_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(tc_volatile_ranges),
};


static const struct regmap_config lcd2dp_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.max_register = 0xb00,
	.reg_stride = 4,
	.cache_type = REGCACHE_RBTREE,
	.readable_reg = tc_readable_reg,
	.volatile_table = &tc_volatile_table,
	.writeable_reg = tc_writeable_reg,
    .reg_format_endian = REGMAP_ENDIAN_BIG,
    .val_format_endian = REGMAP_ENDIAN_LITTLE,
};


static ssize_t lcd2dp_sysfs_show(struct device* dev,
				 struct device_attribute* attr, char* buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc_lcd2dp *lcd2dp =
		(struct mxc_lcd2dp *)i2c_get_clientdata(client);
	unsigned val = 0;

	if (!strcmp(attr->attr.name, "audio_attached"))
		val = lcd2dp->state.audio_attach;
	else if (!strcmp(attr->attr.name, "audio_channels"))
		val = lcd2dp->sink_cap.audio_channels;
	else if (!strcmp(attr->attr.name, "audio_rate"))
		val = lcd2dp->sink_cap.audio_rate;
	else if (!strcmp(attr->attr.name, "video_attached"))
		val = lcd2dp->state.video_attach;
	else if (!strcmp(attr->attr.name, "sync"))
		val = lcd2dp->state.sync;
	else if (!strcmp(attr->attr.name, "sleep"))
		val = lcd2dp->sleep;
	else
		dev_err(dev, "Unknown sysfs file %s\n", attr->attr.name);

	return sprintf(buf, "%u\n", val);
}

static ssize_t lcd2dp_sysfs_store(struct device* dev,
				  struct device_attribute* attr,
				  const char* buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc_lcd2dp *lcd2dp =
		(struct mxc_lcd2dp *)i2c_get_clientdata(client);
	ssize_t err = 0;

	if (!strcmp(attr->attr.name, "sync")) {

		mutex_lock(&lcd2dp->lock);
		if (((count == 1) || ((count >= 2) && (buf[1] == '\n'))) &&
		    (buf[0] >= '0') && (buf[0] <= '3')) {
			lcd2dp->state.sync_req = buf[0] - '0';
			lcd2dp->dp_req.sync = true;
			if (lcd2dp->thread)
				wake_up_process(lcd2dp->thread);
		} else {
			err = -EINVAL;
		}
		mutex_unlock(&lcd2dp->lock);

	} else if (!strcmp(attr->attr.name, "sleep")) {

		mutex_lock(&lcd2dp->lock);
		if (((count == 1) || ((count >= 2) && (buf[1] == '\n'))) &&
		    (buf[0] >= '0') && (buf[0] <= '1')) {
			lcd2dp->sleep = buf[0] - '0';
			lcd2dp->dp_req.sync = true;
			if (lcd2dp->thread)
				wake_up_process(lcd2dp->thread);
		} else {
			err = -EINVAL;
		}
		mutex_unlock(&lcd2dp->lock);

	} else {
		dev_err(dev, "Unknown sysfs file %s\n", attr->attr.name);
		err = -ENODEV;
	}

	return err ? err : count;
}

static ssize_t lcd2dp_sysfs_debug_show(struct device* dev,
				       struct device_attribute* attr, char* buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc_lcd2dp *lcd2dp =
		(struct mxc_lcd2dp *)i2c_get_clientdata(client);
	ssize_t len;

	len = sprintf(buf,
		"i2c_shutdown  = %d\n"
		"audio_mute    = %d\n"
		"sleep         = %d\n"
		"state.word    = 0x%08x\n"
		"dp_req.word   = 0x%08x\n"
		"sink_cap.word = 0x%08x\n"
		"setup_retry   = %u\n"
		"edid_retry    = %u\n",
		lcd2dp->i2c_shutdown ? 1 : 0,
		lcd2dp->audio_mute, lcd2dp->sleep,
		lcd2dp->state.word, lcd2dp->dp_req.word, lcd2dp->sink_cap.word,
		lcd2dp->setup_retry, lcd2dp->edid_retry
	);

	return len;
}

static DEVICE_ATTR(audio_attached, S_IRUSR|S_IRGRP|S_IROTH, lcd2dp_sysfs_show, NULL);
static DEVICE_ATTR(audio_channels, S_IRUSR|S_IRGRP|S_IROTH, lcd2dp_sysfs_show, NULL);
static DEVICE_ATTR(audio_rate,     S_IRUSR|S_IRGRP|S_IROTH, lcd2dp_sysfs_show, NULL);
static DEVICE_ATTR(video_attached, S_IRUSR|S_IRGRP|S_IROTH, lcd2dp_sysfs_show, NULL);
static DEVICE_ATTR(sync, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH,
					lcd2dp_sysfs_show, lcd2dp_sysfs_store);
static DEVICE_ATTR(sleep, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH,
					lcd2dp_sysfs_show, lcd2dp_sysfs_store);
static DEVICE_ATTR(debug, S_IRUSR|S_IRGRP|S_IROTH, lcd2dp_sysfs_debug_show, NULL);

static struct attribute *lcd2dp_sysfs_sink_caps_attrs[] = {
	&dev_attr_audio_attached.attr,
	&dev_attr_audio_channels.attr,
	&dev_attr_audio_rate.attr,
	&dev_attr_video_attached.attr,
	&dev_attr_sync.attr,
	&dev_attr_sleep.attr,
	&dev_attr_debug.attr,
	NULL
};

static struct attribute_group lcd2dp_sysfs_sink_caps_attr_grp = {
    .name = "sink_caps",
    .attrs = lcd2dp_sysfs_sink_caps_attrs,
};


static int lcd2dp_enable(struct mxc_lcd2dp *lcd2dp)
{
	unsigned int val;

	dev_info(&lcd2dp->client_i2c->dev, "enable\n");

	// DP display output control
	lcd2dp_reg_write(lcd2dp, VIDEO_PATH0_CTRL, 0x00800100);	// VSDELAY=8, RGB888, FRMSYNC=from input
#if 0  // Output timing taken from input pixel stream
	lcd2dp_reg_write(lcd2dp, H_TIMING_CTRL01, 0x00300060);	// HBP=48, HSYNC=96
	lcd2dp_reg_write(lcd2dp, H_TIMING_CTRL02, 0x00100280);	// HFP=16, HW=640
	lcd2dp_reg_write(lcd2dp, V_TIMING_CTRL01, 0x00210002);	// VBP=33, VSYNC=2
	lcd2dp_reg_write(lcd2dp, V_TIMING_CTRL02, 0x000A01E0);	// VFP=10, VH=480
	lcd2dp_reg_write(lcd2dp, VIDEO_FRAME_TIMING_UPLOAD_ENB0, 0x1); // Set timing
#endif

	// Parallel input control
	lcd2dp_reg_write(lcd2dp, DPIP_PXL_FMT, 0x0600);		// VSYNC=lo, HSYNC=lo, DE=hi, LSB aligned, RGB888

	// DP Main stream attributes
	lcd2dp_reg_write(lcd2dp, DP0_SEC_SAMPLE, 0x000a000a);	// Max 10 audio samples/line
	lcd2dp_reg_write(lcd2dp, DP0_VIDEO_SYNC_DELAY, 0x003e0310);
	lcd2dp_reg_write(lcd2dp, DP0_TOT_VAL, 0x020D0320);	// Total size
	lcd2dp_reg_write(lcd2dp, DP0_START_VAL, 0x00230090);
	lcd2dp_reg_write(lcd2dp, DP0_ACTIVE_VAL, 0x01e00280);	// Active size
	lcd2dp_reg_write(lcd2dp, DP0_SYNC_VAL, 0x80028060);	// HSYNC / VSYNC

	// DP Flow Shape & TimeStamp
	lcd2dp_reg_write(lcd2dp, DP0_MISC, 0x1F3f0020);

#if COLOR_BAR
	// Color bar test
	lcd2dp_reg_write(lcd2dp, TEST_CTL, 0x78006312);
#else
	// Color bar test
	lcd2dp_reg_write(lcd2dp, TEST_CTL, 0xFFFFFF10);
#endif

	// Video M / N setting
	lcd2dp_reg_write(lcd2dp, VIDEO_VID_FM, 0x179a);
	lcd2dp_reg_write(lcd2dp, VIDEO_VID_N, 0x97e0);

	if (lcd2dp->state.audio_attach == LCD2DP_A_ATTACHED) {

		/* FIXME:
		 *
		 * DP 1.4 spec refers to EIA CEA-861-F, I have only rev D.
		 * DP 1.4 spec refers to EIC 60958-1:2014, I have only rev 2004.
		 * DP 1.4 spec refers to EIC 60958-3:2015, I have only rev 2003.
		 *
		 * From EIA CEA-861-D:
		 * NOTE—HDMI requires the CT, SS and SF fields to be set to 0
		 * ("Refer to Stream Header") as these items are carried in the
		 * audio stream.
		 * However we are sending DP that is later converted to HDMI so
		 * I have chosen to supply the values anyway.
		 *
		 * From DP 1.4 spec:
		 * When 1- or 2-channel LPCM or One Bit audio is transported,
		 * CEA-861-F defines only a single channel-to-speaker mapping.
		 * Therefore, the transmission of an Audio INFOFRAME SDP may be
		 * omitted altogether.
		 * -- How to disable?
		 */

		// Audio info frame packet (EIA CEA-861-D, DP 1.4)
		// 0x1X - DB0 7-4:0001=PCM, 3:0=Reserved, 2-0:000=1ch:001=2ch
		// 0x0X - DB1 7-5:000=Reserved, 4-2:001=32kHz:010=44.1kHz:011=48kHz, 1-0:01=16bit
		// 0x00 - DB2 (reserved)
		// 0x00 - DB3 0x00=Ch2 is Front Right, Ch1 is Front Left
		val = 0x00000110 | (lcd2dp->sink_cap.audio_channels - 1);
		switch (lcd2dp->sink_cap.audio_rate) {
		case 32000: val |= (1 << 10); break;
		case 44100: val |= (2 << 10); break;
		case 48000: val |= (3 << 10); break;
		default: BUG();
		}
		lcd2dp_reg_write(lcd2dp, AUDIFDATA0, val);
		// 0x00 - DB4 7:0=No info, 6-3:0000=Level shift n/a, 2-0:000=Reserved
		// 0x00 - DB5-27=0x00 (reserved/padding), AUDIFDATAx default=0
		lcd2dp_reg_write(lcd2dp, AUDIFDATA1, 0);

		// AUDCFG0: 24:x=Audio Mute, 8:1=Num chan - 1, 7-0:0x00=Audio packet ID
		val = lcd2dp->audio_mute ? 0x01000000 : 0x00000000;
		val |= (lcd2dp->sink_cap.audio_channels - 1) << 8;
		lcd2dp_reg_write(lcd2dp, AUDCFG0, val);

		// AUDCFG1: 7-0:0x84 CEA-861-F Audio INFOFRAME (HB1 value)
		lcd2dp_reg_write(lcd2dp, AUDCFG1, 0x84);

		// I2SCHxSTAT0: Channel status bits (IEC 60958-1 and -3)
		// 31-30:00=Reserved, 29-28:11=Accuracy n/a,
		// 27-24:0011=32kHz:0000=44.1kHz:0010=48kHz,
		// 23-20:0000=Channel n/a, 19-16:0000=Source n/a,
		// 15-8:0x00=General category (temporary use),
		// 7-6:00=Mode 0, 5-3:000=2 ch without pre-emphasis,
		// 2:0=copyright, 1:0=Main data field is LPCM, 0:0=Consumer use
		val = 0x30000000;
		switch (lcd2dp->sink_cap.audio_rate) {
		case 32000: val |= (3 << 24); break;
		case 44100: val |= (0 << 24); break;
		case 48000: val |= (2 << 24); break;
		default: BUG();
		}
		lcd2dp_reg_write(lcd2dp, I2SCH0STAT0, val);
		lcd2dp_reg_write(lcd2dp, I2SCH1STAT0, val);
		// 63-40:0x000000=Reserved
		// 39-36:0000=Original freq not indicated, 35-32:0010=16bit
		val = 0x00000002;
		lcd2dp_reg_write(lcd2dp, I2SCH0STAT1, val);
		lcd2dp_reg_write(lcd2dp, I2SCH1STAT1, val);
		// 191-64:0x0=Reserved, I2SCHxSTATx default=0

		// I2SCFG: 9-8:0=std align, 7-4:0=16 bit, 2:0=IEC60958 valid bit,
		//         1:1=Enable IEC60958 gen, 0:0=Audio input disable
		lcd2dp_reg_write(lcd2dp, I2SCFG, 0x00000002);

		// Audio M / N setting (Acc to DP spec look-up table for 1.62Gb)
		switch (lcd2dp->sink_cap.audio_rate) {
		case 32000:
			lcd2dp_reg_write(lcd2dp, AUDIO_VID_FM, 1024);
			lcd2dp_reg_write(lcd2dp, AUDIO_VID_N, 10125);
			break;
		case 44100:
			lcd2dp_reg_write(lcd2dp, AUDIO_VID_FM, 784);
			lcd2dp_reg_write(lcd2dp, AUDIO_VID_N, 5625);
			break;
		case 48000:
			lcd2dp_reg_write(lcd2dp, AUDIO_VID_FM, 512);
			lcd2dp_reg_write(lcd2dp, AUDIO_VID_N, 3375);
			break;
		default:
			BUG();
		}
	}

	// Enable video channel
	val = 0x61;					// Vid MN gen + Enh frm + DPTX en
	if (lcd2dp->state.audio_attach == LCD2DP_A_ATTACHED)
		val |= 0x80;				// + Aud MN gen
	lcd2dp_reg_write(lcd2dp, DP0_CTRL, val);

	lcd2dp->state.sync_req = LCD2DP_S_VIDEO;
	if (lcd2dp->sleep) {
		lcd2dp->state.sync = LCD2DP_S_OFF;
	} else {
		// Video tx enable
		val |= 0x02;
		lcd2dp_reg_write(lcd2dp, DP0_CTRL, val);

		// Enable video source
#if COLOR_BAR
		lcd2dp_reg_write(lcd2dp, SYS_CTRL, 0x0003);	// VideoSrc=Bars, AudioSrc=OFF
#else
		lcd2dp_reg_write(lcd2dp, SYS_CTRL, 0x0002);	// VideoSrc=DPI, AudioSrc=OFF
#endif

		lcd2dp->state.sync = LCD2DP_S_VIDEO;
	}
	sysfs_notify(&lcd2dp->client_i2c->dev.kobj, "sink_caps", "sync");

	// Reset status
	lcd2dp_reg_read(lcd2dp, SYS_STAT, &val);

	msleep(100);

	// Check link
	lcd2dp_reg_read(lcd2dp, VIDEO_VID_M, &val);
	if ((val < (0x179a-1)) || (val > 0x179a+1))
		dev_err(&lcd2dp->client_i2c->dev, "MN status %X\n", val);
	lcd2dp_reg_read(lcd2dp, SYS_STAT, &val);
	if ((val & 0x0353) != 0) {			// Ignore ABOVF
		dev_err(&lcd2dp->client_i2c->dev, "Sysstat %X\n", val);
		return 1;
	}
	lcd2dp_reg_read(lcd2dp, DSI_INTSTATUS, &val);
	if (val)
		dev_err(&lcd2dp->client_i2c->dev, "DSI int %X\n", val);

	return 0;
}

#if 0 // Only "basic audio" used
static void lcd2dp_parse_edid_short_audio_descr(struct mxc_lcd2dp *lcd2dp,
			unsigned char b0, unsigned char b1, unsigned char b2)
{
	/* Parsing of 3-byte audio descriptor according to:
	 * - CEA-861-D A DTV Profile for Uncompressed High Speed Digital Interfaces
	 * b0: bit6-3 = Audio Format Code, 0001 = LPCM, bit2-0 = # of channels -1
	 * b1: bit6-0 = Sample Rate (bitmask), bit2=48, bit1=44.1, bit0=32 kHz
	 * b2: bit2-0 = Word length (bitmask), bit2=24, bit1=20, bit0=16 bits
	 */
	unsigned int channels;
	unsigned int rate;

	if ((b0 & 0x78) != 0x08)
		return;	// Not LPCM

	if ((b2 & 0x01) == 0)
		return;	// Not 16 bit

	channels = (b0 & 0x07) + 1;
	if (channels > 2)
		return;

	/* Check for sample rates in order of preference */
	if (b1 & 0x01)
		rate = 32000;
	else if (b1 & 0x02)
		rate = 44100;
	else if (b1 & 0x04)
		rate = 48000;
	else
		return;

	/* A valid mode found */
	dev_info(&lcd2dp->client_i2c->dev, "- %uch %uHz 16b LPCM supported\n",
		 channels, rate);

	if (lcd2dp->state.audio_attach == LCD2DP_A_ATTACHED) {
		if ((rate * channels) >=
                    (lcd2dp->sink_cap.audio_rate *
		     lcd2dp->sink_cap.audio_channels))
			return;	// Lower BW is better
	}

	/* Save this setting */
	lcd2dp->sink_cap.audio_channels = channels;
	lcd2dp->sink_cap.audio_rate = rate;
	lcd2dp->state.audio_attach = LCD2DP_A_ATTACHED;
}
#endif

static int lcd2dp_read_edid_bytes(struct mxc_lcd2dp *lcd2dp,
				  unsigned char offs, unsigned int count,
				  unsigned char *dest)
{
	unsigned int ix = 0;
	unsigned int aux_tries;
	unsigned int val;

	while (count--) {
		/* Read one byte from Sink external I2C memory */
		aux_tries = 7;
		for (;;) {
			lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x50);	// EDID EEPROM address
			lcd2dp_reg_write(lcd2dp, DP0_AUX_WDATA0, offs + ix); // Register address
			lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x004);	// Write 1 byte, middle of transaction
			lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x001);	// Read 1 byte, end of transaction
			msleep(2);
			lcd2dp_reg_read(lcd2dp, DP0_AUX_STAT, &val);
			if ((val & 0xffff) == 0x0100)
				break;
			dev_dbg(&lcd2dp->client_i2c->dev,
				"Read edid: AUX status %x\n", val);
			msleep(2);
			if (--aux_tries == 0) {
				dev_err(&lcd2dp->client_i2c->dev,
					"Read edid: AUX rd data fail at %u\n",
					offs + ix);
				return 1;
			}
		}
		lcd2dp_reg_read(lcd2dp, DP0_AUX_RDATA0, &val);
		dest[ix++] = (unsigned char)val;
	}

	return 0;
}

enum {
	EDID_OK,		// All OK
	EDID_READ_FAIL,		// Could not read EDID data
	EDID_NO_SUPPORT		// Read OK but required setting not supported
};

static int lcd2dp_read_edid(struct mxc_lcd2dp *lcd2dp)
{
	/* Reading of display E-EDID EEPROM according to:
	 * - VESA Enhanced Extended Display Identification Data Standard, rel A rev 2
	 * - CEA-861-D A DTV Profile for Uncompressed High Speed Digital Interfaces
	 */
	static const unsigned char edid_header[8] =
		{ 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };
	unsigned char edid[8];		// read buffer
	// unsigned int offs, end, next;
	int ret = EDID_READ_FAIL;

	dev_info(&lcd2dp->client_i2c->dev, "edid\n");
	lcd2dp->sink_cap.word = 0;

	/*** Base EDID Structure (offset 0x00-0x7f) ***/

	/* [0x00-0x07]: Fixed header */
	if (lcd2dp_read_edid_bytes(lcd2dp, 0x00, 8, edid))
		goto edid_fail;
	if (memcmp(edid_header, edid, 8)) {
		dev_err(&lcd2dp->client_i2c->dev, "- NOT EDID data!\n");
		goto edid_fail;
	}

	/* [0x12-0x13]: EDID Structure Version & Revision       */
	/* Revisions are backwards compatible, versions are not */
	if (lcd2dp_read_edid_bytes(lcd2dp, 0x12, 2, edid))
		goto edid_fail;
	if ((edid[0] != 1) || (edid[1] < 3)) {
		dev_err(&lcd2dp->client_i2c->dev, "- Unknown EDID version %u.%u!\n",
			(unsigned int)edid[0], (unsigned int)edid[1]);
		goto edid_fail;
	}

	/* [0x23]: Established Timings I (bitmask) */
	if (lcd2dp_read_edid_bytes(lcd2dp, 0x23, 1, edid))
		goto edid_fail;
	if ((edid[0] & 0x20) == 0) {
		dev_err(&lcd2dp->client_i2c->dev, "- 640x480@60Hz NOT supported!\n");
		ret = EDID_NO_SUPPORT;
		goto edid_fail;
	}
	dev_info(&lcd2dp->client_i2c->dev, "- 640x480@60Hz supported\n");
	lcd2dp->sink_cap.v640x480x60 = 1;
	lcd2dp->state.video_attach = LCD2DP_V_ATTACHED;
	ret = EDID_OK;

	/* [0x7e]: Extension Block Count */
	if (lcd2dp_read_edid_bytes(lcd2dp, 0x7e, 1, edid))
		goto edid_ok;
	if (edid[0] == 0) {
		dev_info(&lcd2dp->client_i2c->dev, "- No extension block\n");
		goto edid_ok;
	}

	/*** First Extension Block (offset 0x80-0xff) ***/

	/* [0x00]: Extension Block ID, 0x02 = EIA-CEA-861      */
	/* [0x01]: Structure revision (backwards compaible)    */
	/* [0x02]: Timing descriptor offset (from block start) */
	/* [0x03]: Misc flags, 0x40 = Support for basic audio  */
	if (lcd2dp_read_edid_bytes(lcd2dp, 0x80, 4, edid))
		goto edid_ok;
	if ((edid[0] != 0x02) || (edid[1] < 3)) {
		dev_info(&lcd2dp->client_i2c->dev, "- Unknown audio support\n");
		goto edid_ok;
	}
	/* EIA-CEA-861 rev 3 or later extension data is available */

	if ((edid[3] & 0x40) == 0) {
		dev_info(&lcd2dp->client_i2c->dev, "- Basic audio NOT supported\n");
		goto edid_ok;
	}
	/* From HDMI Interface Specification, version 1.0:
	 * Basic audio functionality consists of a single IEC 60958 audio
	 * stream at sample rates of 32kHz, 44.1kHz or 48kHz. This can
	 * accommodate any normal stereo stream.
	 */
	dev_info(&lcd2dp->client_i2c->dev, "- Basic audio supported\n");
	lcd2dp->sink_cap.audio_channels = 2;
	lcd2dp->sink_cap.audio_rate = 32000;
	lcd2dp->state.audio_attach = LCD2DP_A_ATTACHED;

#if 0  // Only basic audio used
	/*
	 * Audio data blocks are only required to list what's in addition
	 * to basic audio. (But NEC lists 2ch 32kHz 16b LPCM anyway...)
	 */

	/* Scan Data Block Collection */
	/* [0x04-(start of timing descriptors)]: CEA Data Block Collection */
	offs = 0x84;
	end = 0x80 + edid[2];
	for (;;) {
		if (offs >= end)
			goto edid_ok;	// No more data blocks
		if (lcd2dp_read_edid_bytes(lcd2dp, offs, 1, edid))
			goto edid_ok;
		next = offs + 1 + (edid[0] & 0x1f); // Low 5 bits = # of following bytes
		if ((edid[0] & 0xe0) == 0x20)       // High 3 bits = block type, 1 = Audio
			break;
		offs = next;
	}
	/* Audio Data Block, N * 3 byte descriptors */
	++offs;
	while (offs < next) {
		if (lcd2dp_read_edid_bytes(lcd2dp, offs, 3, edid))
			goto edid_ok;
		lcd2dp_parse_edid_short_audio_descr(lcd2dp,
					 edid[0], edid[1], edid[2]);
		offs += 3;
	}
#endif

  edid_ok:
	sysfs_notify(&lcd2dp->client_i2c->dev.kobj,
			"sink_caps", "audio_attached");
  edid_fail:
	return ret;
}

static int lcd2dp_setup(struct mxc_lcd2dp *lcd2dp)
{
	unsigned int val;
    int ret;
	u32 dp_phy_ctrl, reg, pllparam;

	dev_info(&lcd2dp->client_i2c->dev, "setup\n");

	// Setting part 1 according to Excel sheet
	lcd2dp_reg_read(lcd2dp, DP_IDREG, &val);
	if (val != 0x6603) {
		dev_err(&lcd2dp->client_i2c->dev, "wrong chip ID\n");
		return 1;
        }

	lcd2dp_reg_write(lcd2dp, DP0_CTRL, 0);

	// Setup PLL
	reg = DP0_SRCCTRL_NOTP | DP0_SRCCTRL_LANESKEW | DP0_SRCCTRL_EN810B | DP0_SRCCTRL_SCRMBLDIS;
	lcd2dp_reg_write(lcd2dp, DP0_SRC_CTRL, reg);
	lcd2dp_reg_write(lcd2dp, DP1_SRC_CTRL, 0x0);

	pllparam = LSCLK_DIV_2 | REF_FREQ_38M4;
	//lcd2dp_reg_write(lcd2dp, SYS_PLL_PARAM, 0x0001); // PLL multiplier = 1
	lcd2dp_reg_write(lcd2dp, SYS_PLL_PARAM, pllparam);
#if COLOR_BAR
	lcd2dp_reg_write(lcd2dp, PXL_PLL_PARAM, 0x220205);	// 38.4MHz * 5 / (2*2*2) = 24MHz
								// 38.4 * 45 / 2 / 16 = 54 [150..650]
#endif

	// Setup main link
	dp_phy_ctrl = BGREN | PWR_SW_EN | PHY_A0_EN | PHY_M0_EN;
	lcd2dp_reg_write(lcd2dp, DP_PHY_CTRL, dp_phy_ctrl);

	// Update PLL
	lcd2dp_reg_write(lcd2dp, DP0_PLL_CTRL, PLLUPDATE | PLLEN); // PLL bypass disabled , enable PLL
	/* Wait for PLL to lock: up to 2.09 ms, depending on refclk */
	usleep_range(3000, 6000);
	lcd2dp_reg_write(lcd2dp, DP1_PLL_CTRL, PLLUPDATE | PLLEN);		// PLL bypass disabled , enable PLL
	/* Wait for PLL to lock: up to 2.09 ms, depending on refclk */
	usleep_range(3000, 6000);

	ret = lcd2dp_poll_timeout(lcd2dp, DP_PHY_CTRL, PHY_RDY, PHY_RDY, 100, 100000);
	if (ret == -ETIMEDOUT) {
		dev_err(&lcd2dp->client_i2c->dev, "Timeout waiting for PHY to become ready");
		return 1;
	} else if (ret) {
		dev_err(&lcd2dp->client_i2c->dev, "AUX link setup failed: %d", ret);
		return 1;
	}

	// reset main channel
	dp_phy_ctrl |= DP_PHY_RST | PHY_M1_RST | PHY_M0_RST;
	lcd2dp_reg_write(lcd2dp, DP_PHY_CTRL, dp_phy_ctrl); //0x13001103);	// Reset
	usleep_range(100, 200);
	dp_phy_ctrl &= ~(DP_PHY_RST | PHY_M1_RST | PHY_M0_RST);
	lcd2dp_reg_write(lcd2dp, DP_PHY_CTRL, dp_phy_ctrl);	// !Reset
	ret = lcd2dp_poll_timeout(lcd2dp, DP_PHY_CTRL, PHY_RDY, PHY_RDY, 500, 100000);
	if (ret) {
		dev_err(&lcd2dp->client_i2c->dev, "timeout waiting for phy become ready");
		return 1;
	}

	//msleep(100);
	//lcd2dp_reg_read(lcd2dp, DP_PHY_CTRL, &val);
	//if (!(val & (1<<16))) {
	//	dev_err(&lcd2dp->client_i2c->dev, "main link not ready\n");
	//	return 1;
	//}

	// Read DP Rx Link Capability
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG1, 0x1063f);    // Setup AUX link
	lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x1);        // Read MAX_LINE_RATE
	/* Start transfer */
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x109);		// AUX Native Read (0x09) bsize=1 byte Address Only=0

	ret = lcd2dp_aux_wait_busy(lcd2dp);
	if (ret) {
		dev_err(&lcd2dp->client_i2c->dev, "AUX transfer busy\n");
		return 1;
    }

	lcd2dp_reg_read(lcd2dp, DP0_AUX_STAT, &val);
	if ((val & 0xffff) != 0x0200) {
		dev_err(&lcd2dp->client_i2c->dev, "Read cap: AUX status fail %X\n", val);
		return 1;
	}
	lcd2dp_reg_read(lcd2dp, DP0_AUX_RDATA0, &val);
	if ((val & 0xff) < 6) {
		dev_err(&lcd2dp->client_i2c->dev, "Sink does not support 1.6 Gbps %X\n", val);
		return 1;
	}
	val = (val >> 8) & 0x9f;				// Ignore TPS3 and POST_LT_ADJ_REQ support
	if ((val != 0x84) && (val != 0x82) && (val != 0x81)) {	// Enh frm and 1/2/4 lane support
		dev_err(&lcd2dp->client_i2c->dev, "Sink lane support %X\n", val);
		return 1;
	}

	// Setup Link & DPRx Config for training
	lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x100);
	lcd2dp_reg_write(lcd2dp, DP0_AUX_WDATA0, 0x8106);	// Enh frm, 1 lane, 1.62 Gbit
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x108);		// Write LINK_BW_SET
	lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x108);
	lcd2dp_reg_write(lcd2dp, DP0_AUX_WDATA0, 0x01);		// 8B10B
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x8);

	// Set DPCD for training part 1
	lcd2dp_reg_write(lcd2dp, DP0_SNK_LTCTRL, 0x21);
	lcd2dp_reg_write(lcd2dp, DP0_LTLOOP_CTRL, 0xf600000d);	// Link Training param

	// Set DP0 training pattern 1
	lcd2dp_reg_write(lcd2dp, DP0_SRC_CTRL, 0x3181);

	// Enable DP0 to start link training
	lcd2dp_reg_write(lcd2dp, DP0_CTRL, 0x21);		// Enhanced framing
	msleep(100);
	lcd2dp_reg_read(lcd2dp, DP0_LTSTAT, &val);
	if ((val & 0x3f77) != 0x2801) {				// ILAlign may be set
		dev_err(&lcd2dp->client_i2c->dev, "LT1 status %X\n", val);
		return 1;
	}

	// Set DPCD for training part 2
	lcd2dp_reg_write(lcd2dp, DP0_SNK_LTCTRL, 0x22);

	// Set DP0 training pattern 2
	lcd2dp_reg_write(lcd2dp, DP0_SRC_CTRL, 0x3281);		// Training pattern 2
	msleep(100);
	lcd2dp_reg_read(lcd2dp, DP0_LTSTAT, &val);
	if ((val & 0x3f7f) != 0x300F) {
		dev_err(&lcd2dp->client_i2c->dev, "LT2 status %X\n", val);
		return 1;
	}

	// Clear DPCD
	lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x102);
	lcd2dp_reg_write(lcd2dp, DP0_AUX_WDATA0, 0x00);		// Enable scrambler
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x8);		// Write TRAINING_PATTERN_SET

	// Clear DP0 training pattern
	lcd2dp_reg_write(lcd2dp, DP0_SRC_CTRL, 0x1081);		// Scrambler enabled

	// Read DPCD
	lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x200);
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x409);
	msleep(10);

	// Check training status
	lcd2dp_reg_read(lcd2dp, DP0_AUX_STAT, &val);
	if ((val & 0xffff) != 0x0500) {
		dev_err(&lcd2dp->client_i2c->dev, "Read DPCD: AUX status fail %X\n", val);
		return 1;
	}
	lcd2dp_reg_read(lcd2dp, DP0_AUX_RDATA0, &val);
	val &= 0xffff003f;  // Ignore IRQ's and if Content Protection capable
	if (val != 0x00070001) {
		dev_err(&lcd2dp->client_i2c->dev, "Link training %X\n", val);
		return 1;
	}
	lcd2dp_reg_read(lcd2dp, DP0_AUX_RDATA1, &val);
	val &= 0x7f;  // Ignore b7: Status change
	if (val != 0x01) {
		dev_err(&lcd2dp->client_i2c->dev, "Interlane aligned %X\n", val);
		return 1;
	}

	// DPCD ASSR Setting
	lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x10A);
	lcd2dp_reg_write(lcd2dp, DP0_AUX_WDATA0, 0x0);		// Disable ASSR
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x8);

	return 0;
}

static int lcd2dp_adjust(struct mxc_lcd2dp *lcd2dp)
{
	unsigned int stat, val0, val1, irq;

	dev_info(&lcd2dp->client_i2c->dev, "adjust\n");

	lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x200);
	lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x709);
	msleep(10);

	lcd2dp_reg_read(lcd2dp, DP0_AUX_STAT, &stat);
	if ((stat & 0xffff) != 0x0800) {
		dev_err(&lcd2dp->client_i2c->dev, "AUX status: 0x%08x\n", stat);
	} else {
		lcd2dp_reg_read(lcd2dp, DP0_AUX_RDATA0, &val0);
		lcd2dp_reg_read(lcd2dp, DP0_AUX_RDATA1, &val1);
		dev_info(&lcd2dp->client_i2c->dev,
			 "DPCD st: %02x 207..200: %08x %08x\n",
			 ((stat >> 16) & 0xff), val1, val0);

		/* Clear irq flags */
		irq = (val0 >> 8) & 0xff; 
		if (irq != 0) {
			lcd2dp_reg_write(lcd2dp, DP0_AUX_ADDR, 0x201);
			lcd2dp_reg_write(lcd2dp, DP0_AUX_WDATA0, irq);
			lcd2dp_reg_write(lcd2dp, DP0_AUX_CFG0, 0x8);
		}
		/* CMP adapter asserts SINK_SPECIFIC_IRQ, meaning?? */

		/* Check if lane0 is symbol locked */
		if (((val0 >> 16) & 0x7) != 0x7)
			return 1;  // Go back to setup...
	}

	return 0;
}

static int lcd2dp_avtx(struct mxc_lcd2dp *lcd2dp)
{
	unsigned int dp0ctrl;
	unsigned int sysctrl;
	unsigned int i2scfg;
	unsigned int sync_req;
	bool video_is_on;
	bool audio_is_on;
	enum { NO_UPDATE, ENABLE, DISABLE } update = NO_UPDATE;
	int video_changed = 0;

	sync_req = lcd2dp->sleep ? LCD2DP_S_OFF : lcd2dp->state.sync_req;
	dev_info(&lcd2dp->client_i2c->dev, "avtx (%u)\n", sync_req);

	if ((lcd2dp->state.audio_attach != LCD2DP_A_ATTACHED) &&
	    ((sync_req == LCD2DP_S_VIDEO_SWITCH) ||
	     (sync_req == LCD2DP_S_PLAYBACK))) {
		dev_err(&lcd2dp->client_i2c->dev,
			"Can't enable audio with no audio attached\n");
		return 0;
	}

	lcd2dp_reg_read(lcd2dp, DP0_CTRL, &dp0ctrl);
	lcd2dp_reg_read(lcd2dp, SYS_CTRL, &sysctrl);
	lcd2dp_reg_read(lcd2dp, I2SCFG, &i2scfg);

	/* Use actual state instead of state.sync to avoid loosing track */
	video_is_on = (dp0ctrl & 0x2) ? true : false;
	audio_is_on = (dp0ctrl & 0x4) ? true : false;

	switch (sync_req) {

	case LCD2DP_S_OFF:
		/* Disable video and audio */
		dp0ctrl &= ~0x6;		// Clear video & audio tx enable
		sysctrl &= ~0x0f0b;		// VideoSrc=None AudioSrc=None, OSCLK=n/a, OSCLK=None
		i2scfg &= ~0x1;			// Audio input disable
		update = DISABLE;
		if (video_is_on)
			video_changed = 1;
		break;

	case LCD2DP_S_VIDEO:
		if (!video_is_on && !audio_is_on) {
			/* Enable video */
			dp0ctrl |= 0x2;		// Set video tx enable
			sysctrl |= 0x2;		// VideoSrc=DPI
			update = ENABLE;
			video_changed = 1;
		} else if (video_is_on && audio_is_on) {
			/* Disable audio */
			i2scfg &= ~0x1;		// Audio input disable
			dp0ctrl &= ~0x4;	// Clear audio tx enable
			sysctrl &= ~0x0f08;	// AudioSrc=None, OSCLK=n/a, OSCLK=None
			update = DISABLE;
		}
		break;

	case LCD2DP_S_VIDEO_SWITCH:
		if (!video_is_on && !audio_is_on) {
			/* Enable video and audio I2S input */
			dp0ctrl |= 0x2;		// Set video tx enable
			sysctrl |= 0x2;		// VideoSrc=DPI
			i2scfg |= 0x1;		// Audio input enable
			update = ENABLE;
			video_changed = 1;
		}
		break;

	case LCD2DP_S_PLAYBACK:
		if (video_is_on && !audio_is_on) {
			/* Enable audio */
			i2scfg |= 0x1;		// Audio input enable
			dp0ctrl |= 0x4;		// Set audio tx enable
			sysctrl |= 0x0b08;	// AudioSrc=I2S, OSCLK=64xfSAMP, OSCLK=onSDpin
			update = ENABLE;
		} else if (!video_is_on && !audio_is_on) {
			/* Enable video and audio*/
			i2scfg |= 0x1;		// Audio input enable
			dp0ctrl |= 0x6;		// Set video & audio tx enable
			sysctrl |= 0x0b0a;	// VideoSrc=DPI AudioSrc=I2S, OSCLK=64xfSAMP, OSCLK=onSDpin
			update = ENABLE;
			video_changed = 1;
		}
		break;

	default:
		BUG();
		break;
	}

	if (update != NO_UPDATE) {
		if (update == ENABLE) {

			lcd2dp_reg_write(lcd2dp, I2SCFG, i2scfg);
			lcd2dp_reg_write(lcd2dp, SYS_CTRL, sysctrl);
			msleep(2);
			lcd2dp_reg_write(lcd2dp, DP0_CTRL, dp0ctrl);

		} else /* update == DISABLE */ {

			lcd2dp_reg_write(lcd2dp, DP0_CTRL, dp0ctrl);
			msleep(2);
			lcd2dp_reg_write(lcd2dp, SYS_CTRL, sysctrl);
			lcd2dp_reg_write(lcd2dp, I2SCFG, i2scfg);
		}

		lcd2dp->state.sync = sync_req;
		sysfs_notify(&lcd2dp->client_i2c->dev.kobj,
				"sink_caps", "sync");
	}

	return video_changed;
}

static void lcd2dp_disable(struct mxc_lcd2dp *lcd2dp)
{
	dev_info(&lcd2dp->client_i2c->dev, "disable\n");

	lcd2dp->state.sync_req = LCD2DP_S_OFF;
	(void)lcd2dp_avtx(lcd2dp);
	msleep(2);
	lcd2dp_reg_write(lcd2dp, SYS_CTRL, 0x0); // Disable all sources
	lcd2dp_reg_write(lcd2dp, DP0_CTRL, 0x0); // Disable rest of DP tx
}

static void set_fb_pixel_flow_enable(bool enable)
{
	int i;

	for (i = 0; i < num_registered_fb; i++) {
		char *idstr = registered_fb[i]->fix.id;
		if (strcmp(idstr, "DISP3 BG") == 0) {
			fb_blank(registered_fb[i],
			         enable ? FB_BLANK_UNBLANK : FB_BLANK_NORMAL);
			break;
		}
	}
}

static int lcd2dp_device_match(struct device *dev, const void *data)
{
	return 1;  /* Match with every instance found... */
}

static struct i2c_driver lcd2dp_i2c_drv;

void *lcd2dp_get_handle(const char *idstr)
{
	struct device * dev;

	dev = driver_find_device(&lcd2dp_i2c_drv.driver, NULL,
				    (void *)idstr, lcd2dp_device_match);
    if (dev) {
		struct i2c_client *client = to_i2c_client(dev);
		return i2c_get_clientdata(client);
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(lcd2dp_get_handle);

void lcd2dp_set_hotplug(void *handle, bool active)
{
	struct mxc_lcd2dp *lcd2dp = (struct mxc_lcd2dp *)handle;

	mutex_lock(&lcd2dp->lock);
	if (lcd2dp->state.init == LCD2DP_SUSPENDED) {
		mutex_unlock(&lcd2dp->lock);
		return;
	}

	if (active) {
		if (!lcd2dp->dp_req.enable) {
			lcd2dp->dp_req.enable = true;
			lcd2dp->setup_retry = DP_RETRIES;
			if (lcd2dp->thread)
				wake_up_process(lcd2dp->thread);
		}
	} else {
		if (!lcd2dp->dp_req.disable) {
			lcd2dp->dp_req.word = 0; // Clear all pending requests
			lcd2dp->dp_req.disable = true;
			if (lcd2dp->thread)
				wake_up_process(lcd2dp->thread);
		}
	}
	mutex_unlock(&lcd2dp->lock);
}
EXPORT_SYMBOL_GPL(lcd2dp_set_hotplug);

void lcd2dp_signal_hotplug_irq(void *handle)
{
	struct mxc_lcd2dp *lcd2dp = (struct mxc_lcd2dp *)handle;

	mutex_lock(&lcd2dp->lock);
	if (lcd2dp->state.init == LCD2DP_SUSPENDED) {
		mutex_unlock(&lcd2dp->lock);
		return;
	}

	lcd2dp->dp_req.hpd_irq = true;
	if (lcd2dp->thread)
		wake_up_process(lcd2dp->thread);
	mutex_unlock(&lcd2dp->lock);
}
EXPORT_SYMBOL_GPL(lcd2dp_signal_hotplug_irq);

static int lcd2dp_thread(void *data)
{
	struct mxc_lcd2dp *lcd2dp = (struct mxc_lcd2dp *)data;
	bool yield;
	unsigned delay = 0;
	int result;

	dev_info(&lcd2dp->client_i2c->dev, "thread started\n");

	mutex_lock(&lcd2dp->lock);
	set_fb_pixel_flow_enable(false);
	lcd2dp->i2c_shutdown = false;
	gpio_set_value_cansleep(lcd2dp->reset_gpio, 1);
	msleep(2);

	for (;;) {
		yield = false;
		if (lcd2dp->dp_req.word == 0 && !delay) {
			/* No pending requests so yield */
			yield = true;
			set_current_state(TASK_UNINTERRUPTIBLE);
		}

		mutex_unlock(&lcd2dp->lock);

		if (delay) {
			msleep(delay);
			delay = 0;
		} else if (yield) {
			schedule();
		}
		if (kthread_should_stop())
			break;

		mutex_lock(&lcd2dp->lock);

		switch (lcd2dp->state.init) {

		case LCD2DP_IDLE:
			if (lcd2dp->dp_req.disable) {
				lcd2dp->state.word = 0;
				lcd2dp->dp_req.disable = false;
				lcd2dp->sink_cap.word = 0;
				sysfs_notify(&lcd2dp->client_i2c->dev.kobj,
					     "sink_caps", "audio_attached");
				sysfs_notify(&lcd2dp->client_i2c->dev.kobj,
					     "sink_caps", "sync");
			} else if (lcd2dp->state.sync != LCD2DP_S_OFF) {
				lcd2dp->state.sync = LCD2DP_S_OFF;
				sysfs_notify(&lcd2dp->client_i2c->dev.kobj,
					     "sink_caps", "sync");
				delay = SYNC_NOTIFY_MDELAY;
			} else if (lcd2dp->dp_req.enable) {
				result = lcd2dp_setup(lcd2dp);
				lcd2dp->dp_req.hpd_irq = false;
				lcd2dp->dp_req.sync = false;
				lcd2dp->edid_retry = EDID_RETRIES;
				if (result)
					lcd2dp->state.init = LCD2DP_ERROR;
				else if (lcd2dp->state.edid == LCD2DP_E_OK)
					lcd2dp->state.init = LCD2DP_EDID_OK;
				else
					lcd2dp->state.init = LCD2DP_SETUP_OK;
			} else {
				/* Only enable/disable requests valid here */
				lcd2dp->dp_req.word = 0;
			}
			break;

		case LCD2DP_SETUP_OK:
			if (lcd2dp->dp_req.disable) {
				lcd2dp_disable(lcd2dp);
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else {
				result = lcd2dp_read_edid(lcd2dp);
				if (result == EDID_OK) {
					lcd2dp->state.edid = LCD2DP_E_OK;
					lcd2dp->state.init = LCD2DP_EDID_OK;
				} else if (result == EDID_READ_FAIL) {
					if (lcd2dp->edid_retry) {
						--lcd2dp->edid_retry;
						delay = EDID_RETRY_MDELAY;
					} else {
						/* Could not read EDID,
						 * try to enable DP anyway */
						lcd2dp->sink_cap.v640x480x60 = 1;
						lcd2dp->state.video_attach = LCD2DP_V_ATTACHED;
						lcd2dp->state.edid = LCD2DP_E_OK;
						lcd2dp->state.init = LCD2DP_EDID_OK;
					}
				} else /* result == EDID_NO_SUPPORT */ {
					lcd2dp->state.edid = LCD2DP_E_NO_SUPP;
					lcd2dp->state.init = LCD2DP_NO_SUPPORT;
				}
			}
			break;

		case LCD2DP_NO_SUPPORT:
			if (lcd2dp->dp_req.disable) {
				lcd2dp_disable(lcd2dp);
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else {
				/* Require detach to try again */
				lcd2dp->dp_req.word = 0;
			}
			break;

		case LCD2DP_EDID_OK:
			if (lcd2dp->dp_req.disable) {
				lcd2dp_disable(lcd2dp);
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else {
				result = lcd2dp_enable(lcd2dp);
				if (result) {
					lcd2dp_disable(lcd2dp);
					lcd2dp->state.init = LCD2DP_ERROR;
				} else if (lcd2dp->sleep) {
					lcd2dp->dp_req.enable = false;
					lcd2dp->state.init = LCD2DP_SLEEP;
				} else {
					set_fb_pixel_flow_enable(true);
					lcd2dp->dp_req.enable = false;
					lcd2dp->state.init = LCD2DP_ACTIVE;
				}
			}
			break;

		case LCD2DP_ACTIVE:
			if (lcd2dp->dp_req.disable) {
				set_fb_pixel_flow_enable(false);
				lcd2dp_disable(lcd2dp);
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else if (lcd2dp->dp_req.sync) {
				result = lcd2dp_avtx(lcd2dp);
				if (result) {
					set_fb_pixel_flow_enable(false);
					lcd2dp->state.init = LCD2DP_SLEEP;
				}
				lcd2dp->dp_req.sync = false;
			} else if (lcd2dp->dp_req.hpd_irq) {
				lcd2dp->dp_req.hpd_irq = false;
				result = lcd2dp_adjust(lcd2dp);
				if (result) {
					set_fb_pixel_flow_enable(false);
					lcd2dp_disable(lcd2dp);
					lcd2dp->dp_req.enable = true;
					lcd2dp->state.init = LCD2DP_ERROR;
				}
			} else {
				/* Don't keep driver awake */
				lcd2dp->dp_req.word = 0;
			}
			break;

		case LCD2DP_SLEEP:
			if (lcd2dp->dp_req.disable) {
				lcd2dp_disable(lcd2dp);
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else if (lcd2dp->dp_req.sync) {
				result = lcd2dp_avtx(lcd2dp);
				if (result) {
					set_fb_pixel_flow_enable(true);
					lcd2dp->state.init = LCD2DP_ACTIVE;
				}
				lcd2dp->dp_req.sync = false;
			} else if (!lcd2dp->dp_req.hpd_irq) {
				/* Don't keep driver awake */
				lcd2dp->dp_req.word = 0;
			}
			break;

		case LCD2DP_ERROR:
			if (lcd2dp->dp_req.disable) {
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else if (lcd2dp->dp_req.hpd_irq) {
				lcd2dp->dp_req.enable = true;
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else if (lcd2dp->setup_retry) {
				--lcd2dp->setup_retry;
				delay = DP_RETRY_MDELAY;
				lcd2dp->state.init = LCD2DP_IDLE;
			} else {
				/* Don't keep driver awake */
				lcd2dp->dp_req.word = 0;
			}
			break;

		case LCD2DP_SUSPENDED:
			/* Don't keep driver awake */
			lcd2dp->dp_req.word = 0;
			break;

		default:
			BUG();
			break;
		}
	}

	/* Shut down chip */
	lcd2dp_disable(lcd2dp);
	lcd2dp->i2c_shutdown = true;
	gpio_set_value_cansleep(lcd2dp->reset_gpio, 0);
	mutex_unlock(&lcd2dp->lock);	// Don't block other callers

	dev_info(&lcd2dp->client_i2c->dev, "thread ended\n");

	return 0;
}

static int lcd2dp_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	int ret = 0;
	struct device_node *node = client->dev.of_node;
	struct mxc_lcd2dp *lcd2dp;
	enum of_gpio_flags flags;

	lcd2dp = devm_kzalloc(&client->dev, sizeof(*lcd2dp), GFP_KERNEL);
	if (!lcd2dp)
		return -ENOMEM;

	lcd2dp->dev = &client->dev;
	lcd2dp->client_i2c = client;
	lcd2dp->i2c_shutdown = true;

	dev_set_drvdata(lcd2dp->dev, lcd2dp);

#if 0  /* shdn gpio output not used */
	lcd2dp->shdn_gpio = of_get_named_gpio_flags(node, "shdn_gpio", 0, &flags);
	if (lcd2dp->shdn_gpio < 0)
		lcd2dp->shdn_gpio = 0;
	else
	{
		err = gpio_request(lcd2dp->shdn_gpio, "DPshdn");
		if (err)
			dev_err(&client->dev, "Failed to request gpio DPshdn %d\n", err);
		err = gpio_direction_output(lcd2dp->shdn_gpio, 0);
		if (err)
			dev_err(&client->dev, "Failed to set gpio DPshdn %d\n", err);
	}
#endif

	lcd2dp->reset_gpio = of_get_named_gpio_flags(node, "reset_gpio", 0, &flags);
	if (lcd2dp->reset_gpio < 0)
		lcd2dp->reset_gpio = 0;
	else
	{
		err = gpio_request(lcd2dp->reset_gpio, "DPreset");
		if (err)
			dev_err(&client->dev, "Failed to request gpio DPreset %d\n", err);
		err = gpio_direction_output(lcd2dp->reset_gpio, 0);
		if (err)
			dev_err(&client->dev, "Failed to set gpio low DPreset %d\n", err);
		udelay(5);
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C)) {
		dev_err(&client->dev, "not present\n");
		return -ENODEV;
	}

#if 0  /* irq output not used */
	lcd2dp->irq = irq_of_parse_and_map(node, 0);
	if (lcd2dp->irq <= 0) {
		dev_err(&client->dev, "Can't get irq from DT\n");
		return -1;
	}
#endif

	lcd2dp->regmap = devm_regmap_init_i2c(client, &lcd2dp_regmap_config);
	if (IS_ERR(lcd2dp->regmap)) {
		err = PTR_ERR(lcd2dp->regmap);
		dev_err(&client->dev, "regmap init failed\n");
	}

	mutex_init(&lcd2dp->lock);

	i2c_set_clientdata(client, lcd2dp);

    ret = devm_snd_soc_register_component(&client->dev,
				&soc_codec_driver_tc358767,
				tc358767_dai_driver,
				ARRAY_SIZE(tc358767_dai_driver));

	if (ret)
		dev_err(&client->dev, "Cannot register codec\n");

	ret = sysfs_create_group(&client->dev.kobj,
				 &lcd2dp_sysfs_sink_caps_attr_grp);
	if (ret)
		dev_err(&client->dev, "Cannot create sysfs group\n");

	lcd2dp->thread = kthread_run(lcd2dp_thread, lcd2dp, "lcd2dp");
	if (IS_ERR(lcd2dp->thread)) {
		dev_err(&client->dev, "Cannot start thread (%ld)\n",
			 PTR_ERR(lcd2dp->thread));
		lcd2dp->thread = NULL;
	}

	return 0;
}

static int lcd2dp_i2c_remove(struct i2c_client *client)
{
//	struct mxc_lcd2dp *lcd2dp = i2c_get_clientdata(client);
	return 0;
}

static void lcd2dp_i2c_shutdown(struct i2c_client *client)
{
	struct mxc_lcd2dp *lcd2dp = i2c_get_clientdata(client);

	if (lcd2dp->thread) {
		kthread_stop(lcd2dp->thread);
		lcd2dp->thread = NULL;
	}
}

#ifdef CONFIG_PM_SLEEP
static int lcd2dp_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc_lcd2dp *lcd2dp =
		(struct mxc_lcd2dp *)i2c_get_clientdata(client);

	dev_info(dev, "suspend\n");

	mutex_lock(&lcd2dp->lock);
	lcd2dp_disable(lcd2dp);
	lcd2dp->state.word = 0;
	lcd2dp->state.init = LCD2DP_SUSPENDED;
	lcd2dp->dp_req.word = 0;
	lcd2dp->sink_cap.word = 0;
	lcd2dp->i2c_shutdown = true;
	gpio_set_value_cansleep(lcd2dp->reset_gpio, 0);
	mutex_unlock(&lcd2dp->lock);

	return 0;
}

static int lcd2dp_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc_lcd2dp *lcd2dp =
		(struct mxc_lcd2dp *)i2c_get_clientdata(client);

	dev_info(dev, "resume\n");

	mutex_lock(&lcd2dp->lock);
	gpio_set_value_cansleep(lcd2dp->reset_gpio, 1);
	msleep(10);
	lcd2dp->i2c_shutdown = false;
	lcd2dp->state.init = LCD2DP_IDLE;
	mutex_unlock(&lcd2dp->lock);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(lcd2dp_dev_pm_ops, lcd2dp_suspend, lcd2dp_resume);

static const struct i2c_device_id lcd2dp_id_table[] = {
		{"tc358767_lcd2dp", 0},
		{},
};

MODULE_DEVICE_TABLE(i2c, lcd2dp_id_table);

#if defined(CONFIG_OF)
static const struct of_device_id lcd2dp_of_match[] = {
	{ .compatible = "tc358767_lcd2dp" },
	{},
};
MODULE_DEVICE_TABLE(of, lcd2dp_of_match);
#endif /* CONFIG_OF */

static struct i2c_driver lcd2dp_i2c_drv = {
	.driver = {
		.name = "tc358767_lcd2dp",
		.of_match_table = of_match_ptr(lcd2dp_of_match),
		.owner = THIS_MODULE,
		.pm   = &lcd2dp_dev_pm_ops,
	},
	.probe = lcd2dp_i2c_probe,
	.remove = lcd2dp_i2c_remove,
	.shutdown = lcd2dp_i2c_shutdown,
	.id_table = lcd2dp_id_table,
};

static int __init lcd2dp_i2c_client_init(void)
{
	int err = 0;
	err = i2c_add_driver(&lcd2dp_i2c_drv);
	if (err)
		pr_err("tc358767_lcd2dp: Failed to add i2c client driver\n");
	return err;
}

static void __exit lcd2dp_i2c_client_exit(void)
{
	i2c_del_driver(&lcd2dp_i2c_drv);
}

subsys_initcall(lcd2dp_i2c_client_init);
module_exit(lcd2dp_i2c_client_exit);

MODULE_LICENSE("GPL");
