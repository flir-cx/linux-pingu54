/*
 * Copyright 2012, 2014 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include "imx-ssi.h"
#include "imx-audmux.h"

#define CODEC_CLOCK 40000000
#define DAI_NAME_SIZE	32

struct imx_tc358767a_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	unsigned int clk_frequency;
};

static int imx_tc358767a_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_tc358767a_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct device *dev = rtd->card->dev;
	int ret;

	dev_dbg(dev, "set codec driver clock params\n");
	ret = snd_soc_dai_set_sysclk(asoc_rtd_to_codec(rtd, 0),
                                     0, 
                                     data->clk_frequency,
                                     SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}

	return 0;
}

static int imx_tc358767a_audmux_config(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int int_port, ext_port;
	int ptcr, pdcr;
	int ret;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	dev_dbg(&pdev->dev, "use int port %d external port %d\n",int_port,ext_port);

	ptcr = IMX_AUDMUX_V2_PTCR_SYN; // 4-wire
        pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port);

	ret = imx_audmux_v2_configure_port(int_port, ptcr, pdcr);
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}

	ptcr = 	IMX_AUDMUX_V2_PTCR_SYN | // 4-wire
		IMX_AUDMUX_V2_PTCR_TFSDIR |
		IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TFSEL(int_port) |
		IMX_AUDMUX_V2_PTCR_TCSEL(int_port);

        pdcr = IMX_AUDMUX_V2_PDCR_RXDSEL(int_port);

	ret = imx_audmux_v2_configure_port(ext_port, ptcr, pdcr); 
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

	return 0;
}

static int imx_tc358767a_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np;
	struct platform_device *cpu_pdev;
	struct i2c_client *codec_dev;
	struct imx_tc358767a_data *data = NULL;
	struct snd_soc_dai_link_component *comp;
	int ret;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!cpu_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	if (strstr(cpu_np->name, "ssi")) {
		// Configure port in I2S slave mode
		ret = imx_tc358767a_audmux_config(pdev);
		if (ret)
			goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		return -EPROBE_DEFER;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!comp) {
		ret = -ENOMEM;
		goto fail;
	}

	data->dai.cpus		= &comp[0];
	data->dai.codecs	= &comp[1];
	data->dai.platforms	= &comp[2];

	data->dai.num_cpus	= 1;
	data->dai.num_codecs	= 1;
	data->dai.num_platforms	= 1;

	data->clk_frequency = CODEC_CLOCK;
	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codecs->dai_name = "tc358767";
	data->dai.codecs->of_node = codec_np;
	data->dai.cpus->dai_name = dev_name(&cpu_pdev->dev);
	data->dai.cpus->of_node = cpu_np;
	data->dai.platforms->of_node = cpu_np;

	data->dai.init = &imx_tc358767a_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret) {
		dev_err(&pdev->dev, "failed to get model\n");
		goto fail;
        }

	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	of_node_put(cpu_np);
	of_node_put(codec_np);

	return 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_tc358767a_remove(struct platform_device *pdev)
{
//	struct snd_soc_card *card = platform_get_drvdata(pdev);
//	struct imx_tc358767a_data *data = snd_soc_card_get_drvdata(card);
	return 0;
}

static const struct of_device_id imx_tc358767a_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-tc358767a", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tc358767a_dt_ids);

static struct platform_driver imx_tc358767a_driver = {
	.driver = {
		.name = "imx-tc358767a",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_tc358767a_dt_ids,
	},
	.probe = imx_tc358767a_probe,
	.remove = imx_tc358767a_remove,
};
module_platform_driver(imx_tc358767a_driver);

MODULE_AUTHOR("Klas Malmborg <klas.malmborg@flir.se>");
MODULE_DESCRIPTION("Freescale i.MX TC358767A ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-tc358767a");
