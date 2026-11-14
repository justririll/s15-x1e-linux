// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2023, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soundwire/sdw.h>
#include <linux/thermal.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "common.h"
#include "qdsp6/audioreach.h"
#include "qdsp6/q6afe.h"
#include "qdsp6/q6dsp-common.h"
#include "sdw.h"

struct x1e80100_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack dp_jack[8];
	bool jack_setup;
};

/* Set when the topology provides DSP speaker protection for the WSA speakers */
static bool speaker_protection;
module_param(speaker_protection, bool, 0444);
MODULE_PARM_DESC(speaker_protection, "Speakers are protected by the DSP: do not limit WSA volumes");

/* Saved speaker calibration, loaded into "Speaker Protection R0T0" at boot */
static long sp_r0t0[2 * MAX_SP_VI_SPEAKERS];
static int sp_r0t0_count;
module_param_array(sp_r0t0, long, &sp_r0t0_count, 0444);
MODULE_PARM_DESC(sp_r0t0,
		 "Speaker calibration: R0 (Q24 ohm),T0 (Q6 °C) per speaker, as read back from the \"Speaker Protection R0T0\" control");

/*
 * Register the thermal zones of the link's wsa884x VI DAIs, in DT order,
 * which is the VI channel order.
 */
static void x1e80100_register_sp_speakers(struct snd_soc_pcm_runtime *rtd)
{
	struct thermal_zone_device *tzs[MAX_SP_VI_SPEAKERS];
	struct snd_soc_dai *codec_dai;
	unsigned int num = 0;
	int i;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		struct snd_soc_component *component = codec_dai->component;
		struct thermal_zone_device *tz;
		struct sdw_slave *slave;
		char tz_name[THERMAL_NAME_LENGTH];

		if (strcmp(codec_dai->name, "VI"))
			continue;
		if (!component || !component->dev)
			continue;
		if (num >= ARRAY_SIZE(tzs))
			break;

		/* The name wsa884x registers its thermal zone under */
		slave = dev_to_sdw_dev(component->dev);
		scnprintf(tz_name, sizeof(tz_name), "wsa-%d-%u",
			  slave->bus->id, slave->id.unique_id);
		tz = thermal_zone_get_zone_by_name(tz_name);
		if (IS_ERR(tz)) {
			dev_warn(rtd->dev,
				 "no thermal zone '%s' for %s (%ld); SP T0 kcontrol will read 0 for this slot\n",
				 tz_name, dev_name(component->dev),
				 PTR_ERR(tz));
			tz = NULL;
		}
		tzs[num++] = tz;
	}

	/* Also drops the zones of a previous bind when none are found */
	audioreach_sp_register_speaker_map(tzs, num);
}

static void x1e80100_snd_exit(struct snd_soc_pcm_runtime *rtd)
{
	if (snd_soc_rtd_to_cpu(rtd, 0)->id == WSA_CODEC_DMA_RX_0)
		audioreach_sp_register_speaker_map(NULL, 0);
}

static int x1e80100_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_jack *dp_jack = NULL;
	int dp_pcm_id = 0;

	switch (cpu_dai->id) {
	case WSA_CODEC_DMA_RX_0:
		x1e80100_register_sp_speakers(rtd);
		fallthrough;
	case WSA_CODEC_DMA_RX_1:
		if (speaker_protection)
			break;
		/*
		 * Set limit of -3 dB on Digital Volume and 0 dB on PA Volume
		 * to reduce the risk of speaker damage until we have active
		 * speaker protection in place.
		 */
		snd_soc_limit_volume(card, "WSA WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA2 WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "SpkrLeft PA Volume", 6);
		snd_soc_limit_volume(card, "SpkrRight PA Volume", 6);
		snd_soc_limit_volume(card, "WooferLeft PA Volume", 6);
		snd_soc_limit_volume(card, "TweeterLeft PA Volume", 6);
		snd_soc_limit_volume(card, "WooferRight PA Volume", 6);
		snd_soc_limit_volume(card, "TweeterRight PA Volume", 6);
		break;
	case DISPLAY_PORT_RX_0:
		dp_pcm_id = 0;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	case DISPLAY_PORT_RX_1 ... DISPLAY_PORT_RX_7:
		dp_pcm_id = cpu_dai->id - DISPLAY_PORT_RX_1 + 1;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	default:
		break;
	}

	if (dp_jack)
		return qcom_snd_dp_jack_setup(rtd, dp_jack, dp_pcm_id);

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

/* VI codec DAIs on the WSA playback link run at the V/I rate (sp_vi_rate), S32_LE */
static int x1e80100_wsa_vi_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params,
					int stream)
{
	struct snd_interval *rate =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_mask *fmt =
		hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	if (stream != SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	rate->min = rate->max = audioreach_sp_vi_rate();
	snd_mask_none(fmt);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

static int x1e80100_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
						      SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
							  SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	switch (cpu_dai->id) {
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
		channels->min = 1;
		break;
	default:
		break;
	}

	return 0;
}

static int x1e80100_snd_hw_map_channels(unsigned int *ch_map, int num)
{
	switch (num) {
	case 1:
		ch_map[0] = PCM_CHANNEL_FC;
		break;
	case 2:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_FR;
		break;
	case 3:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_FR;
		ch_map[2] = PCM_CHANNEL_FC;
		break;
	case 4:
		ch_map[0] = PCM_CHANNEL_FL;
		ch_map[1] = PCM_CHANNEL_LB;
		ch_map[2] = PCM_CHANNEL_FR;
		ch_map[3] = PCM_CHANNEL_RB;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int x1e80100_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int channels = substream->runtime->channels;
	unsigned int rx_slot[4];
	int ret;

	switch (cpu_dai->id) {
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
		ret = x1e80100_snd_hw_map_channels(rx_slot, channels);
		if (ret)
			return ret;

		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						  channels, rx_slot);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int x1e80100_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct x1e80100_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops x1e80100_be_ops = {
	.startup = qcom_snd_sdw_startup,
	.shutdown = qcom_snd_sdw_shutdown,
	.hw_free = x1e80100_snd_hw_free,
	.prepare = x1e80100_snd_prepare,
};

static void x1e80100_set_ctl(struct snd_soc_card *card, const char *name,
			     const long *vals, unsigned int count)
{
	struct snd_kcontrol *kctl = snd_soc_card_get_kcontrol(card, name);
	struct snd_ctl_elem_value *ev;
	unsigned int i;
	int ret;

	if (!kctl) {
		dev_warn(card->dev, "no control \"%s\"\n", name);
		return;
	}

	ev = kzalloc(sizeof(*ev), GFP_KERNEL);
	if (!ev)
		return;

	ev->id = kctl->id;
	for (i = 0; i < count && i < ARRAY_SIZE(ev->value.integer.value); i++)
		ev->value.integer.value[i] = vals[i];

	ret = kctl->put(kctl, ev);
	if (ret < 0)
		dev_warn(card->dev, "failed to set \"%s\": %d\n", name, ret);
	kfree(ev);
}

/*
 * Speaker protection needs the WSA V/I feedback path, which UCM does not
 * enable, plus the speakers' saved calibration. Set both up before the card
 * is registered, so no userspace helper has to.
 */
static void x1e80100_fixup_controls(struct snd_soc_card *card)
{
	static const char * const vi_paths[] = {
		"WSA WSA_AIF_VI Mixer WSA_SPKR_VI_1",
		"WSA WSA_AIF_VI Mixer WSA_SPKR_VI_2",
		"MultiMedia5 Mixer WSA_CODEC_DMA_TX_0",	/* V/I tap */
	};
	static const long on = 1;
	unsigned int i;

	if (!speaker_protection)
		return;

	for (i = 0; i < ARRAY_SIZE(vi_paths); i++)
		if (snd_soc_card_get_kcontrol(card, vi_paths[i]))
			x1e80100_set_ctl(card, vi_paths[i], &on, 1);

	if (sp_r0t0_count) {
		x1e80100_set_ctl(card, "Speaker Protection R0T0", sp_r0t0, sp_r0t0_count);
		dev_info(card->dev, "speaker calibration (R0/T0) loaded\n");
	} else {
		dev_warn(card->dev, "no speaker calibration (sp_r0t0=), the DSP assumes hot coils\n");
	}
}

static void x1e80100_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = x1e80100_snd_init;
			link->exit = x1e80100_snd_exit;
			link->be_hw_params_fixup = x1e80100_be_hw_params_fixup;
			link->ops = &x1e80100_be_ops;
		}

		/* The WSA playback link also carries the VI codec DAIs */
		if (link->id == WSA_CODEC_DMA_RX_0) {
			link->multi_direction_codecs = 1;
			link->multi_direction_params_fixup =
				x1e80100_wsa_vi_params_fixup;
		}
	}
}

static int x1e80100_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct x1e80100_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->owner = THIS_MODULE;
	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	card->fixup_controls = x1e80100_fixup_controls;
	x1e80100_add_be_ops(card);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_x1e80100_dt_match[] = {
	{ .compatible = "qcom,x1e80100-sndcard", .data = "x1e80100" },
	{ .compatible = "qcom,glymur-sndcard", .data = "glymur" },
	{}
};
MODULE_DEVICE_TABLE(of, snd_x1e80100_dt_match);

static struct platform_driver snd_x1e80100_driver = {
	.probe  = x1e80100_platform_probe,
	.driver = {
		.name = "snd-x1e80100",
		.of_match_table = snd_x1e80100_dt_match,
	},
};
module_platform_driver(snd_x1e80100_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_AUTHOR("Krzysztof Kozlowski <krzysztof.kozlowski@linaro.org>");
MODULE_DESCRIPTION("Qualcomm X1E80100 ASoC Machine Driver");
MODULE_LICENSE("GPL");
