/*
 * sound/soc/tegra/tegra_wired_jack.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/switch.h>
#include <linux/notifier.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/wm8903.h>
#include <mach/audio.h>

#include "tegra_soc.h"
#include "../codecs/wm8903.h"
#include "tegra_wired_jack.h"

#define HEAD_DET_GPIO 0
#define DOCK_HEAD_DET_GPIO 0
#define TEGRA_GPIO_PR0	136
#define TEGRA_GPIO_PX6	190
struct snd_soc_codec* g_codec;
int g_mic_state;
static int mHeadsetState;
static int mDockHeadsetState;
static int mHeadsetSkip=0;
static int mDockHeadsetSkip=0;
static void acer_dock_headset_delay_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(set_dock_headset_wq, acer_dock_headset_delay_work);

static int wired_jack_detect(void)
{
	int i;
	int withMic;
	int withoutMic;
	int MICDET_EINT_14;
	int MICSHRT_EINT_15;
	int irqStatus;
	int irq_mask = 0x3fff;
	int all_mask = 0xffff;
	int CtrlReg = 0;

	snd_soc_write(g_codec, WM8903_INTERRUPT_STATUS_1_MASK, irq_mask);
	snd_soc_write(g_codec, WM8903_MIC_BIAS_CONTROL_0, WM8903_MICDET_ENA | WM8903_MICBIAS_ENA);

	for(i = 0; i <= 15; i++)
	{
		msleep(1);
		irqStatus = snd_soc_read(g_codec, WM8903_INTERRUPT_STATUS_1);
		MICDET_EINT_14 = (irqStatus >> 14) & 0x1;
		MICSHRT_EINT_15 = (irqStatus >> 15) & 0x1;

		if(MICDET_EINT_14 == MICSHRT_EINT_15)
			withoutMic++;
		else
			withMic++;

		if(i%2 == 0)
			snd_soc_write(g_codec, WM8903_INTERRUPT_POLARITY_1, irq_mask);
		else
			snd_soc_write(g_codec, WM8903_INTERRUPT_POLARITY_1, all_mask);
	}

	CtrlReg &= ~(WM8903_MICDET_ENA | WM8903_MICBIAS_ENA);
	snd_soc_write(g_codec, WM8903_MIC_BIAS_CONTROL_0, CtrlReg);

	if (withMic > withoutMic)
		return 1;
	else
		return 2;
}

static void select_mic_input(int state)
{
	int CtrlReg = 0;

	switch (state) {
		case 0:
		case 2:
		{
			CtrlReg = (0x0<<B06_IN_CM_ENA) |
				(0x0<<B04_IP_SEL_N) | (0x1<<B02_IP_SEL_P) | (0x0<<B00_MODE);
		}
		break;

		case 1:
		{
			CtrlReg = (0x0<<B06_IN_CM_ENA) |
				(0x1<<B04_IP_SEL_N) | (0x1<<B02_IP_SEL_P) | (0x0<<B00_MODE);
		}
		break;
	}
	snd_soc_write(g_codec, WM8903_ANALOGUE_LEFT_INPUT_1, CtrlReg);
	snd_soc_write(g_codec, WM8903_ANALOGUE_RIGHT_INPUT_1, CtrlReg);
}

int wired_jack_state(void)
{
	return g_mic_state;
}

/* jack */
static struct snd_soc_jack *tegra_wired_jack;

static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headset Jack",
		.mask = SND_JACK_HEADSET,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	{
		/* gpio pin depends on board traits */
		.name = "headphone-detect-gpio",
		.report = SND_JACK_HEADPHONE,
		.invert = 1,
		.debounce_time = 200,
	},
};

static struct snd_soc_jack_gpio dock_hs_jack_gpios[] = {
	{
		/* gpio pin depends on board traits */
		.name = "dock_headphone-detect-gpio",
		.report = SND_JACK_HEADPHONE,
		.invert = 0,
		.debounce_time = 200,
	},
};

static struct switch_dev wired_switch_dev = {
	.name = "h2w",
};


static struct switch_dev dock_wired_switch_dev = {
	.name = "dock_h2w",
};

void dock_state_change(int state) {
	schedule_delayed_work(&set_dock_headset_wq, 10);
}

void tegra_switch_set_state(int state)
{

	switch_set_state(&wired_switch_dev, state);
}

void tegra_switch_set_dock_state(int state)
{
	switch_set_state(&dock_wired_switch_dev, state);
}

static void acer_dock_headset_delay_work(struct work_struct *work) {
	int dock_state = gpio_get_value(TEGRA_GPIO_PR0);
	int dock_headset_state = gpio_get_value(TEGRA_GPIO_PX6);

	if (dock_state == 0 && mHeadsetState == 0) {
		g_mic_state = mHeadsetState = mDockHeadsetState = 0;
		tegra_switch_set_dock_state(0);
	} else if (dock_state == 0 && mDockHeadsetState ==1) {
		mDockHeadsetSkip = 1;
		mDockHeadsetState = 0;
	} else if (dock_state == 0) {
		mDockHeadsetState = mDockHeadsetSkip = 0;
	} else if (dock_state == 1 && dock_headset_state == 1) {
		g_mic_state = mDockHeadsetState = 1;
		if (mDockHeadsetSkip) {
			tegra_switch_set_dock_state(0);
			mDockHeadsetSkip = 0;
		}
		select_mic_input(1);
		tegra_switch_set_dock_state(1);
	}

}

static int wired_swith_notify(struct notifier_block *self,
			      unsigned long action, void* dev)
{
	int state = 0;

	mHeadsetState = action;

	if (mDockHeadsetState!=0) {
		mHeadsetSkip = 1;
		return NOTIFY_OK;
	}

	switch (action) {
	case SND_JACK_HEADSET:
	case SND_JACK_HEADPHONE:
		state = wired_jack_detect();
		break;
	default:
		state = 0;
	}

	g_mic_state = state;
	select_mic_input(state);

	if (mHeadsetSkip) {
		tegra_switch_set_state((state?0:1));
		mHeadsetSkip = 0;
	}

	tegra_switch_set_state(state);

	return NOTIFY_OK;
}

static int dock_wired_swith_notify(struct notifier_block *self,
			      unsigned long action, void* dev)
{
	int state = 0;
	int dock_state = gpio_get_value(TEGRA_GPIO_PR0);

	if (action==1 && dock_state==1) {
		mDockHeadsetState = 1;
		state=1;
	} else {
		mDockHeadsetState = 0;
	}

	if (mHeadsetState!=0) {
		mDockHeadsetSkip = 1;
		return NOTIFY_OK;
	}

	g_mic_state = state;
	select_mic_input(state);

	if (mDockHeadsetSkip) {
		tegra_switch_set_dock_state((state?0:1));
		mDockHeadsetSkip = 0;
	}

	tegra_switch_set_dock_state(state);

	return NOTIFY_OK;
}

static struct notifier_block wired_switch_nb = {
	.notifier_call = wired_swith_notify,
};


static struct notifier_block dock_wired_switch_nb = {
	.notifier_call = dock_wired_swith_notify,
};

/* platform driver */
static int tegra_wired_jack_probe(struct platform_device *pdev)
{
	int ret;
	int hp_det_n;
	int dock_hp_det_n;
	struct tegra_wired_jack_conf *pdata;

	pdata = (struct tegra_wired_jack_conf *)pdev->dev.platform_data;
	if (!pdata || !pdata->hp_det_n) {
		pr_err("Please set up gpio pins for jack.\n");
		return -EBUSY;
	}

	hp_det_n = pdata->hp_det_n;
	hs_jack_gpios[HEAD_DET_GPIO].gpio = hp_det_n;

	ret = snd_soc_jack_add_gpios(&tegra_wired_jack[0],
				     ARRAY_SIZE(hs_jack_gpios),
				     hs_jack_gpios);
	if (ret) {
		pr_err("Could NOT set up gpio pins for headset jack.\n");
		snd_soc_jack_free_gpios(&tegra_wired_jack[0],
					ARRAY_SIZE(hs_jack_gpios),
					hs_jack_gpios);
		return ret;
	}

	dock_hp_det_n = pdata->dock_hp_det_n;
	dock_hs_jack_gpios[DOCK_HEAD_DET_GPIO].gpio = dock_hp_det_n;

	ret = snd_soc_jack_add_gpios(&tegra_wired_jack[1],
				     ARRAY_SIZE(dock_hs_jack_gpios),
				     dock_hs_jack_gpios);
	if (ret) {
		pr_err("Could NOT set up gpio pins for dock headset jack.\n");
		snd_soc_jack_free_gpios(&tegra_wired_jack[1],
					ARRAY_SIZE(dock_hs_jack_gpios),
					dock_hs_jack_gpios);
		return ret;
	}

	return 0;
}

static int tegra_wired_jack_remove(struct platform_device *pdev)
{
	snd_soc_jack_free_gpios(tegra_wired_jack,
				ARRAY_SIZE(hs_jack_gpios),
				hs_jack_gpios);

	snd_soc_jack_free_gpios(tegra_wired_jack,
				ARRAY_SIZE(dock_hs_jack_gpios),
				dock_hs_jack_gpios);
	return 0;
}

static struct platform_driver tegra_wired_jack_driver = {
	.probe = tegra_wired_jack_probe,
	.remove = __devexit_p(tegra_wired_jack_remove),
	.driver = {
		.name = "tegra_wired_jack",
		.owner = THIS_MODULE,
	},
};


int tegra_jack_init(struct snd_soc_codec *codec)
{
	int ret;

	if (!codec)
		return -1;

	g_codec = codec;

	mHeadsetState = 0;
	mDockHeadsetState = 0;

	tegra_wired_jack = kzalloc((sizeof(*tegra_wired_jack)*2), GFP_KERNEL);
	if (!tegra_wired_jack) {
		pr_err("failed to allocate tegra_wired_jack \n");
		return -ENOMEM;
	}

	/* Add jack detection */
	ret = snd_soc_jack_new(codec->socdev->card, "Headset Jack",
			       SND_JACK_HEADSET, &tegra_wired_jack[0]);
	ret = snd_soc_jack_new(codec->socdev->card, "Dock Headset Jack",
			       SND_JACK_HEADSET, &tegra_wired_jack[1]);

	if (ret < 0)
		goto failed;

	ret = snd_soc_jack_add_pins(tegra_wired_jack,
				    ARRAY_SIZE(hs_jack_pins),
				    hs_jack_pins);
	if (ret < 0)
		goto failed;

	/* Addd h2w swith class support */
	ret = switch_dev_register(&wired_switch_dev);
	if (ret < 0)
		goto switch_dev_failed;

	/* Addd dock_h2w swith class support */
	ret = switch_dev_register(&dock_wired_switch_dev);
	if (ret < 0)
		goto switch_dev_dock_failed;

	snd_soc_jack_notifier_register(&tegra_wired_jack[0],
				       &wired_switch_nb);

	snd_soc_jack_notifier_register(&tegra_wired_jack[1],
				       &dock_wired_switch_nb);

	ret = platform_driver_register(&tegra_wired_jack_driver);
	if (ret < 0)
		goto platform_dev_failed;

	return 0;

switch_dev_failed:
	switch_dev_unregister(&wired_switch_dev);
switch_dev_dock_failed:
	switch_dev_unregister(&dock_wired_switch_dev);
platform_dev_failed:
	platform_driver_unregister(&tegra_wired_jack_driver);
failed:
	if (tegra_wired_jack) {
		kfree(tegra_wired_jack);
		tegra_wired_jack = 0;
	}
	return ret;
}

void tegra_jack_exit(void)
{
	switch_dev_unregister(&wired_switch_dev);
	switch_dev_unregister(&dock_wired_switch_dev);
	platform_driver_unregister(&tegra_wired_jack_driver);

	if (tegra_wired_jack) {
		kfree(tegra_wired_jack);
		tegra_wired_jack = 0;
	}
}
