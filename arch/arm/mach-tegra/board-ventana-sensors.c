/*
 * arch/arm/mach-tegra/board-ventana-sensors.c
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/akm8975.h>
#include <linux/mpu.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>

#include <mach/gpio.h>

#include <linux/delay.h>       //ddebug
#ifdef CONFIG_VIDEO_OV5650
#include <media/ov5650.h>
#endif
#include <media/ov2710.h>
#ifdef CONFIG_VIDEO_YUV
#include <media/yuv_sensor.h>
#endif
#ifdef CONFIG_VIDEO_YUV5
#include <media/yuv5_sensor.h>
#endif

#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-ventana.h"

#define ISL29018_IRQ_GPIO	TEGRA_GPIO_PZ2
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5
#define CAMERA_POWER_GPIO	TEGRA_GPIO_PV4
#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4
#define AC_PRESENT_GPIO		TEGRA_GPIO_PV3
#define NCT1008_THERM2_GPIO	TEGRA_GPIO_PN6
#ifdef CONFIG_VIDEO_OV5650
#define OV5650_PWR_DN_GPIO      TEGRA_GPIO_PL0
#define OV5650_RST_L_GPIO       TEGRA_GPIO_PL6
#endif
#ifdef CONFIG_VIDEO_YUV
#define YUV_SENSOR_OE_L_GPIO    TEGRA_GPIO_PL2
#define YUV_SENSOR_RST_GPIO     TEGRA_GPIO_PL4
#endif
#ifdef CONFIG_VIDEO_YUV5
#define YUV5_PWR_DN_GPIO        TEGRA_GPIO_PL0
#define YUV5_RST_L_GPIO         TEGRA_GPIO_PL6
#endif

//ddebug - start
struct camera_gpios {
	const char *name;
	int gpio;
	int enabled;
        int milliseconds;
};

#define CAMERA_GPIO(_name, _gpio, _enabled, _milliseconds)		        \
	{						                        \
		.name = _name,				                        \
		.gpio = _gpio,				                        \
		.enabled = _enabled,			                        \
		.milliseconds = _milliseconds,				        \
	}
//ddebug - end

extern void tegra_throttling_enable(bool enable);

//ddebug - start
#ifdef CONFIG_VIDEO_OV5650
static struct camera_gpios ov5650_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 1, 0),
	[1] = CAMERA_GPIO("cam_pwdn", OV5650_PWR_DN_GPIO, 0, 0),
	[2] = CAMERA_GPIO("cam_rst_lo", OV5650_RST_L_GPIO, 1, 0),
};
//ddebug - end

static int ventana_ov5650_power_on(void)
{
//ddebug - start
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		tegra_gpio_enable(ov5650_gpio_keys[i].gpio);
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
				ov5650_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}

	gpio_direction_output(OV5650_PWR_DN_GPIO, 1);
	gpio_direction_output(OV5650_RST_L_GPIO, 1);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(5);
	gpio_direction_output(OV5650_PWR_DN_GPIO, 0);
	msleep(20);
	gpio_direction_output(OV5650_RST_L_GPIO, 0);
	msleep(1);
	gpio_direction_output(OV5650_RST_L_GPIO, 1);
	msleep(20);

	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		gpio_export(ov5650_gpio_keys[i].gpio, false);
	}

	return 0;

fail:
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return ret;
//ddebug - end
}

static int ventana_ov5650_power_off(void)
{
//ddebug - start
        int i;

	gpio_direction_output(OV5650_PWR_DN_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(OV5650_PWR_DN_GPIO, 0);
	gpio_direction_output(OV5650_RST_L_GPIO, 0);

	i = ARRAY_SIZE(ov5650_gpio_keys);
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
//ddebug - end
	return 0;
}

struct ov5650_platform_data ventana_ov5650_data = {
	.power_on = ventana_ov5650_power_on,
	.power_off = ventana_ov5650_power_off,
};
#endif /*CONFIG_VIDEO_OV5650*/

static int ventana_ov2710_power_on(void)
{
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);
	return 0;
}

static int ventana_ov2710_power_off(void)
{
	return 0;
}

struct ov2710_platform_data ventana_ov2710_data = {
	.power_on = ventana_ov2710_power_on,
	.power_off = ventana_ov2710_power_off,
};
//ddebug - start
#ifdef CONFIG_VIDEO_YUV
static struct camera_gpios yuv_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 1, 0),
	[1] = CAMERA_GPIO("yuv_sensor_oe_l", YUV_SENSOR_OE_L_GPIO, 0, 0),
	[2] = CAMERA_GPIO("yuv_sensor_rst_lo", YUV_SENSOR_RST_GPIO, 1, 0),
};

static int yuv_sensor_power_on(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
				yuv_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}


	gpio_direction_output(YUV_SENSOR_OE_L_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 1);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(1);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);
	msleep(1);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 1);

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
	}

	return 0;

fail:
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv_sensor_power_off(void)
{
        int i;

	gpio_direction_output(YUV_SENSOR_OE_L_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_OE_L_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);

	i = ARRAY_SIZE(yuv_sensor_gpio_keys);
	while (i--)
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return 0;
}

struct yuv_sensor_platform_data yuv_sensor_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};
#endif /* CONFIG_VIDEO_YUV */
//ddebug - end

#ifdef CONFIG_VIDEO_YUV5
static struct camera_gpios yuv5_sensor_gpio_keys[] = {
	[0] = CAMERA_GPIO("cam_power_en", CAMERA_POWER_GPIO, 1, 0),
	[1] = CAMERA_GPIO("yuv5_sensor_pwdn", YUV5_PWR_DN_GPIO, 0, 0),
	[2] = CAMERA_GPIO("yuv5_sensor_rst_lo", YUV5_RST_L_GPIO, 1, 0),
};

static int yuv5_sensor_power_on(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv5_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv5_sensor_gpio_keys[i].gpio,
				yuv5_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
	}

	gpio_direction_output(YUV5_PWR_DN_GPIO, 0);
	gpio_direction_output(YUV5_RST_L_GPIO, 0);
	msleep(1);
	gpio_direction_output(CAMERA_POWER_GPIO, 1);
	msleep(1);
	gpio_direction_output(YUV5_RST_L_GPIO, 1);
	msleep(1);

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		gpio_export(yuv5_sensor_gpio_keys[i].gpio, false);
	}

	return 0;

fail:
	while (i--)
	gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return ret;
}

static int yuv5_sensor_power_off(void)
{
	int i;

	gpio_direction_output(YUV5_RST_L_GPIO, 0);
	msleep(1);
	gpio_direction_output(YUV5_PWR_DN_GPIO, 1);
	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV5_PWR_DN_GPIO, 0);

	i = ARRAY_SIZE(yuv5_sensor_gpio_keys);
	while (i--)
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return 0;
};

struct yuv5_sensor_platform_data yuv5_sensor_data = {
	.power_on = yuv5_sensor_power_on,
	.power_off = yuv5_sensor_power_off,
};
#endif

static int ventana_camera_init(void)
{
	int i, ret;

#ifdef CONFIG_VIDEO_OV5650
	// initialize OV5650
	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		tegra_gpio_enable(ov5650_gpio_keys[i].gpio);
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
				ov5650_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail1;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(OV5650_PWR_DN_GPIO, 0);
	gpio_direction_output(OV5650_RST_L_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		gpio_free(ov5650_gpio_keys[i].gpio);
		gpio_export(ov5650_gpio_keys[i].gpio, false);
        }
#endif

#ifdef CONFIG_VIDEO_YUV
	// initialize MT9D115
	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv_sensor_gpio_keys[i].gpio,
				yuv_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail2;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_OE_L_GPIO, 0);
	gpio_direction_output(YUV_SENSOR_RST_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(yuv_sensor_gpio_keys); i++) {
		gpio_free(yuv_sensor_gpio_keys[i].gpio);
		gpio_export(yuv_sensor_gpio_keys[i].gpio, false);
	}
#endif

#ifdef CONFIG_VIDEO_YUV5
	// initialize MT9P111
	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		tegra_gpio_enable(yuv5_sensor_gpio_keys[i].gpio);
		ret = gpio_request(yuv5_sensor_gpio_keys[i].gpio,
				yuv5_sensor_gpio_keys[i].name);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail3;
		}
	}

	gpio_direction_output(CAMERA_POWER_GPIO, 0);
	gpio_direction_output(YUV5_PWR_DN_GPIO, 0);
	gpio_direction_output(YUV5_RST_L_GPIO, 0);

	for (i = 0; i < ARRAY_SIZE(yuv5_sensor_gpio_keys); i++) {
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
		gpio_export(yuv5_sensor_gpio_keys[i].gpio, false);
	}
#endif
	return 0;

#ifdef CONFIG_VIDEO_OV5650
fail1:
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return ret;
#endif

#ifdef CONFIG_VIDEO_YUV
fail2:
        while (i--)
                gpio_free(yuv_sensor_gpio_keys[i].gpio);
	return ret;
#endif

#ifdef CONFIG_VIDEO_YUV5
fail3:
	while (i--)
		gpio_free(yuv5_sensor_gpio_keys[i].gpio);
	return ret;
#endif
}


static void ventana_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}

#ifdef CONFIG_SENSORS_AK8975
static void ventana_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}
#endif

//ddebug static void ventana_bq20z75_init(void)
//ddebug {
//ddebug 	tegra_gpio_enable(AC_PRESENT_GPIO);
//ddebug 	gpio_request(AC_PRESENT_GPIO, "ac_present");
//ddebug 	gpio_direction_input(AC_PRESENT_GPIO);
//ddebug }
//ddebug - start
static void ventana_ECBat_init(void)
{
	tegra_gpio_enable(AC_PRESENT_GPIO);
	gpio_request(AC_PRESENT_GPIO, "ac_present");
	gpio_direction_input(AC_PRESENT_GPIO);
}
//ddebug - end

static void ventana_nct1008_init(void)
{
	tegra_gpio_enable(NCT1008_THERM2_GPIO);
	gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	gpio_direction_input(NCT1008_THERM2_GPIO);
}

static struct nct1008_platform_data ventana_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 85,
	.shutdown_local_limit = 90,
	.throttling_ext_limit = 65,
	.alarm_fn = tegra_throttling_enable,
};

static const struct i2c_board_info ventana_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("al3000a_ls", 0x1C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
};

static const struct i2c_board_info ventana_i2c2_board_info[] = {
	{
//ddebug 		I2C_BOARD_INFO("bq20z75-battery", 0x0B),
//ddebug 		.irq = TEGRA_GPIO_TO_IRQ(AC_PRESENT_GPIO),
		I2C_BOARD_INFO("EC_Battery", 0x58),        //ddebug
		.irq = TEGRA_GPIO_TO_IRQ(AC_PRESENT_GPIO), //ddebug
	},
};

//ddebug static struct pca953x_platform_data ventana_tca6416_data = {
//ddebug 	.gpio_base      = TEGRA_NR_GPIOS + 4, /* 4 gpios are already requested by tps6586x */
//ddebug };

//ddebug static struct pca954x_platform_mode ventana_pca9546_modes[] = {
//ddebug 	{ .adap_id = 6, }, /* REAR CAM1 */
//ddebug 	{ .adap_id = 7, }, /* REAR CAM2 */
//ddebug 	{ .adap_id = 8, }, /* FRONT CAM3 */
//ddebug };

//ddebug static struct pca954x_platform_data ventana_pca9546_data = {
//ddebug 	.modes	  = ventana_pca9546_modes,
//ddebug 	.num_modes      = ARRAY_SIZE(ventana_pca9546_modes),
//ddebug };

//ddebug static const struct i2c_board_info ventana_i2c3_board_info_tca6416[] = {
//ddebug 	{
//ddebug 		I2C_BOARD_INFO("tca6416", 0x20),
//ddebug 		.platform_data = &ventana_tca6416_data,
//ddebug 	},
//ddebug };

//ddebug static const struct i2c_board_info ventana_i2c3_board_info_pca9546[] = {
//ddebug 	{
//ddebug 		I2C_BOARD_INFO("pca9546", 0x70),
//ddebug 		.platform_data = &ventana_pca9546_data,
//ddebug 	},
//ddebug };

static struct i2c_board_info ventana_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
		.platform_data = &ventana_nct1008_pdata,
	},

#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
#endif
};

//ddebug static struct i2c_board_info ventana_i2c7_board_info[] = {
static struct i2c_board_info ventana_i2c3_board_info[] = {  //ddebug
#ifdef CONFIG_VIDEO_OV5650
	{
		I2C_BOARD_INFO("ov5650", 0x36),
		.platform_data = &ventana_ov5650_data,
	},
#endif /*CONFIG_VIDEO_OV5650*/
//ddebug - start
#ifdef CONFIG_VIDEO_YUV
	{
		I2C_BOARD_INFO("mt9d115", 0x3C),
		.platform_data = &yuv_sensor_data,
 	},
#endif /* CONFIG_VIDEO_YUV */
#ifdef CONFIG_VIDEO_YUV5
	{
		I2C_BOARD_INFO("mt9p111", 0x3D),
		.platform_data = &yuv5_sensor_data,
	},
#endif /* CONFIG_VIDEO_YUV5 */
#ifdef CONFIG_VIDEO_AD5820
	{
		I2C_BOARD_INFO("ad5820", 0x0C),
	},
#endif /*CONFIG_VIDEO_AD5820*/
#ifdef CONFIG_VIDEO_LTC3216
	{
		I2C_BOARD_INFO("ltc3216", 0x33),
	},
#endif /*CONFIG_VIDEO_LTC3216*/
//ddebug - end

};

//ddebug static struct i2c_board_info ventana_i2c8_board_info[] = {
//ddebug 	{
//ddebug 		I2C_BOARD_INFO("ov2710", 0x36),
//ddebug 		.platform_data = &ventana_ov2710_data,
//ddebug 	},
//ddebug };

#ifdef CONFIG_MPU_SENSORS_MPU3050
#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
#ifdef CONFIG_MACH_ACER_PICASSO
	.orientation = {
		 0, -1,  0,
		-1,  0,  0,
		 0,  0, -1
	},
#endif
#ifdef CONFIG_MACH_ACER_VANGOGH
	.orientation = {
		 0, -1,  0,
		 1,  0,  0,
		 0,  0,  1
	},
#endif

	.level_shifter = 0,
	.accel = {
#ifdef CONFIG_MPU_SENSORS_KXTF9
		.get_slave_descr = get_accel_slave_descr,
#else
		.get_slave_descr = NULL,
#endif
		.adapt_num   = 0,
		.irq         = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7),
		.bus         = EXT_SLAVE_BUS_SECONDARY,
		.address     = 0x0F,
#ifdef CONFIG_MACH_ACER_PICASSO
		.orientation = {
			 0, -1,  0,
			-1,  0,  0,
			 0,  0, -1
		},
#endif
#ifdef CONFIG_MACH_ACER_VANGOGH
		.orientation = {
			 0,  1,  0,
			-1,  0,  0,
			 0,  0,  1
		},
#endif
        },

	.compass = {
#ifdef CONFIG_MPU_SENSORS_AK8975
		.get_slave_descr = get_compass_slave_descr,
#else
		.get_slave_descr = NULL,
#endif
		.adapt_num   = 4,            /* bus number 4 on ventana */
		.irq         = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
		.bus         = EXT_SLAVE_BUS_PRIMARY,
		.address     = 0x0C,
#ifdef CONFIG_MACH_ACER_PICASSO
		.orientation = {
			1,  0,  0,
			0, -1,  0,
			0,  0, -1
		},
#endif
#ifdef CONFIG_MACH_ACER_VANGOGH
		.orientation = {
			0, -1,  0,
		       -1,  0,  0,
			0,  0, -1
		},
#endif
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ4),
		.platform_data = &mpu3050_data,
	},
};

static void ventana_mpuirq_init(void)
{
	pr_info("*** MPU START *** ventana_mpuirq_init...\n");
	tegra_gpio_enable(TEGRA_GPIO_PZ4);
	gpio_request(TEGRA_GPIO_PZ4, SENSOR_MPU_NAME);
	gpio_direction_input(TEGRA_GPIO_PZ4);

	tegra_gpio_enable(TEGRA_GPIO_PS7);
	gpio_request(TEGRA_GPIO_PS7, "mpu_kxtf9");
	gpio_direction_input(TEGRA_GPIO_PS7);

	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "mpu_akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);

	pr_info("*** MPU END *** ventana_mpuirq_init...\n");
}
#endif

int __init ventana_sensors_init(void)
{
	struct board_info BoardInfo;

	ventana_isl29018_init();
#ifdef CONFIG_SENSORS_AK8975
	ventana_akm8975_init();
#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
	ventana_mpuirq_init();
#endif
	ventana_camera_init();
	ventana_nct1008_init();

	i2c_register_board_info(0, ventana_i2c0_board_info,
		ARRAY_SIZE(ventana_i2c0_board_info));

	tegra_get_board_info(&BoardInfo);

	/*
	 * battery driver is supported on FAB.D boards and above only,
	 * since they have the necessary hardware rework
	 */
//ddebug 	if (BoardInfo.sku > 0) {
//ddebug 		ventana_bq20z75_init();
	        ventana_ECBat_init();
		i2c_register_board_info(2, ventana_i2c2_board_info,
			ARRAY_SIZE(ventana_i2c2_board_info));
//ddebug 	}

//ddebug - start
	i2c_register_board_info(3, ventana_i2c3_board_info,
		ARRAY_SIZE(ventana_i2c3_board_info));
//ddebug -end

	i2c_register_board_info(4, ventana_i2c4_board_info,
		ARRAY_SIZE(ventana_i2c4_board_info));

//ddebug 	i2c_register_board_info(7, ventana_i2c7_board_info,
//ddebug 		ARRAY_SIZE(ventana_i2c7_board_info));

//ddebug 	i2c_register_board_info(8, ventana_i2c8_board_info,
//ddebug 		ARRAY_SIZE(ventana_i2c8_board_info));


#ifdef CONFIG_MPU_SENSORS_MPU3050
	i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
		ARRAY_SIZE(mpu3050_i2c0_boardinfo));
#endif

	return 0;
}

#ifdef CONFIG_VIDEO_OV5650
//ddebug - start
//ddebug struct camera_gpios {
//ddebug 	const char *name;
//ddebug 	int gpio;
//ddebug 	int enabled;
//ddebug };
//ddebug 
//ddebug #define CAMERA_GPIO(_name, _gpio, _enabled)		\
//ddebug 	{						\
//ddebug 		.name = _name,				\
//ddebug 		.gpio = _gpio,				\
//ddebug 		.enabled = _enabled,			\
//ddebug 	}
//ddebug 
//ddebug 
//ddebug static struct camera_gpios ov5650_gpio_keys[] = {
//ddebug 	[0] = OV5650_GPIO("en_avdd_csi", AVDD_DSI_CSI_ENB_GPIO, 1),
//ddebug 	[1] = OV5650_GPIO("cam2_pwdn", CAM2_PWR_DN_GPIO, 0),
//ddebug 	[2] = OV5650_GPIO("cam2_rst_lo", CAM2_RST_L_GPIO, 1),
//ddebug 	[3] = OV5650_GPIO("cam2_af_pwdn_lo", CAM2_AF_PWR_DN_L_GPIO, 0),
//ddebug 	[4] = OV5650_GPIO("cam2_ldo_shdn_lo", CAM2_LDO_SHUTDN_L_GPIO, 1),
//ddebug 	[5] = OV5650_GPIO("cam2_i2c_mux_rst_lo", CAM2_I2C_MUX_RST_GPIO, 1),
//ddebug };
//ddebug 
//ddebug int __init ventana_ov5650_late_init(void)
//ddebug {
//ddebug 	int ret;
//ddebug 	int i;
//ddebug 
//ddebug 	if (!machine_is_ventana())
//ddebug 		return 0;
//ddebug 
//ddebug 	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_tca6416);
//ddebug 
//ddebug 	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
//ddebug 		ret = gpio_request(ov5650_gpio_keys[i].gpio,
//ddebug 			ov5650_gpio_keys[i].name);
//ddebug 		if (ret < 0) {
//ddebug 			pr_err("%s: gpio_request failed for gpio #%d\n",
//ddebug 				__func__, i);
//ddebug 			goto fail;
//ddebug 		}
//ddebug 		gpio_direction_output(ov5650_gpio_keys[i].gpio,
//ddebug 			ov5650_gpio_keys[i].enabled);
//ddebug 		gpio_export(ov5650_gpio_keys[i].gpio, false);
//ddebug 	}
//ddebug 
//ddebug 	i2c_new_device(i2c_get_adapter(3), ventana_i2c3_board_info_pca9546);
//ddebug 
//ddebug 	return 0;
//ddebug 
//ddebug fail:
//ddebug 	while (i--)
//ddebug 		gpio_free(ov5650_gpio_keys[i].gpio);
//ddebug 	return ret;
//ddebug }
//ddebug 
//ddebug late_initcall(ventana_ov5650_late_init);

#endif /* CONFIG_VIDEO_OV5650 */
