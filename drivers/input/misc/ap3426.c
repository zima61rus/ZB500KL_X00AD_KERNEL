/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "ap3426.h"

/* 0067273:zhiyu add for power-key virtual far event */
unsigned char g_ps_status=1;
struct ap3426_data *g_ap3426_data=NULL;
int pwr_key_far_evt (void);
/* 0067273:zhiyu add for power-key virtual far event */

static struct regulator_map power_config[] = {
	{.supply = "vdd", .min_uv = 2000000, .max_uv = 3300000, },
	{.supply = "vio", .min_uv = 1750000, .max_uv = 1950000, },
};

static struct pinctrl_config pin_config = {
	.name = { "default", "sleep" },
};

static int gain_table[] = { 32768, 8192, 2048, 512 };
/* within 2% percent of jitter will trigger interrupt */
static int sensitivity_table[] = { 3000, 400, 100, 1 };
static int pmt_table[] = { 5, 10, 14, 19 }; /* 5.0 9.6, 14.1 18.7 */

/* PS distance table */
static int ps_distance_table[] = { 887, 282, 111, 78, 53, 46, };

static int alsps_sensor=0;
static char * alsps_ic;

static unsigned int psData=0;

static struct sensors_classdev als_cdev = {
	.name = "ap3426-light",
	.vendor = "Dyna Image Corporation",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "655360",
	.resolution = "1.0",
	.sensor_power = "0.35",
	.min_delay = 100000,
	.max_delay = 1375,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 2,
	.enabled = 0,
	.delay_msec = 50,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};

static struct sensors_classdev ps_cdev = {
	.name = "ap3426-proximity",
	.vendor = "Dyna Image Corporation",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
#ifdef DI_AUTO_TUNE
	.max_range = "5",
	.resolution = "5.0",
#else
	.max_range = "6",
	.resolution = "1.0",
#endif
	.sensor_power = "0.35",
	.min_delay = 5000,
	.max_delay = 1280,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 3,
	.enabled = 0,
	.delay_msec = 50,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};

/* 0067273:zhiyu add for power-key virtual far event */
int pwr_key_far_evt (void)
{
    ktime_t timestamp;

    if (g_ap3426_data == NULL)
	return -EINVAL;
    if (!g_ap3426_data->ps_enabled)
	return -EINVAL;

    //printk("%s \n",__func__);
    timestamp = ktime_get_boottime();
    input_report_abs(g_ap3426_data->input_proximity, ABS_DISTANCE,
	    1);
    input_event(g_ap3426_data->input_proximity, EV_SYN, SYN_TIME_SEC,
	    ktime_to_timespec(timestamp).tv_sec);
    input_event(g_ap3426_data->input_proximity, EV_SYN, SYN_TIME_NSEC,
	    ktime_to_timespec(timestamp).tv_nsec);
    input_sync(g_ap3426_data->input_proximity);

    return 0;
}
/* 0067273:zhiyu add for power-key virtual far event */

#ifdef CONFIG_HY_DRV_ASSIST
static ssize_t ap3426_ic_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n",alsps_ic);
}
static ssize_t ap3426_vendor_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n","DYNA IMAGE");
}
static ssize_t ap3426_exist_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", alsps_sensor);
}
static ssize_t ap3426_ps_data_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", psData);
}
#endif

#ifdef DI_AUTO_TUNE
static int di_ps_tune_zero_func_fae(struct ap3426_data *ps_data);
static inline uint32_t ap3426_get_ps_reading(struct ap3426_data *di);
static int ap3426_get_object(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3426_OBJ_COMMAND);
   	
    val &= AP3426_OBJ_MASK;

//    return val >> AP3426_OBJ_SHIFT;
	return !(val >> AP3426_OBJ_SHIFT);
}
#endif
static int ap3426_get_ir_value(struct ap3426_data *di); //add by kevin 20150717

static int sensor_power_init(struct device *dev, struct regulator_map *map,
		int size)
{
	int rc;
	int i;

	for (i = 0; i < size; i++) {
		map[i].regulator = devm_regulator_get(dev, map[i].supply);
		if (IS_ERR(map[i].regulator)) {
			rc = PTR_ERR(map[i].regulator);
			dev_err(dev, "Regualtor get failed vdd rc=%d\n", rc);
			goto exit;
		}
		if (regulator_count_voltages(map[i].regulator) > 0) {
			rc = regulator_set_voltage(map[i].regulator,
					map[i].min_uv, map[i].max_uv);
			if (rc) {
				dev_err(dev, "Regulator set failed vdd rc=%d\n",
						rc);
				goto exit;
			}
		}
	}

	return 0;

exit:
	/* Regulator not set correctly */
	for (i = i - 1; i >= 0; i--) {
		if (regulator_count_voltages(map[i].regulator))
			regulator_set_voltage(map[i].regulator, 0,
					map[i].max_uv);
	}

	return rc;
}

static int sensor_power_deinit(struct device *dev, struct regulator_map *map,
		int size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (!IS_ERR_OR_NULL(map[i].regulator)) {
			if (regulator_count_voltages(map[i].regulator) > 0)
				regulator_set_voltage(map[i].regulator, 0,
						map[i].max_uv);
		}
	}

	return 0;
}

static int sensor_power_config(struct device *dev, struct regulator_map *map,
		int size, bool enable)
{
	int i;
	int rc = 0;

	if (enable) {
		for (i = 0; i < size; i++) {
			rc = regulator_enable(map[i].regulator);
			if (rc) {
				dev_err(dev, "enable %s failed.\n",
						map[i].supply);
				goto exit_enable;
			}
		}
	} else {
		for (i = 0; i < size; i++) {
			rc = regulator_disable(map[i].regulator);
			if (rc) {
				dev_err(dev, "disable %s failed.\n",
						map[i].supply);
				goto exit_disable;
			}
		}
	}

	return 0;

exit_enable:
	for (i = i - 1; i >= 0; i--)
		regulator_disable(map[i].regulator);

	return rc;

exit_disable:
	for (i = i - 1; i >= 0; i--)
		if (regulator_enable(map[i].regulator))
			dev_err(dev, "enable %s failed\n", map[i].supply);

	return rc;
}

static int sensor_pinctrl_init(struct device *dev,
		struct pinctrl_config *config)
{
	config->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(config->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl\n");
		return PTR_ERR(config->pinctrl);
	}

	config->state[0] =
		pinctrl_lookup_state(config->pinctrl, config->name[0]);
	if (IS_ERR_OR_NULL(config->state[0])) {
		dev_err(dev, "Failed to look up default state\n");
		return PTR_ERR(config->state[0]);
	}

	config->state[1] =
		pinctrl_lookup_state(config->pinctrl, config->name[1]);
	if (IS_ERR_OR_NULL(config->state[1])) {
		dev_err(dev, "Failed to look up default state\n");
		return PTR_ERR(config->state[1]);
	}

	return 0;
}

static int ap3426_parse_dt(struct device *dev, struct ap3426_data *di)
{
	struct device_node *dp = dev->of_node;
	int rc;
	u32 value;
	int i;

	rc = of_get_named_gpio_flags(dp, "di,irq-gpio", 0,
			&di->irq_flags);
	if (rc < 0) {
		dev_err(dev, "unable to read irq gpio\n");
		return rc;
	}

	di->irq_gpio = rc;

	rc = of_property_read_u32(dp, "di,als-cal", &value);
	if (rc) {
		dev_err(dev, "read di,als-cal failed\n");
		return rc;
	}
	di->als_cal = value;

	rc = of_property_read_u32(dp, "di,als-gain", &value);
	if (rc) {
		dev_err(dev, "read di,als-gain failed\n");
		return rc;
	}
	if (value >= ARRAY_SIZE(gain_table)) {
		dev_err(dev, "di,als-gain out of range\n");
		return -EINVAL;
	}
	di->als_gain = value;

	rc = of_property_read_u32(dp, "di,als-persist", &value);
	if (rc) {
		dev_err(dev, "read di,als-persist failed\n");
		return rc;
	}
	if (value > 0x3f) { /* The maximum value is 63 conversion time. */
		dev_err(dev, "di,als-persist out of range\n");
		return -EINVAL;
	}
	di->als_persist = value;

	rc = of_property_read_u32(dp, "di,ps-gain", &value);
	if (rc) {
		dev_err(dev, "read di,ps-gain failed\n");
		return rc;
	}
	if (value > 0x03) { /* The maximum value is 3, stands for 8x gain. */
		dev_err(dev, "proximity gain out of range\n");
		return -EINVAL;
	}
	di->ps_gain = value;

	rc = of_property_read_u32(dp, "di,ps-persist", &value);
	if (rc) {
		dev_err(dev, "read di,ps-persist failed\n");
		return rc;
	}
	if (value > 0x3f) { /* The maximum value is 63 conversion time. */
		dev_err(dev, "di,ps-persist out of range\n");
		return -EINVAL;
	}
	di->ps_persist = value;

	rc = of_property_read_u32(dp, "di,ps-led-driver", &value);
	if (rc) {
		dev_err(dev, "read di,ps-led-driver failed\n");
		return rc;
	}
	if (value > 0x03) { /* The maximum value is 3, stands for 100% duty. */
		dev_err(dev, "led driver out of range\n");
		return -EINVAL;
	}
	di->ps_led_driver = value;

	rc = of_property_read_u32(dp, "di,ps-mean-time", &value);
	if (rc) {
		dev_err(dev, "di,ps-mean-time incorrect\n");
		return rc;
	}
	if (value >= ARRAY_SIZE(pmt_table)) {
		dev_err(dev, "ps mean time out of range\n");
		return -EINVAL;
	}
	di->ps_mean_time = value;

	rc = of_property_read_u32(dp, "di,ps-integrated-time", &value);
	if (rc) {
		dev_err(dev, "read di,ps-intergrated-time failed\n");
		return rc;
	}
	if (value > 0x3f) { /* The maximum value is 63. */
		dev_err(dev, "ps integrated time out of range\n");
		return -EINVAL;
	}
	di->ps_integrated_time = value;

	rc = of_property_read_u32(dp, "di,wakeup-threshold", &value);
	if (rc) {
		dev_info(dev, "di,wakeup-threshold incorrect, drop to default\n");
		value = AP3426_WAKEUP_ANY_CHANGE;
	}
	if ((value >= ARRAY_SIZE(ps_distance_table)) &&
			(value != AP3426_WAKEUP_ANY_CHANGE)) {
		dev_err(dev, "wakeup threshold too big\n");
		return -EINVAL;
	}
	di->ps_wakeup_threshold = value;

	rc = of_property_read_u32_array(dp, "di,als-sensitivity",
			sensitivity_table, ARRAY_SIZE(sensitivity_table));
	if (rc)
		dev_info(dev, "read di,als-sensitivity failed. Drop to default\n");

	rc = of_property_read_u32_array(dp, "di,ps-distance-table",
			ps_distance_table, ARRAY_SIZE(ps_distance_table));
	if ((rc == -ENODATA) || (rc == -EOVERFLOW)) {
		dev_err(dev, "di,ps-distance-table is not correctly set\n");
		return rc;
	}

	for (i = 1; i < ARRAY_SIZE(ps_distance_table); i++) {
		if (ps_distance_table[i - 1] < ps_distance_table[i]) {
			dev_err(dev, "ps distance table should in descend order\n");
			return -EINVAL;
		}
	}

	if (ps_distance_table[0] > PS_DATA_MASK) {
		dev_err(dev, "distance table out of range\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(dp, "di,ps-thdl", &value);
	if (!rc)
		di->ps_thd_l = (u16)value;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	rc = of_property_read_u32(dp, "di,ps-thdh", &value);
	if (!rc)
		di->ps_thd_h= (u16)value;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}
	
	return 0;
}

static int ap3426_check_device(struct ap3426_data *di)
{
	#if 0
	unsigned int part_id;
	int rc;

	/* AP3426 don't have part id registers */
	rc = regmap_read(di->regmap, AP3426_REG_CONFIG, &part_id);
	if (rc) {
		dev_err(&di->i2c->dev, "read reg %d failed.(%d)\n",
				AP3426_REG_CONFIG, rc);
		return rc;
	}

	dev_dbg(&di->i2c->dev, "register 0x%d:0x%x\n", AP3426_REG_CONFIG,
			part_id);
	#endif
	int pid;
	pid = i2c_smbus_read_byte_data(di->i2c, AP3426_REG_PID);
	printk("%s pid=%d\n",__func__,pid);
	if (pid !=AP3426_REG_PID_VALUE) {
		alsps_ic="NONE";
		return -1;
	}
	alsps_ic="AP3426";
	return 0;
}

static int ap3426_init_input(struct ap3426_data *di)
{
	struct input_dev *input;
	int status;

	input = devm_input_allocate_device(&di->i2c->dev);
	if (!input) {
		dev_err(&di->i2c->dev, "allocate light input device failed\n");
		return PTR_ERR(input);
	}

	input->name = AP3426_LIGHT_INPUT_NAME;
	input->phys = "ap3426/input0";
	input->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_MISC, 0, 655360, 0, 0);

	status = input_register_device(input);
	if (status) {
		dev_err(&di->i2c->dev, "register light input device failed.\n");
		return status;
	}

	di->input_light = input;

	input = devm_input_allocate_device(&di->i2c->dev);
	if (!input) {
		dev_err(&di->i2c->dev, "allocate light input device failed\n");
		return PTR_ERR(input);
	}

	input->name = AP3426_PROXIMITY_INPUT_NAME;
	input->phys = "ap3426/input1";
	input->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_DISTANCE, 0, 1023, 0, 0);

	status = input_register_device(input);
	if (status) {
		dev_err(&di->i2c->dev, "register proxmity input device failed.\n");
		return status;
	}

	di->input_proximity = input;

	return 0;
}

static int ap3426_init_device(struct ap3426_data *di)
{
	int rc;

#ifdef ALS_POLLING_MODE
	u8 als_data0[4] = {0x00,0x00,0xff,0xff};

	//ps interrput mode als polling mode
	/*77288:fix ps int flag is cleared by auto tune,zhanghaibin,20151130*/
	rc = regmap_write(di->regmap, AP3426_REG_INT_CTL, 0x81);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_INT_CTL);
		return rc;
	}
#else
	/* Enable als/ps interrupt and clear interrupt by software */
	rc = regmap_write(di->regmap, AP3426_REG_INT_CTL, AP3426_INT_CONFIG);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_INT_CTL);
		return rc;
	}

#endif

	/* Set als gain */
	rc = regmap_write(di->regmap, AP3426_REG_ALS_GAIN, di->als_gain << 4);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_ALS_GAIN);
		return rc;
	}

	/* Set als persistense */
	rc = regmap_write(di->regmap, AP3426_REG_ALS_PERSIST, di->als_persist);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_ALS_PERSIST);
		return rc;
	}

	/* Set ps interrupt form */
	#ifdef DI_AUTO_TUNE
	rc = regmap_write(di->regmap, AP3426_REG_PS_INT_FORM, 1);
	#else
	rc = regmap_write(di->regmap, AP3426_REG_PS_INT_FORM, 0);
	#endif
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_INT_FORM);
		return rc;
	}

	/* Set ps gain */
	rc = regmap_write(di->regmap, AP3426_REG_PS_GAIN, di->ps_gain << 2);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_GAIN);
		return rc;
	}

	/* Set ps persist */
	rc = regmap_write(di->regmap, AP3426_REG_PS_PERSIST, di->ps_persist);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_PERSIST);
		return rc;
	}

	/* Set PS LED driver strength */
	rc = regmap_write(di->regmap, AP3426_REG_PS_LED_DRIVER,
			di->ps_led_driver);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_LED_DRIVER);
		return rc;
	}

	/* Set PS mean time */
	rc = regmap_write(di->regmap, AP3426_REG_PS_MEAN_TIME,
			di->ps_mean_time);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_MEAN_TIME);
		return rc;
	}

	/* Set PS integrated time */
	rc = regmap_write(di->regmap, AP3426_REG_PS_INT_TIME,
			di->ps_integrated_time);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_INT_TIME);
		return rc;
	}

	/* Set calibration parameter low byte */
	rc = regmap_write(di->regmap, AP3426_REG_PS_CAL_L,
			PS_LOW_BYTE(di->bias));
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_CAL_L);
		return rc;
	}

	/* Set calibration parameter high byte */
	rc = regmap_write(di->regmap, AP3426_REG_PS_CAL_H,
			PS_HIGH_BYTE(di->bias));
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_CAL_H);
		return rc;
	}

	dev_dbg(&di->i2c->dev, "ap3426 initialize sucessful\n");

#ifdef DI_AUTO_TUNE	
	di->psa = 0x0;
	di->psi = 0xFFFF;	
	//di_ps_tune_zero_init(ps_data);
	di->tune_zero_init_proc = false;
	//di->ps_high_thd_boot = di->ps_thd_h;
	//di->ps_low_thd_boot = di->ps_thd_l;	
#endif	

#ifdef ALS_POLLING_MODE
	
	rc = regmap_bulk_write(di->regmap, AP3426_REG_ALS_LOW_THRES_0,
			als_data0, 4);
	if (rc) 
	{
		dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
				AP3426_REG_ALS_LOW_THRES_0, rc);
		return rc;
	}

	dev_dbg(&di->i2c->dev, "als threshold: 0x%x 0x%x 0x%x 0x%x\n",
			als_data0[0], als_data0[1], als_data0[2],
			als_data0[3]);

#endif
	return 0;
}

static int ap3426_calc_conversion_time(struct ap3426_data *di, int als_enabled,
		int ps_enabled)
{
	int conversion_time = 0;

	/* ALS conversion time is 100ms */
	if (als_enabled)
		conversion_time = 100;

	if (ps_enabled)
		conversion_time += pmt_table[di->ps_mean_time] +
			di->ps_mean_time * di->ps_integrated_time / 16;

	return conversion_time;
}

/* update als gain and threshold */
static int ap3426_als_update_setting(struct ap3426_data *di,
		unsigned int raw_value)
{
	int i;
	int rc;
	unsigned int lux_pre;
	unsigned int config;
	unsigned int adc_threshold;
	unsigned int adc_base;
	int gain_index; /* new gain index */
	u8 als_data[4];

	lux_pre = (raw_value * gain_table[di->als_gain]) >> 16;

	for (i = ARRAY_SIZE(gain_table) - 1; i >= 0; i--) {
		if (lux_pre < gain_table[i] *  ALS_GAIN_SWITCH_RATIO / 100)
			break;
	}

	gain_index = i < 0 ? 0 : i;

	/*
	 * Disable als and enable it again to avoid incorrect value.
	 * Updating als gain during als measurement cycle will cause
	 * incorrect light sensor adc value. The logic here is to handle
	 * this scenario.
	 */
	if (di->als_gain != gain_index) {
		/* read the system config register */
		rc = regmap_read(di->regmap, AP3426_REG_CONFIG, &config);
		if (rc) {
			dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			return rc;
		}

		/* disable als_sensor */
		rc = regmap_write(di->regmap, AP3426_REG_CONFIG,
				config & (~0x01));
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			return rc;
		}

		/* set als gain */
		rc = regmap_write(di->regmap, AP3426_REG_ALS_GAIN, i << 4);
		if (rc) {
			dev_err(&di->i2c->dev, "write %d register failed\n",
					AP3426_REG_ALS_GAIN);
			return rc;
		}
	}

	adc_base = raw_value * gain_table[di->als_gain] / gain_table[i];
	adc_threshold = ((10 * sensitivity_table[i]) << 16) /
		(di->als_cal * gain_table[i]);
	if (adc_threshold < 1)
		adc_threshold = 1;

	dev_dbg(&di->i2c->dev, "adc_base:%d adc_threshold:%d\n", adc_base,
			adc_threshold);

	/* lower threshold */
	if (adc_base < adc_threshold) {
		als_data[0] = 0x0;
		als_data[1] = 0x0;
	} else {
		als_data[0] = ALS_LOW_BYTE(adc_base - adc_threshold);
		als_data[1] = ALS_HIGH_BYTE(adc_base - adc_threshold);
	}

	/* upper threshold */
	if (adc_base + adc_threshold > ALS_DATA_MASK) {
		if (di->als_gain != 0) { /* trigger interrupt anyway */
			als_data[2] = als_data[0];
			als_data[3] = als_data[1];
		} else {
			als_data[2] = ALS_LOW_BYTE(ALS_DATA_MASK);
			als_data[3] = ALS_HIGH_BYTE(ALS_DATA_MASK);
		}
	} else {
		als_data[2] = ALS_LOW_BYTE(adc_base + adc_threshold);
		als_data[3] = ALS_HIGH_BYTE(adc_base + adc_threshold);
	}

	rc = regmap_bulk_write(di->regmap, AP3426_REG_ALS_LOW_THRES_0,
			als_data, 4);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
				AP3426_REG_ALS_LOW_THRES_0, rc);
		return rc;
	}

	dev_dbg(&di->i2c->dev, "als threshold: 0x%x 0x%x 0x%x 0x%x\n",
			als_data[0], als_data[1], als_data[2],
			als_data[3]);

	/* Enable als again. */
	if (di->als_gain != gain_index) {
		rc = regmap_write(di->regmap, AP3426_REG_CONFIG, config | 0x01);
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			return rc;
		}

		di->als_gain = i;
	}

	return 0;
}


/* Read raw data, convert it to human readable values, report it and
 * reconfigure the sensor.
 */
static int ap3426_process_data(struct ap3426_data *di, int als_ps)
{
	unsigned int gain;
	ktime_t timestamp;
	int rc = 0;

	unsigned int tmp;
	u8 als_data[2];
	int lux;

	u8 ps_data[4];
	#ifndef DI_AUTO_TUNE
	int i;
	#endif
	int distance;

	int ir_value;
	
	timestamp = ktime_get_boottime();

	if (als_ps) { /* process als value */
		/* Read data */
		rc = regmap_bulk_read(di->regmap, AP3426_REG_ALS_DATA_LOW,
				als_data, 2);
		if (rc) {
			dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
					AP3426_REG_ALS_DATA_LOW, rc);
			goto exit;
		}

		gain = gain_table[di->als_gain];

		/* lower bit */
		lux = (als_data[0] * di->als_cal * gain / 10) >> 16;
		/* higher bit */
		lux += (als_data[1] * di->als_cal * gain / 10) >> 8;

		dev_dbg(&di->i2c->dev, "lux:%d als_data:0x%x-0x%x\n",
				lux, als_data[0], als_data[1]);

		tmp = als_data[0] | (als_data[1] << 8);
		printk("----before----als lux = %d,ir_value = %d\n",lux,ir_value);
		
		ir_value = ap3426_get_ir_value(di);
		
		if(ir_value < 255)
		{
			lux =lux *1756/1000;
		}
		else
		{
			lux = lux * 922/1000;
		}
		printk("----after----als lux = %d,ir_value = %d\n",lux,ir_value);
		if (lux != di->last_als && ((tmp != ALS_DATA_MASK) ||
					((tmp == ALS_DATA_MASK) &&
					 (di->als_gain == 0)))) {
			input_report_abs(di->input_light, ABS_MISC, lux);
			input_event(di->input_light, EV_SYN, SYN_TIME_SEC,
					ktime_to_timespec(timestamp).tv_sec);
			input_event(di->input_light, EV_SYN, SYN_TIME_NSEC,
					ktime_to_timespec(timestamp).tv_nsec);
			input_sync(di->input_light);
		}

		di->last_als = lux;

		dev_dbg(&di->i2c->dev, "previous als_gain:%d\n", di->als_gain);

		rc = ap3426_als_update_setting(di, tmp);
		if (rc) {
			dev_err(&di->i2c->dev, "update setting failed\n");
			goto exit;
		}
	} else { /* process ps value*/
		//when register 0x02 bit 0 setting 0,then need to read ps raw data to auto clear the interrput flag
		rc = regmap_bulk_read(di->regmap, AP3426_REG_PS_DATA_LOW,
				ps_data, 2);
		if (rc) {
			dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
					AP3426_REG_PS_DATA_LOW, rc);
			goto exit;
		}

		dev_dbg(&di->i2c->dev, "ps data: 0x%x 0x%x\n",
				ps_data[0], ps_data[1]);

		tmp = ps_data[0] | (ps_data[1] << 8);
	#ifndef DI_AUTO_TUNE
		for (i = 0; i < ARRAY_SIZE(ps_distance_table); i++) {
			if (tmp > ps_distance_table[i])
				break;
		}
		distance = i;
	#else
		distance = ap3426_get_object(di->i2c);
	#endif
              psData=tmp;
		dev_err(&di->i2c->dev, "reprt work ps_data:%d, (last_ps,distance)=(%d,%d)\n", tmp,di->last_ps,distance);
	
		/* Report ps data */
		if (distance != di->last_ps) {
		    	if (distance == 0) {
			    g_ps_status=0;
			}
			input_report_abs(di->input_proximity, ABS_DISTANCE,
					distance);
			input_event(di->input_proximity, EV_SYN, SYN_TIME_SEC,
					ktime_to_timespec(timestamp).tv_sec);
			input_event(di->input_proximity, EV_SYN, SYN_TIME_NSEC,
					ktime_to_timespec(timestamp).tv_nsec);
			input_sync(di->input_proximity);
		}

		di->last_ps = distance;
#ifndef DI_AUTO_TUNE
		/* lower threshold */
		if (distance < ARRAY_SIZE(ps_distance_table))
			tmp = ps_distance_table[distance];
		else
			tmp = 0;

		ps_data[0] = PS_LOW_BYTE(tmp);
		ps_data[1] = PS_HIGH_BYTE(tmp);

		/* upper threshold */
		if (distance > 0)
			tmp = ps_distance_table[distance - 1];
		else
			tmp = 0x3ff;

		ps_data[2] = PS_LOW_BYTE(tmp);
		ps_data[3] = PS_HIGH_BYTE(tmp);

		dev_dbg(&di->i2c->dev, "ps threshold: 0x%x 0x%x 0x%x 0x%x\n",
				ps_data[0], ps_data[1], ps_data[2], ps_data[3]);

		rc = regmap_bulk_write(di->regmap, AP3426_REG_PS_LOW_THRES_0,
				ps_data, 4);
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_PS_LOW_THRES_0, rc);
			goto exit;
		}
#endif
		dev_dbg(&di->i2c->dev, "ps report exit\n");
	}

exit:
	return rc;
}

static irqreturn_t ap3426_irq_handler(int irq, void *data)
{
	struct ap3426_data *di = data;
	bool rc;

	rc = queue_work(di->workqueue, &di->report_work);
	/* wake up event should hold a wake lock until reported */
	if (rc && (atomic_inc_return(&di->wake_count) == 1))
		pm_stay_awake(&di->i2c->dev);


	return IRQ_HANDLED;
}

static void ap3426_report_work(struct work_struct *work)
{
	struct ap3426_data *di = container_of(work, struct ap3426_data,
			report_work);
	int rc;
	unsigned int status;
	
	mutex_lock(&di->ops_lock);

	/* avoid fake interrupt */
	if (!di->power_enabled) {
		dev_dbg(&di->i2c->dev, "fake interrupt triggered\n");
		goto exit;
	}

	rc = regmap_read(di->regmap, AP3426_REG_INT_FLAG, &status);
	if (rc) {
		dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
				AP3426_REG_INT_FLAG, rc);
		status |= AP3426_PS_INT_MASK;
		goto exit;
	}

	dev_dbg(&di->i2c->dev, "interrupt issued status=0x%x.\n", status);

	/* als interrupt issueed */
	if ((status & AP3426_ALS_INT_MASK) && (di->als_enabled)) {
		rc = ap3426_process_data(di, 1);
		if (rc)
			goto exit;
		dev_dbg(&di->i2c->dev, "process als data done!\n");
	}

	if ((status & AP3426_PS_INT_MASK) && (di->ps_enabled)) {
		rc = ap3426_process_data(di, 0);
		if (rc)
			goto exit;
		dev_dbg(&di->i2c->dev, "process ps data done!\n");
		pm_wakeup_event(&di->input_proximity->dev, 200);
	}

exit:
	if (atomic_dec_and_test(&di->wake_count)) {
		pm_relax(&di->i2c->dev);
		dev_dbg(&di->i2c->dev, "wake lock released\n");
	}

	/*77288:fix ps int flag is cleared by auto tune,zhanghaibin,20151130*/
	/* clear interrupt */
	#ifdef ALS_POLLING_MODE 
	if (di->power_enabled) {
		if (regmap_write(di->regmap, AP3426_REG_INT_FLAG, 0x0))
			dev_err(&di->i2c->dev, "clear interrupt failed\n");
	}
	#endif
	mutex_unlock(&di->ops_lock);
}

#ifdef DI_AUTO_TUNE
static int32_t di_ap3426_set_ps_thd_l(struct ap3426_data *ps_data, uint16_t thd_l)
{
	return i2c_smbus_write_word_data(ps_data->i2c,AP3426_REG_PS_LOW_THRES_0,thd_l);
}
static int32_t di_ap3426_set_ps_thd_h(struct ap3426_data *ps_data, uint16_t thd_h)
{
	return i2c_smbus_write_word_data(ps_data->i2c,AP3426_REG_PS_HIGH_THRES_0,thd_h);
}
#endif


#if 1
static int ap3426_get_ir_value(struct ap3426_data *di)
{
    unsigned int val;
    int rc;
    u8 ir_data[2];


	/*rc = regmap_write(di->regmap, AP3426_REG_PS_LED_DRIVER, 0x01);//led driver 16.7%
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_LED_DRIVER);
		return rc;
	}
	rc = regmap_write(di->regmap, AP3426_REG_PS_MEAN_TIME, 0x03);//ps mean time = 3
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_MEAN_TIME);
		return rc;
	}
	rc = regmap_write(di->regmap, AP3426_REG_PS_INT_TIME, 0x8);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d register failed\n",
				AP3426_REG_PS_INT_TIME);
		return rc;
	}*/

	rc = regmap_bulk_read(di->regmap, AP3426_REG_IR_DATA_LOW,
				ir_data, 2);
	if (rc) {
		dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
				AP3426_REG_IR_DATA_LOW, rc);
	}
	val = ir_data[0] | ((ir_data[1] & 0x03) << 8);
	
	return val;
}
#endif

#ifdef DI_AUTO_CAL
//u8 Calibration_Flag = 0;

static int AP3xx6_set_pcrosstalk(struct ap3426_data *di, int val)
{
    int lsb, msb, err;

	msb = val >> 8;
	lsb = val & 0xFF;
	err = regmap_write(di->regmap, 0x28, lsb);
	    
	err =regmap_write(di->regmap, 0x29, msb);
          
	return err;
}

int AP3xx6_Calibration(struct ap3426_data *di)
{
       int err;
	int i = 0;
	u16 ps_data = 0;
        u16 data = 0;
	 u16 data_min =999;
	u8 save_offset[2];
	err= regmap_bulk_read(di->regmap, AP3426_REG_PS_CAL_L,
			save_offset, 2);
	if(err)  return -1;
	
	 AP3xx6_set_pcrosstalk(di,0);
	 msleep(100);
	//if(Calibration_Flag == 0)
	{
		for(i=0; i<4; i++)
		{

			msleep(30);
			data = ap3426_get_ps_reading(di);

			printk("AP3426 ps =%d \n",data);
			if((data) > DI_PS_CAL_THR)
			{
				//Calibration_Flag = 0;
				goto err_out;
			}
			else
			{
				if(data_min>data)
					data_min =data;
				ps_data += data;
			}
			
		}
		ps_data -=data_min;
		printk("AP3426 ps_data1 =%d \n",ps_data);
		ps_data = ps_data/3;
		printk("AP3426 ps_data2 =%d \n",ps_data);
		AP3xx6_set_pcrosstalk(di,ps_data);
		msleep(100);
	}
	return 1;
err_out:
	regmap_bulk_write(di->regmap, AP3426_REG_PS_CAL_L,
			save_offset, 2);
	printk("AP3xx6_read_ps fail\n");
	return -1;	
}
#endif 


 int ap3426_enable_ps(struct ap3426_data *di, int enable)
{
	unsigned int config;
	int rc = 0;

	rc = regmap_read(di->regmap, AP3426_REG_CONFIG, &config);
	if (rc) {
		dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
				AP3426_REG_CONFIG, rc);
		goto exit;
	}

#ifdef DI_AUTO_TUNE
	if (!(di->psi_set) && !enable)
	{
		hrtimer_cancel(&di->ps_tune0_timer);					
		cancel_work_sync(&di->di_ps_tune0_work);
	}
#endif		

	/* avoid operate sensor in different executing context */
	if (enable) {
		
#ifdef DI_AUTO_TUNE
		di->psi_set = 0;
		di->psa = 0;
		di->psi = 0xFFFF;
		//di->ps_thd_h = di->ps_high_thd_boot;
		//di->ps_thd_l = di->ps_low_thd_boot;
		printk("--------%s set threshold (%d,%d)\n",__func__,di->ps_thd_l,di->ps_thd_h);
		di_ap3426_set_ps_thd_l(di,di->ps_thd_l);
		di_ap3426_set_ps_thd_h(di,di->ps_thd_h);
	//	hrtimer_start(&di->ps_tune0_timer, di->ps_tune0_delay, HRTIMER_MODE_REL);					
#endif			
		msleep(5);
		/* Enable ps sensor */
		rc = regmap_write(di->regmap, AP3426_REG_CONFIG, config | 0x02);
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			goto exit;
		}
		
		/* wait the data ready */
		msleep(ap3426_calc_conversion_time(di, di->als_enabled, 1));

		/* Clear last value and report it even not change. */
		di->input_proximity->absinfo[ABS_DISTANCE].value = 5;
		di->last_ps = -1;
		rc = ap3426_process_data(di, 0);
		if (rc) {
			dev_err(&di->i2c->dev, "process ps data failed\n");
			goto exit;
		}

		/*77288:fix ps int flag is cleared by auto tune,zhanghaibin,20151130*/
		/* clear interrupt */
		#ifdef ALS_POLLING_MODE
		rc = regmap_write(di->regmap, AP3426_REG_INT_FLAG, 0x0);
		if (rc) {
			dev_err(&di->i2c->dev, "clear interrupt failed\n");
			goto exit;
		}
		#endif
		di->ps_enabled = true;
	} 
	else
	{
/*
		// disable the ps_sensor 
		rc = regmap_write(di->regmap, AP3426_REG_CONFIG,
				config & (~0x2));
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			goto exit;
		}
		di->ps_enabled = false;
*/
		if(config & 0x01)//als enable, don't disable the ps and need to set the threshold avoid ps was avoided
		{
			di_ap3426_set_ps_thd_l(di,0);
			di_ap3426_set_ps_thd_h(di,1023);
		}
		else //als disable,then just disable the ps
		{
			// disable the ps_sensor 
			rc = regmap_write(di->regmap, AP3426_REG_CONFIG,
					config & (~0x2));
			if (rc) 
			{
				dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			goto exit;
			}
		}

		di->ps_enabled = false;
	}

	#ifdef DI_AUTO_CAL
		if(enable ==1)
		{
	        AP3xx6_Calibration(di);
		}
	#endif
	
exit:
	return rc;
}

EXPORT_SYMBOL(ap3426_enable_ps);

static int ap3426_enable_als(struct ap3426_data *di, int enable)
{
	unsigned int config;
	int rc = 0;

	/* Read the system config register */
	rc = regmap_read(di->regmap, AP3426_REG_CONFIG, &config);
	if (rc) {
		dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
				AP3426_REG_CONFIG, rc);
		goto exit;
	}

	if (enable)
	{
/*
		// enable als_sensor 
		rc = regmap_write(di->regmap, AP3426_REG_CONFIG, config | 0x01);
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			goto exit;
		}
*/
		if(config &0x02) //ps enable
		{
			rc = regmap_write(di->regmap, AP3426_REG_CONFIG, config | 0x01);
			if (rc) {
				dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
						AP3426_REG_CONFIG, rc);
				goto exit;
			}
		}
		else  //ps disable, then need enable ps
		{
			di_ap3426_set_ps_thd_l(di,0);
			di_ap3426_set_ps_thd_h(di,1023);

			msleep(5);

			rc = regmap_write(di->regmap, AP3426_REG_CONFIG, config | 0x03);
			if (rc) 
			{
				dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
						AP3426_REG_CONFIG, rc);
				goto exit;
			}
			
		}
#ifndef ALS_POLLING_MODE
		/* wait data ready */
		msleep(ap3426_calc_conversion_time(di, 1, di->ps_enabled));

		/* Clear last value and report even not change. */
		di->last_als = -1;

		rc = ap3426_process_data(di, 1);
		if (rc) {
			dev_err(&di->i2c->dev, "process als data failed\n");
			goto exit;
		}

		/* clear interrupt */
		rc = regmap_write(di->regmap, AP3426_REG_INT_FLAG, 0x0);
		if (rc) {
			dev_err(&di->i2c->dev, "clear interrupt failed\n");
			goto exit;
		}
#endif

		di->als_enabled = 1;

	} else {
		/* disable the als_sensor */
		rc = regmap_write(di->regmap, AP3426_REG_CONFIG,
				config & (~0x1));
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
					AP3426_REG_CONFIG, rc);
			goto exit;
		}
		di->als_enabled = 0;
	}
exit:
	return rc;
}

#ifdef DI_AUTO_TUNE
static int di_ps_tune_zero_final(struct ap3426_data *ps_data)
{

	ps_data->tune_zero_init_proc = false;

	if(ps_data->data_count == -1)
	{
		printk(KERN_INFO "%s: exceed limit\n", __func__);
		hrtimer_cancel(&ps_data->ps_tune0_timer);	
		return 0;
	}
	
	ps_data->psa = ps_data->ps_stat_data[0];
	ps_data->psi = ps_data->ps_stat_data[2];	

	ps_data->ps_high_thd_boot = ps_data->ps_stat_data[1] + DI_HT_N_CT*3;
	ps_data->ps_low_thd_boot = ps_data->ps_stat_data[1] + DI_LT_N_CT*3;
	ps_data->ps_thd_h = ps_data->ps_high_thd_boot ;
	ps_data->ps_thd_l = ps_data->ps_low_thd_boot ;		

	printk(KERN_INFO "%s: ps_data->ps_stat_data[1]=%d\n", __func__, ps_data->ps_stat_data[1]);
	printk("--------%s set threshold (%d,%d)\n",__func__,ps_data->ps_thd_l,ps_data->ps_thd_h);	
	di_ap3426_set_ps_thd_l(ps_data,ps_data->ps_thd_l);
	di_ap3426_set_ps_thd_h(ps_data,ps_data->ps_thd_h);	
	hrtimer_cancel(&ps_data->ps_tune0_timer);					
	return 0;
}
static inline uint32_t ap3426_get_ps_reading(struct ap3426_data *di)
{
	int rc = 0;
	uint32_t tmp;
	u8 ps_data[4]={0};
	rc = regmap_bulk_read(di->regmap, AP3426_REG_PS_DATA_LOW,
			ps_data, 2);
	if (rc) {
		dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
				AP3426_REG_PS_DATA_LOW, rc);
	}

	dev_dbg(&di->i2c->dev, "ps data: 0x%x 0x%x\n",
			ps_data[0], ps_data[1]);

	tmp = ps_data[0] | (ps_data[1] << 8);

	dev_dbg(&di->i2c->dev, "reprt work ps_data:%d\n", tmp);
	return tmp;
}
static int32_t di_tune_zero_get_ps_data(struct ap3426_data *ps_data)
{
	uint32_t ps_adc;

	ps_adc = ap3426_get_ps_reading(ps_data);
	printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, ps_data->data_count, ps_adc);
	if(ps_adc < 0)		
		return ps_adc;		
	 
	ps_data->ps_stat_data[1]  +=  ps_adc;			
	if(ps_adc > ps_data->ps_stat_data[0])
		ps_data->ps_stat_data[0] = ps_adc;
	if(ps_adc < ps_data->ps_stat_data[2])
		ps_data->ps_stat_data[2] = ps_adc;						
	ps_data->data_count++;	
	
	if(ps_data->data_count == 5)
	{
		ps_data->ps_stat_data[1]  /= ps_data->data_count;			
		di_ps_tune_zero_final(ps_data);
	}		
	
	return 0;
}
#if 0
static int di_ps_tune_zero_init(struct ap3426_data *ps_data)
{
	int32_t ret = 0;
	uint8_t w_state_reg;	
	
	ps_data->psi_set = 0;	
	ps_data->tune_zero_init_proc = true;		
	ps_data->ps_stat_data[0] = 0;
	ps_data->ps_stat_data[2] = 9999;
	ps_data->ps_stat_data[1] = 0;
	ps_data->data_count = 0;
	
	ret = di_i2c_smbus_write_byte_data(ps_data->i2c, DI_INT_REG, 0);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}			
	
	w_state_reg = (DI_STATE_EN_PS_MASK | DI_STATE_EN_WAIT_MASK);			
	ret = di_i2c_smbus_write_byte_data(ps_data->i2c, DI_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);		
	return 0;	
}
#endif

static int di_ps_tune_zero_func_fae(struct ap3426_data *ps_data)
{
	int32_t word_data;
	int diff;

	if(!(ps_data->ps_enabled))
	{
		return 0;
	}	

	word_data=ap3426_get_ps_reading(ps_data);
	if(word_data <0)
	{
		printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
		return 0xFFFF;
	}

	if(word_data > ps_data->psa)
	{
		ps_data->psa = word_data;
		printk(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);
	}
	if(word_data < ps_data->psi)
	{
		ps_data->psi = word_data;	
		printk(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);	
	}	
	
	diff = ps_data->psa - ps_data->psi;
	if(diff > DI_MAX_MIN_DIFF)
	{
		printk("--------%s Found psa-psi>DI_MAX_MIN_DIFF !!!\n",__func__);
		ps_data->psi_set = ps_data->psi;
		if(ps_data->psi < 500)
		{
			ps_data->ps_thd_h = ps_data->psi + DI_HT_N_CT;
			ps_data->ps_thd_l = ps_data->psi + DI_LT_N_CT;
			printk("--------%s set threshold (%d,%d)\n",__func__,ps_data->ps_thd_l,ps_data->ps_thd_h);
			di_ap3426_set_ps_thd_l(ps_data,ps_data->ps_thd_l);
            di_ap3426_set_ps_thd_h(ps_data,ps_data->ps_thd_h);	
			hrtimer_cancel(&ps_data->ps_tune0_timer);
		}
		
	}
	
	return 0;
}

static void di_ps_tune0_work_func(struct work_struct *work)
{
	struct ap3426_data *ps_data = container_of(work, struct ap3426_data, di_ps_tune0_work);		
	if(ps_data->tune_zero_init_proc)
		di_tune_zero_get_ps_data(ps_data);
	else
		di_ps_tune_zero_func_fae(ps_data);
	return;
}	


static enum hrtimer_restart di_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct ap3426_data *ps_data = container_of(timer, struct ap3426_data, ps_tune0_timer);
	queue_work(ps_data->di_ps_tune0_wq, &ps_data->di_ps_tune0_work);	
	hrtimer_forward_now(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
#endif

#ifdef ALS_POLLING_MODE
static void ap3426_ls_work_handler(struct work_struct *work)
{
	struct ap3426_data *ls_data = container_of(work, struct ap3426_data, di_ls_work);	
	ktime_t timestamp;
	int rc = 0;
	unsigned int tmp;
	unsigned int gain;
	u8 als_data[2];
	int lux;
	int ir_value;

	timestamp = ktime_get_boottime();
	

	rc = regmap_bulk_read(ls_data->regmap, AP3426_REG_ALS_DATA_LOW,
				als_data, 2);
	if (rc) 
	{
		dev_err(&ls_data->i2c->dev, "read %d failed.(%d)\n",
				AP3426_REG_ALS_DATA_LOW, rc);
		goto exit;
	}
#if 1
	gain = gain_table[ls_data->als_gain];

	/* lower bit */
	lux = (als_data[0] * ls_data->als_cal * gain / 10) >> 16;
	/* higher bit */
	lux += (als_data[1] * ls_data->als_cal * gain / 10) >> 8;
	dev_dbg(&ls_data->i2c->dev, "lux:%d als_data:0x%x-0x%x\n",
			lux, als_data[0], als_data[1]);

#endif
	tmp= als_data[0] | (als_data[1] << 8); 
	ir_value = ap3426_get_ir_value(ls_data);
	if(ir_value < 150)//cwf 
	{
		/*0064501:fix cwf test fail,zhanghaibin,20150831*/
		lux = lux *1;//100/74;
/*
		if(lux < 41)
		{
			//lux  = lux *5814/10000;
			lux = lux *5000/10000;
		}
		else
		{
			lux = lux *4400/10000;
		}
		*/
	}
	else//A and D65
	{
		if(lux <= 61) //lux <= 15
		{
			//lux = lux *1138/10000;
			//lux = lux *569/10000;
			lux = lux*342*6/10000;
		}
		else //A lux >15
		{
			lux = lux *6650/10000;
			//lux = lux *2276/10000;
			//lux = lux *2257/10000;
		}
	}
	
	if(lux != ls_data->last_als)
	{
		input_report_abs(ls_data->input_light, ABS_MISC, lux);
		input_event(ls_data->input_light, EV_SYN, SYN_TIME_SEC,
				ktime_to_timespec(timestamp).tv_sec);
		input_event(ls_data->input_light, EV_SYN, SYN_TIME_NSEC,
				ktime_to_timespec(timestamp).tv_nsec);
		input_sync(ls_data->input_light);
		ls_data->last_als = lux;
	}

	rc = ap3426_als_update_setting(ls_data, tmp);
	if (rc) {
			dev_err(&ls_data->i2c->dev, "update setting failed\n");
			goto exit;
	}
	

exit:
	return;

	
}	


static enum hrtimer_restart di_ls_timer_func(struct hrtimer *timer)
{
	struct ap3426_data *ls_data = container_of(timer, struct ap3426_data, ls_timer);
	queue_work(ls_data->di_ls_wq, &ls_data->di_ls_work);	
	hrtimer_forward_now(&ls_data->ls_timer, ls_data->ls_delay);
	return HRTIMER_RESTART;	
}

#endif

/* Sync delay to hardware according configurations
 * Note one of the sensor may report faster than expected.
 */
static int ap3426_sync_delay(struct ap3426_data *di, int als_enabled,
		int ps_enabled, unsigned int als_delay, unsigned int ps_delay)
{
	unsigned int convert_msec;
	unsigned int delay;
	int rc;

	/* ignore delay synchonization while power not enabled */
	if (!di->power_enabled) {
		dev_dbg(&di->i2c->dev, "power is not enabled\n");
		return 0;
	}
	convert_msec = ap3426_calc_conversion_time(di, als_enabled, ps_enabled);

	if (als_enabled && ps_enabled)
		delay = min(als_delay, ps_delay);
	else if (als_enabled)
		delay = als_delay;
	else if (ps_enabled)
		delay = ps_delay;
	else
		return 0;

	if (delay < convert_msec)
		delay = 0;
	else
		delay -= convert_msec;

	/* Insert delay_msec into wait slots. The maximum is 255 * 5ms */
	dev_dbg(&di->i2c->dev, "wait time: %lu\n", min(delay / 5UL, 255UL));
	rc = regmap_write(di->regmap, AP3426_REG_WAIT_TIME,
			min(delay / 5UL, 255UL));
	if (rc) {
		dev_err(&di->i2c->dev, "write %d failed\n",
				AP3426_REG_WAIT_TIME);
		return rc;
	}

	return 0;
}

static void ap3426_als_enable_work(struct work_struct *work)
{
	struct ap3426_data *di = container_of(work, struct ap3426_data,
			als_enable_work);

	mutex_lock(&di->ops_lock);
	if (!di->power_enabled) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), true)) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}

		msleep(AP3426_BOOT_TIME_MS);
		di->power_enabled = true;
		if (ap3426_init_device(di)) {
			dev_err(&di->i2c->dev, "init device failed\n");
			goto exit_power_off;
		}
	}

	/* Old HAL: Sync to last delay. New HAL: Sync to current delay */
	if (ap3426_sync_delay(di, 1, di->ps_enabled, di->als_delay,
				di->ps_delay))
		goto exit_power_off;

	if (ap3426_enable_als(di, 1)) {
		dev_err(&di->i2c->dev, "enable als failed\n");
		goto exit_power_off;
	}

exit_power_off:
	if ((!di->als_enabled) && (!di->ps_enabled) &&
			di->power_enabled) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}
		di->power_enabled = false;
	}

exit:
	mutex_unlock(&di->ops_lock);
	return;
}


static void ap3426_als_disable_work(struct work_struct *work)
{
	struct ap3426_data *di = container_of(work, struct ap3426_data,
			als_disable_work);

	mutex_lock(&di->ops_lock);

	if (ap3426_enable_als(di, 0)) {
		dev_err(&di->i2c->dev, "disable als failed\n");
		goto exit;
	}

	if (ap3426_sync_delay(di, 0, di->ps_enabled, di->als_delay,
			di->ps_delay))
		goto exit;

	if ((!di->als_enabled) && (!di->ps_enabled) && di->power_enabled) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}

		di->power_enabled = false;
	}

exit:
	mutex_unlock(&di->ops_lock);
}

static void ap3426_ps_enable_work(struct work_struct *work)
{
	struct ap3426_data *di = container_of(work, struct ap3426_data,
			ps_enable_work);

	mutex_lock(&di->ops_lock);
	if (!di->power_enabled) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), true)) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}

		msleep(AP3426_BOOT_TIME_MS);
		di->power_enabled = true;

		if (ap3426_init_device(di)) {
			dev_err(&di->i2c->dev, "init device failed\n");
			goto exit_power_off;
		}
	}

	/* Old HAL: Sync to last delay. New HAL: Sync to current delay */
	if (ap3426_sync_delay(di, di->als_enabled, 1, di->als_delay,
				di->ps_delay))
		goto exit_power_off;

	if (ap3426_enable_ps(di, 1)) {
		dev_err(&di->i2c->dev, "enable ps failed\n");
		goto exit_power_off;
	}

exit_power_off:
	if ((!di->als_enabled) && (!di->ps_enabled) &&
			di->power_enabled) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}
		di->power_enabled = false;
	}

exit:
	mutex_unlock(&di->ops_lock);
	return;
}

static void ap3426_ps_disable_work(struct work_struct *work)
{
	struct ap3426_data *di = container_of(work, struct ap3426_data,
			ps_disable_work);

	mutex_lock(&di->ops_lock);

	if (ap3426_enable_ps(di, 0)) {
		dev_err(&di->i2c->dev, "disable ps failed\n");
		goto exit;
	}

	if (ap3426_sync_delay(di, di->als_enabled, 0, di->als_delay,
			di->ps_delay))
		goto exit;

	if ((!di->als_enabled) && (!di->ps_enabled) && di->power_enabled) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}

		di->power_enabled = false;
	}
exit:
	mutex_unlock(&di->ops_lock);
}

static struct regmap_config ap3426_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ap3426_cdev_enable_als(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	int res = 0;
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, als_cdev);

	mutex_lock(&di->ops_lock);
       printk("%s-------------enable=%d\n",__func__,enable);
	if (enable){
		queue_work(di->workqueue, &di->als_enable_work);
#ifdef ALS_POLLING_MODE
		hrtimer_start(&di->ls_timer, di->ls_delay, HRTIMER_MODE_REL);
#endif
	}
	else{
		queue_work(di->workqueue, &di->als_disable_work);
#ifdef ALS_POLLING_MODE
		hrtimer_cancel(&di->ls_timer);					
		cancel_work_sync(&di->di_ls_work);
#endif
	}
	mutex_unlock(&di->ops_lock);

	return res;
}

static int ap3426_cdev_enable_ps(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, ps_cdev);

	mutex_lock(&di->ops_lock);
	printk("%s=========enable=%d\n",__func__,enable);
	if (enable)
		queue_work(di->workqueue, &di->ps_enable_work);
	else
		queue_work(di->workqueue, &di->ps_disable_work);

	mutex_unlock(&di->ops_lock);

	return 0;
}

static int ap3426_cdev_set_als_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, als_cdev);

	mutex_lock(&di->ops_lock);

	di->als_delay = delay_msec;
	ap3426_sync_delay(di, di->als_enabled, di->ps_enabled,
			di->als_delay, di->ps_delay);

	mutex_unlock(&di->ops_lock);

	return 0;
}

static int ap3426_cdev_set_ps_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, ps_cdev);

	mutex_lock(&di->ops_lock);

	di->ps_delay = delay_msec;
	ap3426_sync_delay(di, di->als_enabled, di->ps_enabled,
			di->als_delay, di->ps_delay);

	mutex_unlock(&di->ops_lock);

	return 0;
}

static int ap3426_cdev_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, ps_cdev);

	input_event(di->input_proximity, EV_SYN, SYN_CONFIG,
			di->flush_count++);
	input_sync(di->input_proximity);

	return 0;
}

static int ap3426_cdev_als_flush(struct sensors_classdev *sensors_cdev)
{
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, als_cdev);

	input_event(di->input_light, EV_SYN, SYN_CONFIG, di->flush_count++);
	input_sync(di->input_light);

	return 0;
}

/* This function should be called when sensor is disabled */
static int ap3426_cdev_ps_calibrate(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now)
{
	int rc;
	int power;
	unsigned int config;
	unsigned int interrupt;
	u16 min = PS_DATA_MASK;
	u16 max = 0;
	u16 average =0;
	u32 sum =0;
	u8 ps_data[2];
	int count = AP3426_CALIBRATE_SAMPLES;
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, ps_cdev);

	if (axis != AXIS_BIAS)
		return 0;

	mutex_lock(&di->ops_lock);

	power = di->power_enabled;
	if (!power) {
		rc = sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), true);
		if (rc) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}

		msleep(AP3426_BOOT_TIME_MS);

		rc = ap3426_init_device(di);
		if (rc) {
			dev_err(&di->i2c->dev, "init ap3426 failed\n");
			goto exit;
		}
	}

	rc = regmap_read(di->regmap, AP3426_REG_INT_CTL, &interrupt);
	if (rc) {
		dev_err(&di->i2c->dev, "read interrupt configuration failed\n");
		goto exit_power_off;
	}

	/* disable interrupt */
	rc = regmap_write(di->regmap, AP3426_REG_INT_CTL, 0x0);
	if (rc) {
		dev_err(&di->i2c->dev, "disable interrupt failed\n");
		goto exit_power_off;
	}

	rc = regmap_read(di->regmap, AP3426_REG_CONFIG, &config);
	if (rc) {
		dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
				AP3426_REG_CONFIG, rc);
		goto exit_enable_interrupt;
	}

	/* disable the als_sensor */
	rc = regmap_write(di->regmap, AP3426_REG_CONFIG,
			config & (~0x1));
	if (rc) {
		dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
				AP3426_REG_CONFIG, rc);
		goto exit_enable_interrupt;
	}

	msleep(10);
	
	/* clear wait time */
	rc = regmap_write(di->regmap, AP3426_REG_WAIT_TIME, 0x0);
	if (rc) {
		dev_err(&di->i2c->dev, "clear wait time failed\n");
		goto exit_disable_als;
	}

	/* clear offset */
	ps_data[0] = 0;
	ps_data[1] = 0;
	rc = regmap_bulk_write(di->regmap, AP3426_REG_PS_CAL_L,
			ps_data, 2);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
				AP3426_REG_PS_CAL_L, rc);
		goto exit_disable_als;
	}
	/*0070330:fix cali failed issue,zhanghaibin,20150925*/
	/* enable ps sensor */
	rc = regmap_write(di->regmap, AP3426_REG_CONFIG, config | 0x02);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
				AP3426_REG_CONFIG, rc);
		goto exit_disable_als;
	}
	regmap_write(di->regmap, 0x28, 0);
	regmap_write(di->regmap, 0x29, 0);
	while (--count) {
		/*
		 * This function is expected to be executed only 1 time in
		 * factory and never be executed again during the device's
		 * life time. It's fine to busy wait for data ready.
		 */
		usleep_range(ap3426_calc_conversion_time(di, 0, 1) * 1000,
			(ap3426_calc_conversion_time(di, 0, 1) + 1) * 1000);
		rc = regmap_bulk_read(di->regmap, AP3426_REG_PS_DATA_LOW,
				ps_data, 2);
		if (rc || ((ps_data[1] << 8) | ps_data[0])>500) {
			dev_err(&di->i2c->dev, "read PS data failed\n");
			break;
		}
		if (min > ((ps_data[1] << 8) | ps_data[0]))
			min = (ps_data[1] << 8) | ps_data[0];
		if (max < ((ps_data[1] << 8) | ps_data[0]))
			max = (ps_data[1] << 8) | ps_data[0];
		sum +=  (ps_data[1] << 8) | ps_data[0];
	}

	if (!count) {
		if (min > (PS_DATA_MASK >> 1)) {
			dev_err(&di->i2c->dev, "ps data out of range, check if shield\n");
			rc = -EINVAL;
			goto exit_disable_ps;
		}
		sum = sum - max -min;
		average = sum >>4;
		if (apply_now) {
			ps_data[0] = PS_LOW_BYTE(average);
			ps_data[1] = PS_HIGH_BYTE(average);
			rc = regmap_bulk_write(di->regmap, AP3426_REG_PS_CAL_L,
					ps_data, 2);
			if (rc) {
				dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
						AP3426_REG_PS_CAL_L, rc);
				goto exit_disable_ps;
			}
			di->bias = average;
		}

		snprintf(di->calibrate_buf, sizeof(di->calibrate_buf), "0,0,%d",
				average);
		sensors_cdev->params = di->calibrate_buf;
		dev_dbg(&di->i2c->dev, "result: %s\n", di->calibrate_buf);
	} else {
		dev_err(&di->i2c->dev, "calibration failed\n");
		rc = -EINVAL;
	}

exit_disable_ps:
	if (regmap_write(di->regmap, AP3426_REG_CONFIG, config & (~0x02))) {
		dev_err(&di->i2c->dev, "write %d failed.\n",
				AP3426_REG_CONFIG);
	}

exit_disable_als:
	/*0070330:fix cali failed issue,zhanghaibin,20150925*/
	/*enable als*/
	if (regmap_write(di->regmap, AP3426_REG_CONFIG, config | 0x01)) {
		dev_err(&di->i2c->dev, "write %d failed.\n",
				AP3426_REG_CONFIG);
	}
		
exit_enable_interrupt:
	if (regmap_write(di->regmap, AP3426_REG_INT_CTL, interrupt)) {
		dev_err(&di->i2c->dev, "enable interrupt failed\n");
	}

exit_power_off:
	if (!power) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&di->i2c->dev, "power off sensor failed.\n");
		}
	}
exit:
	mutex_unlock(&di->ops_lock);
	return rc;
}

static int ap3426_cdev_ps_write_cal(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	int power;
	u8 ps_data[2];
	int rc;
	struct ap3426_data *di = container_of(sensors_cdev,
			struct ap3426_data, ps_cdev);

	mutex_lock(&di->ops_lock);
	power = di->power_enabled;
	if (!power) {
		rc = sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), true);
		if (rc) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}
	}

	di->bias = cal_result->bias;
	ps_data[0] = PS_LOW_BYTE(cal_result->bias);
	ps_data[1] = PS_HIGH_BYTE(cal_result->bias);
	rc = regmap_bulk_write(di->regmap, AP3426_REG_PS_CAL_L,
			ps_data, 2);
	if (rc) {
		dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
				AP3426_REG_PS_CAL_L, rc);
		goto exit_power_off;
	}

	snprintf(di->calibrate_buf, 10, "0,0,%d", di->bias);

exit_power_off:
	if (!power) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&di->i2c->dev, "power off sensor failed.\n");
			goto exit;
		}
	}
exit:

	mutex_unlock(&di->ops_lock);
	return 0;
};

static ssize_t ap3426_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ap3426_data *di = dev_get_drvdata(dev);
	unsigned int val;
	int rc;
	ssize_t count = 0;
	int i;

	//if (di->reg_addr == AP3426_REG_MAGIC) {
		for (i = 0; i < AP3426_REG_COUNT; i++) {
			rc = regmap_read(di->regmap, AP3426_REG_CONFIG + i,
					&val);
			if (rc) {
				dev_err(&di->i2c->dev, "read %d failed\n",
						AP3426_REG_CONFIG + i);
				break;
			}
			count += snprintf(&buf[count], PAGE_SIZE,
					"0x%x: 0x%x\n", AP3426_REG_CONFIG + i,
					val);
		}
	/*} else {
		rc = regmap_read(di->regmap, di->reg_addr, &val);
		if (rc) {
			dev_err(&di->i2c->dev, "read %d failed\n",
					di->reg_addr);
			return rc;
		}
		count += snprintf(&buf[count], PAGE_SIZE, "0x%x:0x%x\n",
				di->reg_addr, val);
	}
*/
	return count;
}

static ssize_t ap3426_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct ap3426_data *di = dev_get_drvdata(dev);
	unsigned int reg;
	unsigned int val;
	unsigned int cmd;
	int rc;

	if (sscanf(buf, "%u %u %u\n", &cmd, &reg, &val) < 2) {
		dev_err(&di->i2c->dev, "argument error\n");
		return -EINVAL;
	}

	if (cmd == CMD_WRITE) {
		rc = regmap_write(di->regmap, reg, val);
		if (rc) {
			dev_err(&di->i2c->dev, "write %d failed\n", reg);
			return rc;
		}
	} else if (cmd == CMD_READ) {
		di->reg_addr = reg;
		dev_dbg(&di->i2c->dev, "register address set to 0x%x\n", reg);
	}

	return size;
}

static DEVICE_ATTR(register, S_IWUSR | S_IRUGO,
		ap3426_register_show,
		ap3426_register_store);

static struct attribute *ap3426_attr[] = {
	&dev_attr_register.attr,
	NULL
};

static const struct attribute_group ap3426_attr_group = {
	.attrs = ap3426_attr,
};

static int ap3426_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ap3426_data *di;
	int res = 0;
	unsigned int value = 0;
	dev_dbg(&client->dev, "probing ap3426...\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "ap3426 i2c check failed.\n");
		return -ENODEV;
	}

	di = devm_kzalloc(&client->dev, sizeof(struct ap3426_data),
			GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "memory allocation failed,\n");
		return -ENOMEM;
	}

	di->i2c = client;

	if (client->dev.of_node) {
		res = ap3426_parse_dt(&client->dev, di);
		if (res) {
			dev_err(&client->dev,
				"unable to parse device tree.(%d)\n", res);
			goto out;
		}
	} else {
		dev_err(&client->dev, "device tree not found.\n");
		res = -ENODEV;
		goto out;
	}

	dev_set_drvdata(&client->dev, di);
	mutex_init(&di->ops_lock);

	di->regmap = devm_regmap_init_i2c(client, &ap3426_regmap_config);
	if (IS_ERR(di->regmap)) {
		dev_err(&client->dev, "init regmap failed.(%ld)\n",
				PTR_ERR(di->regmap));
		res = PTR_ERR(di->regmap);
		goto out;
	}

	res = sensor_power_init(&client->dev, power_config,
			ARRAY_SIZE(power_config));
	if (res) {
		dev_err(&client->dev, "init power failed.\n");
		goto out;
	}

	res = sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), true);
	if (res) {
		dev_err(&client->dev, "power up sensor failed.\n");
		goto err_power_config;
	}

	res = sensor_pinctrl_init(&client->dev, &pin_config);
	if (res) {
		dev_err(&client->dev, "init pinctrl failed.\n");
		goto err_pinctrl_init;
	}

	/* wait the device to boot up */
	msleep(AP3426_BOOT_TIME_MS);

	res = ap3426_check_device(di);
	if (res) {
		dev_err(&client->dev, "check device failed.\n");
		goto err_check_device;
	}

	res = ap3426_init_device(di);
	if (res) {
		dev_err(&client->dev, "check device failed.\n");
		goto err_init_device;
	}

	/* configure interrupt */
	if (gpio_is_valid(di->irq_gpio)) {
		res = gpio_request(di->irq_gpio, "ap3426_interrupt");
		if (res) {
			dev_err(&client->dev,
				"unable to request interrupt gpio %d\n",
				di->irq_gpio);
			goto err_request_gpio;
		}

		res = gpio_direction_input(di->irq_gpio);
		if (res) {
			dev_err(&client->dev,
				"unable to set direction for gpio %d\n",
				di->irq_gpio);
			goto err_set_direction;
		}

		di->irq = gpio_to_irq(di->irq_gpio);

		res = devm_request_irq(&client->dev, di->irq,
				ap3426_irq_handler,
				di->irq_flags | IRQF_ONESHOT,
				"ap3426", di);

		if (res) {
			dev_err(&client->dev,
					"request irq %d failed(%d),\n",
					di->irq, res);
			goto err_request_irq;
		}

		/* device wakeup initialization */
		device_init_wakeup(&client->dev, 1);

		di->workqueue = alloc_workqueue("ap3426_workqueue",
				WQ_NON_REENTRANT | WQ_FREEZABLE, 0);
		INIT_WORK(&di->report_work, ap3426_report_work);
		INIT_WORK(&di->als_enable_work, ap3426_als_enable_work);
		INIT_WORK(&di->als_disable_work, ap3426_als_disable_work);
		INIT_WORK(&di->ps_enable_work, ap3426_ps_enable_work);
		INIT_WORK(&di->ps_disable_work, ap3426_ps_disable_work);

#ifdef DI_AUTO_TUNE
		di->di_ps_tune0_wq = create_singlethread_workqueue("di_ps_tune0_wq");
		INIT_WORK(&di->di_ps_tune0_work, di_ps_tune0_work_func);
		hrtimer_init(&di->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		di->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
		di->ps_tune0_timer.function = di_ps_tune0_timer_func;
#endif

#ifdef ALS_POLLING_MODE
		di->di_ls_wq = create_singlethread_workqueue("di_ls_wq ");
		INIT_WORK(&di->di_ls_work, ap3426_ls_work_handler);
		hrtimer_init(&di->ls_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		di->ls_delay= ns_to_ktime(200 * NSEC_PER_MSEC);
		di->ls_timer.function = di_ls_timer_func;
#endif
	} else {
		res = -ENODEV;
		goto err_init_device;
	}

	res = sysfs_create_group(&client->dev.kobj, &ap3426_attr_group);
	if (res) {
		dev_err(&client->dev, "sysfs create group failed\n");
		goto err_create_group;
	}

	res = ap3426_init_input(di);
	if (res) {
		dev_err(&client->dev, "init input failed.\n");
		goto err_init_input;
	}

	/* input device should hold a 200ms wake lock */
	device_init_wakeup(&di->input_proximity->dev, 1);

	di->als_cdev = als_cdev;
	di->als_cdev.sensors_enable = ap3426_cdev_enable_als;
	di->als_cdev.sensors_poll_delay = ap3426_cdev_set_als_delay;
	di->als_cdev.sensors_flush = ap3426_cdev_als_flush;
	res = sensors_classdev_register(&di->input_light->dev, &di->als_cdev);
	if (res) {
		dev_err(&client->dev, "sensors class register failed.\n");
		goto err_register_als_cdev;
	}

	di->ps_cdev = ps_cdev;
	di->ps_cdev.sensors_enable = ap3426_cdev_enable_ps;
	di->ps_cdev.sensors_poll_delay = ap3426_cdev_set_ps_delay;
	di->ps_cdev.sensors_flush = ap3426_cdev_ps_flush;
	di->ps_cdev.sensors_calibrate = ap3426_cdev_ps_calibrate;
	di->ps_cdev.sensors_write_cal_params = ap3426_cdev_ps_write_cal;
	di->ps_cdev.params = di->calibrate_buf;
	res = sensors_classdev_register(&di->input_proximity->dev,
			&di->ps_cdev);
	if (res) {
		dev_err(&client->dev, "sensors class register failed.\n");
		goto err_register_ps_cdev;
	}

	sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), false);
	di->power_enabled = false;

	#ifdef CONFIG_HY_DRV_ASSIST
	alsps_assist_register_attr("ic",&ap3426_ic_show,NULL);
	alsps_assist_register_attr("vendor",&ap3426_vendor_show,NULL);
	alsps_assist_register_attr("exist",&ap3426_exist_show,NULL);
	alsps_assist_register_attr("ps_data",&ap3426_ps_data_show,NULL);
	#endif
	alsps_sensor=1;
	
	dev_info(&client->dev, "ap3426 successfully probed!\n");
	/* 0067273:zhiyu add for power-key virtual far event */
	g_ap3426_data=di;
	/* 0067273:zhiyu add for power-key virtual far event */
	#ifdef DI_AUTO_CAL
	regmap_read(di->regmap, AP3426_REG_CONFIG, &value);
	value |= 0x02;
	 regmap_write(di->regmap,
					AP3426_REG_CONFIG, value);
	msleep(100);	
       AP3xx6_Calibration(di);
	 regmap_read(di->regmap, AP3426_REG_CONFIG, &value);
	value &= ~(0x02);
	 regmap_write(di->regmap,AP3426_REG_CONFIG, value);

	#endif  
	return 0;

err_register_ps_cdev:
	sensors_classdev_unregister(&di->als_cdev);
err_register_als_cdev:
	device_init_wakeup(&di->input_proximity->dev, 0);
err_init_input:
	sysfs_remove_group(&client->dev.kobj, &ap3426_attr_group);
err_create_group:
#ifdef ALS_POLLING_MODE
	destroy_workqueue(di->di_ls_wq);	
#endif	
#ifdef DI_AUTO_TUNE
	destroy_workqueue(di->di_ps_tune0_wq);	
#endif	
	destroy_workqueue(di->workqueue);

err_request_irq:
err_set_direction:
	gpio_free(di->irq_gpio);
err_request_gpio:
err_init_device:
	device_init_wakeup(&client->dev, 0);
err_check_device:
err_pinctrl_init:
	sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), false);
err_power_config:
	sensor_power_deinit(&client->dev, power_config,
			ARRAY_SIZE(power_config));
out:
	return res;
}

static int ap3426_remove(struct i2c_client *client)
{
	struct ap3426_data *di = dev_get_drvdata(&client->dev);

	sensors_classdev_unregister(&di->ps_cdev);
	sensors_classdev_unregister(&di->als_cdev);

	destroy_workqueue(di->workqueue);
	device_init_wakeup(&di->i2c->dev, 0);
	device_init_wakeup(&di->input_proximity->dev, 0);

	sensor_power_config(&client->dev, power_config,
			ARRAY_SIZE(power_config), false);
	sensor_power_deinit(&client->dev, power_config,
			ARRAY_SIZE(power_config));
#ifdef DI_AUTO_TUNE
	destroy_workqueue(di->di_ps_tune0_wq);	
#endif	
#ifdef ALS_POLLING_MODE
	destroy_workqueue(di->di_ls_wq);	
#endif	

	return 0;
}

static int ap3426_suspend(struct device *dev)
{
	int res = 0;
	struct ap3426_data *di = dev_get_drvdata(dev);
	unsigned int config;
#ifndef DI_AUTO_TUNE
	u8 ps_data[4];
	int idx = di->ps_wakeup_threshold;
#endif

	dev_dbg(dev, "suspending ap3426...");

	mutex_lock(&di->ops_lock);

	/* proximity is enabled */
	if (di->ps_enabled) {
		/* disable als sensor to avoid wake up by als interrupt */
		if (di->als_enabled) {
			res = regmap_read(di->regmap, AP3426_REG_CONFIG,
					&config);
			if (res) {
				dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
						AP3426_REG_CONFIG, res);
				goto exit;
			}


			res = regmap_write(di->regmap, AP3426_REG_CONFIG,
					config & (~0x1));
			if (res) {
				dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
						AP3426_REG_CONFIG, res);
				goto exit;
			}

			ap3426_sync_delay(di, 0, 1, 0, di->ps_delay);
		}

		/* Don't power off sensor because proximity is a
		 * wake up sensor.
		 */
		if (device_may_wakeup(&di->i2c->dev)) {
			dev_dbg(&di->i2c->dev, "enable irq wake\n");
			enable_irq_wake(di->irq);
		}
#ifndef DI_AUTO_TUNE
		/* Setup threshold to avoid frequent wakeup */
		if (device_may_wakeup(&di->i2c->dev) &&
				(idx != AP3426_WAKEUP_ANY_CHANGE)) {
			dev_dbg(&di->i2c->dev, "last ps: %d\n", di->last_ps);
			if (di->last_ps > idx) {
				ps_data[0] = 0x0;
				ps_data[1] = 0x0;
				ps_data[2] =
					PS_LOW_BYTE(ps_distance_table[idx]);
				ps_data[3] =
					PS_HIGH_BYTE(ps_distance_table[idx]);
			} else {
				ps_data[0] =
					PS_LOW_BYTE(ps_distance_table[idx]);
				ps_data[1] =
					PS_HIGH_BYTE(ps_distance_table[idx]);
				ps_data[2] = PS_LOW_BYTE(PS_DATA_MASK);
				ps_data[3] = PS_HIGH_BYTE(PS_DATA_MASK);
			}

			res = regmap_bulk_write(di->regmap,
					AP3426_REG_PS_LOW_THRES_0, ps_data, 4);
			if (res) {
				dev_err(&di->i2c->dev, "set up threshold failed\n");
				goto exit;
			}
		}
#endif
	} else {
		/* power off */
		disable_irq(di->irq);
		if (di->power_enabled) {
			res = sensor_power_config(dev, power_config,
					ARRAY_SIZE(power_config), false);
			if (res) {
				dev_err(dev, "failed to suspend ap3426\n");
				enable_irq(di->irq);
				goto exit;
			}
		}
		pinctrl_select_state(pin_config.pinctrl, pin_config.state[1]);
	}
exit:
	mutex_unlock(&di->ops_lock);
	return res;
}

static int ap3426_resume(struct device *dev)
{
	int res = 0;
	struct ap3426_data *di = dev_get_drvdata(dev);
	unsigned int config;

	dev_dbg(dev, "resuming ap3426...");
	if (di->ps_enabled) {
		if (device_may_wakeup(&di->i2c->dev)) {
			dev_dbg(&di->i2c->dev, "disable irq wake\n");
			disable_irq_wake(di->irq);
		}

		if (di->als_enabled) {
			res = regmap_read(di->regmap, AP3426_REG_CONFIG,
					&config);
			if (res) {
				dev_err(&di->i2c->dev, "read %d failed.(%d)\n",
						AP3426_REG_CONFIG, res);
				goto exit;
			}

			res = regmap_write(di->regmap, AP3426_REG_CONFIG,
					config | 0x1);
			if (res) {
				dev_err(&di->i2c->dev, "write %d failed.(%d)\n",
						AP3426_REG_CONFIG, res);
				goto exit;
			}

			ap3426_sync_delay(di, 1, 1, di->als_delay,
					di->ps_delay);
		}

	} else {
		pinctrl_select_state(pin_config.pinctrl, pin_config.state[0]);
		/* Power up sensor */
		if (di->power_enabled) {
			res = sensor_power_config(dev, power_config,
					ARRAY_SIZE(power_config), true);
			if (res) {
				dev_err(dev, "failed to power up ap3426\n");
				goto exit;
			}
			msleep(AP3426_BOOT_TIME_MS);

			res = ap3426_init_device(di);
			if (res) {
				dev_err(dev, "failed to init ap3426\n");
				goto exit_power_off;
			}

			ap3426_sync_delay(di, di->als_enabled, 0, di->als_delay,
					di->ps_delay);
		}

		if (di->als_enabled) {
			res = ap3426_enable_als(di, di->als_enabled);
			if (res) {
				dev_err(dev, "failed to enable ap3426\n");
				goto exit_power_off;
			}
		}

		enable_irq(di->irq);
	}

	return res;

exit_power_off:
	if ((!di->als_enabled) && (!di->ps_enabled) &&
			di->power_enabled) {
		if (sensor_power_config(&di->i2c->dev, power_config,
					ARRAY_SIZE(power_config), false)) {
			dev_err(&di->i2c->dev, "power up sensor failed.\n");
			goto exit;
		}
		di->power_enabled = false;
	}

exit:
	return res;
}

static const struct i2c_device_id ap3426_id[] = {
	{ AP3426_I2C_NAME, 0 },
	{ }
};

static struct of_device_id ap3426_match_table[] = {
	{ .compatible = "di,ap3426", },
	{ },
};

static const struct dev_pm_ops ap3426_pm_ops = {
	.suspend = ap3426_suspend,
	.resume = ap3426_resume,
};

static struct i2c_driver ap3426_driver = {
	.probe		= ap3426_probe,
	.remove	= ap3426_remove,
	.id_table	= ap3426_id,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= AP3426_I2C_NAME,
		.of_match_table = ap3426_match_table,
		.pm = &ap3426_pm_ops,
	},
};

module_i2c_driver(ap3426_driver);

MODULE_DESCRIPTION("AP3426 ALPS Driver");
MODULE_LICENSE("GPLv2");

