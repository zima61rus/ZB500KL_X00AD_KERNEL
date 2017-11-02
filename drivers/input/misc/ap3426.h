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

#ifndef _AP3426_H
#define _AP3426_H

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/sensors.h>
#include <linux/pm_wakeup.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#ifdef CONFIG_HY_DRV_ASSIST
#include <linux/hy-assist.h>
#endif

#define AP3426_I2C_NAME			"ap3426"
#define AP3426_LIGHT_INPUT_NAME		"ap3426-light"
#define AP3426_PROXIMITY_INPUT_NAME	"ap3426-proximity"

#define DI_AUTO_CAL
#ifdef DI_AUTO_CAL
       #define DI_PS_CAL_THR 500
#endif

/* AP3426 registers */
#define AP3426_REG_CONFIG		0x00
#define AP3426_REG_INT_FLAG		0x01
#define AP3426_REG_INT_CTL		0x02
#define AP3426_REG_WAIT_TIME		0x06
#define AP3426_REG_IR_DATA_LOW		0x0A
#define AP3426_REG_IR_DATA_HIGH		0x0B
#define AP3426_REG_ALS_DATA_LOW		0x0C
#define AP3426_REG_ALS_DATA_HIGH	0x0D
#define AP3426_REG_PS_DATA_LOW		0x0E
#define AP3426_REG_PS_DATA_HIGH		0x0F
#define AP3426_REG_ALS_GAIN		0x10
#define AP3426_REG_ALS_PERSIST		0x14
#define AP3426_REG_ALS_LOW_THRES_0	0x1A
#define AP3426_REG_ALS_LOW_THRES_1	0x1B
#define AP3426_REG_ALS_HIGH_THRES_0	0x1C
#define AP3426_REG_ALS_HIGH_THRES_1	0x1D
#define AP3426_REG_PS_GAIN		0x20
#define AP3426_REG_PS_LED_DRIVER	0x21
#define AP3426_REG_PS_INT_FORM		0x22
#define AP3426_REG_PS_MEAN_TIME		0x23
#define AP3426_REG_PS_SMART_INT		0x24
#define AP3426_REG_PS_INT_TIME		0x25
#define AP3426_REG_PS_PERSIST		0x26
#define AP3426_REG_PS_CAL_L		0x28
#define AP3426_REG_PS_CAL_H		0x29
#define AP3426_REG_PS_LOW_THRES_0	0x2A
#define AP3426_REG_PS_LOW_THRES_1	0x2B
#define AP3426_REG_PS_HIGH_THRES_0	0x2C
#define AP3426_REG_PS_HIGH_THRES_1	0x2D
#define AP3426_REG_PID		0x04
#define AP3426_REG_PID_VALUE		0x06

#define AP3426_REG_MAGIC		0xFF
#define AP3426_REG_COUNT		0x2E

#define AP3426_ALS_INT_MASK		0x01
#define AP3426_PS_INT_MASK		0x02

#define ALS_GAIN_SWITCH_RATIO		80

/* AP3426 ALS data is 16 bit */
#define ALS_DATA_MASK		0xffff
#define ALS_LOW_BYTE(data)	((data) & 0xff)
#define ALS_HIGH_BYTE(data)	(((data) >> 8) & 0xff)

/* AP3426 PS data is 10 bit */
#define PS_DATA_MASK		0x3ff
#define PS_LOW_BYTE(data)	((data) & 0xff)
#define PS_HIGH_BYTE(data)	(((data) >> 8) & 0x3)

/* default als sensitivity in lux */
#define AP3426_ALS_SENSITIVITY		50

/* AP3426 takes at least 10ms to boot up */
#define AP3426_BOOT_TIME_MS		12

#define AP3426_CALIBRATE_SAMPLES	19
/* als and ps interrupt enabled, clear by software */
#define AP3426_INT_CONFIG		0x89

/* Any proximity distance change will wakeup SoC */
#define AP3426_WAKEUP_ANY_CHANGE	0xff

#define CAL_BUF_LEN			16

#define DI_AUTO_TUNE
#ifdef DI_AUTO_TUNE
	#define DI_MAX_MIN_DIFF	150
	#define DI_LT_N_CT	65
	#define DI_HT_N_CT	200
	#define AP3426_OBJ_COMMAND	0x01
	#define AP3426_OBJ_MASK		0x10
	#define AP3426_OBJ_SHIFT	(4)
#endif

#define ALS_POLLING_MODE

enum {
	CMD_WRITE = 0,
	CMD_READ = 1,
};

struct regulator_map {
	struct regulator	*regulator;
	int			min_uv;
	int			max_uv;
	char			*supply;
};

struct pinctrl_config {
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*state[2];
	char			*name[2];
};

struct ap3426_data {
	struct i2c_client	*i2c;
	struct regmap		*regmap;
	struct regulator	*config;
	struct input_dev	*input_light;
	struct input_dev	*input_proximity;
	struct workqueue_struct	*workqueue;

	struct sensors_classdev	als_cdev;
	struct sensors_classdev	ps_cdev;
	struct mutex		ops_lock;
	struct work_struct	report_work;
	struct work_struct	als_enable_work;
	struct work_struct	als_disable_work;
	struct work_struct	ps_enable_work;
	struct work_struct	ps_disable_work;
	atomic_t		wake_count;

	int			irq_gpio;
	int			irq;
	bool			als_enabled;
	bool			ps_enabled;
	u32			irq_flags;
	unsigned int		als_delay;
	unsigned int		ps_delay;
	int			als_cal;
	int			ps_cal;
	int			als_gain;
	int			als_persist;
	int			ps_gain;
	int			ps_persist;
	int			ps_led_driver;
	int			ps_mean_time;
	int			ps_integrated_time;
	int			ps_wakeup_threshold;

	int			last_als;
	int			last_ps;
	int			flush_count;
	int			power_enabled;

	unsigned int		reg_addr;
	char			calibrate_buf[CAL_BUF_LEN];
	unsigned int		bias;

	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
#ifdef DI_AUTO_TUNE
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *di_ps_tune0_wq;
    	struct work_struct di_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif	

#ifdef ALS_POLLING_MODE
	struct hrtimer ls_timer;	
	struct workqueue_struct *di_ls_wq;
    	struct work_struct di_ls_work;
	ktime_t ls_delay;
#endif


};
   
   int ap3426_enable_ps(struct ap3426_data *di, int enable);


#endif
