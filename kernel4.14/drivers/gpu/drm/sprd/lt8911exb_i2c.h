/*:
 * Copyright (C) 2019 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef _LT8911EXB_I2C_H_
#define _LT8911EXB_I2C_H_

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <asm/uaccess.h> 
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define TAG "[LT8911EXB]"

#define LT_ERR(fmt, ...) \
		pr_err(TAG " %s() " fmt "\n", __func__, ##__VA_ARGS__)
#define LT_WARN(fmt, ...) \
		pr_warn(TAG " %s() " fmt "\n", __func__, ##__VA_ARGS__)
#define LT_INFO(fmt, ...) \
		pr_info(TAG " %s() " fmt "\n", __func__, ##__VA_ARGS__)

#define RESISTER_HSHCAL_I2C

#define HSHCAL_DEVICE_NAME    "lt8911b"

#define I2C_RETRY_DELAY       5
#define I2C_RETRIES           5

#define HSHCAL_DRIVER_NAME    "lt8911b"
#define I2C_HSHCAL_ADDR       (0x29)        /* 001 1000    */
#define I2C_BUS_NUMBER        1

#define EVENT_TYPE_RELATIVE_HUMIDITY   ABS_PRESSURE
#define EVENT_TYPE_AMBIENT_TEMPERATURE ABS_DISTANCE

#define ALPS_INPUT_FUZZ       0    /* input event threshold */
#define ALPS_INPUT_FLAT       0
#define ALPS_HMD_DEBUG       	1

enum
{
	H_act = 0,
	V_act,
	H_tol,
	V_tol,
	H_bp,
	H_sync,
	V_sync,
	V_bp
};

enum {
	hfp = 0,
	hs,
	hbp,
	hact,
	htotal,
	vfp,
	vs,
	vbp,
	vact,
	vtotal,
	pclk_10khz
};

int it8911b_stop(void);
int it8911b_run(void);

#endif
