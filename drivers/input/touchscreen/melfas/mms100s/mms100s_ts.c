/*
 *  Copyright (C) 2014 Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#ifdef SEC_TSP_FACTORY_TEST
#include <linux/sec-common.h>
#endif

#include "mms100s_ts.h"

#if defined(CONFIG_OF) && defined(USE_OPEN_CLOSE)
#undef CONFIG_HAS_EARLYSUSPEND
#undef CONFIG_PM
#endif

#if INFORM_CHARGER
#include <linux/power_supply.h>
extern void (*tsp_charger_status_cb)(int);
extern u8 current_cable_type;
static struct mms_info *m_info;
#endif

#ifdef CONFIG_OF

static int mms_power_control(struct mms_info *info, int on)
{
	struct device *dev = &info->client->dev;
	const struct mms_platform_data *pdata = info->pdata;
	struct regulator *regulator_dvdd = NULL;
	struct regulator *regulator_avdd = NULL;

	int retval = 0;

	if (info->pwr_enabled == on) {
		tsp_debug_err(true, dev, "%s: Already tsp ic power %s.\n",
					__func__, on == 1 ? "ON" : "OFF");
		return 1;
	}

	regulator_dvdd = regulator_get(NULL, pdata->regulator_dvdd);
	if (IS_ERR_OR_NULL(regulator_dvdd)) {
		tsp_debug_err(true, dev, "%s: Failed to get %s regulator.\n",
			 __func__, pdata->regulator_dvdd);
		goto out;
	}

	regulator_avdd = regulator_get(NULL, pdata->regulator_avdd);
	if (IS_ERR_OR_NULL(regulator_avdd)) {
		tsp_debug_err(true, dev, "%s: Failed to get %s regulator.\n",
			 __func__, pdata->regulator_avdd);
		goto out;
	}

	tsp_debug_info(true, dev, "%s: %s\n", __func__, on ? "on" : "off");

	if (on) {
		retval = regulator_enable(regulator_dvdd);
		if (retval) {
			tsp_debug_err(true, dev, "%s: Failed to enable vdd: %d\n", __func__, retval);
			goto out;
		}

		retval = regulator_enable(regulator_avdd);
		if (retval) {
			tsp_debug_err(true, dev, "%s: Failed to enable avdd: %d\n", __func__, retval);
			goto out;
		}

//		retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_default);
//		if (retval < 0)
//			tsp_debug_err(true, dev, "%s: Failed to configure tsp_attn pin\n", __func__);

		// CHECK it!!!!
//		msleep(50);
	} else {
		if (regulator_is_enabled(regulator_avdd))
			regulator_disable(regulator_avdd);
		if (regulator_is_enabled(regulator_dvdd))
			regulator_disable(regulator_dvdd);

//		retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_sleep);
//		if (retval < 0)
//			tsp_debug_err(true, dev, "%s: Failed to configure tsp_attn pin\n", __func__);
	}

	info->pwr_enabled = on;

out:
	regulator_put(regulator_dvdd);
	regulator_put(regulator_avdd);

	return retval;

}


#if 0
static struct regulator *mms_vdd_regulator;

static int mms_setup_power(struct mms_info *info, bool onoff)
{
	struct i2c_client *client = info->client;
	int min_uv, max_uv;
	int ret = 0;

	if (onoff) {
		if (!mms_vdd_regulator) {
			mms_vdd_regulator = regulator_get(&client->dev, "v_tsp_3v0");

			if (IS_ERR(mms_vdd_regulator)) {
				ret = PTR_ERR(mms_vdd_regulator);
				tsp_debug_err(true, &client->dev,
							"Failed to get mms_vdd_regulator(%d)\n", ret);
				return ret;
			}

			min_uv = max_uv = info->pdata->vdd_regulator_volt;

			ret = regulator_set_voltage(mms_vdd_regulator, min_uv, max_uv);
			if (ret < 0) {
				tsp_debug_err(true, &client->dev, "Failed to set mms_vdd_regulator to \
					%d, %d uV(%d)\n", min_uv, max_uv, ret);
				regulator_put(mms_vdd_regulator);
				return ret;
			}

			tsp_debug_info(true, &client->dev, "Set mms_vdd_regulator to %d, %d uV\n",
				min_uv, max_uv);
		}
	}
	else {
		if (mms_vdd_regulator) {
			regulator_put(mms_vdd_regulator);
			tsp_debug_info(true, &client->dev, "Put mms_vdd_regulator\n");
		}
	}

	return ret;
}

static int mms_onoff_power(struct mms_info *info, bool onoff)
{
	struct i2c_client *client = info->client;
	int ret = 0;

	if (!mms_vdd_regulator) {
		tsp_debug_err(true, &client->dev, "%s : no regulator\n", __func__);
		ret = -EPERM;
		goto out;
	}

	if (onoff)
		ret = regulator_enable(mms_vdd_regulator);
	else
		ret = regulator_disable(mms_vdd_regulator);

	if (ret)
		tsp_debug_err(true, &client->dev, "%s : Failed to %s regulator\n", __func__,
					(onoff) ? "enable" : "disable");
	else
		tsp_debug_info(true, &client->dev, "%s : %s\n", __func__, (onoff) ? "on" : "off");

out:
	return ret;
}
#endif
#endif

#if INFORM_CHARGER
static void mms_set_operation_mode(struct mms_info *info)
{
	char mode_bit = 0;
	char ta_bit, operation_bit;

	/*
		0xx , noise mode
		1XX , normal mode
		x01 , insert TA
		x10 , pull out TA
	*/
	if (!info->noise_mode)
		mode_bit = 0b100;

	if (info->ta_connected) {
		ta_bit = 0b1;
	} else {
		ta_bit = 0b10;
		info->noise_mode = false;
	}

	operation_bit = mode_bit | ta_bit;

	i2c_smbus_write_byte_data(info->client,
								MMS_OPERATION_MODE, operation_bit);

	tsp_debug_info(true, &info->client->dev, "Operation mode is %d\n", operation_bit);
}

static void mms_charger_status_cb(int status)
{
	int mode = (status == POWER_SUPPLY_TYPE_BATTERY);

	if (mode)
		m_info->ta_connected = false;
	else
		m_info->ta_connected = true;

	if (m_info->enabled)
		mms_set_operation_mode(m_info);

	tsp_debug_info(true, &m_info->client->dev, "TA %s\n",
				mode ? "disconnected" : "connected");
}
#endif

/* mms_enable - wake-up func (VDD on)  */
static int mms_enable(struct mms_info *info)
{
	if (info->enabled) {
		tsp_debug_err(true, &info->client->dev, "%s : Already enable\n", __func__);
		return -1;
	}

	/* use vdd control instead */
	mms_power_control(info, 1);

#ifdef MMS_RUNTIME_CONF
	/* Support Runtime Config and Use delayed_work queue */
	info->resume_done = false;
	schedule_delayed_work(&info->work_config_set, msecs_to_jiffies(MMS_HW_INIT_TIME));
#else

#if INFORM_CHARGER
	if (info->ta_connected)
		mms_set_operation_mode(info);
#endif

	enable_irq(info->irq);
	info->enabled = true;
#endif

	return 0;
}

/* mms_disable - sleep func (VDD off) */
static int mms_disable(struct mms_info *info)
{
#ifdef MMS_RUNTIME_CONF
	int retries = 50;
#endif

	if (!info->enabled) {
		tsp_debug_err(true, &info->client->dev, "%s : Already disable\n", __func__);
		return -1;
	}

#ifdef MMS_RUNTIME_CONF
	while (!info->resume_done) {
		if (!retries--)
			break;
		msleep(10);
	}
#endif

	disable_irq(info->irq);
	mms_power_control(info, 0);

	info->enabled = false;

	return 0;
}

/* mms_reboot_hw - IC reset */
static void mms_reboot_hw(struct mms_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_lock_adapter(adapter);

	mms_power_control(info, 0);
	msleep(1);
	mms_power_control(info, 1);
	msleep(MMS_HW_INIT_TIME);

	i2c_unlock_adapter(adapter);
}

/* mms_reboot_all - IC reset after probe */
static void mms_reboot_all(struct mms_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_lock_adapter(adapter);

	mms_power_control(info, 0);
	msleep(1);
	mms_power_control(info, 1);
	msleep(MMS_HW_INIT_TIME);

	i2c_unlock_adapter(adapter);

#ifdef MMS_RUNTIME_CONF
	/* Bring up : Support Runtime Config */
	mms_config_set(info);
#endif
	tsp_debug_info(true, &info->client->dev, "%s called!\n", __func__);
}

/*
 * mms_clear_input_data - all finger point release
 */
static void mms_clear_input_data(struct mms_info *info)
{
	int i;

	for (i = 0; i < MAX_FINGER_NUM; i++) {
		input_mt_slot(info->input_dev, i);

		if (!info->finger_state[i])
			continue;

		info->finger_state[i] = false;
		info->mcount[i] = 0;
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
		tsp_debug_info(true, &info->client->dev, "[R][%d] : forced clear\n", i);
	}

	input_sync(info->input_dev);

	return;
}

/* mms_report_input_data - The position of a touch send to platfrom  */
static void mms_report_input_data(struct mms_info *info, u8 sz, u8 *buf)
{
	struct i2c_client *client = info->client;
	static int esd_cnt = 0;
	int x, y, id, i;
//	int major, pressure, palm;
	int major_axis, minor_axis;
	u8 *tmp;
#if MMS_HAS_TOUCH_KEY
	int key_code, key_state;
#endif

	if (buf[0] == MMS_TEST_MODE) {
		tsp_debug_info(true, &client->dev, "Universal cmd is finished(%d)[%x][%x]\n",
						buf[1], buf[2], buf[3]);
		goto out;
	} else if (buf[0] == MMS_NOTIFY_EVENT) {
#if INFORM_CHARGER
		if (buf[1] == 1)
			info->noise_mode = info->ta_connected;
		else if (buf[1] == 2)
			info->noise_mode = false;
		else {
			tsp_debug_info(true, &client->dev,
						"TSP mode changed to Unknown (%d)\n", buf[1]);
			goto out;
		}

		tsp_debug_info(true, &client->dev, "TSP mode changed to %s\n",
					info->noise_mode ? "Noise" : "Normal");
#endif
		goto out;
	} else if (buf[0] == MMS_ERROR_EVENT) {
		tsp_debug_info(true, &client->dev, "Error detected, restarting TSP\n");
		mms_clear_input_data(info);
		mms_reboot_all(info);
		goto out;
	}

	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		tmp = buf + i;
		esd_cnt = 0;

#if MMS_HAS_TOUCH_KEY
		if (tmp[0] & MMS_TOUCH_KEY_EVENT) {
			switch (tmp[0] & 0xf) {
			case 1:
				key_code = KEY_RECENT;
				break;
			case 2:
				key_code = KEY_BACK;
				break;
			default:
				tsp_debug_err(true, &client->dev, "Unknown key type\n");
				continue;
			}

			key_state = (tmp[0] & 0x80) ? 1 : 0;
			input_report_key(info->input_dev, key_code, key_state);
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
			tsp_debug_info(true, &client->dev, "key[%3d] is %s\n", key_code,
				key_state ? "pressed" : "releaseed");
#else
			tsp_debug_info(true, &client->dev,
				"key is %s\n", key_state ? "pressed" : "releaseed");
#endif

		} else
#endif
		{
			id = (tmp[0] & 0xf) - 1;
			x = tmp[2] | ((tmp[1] & 0xf) << 8);
			y = tmp[3] | (((tmp[1] >> 4 ) & 0xf) << 8);
//			major = tmp[4];
//			pressure = tmp[5];
//			palm = (tmp[0] & 0x10) >> 4;
			major_axis = tmp[6];
			minor_axis = tmp[7];

			input_mt_slot(info->input_dev, id);

			if (!(tmp[0] & 0x80)) {
				if (info->finger_state[id]) {
					info->finger_state[id] = false;
					input_mt_report_slot_state(info->input_dev,
						MT_TOOL_FINGER, false);

#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
					tsp_debug_info(true, &client->dev,
								"[R][%d]: X[%4d], Y[%4d], [%d] [0x%X/0x%X/0x%X//0x%X]\n",
									id, x, y, info->mcount[id], info->fw_ver_ic[0],
									info->fw_ver_ic[1], info->fw_ver_ic[2],
									info->fw_ver_ic_config);
#else
					tsp_debug_info(true, &client->dev, "[R][%d], [%d]\n", id, info->mcount[id]);
#endif
				}
				info->mcount[id] = 0;

			} else {
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, major_axis);
				input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, minor_axis);

				if(info->mcount[id] > 60000)
					info->mcount[id] = 60000;
				else
					info->mcount[id]++;

				if (!info->finger_state[id]) {
					info->finger_state[id] = true;
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
					tsp_debug_info(true, &client->dev,
						"[P][%d]: X[%4d], Y[%4d], Ma[%d], Mi[%d]\n",
						id, x, y, major_axis, minor_axis);
#else
					tsp_debug_info(true, &client->dev, "[P][%d]\n", id);
#endif
				}
			}
		}
	}

	input_sync(info->input_dev);

out:
	return;
}

/* mms_interrupt - interrupt thread */
static irqreturn_t mms_interrupt(int irq, void *dev_id)
{
	struct mms_info *info = (struct mms_info *)dev_id;
	struct i2c_client *client = info->client;
	u8 reg = MMS_EVENT_PKT;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &reg,
			.len = 1,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
		},
	};
	int ret, sz;
	int buf_len = (MAX_FINGER_NUM + info->key_num) * FINGER_EVENT_SZ;
	static u8 *buf = NULL;

	if (!buf) {
		buf = kzalloc(buf_len, GFP_KERNEL);
		if (!buf) {
			tsp_debug_err(true, &client->dev,
				"%s : Failed to allocate memory\n", __func__);
			buf = NULL;
			goto out;
		}
	}

	msg[1].buf = buf;

	sz = i2c_smbus_read_byte_data(client, MMS_EVENT_PKT_SZ);

	if (sz <= 0)
		return IRQ_HANDLED;

//	if (sz > buf_len || sz % FINGER_EVENT_SZ) {
//		tsp_debug_err(true, &client->dev, "%s : Failed to read event packet size(%d)\n",
//							__func__, sz);
//		goto out;
//	}

	msg[1].len = sz;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg))
		tsp_debug_err(true, &client->dev,
			"Failed to read %d bytes of touch data (%d)\n", sz, ret);
	else
		mms_report_input_data(info, sz, buf);

out:
	return IRQ_HANDLED;
}

static int mms_config(struct mms_info *info)
{
//	struct i2c_client *client = info->client;
	int ret = 0;

#if MMS_HAS_TOUCH_KEY
	ret = i2c_smbus_read_byte_data(client, MMS_KEY_NUM);
	if (ret <= 0) {
		tsp_debug_err(true, &client->dev, "Failed to read keynode num(%d)\n", ret);
		return ret;
	}

	info->key_num = ret;
	ret = 0;
#endif

#if INFORM_CHARGER
	m_info = info;
	mms_charger_status_cb(current_cable_type);
	tsp_charger_status_cb = mms_charger_status_cb;
#endif

	return ret;
}

/*
 * mms_isc_read_status - Check erase state function
 */
static int mms_isc_read_status(struct mms_info *info, u32 val)
{
	struct i2c_client *client = info->client;
	u8 cmd = ISC_CMD_READ_STATUS;
	u32 result = 0;
	int cnt = 100;
	int ret = 0;

	do {
		i2c_smbus_read_i2c_block_data(client, cmd, 4, (u8 *)&result);

		if (result == val)
			break;

		msleep(1);
	} while (--cnt);

	if (!cnt) {
		tsp_debug_err(true, &client->dev,
			"Failed to read status. cnt : %d, val : 0x%x != 0x%x\n",
			cnt, result, val);
	}

	return ret;
}

static int mms_isc_transfer_cmd(struct mms_info *info, int cmd)
{
	struct i2c_client *client = info->client;
	struct isc_packet pkt = { ISC_ADDR, cmd };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = sizeof(struct isc_packet),
		.buf = (u8 *)&pkt,
	};

	return i2c_transfer(client->adapter, &msg, 1) != 1;
}

/*
 * mms_isc_erase_page - ic page erase(1 kbyte)
 */
static int mms_isc_erase_page(struct mms_info *info, int page)
{
	return mms_isc_transfer_cmd(info, ISC_CMD_PAGE_ERASE | page) ||
		mms_isc_read_status(
			info, ISC_PAGE_ERASE_DONE | ISC_PAGE_ERASE_ENTER | page
		);
}

/*
 * mms_isc_enter - isc enter command
 */
static int mms_isc_enter(struct mms_info *info)
{
	return i2c_smbus_write_byte_data(info->client, MMS_CMD_ENTER_ISC, true);
}

static int mms_isc_exit(struct mms_info *info)
{
	return mms_isc_transfer_cmd(info, ISC_CMD_EXIT);
}

/*
 * mms_get_fw_version - f/w version read
 */
static int mms_get_fw_version(struct i2c_client *client, u8 *buf)
{
	u8 cmd = MMS_BL_VERSION;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = MAX_SECTION_NUM,
		},
	};

	return i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg))
				!= ARRAY_SIZE(msg);
}

static int mms_get_fw_version_ic(struct mms_info *info)
{
	u8 cmd = MMS_BL_VERSION;
	u8 buf[MAX_SECTION_NUM] = {0,0,0};
	int i = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = MAX_SECTION_NUM,
		},
	};

	i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));
	for(i = 0 ; i < MAX_SECTION_NUM ; i++)
		info->fw_ver_ic[i] = buf[i];

	tsp_debug_info(true, &info->client->dev, "%s : Version : [0x%X/0x%X/0x%X]\n",
							__func__, buf[0], buf[1], buf[2]);
	return 0;
}




/*
 * mms_flash_section - f/w section(boot,core,config) download func
 */
static int mms_flash_section(struct mms_info *info, struct mms_fw_img *img,
			const u8 *data)
{
	struct i2c_client *client = info->client;
	struct isc_packet *isc_packet;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = ISC_XFER_LEN,
		},
	};
	int ptr = img->offset;
	int ret;
	int page, i;
	u32 tmp;

	isc_packet = kzalloc(sizeof(struct isc_packet) + ISC_XFER_LEN, GFP_KERNEL);
	isc_packet->cmd = ISC_ADDR;

	msg[0].buf = (u8 *)isc_packet;
	msg[1].buf = kzalloc(ISC_XFER_LEN, GFP_KERNEL);

	for (page = img->start_page; page <= img->end_page; page++) {
		mms_isc_erase_page(info, page);

		for (i = 0; i < ISC_BLOCK_NUM; i++, ptr += ISC_XFER_LEN) {
			/* flash firmware */
			tmp = page * 256 + i * (ISC_XFER_LEN / 4);
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet) + ISC_XFER_LEN;

			memcpy(isc_packet->data, data + ptr, ISC_XFER_LEN);

			if (i2c_transfer(client->adapter, msg, 1) != 1)
				goto i2c_err;

			/* verify firmware */
			tmp |= ISC_CMD_READ;
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet);

			if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg))
					!= ARRAY_SIZE(msg))
				goto i2c_err;

			if (memcmp(isc_packet->data, msg[1].buf, ISC_XFER_LEN)) {
#if FLASH_VERBOSE_DEBUG
				print_hex_dump(KERN_ERR, "mms134s fw wr : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						isc_packet->data, ISC_XFER_LEN, false);

				print_hex_dump(KERN_ERR, "mms134s fw rd : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						msg[1].buf, ISC_XFER_LEN, false);
#endif
				tsp_debug_err(true, &client->dev, "Failed to flash verify\n");
				ret = -1;
				goto out;
			}
		}
	}

	tsp_debug_info(true, &client->dev, "Section [%d] update succeeded\n", img->type);

	ret = 0;
	goto out;

i2c_err:
	tsp_debug_err(true, &client->dev, "%s : i2c failed\n", __func__);
	ret = -1;

out:
	kfree(isc_packet);
	kfree(msg[1].buf);

	return ret;
}

/*
 * mms_flash_fw -flash_section of parent function
 * this function is IC version binary version compare and Download
 */
static int mms_flash_fw(const struct firmware *fw, struct mms_info *info,
						bool force)
{
	struct mms_bin_hdr *fw_hdr = (struct mms_bin_hdr *)fw->data;
	struct mms_fw_img **img;
	struct i2c_client *client = info->client;
	u8 ver[MAX_SECTION_NUM];
	u8 target[MAX_SECTION_NUM];
	int offset = sizeof(struct mms_bin_hdr);
	int retires;
	bool update_flag = false;
	bool isc_flag = true;
	int ret = 0;
	int i;

	if (fw == NULL) {
		tsp_debug_err(true, &client->dev, "%s : fw_data is NULL!", __func__);
		return 1;
	}

	img = kzalloc(sizeof(struct mms_fw_img *) * fw_hdr->section_num, GFP_KERNEL);

	if (mms_get_fw_version(client, ver)) {
		tsp_debug_err(true, &client->dev, "Failed to obtain ver. info\n");
		isc_flag = false;
		memset(ver, 0xff, sizeof(ver));
	}
	tsp_debug_info(true, &client->dev, "MMS-128S Before FW update : [0x%02x][0x%02x][0x%02x]\n",
						ver[SEC_BOOT], ver[SEC_CORE], ver[SEC_CONF]);

	retires = 3;
update_retry:
	for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw->data + offset);
		target[i] = img[i]->version;
		info->fw_ver_bin[i] = target[i];

		if (force || ver[img[i]->type] == 0xff || (ver[img[i]->type] < target[i])) {
			if(isc_flag){
				mms_isc_enter(info);
				isc_flag = false;
			}

			update_flag = true;
			tsp_debug_info(true, &client->dev,
				"Section [%d] is need to be updated. ver : 0x%x, bin : 0x%x\n",
				img[i]->type, ver[img[i]->type], target[i]);

			if (mms_flash_section(info, img[i], fw->data + fw_hdr->binary_offset)) {
				mms_reboot_hw(info);
				goto out;
			}
		} else {
			tsp_debug_info(true, &client->dev, "Section [%d] is up to date\n", i);
		}
	}

	if (update_flag) {
		mms_isc_exit(info);
		mms_reboot_hw(info);
	}

	if (mms_get_fw_version(client, ver)) {
		tsp_debug_err(true, &client->dev, "Failed to obtain version after flash\n");
		ret = -1;
		goto out;

	} else {
		/* Check fw version between ic and bin file */
		for (i = 0; i < fw_hdr->section_num; i++) {
			if (ver[img[i]->type] == 0xff || ver[img[i]->type] < target[i]) {
				tsp_debug_info(true, &client->dev,
					"Version mismatch after flash."
					" [%d] IC Ver 0x%x, FW Ver 0x%x\n",
					i, ver[img[i]->type], target[i]);

				if (retires-- < 0)
					goto update_retry;

				ret = -1;
				goto out;
			}
		}
	}

out:
	kfree(img);

	return ret;
}

#ifdef MMS_RUNTIME_CONF
static int mms_verify_config(struct mms_info *info,const u8 *buf,const u8 *tmp,int len)
{
	int count = 0;
	int ret = 0 ;
	if (memcmp(buf, tmp , len))
	{
		tsp_debug_info(true, &info->client->dev, "Run-time config verify failed\n");
		mms_reboot_hw(info);
		mms_config_set(info);
		ret = 1;
		count++;
	}

	if (count > 20){
		mms_reboot_hw(info);
		tsp_debug_info(true, &info->client->dev, "Run-time config failed\n");
		ret = 1;
	}
	return ret;
}

static int mms_read_config(struct i2c_client *client, u8 *buf, u8 *get_buf,int len)
{
	u8 cmd = MMS_GET_RUN_CONF;
	struct i2c_msg msg[3] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = buf,
			.len = 4,
		}, {
			.addr =client ->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = get_buf,
			.len = len,
		},
	};
	return (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg));
}

static int mms_config_flash(struct mms_info *info, const u8 *buf,const u8 len, char *text)
{
	struct i2c_client *client;
	struct i2c_msg msg;
	int ret;
	client = info->client;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = (u8 *)buf;
	msg.len = len;

	if(i2c_transfer(client->adapter, &msg,1) != 1){
		tsp_debug_err(true, &client->dev, "failed to transfer %s data\n",text);
		mms_reboot_hw(info);
		mms_config_set(info);
		ret = 0;
	} else {
		ret = 1;
	}
	return ret;
}

static int mms_config_start(struct mms_info *info)
{
	struct i2c_client *client;
	u8 mms_conf_buffer[4] = {MMS_UNIVERSAL_CMD, MMS_CMD_CONTROL, MMS_SUBCMD_START, RUN_START};
	client = info->client;

	tsp_debug_info(true, &client->dev, "runtime-config firmware update start!\n");
//	msleep(40); // Bring up check it!
	mms_config_flash(info, mms_conf_buffer, 4, "start-packit");
	return 0;
}
static int mms_config_finish(struct mms_info *info)
{
	struct i2c_client *client;
	u8 mms_conf_buffer[4] = {MMS_UNIVERSAL_CMD, MMS_CMD_CONTROL, MMS_SUBCMD_START, RUN_STOP};
	client = info->client;

	mms_config_flash(info, mms_conf_buffer, 4, "finish-packit" );
	tsp_debug_info(true, &client->dev, "succeed to get runtime-config firmware data.\n");
	return 0;
}

static int mms_config_get(struct mms_info *info, u8 mode)
{
	struct i2c_client *client = info->client;
	const struct firmware *fw;
	int ret = 0;

	mm_segment_t old_fs = {0};
	struct file *fp = NULL;
	long fsize = 0, nread = 0;

	if (mode == FW_FROM_BUILT_IN) {
		ret = request_firmware(&fw, info->pdata->fw_config_name, &client->dev);
		if (ret) {
			tsp_debug_err(true, &client->dev, "fail config request_firmware[%s]\n", info->pdata->fw_config_name);
			release_firmware(fw);
			return -1;
		}

		kfree(info->config_fw);
		info->config_fw = kzalloc((size_t)fw->size, GFP_KERNEL);
		memcpy((void *)info->config_fw, fw->data, fw->size);
		release_firmware(fw);
	}
	else if (mode == FW_FROM_UMS)
	{
		old_fs = get_fs();
		set_fs(get_ds());

		fp = filp_open(MMS_DEFAULT_UMS_FW_CONF, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			tsp_debug_err(true, &client->dev, "file %s open error!\n",
									MMS_DEFAULT_UMS_FW_CONF);
			return -1;
		} else {
			fsize = fp->f_path.dentry->d_inode->i_size;
			kfree(info->config_fw);
			info->config_fw = kzalloc((size_t)fsize, GFP_KERNEL);

			nread = vfs_read(fp, (char __user *)info->config_fw, fsize, &fp->f_pos);
			if (nread != fsize) {
				tsp_debug_err(true, &client->dev, "nread != fsize error\n");
			}

			filp_close(fp, current->files);
		}
		set_fs(old_fs);
	} 
	else
	{
		tsp_debug_err(true, &client->dev, "%s error mode[%d]\n", __func__, mode);
		return -1;
	}

	tsp_debug_info(true, &client->dev, "succeed to get runtime-config firmware\n");

	return 0;
}
static void mms_config_set(void *context)
{
	struct mms_info *info = context;
	struct i2c_client *client = info->client;
	struct mms_config_hdr	*conf_hdr;
	struct mms_config_item **conf_item;
	int offset;
	int offset_tmp = 0;
	int i;
	u8 config_flash_set[4];
	u8 config_flash_data[5];
	u8 *config_array;
	u8 flash_array[50];
	u8 cmp_data[30];

	if(info->config_fw== NULL) {
		tsp_debug_err(true, &client->dev, "failed to get config fw\n");
		return;
	}

	mms_config_start(info);

	conf_hdr = (struct mms_config_hdr *)info->config_fw;

	if ((conf_hdr->core_version & 0xff ) != info->fw_ver_ic[1]){
		mms_reboot_hw(info);
		tsp_debug_err(true, &client->dev, "mfsp-version is not correct : Bin:[0x%x 0x%x] :: IC:[0x%x 0x%x]\n",
			conf_hdr->core_version, conf_hdr->config_version& 0xff,info->fw_ver_ic[1],info->fw_ver_ic[2]);
		return;
	}

	if (conf_hdr->mark[3] != 0x02){
		mms_reboot_hw(info);
		tsp_debug_err(true, &client->dev, "failed to mfsp-version : %x \n",conf_hdr->mark[3]);
		return;
	}

	offset = conf_hdr->data_offset;
	conf_item = kzalloc(sizeof(*conf_item)*conf_hdr->data_count, GFP_KERNEL);

	for ( i=0 ; ; i++ , offset += MMS_MFSP_OFFSET)
	{
		conf_item[i] = (struct mms_config_item *)(info->config_fw + offset);

		if (i == MMS_GET_CONF_VER)
			tsp_debug_info(true, &info->client->dev, "Runtime Conf_Ver[%02x]\n", conf_item[i]->value);

		if(conf_item[i]->type == MMS_RUN_TYPE_INFO)
		{
			offset_tmp = conf_item[i]->data_blocksize;
			offset += offset_tmp;
		}

		if(conf_item[i]->type == MMS_RUN_TYPE_SINGLE)
		{
			config_flash_set[0] = MMS_RUN_CONF_POINTER;
			config_flash_set[1] = conf_item[i]->category;
			config_flash_set[2] = conf_item[i]->offset;
			config_flash_set[3] = conf_item[i]->datasize;

			mms_config_flash(info, config_flash_set,4,"config-set");
		}

		if(conf_item[i]->type == MMS_RUN_TYPE_ARRAY)
		{
			config_flash_set[0] = MMS_RUN_CONF_POINTER;
			config_flash_set[1] = conf_item[i]->category;
			config_flash_set[2] = conf_item[i]->offset;
			config_flash_set[3] = conf_item[i]->datasize;

			mms_config_flash(info,config_flash_set,4,"array-set");

            offset_tmp = conf_item[i]->data_blocksize;
            config_array =(u8 *)(info->config_fw + (offset + MMS_MFSP_OFFSET));
			offset += offset_tmp;

			flash_array[0] = MMS_SET_RUN_CONF;
			memcpy(&flash_array[1], config_array, conf_item[i]->datasize);

			mms_config_flash(info, flash_array, conf_item[i]->datasize + 1, "array_data");
			mms_read_config(client, config_flash_set, cmp_data, conf_item[i]->datasize);
			if (mms_verify_config(info, &flash_array[1], cmp_data, conf_item[i]->datasize)!=0)
			{
				break;
			}
		}

		config_flash_data[0] = MMS_SET_RUN_CONF;
		if(conf_item[i]->datasize == 1)
		{
			config_flash_data[1] = (u8)conf_item[i]->value;
			mms_config_flash(info, config_flash_data, 2, "config-data1");
			mms_read_config(client, config_flash_set, cmp_data, conf_item[i]->datasize);

			if (mms_verify_config(info, &config_flash_data[1], cmp_data, 1)!=0)
			{
			        break;
			}
		}
		else if(conf_item[i]->datasize == 2)
		{
			config_flash_data[1] = (u8)((conf_item[i]->value&0x00FF)>>0);
			config_flash_data[2] = (u8)((conf_item[i]->value&0xFF00)>>8);
			mms_config_flash(info,config_flash_data,3,"config-data2");
			mms_read_config(client, config_flash_set, cmp_data, conf_item[i]->datasize);
			if (mms_verify_config(info, &config_flash_data[1], cmp_data, 2)!=0)
			{
			        break;
			}
		}
		else if(conf_item[i]->datasize == 4)
		{
			config_flash_data[1] = (u8)((conf_item[i]->value&0x000000FF)>>0);
			config_flash_data[2] = (u8)((conf_item[i]->value&0x0000FF00)>>8);
			config_flash_data[3] = (u8)((conf_item[i]->value&0x00FF0000)>>16);
			config_flash_data[4] = (u8)((conf_item[i]->value&0xFF000000)>>24);
			mms_config_flash(info, config_flash_data, 5, "config-data4");
			mms_read_config(client, config_flash_set, cmp_data, conf_item[i]->datasize);
			if (mms_verify_config(info, &config_flash_data[1], cmp_data, 4)!=0)
			{
			        break;
			}
		}
		if(conf_item[i]->type == MMS_RUN_TYPE_END)
		{
			mms_config_finish(info);
			break;
		}
	}

	kfree(conf_item);

	info->fw_ver_ic_config = conf_hdr->config_version;
	tsp_debug_info(true, &client->dev, "mfsp-version is checked : 0x%x:0x%x:0x%x[0x%x]\n",
				info->fw_ver_ic[0], info->fw_ver_ic[1], info->fw_ver_ic[2], info->fw_ver_ic_config);

	return;
}

static void work_mms_config_set(struct work_struct *work)
{
	struct mms_info *info = container_of(work,
				struct mms_info, work_config_set.work);
	mms_config_set(info);

	enable_irq(info->irq);

	info->enabled = true;
	info->resume_done = true;
}
#endif

/*
 * mms_fw_update_controller - firmware check & update
 */
static int mms_fw_update_on_probe(struct mms_info *info)
{
	struct i2c_client *client = info->client;
	const struct firmware *fw;
	int retires = 3;
	int ret = 0;

	if (info->pdata->fw_name == NULL) {
		tsp_debug_err(true, &client->dev, "%s : TSP fw name is NULL, skip fw update.\n", __func__);
		goto out;
	}

	ret = request_firmware(&fw, info->pdata->fw_name, &client->dev);
	if (ret) {
		tsp_debug_err(true, &client->dev, "Failed to requesting firmware\n");
		goto out;
	}

	do {
		ret = mms_flash_fw(fw, info, COMPARE_UPDATE);
	} while (ret && --retires);

	if (!retires) {
		tsp_debug_err(true, &client->dev, "Failed to flash fw after retires\n");
		goto out;
	}

	info->fw = fw;

out:
	mms_get_fw_version_ic(info);
#ifdef MMS_RUNTIME_CONF
	/* Bring up : Support Runtime Config */
	mms_config_get(info, FW_FROM_BUILT_IN);
	mms_config_set(info);
#endif

	return ret;
}

#ifdef CONFIG_OF
static int mms_probe_dt(struct i2c_client *client, struct mms_platform_data *pdata)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	//Read property
	ret = of_property_read_u32(np, "melfas,max_x", &pdata->max_x);
	if (ret) {
		tsp_debug_err(true, dev, "Failed to read max_x\n");
		pdata->max_x = 1080;
	}

	ret = of_property_read_u32(np, "melfas,max_y", &pdata->max_y);
	if (ret) {
		tsp_debug_err(true, dev, "Failed to read max_y\n");
		pdata->max_y = 1920;
	}

	ret = of_property_read_u32(np, "melfas,max_finger", &pdata->max_finger_cnt);
	if (ret) {
		tsp_debug_err(true, dev, "Failed to read max_finger_cnt\n");
		pdata->max_finger_cnt = MAX_FINGER_NUM;
	}

	if (of_property_read_string(np, "melfas,regulator_dvdd", &pdata->regulator_dvdd)) {
		tsp_debug_err(true, dev, "Failed to get regulator_dvdd name property\n");
		return -EINVAL;
	}
	if (of_property_read_string(np, "melfas,regulator_avdd", &pdata->regulator_avdd)) {
		tsp_debug_err(true, dev, "Failed to get regulator_avdd name property\n");
		return -EINVAL;
	}

	pdata->gpio_int = of_get_named_gpio(np, "melfas,irq-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_int)) {
		tsp_debug_err(true, dev, "%s [ERROR] irq-gpio\n", __func__);
		return -1;
	}
	ret = gpio_request(pdata->gpio_int, "melfas,tsp_int");
	if (ret < 0){
		tsp_debug_err(true, dev, "%s [ERROR] gpio_request\n", __func__);
		return ret;
	}
	client->irq = gpio_to_irq(pdata->gpio_int);

	of_property_read_string(np, "melfas,fw_name", &pdata->fw_name);
	of_property_read_string(np, "melfas,fw_config_name", &pdata->fw_config_name);

	tsp_debug_info(true, dev, "%s: fw_name [%s], fw_config_name [%s]\n",
						__func__, pdata->fw_name, pdata->fw_config_name);

	if (of_property_read_u32(np, "melfas,device_num", &pdata->device_num))
		tsp_debug_err(true, dev, "Failed to get device_num property\n");

	tsp_debug_info(true, dev, "%s: max_x:%d max_y:%d int:%d irq:%d device_num:%d\n",
						__func__, pdata->max_x, pdata->max_y,pdata->gpio_int, client->irq, pdata->device_num);
	return ret;
}
#endif

#ifdef SEC_TSP_FACTORY_TEST
#include "mms100s_sec.c"
#endif

static int mms_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mms_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct input_dev *input_dev;
	struct mms_info *info;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		tsp_debug_err(true, &client->dev, "Not compatible i2c function\n");
		return -EIO;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		pdata = kzalloc(sizeof(struct mms_platform_data), GFP_KERNEL);
		if (!pdata) {
			tsp_debug_err(true, &client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
		ret = mms_probe_dt(client, pdata);
		if (ret < 0) {
			tsp_debug_err(true, &client->dev, "Failed to get platform data from node\n");
			goto err_probe_dt;
		}
	}
#else
	if (!pdata) {
		tsp_debug_err(true, &client->dev, "Platform data not found\n");
		return -EINVAL;
	}
#endif

	info = kzalloc(sizeof(struct mms_info), GFP_KERNEL);
	if (!info) {
		tsp_debug_err(true, &client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_info_alloc;
	}

	info->client = client;
	info->pdata = pdata;

	mms_power_control(info, 1);
	msleep(MMS_HW_INIT_TIME);

	ret = mms_fw_update_on_probe(info);
	if (ret) {
		tsp_debug_err(true, &client->dev, "Failed to firmware update (%d)\n", ret);
		goto err_power;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		tsp_debug_err(true, &client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_power;
	}

	info->input_dev = input_dev;

	snprintf(info->phys, sizeof(info->phys), "%s/input0",
				dev_name(&client->dev));
	/* Only for U model */
	input_dev->name = "sec_touchscreen_ex";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

#ifdef USE_OPEN_CLOSE
	input_dev->open = mms_input_open;
	input_dev->close = mms_input_close;
#endif

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if MMS_HAS_TOUCH_KEY
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_RECENT, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
#endif

	INIT_DELAYED_WORK(&info->work_config_set, work_mms_config_set);
	mutex_init(&info->device_mutex);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->pdata->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_MAJOR, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, MAX_MINOR, 0, 0);

	input_set_drvdata(input_dev, info);
	i2c_set_clientdata(client, info);

	ret = input_register_device(input_dev);
	if (ret) {
		tsp_debug_err(true, &client->dev, "Failed to register input dev(%d)\n", ret);
		goto err_input_register;
	}

	info->irq = client->irq;

	ret = mms_config(info);
	if (ret) {
		tsp_debug_err(true, &client->dev, "Failed mms134 configuration\n");
		goto err_mms_config;
	}

	ret = request_threaded_irq(info->irq, NULL, mms_interrupt,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, "mms_ts", info);

	if (ret) {
		tsp_debug_err(true, &client->dev, "Failed to register irq(%d)\n", ret);
		goto err_request_irq;
	}

	info->enabled = true;
	info->resume_done = true;

#ifdef SEC_TSP_FACTORY_TEST
	ret = init_sec(info);
	if (ret)
		tsp_debug_err(true, &client->dev, "Failed to initialize SEC_FACTORY\n");
#endif

	tsp_debug_info(true, &client->dev, "mms100s initialized\n");

	return 0;

err_mms_config:
	free_irq(info->irq, info);
err_request_irq:
	input_unregister_device(input_dev);
err_input_register:
	input_free_device(input_dev);
err_power:
	kfree(info);
err_info_alloc:
#ifdef CONFIG_OF
err_probe_dt:
	kfree(pdata);
#endif
	tsp_debug_info(true, &client->dev, "Failed to initialization\n");

	return ret;
}

static void mms_shutdown(struct i2c_client *client)
{
	struct mms_info *info = i2c_get_clientdata(client);

	tsp_debug_info(true, &client->dev, "%s\n", __func__);

	mms_disable(info);
}

static int mms_remove(struct i2c_client *client)
{
	struct mms_info *info = i2c_get_clientdata(client);

	free_irq(info->irq, info);
	kfree(info->finger_state);
	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
#ifdef SEC_TSP_FACTORY_TEST
	destroy_sec(info);
#endif
#ifdef CONIFG_OF
	kfree(info->pdata);
#endif
	kfree(info);

	return 0;
}

#ifdef USE_OPEN_CLOSE
static void mms_input_close(struct input_dev *dev)
{
	struct mms_info *info = input_get_drvdata(dev);

	mutex_lock(&info->device_mutex);

	mms_clear_input_data(info);
	mms_disable(info);

	mutex_unlock(&info->device_mutex);
}

static int mms_input_open(struct input_dev *dev)
{
	struct mms_info *info = input_get_drvdata(dev);

	mutex_lock(&info->device_mutex);

	mms_enable(info);

	mutex_unlock(&info->device_mutex);

	return 0;
}
#endif

static struct i2c_device_id mms_id[] = {
	{MMS_DEV_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, mms_id);

#ifdef CONFIG_OF
static struct of_device_id mms_dt_ids[] = {
	{ .compatible = "melfas,mms_ts", },
	{}
};
#else
#define mms_dt_ids NULL
#endif

static struct i2c_driver mms_driver = {
	.id_table	= mms_id,
	.probe		= mms_probe,
	.shutdown	= mms_shutdown,
	.remove		= mms_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= MMS_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table	= mms_dt_ids,
#endif
	},
};

//module_i2c_driver(mms_driver);

static int __init mms_driver_init(void)
{
	return i2c_add_driver(&mms_driver);
}

static void __exit mms_driver_exit(void)
{
	i2c_del_driver(&mms_driver);
}


module_init(mms_driver_init);
module_exit(mms_driver_exit);


MODULE_DESCRIPTION("Touchscreen driver for MELFAS MMS134S");
MODULE_LICENSE("GPL");
