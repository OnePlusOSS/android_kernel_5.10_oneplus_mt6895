/*
 * HID over I2C protocol implementation
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * This code is partly based on "USB HID support for Linux":
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include "../hid-ids.h"
#include "i2c-hid.h"
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <soc/oplus/system/oplus_project.h>
#include <soc/oplus/system/kernel_fb.h>

/* debug option */
static bool debug_log = 0;
bool kb_connected = false;
#define TP_PRINT_POINT_NUM 100
int sn_report_count = 0;
EXPORT_SYMBOL(kb_connected);

static DEFINE_MUTEX(desc_lock);

#define KB_CON_PAC_ID 0x06
#define KB_BUTTON_ID 0x03
#define KB_TOUCHPAD_ID 0x04

#define KB_SN_HIDE_BIT_START  2
#define KB_SN_HIDE_SEVENTEEN_END  9
#define KB_SN_HIDE_TWENTY_TWO_END 14
#define KB_SN_HIDE_STAR_ASCII  42

#define i2c_hid_err(ihid, fmt, arg...)					  \
do {									  \
	dev_printk(KERN_ERR, &(ihid)->client->dev, fmt, ##arg); \
} while (0)

#define i2c_hid_dbg(ihid, fmt, arg...)					  \
do {									  \
	if (debug_log)							  \
		dev_printk(KERN_ERR, &(ihid)->client->dev, fmt, ##arg); \
} while (0)

#define I2C_HID_CMD(opcode_) \
	.opcode = opcode_, .length = 4, \
	.registerIndex = offsetof(struct i2c_hid_desc, wCommandRegister)

/* fetch HID descriptor */
static const struct i2c_hid_cmd hid_descr_cmd = { .length = 2 };
/* fetch report descriptors */
static const struct i2c_hid_cmd hid_report_descr_cmd = {
		.registerIndex = offsetof(struct i2c_hid_desc,
			wReportDescRegister),
		.opcode = 0x00,
		.length = 2 };
/* commands */
static const struct i2c_hid_cmd hid_reset_cmd =		{ I2C_HID_CMD(0x01),
							  .wait = true };
static const struct i2c_hid_cmd hid_get_report_cmd =	{ I2C_HID_CMD(0x02) };
static const struct i2c_hid_cmd hid_set_report_cmd =	{ I2C_HID_CMD(0x03) };
static const struct i2c_hid_cmd hid_set_power_cmd =	{ I2C_HID_CMD(0x08) };
static const struct i2c_hid_cmd hid_no_cmd =		{ .length = 0 };

static bool Check_kpdmcu_fw_update(struct i2c_hid *ihid);
static bool Check_touchmcu_fw_update(struct i2c_hid *ihid);
struct i2c_hid *fw_ihid;
struct i2c_hid *keyboard_fw_ihid;

/*
 * i2c_hid_lookup_quirk: return any quirks associated with a I2C HID device
 * @idVendor: the 16-bit vendor ID
 * @idProduct: the 16-bit product ID
 *
 * Returns: a u32 quirks value.
 */
static u32 i2c_hid_lookup_quirk(const u16 idVendor, const u16 idProduct)
{
	u32 quirks = 0;
	int n;

	for (n = 0; i2c_hid_quirks[n].idVendor; n++)
		if (i2c_hid_quirks[n].idVendor == idVendor &&
		    (i2c_hid_quirks[n].idProduct == (__u16)HID_ANY_ID ||
		     i2c_hid_quirks[n].idProduct == idProduct))
			quirks = i2c_hid_quirks[n].quirks;

	return quirks;
}

static int hid_fw_update_cmd_send_recv(struct i2c_client *client,
		u16 addr, u8 *send_buf, u32 send_buf_len,
		u8 *recv_buf, u32 recv_buf_len)
{
	int ret;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct i2c_msg msg[2];
	int send_msg_num = 1;

	i2c_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, send_buf_len, send_buf);
	msg[0].addr = addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = send_buf_len;
	msg[0].buf = send_buf;
	if (recv_buf_len > 0) {
		msg[1].addr = addr;
		msg[1].flags = client->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = recv_buf_len;
		msg[1].buf = recv_buf;
		send_msg_num = 2;
		set_bit(I2C_HID_READ_PENDING, &ihid->flags);
	}

	ret = i2c_transfer(client->adapter, msg, send_msg_num);
	if (ret != send_msg_num) {
		msleep(10);
		ret = i2c_transfer(client->adapter, msg, send_msg_num);
	}

	if (recv_buf_len > 0)
		clear_bit(I2C_HID_READ_PENDING, &ihid->flags);

	if (ret != send_msg_num) {
		dev_err(&client->dev, "write fw_update_cmd fail\n");
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int __i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command, u8 reportID,
		u8 reportType, u8 *args, int args_len,
		unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	union command *cmd = (union command *)ihid->cmdbuf;
	int ret;
	struct i2c_msg msg[2];
	int msg_num = 1;

	int length = command->length;
	bool wait = command->wait;
	unsigned int registerIndex = command->registerIndex;

	/* special case for hid_descr_cmd */
	if (command == &hid_descr_cmd) {
		cmd->c.reg = ihid->wHIDDescRegister;
	} else {
		cmd->data[0] = ihid->hdesc_buffer[registerIndex];
		cmd->data[1] = ihid->hdesc_buffer[registerIndex + 1];
	}

	if (length > 2) {
		cmd->c.opcode = command->opcode;
		cmd->c.reportTypeID = reportID | reportType << 4;
	}

	memcpy(cmd->data + length, args, args_len);
	length += args_len;

	i2c_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, length, cmd->data);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = cmd->data;
	if (data_len > 0) {
		msg[1].addr = client->addr;
		msg[1].flags = client->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = data_len;
		msg[1].buf = buf_recv;
		msg_num = 2;
		set_bit(I2C_HID_READ_PENDING, &ihid->flags);
	}

	if (wait)
		set_bit(I2C_HID_RESET_PENDING, &ihid->flags);

	ret = i2c_transfer(client->adapter, msg, msg_num);
	if (ret != msg_num) {
		msleep(10);
		ret = i2c_transfer(client->adapter, msg, msg_num);
	}

	if (data_len > 0)
		clear_bit(I2C_HID_READ_PENDING, &ihid->flags);

	if (ret != msg_num)
		return ret < 0 ? ret : -EIO;

	ret = 0;

	if (wait && (ihid->quirks & I2C_HID_QUIRK_NO_IRQ_AFTER_RESET)) {
		msleep(100);
	} else if (wait) {
		i2c_hid_dbg(ihid, "%s: waiting...\n", __func__);
		if (!wait_event_timeout(ihid->wait, !test_bit(I2C_HID_RESET_PENDING, &ihid->flags), msecs_to_jiffies(5000)))
			ret = -ENODATA;
		i2c_hid_dbg(ihid, "%s: finished.\n", __func__);
	}

	return ret;
}

static int i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command,
		unsigned char *buf_recv, int data_len)
{
	return __i2c_hid_command(client, command, 0, 0, NULL, 0, buf_recv, data_len);
}

static int i2c_hid_get_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 args[3];
	int ret;
	int args_len = 0;
	u16 readRegister = le16_to_cpu(ihid->hdesc.wDataRegister);

	i2c_hid_dbg(ihid, "%s reportType:%d reportID:%d\n", __func__, reportType, reportID);

	if (reportID >= 0x0F) {
		args[args_len++] = reportID;
		reportID = 0x0F;
	}

	args[args_len++] = readRegister & 0xFF;
	args[args_len++] = readRegister >> 8;

	ret = __i2c_hid_command(client, &hid_get_report_cmd, reportID,
		reportType, args, args_len, buf_recv, data_len);
	if (ret) {
		dev_err(&client->dev, "failed to retrieve report from device.\n");
		return ret;
	}

	return 0;
}

/**
 * i2c_hid_set_or_send_report: forward an incoming report to the device
 * @client: the i2c_client of the device
 * @reportType: 0x03 for HID_FEATURE_REPORT ; 0x02 for HID_OUTPUT_REPORT
 * @reportID: the report ID
 * @buf: the actual data to transfer, without the report ID
 * @data_len: size of buf
 * @use_data: true: use SET_REPORT HID command, false: send plain OUTPUT report
 */
static int i2c_hid_set_or_send_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf, size_t data_len, bool use_data)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 *args = ihid->argsbuf;
	const struct i2c_hid_cmd *hidcmd;
	int ret;
	u16 dataRegister = le16_to_cpu(ihid->hdesc.wDataRegister);
	u16 outputRegister = le16_to_cpu(ihid->hdesc.wOutputRegister);
	u16 maxOutputLength = le16_to_cpu(ihid->hdesc.wMaxOutputLength);
	u16 size;
	int args_len;
	int index = 0;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (data_len > ihid->bufsize)
		return -EINVAL;

	size =		2			/* size */ +
			(reportID ? 1 : 0)	/* reportID */ +
			data_len		/* buf */;
	args_len =	(reportID >= 0x0F ? 1 : 0) /* optional third byte */ +
			2			/* dataRegister */ +
			size			/* args */;

	if (!use_data && maxOutputLength == 0)
		return -ENOSYS;

	if (reportID >= 0x0F) {
		args[index++] = reportID;
		reportID = 0x0F;
	}

	/*
	 * use the data register for feature reports or if the device does not
	 * support the output register
	 */
	if (use_data) {
		args[index++] = dataRegister & 0xFF;
		args[index++] = dataRegister >> 8;
		hidcmd = &hid_set_report_cmd;
	} else {
		args[index++] = outputRegister & 0xFF;
		args[index++] = outputRegister >> 8;
		hidcmd = &hid_no_cmd;
	}

	args[index++] = size & 0xFF;
	args[index++] = size >> 8;

	if (reportID)
		args[index++] = reportID;

	memcpy(&args[index], buf, data_len);

	ret = __i2c_hid_command(client, hidcmd, reportID,
		reportType, args, args_len, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to set a report to device.\n");
		return ret;
	}

	return data_len;
}

static int i2c_hid_set_power(struct i2c_client *client, int power_state)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_err(ihid, "%s power_state:%d\n", __func__, power_state);

	/*
	 * Some devices require to send a command to wakeup before power on.
	 * The call will get a return value (EREMOTEIO) but device will be
	 * triggered and activated. After that, it goes like a normal device.
	 */
	if (power_state == I2C_HID_PWR_ON &&
	    ihid->quirks & I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV) {
		ret = i2c_hid_command(client, &hid_set_power_cmd, NULL, 0);

		/* Device was already activated */
		if (!ret)
			goto set_pwr_exit;
	}

	ret = __i2c_hid_command(client, &hid_set_power_cmd, power_state,
		0, NULL, 0, NULL, 0);

	if (ret)
		dev_err(&client->dev, "failed to change power setting.\n");

set_pwr_exit:

	/*
	 * The HID over I2C specification states that if a DEVICE needs time
	 * after the PWR_ON request, it should utilise CLOCK stretching.
	 * However, it has been observered that the Windows driver provides a
	 * 1ms sleep between the PWR_ON and RESET requests.
	 * According to Goodix Windows even waits 60 ms after (other?)
	 * PWR_ON requests. Testing has confirmed that several devices
	 * will not work properly without a delay after a PWR_ON request.
	 */
	if (!ret && power_state == I2C_HID_PWR_ON)
		msleep(10);

	return ret;
}

static int i2c_hid_hwreset(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	/*
	 * This prevents sending feature reports while the device is
	 * being reset. Otherwise we may lose the reset complete
	 * interrupt.
	 */
	mutex_lock(&ihid->reset_lock);

	ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);
	if (ret)
		goto out_unlock;

	i2c_hid_err(ihid, "already powered on, resetting...\n");

	ret = i2c_hid_command(client, &hid_reset_cmd, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to reset device.\n");
		i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
		goto out_unlock;
	}

	/* At least some SIS devices need this after reset */
	if (!(ihid->quirks & I2C_HID_QUIRK_NO_WAKEUP_AFTER_RESET))
		ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);

out_unlock:
	mutex_unlock(&ihid->reset_lock);
	return ret;
}

static void i2c_hid_get_input(struct i2c_hid *ihid)
{
	int ret;
	u32 ret_size;
	static int point_count = 0;
	int size = le16_to_cpu(ihid->hdesc.wMaxInputLength);
	u8 keyboard_brand;
	u8 touch_status;
	u8 brand_mask = KEYBOARD_BRAND_MASK;
	u8 touch_mask = KEYBOARD_TOUCH_MASK;

	if (size > ihid->bufsize)
		size = ihid->bufsize;

	ret = i2c_master_recv(ihid->client, ihid->inbuf, size);
	if (ret != size) {
		if (ret < 0)
			return;

		dev_err(&ihid->client->dev, "%s: got %d data instead of %d\n", __func__, ret, size);
		return;
	}

	ret_size = ihid->inbuf[0] | ihid->inbuf[1] << 8;

	if (!ret_size) {
		/* host or device initiated RESET completed */
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &ihid->flags))
			wake_up(&ihid->wait);
		return;
	}

	if (ihid->quirks & I2C_HID_QUIRK_BOGUS_IRQ && ret_size == 0xffff) {
		dev_err(&ihid->client->dev, "%s: IRQ triggered but there's no data\n", __func__);
		return;
	}

	if ((ret_size > size) || (ret_size < 2)) {
		if (ihid->quirks & I2C_HID_QUIRK_BAD_INPUT_SIZE) {
			ihid->inbuf[0] = size & 0xff;
			ihid->inbuf[1] = size >> 8;
			ret_size = size;
		} else {
			dev_err(&ihid->client->dev, "%s: incomplete report (%d/%d)\n", __func__, size, ret_size);
			return;
		}
	}

	i2c_hid_dbg(ihid, "input: %*ph\n", ret_size, ihid->inbuf);

	if (ihid->inbuf[2] == KB_CON_PAC_ID) {
		if (ihid->inbuf[3] & 0x1) {
			/* bit6: touch status, 0:enabled 1:disabled */
			touch_status = (ihid->inbuf[3] & touch_mask) >> KEYBOARD_TOUCH_BIT;
			fw_ihid->keyboard_touch_status = touch_status;
			if (!kb_connected) {
				if(fw_ihid) {
					keyboard_brand = (ihid->inbuf[3] & brand_mask) >> KEYBOARD_BRAND_SHIFT;
					if(keyboard_brand == KEYBOARD_BRAND_ONEPLUE) {
						fw_ihid->is_oneplus_keyboard_or_not = true;
						fw_ihid->kpdmcu_fw_data_ver = ihid->kpdmcu_fw_data_version[1];
						fw_ihid->kpdmcu_fw_cnt = ihid->kpdmcu_fw_count[1];
						snprintf(ihid->hid->name, sizeof(ihid->hid->name), "%s", ihid->keyboard_dev_name[1]);
						dev_info(&ihid->client->dev, "%s:keyboard brand is oneplus\n", __func__);
					} else {
						fw_ihid->is_oneplus_keyboard_or_not = false;
						fw_ihid->kpdmcu_fw_data_ver = ihid->kpdmcu_fw_data_version[0];
						fw_ihid->kpdmcu_fw_cnt = ihid->kpdmcu_fw_count[0];
						snprintf(ihid->hid->name, sizeof(ihid->hid->name), "%s", ihid->keyboard_dev_name[0]);
						dev_info(&ihid->client->dev, "%s:keyboard brand is oplus\n", __func__);
					}
				}
				if (ihid->hid->driver == NULL) {
					dev_err(&ihid->client->dev, "%s: ihid->driver is NULL, kb_status = %#x, kb_connected = %d...\n", __func__, ihid->inbuf[3], kb_connected);
					return;
				}
				ret = hid_hw_start(ihid->hid, HID_CONNECT_HIDINPUT);
				if (ret) {
					dev_err(&ihid->client->dev, "%s: keyboard detected,register input dev fail\n", __func__);
					return;
				} else {
					kb_connected = true;
					dev_info(&ihid->client->dev, "%s:keyboard detected,register input dev success. touch status [%u]\n", __func__, touch_status);
					schedule_work(&ihid->kpdmcu_fw_mcu_version_work);
				}
			} else {
				dev_info(&ihid->client->dev, "%s: keyboard detected again. touch status [%u]\n", __func__, touch_status);
			}
		} else {
			if (kb_connected) {
				hid_hw_stop(ihid->hid);
				kb_connected = false;
				dev_info(&ihid->client->dev, "%s: keyboard disconnect,unregiter input dev\n", __func__);
				cancel_work_sync(&ihid->kpdmcu_fw_mcu_version_work);
			} else {
				dev_info(&ihid->client->dev, "%s: keyboard disconnect again\n", __func__);
			}
                }
	}

	if (test_bit(I2C_HID_STARTED, &ihid->flags))
		hid_input_report(ihid->hid, HID_INPUT_REPORT, ihid->inbuf + 2, ret_size - 2, 1);

	if (((ihid->inbuf[2] == KB_BUTTON_ID) || (ihid->inbuf[2] == KB_TOUCHPAD_ID)) && (point_count++ == TP_PRINT_POINT_NUM)) {
		i2c_hid_err(ihid, "input: %*ph\n", ret_size, ihid->inbuf);
		point_count = 0;
	}

	return;
}

static irqreturn_t i2c_hid_irq(int irq, void *dev_id)
{
	struct i2c_hid *ihid = dev_id;

	if(ihid == NULL)
		return IRQ_HANDLED;

	if (test_bit(I2C_HID_READ_PENDING, &ihid->flags))
		return IRQ_HANDLED;

	i2c_hid_get_input(ihid);

	return IRQ_HANDLED;
}

static int i2c_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
		report->device->report_enum[report->type].numbered + 2;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void i2c_hid_find_max_report(struct hid_device *hid, unsigned int type,
		unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	/* We should not rely on wMaxInputLength, as some devices may set it to
	 * a wrong length. */
	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = i2c_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

static void i2c_hid_free_buffers(struct i2c_hid *ihid)
{
	kfree(ihid->inbuf);
	kfree(ihid->rawbuf);
	kfree(ihid->argsbuf);
	kfree(ihid->cmdbuf);
	ihid->inbuf = NULL;
	ihid->rawbuf = NULL;
	ihid->cmdbuf = NULL;
	ihid->argsbuf = NULL;
	ihid->bufsize = 0;
}

static int i2c_hid_alloc_buffers(struct i2c_hid *ihid, size_t report_size)
{
	/* the worst case is computed from the set_report command with a
	 * reportID > 15 and the maximum report length */
	int args_len = sizeof(__u8) + /* ReportID */
		       sizeof(__u8) + /* optional ReportID byte */
		       sizeof(__u16) + /* data register */
		       sizeof(__u16) + /* size of the report */
		       report_size; /* report */

	ihid->inbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->rawbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->argsbuf = kzalloc(args_len, GFP_KERNEL);
	ihid->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

	if (!ihid->inbuf || !ihid->rawbuf || !ihid->argsbuf || !ihid->cmdbuf) {
		i2c_hid_free_buffers(ihid);
		return -ENOMEM;
	}

	ihid->bufsize = report_size;

	return 0;
}

static int i2c_hid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	size_t ret_count, ask_count;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	/*
	 * In case of unnumbered reports the response from the device will
	 * not have the report ID that the upper layers expect, so we need
	 * to stash it the buffer ourselves and adjust the data size.
	 */
	if (!report_number) {
		buf[0] = 0;
		buf++;
		count--;
	}

	/* +2 bytes to include the size of the reply in the query buffer */
	ask_count = min(count + 2, (size_t)ihid->bufsize);

	ret = i2c_hid_get_report(client,
			report_type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report_number, ihid->rawbuf, ask_count);

	if (ret < 0)
		return ret;

	ret_count = ihid->rawbuf[0] | (ihid->rawbuf[1] << 8);

	if (ret_count <= 2)
		return 0;

	ret_count = min(ret_count, ask_count);

	/* The query buffer contains the size, dropping it in the reply */
	count = min(count, ret_count - 2);
	memcpy(buf, ihid->rawbuf + 2, count);

	if (!report_number)
		count++;

	return count;
}

static int i2c_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
		size_t count, unsigned char report_type, bool use_data)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;

	mutex_lock(&ihid->reset_lock);

	/*
	 * Note that both numbered and unnumbered reports passed here
	 * are supposed to have report ID stored in the 1st byte of the
	 * buffer, so we strip it off unconditionally before passing payload
	 * to i2c_hid_set_or_send_report which takes care of encoding
	 * everything properly.
	 */
	ret = i2c_hid_set_or_send_report(client,
				report_type == HID_FEATURE_REPORT ? 0x03 : 0x02,
				report_id, buf + 1, count - 1, use_data);

	if (ret >= 0)
		ret++; /* add report_id to the number of transferred bytes */

	mutex_unlock(&ihid->reset_lock);

	return ret;
}

static int i2c_hid_output_report(struct hid_device *hid, __u8 *buf,
		size_t count)
{
	return i2c_hid_output_raw_report(hid, buf, count, HID_OUTPUT_REPORT,
			false);
}

static int i2c_hid_raw_request(struct hid_device *hid, unsigned char reportnum,
			       __u8 *buf, size_t len, unsigned char rtype,
			       int reqtype)
{
	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		return i2c_hid_get_raw_report(hid, reportnum, buf, len, rtype);
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		return i2c_hid_output_raw_report(hid, buf, len, rtype, true);
	default:
		return -EIO;
	}
}

static int i2c_hid_parse(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int rsize;
	char *rdesc;
	int ret;
	int tries = 3;
	char *use_override;

	i2c_hid_err(ihid, "entering %s\n", __func__);

	rsize = le16_to_cpu(hdesc->wReportDescLength);
	if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
		dbg_hid("%s : weird size of report descriptor (%u)\n", __func__, rsize);
		return -EINVAL;
	}

	do {
		ret = i2c_hid_hwreset(client);
		if (ret)
			msleep(1000);
	} while (tries-- > 0 && ret);

	if (ret) {
		i2c_hid_err(ihid, "%s: i2c_hid_hwreset failed.\n", __func__);
		return ret;
	}

	use_override = i2c_hid_get_dmi_hid_report_desc_override(client->name,
								&rsize);

	if (use_override) {
		rdesc = use_override;
		i2c_hid_err(ihid, "%s: Using a HID report descriptor override\n", __func__);
	} else {
		rdesc = kzalloc(rsize, GFP_KERNEL);

		if (!rdesc) {
			dbg_hid("%s: couldn't allocate rdesc memory\n", __func__);
			return -ENOMEM;
		}

		i2c_hid_err(ihid, "%s: asking HID report descriptor\n", __func__);

		ret = i2c_hid_command(client, &hid_report_descr_cmd,
				      rdesc, rsize);
		if (ret) {
			hid_err(hid, "%s: reading report descriptor failed\n", __func__);
			kfree(rdesc);
			return -EIO;
		}
	}

	i2c_hid_dbg(ihid, "Report Descriptor: %*ph\n", rsize, rdesc);

	ret = hid_parse_report(hid, rdesc, rsize);
	if (!use_override)
		kfree(rdesc);

	if (ret) {
		dbg_hid("%s: parsing report descriptor failed\n", __func__);
		return ret;
	}

	return 0;
}

static int i2c_hid_start(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;
	unsigned int bufsize = HID_MIN_BUFFER_SIZE;

	i2c_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

	if (bufsize > ihid->bufsize) {
		disable_irq(client->irq);
		i2c_hid_free_buffers(ihid);

		ret = i2c_hid_alloc_buffers(ihid, bufsize);
		enable_irq(client->irq);

		if (ret)
			return ret;
	}

	return 0;
}

static void i2c_hid_stop(struct hid_device *hid)
{
	hid->claimed = 0;
}

static int i2c_hid_open(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	set_bit(I2C_HID_STARTED, &ihid->flags);
	return 0;
}

static void i2c_hid_close(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	clear_bit(I2C_HID_STARTED, &ihid->flags);
}

struct hid_ll_driver i2c_hid_ll_driver = {
	.parse = i2c_hid_parse,
	.start = i2c_hid_start,
	.stop = i2c_hid_stop,
	.open = i2c_hid_open,
	.close = i2c_hid_close,
	.output_report = i2c_hid_output_report,
	.raw_request = i2c_hid_raw_request,
};
EXPORT_SYMBOL_GPL(i2c_hid_ll_driver);

static int i2c_hid_init_irq(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	unsigned long irqflags = 0;
	int ret;

	dev_dbg(&client->dev, "%s: Requesting IRQ: %d\n", __func__, client->irq);

	if (!irq_get_trigger_type(client->irq))
		irqflags = IRQF_TRIGGER_LOW;

	ret = request_threaded_irq(client->irq, NULL, i2c_hid_irq,
				   irqflags | IRQF_ONESHOT, client->name, ihid);
	if (ret < 0) {
		dev_warn(&client->dev,
			"Could not register for %s interrupt, irq = %d,"
			" ret = %d\n",
			client->name, client->irq, ret);

		return ret;
	}

	return 0;
}

static int i2c_hid_fetch_hid_descriptor(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int dsize;
	int ret;

	/* i2c hid fetch using a fixed descriptor size (30 bytes) */
	if (i2c_hid_get_dmi_i2c_hid_desc_override(client->name)) {
		i2c_hid_dbg(ihid, "Using a HID descriptor override\n");
		ihid->hdesc =
			*i2c_hid_get_dmi_i2c_hid_desc_override(client->name);
	} else {
		i2c_hid_err(ihid, "%s: Fetching the HID descriptor\n", __func__);
		ret = i2c_hid_command(client, &hid_descr_cmd,
				      ihid->hdesc_buffer,
				      sizeof(struct i2c_hid_desc));
		if (ret) {
			dev_err(&client->dev, "%s: hid_descr_cmd failed\n", __func__);
			return -ENODEV;
		}
	}

	/* Validate the length of HID descriptor, the 4 first bytes:
	 * bytes 0-1 -> length
	 * bytes 2-3 -> bcdVersion (has to be 1.00) */
	/* check bcdVersion == 1.0 */
	if (le16_to_cpu(hdesc->bcdVersion) != 0x0100) {
		dev_err(&client->dev,
			"unexpected HID descriptor bcdVersion (0x%04hx)\n",
			le16_to_cpu(hdesc->bcdVersion));
		return -ENODEV;
	}

	/* Descriptor length should be 30 bytes as per the specification */
	dsize = le16_to_cpu(hdesc->wHIDDescLength);
	if (dsize != sizeof(struct i2c_hid_desc)) {
		dev_err(&client->dev, "weird size of HID descriptor (%u)\n",
			dsize);
		return -ENODEV;
	}
	i2c_hid_dbg(ihid, "HID Descriptor: %*ph\n", dsize, ihid->hdesc_buffer);
	return 0;
}

static int i2c_hid_of_probe(struct i2c_client *client, struct i2c_hid_platform_data *pdata)
{
	struct device *dev = &client->dev;
	u32 val;
	int ret;

	ret = of_property_read_u32(dev->of_node, "hid-descr-addr", &val);
	if (ret) {
		dev_err(&client->dev, "%s HID register address not provided\n", __func__);
		return -ENODEV;
	}
	if (val >> 16) {
		dev_err(&client->dev, "%s Bad HID register address: 0x%08x\n", __func__, val);
		return -EINVAL;
	}
	pdata->hid_descriptor_address = val;

	return 0;
}

static const struct of_device_id i2c_hid_of_match[] = {
	{ .compatible = "hid-over-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_hid_of_match);

static void i2c_hid_fwnode_probe(struct i2c_client *client, struct i2c_hid_platform_data *pdata)
{
	u32 val;
	if (!device_property_read_u32(&client->dev, "post-power-on-delay-ms",
				      &val))
		pdata->post_power_delay_ms = val;
}

static void hid_pogopin_parse_dts(struct i2c_hid *ihid)
{
	u32 val;
	int ret;
	struct i2c_client *client = ihid->client;

	ihid->pogopin_detect_check = device_property_read_bool(&client->dev, "pogopin-detect-check");

	if(ihid->pogopin_detect_check) {
		if (device_property_read_string_array(&client->dev, "keyboard-firmware-name",
			ihid->keyboard_firmware_name, KEYBOARD_FIRMWARE_NUM) < 0) {
			ihid->keyboard_firmware_name[0] = "KBD_MCU.bin";
			ihid->keyboard_firmware_name[1] = NULL;
		}
		if (device_property_read_string_array(&client->dev, "keyboard-dev-name",
			ihid->keyboard_dev_name, KEYBOARD_FIRMWARE_NUM) < 0) {
			ihid->keyboard_dev_name[0] = "OPLUS Smart Keyboard";
			ihid->keyboard_dev_name[1] = "NULL";
		}

		dev_err(&client->dev, "keyboard firmware name[1] %s, name[2] %s, keyboard dev name[1] %s, name[2] %s\n",
			ihid->keyboard_firmware_name[0], ihid->keyboard_firmware_name[1],
			ihid->keyboard_dev_name[0], ihid->keyboard_dev_name[1]);
	}

	ret = device_property_read_u32(&client->dev, "hid-fw-address", &val);

	if (val >> 16 || ret) {
		dev_err(&client->dev, "%s Bad HID fw address: 0x%08x\n", __func__, val);
		return;
	}

	ihid->hid_fw_address = val;

	ihid->pogopin_fw_support = device_property_read_bool(&client->dev, "pogopin-fw-support");
}

static ssize_t proc_debug_log_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[PAGESIZE] = {0};

	snprintf(page, PAGESIZE - 1, "%u", debug_log);
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_debug_log_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	int tmp = 0;
	char buffer[4] = {0};
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));

	if (count > 2) {
		return count;
	}

	if (copy_from_user(buffer, buf, count)) {
		i2c_hid_err(ihid, "%s: read proc input error.\n", __func__);
		return count;
	}

	if (1 == sscanf(buffer, "%d", &tmp)) {
		debug_log = tmp;
	} else {
		i2c_hid_err(ihid, "invalid content: '%s', length = %zd\n", buf, count);
	}

	return count;
}

static const struct proc_ops proc_debug_log_ops = {
	.proc_open = simple_open,
	.proc_read = proc_debug_log_read,
	.proc_write = proc_debug_log_write,
};

static int proc_hid_dump_read(struct seq_file *s, void *v)
{
	struct i2c_hid *ihid = s->private;
	int i = 0;

	if (!ihid) {
		return 0;
	}

	seq_printf(s, "wHIDDescLength: 0x%04x\n", ihid->hdesc.wHIDDescLength);
	seq_printf(s, "bcdVersion: 0x%04x\n", ihid->hdesc.bcdVersion);
	seq_printf(s, "wReportDescLength: 0x%04x\n", ihid->hdesc.wReportDescLength);
	seq_printf(s, "wReportDescRegister: 0x%04x\n", ihid->hdesc.wReportDescRegister);
	seq_printf(s, "wInputRegister: 0x%04x\n", ihid->hdesc.wInputRegister);
	seq_printf(s, "wMaxInputLength: 0x%04x\n", ihid->hdesc.wMaxInputLength);
	seq_printf(s, "wOutputRegister: 0x%04x\n", ihid->hdesc.wOutputRegister);
	seq_printf(s, "wMaxOutputLength: 0x%04x\n", ihid->hdesc.wMaxOutputLength);
	seq_printf(s, "wCommandRegister: 0x%04x\n", ihid->hdesc.wCommandRegister);
	seq_printf(s, "wDataRegister: 0x%04x\n", ihid->hdesc.wDataRegister);
	seq_printf(s, "wVendorID: 0x%04x\n", ihid->hdesc.wVendorID);
	seq_printf(s, "wProductID: 0x%04x\n", ihid->hdesc.wProductID);
	seq_printf(s, "wVersionID: 0x%04x\n", ihid->hdesc.wVersionID);

	if ((NULL != ihid->hid->dev_rdesc) && (0 != ihid->hid->dev_rsize)) {
		seq_printf(s, "---hid descriptor start------\n");
		for (i = 0; i < ihid->hid->dev_rsize/2; i++) {
			seq_printf(s, "0x%02x 0x%02x", ihid->hid->dev_rdesc[2*i], ihid->hid->dev_rdesc[2*i+1]);
			seq_printf(s, "\n");
		}
		if (ihid->hid->dev_rsize%2) {
			seq_printf(s, "0x%02x \n", ihid->hid->dev_rdesc[ihid->hid->dev_rsize - 1]);
		}
		seq_printf(s, "---hid descriptor end--------\n");
	}

	return 0;
}

static int proc_hid_dump_open(struct inode *inode, struct file *file) {
	return single_open(file, proc_hid_dump_read, PDE_DATA(inode));
}

static const struct proc_ops proc_hid_dump_ops = {
	.proc_open = proc_hid_dump_open,
	.proc_read = seq_read,
};

static ssize_t proc_hid_driver_check_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	uint8_t ret = 0;
	char page[PAGESIZE] = {0};
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));

	snprintf(page, PAGESIZE - 1, "%d", (NULL != ihid->hid->driver));
	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static const struct proc_ops proc_hid_driver_check_ops = {
	.proc_open = simple_open,
	.proc_read = proc_hid_driver_check_read,
};

static int proc_kpdmcu_sn_read(struct seq_file *s, void *v)
{
	struct i2c_hid *ihid = s->private;
	int i = 0;

	if (!ihid) {
		return 0;
	}

	for (i = 0; i < DEFAULT_SN_LEN - 1; i++) {
		seq_printf(s, "%c", ihid->report_sn[i]);
	}
	seq_printf(s, "\n");

	return 0;
}

static int proc_kpdmcu_sn_open(struct inode *inode, struct file *file) {
	return single_open(file, proc_kpdmcu_sn_read, PDE_DATA(inode));
}

static const struct proc_ops  proc_kpdmcu_sn_ops = {
	.proc_open    = proc_kpdmcu_sn_open,
	.proc_read    = seq_read,
	.proc_release = single_release,
	.proc_lseek   = default_llseek,
};

static ssize_t proc_kpdmcu_fw_update_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char page[4] = {0};

	snprintf(page, 3, "%d\n", ihid->kpd_fw_status);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_kpdmcu_fw_update_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char write_data[2] = { 0 };
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));

	if (count > 2) {
		return count;
	}

	if (copy_from_user(&write_data, buf, count)) {
		i2c_hid_err(ihid, "%s: read proc input error.\n", __func__);
		return count;
	}

	if(ihid->kpd_fw_status == FW_UPDATE_START)
		return count;


	if(write_data[0] == '1')
		schedule_work(&ihid->kpdmcu_fw_update_work);
	else if(write_data[0] == '2') {
		ihid->kpdmcu_fw_update_force = true;
		schedule_work(&ihid->kpdmcu_fw_update_work);
	} else {
		ihid->kpdmcu_fw_update_force = false;
	}

	return count;
}

static const struct proc_ops proc_kpdmcu_fw_update_ops = {
	.proc_read = proc_kpdmcu_fw_update_read,
	.proc_write = proc_kpdmcu_fw_update_write,
};

static ssize_t proc_kpdmcu_fw_check_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char page[PROC_PAGE_LEN] = {0};
	bool need_update_fw = false;
	u32 fw_count = 0;

	need_update_fw = ihid->is_kpdmcu_need_fw_update | ihid->is_touchmcu_need_fw_update;

	if(ihid->is_kpdmcu_need_fw_update)
		fw_count += ihid->kpdmcu_fw_cnt;
	if(ihid->is_touchmcu_need_fw_update)
		fw_count += ihid->touchmcu_fw_count;

	snprintf(page, PROC_PAGE_LEN - 1, "%02x.%02x:%02x.%02x:%d:%d:%d\n",
			    ihid->kpdmcu_fw_mcu_version, ihid->touchmcu_fw_mcu_version,
				ihid->kpdmcu_fw_data_ver, ihid->touchmcu_fw_data_version, fw_count,
				need_update_fw, ihid->fw_update_progress / FW_PERCENTAGE_100);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_kpdmcu_fw_check_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char write_data[2] = { 0 };
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));

	if (count > 2) {
		return count;
	}

	if (copy_from_user(&write_data, buf, count)) {
		i2c_hid_err(ihid, "%s: read proc input error.\n", __func__);
		return count;
	}

		if(write_data[0] == '1' && keyboard_fw_ihid != NULL)
			schedule_delayed_work(&keyboard_fw_ihid->kpdmcu_fw_data_version_work, 0);

	return count;
}

static const struct proc_ops proc_kpdmcu_fw_check_ops = {
	.proc_read = proc_kpdmcu_fw_check_read,
	.proc_write = proc_kpdmcu_fw_check_write,
};

static ssize_t proc_padmcu_fw_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char page[PROC_PAGE_LEN] = {0};
	ret = 0;

	snprintf(page, PROC_PAGE_LEN - 1, "%02x.%02x\n", ihid->padmcu_fw_data_version, ihid->padmcu_fw_mcu_version);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static ssize_t proc_padmcu_fw_write(struct file *file, const char __user *buf, size_t count, loff_t *lo)
{
	char write_data[PROC_PAGE_LEN] = { 0 };
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));

	if (count > 2) {
		return count;
	}

	if (copy_from_user(&write_data, buf, count)) {
		i2c_hid_err(ihid, "%s: read proc input error.\n", __func__);
		return count;
	}

	if (write_data[0] == '1') {
		ihid->padmcu_fw_update_force = true;
		schedule_delayed_work(&ihid->padmcu_fw_update_work, 0);
	} else {
		ihid->padmcu_fw_update_force = false;
		i2c_hid_err(ihid, "invalid content: '%s', length = %zd\n", buf, count);
	}

	return count;
}

static const struct proc_ops proc_padmcu_fw_ops = {
	.proc_read = proc_padmcu_fw_read,
	.proc_write = proc_padmcu_fw_write,
};

static ssize_t proc_kbd_touch_status_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct i2c_hid *ihid = PDE_DATA(file_inode(file));
	uint8_t ret = 0;
	char page[PROC_PAGE_LEN] = {0};

	snprintf(page, PROC_PAGE_LEN - 1, "%u", ihid->keyboard_touch_status);

	ret = simple_read_from_buffer(buf, count, ppos, page, strlen(page));

	return ret;
}

static const struct proc_ops proc_kbd_touch_status_ops = {
	.proc_read = proc_kbd_touch_status_read,
};

const struct firmware *get_fw_firmware(struct i2c_hid *ihid, const char *patch)
{
	struct i2c_client *client = ihid->client;
	char *fw_patch = NULL;
	int retry = 2;
	int ret = 0;
	const struct firmware *fw_entry = NULL;

	fw_patch = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
	if(fw_patch == NULL)
		return NULL;

	snprintf(fw_patch, MAX_FW_NAME_LENGTH, "pogopin/%d/%s", get_project(), patch);
	do {
		ret = request_firmware(&fw_entry, fw_patch, &client->dev);
		if(ret < 0) {
			i2c_hid_err(ihid, "%s: Failed to request fw\n", __func__);
			msleep(100);
		} else {
			break;
		}
	} while ((ret < 0) && (--retry > 0));

	i2c_hid_err(ihid, "%s: fw_path is [%s]\n", __func__, fw_patch);
	kfree(fw_patch);
	return fw_entry;
}

static bool Check_kpdmcu_fw_update(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	u8 send_cmd[4] = {0x06,0x71,0xf9,0x8e};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;
	int retry = 10;

	if(ihid == NULL || client == NULL)
		return false;

	do {
		ret = hid_fw_update_cmd_send_recv(client, client->addr, send_cmd, 4, NULL, 0);
		if (ret) {
			i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
			continue;
		}
		msleep(5);
		ret = hid_fw_update_cmd_send_recv(client, client->addr, send_mask_cmd, 1, recv_buf, 1);
		if (ret || (recv_buf[0] == 0xFF)) {
			i2c_hid_err(ihid, "%s: read kpdmcu fw_version  fail, retry = %d\n", __func__, retry);
			msleep(10);
		} else {
			ihid->kpdmcu_fw_mcu_version = recv_buf[0];
			break;
		}
	} while (--retry);


	i2c_hid_err(ihid, "%s: kpad mcu version is %d, fw version is %d\n",
			    __func__, ihid->kpdmcu_fw_mcu_version, ihid->kpdmcu_fw_data_ver);

	if(ihid->kpdmcu_fw_mcu_version != ihid->kpdmcu_fw_data_ver)
		return true;
	else
		return false;
}

static int kpdmcu_enter_fw_mode(struct i2c_hid *ihid)
{
	/*SL:0X1,SH:0x81,SCL:0Xfe,SCH:0X7e,add0:0x60,add1:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_cmd[4] = {0x01,0x71,0xfe,0x8e};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid,  "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid,  "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int padmcu_read_panel_mcu_fw(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	u8 send_cmd[4] = {0x06,0x81,0xf9,0x7e};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;

	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}
	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] == 0xFF)) {
		ihid->padmcu_fw_mcu_version = 0x1;
		i2c_hid_err(ihid, "%s mask_cmd fail, default version = %d, recv_buf[0]  = %x\n",
				__func__, ihid->padmcu_fw_mcu_version, recv_buf[0]);
		return ret < 0 ? ret : -EIO;
	}
	ihid->padmcu_fw_mcu_version = recv_buf[0];
	dev_info(&client->dev, "%s padmcu fw version is %d\n", __func__, ihid->padmcu_fw_mcu_version);

	return 0;
}

static int padmcu_enter_fw_mode(struct i2c_hid *ihid)
{
	/*SL:0X1,SH:0x81,SCL:0Xfe,SCH:0X7e,add0:0x60,add1:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_cmd[4] = {0x01,0x81,0xfe,0x7e};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s hid_fw_address send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s hid_fw_address send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static u16 _crc(u32 n)
{
  u32 i, acc;

  for (n <<= 8, acc = 0, i = 8; i > 0; i--, n <<= 1)
  {
    acc = ((n ^ acc) & 0x8000) ? ((acc << 1) ^ CCITT) : (acc << 1);
  }

  return (u16)(acc);
}


static u16 crc16_sram(u16 crc, u8 *buffer, u32 length)
{
	u32 i, j = 0;

	for (i = 0; i < length; i++)
	{
	  j = (crc >> 8) ^ buffer[i];
	  crc = (crc << 8) ^ _crc(j);
	}

	return crc;

}

static int kpdmcu_clear_fw_area(struct i2c_hid *ihid, u32 count)
{
	/*SL:0X3,SH:0X81,SCL:0Xfc,SCH:0X7e,add:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_rx[16] = {0x03,0x71,0xfc,0x8e,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;
	u16 new_fw_crc16 = 0;

	send_rx[12] = count & 0xff;
	send_rx[13] = (count >> 8) & 0xff;

	new_fw_crc16 = 0;
	new_fw_crc16 = crc16_sram(new_fw_crc16, &send_rx[4], 12);
	send_rx[6] = new_fw_crc16 & 0xff;
	send_rx[7] = (new_fw_crc16 >> 8) & 0xff;

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, 16, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(100);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int padmcu_clear_fw_area(struct i2c_hid *ihid, u32 count)
{
	/*SL:0X3,SH:0X81,SCL:0Xfc,SCH:0X7e,add:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_rx[16] = {0x03,0x81,0xfc,0x7e,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 send_mask_cmd[1] = {0x5a};
	u8 recv_buf[1] = {0};
	int ret = 0;
	u16 new_fw_crc16 = 0;

	send_rx[12] = count & 0xff;
	send_rx[13] = (count >> 8) & 0xff;

	new_fw_crc16 = 0;
	new_fw_crc16 = crc16_sram(new_fw_crc16, &send_rx[4], 12);
	send_rx[6] = new_fw_crc16 & 0xff;
	send_rx[7] = (new_fw_crc16 >> 8) & 0xff;

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, 16, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(100);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int kpdmcu_write_fw_area(struct i2c_hid *ihid, u32 count)
{
	/*SL:0X4,SH:0X81,SCL:0Xfb,SCH:0X7e,add:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_rx[16] = {0x09,0x71,0xf6,0x8e,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 send_mask_cmd[1] = {0x5a};
	u8 recv_buf[1] = {0};
	int ret = 0;
	u16 new_fw_crc16 = 0;

	send_rx[12] = count & 0xff;
	send_rx[13] = (count >> 8) & 0xff;

	new_fw_crc16 = 0;
	new_fw_crc16 = crc16_sram(new_fw_crc16, &send_rx[4], 12);
	send_rx[6] = new_fw_crc16 & 0xff;
	send_rx[7] = (new_fw_crc16 >> 8) & 0xff;

	msleep(10);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, 16, NULL, 0);

	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}


static int padmcu_write_fw_area(struct i2c_hid *ihid, u32 count)
{
	/*SL:0X4,SH:0X81,SCL:0Xfb,SCH:0X7e,add:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_rx[16] = {0x04,0x81,0xfb,0x7e,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 send_mask_cmd[1] = {0x5a};
	u8 recv_buf[1] = {0};
	int ret = 0;
	u16 new_fw_crc16 = 0;

	if(ihid->padmcu_fw_mcu_version > 0x1 && ihid->padmcu_fw_update_force == false) {
		send_rx[0] = 0x09;
		send_rx[2] = 0xf6;
	}
	send_rx[12] = count & 0xff;
	send_rx[13] = (count >> 8) & 0xff;

	new_fw_crc16 = 0;
	new_fw_crc16 = crc16_sram(new_fw_crc16, &send_rx[4], 12);
	send_rx[6] = new_fw_crc16 & 0xff;
	send_rx[7] = (new_fw_crc16 >> 8) & 0xff;

	msleep(10);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, 16, NULL, 0);

	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int kpdmcu_write_fw_update(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count)
{
	struct i2c_client *client = ihid->client;
	u8 send_rx[ONE_WRITY_LEN_MAX + 2] = {0};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;
	u16 new_fw_crc16 = 0;
	u32 write_len = 0, fw_len = 0, fw_offset = 0;
	int i;

	fw_len = count;
	while (fw_len) {
		write_len = (fw_len < ONE_WRITY_LEN_MAX) ? fw_len : ONE_WRITY_LEN_MAX;
		for(i=0; i<ONE_WRITY_LEN_MAX;i++) {
			send_rx[i] = fw_data[fw_offset + i];
		}

		if(fw_len < ONE_WRITY_LEN_MAX) {
			for(i = 0; i < ONE_WRITY_LEN_MAX - fw_len; i++) {
				send_rx[ONE_WRITY_LEN_MAX - i] = 0xff;
			}
		}
		new_fw_crc16 = 0;
		new_fw_crc16 = crc16_sram(new_fw_crc16, send_rx, ONE_WRITY_LEN_MAX);
		send_rx[ONE_WRITY_LEN_MAX] = new_fw_crc16 & 0xff;
		send_rx[ONE_WRITY_LEN_MAX + 1] = (new_fw_crc16 >> 8) & 0xff;

		msleep(10);
		ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, ONE_WRITY_LEN_MAX + 2, NULL, 0);
		if (ret) {
			i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
			return ret;
		}

		msleep(5);
		ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
		if (ret || (recv_buf[0] != 0x01)) {
			i2c_hid_err(ihid, "%s send mask_cmd fail, recv_buf = %x\n", __func__, recv_buf[0]);
			return ret < 0 ? ret : -EIO;
		}

		fw_offset += write_len;
		fw_len -= write_len;

		if(ihid->is_touchmcu_need_fw_update) {
			if(ihid->fw_update_progress < FW_PROGRESS_25 * FW_PERCENTAGE_100) {
				ihid->fw_update_progress = FW_PROGRESS_3 * FW_PERCENTAGE_100 +
				    FW_PROGRESS_22 * (count - fw_len) * FW_PERCENTAGE_100 / count;
			} else {
				ihid->fw_update_progress = FW_PROGRESS_25 * FW_PERCENTAGE_100 +
				    FW_PROGRESS_25 * (count - fw_len) * FW_PERCENTAGE_100 / count;
			}
		} else {
			if(ihid->fw_update_progress < FW_PROGRESS_50 * FW_PERCENTAGE_100) {
				ihid->fw_update_progress = FW_PROGRESS_3 * FW_PERCENTAGE_100 +
				    FW_PROGRESS_47 * (count - fw_len) * FW_PERCENTAGE_100 / count;
			} else {
				ihid->fw_update_progress = FW_PROGRESS_50 * FW_PERCENTAGE_100  +
				    FW_PROGRESS_47 * (count - fw_len) * FW_PERCENTAGE_100 / count;
			}
		}
	}

	return 0;
}

static int padmcu_write_fw_update(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count)
{
	struct i2c_client *client = ihid->client;
	u8 *send_rx;
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;
	u16 new_fw_crc16 = 0;
	u32 write_len = 0, fw_len = 0, fw_offset = 0;
	int i;
	u32 one_writy_len_max = ONE_WRITY_LEN_MIN;

	if(ihid->padmcu_fw_mcu_version > 0x1 && ihid->padmcu_fw_update_force == false) {
		one_writy_len_max = ONE_WRITY_LEN_MAX;
	}
	send_rx = kzalloc(one_writy_len_max + 2, GFP_KERNEL);
	fw_len = count;
	fw_offset = 0;
	while (fw_len) {
		write_len = (fw_len < one_writy_len_max) ? fw_len : one_writy_len_max;
		for(i=0; i<one_writy_len_max;i++) {
			send_rx[i] = fw_data[fw_offset + i];
		}

		if(fw_len < one_writy_len_max) {
			for(i = 0; i < one_writy_len_max - fw_len; i++) {
				send_rx[one_writy_len_max - i] = 0xff;
			}
		}
		new_fw_crc16 = 0;
		new_fw_crc16 = crc16_sram(new_fw_crc16, send_rx, one_writy_len_max);
		send_rx[one_writy_len_max] = new_fw_crc16 & 0xff;
		send_rx[one_writy_len_max + 1] = (new_fw_crc16 >> 8) & 0xff;

		msleep(10);
		ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, one_writy_len_max + 2, NULL, 0);
		if (ret) {
			i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
			kfree(send_rx);
			return ret;
		}

		msleep(5);
		ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
		if (ret || (recv_buf[0] != 0x01)) {
			i2c_hid_err(ihid, "%s send mask_cmd fail, recv_buf = %x\n", __func__, recv_buf[0]);
			kfree(send_rx);
			return ret < 0 ? ret : -EIO;
		}

		fw_offset += write_len;
		fw_len -= write_len;
	}

	kfree(send_rx);
	return 0;
}

static int kpdmcu_check_fw_area(struct i2c_hid *ihid, u32 count)
{
	/*SL:0X4,SH:0X81,SCL:0Xfb,SCH:0X7e,add:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_rx[16] = {0x09,0x71,0xf6,0x8e,0x0c,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;
	u16 new_fw_crc16 = 0;

	send_rx[12] = count & 0xff;
	send_rx[13] = (count >> 8) & 0xff;

	new_fw_crc16 = 0;
	new_fw_crc16 = crc16_sram(new_fw_crc16, &send_rx[4], 12);
	send_rx[6] = new_fw_crc16 & 0xff;
	send_rx[7] = (new_fw_crc16 >> 8) & 0xff;

	msleep(10);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, 16, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;

}

static int padmcu_check_fw_area(struct i2c_hid *ihid, u32 count)
{
	/*SL:0X4,SH:0X81,SCL:0Xfb,SCH:0X7e,add:0x70*/
	struct i2c_client *client = ihid->client;
	u8 send_rx[16] = {0x04,0x81,0xfb,0x7e,0x0c,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;
	u16 new_fw_crc16 = 0;

	if(ihid->padmcu_fw_mcu_version > 0x1 && ihid->padmcu_fw_update_force == false) {
		send_rx[0] = 0x09;
		send_rx[2] = 0xf6;
	}

	send_rx[12] = count & 0xff;
	send_rx[13] = (count >> 8) & 0xff;

	new_fw_crc16 = 0;
	new_fw_crc16 = crc16_sram(new_fw_crc16, &send_rx[4], 12);
	send_rx[6] = new_fw_crc16 & 0xff;
	send_rx[7] = (new_fw_crc16 >> 8) & 0xff;

	msleep(10);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_rx, 16, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int kpdmcu_reset_mcu(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	u8 send_cmd[4] = {0x02,0x71,0xfd,0x8e};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int padmcu_reset_mcu(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	u8 send_cmd[4] = {0x02,0x81,0xfd,0x7e};
	u8 recv_buf[1] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret = 0;

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}

	msleep(5);
	ret = hid_fw_update_cmd_send_recv(client, ihid->hid_fw_address, send_mask_cmd, 1, recv_buf, 1);
	if (ret || (recv_buf[0] != 0x01)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail\n", __func__);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static void init_i2c_hid_proc(struct i2c_hid *ihid)
{
	struct proc_dir_entry *prEntry_hid = NULL, *prEntry_tmp = NULL;
	struct proc_dir_entry *prEntry_pogopin = NULL;

	if (!ihid) {
		return;
	}

	prEntry_hid = proc_mkdir(ihid->hid->name, NULL);
	if (NULL == prEntry_hid) {
		i2c_hid_err(ihid, "%s: proc mkdir %s failed.\n", __func__, ihid->hid->name);
		return;
	}

	prEntry_tmp = proc_create_data("hid_dump", 0444, prEntry_hid, &proc_hid_dump_ops, ihid);
	if (prEntry_tmp == NULL) {
		i2c_hid_err(ihid, "%s: mkdir hid_dump proc entry failed.\n", __func__);
	}

	prEntry_tmp = proc_create_data("hid_driver_check", 0444, prEntry_hid, &proc_hid_driver_check_ops, ihid);
	if (prEntry_tmp == NULL) {
		i2c_hid_err(ihid, "%s: mkdir hid driver check proc entry failed.\n", __func__);
	}

	if(ihid->pogopin_fw_support) {
		prEntry_pogopin = proc_mkdir(POGOPIN_INFO, NULL);
		if (NULL == prEntry_pogopin) {
			i2c_hid_err(ihid, "%s: proc mkdir %s failed.\n", __func__, POGOPIN_INFO);
			return;
		}

		prEntry_tmp = proc_create_data("debug_log", 0644, prEntry_pogopin, &proc_debug_log_ops, ihid);
		if (prEntry_tmp == NULL) {
			i2c_hid_err(ihid, "%s: mkdir debug_level proc entry failed.\n", __func__);
		}

		prEntry_tmp = proc_create_data("kpdmcu_fw_update", 0666, prEntry_pogopin, &proc_kpdmcu_fw_update_ops, ihid);
		if (prEntry_tmp == NULL) {
			i2c_hid_err(ihid, "%s: create kpdmcu_fw_update proc entry failed.\n", __func__);
		}

		prEntry_tmp = proc_create_data("padmcu_fw", 0644, prEntry_pogopin, &proc_padmcu_fw_ops, ihid);
		if (prEntry_tmp == NULL) {
			i2c_hid_err(ihid, "%s: create padmcu_fw proc entry failed.\n", __func__);
		}

		prEntry_tmp = proc_create_data("kpdmcu_fw_check", 0644, prEntry_pogopin, &proc_kpdmcu_fw_check_ops, ihid);
		if (prEntry_tmp == NULL) {
			i2c_hid_err(ihid, "%s: create kpdmcu_fw_check proc entry failed.\n", __func__);
		}

		prEntry_tmp = proc_create_data("kpdmcu_sn", 0644, prEntry_pogopin, &proc_kpdmcu_sn_ops, ihid);
		if (prEntry_tmp == NULL) {
			i2c_hid_err(ihid, "%s: create kpdmcu_sn proc entry failed.\n", __func__);
		}

		prEntry_tmp = proc_create_data("kbd_touch_status", 0644, prEntry_pogopin, &proc_kbd_touch_status_ops, ihid);
		if (prEntry_tmp == NULL) {
			i2c_hid_err(ihid, "%s: create kbd_touch_status proc entry failed.\n", __func__);
		}
	}

	return;
}

static int Touchmcu_getreport(struct i2c_hid *ihid,u8 reportId, u8 *buf)
{
	int ret = 0;
	u8 rcv_buf[GDIX_OUT_PUT_REPORT_SIZE + 1] = {0};

	rcv_buf[0] = reportId;
	ret = i2c_hid_raw_request(ihid->hid, rcv_buf[0], rcv_buf, GDIX_OUT_PUT_REPORT_SIZE,  HID_FEATURE_REPORT,  HID_REQ_GET_REPORT);
	msleep(12);
	if (ret < 0) {
		i2c_hid_err(ihid, "%s failed get feature retry, ret=%d\n", __func__, ret);
		return ret;
	} else {
		if (rcv_buf[0] == reportId) {
			memcpy(buf, rcv_buf, GDIX_OUT_PUT_REPORT_SIZE);
			return 0;
		} else {
			i2c_hid_err(ihid, "%s Get Wrong reportId:id=0x%x\n", __func__, rcv_buf[0]);
			return ret < 0 ? ret : -EIO;
		}
	}
}

static int Touchmcu_write(struct i2c_hid *ihid, const u8 *buf,u32 len)
{
	u8 temp_buf[GDIX_OUT_PUT_REPORT_SIZE];
	int ret = 0;
	int retry = GDIX_RETRY_TIMES;

	if (len > GDIX_OUT_PUT_REPORT_SIZE)
		return -1;
	memset(temp_buf, 0, GDIX_OUT_PUT_REPORT_SIZE);
	memcpy(&temp_buf[0], buf, len);
	temp_buf[0] = 0x0E;
	do {
		ret = i2c_hid_raw_request(ihid->hid, temp_buf[0], temp_buf, len,  HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
		msleep(12);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s failed set feature, retry: ret=%d,retry:%d\n", __func__, ret,retry);
			msleep(10);
		} else {
			break;
		}
	} while(--retry);
	return ret;
}

/*
 * write special data to IC directly buf_len <= 65
 * return < 0 failed
*/
static int Touchmcu_WriteSpeCmd(struct i2c_hid *ihid, const u8 *buf,u32 len)
{
	unsigned char temp_buf[GDIX_OUT_PUT_REPORT_SIZE];
	int ret = 0;
	int retry = GDIX_RETRY_TIMES;

	if (len > GDIX_OUT_PUT_REPORT_SIZE)
		return -1;

	memset(temp_buf, 0, GDIX_OUT_PUT_REPORT_SIZE);
	memcpy(&temp_buf[0], buf, len);
	do {
		ret = i2c_hid_raw_request(ihid->hid, temp_buf[0], temp_buf, len,  HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
		msleep(12);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s failed set feature, retry: ret=%d,retry:%d\n", __func__, ret,retry);
			msleep(10);
		} else {
			break;
		}
	} while(--retry);

	return ret;
}

int Touchmcu_write_cmd(struct i2c_hid *ihid, u16 addr, const u8 *buf, u32 len)
{
	u8 tmpBuf[GDIX_OUT_PUT_REPORT_SIZE] = {0x0e,0x20, 0, 0, 5};
	u16 current_addr = addr;
	u32 pos = 0, transfer_length = 0;
	bool has_follow_pkg = false;
	u8 pkg_num = 0;
	int ret = 0;

	while (pos != len) {
		if (len - pos > GDIX_OUT_PUT_REPORT_SIZE - GDIX_PRE_HEAD_LEN - GDIX_DATA_HEAD_LEN) {
			/* data len more than 55, need to send with subpackages */
			transfer_length = GDIX_OUT_PUT_REPORT_SIZE - GDIX_PRE_HEAD_LEN - GDIX_DATA_HEAD_LEN;
			has_follow_pkg = true;
		} else {
			transfer_length = len - pos;
			has_follow_pkg = false;
		}
		if (has_follow_pkg)
			tmpBuf[2] = 0x01; /* set follow-up package flag */
		else
			tmpBuf[2] = 0x00;
		tmpBuf[3] = pkg_num++; /* set pack num */
		/* set HID package length = data_len + GDIX_DATA_HEAD_LEN */
		tmpBuf[4] = (unsigned char)(transfer_length + GDIX_DATA_HEAD_LEN);
		tmpBuf[5] = 0;	   /* write operation flag */
		tmpBuf[6] = (current_addr >> 8) & 0xff;
		tmpBuf[7] = current_addr & 0xff;
		tmpBuf[8] = (unsigned char)((transfer_length >> 8) & 0xff);
		tmpBuf[9] = (unsigned char)(transfer_length & 0xff);
		memcpy(&tmpBuf[GDIX_PRE_HEAD_LEN + GDIX_DATA_HEAD_LEN], &buf[pos], transfer_length);
		ret = Touchmcu_write(ihid, tmpBuf, transfer_length + GDIX_PRE_HEAD_LEN + GDIX_DATA_HEAD_LEN);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s Failed write data to addr=0x%x, len=%d,ret = %d\n",
				__func__, current_addr, transfer_length, ret);
			break;
		} else {
			pos += transfer_length;
			current_addr += transfer_length;
		}
	}
	if (ret < 0)
		return ret < 0 ? ret : -EIO;
	else
		return len;
}

static int Touchmcu_readpkg(struct i2c_hid *ihid, u16 addr, u8 *buf, u32 len)
{
	int ret = 0;
	int retry = 0;
	u8 HidBuf[65] =	{0x0e, 0x20,0,0,5,1};
	u32 pkg_index = 0;
	u32 read_data_len = 0;

re_start:
	pkg_index = 0;
	read_data_len = 0;
	HidBuf[0] = 0x0e;
	HidBuf[1] = 0x20;
	HidBuf[2] = 0;
	HidBuf[3] = 0;
	HidBuf[4] = 5;
	HidBuf[5] = 1; /* read operation flag */
	HidBuf[6] = (addr >> 8) & 0xff;
	HidBuf[7] = addr & 0xff;
	HidBuf[8] = (len >> 8) & 0xff;
	HidBuf[9] = len & 0xff;
	ret = Touchmcu_write(ihid, HidBuf, 10);

	if (ret < 0) {
		i2c_hid_err(ihid, "%s Failed send read start package, ret = %d\n", __func__, ret);
		return -1;
	}
	do {
		ret = Touchmcu_getreport(ihid, 0x0e, HidBuf);
		if (ret) {
			i2c_hid_err(ihid, "%s Failed read addr=0x%x, len=%d\n", __func__, addr, len);
			break;
		} else {
			if (pkg_index != HidBuf[3]) {
				if (retry++ < GDIX_RETRY_TIMES) {
					i2c_hid_err(ihid, "%s Read retry %d, pkg_index %d != HidBuf[3](%d)\n",
						 __func__, retry,pkg_index, HidBuf[3]);
					msleep(1);
					goto re_start;
				}
				ret = -1;
				break;
			} else {
				if (HidBuf[4] == len - read_data_len) {
					memcpy(buf + read_data_len, &HidBuf[5], HidBuf[4]);
					read_data_len += HidBuf[4];
					pkg_index++;
				} else {
					i2c_hid_err(ihid, "%s Data length err: %d != %d\n",
						 __func__, HidBuf[4], len - read_data_len);
					if (retry++ < GDIX_RETRY_TIMES) {
						i2c_hid_err(ihid, "%s Read retry: %d\n", __func__, retry);
						msleep(1);
						goto re_start;
					}
					ret = -1;
					break;
				}
			}
		}
	} while (read_data_len != len && (retry < GDIX_RETRY_TIMES));
	if (ret < 0)
		return ret;
	else
		return len;
}

static int Touchmcu_read(struct i2c_hid *ihid, u16 addr, u8 *buf, u32 len)
{
	int ret = 0;
	int i = 0;
	u16 tmp_addr;
	int pkg_size = 55;
	int pkg_num = 0;
	int read_len = 0;
	int code_len = len;

	pkg_num = code_len/pkg_size +1;
	tmp_addr = addr;
	read_len = pkg_size;
	for (i=0; i<pkg_num; i++) {
	   if (read_len > code_len)
		   read_len = code_len;
	   /*read data*/
	   ret = Touchmcu_readpkg(ihid, tmp_addr, &buf[tmp_addr-addr], read_len);
	   if (ret < 0)
		   return ret;
	   tmp_addr += read_len;
	   code_len -= read_len;
	   if (code_len <= 0)
		   break;
	}
	return ret;
}

static u8 Get_touch_update_flag(const unsigned char *fw_data, u32 count)
{
	return (u8)(fw_data[count + 6 + 2]);
}

static bool Check_touchmcu_fw_update(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	u8 mcu_cfg_ver = 0;
	u8 fw_info[72] = {0};
	int ret = 0;
	int retry = 5;

	if(ihid == NULL || client == NULL)
		return false;

	do {
		ret = Touchmcu_read(ihid, CFG_START_ADDR, &mcu_cfg_ver, 1);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s read_touchpad_fw_version cmd fail, retry is %d\n", __func__, retry);
			continue;
		} else {
			ihid->touchmcu_fw_mcu_version = mcu_cfg_ver;
		}
		ret = Touchmcu_read(ihid, VER_ADDR, fw_info, sizeof(fw_info));
		if(ret < 0) {
			i2c_hid_err(ihid, "%s read_touchpad_fw_version cmd fail, retry is %d\n", __func__, retry);
		} else {
			ihid->touchmcu_cid_version_major = fw_info[18];
			ihid->touchmcu_cid_version_minor = (fw_info[19] << 8) | fw_info[20];
			break;
		}
	} while (--retry);

	i2c_hid_err(ihid, "touch mcu version is %d, fw version is %d\n",
				    ihid->touchmcu_fw_mcu_version, ihid->touchmcu_fw_data_version);
	i2c_hid_err(ihid, "touch mcu cid version major is %d, cid version minor is %d",
				    ihid->touchmcu_cid_version_major, ihid->touchmcu_cid_version_minor);
	i2c_hid_err(ihid, "touch fw cid version major is %d, cid version minor is %d",
				    ihid->touchfw_cid_version_major, ihid->touchfw_cid_version_minor);

	if ((ihid->touchmcu_fw_mcu_version != ihid->touchmcu_fw_data_version) ||
		    (ihid->touchmcu_cid_version_major != ihid->touchfw_cid_version_major) ||
		    (ihid->touchmcu_cid_version_minor != ihid->touchfw_cid_version_minor))
		return true;
	else
		return false;
}

static int Touchmcu_switch_to_patch_mode(struct i2c_hid *ihid)
{
	int ret = 0;
	int retry = 4;
	u8 buf_switch_to_patch[6] = {0x00, 0x10, 0x00, 0x00, 0x01, 0x01};
	u8 temp_buf[2];

	/* switch to patch mode */
	do {
		ret = Touchmcu_write(ihid, buf_switch_to_patch, 6);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s Failed send switch to patch\n", __func__);
			msleep(50);
			continue;
		}
		msleep(250);
		temp_buf[0] = 0;
		ret = Touchmcu_read(ihid, BL_STATE_ADDR, temp_buf, 1);
		if (ret >= 0 && temp_buf[0] == 0xDD) {
			i2c_hid_err(ihid, "%s switch to patch mode\n", __func__);
			return 0;
		}
		i2c_hid_err(ihid, "%s retry send switch patch mode cmd, ret %d, value 0x%x\n", __func__, ret, temp_buf[0]);
	} while (--retry);

	i2c_hid_err(ihid, "%s failed switch to patch mode\n", __func__);
	return ret < 0 ? ret : -EIO;
}

static int Touchmcu_disable_report(struct i2c_hid *ihid)
{
	int ret = 0;
	int i = 0, j = 0;
	u8 dis_report_cmd[5] = {0x33, 0x00, 0xCD};
	u8 chk_state_cmd[5] = {0x35, 0x00, 0xCB};
	u8 cmd_ack_buf[3];
	u8 checksum;

	for (j = 0; j < 3; j++) {
		for(i = 0; i < 3; i++){
			ret = Touchmcu_write_cmd(ihid, CMD_ADDR, dis_report_cmd, 5);
			msleep(10);
			if (ret >= 0)
				break;
	}

		msleep(10);
		ret = Touchmcu_write_cmd(ihid, CMD_ADDR, chk_state_cmd, 5);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s failed send get state cmd. ret = %d. retry", __func__, ret);
			continue;
		}
		msleep(50);
		ret = Touchmcu_read(ihid, CMD_ADDR, cmd_ack_buf, 3);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s failed read cmd state, %d. retry", __func__, ret);
			continue;
		}

		checksum = 0x36 + cmd_ack_buf[1] + cmd_ack_buf[2];
		if (checksum) {
			i2c_hid_err(ihid, "%s cmd ack checksum error, cheksum 0x%x, ack %x %x %x",
					__func__, checksum, cmd_ack_buf[0], cmd_ack_buf[1], cmd_ack_buf[2]);
			continue;
		}
		if (cmd_ack_buf[1] == 0x01) {
			return 0;
		}
		i2c_hid_err(ihid, "%s invalid cmd ack data 0x%x", __func__, cmd_ack_buf[1]);
	}
	return ret < 0 ? ret : -EIO;
}

static int Touchmcu_enable_report(struct i2c_hid *ihid)
{
    int index = 0;
    u8 en_report_cmd[5] = {0x34, 0x00, 0xCC};

    for (index = 0; index < 3; index++) {
        Touchmcu_write_cmd(ihid, CMD_ADDR, en_report_cmd, 5);
        msleep(10);
    }
    return 0;
}

static int Touchmcu_load_sub_firmware(struct i2c_hid *ihid, u32 flash_addr,const unsigned char *fw_data, u32 count)
{
    int ret = 0;
	int retry;
	u32 i;
	u32 unitlen = 0;
	u8 temp_buf[2] = {0};
	u32 load_data_len = 0;
	u8 buf_load_flash[15] = {0x0e, 0x12, 0x00, 0x00, 0x06};
	u16 check_sum = 0;
	int retry_load = 0;

	while (retry_load < GDIX_RETRY_TIMES && load_data_len != count) {
		unitlen = (count - load_data_len > RAM_BUFFER_SIZE) ?
			  RAM_BUFFER_SIZE : (count - load_data_len);

		ret = Touchmcu_write_cmd(ihid, FLASH_BUFFER_ADDR, &fw_data[load_data_len], unitlen);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s Failed load fw, len %d : addr 0x%x, ret=%d\n",
				 __func__, unitlen, flash_addr, ret);
			goto load_fail;
		}

		/* inform IC to load 4K data to flash */
		for (check_sum = 0, i = 0; i < unitlen; i += 2) {
			check_sum += (fw_data[load_data_len + i] << 8) +
					fw_data[load_data_len + i + 1];
		}
		buf_load_flash[5] = (unitlen >> 8)&0xFF;
		buf_load_flash[6] = unitlen & 0xFF;
		buf_load_flash[7] = (flash_addr >> 16) & 0xFF;
		buf_load_flash[8] = (flash_addr >> 8) & 0xFF;
		buf_load_flash[9] = (check_sum >> 8) & 0xFF;
		buf_load_flash[10] = check_sum & 0xFF;

		ret = Touchmcu_write(ihid, buf_load_flash, 11);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s Failed write load flash command, ret =%d\n", __func__, ret);
			goto load_fail;
		}

		msleep(80);
		retry = 100;
		do {
			memset(temp_buf, 0, sizeof(temp_buf));
			ret = Touchmcu_read(ihid, FLASH_RESULT_ADDR,  temp_buf, 1);
			if (ret < 0) {
				i2c_hid_err(ihid, "Failed read 0x%x, ret=%d, kb_connected = %d\n",
					FLASH_RESULT_ADDR, ret, kb_connected);
				if(kb_connected == false)
					goto load_fail;
			}
			if (temp_buf[0] == 0xAA)
				break;

			msleep(2);
		} while(--retry);

		if (!retry) {
			i2c_hid_err(ihid, "Read back 0x%x(0x%x) != 0xAA\n", FLASH_RESULT_ADDR, temp_buf[0]);
			i2c_hid_err(ihid, "Reload(%d) subFW:addr:0x%x\n", retry_load, flash_addr);
			/* firmware chechsum err */
			retry_load++;
			ret = -1;
		} else {
			load_data_len += unitlen;
			flash_addr += unitlen;
			retry_load = 0;
			ret = 0;
		}

	}

load_fail:
	return ret;
}

static int Touchmcu_flash_cfg_with_isp(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count, u8 flag)
{
	int ret = -1, i = 0;
	const unsigned char* cfg = NULL;
	u8 cfg_4K[4096];
	u8 sub_cfg_id;
	u32 sub_cfg_len;
	u8 cfg_ver_infile;
	int sub_cfg_num = fw_data[count + 6 + 3];
	u32 sub_cfg_info_pos = count + 6 + 6;
	u32 cfg_offset = count + 6 + 64;

	if (sub_cfg_num <= 0) {
		/* no config found in the bin file */
		return 0;
	}

	if (!(flag & NEED_UPDATE_CONFIG)) {
		/* config update flag not set */
		i2c_hid_err(ihid, "flag UPDATE_CONFIG unset\n");
		return 0;
	}

	/* Start load config */
	if (!fw_data) {
		i2c_hid_err(ihid, "%s No valid fw data \n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	//fw_data[FW_IMAGE_SUB_FWNUM_OFFSET];
	i2c_hid_err(ihid, "%s load sub config, sub_cfg_num=%d\n", __func__, sub_cfg_num);
	for (i = 0; i < sub_cfg_num; i++) {
		sub_cfg_id = fw_data[sub_cfg_info_pos];
		i2c_hid_err(ihid, "load sub config, sub_cfg_id=0x%x\n", sub_cfg_id);

		sub_cfg_len = (fw_data[sub_cfg_info_pos + 1] << 8) |
			       fw_data[sub_cfg_info_pos + 2];

		if(sub_cfg_id == TOUCH_SENSOR_ID){
			cfg = &fw_data[cfg_offset];
			cfg_ver_infile = cfg[0];
			i2c_hid_err(ihid, "%s Find a cfg match cfg version=%d\n", __func__, cfg_ver_infile);
			break;
		}

		cfg_offset += sub_cfg_len;
		sub_cfg_info_pos += 3;
	}

	if (!cfg) {
		/* failed found config for sensorID */
		i2c_hid_err(ihid, "%s Failed found config sub_cfg_num %d\n",
				__func__, sub_cfg_num);
		return ret < 0 ? ret : -EIO;
	}

	i2c_hid_err(ihid, "%s load cfg addr len:0x%x\n", __func__, sub_cfg_len);
	if (sub_cfg_len > sizeof(cfg_4K)) {
		i2c_hid_err(ihid, "%s invalid cfg len %d", __func__, sub_cfg_len);
		return ret < 0 ? ret : -EIO;
	}
	// for Normandy serial device need align cfg to 4KB when flash with ISP
	memset(cfg_4K, 0xFF, sizeof(cfg_4K));
	memcpy(cfg_4K, cfg, sub_cfg_len);
	ret = Touchmcu_load_sub_firmware(ihid, CFG_FLASH_ADDR, cfg_4K, sizeof(cfg_4K));
	if (ret < 0) {
		i2c_hid_err(ihid, "%s Failed flash cofig with ISP, ret=%d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static int _touchmcu_fw_update(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count, u8 flag, bool cfg_flash)
{
	int retry;
	int ret = 0, i;
	u8 buf_start_update[6] = {0x00, 0x11, 0x00, 0x00, 0x01, 0x01};
	u8 buf_restart[6] = {0x0E, 0x13, 0x00, 0x00, 0x01, 0x01};
	u8 buf_switch_ptp_mode[6] = {0x03, 0x03, 0x00, 0x00, 0x01, 0x01};
	int sub_fw_num = 0;
	u8 sub_fw_type;
	u32 sub_fw_len;
	u32 sub_fw_flash_addr;
	u32 fw_image_offset = GTX8_SUB_FW_DATA_OFFSET;
	u32 sub_fw_info_pos;

	if (Touchmcu_switch_to_patch_mode(ihid)) {
		goto update_err;
	}
	i2c_hid_err(ihid, "%s success switch to patch mode\n", __func__);

	if (Touchmcu_disable_report(ihid)) {
		// just ignore this error
		i2c_hid_err(ihid, "%s failed disable touch report\n", __func__);
		//goto update_err;
	} else {
		i2c_hid_err(ihid, "%s success disable touch report\n", __func__);
	}

	/* Start update */
	ret = Touchmcu_write(ihid, buf_start_update, 6);
	if (ret < 0) {
		i2c_hid_err(ihid, "%s Failed start update, ret=%d\n", __func__, ret);
		goto update_err;
	}
	msleep(100);

	i2c_hid_err(ihid, "%s start update fw", __func__);
	sub_fw_info_pos = GTX8_SUB_FW_INFO_OFFSET;
	sub_fw_num = fw_data[GTX8_FW_IMAGE_SUB_FWNUM_OFFSET];
	i2c_hid_err(ihid, "%s load sub firmware, sub_fw_num=%d\n", __func__, sub_fw_num);

	/* load normal firmware package */
	for (i = 0; i < sub_fw_num; i++) {
		sub_fw_type = fw_data[sub_fw_info_pos];
		i2c_hid_err(ihid, "%s load sub firmware, sub_fw_type=0x%x\n", __func__, sub_fw_type);
		sub_fw_len = (fw_data[sub_fw_info_pos + 1] << 24) | (fw_data[sub_fw_info_pos + 2] << 16) |
					    (fw_data[sub_fw_info_pos + 3] << 8) | fw_data[sub_fw_info_pos + 4];

		sub_fw_flash_addr = (fw_data[sub_fw_info_pos + 5] << 8) | fw_data[sub_fw_info_pos + 6];
		sub_fw_flash_addr = sub_fw_flash_addr << 8;

		if (!(0x0C & (0x01 << sub_fw_type ))){
			i2c_hid_err(ihid, "%s Sub firmware type does not math:type=%d\n",
				  __func__, sub_fw_type);
			fw_image_offset += sub_fw_len;
			sub_fw_info_pos += 8;
			continue;
		}

		/* if sub fw type is HID subsystem we need compare version before update */
		// TODO update hid subsystem
		i2c_hid_err(ihid, "%s load sub firmware addr:0x%x,len:0x%x\n", __func__, sub_fw_flash_addr, sub_fw_len);
		ret = Touchmcu_load_sub_firmware(ihid, sub_fw_flash_addr, &fw_data[fw_image_offset], sub_fw_len);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s Failed load sub firmware, ret=%d\n", __func__, ret);
			goto update_err;
		}
		fw_image_offset += sub_fw_len;
		sub_fw_info_pos += 8;

		if(ihid->kpdmcu_update_end)
			ihid->fw_update_progress = FW_PROGRESS_50 * FW_PERCENTAGE_100 +
			    FW_PROGRESS_48 * ( i * FW_PERCENTAGE_100/ sub_fw_num);
		else
			ihid->fw_update_progress = FW_PROGRESS_5 * FW_PERCENTAGE_100 +
			    FW_PROGRESS_93 * ( i * FW_PERCENTAGE_100 / sub_fw_num);
	}

	/* flash config with isp if NEED_UPDATE_CONFIG_WITH_ISP flag is setted or
	* hid subsystem updated.
	*/
	if (flag & NEED_UPDATE_CONFIG_WITH_ISP) {
		cfg_flash = true;
		ret = Touchmcu_flash_cfg_with_isp(ihid, fw_data, count, flag);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s failed flash config with isp, ret %d\n", __func__, ret);
			goto update_err;
		}
		i2c_hid_err(ihid, "%s success flash config with isp\n", __func__);
	}

	ihid->fw_update_progress = FW_PROGRESS_99 * FW_PERCENTAGE_100;

	/* reset IC */
	i2c_hid_err(ihid, "%s flash fw finish, reset ic ret = %d\n", __func__, ret);
	retry = 3;
	do {
		ret = Touchmcu_write(ihid, buf_restart, 6);
		if (ret == 0)
			break;
		msleep(20);
	} while(--retry);
	if (retry == 0 && ret < 0)
		i2c_hid_err(ihid, "%s Failed write restart command, ret=%d\n", __func__, ret);
	else
		ret = 0;

	msleep(300);

	ret = Touchmcu_WriteSpeCmd(ihid, buf_switch_ptp_mode, 6);
	if (ret < 0) {
		i2c_hid_err(ihid, "%s Failed switch to ptp mode\n", __func__);
	} else {
		i2c_hid_err(ihid, "%s switch to ptp mode, ret=%d\n", __func__, ret);
		ret = 0;
	}
	return ret;

update_err:
	/* reset IC */
	i2c_hid_err(ihid, "%s update failed, reset ic\n", __func__);
	retry = 3;
	do {
		ret = Touchmcu_write(ihid, buf_restart, 6);
		if (ret == 0)
			break;
		msleep(20);
	} while(--retry);
	if (!retry)
		i2c_hid_err(ihid, "%s Failed write restart command, ret=%d\n", __func__, ret);

	msleep(300);
	Touchmcu_enable_report(ihid);
	return ret;
}

static int Touchmcu_cfg_update(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count)
{
	int retry;
	int ret = -1, i;
	u8 temp_buf[65];
	u8 cfg_ver_after[3];
	u8 cfg_ver_before[3];
	u8 cfg_ver_infile;
	bool findMatchCfg = false;
	const unsigned char *cfg = NULL;
	int sub_cfg_num = fw_data[count + 6 + 3];
	u8 sub_cfg_id;
	u32 sub_cfg_len;
	u32 sub_cfg_info_pos = count + 6 + 6;
	u32 cfg_offset = count + 6 + 64;

	if(sub_cfg_num == 0)
		return -5;

	//before update config,read curr config version
	Touchmcu_read(ihid, CFG_START_ADDR, cfg_ver_before,3);
	i2c_hid_err(ihid, "%s Before update,cfg version is 0x%02x 0x%02x 0x%02x\n",
		__func__, cfg_ver_before[0], cfg_ver_before[1], cfg_ver_before[2]);

	/* Start load config */
	if (!fw_data) {
		i2c_hid_err(ihid, "%s No valid fw data \n", __func__);
		return ret < 0 ? ret : -EIO;
	}

	//fw_data[FW_IMAGE_SUB_FWNUM_OFFSET];
	i2c_hid_err(ihid, "%s load sub config, sub_cfg_num=%d\n", __func__, sub_cfg_num);
	for (i = 0; i < sub_cfg_num; i++) {
		sub_cfg_id = fw_data[sub_cfg_info_pos];
		i2c_hid_err(ihid, "%s load sub config, sub_cfg_id=0x%x\n", __func__, sub_cfg_id);

		sub_cfg_len = (fw_data[sub_cfg_info_pos + 1] << 8) |
						    fw_data[sub_cfg_info_pos + 2];

		if(sub_cfg_id == TOUCH_SENSOR_ID){
			findMatchCfg = true;
			cfg = &fw_data[cfg_offset];
			cfg_ver_infile = cfg[0];
			i2c_hid_err(ihid, "%s Find a cfg match cfg version=%d\n", __func__, cfg_ver_infile);
			break;
		}
		cfg_offset += sub_cfg_len;
		sub_cfg_info_pos += 3;
	}

	if(!findMatchCfg) {
		i2c_hid_err(ihid, "%s no config found for sensorID %d\n", __func__, TOUCH_SENSOR_ID);
		return CFG_NOT_MATCH;
	}

	//wait untill ic is free
	retry = 10;
	do {
		ret = Touchmcu_read(ihid, CMD_ADDR, temp_buf, 1);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s Failed read cfg cmd, ret = %d\n", __func__, ret);
			return ret;
		}
		if (temp_buf[0] == 0xff)
			break;
		i2c_hid_err(ihid, "%s 0x%x value is 0x%x != 0xff, retry\n", __func__, CMD_ADDR, temp_buf[0]);
		msleep(10);
	} while (--retry);
	if (!retry) {
		i2c_hid_err(ihid, "%s Reg 0x%x != 0xff\n", __func__, CMD_ADDR);
		return ret < 0 ? ret : -EIO;
	}

	//tell ic i want to send cfg
	temp_buf[0] = 0x80;
	temp_buf[1] = 0;
	temp_buf[2] = 0x80;
	ret = Touchmcu_write_cmd(ihid, CMD_ADDR,temp_buf, 3) ;
	if (ret < 0) {
		i2c_hid_err(ihid, "%s Failed write send cfg cmd\n", __func__);
		return ret;
	}

	//wait ic to comfirm
	msleep(250);
	retry = GDIX_RETRY_TIMES;
	do {
		ret = Touchmcu_read(ihid, CMD_ADDR, temp_buf, 1);
		i2c_hid_err(ihid, "%s Wait CMD_ADDR == 0x82...\n", __func__);
		if (ret < 0) {
			i2c_hid_err(ihid, "Failed read 0x%x, ret = %d, kb_connected = %d\n",
				CMD_ADDR, ret, kb_connected);
			if(kb_connected == false)
				return ret;
		}
		if (temp_buf[0] == 0x82)
			break;
		i2c_hid_err(ihid, "%s 0x%x value is 0x%x != 0x82, retry\n", __func__, CMD_ADDR, temp_buf[0]);
		msleep(30);
	} while (--retry);

	if (!retry) {
		i2c_hid_err(ihid, "%s Reg 0x%x != 0x82\n", __func__, CMD_ADDR);
		return -2;
	}
	i2c_hid_err(ihid, "%s Wait CMD_ADDR == 0x82 success.\n", __func__);

	/* Start load config */
	ret = Touchmcu_write_cmd(ihid, CFG_START_ADDR, &fw_data[cfg_offset],sub_cfg_len) ;
	if (ret < 0) {
		i2c_hid_err(ihid, "%s Failed write cfg to xdata, ret=%d\n", __func__, ret);
		return ret;
	}
	msleep(100);

	//tell ic cfg is ready in xdata
	temp_buf[0] = 0x83;
	ret = Touchmcu_write_cmd(ihid, CMD_ADDR,temp_buf, 1) ;
	if (ret < 0) {
		i2c_hid_err(ihid, "%s Failed write send cfg finish cmd\n", __func__);
		return ret;
	}

	//check if ic is ok with the cfg
	msleep(80);
	retry = GDIX_RETRY_TIMES;
	do {
		ret = Touchmcu_read(ihid, CMD_ADDR, temp_buf, 1);
		i2c_hid_err(ihid, "%s Wait CMD_ADDR == 0xFF...\n", __func__);
		if (ret < 0) {
			i2c_hid_err(ihid, "%s Failed read 0x%x, ret = %d, kb_connected = %d\n",
				__func__, CMD_ADDR, ret, kb_connected);
			if(kb_connected == false)
				return ret;
		}
		if (temp_buf[0] == 0xff)
			break;
		i2c_hid_err(ihid, "%s 0x%x value is 0x%x != 0xFF, retry\n", __func__, CMD_ADDR, temp_buf[0]);
		msleep(30);
	} while (--retry);

	if (!retry) {
		i2c_hid_err(ihid, "%s Reg 0x%x != 0xFF\n", __func__, CMD_ADDR);
		return ret < 0 ? ret : -EIO;
	}
	i2c_hid_err(ihid, "%s Wait CMD_ADDR == 0xFF success.\n", __func__);

	//before update config,read curr config version
	Touchmcu_read(ihid, CFG_START_ADDR,cfg_ver_after,3);
	i2c_hid_err(ihid, "%s After update,cfg version is 0x%02x 0x%02x 0x%02x\n",
		__func__, cfg_ver_after[0],cfg_ver_after[1],cfg_ver_after[2]);

	return 0;
}

static int padmcu_fw_update(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count)
{
	int ret = 0, i;
	int retry = 3;

	for (i = 0; i < retry; i++)
	{
		if(i > 0) {
			msleep(300);
		}
		i2c_hid_err(ihid, "%s retry num =%d\n", __func__, i);
		/*enter update mode*/
		ret = padmcu_enter_fw_mode(ihid);
		if(ret) {
			i2c_hid_err(ihid, "%s enter fw mode fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s enter fw mode success\n", __func__);
		}

		/* clear update area*/
		ret = padmcu_clear_fw_area(ihid, count);
		if(ret) {
			i2c_hid_err(ihid, "%s clear fw area fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s clear fw area success\n", __func__);
		}

		/* write update area*/
		ret = padmcu_write_fw_area(ihid, count);
		if(ret) {
			i2c_hid_err(ihid, "%s write update area fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s write update area success\n", __func__);
		}

		/* write update data*/
		ret = padmcu_write_fw_update(ihid, fw_data, count);
		if(ret) {
			i2c_hid_err(ihid, "%s write update fw fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s write update fw success\n", __func__);
		}


		/*start check fw update*/
		ret = padmcu_check_fw_area(ihid, count);
		if(ret) {
			i2c_hid_err(ihid, "%s start check fw update fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s start check fw update success\n", __func__);
		}

		/*check fw update*/
		ret = padmcu_write_fw_update(ihid, fw_data, count);
		if(ret) {
			i2c_hid_err(ihid, "%s check fw update fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s check fw update success\n", __func__);
		}

		/*reset mcu*/
		ret = padmcu_reset_mcu(ihid);
		if(ret) {
			i2c_hid_err(ihid, "%s reset mcu fail\n", __func__);
		} else {
			i2c_hid_err(ihid, "%s reset mcu success\n", __func__);
			break;
		}
	}
	if (!ret) {
		msleep(1000);
		ret = padmcu_read_panel_mcu_fw(ihid);
		if(ret) {
			i2c_hid_err(ihid, "%s reset mcu  version fail\n", __func__);
		} else {
			i2c_hid_err(ihid, "%s reset mcu  version success\n", __func__);
		}
	}
	return ret;
}

static int kpdmcu_fw_update(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count)
{
	int ret = 0, i;
	int retry = 3;


	for (i=0; i < retry; i++)
	{
		if(kb_connected == false) {
			i2c_hid_err(ihid, "%s kpdmcu_fw_update fail, ret = %d\n", __func__, ret);
			return ret < 0 ? ret : -EIO;
		}

		if(i > 0) {
			ret = kpdmcu_reset_mcu(ihid);
			msleep(1000);
		}
		/*enter update mode*/
		ret = kpdmcu_enter_fw_mode(ihid);
		if(ret) {
			i2c_hid_err(ihid, "%s enter fw mode fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s enter fw mode success\n", __func__);
		}

		/* clear update area*/
		ret = kpdmcu_clear_fw_area(ihid, count);
		if(ret) {
			i2c_hid_err(ihid, "%s clear fw area fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s clear fw area success\n", __func__);
		}

		/* write update area*/
		ret = kpdmcu_write_fw_area(ihid, count);
		if(ret) {
			i2c_hid_err(ihid, "%s write update area fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s write update area success\n", __func__);
		}

		ihid->fw_update_progress  = FW_PROGRESS_3 * FW_PERCENTAGE_100;

		/* write update data*/
		ret = kpdmcu_write_fw_update(ihid, fw_data, count);
		if(ret) {
			i2c_hid_err(ihid, "%s write update fw fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s write update fw success\n", __func__);
		}

		/*start check fw update*/
		ret = kpdmcu_check_fw_area(ihid, count);
		if(ret) {
			i2c_hid_err(ihid, "%s start check fw update fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s start check fw update success\n", __func__);
		}

		/*check fw update*/
		ret = kpdmcu_write_fw_update(ihid, fw_data, count);
		if(ret) {
			i2c_hid_err(ihid, "%s check fw update fail\n", __func__);
			continue;
		} else {
			i2c_hid_err(ihid, "%s check fw update success\n", __func__);
		}

		/*reset mcu*/
		ret = kpdmcu_reset_mcu(ihid);
		if(ret) {
			i2c_hid_err(ihid, "%s reset mcu fail\n", __func__);
		} else {
			i2c_hid_err(ihid, "%s reset mcu success\n", __func__);
			msleep(1000);
			ihid->is_kpdmcu_need_fw_update = Check_kpdmcu_fw_update(ihid);
			break;
		}
	}

	return ret;
}

static int touchmcu_fw_update(struct i2c_hid *ihid, const unsigned char *fw_data, u32 count) {
	u8 flag = NO_NEED_UPDATE;
	bool is_flash_cfg_isp = false;
	int retry = 0;
	int ret = 0;

	flag = Get_touch_update_flag(fw_data, count);

	if (flag & NEED_UPDATE_FW) {
		retry = 0;
		do {
			ret = _touchmcu_fw_update(ihid, fw_data, count, flag, is_flash_cfg_isp);
			if (ret) {
				i2c_hid_err(ihid,"%s touch Update failed, retry %d, kb_connected %d\n",
					__func__, retry, kb_connected);
				if(kb_connected == false)
					return ret;
				msleep(200);
			} else {
				msleep(300);
				i2c_hid_err(ihid,"%s touch Update FW success\n", __func__);
				if(is_flash_cfg_isp || !(flag & NEED_UPDATE_CONFIG))
					ihid->kpd_fw_status = FW_UPDATE_SUC;
				break;
			}
		} while (retry++ < 3);
		if (ret) {
			return ret;
		}
	}

	if(is_flash_cfg_isp || !(flag & NEED_UPDATE_CONFIG))
		return ret;

	i2c_hid_err(ihid, "%s Update config interactively", __func__);
	retry = 0;
	do {
		ret = Touchmcu_cfg_update(ihid, fw_data, count);
		if (!ret) {
			i2c_hid_err(ihid, "%s Update cfg success\n", __func__);
			msleep(300);
			return ret;
		} else {
			if(kb_connected == false) {
				i2c_hid_err(ihid,"%s Update cfg failed, kb_connected %d\n", __func__, kb_connected);
				return ret;
			}
		}
		if (ret == CFG_NOT_MATCH) {
			i2c_hid_err(ihid, "%s No valid cfg data found for update\n", __func__);
		}
	} while (retry++ < 3);

	i2c_hid_err(ihid, "%s config update err:ret=%d\n", __func__, ret);
	return ret;
}

static int upload_pogopin_kevent_data(struct i2c_hid *ihid, unsigned char *payload)
{
	struct kernel_packet_info *user_msg_info;
	char log_tag[] = KEVENT_LOG_TAG;
	char event_id_pogopin[] = KEVENT_EVENT_ID;
	int len;
	int retry = 3;
	int ret = 0;

	user_msg_info = (struct kernel_packet_info *)kmalloc(
				sizeof(char) * POGOPIN_TRIGGER_MSG_LEN, GFP_KERNEL);

	if (!user_msg_info) {
		i2c_hid_err(ihid, "%s: Allocation failed\n", __func__);
		return -ENOMEM;
	}

	memset(user_msg_info, 0, POGOPIN_TRIGGER_MSG_LEN);
	strncpy(user_msg_info->payload, payload, MAX_POGOPIN_PAYLOAD_LEN);
	user_msg_info->payload[MAX_POGOPIN_PAYLOAD_LEN - 1] = '\0';
	len = strlen(user_msg_info->payload);

	user_msg_info->type = 1;
	strncpy(user_msg_info->log_tag, log_tag, MAX_POGOPIN_EVENT_TAG_LEN);
	user_msg_info->log_tag[MAX_POGOPIN_EVENT_TAG_LEN - 1] = '\0';
	strncpy(user_msg_info->event_id, event_id_pogopin, MAX_POGOPIN_EVENT_ID_LEN);
	user_msg_info->event_id[MAX_POGOPIN_EVENT_ID_LEN - 1] = '\0';
	user_msg_info->payload_length = len + 1;

	do {
		ret = fb_kevent_send_to_user(user_msg_info);
		if(!ret)
			break;
		msleep(POGOPIN_GET_SN_MS);
	} while (retry --);
	kfree(user_msg_info);

	return ret;
}

static int kpdmcu_sn_feedback(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	u8 send_cmd[4] = {0x04,0x00,0x00,0x40};
	u8 recv_buf[DEFAULT_SN_LEN] = {0};
	u8 send_mask_cmd[1] = {0x5a};
	int ret;
	int i;
	int sn_hide_bit_end = 0;
	int index = 0;
	int sn_length = 0;
	char report[MAX_POGOPIN_PAYLOAD_LEN];

	msleep(POGOPIN_GET_SN_MS);
	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_cmd, 4, NULL, 0);
	if (ret) {
		i2c_hid_err(ihid, "%s send cmd fail\n", __func__);
		return ret;
	}
	msleep(10);
	ret = hid_fw_update_cmd_send_recv(client, client->addr, send_mask_cmd, 1, recv_buf, DEFAULT_SN_LEN);
	if (ret || (recv_buf[0] != 0x1)) {
		i2c_hid_err(ihid, "%s send mask_cmd fail, recv_buf[0] = %d\n", __func__, recv_buf[0]);
		return ret < 0 ? ret : -EIO;
	}

	memset(report, 0, sizeof(report));
	ret = memcmp(ihid->report_sn, &recv_buf[1], DEFAULT_SN_LEN -1);
	if(ret == 0 && sn_report_count >= 2) {
		i2c_hid_err(ihid, "same sn ,not report\n");
		return 0;
	}
	index += snprintf(&report[index], MAX_POGOPIN_PAYLOAD_LEN - index, "$$sn@@");

	for(i = 0; i < DEFAULT_SN_LEN; i++) {
		if(recv_buf[i] != 0)
			sn_length++;
	}

	if (DEFAULT_SN_LEN == sn_length) {
		sn_hide_bit_end = KB_SN_HIDE_TWENTY_TWO_END;
	} else {
		sn_hide_bit_end = KB_SN_HIDE_SEVENTEEN_END;
	}

	for(i = KB_SN_HIDE_BIT_START; i < sn_hide_bit_end; i++)
			recv_buf[i] = KB_SN_HIDE_STAR_ASCII;

	for(i = 1; i < DEFAULT_SN_LEN; i++) {
		ihid->report_sn[i - 1] = recv_buf[i];
		index += snprintf(&report[index], MAX_POGOPIN_PAYLOAD_LEN - index, "%c", ihid->report_sn[i - 1]);
		fw_ihid->report_sn[i -1] = ihid->report_sn[i - 1];
	}
	i2c_hid_err(ihid, "pogopin report sn is %s\n", report);
	ret = upload_pogopin_kevent_data(ihid, report);
	if(ret)
		i2c_hid_err(ihid, "pogopin report sn err\n");
	if(sn_report_count < 2)
		sn_report_count ++;
	return ret;
}


static void kpdmcu_fw_mcu_version_thread(struct work_struct *work)
{
	struct i2c_hid *ihid = container_of(work, struct i2c_hid, kpdmcu_fw_mcu_version_work);
	int ret = 0;

	if(fw_ihid == NULL)
		return;

	mutex_lock(&ihid->fw_lock);
	fw_ihid->kpd_fw_status = FW_UPDATE_READY;
	fw_ihid->fw_update_progress = 0;
	fw_ihid->is_touchmcu_need_fw_update = Check_touchmcu_fw_update(fw_ihid);
	fw_ihid->is_kpdmcu_need_fw_update = Check_kpdmcu_fw_update(fw_ihid);
	ret = kpdmcu_sn_feedback(ihid);
	mutex_unlock(&ihid->fw_lock);
	if(ret)
		i2c_hid_err(ihid, "get sn report err\n");
}

static void kpdmcu_fw_data_version_thread(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct i2c_hid *ihid = container_of(dwork, struct i2c_hid, kpdmcu_fw_data_version_work);
	const struct firmware *fw_entry = NULL;
	u32 count;
	int i = 0;

	if(fw_ihid == NULL) {
		i2c_hid_err(ihid, "%s fw_ihid is NULL\n", __func__);
		return;
	}

	for(i=0; i < KEYBOARD_FIRMWARE_NUM; i++) {
		if(ihid->keyboard_firmware_name[i] == NULL)
			break;

		fw_entry = get_fw_firmware(ihid, ihid->keyboard_firmware_name[i]);

		if(fw_entry != NULL) {
			ihid->kpdmcu_fw_count[i] = (u32)(fw_entry->size / 1024);
			ihid->kpdmcu_fw_data_version[i] = (int)fw_entry->data[35];
		} else {
			i2c_hid_err(ihid, "%s kpd mcu request firmware fail\n", __func__);
			continue;
		}

		release_firmware(fw_entry);
		fw_entry = NULL;
	}

	fw_entry = get_fw_firmware(ihid, "TOUCH_MCU.bin");

	if(fw_entry != NULL) {
		count = (u32)(fw_entry->data[0] << 24 | fw_entry->data[1] << 16 | fw_entry->data[2] << 8 | fw_entry->data[3]);
		fw_ihid->touchmcu_fw_count = count / 1024;
		fw_ihid->touchmcu_fw_data_version = (int)(fw_entry->data[count+6+64] & 0xff);
		fw_ihid->touchfw_cid_version_major = (int)(fw_entry->data[GTX8_FW_IMAGE_VID_OFFSET]);
		fw_ihid->touchfw_cid_version_minor = (int)((fw_entry->data[GTX8_FW_IMAGE_VID_OFFSET + 1] << 8) |
								    fw_entry->data[GTX8_FW_IMAGE_VID_OFFSET + 2]);
	} else {
		i2c_hid_err(ihid, "%s kpd mcu request firmware fail\n", __func__);
		return;
	}

	release_firmware(fw_entry);
	fw_entry = NULL;

	mutex_lock(&fw_ihid->fw_lock);
	if(kb_connected) {
                if(fw_ihid->is_oneplus_keyboard_or_not)
                        fw_ihid->kpdmcu_fw_data_ver = ihid->kpdmcu_fw_data_version[1];
                else
                        fw_ihid->kpdmcu_fw_data_ver = ihid->kpdmcu_fw_data_version[0];
		fw_ihid->kpd_fw_status = FW_UPDATE_READY;
		fw_ihid->is_touchmcu_need_fw_update = Check_touchmcu_fw_update(fw_ihid);
		fw_ihid->is_kpdmcu_need_fw_update = Check_kpdmcu_fw_update(fw_ihid);
	}
	mutex_unlock(&fw_ihid->fw_lock);
}

static void kpdmcu_fw_update_thread(struct work_struct *work)
{
	struct i2c_hid *ihid = container_of(work, struct i2c_hid, kpdmcu_fw_update_work);
	const unsigned char *kpdmcu_firmware_data = NULL;
	struct i2c_client *client;
	u32 kpdmcu_fw_data_count;
	const struct firmware *kpdmcu_fw_entry = NULL;
	const unsigned char *touchmcu_firmware_data = NULL;
	u32 touchmcu_fw_data_count;
	u32 touchmcu_total_data_count;
	u32 touchmcu_cfg_data_count;
	const struct firmware *touchmcu_fw_entry = NULL;
	int ret = 0;
	int retry = 0;

	/*read counter bin*/
	if(ihid == NULL || keyboard_fw_ihid == NULL) {
		ihid->kpd_fw_status = FW_UPDATE_READY;
		return;
	}
	client = ihid->client;
	if(client == NULL) {
		ihid->kpd_fw_status = FW_UPDATE_READY;
		return;
	}

	if (ihid->pogopin_wakelock)
		__pm_stay_awake(ihid->pogopin_wakelock);
	mutex_lock(&ihid->fw_lock);

	ihid->kpd_fw_status = FW_UPDATE_START;
	ihid->kpdmcu_update_end = false;

	ihid->fw_update_progress = FW_PROGERSS_1 * FW_PERCENTAGE_100;

	if(ihid->is_kpdmcu_need_fw_update == false && ihid->kpdmcu_fw_update_force == false) {
		i2c_hid_err(ihid, "%s not need fw update kpdmcu\n", __func__);
	} else {
		if(ihid->is_oneplus_keyboard_or_not)
			kpdmcu_fw_entry = get_fw_firmware(ihid, keyboard_fw_ihid->keyboard_firmware_name[1]);
		else
			kpdmcu_fw_entry = get_fw_firmware(ihid, keyboard_fw_ihid->keyboard_firmware_name[0]);
		if(kpdmcu_fw_entry != NULL) {
			kpdmcu_fw_data_count = (u32)kpdmcu_fw_entry->size;
			kpdmcu_firmware_data = kpdmcu_fw_entry->data;
			i2c_hid_err(ihid, "%s kpd_mcu fw count 0X%x\n", __func__, kpdmcu_fw_data_count);
		} else {
			i2c_hid_err(ihid, "%s kpd fw request firmware fail\n", __func__);
			ihid->kpd_fw_status = FW_UPDATE_FAIL;
			goto out3;
		}

		ihid->fw_update_progress = FW_PROGRESS_2 * FW_PERCENTAGE_100;

		ret = kpdmcu_fw_update(ihid, kpdmcu_firmware_data, kpdmcu_fw_data_count);

		if(ret) {
			ihid->kpd_fw_status = FW_UPDATE_FAIL;
			goto out2;
		}
		ihid->kpdmcu_update_end = true;
	}

	if(ihid->is_touchmcu_need_fw_update == false && ihid->kpdmcu_fw_update_force == false) {
		i2c_hid_err(ihid, "%s not need fw update touch mcu\n", __func__);
		ihid->kpd_fw_status = FW_UPDATE_SUC;
		ihid->fw_update_progress = FW_PROGRESS_100 * FW_PERCENTAGE_100;
		goto out2;
	}

	touchmcu_fw_entry = get_fw_firmware(ihid,  "TOUCH_MCU.bin");
	if(touchmcu_fw_entry != NULL) {
		touchmcu_total_data_count = (u32)touchmcu_fw_entry->size;
		touchmcu_firmware_data = touchmcu_fw_entry->data;
		touchmcu_fw_data_count = (u32)(touchmcu_firmware_data[0] << 24 | touchmcu_firmware_data[1] << 16 | touchmcu_firmware_data[2] << 8 | touchmcu_firmware_data[3]);
		touchmcu_cfg_data_count = (u32)((touchmcu_firmware_data[touchmcu_fw_data_count+6]<<8)+touchmcu_firmware_data[touchmcu_fw_data_count+7]);
		if(touchmcu_total_data_count-touchmcu_fw_data_count-6 != touchmcu_cfg_data_count+6) {
			i2c_hid_err(ihid, "%s error touch_fw count total:0X%x, fw_count:0X%x, cfg_count:0X%x\n",
						    __func__, touchmcu_total_data_count, touchmcu_fw_data_count, touchmcu_cfg_data_count);
			ihid->kpd_fw_status = FW_UPDATE_FAIL;
			goto out1;
		} else {
			i2c_hid_err(ihid, "&s touch_fw count total:0X%x, fw_count:0X%x, cfg_count:0X%x\n",
						    __func__, touchmcu_total_data_count, touchmcu_fw_data_count, touchmcu_cfg_data_count);
		}
	} else {
			i2c_hid_err(ihid, "%s touch fw request firmware fail\n", __func__);
			ihid->kpd_fw_status = FW_UPDATE_FAIL;
			goto out2;
	}

	do {
		ret = touchmcu_fw_update(ihid, touchmcu_firmware_data, touchmcu_fw_data_count);
		ihid->is_touchmcu_need_fw_update = Check_touchmcu_fw_update(ihid);
		if(ret || ihid->is_touchmcu_need_fw_update) {
			i2c_hid_err(ihid, "%s touch fw update fail, retry is %d\n", __func__, retry);
			if(retry >= 3)
				ihid->kpd_fw_status = FW_UPDATE_FAIL;
		} else {
			ihid->kpd_fw_status = FW_UPDATE_SUC;
			ihid->fw_update_progress = FW_PROGRESS_100 * FW_PERCENTAGE_100;
			break;
		}
	} while (retry++ < 3);

out1:
	release_firmware(touchmcu_fw_entry);
	touchmcu_fw_entry = NULL;
	touchmcu_firmware_data = NULL;

out2:
	release_firmware(kpdmcu_fw_entry);
	kpdmcu_fw_entry = NULL;
	kpdmcu_firmware_data = NULL;
out3:
	mutex_unlock(&ihid->fw_lock);
	if (ihid->pogopin_wakelock)
		__pm_relax(ihid->pogopin_wakelock);
}


static void padmcu_fw_update_thread(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct i2c_hid *ihid = container_of(dwork, struct i2c_hid, padmcu_fw_update_work);
	struct i2c_client *client;
	const unsigned char *firmware_data = NULL;
	unsigned int fw_data_count;
	const struct firmware *fw_entry = NULL;
	int ret;

	/*read counter bin*/
	if(ihid == NULL) {
		return;
	}
	client = ihid->client;
	if(client == NULL) {
		ihid->kpd_fw_status = FW_UPDATE_READY;
		return;
	}

	fw_entry = get_fw_firmware(ihid, "PAD_MCU.bin");
	if(fw_entry != NULL) {
		fw_data_count = (u32)fw_entry->size;
		firmware_data = fw_entry->data;
		ihid->padmcu_fw_data_version = (int)fw_entry->data[35];
		i2c_hid_err(ihid, "%s pd_mcu count 0X%x, version is 0X%x\n", __func__, fw_data_count, ihid->padmcu_fw_data_version);
	} else {
		i2c_hid_err(ihid, "%s padmcu request firmware fail\n", __func__);
		return;
	}

	if (ihid->pogopin_wakelock)
		__pm_stay_awake(ihid->pogopin_wakelock);
	mutex_lock(&ihid->fw_lock);

	ret = padmcu_read_panel_mcu_fw(ihid);

	if((ihid->padmcu_fw_data_version != ihid->padmcu_fw_mcu_version) || ihid->padmcu_fw_update_force) {
		ret = padmcu_fw_update(ihid, firmware_data, fw_data_count);
		if(ret) {
			i2c_hid_err(ihid, "%s update pad_mcu fw fail!\n", __func__);
		} else {
			i2c_hid_err(ihid, "%s update pad_mcu fw success!\n", __func__);
		}
	}

	mutex_unlock(&ihid->fw_lock);
	if (ihid->pogopin_wakelock)
		__pm_relax(ihid->pogopin_wakelock);
	release_firmware(fw_entry);
	fw_entry = NULL;
	firmware_data = NULL;
}

static int i2c_hid_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
	int ret;
	char *wake_name = NULL;
	const char *device_name =NULL;
	struct i2c_hid *ihid;
	struct hid_device *hid;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pogopin_rst_low;
	struct pinctrl_state *pogopin_rst_high;
	__u16 hidRegister;
	u32 fw_work_delay;

	dev_err(&client->dev, "HID probe called for i2c 0x%02x\n", client->addr);

	if (!client->irq) {
		dev_err(&client->dev, "HID over i2c has not been provided an Int IRQ\n");
		return -EINVAL;
	}

	if (client->irq < 0) {
		if (client->irq != -EPROBE_DEFER)
			dev_err(&client->dev, "HID over i2c doesn't have a valid IRQ\n");
		return client->irq;
	}

	ihid = devm_kzalloc(&client->dev, sizeof(*ihid), GFP_KERNEL);
	if (!ihid)
		return -ENOMEM;

	if (client->dev.of_node) {
		ret = i2c_hid_of_probe(client, &ihid->pdata);
		if (ret)
			return ret;
	} else {
		dev_err(&client->dev, "no devicetree defined, exit.\n");
		return -1;
	}

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&client->dev, "%s: get config pinctrl fail\n", __func__);
	} else {
		pogopin_rst_low = pinctrl_lookup_state(pinctrl, "pogopin_rst_low");
		if (IS_ERR(pogopin_rst_low)) {
			dev_err(&client->dev, "%s: get pogopin_rst_low pinctrl state fail\n", __func__);
		} else {
			pinctrl_select_state(pinctrl, pogopin_rst_low);
			dev_info(&client->dev, "%s :pull rst pinctrl to low\n", __func__);
			msleep(1);
			pogopin_rst_high = pinctrl_lookup_state(pinctrl, "pogopin_rst_high");
			if (IS_ERR(pogopin_rst_high)) {
				dev_err(&client->dev, "%s :get pogopin_rst_high pinctrl state fail\n", __func__);
			} else {
				pinctrl_select_state(pinctrl, pogopin_rst_high);
				dev_info(&client->dev, "%s: pull rst pinctrl to high\n", __func__);
			}
		}
	}
	/* Parse platform agnostic common properties from ACPI / device tree */
	i2c_hid_fwnode_probe(client, &ihid->pdata);

	ihid->pdata.supplies[0].supply = "vdd";
	ihid->pdata.supplies[1].supply = "vddl";

	ret = devm_regulator_bulk_get(&client->dev,
				      ARRAY_SIZE(ihid->pdata.supplies),
				      ihid->pdata.supplies);
	if (ret) {
		dev_err(&client->dev, "regulator get failed, exit!\n");
		return ret;
	}

	ret = regulator_bulk_enable(ARRAY_SIZE(ihid->pdata.supplies),
				    ihid->pdata.supplies);
	if (ret < 0) {
		dev_err(&client->dev, "regulator enable failed, exit!\n");
		return ret;
	}

	if (ihid->pdata.post_power_delay_ms) {
		dev_err(&client->dev, "enter sleep %d ms\n", ihid->pdata.post_power_delay_ms);
		msleep(ihid->pdata.post_power_delay_ms);
	}

	i2c_set_clientdata(client, ihid);

	ihid->client = client;

	hidRegister = ihid->pdata.hid_descriptor_address;
	ihid->wHIDDescRegister = cpu_to_le16(hidRegister);

	init_waitqueue_head(&ihid->wait);
	mutex_init(&ihid->reset_lock);

	/* we need to allocate the command buffer without knowing the maximum
	 * size of the reports. Let's use HID_MIN_BUFFER_SIZE, then we do the
	 * real computation later. */
	ret = i2c_hid_alloc_buffers(ihid, HID_MIN_BUFFER_SIZE);
	if (ret < 0) {
		dev_err(&client->dev, "hid alloc buffer failed, exit!\n");
		goto err_regulator;
	}

	device_init_wakeup(&client->dev, true);
	device_enable_async_suspend(&client->dev);

	/* Make sure there is something at this address */
	ret = i2c_smbus_read_byte(client);
	if (ret < 0) {
		dev_err(&client->dev, "nothing at this address: %d\n", ret);
		ret = -ENXIO;
		goto err_regulator;
	}

	ret = i2c_hid_fetch_hid_descriptor(ihid);
	if (ret < 0) {
		dev_err(&client->dev, "hid fetch descriptor failed, exit!\n");
		goto err_regulator;
	}

	ret = i2c_hid_init_irq(client);
	if (ret < 0) {
		dev_err(&client->dev, "hid init irq failed, exit!\n");
		goto err_regulator;
	}

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		dev_err(&client->dev, "hid alloc device failed, exit!\n");
		ret = PTR_ERR(hid);
		goto err_irq;
	}

	ihid->hid = hid;

	hid->driver_data = client;
	hid->ll_driver = &i2c_hid_ll_driver;
	hid->dev.parent = &client->dev;
	hid->bus = BUS_I2C;
	hid->version = le16_to_cpu(ihid->hdesc.bcdVersion);
	hid->vendor = le16_to_cpu(ihid->hdesc.wVendorID);
	hid->product = le16_to_cpu(ihid->hdesc.wProductID);

	if (!device_property_read_string(&client->dev, "dev_name", &device_name)) {
		snprintf(hid->name, sizeof(hid->name), "%s",device_name);
	} else {
		snprintf(hid->name, sizeof(hid->name), "%s %04X:%04X",
			 client->name, (u16)hid->vendor, (u16)hid->product);
	}
	strlcpy(hid->phys, dev_name(&client->dev), sizeof(hid->phys));

	ihid->quirks = i2c_hid_lookup_quirk(hid->vendor, hid->product);
	if (ihid->quirks) {
		i2c_hid_err(ihid, "%s: get quirks 0x%04x\n", __func__, ihid->quirks);
	}

	mutex_lock(&desc_lock);
	ret = hid_add_device(hid);
	mutex_unlock(&desc_lock);
	if (ret) {
		if (ret != -ENODEV)
			hid_err(client, "can't add hid device: %d\n", ret);
		goto err_mem_free;
	}

	hid_pogopin_parse_dts(ihid);
	init_i2c_hid_proc(ihid);
	wake_name = devm_kasprintf(&hid->dev, GFP_KERNEL, "%s", "pogo fw wakelock");
	ihid->pogopin_wakelock = wakeup_source_register(NULL, wake_name);
	dev_err(&client->dev, "HID probe end for i2c 0x%02x\n", client->addr);

	ihid->padmcu_fw_update_force = false;
	ihid->kpdmcu_fw_update_force = false;
	mutex_init(&ihid->fw_lock);
	if(ihid->pogopin_detect_check) {
		keyboard_fw_ihid = ihid;
		INIT_WORK(&ihid->kpdmcu_fw_mcu_version_work, kpdmcu_fw_mcu_version_thread);
		INIT_DELAYED_WORK(&ihid->kpdmcu_fw_data_version_work, kpdmcu_fw_data_version_thread);
		schedule_delayed_work(&ihid->kpdmcu_fw_data_version_work, round_jiffies_relative(msecs_to_jiffies(6 * MS_TO_S)));
	}
	if(ihid->pogopin_fw_support) {
		fw_ihid = ihid;
		ret = padmcu_read_panel_mcu_fw(ihid);
		INIT_DELAYED_WORK(&ihid->padmcu_fw_update_work, padmcu_fw_update_thread);
		INIT_WORK(&ihid->kpdmcu_fw_update_work, kpdmcu_fw_update_thread);
		if(ihid->padmcu_fw_mcu_version != 0x01) {
			fw_work_delay = 3;
			schedule_delayed_work(&ihid->padmcu_fw_update_work, round_jiffies_relative(msecs_to_jiffies(fw_work_delay * MS_TO_S)));
		} else {
			dev_err(&client->dev, "PAD MCU fw version is 0x1, please update fw using nodes\n", client->addr);
		}
	}
	return 0;

err_mem_free:
	hid_destroy_device(hid);

err_irq:
	free_irq(client->irq, ihid);

err_regulator:
	regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies),
			       ihid->pdata.supplies);
	i2c_hid_free_buffers(ihid);
	return ret;
}

static int i2c_hid_remove(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid;

	hid = ihid->hid;
	hid_destroy_device(hid);

	free_irq(client->irq, ihid);

	if (ihid->bufsize)
		i2c_hid_free_buffers(ihid);

	regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies),
			       ihid->pdata.supplies);

	return 0;
}

static void i2c_hid_shutdown(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
	free_irq(client->irq, ihid);
}

#ifdef CONFIG_PM_SLEEP
static int i2c_hid_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid = ihid->hid;
	int ret;
	int wake_status;

	i2c_hid_err(ihid, "%s: enter.\n", __func__);
	if (hid->driver && hid->driver->suspend) {
		ret = hid->driver->suspend(hid, PMSG_SUSPEND);
		if (ret < 0)
			return ret;
	}

	/* Save some power */
	i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);

	disable_irq(client->irq);

	if (device_may_wakeup(&client->dev)) {
		wake_status = enable_irq_wake(client->irq);
		if (!wake_status)
			ihid->irq_wake_enabled = true;
		else
			hid_warn(hid, "Failed to enable irq wake: %d\n", wake_status);
	} else {
		regulator_bulk_disable(ARRAY_SIZE(ihid->pdata.supplies), ihid->pdata.supplies);
	}

	return 0;
}

static int i2c_hid_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid = ihid->hid;
	int wake_status;
	int tries = 3;

	i2c_hid_err(ihid, "%s: enter.\n", __func__);
	if (!device_may_wakeup(&client->dev)) {
		ret = regulator_bulk_enable(ARRAY_SIZE(ihid->pdata.supplies),
					    ihid->pdata.supplies);
		if (ret)
			hid_warn(hid, "Failed to enable supplies: %d\n", ret);

		if (ihid->pdata.post_power_delay_ms)
			msleep(ihid->pdata.post_power_delay_ms);
	} else if (ihid->irq_wake_enabled) {
		wake_status = disable_irq_wake(client->irq);
		if (!wake_status)
			ihid->irq_wake_enabled = false;
		else
			hid_warn(hid, "Failed to disable irq wake: %d\n", wake_status);
	}

	enable_irq(client->irq);

	/* wait for hid irq handle completed */
	msleep(1);
	synchronize_irq(client->irq);

	/* Instead of resetting device, simply powers the device on. This
	 * solves "incomplete reports" on Raydium devices 2386:3118 and
	 * 2386:4B33 and fixes various SIS touchscreens no longer sending
	 * data after a suspend/resume.
	 *
	 * However some ALPS touchpads generate IRQ storm without reset, so
	 * let's still reset them here.
	 */
	if (ihid->quirks & I2C_HID_QUIRK_RESET_ON_RESUME) {
		do {
			ret = i2c_hid_hwreset(client);
			if (ret)
				msleep(10);
		} while (tries-- > 0 && ret);
	} else {
		ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);
	}

	if (ret)
		return ret;

	if (hid->driver && hid->driver->reset_resume) {
		ret = hid->driver->reset_resume(hid);
		return ret;
	}

	return 0;
}
#endif

static const struct dev_pm_ops i2c_hid_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(i2c_hid_suspend, i2c_hid_resume)
};

static const struct i2c_device_id i2c_hid_id_table[] = {
	{ "hid", 0 },
	{ "hid-over-i2c", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, i2c_hid_id_table);


static struct i2c_driver i2c_hid_driver = {
	.driver = {
		.name	= "i2c_hid",
		.pm	= &i2c_hid_pm,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.acpi_match_table = ACPI_PTR(i2c_hid_acpi_match),
		.of_match_table = of_match_ptr(i2c_hid_of_match),
	},

	.probe		= i2c_hid_probe,
	.remove		= i2c_hid_remove,
	.shutdown	= i2c_hid_shutdown,
	.id_table	= i2c_hid_id_table,
};

module_i2c_driver(i2c_hid_driver);

MODULE_DESCRIPTION("HID over I2C core driver");
MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_LICENSE("GPL");
