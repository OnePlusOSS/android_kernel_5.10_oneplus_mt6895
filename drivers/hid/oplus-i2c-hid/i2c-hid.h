/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef I2C_HID_H
#define I2C_HID_H

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/i2c-hid.h>

struct i2c_hid_desc {
	__le16 wHIDDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wReportDescRegister;
	__le16 wInputRegister;
	__le16 wMaxInputLength;
	__le16 wOutputRegister;
	__le16 wMaxOutputLength;
	__le16 wCommandRegister;
	__le16 wDataRegister;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le32 reserved;
} __packed;

struct i2c_hid_cmd {
	unsigned int registerIndex;
	__u8 opcode;
	unsigned int length;
	bool wait;
};

union command {
	u8 data[0];
	struct cmd {
		__le16 reg;
		__u8 reportTypeID;
		__u8 opcode;
	} __packed c;
};

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct i2c_hid_cmd hid_get_idle_cmd = { I2C_HID_CMD(0x04) };
 * static const struct i2c_hid_cmd hid_set_idle_cmd = { I2C_HID_CMD(0x05) };
 * static const struct i2c_hid_cmd hid_get_protocol_cmd = { I2C_HID_CMD(0x06) };
 * static const struct i2c_hid_cmd hid_set_protocol_cmd = { I2C_HID_CMD(0x07) };
 */

/* The main device structure */
#define PAGESIZE		512
#define MAX_FW_NAME_LENGTH	60
#define ONE_WRITY_LEN_MIN	16
#define ONE_WRITY_LEN_MAX	64
#define PROC_PAGE_LEN		30
#define CCITT			0x1021
#define MS_TO_S			1000
#define POGOPIN_INFO		"pogopin"
#define VERSION_LEN		72

#define GTX8_FW_IMAGE_PID_OFFSET	15
#define GTX8_FW_IMAGE_PID_LEN		8
#define GTX8_FW_IMAGE_VID_OFFSET	24
#define GTX8_FW_IMAGE_CID_OFFSET	23
#define GTX8_FW_IMAGE_SUB_FWNUM_OFFSET	27
#define GTX8_SUB_FW_INFO_OFFSET		32
#define GTX8_SUB_FW_DATA_OFFSET		256

#define GDIX_RETRY_TIMES		6
#define RAM_BUFFER_SIZE			4096
#define GDIX_PRE_HEAD_LEN		5
#define GDIX_DATA_HEAD_LEN		5
#define GDIX_OUT_PUT_REPORT_SIZE	65

#define CFG_FLASH_ADDR		0x1E000
#define CFG_START_ADDR		0X60DC
#define CMD_ADDR		0x60cc
#define VER_ADDR		0x452C
#define FLASH_BUFFER_ADDR	0xc000
#define FLASH_RESULT_ADDR	0x5096
#define BL_STATE_ADDR		0x5095
#define TOUCH_SENSOR_ID		0xF
#define CFG_NOT_MATCH		99

#define KEVENT_LOG_TAG              "psw_bsp_pogopin"
#define KEVENT_EVENT_ID             "pogopin_sn_report"
#define DEFAULT_SN_LEN              23
#define POGOPIN_TRIGGER_MSG_LEN     2048
#define MAX_POGOPIN_EVENT_TAG_LEN   32
#define MAX_POGOPIN_EVENT_ID_LEN    20
#define MAX_POGOPIN_PAYLOAD_LEN     1024
#define POGOPIN_GET_SN_MS           500

#define KEYBOARD_FIRMWARE_NUM       2
#define KEYBOARD_BRAND_MASK         0x30
#define KEYBOARD_BRAND_ONEPLUE      0x2
#define KEYBOARD_BRAND_SHIFT        4

#define KEYBOARD_TOUCH_MASK         0x40
#define KEYBOARD_TOUCH_BIT          6

#define FW_PROGERSS_1		1
#define FW_PROGRESS_2		2
#define FW_PROGRESS_3		3
#define FW_PROGRESS_5		5
#define FW_PROGRESS_22		22
#define FW_PROGRESS_25		25
#define FW_PROGRESS_47		47
#define FW_PROGRESS_48		48
#define FW_PROGRESS_50		50
#define FW_PROGRESS_93		93
#define FW_PROGRESS_99		99
#define FW_PROGRESS_100		100
#define FW_PERCENTAGE_100	100

struct i2c_hid {
	struct i2c_client	*client;	/* i2c client */
	struct hid_device	*hid;	/* pointer to corresponding HID dev */
	union {
		__u8 hdesc_buffer[sizeof(struct i2c_hid_desc)];
		struct i2c_hid_desc hdesc;	/* the HID Descriptor */
	};
	__le16			wHIDDescRegister; /* location of the i2c
						   * register of the HID
						   * descriptor. */
	unsigned int		bufsize;	/* i2c buffer size */
	u8			*inbuf;		/* Input buffer */
	u8			*rawbuf;	/* Raw Input buffer */
	u8			*cmdbuf;	/* Command buffer */
	u8			*argsbuf;	/* Command arguments buffer */

	unsigned long		flags;		/* device flags */
	unsigned long		quirks;		/* Various quirks */

	wait_queue_head_t	wait;		/* For waiting the interrupt */

	struct i2c_hid_platform_data pdata;

	bool			irq_wake_enabled;
	struct mutex		reset_lock;

	unsigned long		sleep_delay;
	struct delayed_work padmcu_fw_update_work;
	struct work_struct  kpdmcu_fw_update_work;
	struct delayed_work kpdmcu_fw_data_version_work;
	struct work_struct  kpdmcu_fw_mcu_version_work;
	int padmcu_fw_mcu_version;
	int padmcu_fw_data_version;
	int kpdmcu_fw_mcu_version;
	int kpdmcu_fw_data_version[KEYBOARD_FIRMWARE_NUM];
	int kpdmcu_fw_data_ver;
	int touchmcu_fw_mcu_version;
	int touchmcu_fw_data_version;
	int touchmcu_cid_version_major;
	int touchmcu_cid_version_minor;
	int touchfw_cid_version_major;
	int touchfw_cid_version_minor;
	int fw_update_progress;
	bool kpdmcu_update_end;
	bool padmcu_fw_update_force;
	bool kpdmcu_fw_update_force;
	bool pogopin_fw_support;
	u16 hid_fw_address;
	struct wakeup_source *pogopin_wakelock;
	struct mutex fw_lock;
	u8 kpd_fw_status;
	bool is_touchmcu_need_fw_update;
	bool is_kpdmcu_need_fw_update;
	u32 kpdmcu_fw_count[KEYBOARD_FIRMWARE_NUM];
	u32 kpdmcu_fw_cnt;
	u32 touchmcu_fw_count;
	u8 keyboard_touch_status;
	u8 report_sn[DEFAULT_SN_LEN - 1];
	bool pogopin_detect_check;
	const char *keyboard_firmware_name[KEYBOARD_FIRMWARE_NUM];
	const char *keyboard_dev_name[KEYBOARD_FIRMWARE_NUM];
	bool is_oneplus_keyboard_or_not;
};

//update type
enum updateFlag {
	NO_NEED_UPDATE = 0,
	NEED_UPDATE_FW = 1,
	NEED_UPDATE_CONFIG = 2,
	NEED_UPDATE_CONFIG_WITH_ISP = 0x10,
	NEED_UPDATE_HID_SUBSYSTEM = 0x80,
};

enum updateStatus{
	FW_UPDATE_READY = 0,
	FW_UPDATE_START,
	FW_UPDATE_FAIL,
	FW_UPDATE_SUC,
};

/* quirks to control the device */
#define I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV	BIT(0)
#define I2C_HID_QUIRK_NO_IRQ_AFTER_RESET	BIT(1)
#define I2C_HID_QUIRK_BOGUS_IRQ			BIT(4)
#define I2C_HID_QUIRK_RESET_ON_RESUME		BIT(5)
#define I2C_HID_QUIRK_BAD_INPUT_SIZE		BIT(6)
#define I2C_HID_QUIRK_NO_WAKEUP_AFTER_RESET	BIT(7)

/* flags */
#define I2C_HID_STARTED		0
#define I2C_HID_RESET_PENDING	1
#define I2C_HID_READ_PENDING	2

#define I2C_HID_PWR_ON		0x00
#define I2C_HID_PWR_SLEEP	0x01

#define OPLUS_KEYBOARD_VID 0x22D9
#define OPLUS_KEYBOARD_PID 0x3868
#define OPLUS_TOUCHPAD_VID 0x22D9
#define OPLUS_TOUCHPAD_PID 0x3869

static const struct i2c_hid_quirks {
	__u16 idVendor;
	__u16 idProduct;
	__u32 quirks;
} i2c_hid_quirks[] = {
	{ USB_VENDOR_ID_WEIDA, HID_ANY_ID,I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV },
	{ I2C_VENDOR_ID_HANTICK, I2C_PRODUCT_ID_HANTICK_5288,I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ I2C_VENDOR_ID_ITE, I2C_DEVICE_ID_ITE_VOYO_WINPAD_A15, I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ I2C_VENDOR_ID_RAYDIUM, I2C_PRODUCT_ID_RAYDIUM_3118, I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ USB_VENDOR_ID_ALPS_JP, HID_ANY_ID, I2C_HID_QUIRK_RESET_ON_RESUME },
	{ I2C_VENDOR_ID_SYNAPTICS, I2C_PRODUCT_ID_SYNAPTICS_SYNA2393, I2C_HID_QUIRK_RESET_ON_RESUME },
	{ USB_VENDOR_ID_ITE, I2C_DEVICE_ID_ITE_LENOVO_LEGION_Y720, I2C_HID_QUIRK_BAD_INPUT_SIZE },
	/* * Sending the wakeup after reset actually break ELAN touchscreen controller */
	{ USB_VENDOR_ID_ELAN, HID_ANY_ID, I2C_HID_QUIRK_NO_WAKEUP_AFTER_RESET | I2C_HID_QUIRK_BOGUS_IRQ },
	{ OPLUS_KEYBOARD_VID, OPLUS_KEYBOARD_PID, I2C_HID_QUIRK_RESET_ON_RESUME | I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ OPLUS_TOUCHPAD_VID, OPLUS_TOUCHPAD_PID, I2C_HID_QUIRK_RESET_ON_RESUME | I2C_HID_QUIRK_NO_IRQ_AFTER_RESET },
	{ 0, 0 }
};

static inline struct i2c_hid_desc
		   *i2c_hid_get_dmi_i2c_hid_desc_override(uint8_t *i2c_name)
{ return NULL; }
static inline char *i2c_hid_get_dmi_hid_report_desc_override(uint8_t *i2c_name,
							     unsigned int *size)
{ return NULL; }

#endif
