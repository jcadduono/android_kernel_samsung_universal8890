#ifndef _LINUX_MMS100S_TOUCH_H_
#define _LINUX_MMS100S_TOUCH_H_


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>
#include <linux/wakelock.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sec_sysfs.h>

#include <linux/i2c/mms_ts.h>

#ifdef CONFIG_SEC_DEBUG_TSP_LOG
#include <linux/sec_debug.h>
#endif

#define USE_OPEN_CLOSE

/* Support Runtime Config : Only for mms126s, Not enough memory in IC */
#define MMS_RUNTIME_CONF

#define MMS_DEV_NAME		"mms100s"
#define MMS_DEFAULT_UMS_FW	"/sdcard/Firmware/TSP/mms.fw"
#define MMS_DEFAULT_UMS_FW_CONF	"/sdcard/Firmware/TSP/mms_config.fw"

/* Flag */
#define MMS_HAS_TOUCH_KEY		0
#define INFORM_CHARGER			0
#define FLASH_VERBOSE_DEBUG		1
#define SEC_TSP_FACTORY_TEST

#define CHECK_CONNECTOR_PROBE

#define MAX_SECTION_NUM			3
#define MAX_FINGER_NUM			2
#define MAX_MAJOR				30	// bring up!!
#define MAX_MINOR				30	// bring up!!
//#define MAX_WIDTH				30
//#define MAX_PRESSURE			255
#define MAX_LOG_LENGTH			128
#define FINGER_EVENT_SZ			8
#define ESD_DETECT_COUNT		10

/*
 * ISC_XFER_LEN	- ISC unit transfer length.
 * Give number of 2 power n, where  n is between 2 and 10
 * i.e. 4, 8, 16 ,,, 1024
 */
#define ISC_XFER_LEN			1024
#define MMS_FLASH_PAGE_SZ		1024
#define ISC_BLOCK_NUM			(MMS_FLASH_PAGE_SZ / ISC_XFER_LEN)

/* Registers */
#define MMS_MODE_CONTROL		0x01
#define MMS_THRESHOLD			0x05
#define MMS_TX_NUM				0x0B
#define MMS_RX_NUM				0x0C
#define MMS_KEY_NUM				0x0D
#define MMS_REF_TRACK_LEVEL		0x0E
#define MMS_EVENT_PKT_SZ		0x0F
#define MMS_EVENT_PKT			0x10
#define MMS_OPERATION_MODE		0x30
#define MMS_UNIVERSAL_CMD		0xA0
#define MMS_UNIVERSAL_RESULT_SZ	0xAE
#define MMS_UNIVERSAL_RESULT	0xAF
#define MMS_CMD_ENTER_ISC		0x5F
#define MMS_BL_VERSION			0xE1
#define MMS_CORE_VERSION		0xE2
#define MMS_CONFIG_VERSION		0xE3
#define MMS_POWER_CONTROL		0xB0
#define MMS_VENDOR_ID			0xC0
#define MMS_HW_ID				0xC2

/* Universal commands */
#define MMS_CMD_SET_LOG_MODE	0x20
#define MMS_UNIV_ENTER_TEST		0x40
#define MMS_UNIV_CM_DELTA		0x41
#define MMS_UNIV_GET_DELTA		0x42
#define MMS_UNIV_CM_ABS			0x43
#define MMS_UNIV_CM_JITTER		0x45
#define MMS_UNIV_GET_DELTA_KEY	0x4A
#define MMS_UNIV_GET_ABS_KEY	0x4B
#define MMS_UNIV_GET_JITTER_KEY	0x4C
#define MMS_UNIV_EXIT_TEST		0x4F
#define MMS_UNIV_INTENSITY		0x70
#define MMS_UNIV_INTENSITY_KEY	0x71
#define MMS_UNIV_REFERENCE		0x72

/* Event types */
#define MMS_TEST_MODE			0X0C
#define MMS_NOTIFY_EVENT		0x0E
#define MMS_ERROR_EVENT			0x0F
#define MMS_INPUT_EVENT			0x10
#define MMS_TOUCH_KEY_EVENT		0x40

#define MMS_HW_INIT_TIME		40

#ifdef SEC_TSP_FACTORY_TEST
#define TSP_CMD_STR_LEN			32
#define TSP_CMD_RESULT_STR_LEN	512
#define TSP_CMD_PARAM_NUM		8
#define TSP_CMD_FULL_VER_LEN	9
#endif

enum {
	SYS_TXNUM	= 3,
	SYS_RXNUM,
	SYS_CLEAR,
	SYS_ENABLE,
	SYS_DISABLE,
	SYS_INTERRUPT,
	SYS_RESET,
};

enum {
	GET_RX_NUM	= 1,
	GET_TX_NUM,
	GET_EVENT_DATA,
};

enum {
	LOG_TYPE_U08	= 2,
	LOG_TYPE_S08,
	LOG_TYPE_U16,
	LOG_TYPE_S16,
	LOG_TYPE_U32	= 8,
	LOG_TYPE_S32,
};

enum {
	ISC_ADDR				= 0xD5,
	ISC_CMD_READ_STATUS		= 0xD9,
	ISC_CMD_READ			= 0x4000,
	ISC_CMD_EXIT			= 0x8200,
	ISC_CMD_PAGE_ERASE		= 0xC000,
	ISC_CMD_ERASE			= 0xC100,
	ISC_PAGE_ERASE_DONE		= 0x10000,
	ISC_PAGE_ERASE_ENTER	= 0x20000,
};

enum {
	SEC_NONE = -1,
	SEC_BOOT = 0,
	SEC_CORE,
	SEC_CONF,
	SEC_LIMIT
};

enum {
	FW_FROM_BUILT_IN = 0,
	FW_FROM_UMS,
};

enum {
	COMPARE_UPDATE = 0,
	FORCED_UPDATE,
};

#ifdef MMS_RUNTIME_CONF
/* Universal commands */
#define MMS_CMD_CONTROL		0x22
#define MMS_SUBCMD_START	0x80

/* Firmware Start Control */
#define RUN_START		0
#define RUN_STOP		1

/* mfsp offset */
#define MMS_MFSP_OFFSET		16

/* Runtime config */
#define MMS_RUN_CONF_POINTER	0xA1
#define MMS_GET_RUN_CONF		0xA2
#define MMS_SET_RUN_CONF		0xA3

#define MMS_GET_CONF_VER	0x01


enum {
	MMS_RUN_TYPE_SINGLE	= 1,
	MMS_RUN_TYPE_ARRAY,
	MMS_RUN_TYPE_END,
	MMS_RUN_TYPE_INFO,
	MMS_RUN_TYPE_UNKNOWN,
};

struct mms_config_hdr {
	char	mark[4];
	char	tag[4];

	u32	core_version;
	u32	config_version;
	u32	data_offset;
	u32	data_count;

	u32	reserved0;
	u32	info_offset;
	u32	reserved2;
	u32	reserved3;
	u32	reserved4;
	u32	reserved5;

} __attribute__ ((packed));

struct mms_config_item {
	u16	type;
	u16	category;
	u16	offset;
	u16	datasize;
	u16	data_blocksize;
	u16	reserved;
	u32     value;
} __attribute__ ((packed));

#endif


#ifdef SEC_TSP_FACTORY_TEST
enum {
	CMD_STATUS_WAITING = 0,
	CMD_STATUS_RUNNING,
	CMD_STATUS_OK,
	CMD_STATUS_FAIL,
	CMD_STATUS_NOT_APPLICABLE,
};

struct mms_fac_data {
	struct device		*fac_tsp_dev;
	struct list_head	cmd_list_head;
	u8					cmd_state;
	char				cmd[TSP_CMD_STR_LEN];
	char				cmd_buff[TSP_CMD_STR_LEN];
	int					cmd_buffer_size;
	int					cmd_param[TSP_CMD_PARAM_NUM];
	char				cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex		cmd_lock;
	bool				cmd_is_running;
	u8					rx_num;
	u8					tx_num;
#if MMS_HAS_TOUCH_KEY
	struct device		*fac_key_dev;
#endif
	s16 *reference;
	s16 *delta;
	s16 *abs;
	s16 *intensity;
	int max;
	int min;
};
#endif /* SEC_TSP_FACTORY_TEST */

struct mms_info {
	struct mms_platform_data	*pdata;
	struct i2c_client			*client;
	struct input_dev			*input_dev;
	char						phys[32];
	int							irq;
	bool						enabled;
	bool						finger_state[MAX_FINGER_NUM];
	u16							mcount[MAX_FINGER_NUM];
	int							pwr_enabled;

	const struct firmware		*fw;
	u8							*config_fw;
	u8							fw_ver_ic[3]; // fw_ic_ver
	u8							fw_ver_ic_config;	//fw_ic_ver_config
	u8							fw_ver_bin[3];	//fw_ic_bin

	struct mutex device_mutex;

	bool	resume_done;
	struct delayed_work work_config_set;

#if INFORM_CHARGER
	bool						ta_connected;
	bool						noise_mode;
#endif
	u8							key_num;
#ifdef SEC_TSP_FACTORY_TEST
	struct mms_fac_data			*fac_data;
	struct device *fac_dev_ts;
#endif
};

struct mms_bin_hdr {
	char	tag[8];
	u16		core_version;
	u16		section_num;
	u16		contains_full_binary;
	u16		reserved0;
	u32		binary_offset;
	u32		binary_length;
	u32		extention_offset;
	u32		reserved1;
} __attribute__ ((packed));

struct mms_fw_img {
	u16	type;
	u16	version;
	u16	start_page;
	u16	end_page;
	u32	offset;
	u32	length;
} __attribute__ ((packed));

struct isc_packet {
	u8	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));

#ifdef CONFIG_SEC_DEBUG_TSP_LOG
#define tsp_debug_dbg(mode, dev, fmt, ...)	\
({											\
	dev_dbg(dev, fmt, ## __VA_ARGS__);		\
	if (mode) {								\
		char msg_buff[10];						\
		snprintf(msg_buff, sizeof(msg_buff), "%s", dev_name(dev));	\
		sec_debug_tsp_log_msg(msg_buff, fmt, ## __VA_ARGS__);	\
	}										\
})

#define tsp_debug_info(mode, dev, fmt, ...)	\
({											\
	dev_info(dev, fmt, ## __VA_ARGS__);		\
	if (mode) {								\
		char msg_buff[10];						\
		snprintf(msg_buff, sizeof(msg_buff), "%s", dev_name(dev));	\
		sec_debug_tsp_log_msg(msg_buff, fmt, ## __VA_ARGS__);	\
	}										\
})

#define tsp_debug_err(mode, dev, fmt, ...)	\
({											\
	dev_err(dev, fmt, ## __VA_ARGS__);		\
	if (mode) {								\
		char msg_buff[10];						\
		snprintf(msg_buff, sizeof(msg_buff), "%s", dev_name(dev));	\
		sec_debug_tsp_log_msg(msg_buff, fmt, ## __VA_ARGS__);	\
	}										\
})
#else
#define tsp_debug_dbg(mode, dev, fmt, ...)	dev_dbg(dev, fmt, ## __VA_ARGS__)
#define tsp_debug_info(mode, dev, fmt, ...)	dev_info(dev, fmt, ## __VA_ARGS__)
#define tsp_debug_err(mode, dev, fmt, ...)	dev_err(dev, fmt, ## __VA_ARGS__)
#endif


static int mms_input_open(struct input_dev *dev);
static void mms_input_close(struct input_dev *dev);

#ifdef MMS_RUNTIME_CONF
static int mms_read_config(struct i2c_client *client, u8 *buf, u8 *get_buf,int len);
static int mms_verify_config(struct mms_info *info,const u8 *buf,const u8 *tmp,int len);
static int mms_config_flash(struct mms_info *info, const u8 *buf,const u8 len, char *text);
static int mms_config_start(struct mms_info *info);
static int mms_config_finish(struct mms_info *info);
static int mms_config_get(struct mms_info *info, u8 mode);
static void mms_config_set(void *context);
static void work_mms_config_set(struct work_struct *work);
#endif

#ifdef CONFIG_BATTERY_SAMSUNG
extern unsigned int lpcharge;
#endif


#endif /* _LINUX_MMS100S_TOUCH_H_ */

