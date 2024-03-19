#ifndef __MHA100KN_HALL_H__
#define __MHA100KN_HALL_H__

#define HALL_DEV_NAME   "oplus,mha100kn_hall"
#define MHALL_DEBUG
#define MHALL_TAG       "[MHALL]"
#if defined(MHALL_DEBUG)
#define MHALL_ERR(fmt, args...) printk(KERN_ERR  MHALL_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MHALL_LOG(fmt, args...) printk(KERN_INFO MHALL_TAG"%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#else
#define MHALL_CHG_REAR_ERR(fmt, args...)
#define MHALL_CHG_REAR_LOG(fmt, args...)
#endif

struct hall_mha100kn_data {
	int id;                     /* show the current id */
	char DEV_NAME[64];

	int irq_gpio;               /* device use gpio number */
	int irq_number;             /* device request irq number */
	uint32_t irq_flags;         /* device irq flags */
	int active_low;             /* gpio active high or low for valid value */
	int hall_status;            /* device status of latest */
	int fb_report_cnt;

	struct mutex            report_mutex;
	struct hf_device        *hf_dev;
	struct pinctrl          *hall_pinctrl;
	struct pinctrl_state    *hall_int_active;
	struct pinctrl_state    *hall_int_sleep;
};

typedef enum {
	TYPE_HALL_UNDEFINED,
	TYPE_HALL_NEAR = 1,     /*means in near status*/
	TYPE_HALL_FAR,          /*means in far status*/
} hall_status;

#endif

