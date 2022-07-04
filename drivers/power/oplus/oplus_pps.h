/**********************************************************************************
* Copyright (c)  2008-2020  Guangdong OPLUS Mobile Comm Corp., Ltd

* VENDOR_EDIT
* Description: Charger IC management module for charger system framework.
*              Manage all charger IC and define abstarct function flow.
* Version    : 1.0
* Date       : 2020-06-29
* Author     : zhouhaikang@oplus.com
* ------------------------------ Revision History: --------------------------------
* <version>           <date>                <author>                       <desc>
***********************************************************************************/
#include <linux/time.h>
#include <linux/list.h>
#ifndef CONFIG_OPLUS_CHARGER_MTK
#include <linux/usb/typec.h>
#include <linux/usb/usbpd.h>
#endif
#include <linux/random.h>
#include <linux/device.h>
#include <linux/types.h>
#define OPLUS_CHG_UPDATE_PPS_DELAY	round_jiffies_relative(msecs_to_jiffies(500))
#define PD_PPS_STATUS_VOLT(pps_status)			(((pps_status) >> 0) & 0xFFFF)
#define PD_PPS_STATUS_CUR(pps_status)			(((pps_status) >> 16) & 0xFF)

#define OPLUS_PPS_10V_STATUS_1               12
#define OPLUS_PPS_10V_STATUS_2               6

#define OPLUS_PPS_20V_STATUS_1               20
#define OPLUS_PPS_20V_STATUS_2               6
#define PPS_KEY_NUMBER                       4

#define OVER_CURRENT_VALUE                   4000
#define FASTCHG_TIMEOUT                      7200
#define BAT_FULL_1TIME_THD                   4430
#define LITTLE_BAT_CUR_VOLT                  4350
#define ADAPTER_PLUG_OUT_PLUS_VOLT           500

#define BAT_CUR_VOLT_FULL_4410               4410
#define BAT_CUR_VOLT_FULL_4400               4400
#define BAT_CUR_CURR_FULL_4410               700
#define BAT_CUR_CURR_FULL_4400               500

#define R1_LIMIT                             50
#define R2_LIMIT                             20

#define CABLE_CURRENT_LIMIT                  1000

#define BATT_SYS_MAX				8

enum {
	PPS_BAT_TEMP_NATURAL = 0,
	PPS_BAT_TEMP_HIGH0,
	PPS_BAT_TEMP_HIGH1,
	PPS_BAT_TEMP_HIGH2,
	PPS_BAT_TEMP_HIGH3,
	PPS_BAT_TEMP_HIGH4,
	PPS_BAT_TEMP_HIGH5,
	PPS_BAT_TEMP_LOW0,
	PPS_BAT_TEMP_LOW1,
	PPS_BAT_TEMP_LOW2,
	PPS_BAT_TEMP_LITTLE_COOL,
	PPS_BAT_TEMP_COOL,
	PPS_BAT_TEMP_EXIT,
	PPS_BAT_TEMP_OVER_HIGH_EXIT,
	PPS_BAT_TEMP_OVER_LOW_EXIT,
};

enum {
	OPLUS_PPS_STATUS_START = 0,
	OPLUS_PPS_STATUS_OPEN_OVP_CP,
	OPLUS_PPS_STATUS_VOLT_CHANGE,
	//OPLUS_PPS_STATUS_CUR_CHANGE,
	OPLUS_PPS_STATUS_CUR_INCREASE,
	OPLUS_PPS_STATUS_CUR_DECREASE,
	OPLUS_PPS_STATUS_CHECK,
};


enum {
	OPLUS_10V_STATUS = 0,
	OPLUS_125W_STATUS1,
	OPLUS_125W_STATUS2,
	OPLUS_10V_30W_STATUS,         //第三方适配器，30w
};
enum {
	PPS_NOT_SUPPORT = 0,
	PPS_CHECKING,
	PPS_CHARGERING,
	PD_CHARGERING,
	PPS_CHECK_TIMEOUT,
	PPS_CHARGE_END,
};

enum {
	BATT_CURVE_TEMP_0_TO_50,
	BATT_CURVE_TEMP_50_TO_120,
	BATT_CURVE_TEMP_120_TO_160,
	BATT_CURVE_TEMP_160_TO_430,
	BATT_CURVE_TEMP_MAX,
};

enum {
	BATT_CURVE_SOC_0_TO_50,
	BATT_CURVE_SOC_50_TO_75,
	BATT_CURVE_SOC_75_TO_85,
	BATT_CURVE_SOC_85_TO_90,
	BATT_CURVE_SOC_MAX,
};

struct batt_curve {
	unsigned int target_vbus;
	unsigned int target_vbat;
	unsigned int target_ibus;
	bool exit;
	unsigned int target_time;
};

struct batt_curves {
	struct batt_curve batt_curves[BATT_SYS_MAX];
	int batt_curve_num;
};

struct batt_curves_soc {
	struct batt_curves batt_curves_temp[BATT_CURVE_TEMP_MAX];
};

struct oplus_pps_chip {
	struct device				*dev;
	int                         oplus_pps_status;
	int                         pps_status;
	int                         work_delay;

	int                         charger_output_volt;
	int                         charger_output_current;

	int                         ap_input_volt;
	int                         ap_system_current;
	int                         ap_input_current;

	int                         ap_batt_volt;
	int                         ap_batt_current;
	int                         ap_batt_soc;
	int                         ap_batt_temperature;

	bool                        need_update_volt_cur;
	int                         target_charger_volt;
	int                         target_charger_current;
	int                         ask_charger_volt;
	int                         ask_charger_current;
	int                         ask_charger_volt_last;
	int                         ask_charger_current_last;
	int                         pps_chging;
	bool                        fg_batt_full;

	char                        curr_over_count;
	bool                        fg_current_over;

	char                        chg_curr_over_count;
	bool                        fg_chg_current_over;

	int                         pps_support_type;

	int                         pps_little_cool_temp;
	int                         pps_cool_temp;
	int                         pps_little_cool_to_normal_temp;
	int                         pps_normal_to_little_cool_power;
	int                         pps_low_temp;
	int                         pps_little_low_temp;
	int                         pps_high_temp;
	int                         pps_little_high_temp;
	int                         pps_low_soc;
	int                         pps_high_soc;
	int                         pps_multistep_initial_batt_temp;
	int                         pps_strategy_normal_power;
	int                         pps_strategy1_batt_high_temp0;
	int                         pps_strategy1_batt_high_temp1;
	int                         pps_strategy1_batt_high_temp2;
	int                         pps_strategy1_batt_low_temp2;
	int                         pps_strategy1_batt_low_temp1;
	int                         pps_strategy1_batt_low_temp0;
	int                         pps_strategy1_high_power0;
	int                         pps_strategy1_high_power1;
	int                         pps_strategy1_high_power2;
	int                         pps_strategy1_low_power2;
	int                         pps_strategy1_low_power1;
	int                         pps_strategy1_low_power0;
	int                         pps_fastchg_batt_temp_status;
	int                         pps_batt_over_high_temp;
	int                         pps_batt_over_low_temp;
	int                         pps_over_high_or_low_power;
	int                         pps_strategy_change_count;
	int                         curve_ov_count;
	int                         curve_uc_count;
	int                         current_adapter_max;
	int                         fastchg_timeout_time;
	int                         fast3c_timeout_time;

	int                         vcpout_volt;
	char                        small_curr_count;
	char                        r1_over_count;
	bool                        fg_r1_over;
	char                        r2_over_count;
	bool                        fg_r2_over;

	char                        cable_over_count;
	bool                        fg_cable_over;

	char                        temp_over_count;
	bool                        temp_over;

	char                        temp_over_fast_count;
	bool                        temp_over_fast;

	char                        btb_temp_over_count;
	bool                        btb_temp_over;

	char                        pps_disconnect_count;
	bool                        pps_disconnect;
	int                         pps_disconnect_volt;

	int                         pps_curr_check_count;

	bool                        need_change_curve;
	struct list_head		temp_list;
	//struct oplus_chg_chip 		*chip;
	struct timespec 		vcp_time;		//阻抗检测time
	struct timespec 		ts_last;		//5s看门狗，保证5s和适配器通讯一次
	struct timespec 		fastchg_time;		//充电曲线time
	struct timespec 		temp_time;		//温控保护，每1s读一次温度
	struct power_supply		*pps_usb_psy;
	struct notifier_block		pps_psy_nb;
	struct usbpd			*pd;
#ifndef CONFIG_OPLUS_CHARGER_MTK
	struct usbpd_svid_handler	svid_handler;
#endif
	struct completion		is_ready;
	struct mutex			status_mutex;

	struct oplus_pps_mcu_operations *ops;
	struct delayed_work		pps_stop_work;
	struct delayed_work		update_pps_work;
	struct delayed_work		register_pps;
	int (*pps_pdo_select)(int vbus_mv, int ibus_ma);
	u32 (*get_pps_status)(void);
	int (*get_pps_max_cur)(int vbus_mv);
	struct batt_curves_soc		batt_curves_soc[BATT_CURVE_SOC_MAX];
	struct batt_curves		batt_curves;
	int 				batt_curve_index;
	int					temperature;
	int					soc;
	int					current_batt_curve;
	int					current_batt_temp;
	int					current_cool_down;
	int					target_charger_volt_pre;
	int					target_charger_current_pre;
	int					ap_asic_batt_volt;
};

struct oplus_pps_mcu_operations {
	int (*get_input_volt)(void);
	int (*get_vbat0_volt)(void);
	int (*check_btb_temp)(void);
	int (*pps_mos_ctrl)(int on);
};

struct oplus_temp_chip {
	struct device *dev;
	struct list_head temp_list;
	/* ep_queue() func will add
	 a request->queue into a udc_ep->queue 'd tail */
	int (*get_temp)(struct device *);
};

int oplus_pps_register_ops(struct oplus_pps_mcu_operations *ops);
int oplus_pps_init(struct oplus_chg_chip *chip);
//int pps_get_adapter_type(void);
void oplus_pps_variables_reset(void);
int oplus_pps_get_chg_status(void);
int oplus_pps_set_chg_status(int status);
int oplus_pps_start(void);
void oplus_pps_stop(void);
bool oplus_is_pps_charging(void);
extern int oplus_chg_set_pps_config(int vbus_mv, int ibus_ma);
extern void oplus_chg_stop_pps_config(void);
extern u32 oplus_chg_get_pps_status(void);
extern int oplus_chg_pps_get_max_cur(int vbus_mv);
void oplus_pps_status_reset(void);
