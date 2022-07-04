
#ifndef _DSI_IRIS_LIGHTUP_H_
#define _DSI_IRIS_LIGHTUP_H_
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/clk.h>

#include "iris_def.h"
#include "iris_lightup_ocp.h"

#define IRIS_CHIP_CNT   2

//#define IRIS_I2C_ENABLE
/**
 * enum dsi_op_mode - dsi operation mode
 * @DSI_OP_VIDEO_MODE: DSI video mode operation
 * @DSI_OP_CMD_MODE:   DSI Command mode operation
 * @DSI_OP_MODE_MAX:
 */
enum dsi_op_mode {
	DSI_OP_VIDEO_MODE = 0,
	DSI_OP_CMD_MODE,
	DSI_OP_MODE_MAX
};

/**
 * enum dsi_display_selection_type - enumerates DSI display selection types
 * @DSI_PRIMARY:	primary DSI display selected from module parameter
 * @DSI_SECONDARY:	Secondary DSI display selected from module parameter
 * @MAX_DSI_ACTIVE_DISPLAY: Maximum acive displays that can be selected
 */
enum dsi_display_selection_type {
	DSI_PRIMARY = 0,
	DSI_SECONDARY,
	MAX_DSI_ACTIVE_DISPLAY,
};
/* iris ip option, it will create according to opt_id.
*  link_state will be create according to the last cmds
*/
struct iris_ip_opt {
	uint8_t opt_id; /*option identifier*/
	uint32_t cmd_cnt; /*option length*/
	uint8_t link_state; /*high speed or low power*/
	struct iris_cmd_desc *cmd; /*the first cmd of desc*/
};

/*ip search index*/
struct iris_ip_index {
	int32_t opt_cnt; /*ip option number*/
	struct iris_ip_opt *opt; /*option array*/
};

struct iris_pq_ipopt_val {
	int32_t opt_cnt;
	uint8_t ip;
	uint8_t *popt;
};

struct iris_pq_init_val {
	int32_t ip_cnt;
	struct iris_pq_ipopt_val *val;
};

/*used to control iris_ctrl opt sequence*/
struct iris_ctrl_opt {
	uint8_t ip;
	uint8_t opt_id;
	uint8_t skip_last;
};

struct iris_ctrl_seq {
	int32_t cnt;
	struct iris_ctrl_opt *ctrl_opt;
};

//will pack all the commands here
struct iris_out_cmds {
	/* will be used before cmds sent out */
	struct iris_cmd_desc *iris_cmds_buf;
	u32 cmds_index;
};

typedef int (*iris_i2c_read_cb)(u32 reg_addr, u32 *reg_val);
typedef int (*iris_i2c_write_cb)(u32 reg_addr, u32 reg_val);
typedef int (*iris_i2c_burst_write_cb)(u32 start_addr, u32 *lut_buffer, u16 reg_num);

enum IRIS_PARAM_VALID {
	PARAM_NONE = 0,
	PARAM_EMPTY,
	PARAM_PARSED,
	PARAM_PREPARED,
	PARAM_LIGHTUP,
};


struct iris_cfg {

	struct platform_device *pdev;
	struct {
		struct pinctrl *pinctrl;
		struct pinctrl_state *active;
		struct pinctrl_state *suspend;
	} pinctrl;
	int iris_reset_gpio;
	int iris_wakeup_gpio;
	int iris_abyp_ready_gpio;
	int iris_osd_gpio;
	int iris_vdd_gpio;

	/* hardware version and initialization status */
	uint8_t chip_id;
	uint32_t chip_ver;
	uint32_t chip_value[2];
	uint8_t valid; /* 0: none, 1: empty, 2: parse ok, 3: minimum light up, 4. full light up */
	bool iris_initialized;
	bool mcu_code_downloaded;
	bool switch_bl_endian;
	uint8_t panel_type;
	uint8_t lut_mode;
	bool is_prepared;

	uint32_t add_last_flag;
	uint32_t add_on_last_flag;
	uint32_t add_pt_last_flag;
	uint32_t split_pkt_size;
	uint32_t loop_back_mode;
	uint32_t loop_back_mode_res;
	uint32_t min_color_temp;
	uint32_t max_color_temp;
	uint8_t rx_mode; /* 0: DSI_VIDEO_MODE, 1: DSI_CMD_MODE */
	uint8_t tx_mode;

	/* current state */
	struct iris_lp_ctrl lp_ctrl;
	struct iris_abypass_ctrl abypss_ctrl;
	uint16_t panel_nits;
	uint32_t panel_dimming_brightness;
	uint8_t panel_hbm[2];
	bool frc_enable;
	bool frc_setting_ready;
	struct iris_frc_setting frc_setting;
	bool frc_low_latency;
	u32 frc_app_info;
	int pwil_mode;

	uint32_t panel_te;
	uint32_t ap_te;
	uint32_t switch_mode;
	uint8_t power_mode;
	bool n2m_enable;
	int dport_output_mode;
	bool dynamic_vfr;

	/* secondary display related */
	//struct dsi_display *display2;	// secondary display
	//struct dsi_panel *panel2;	// secondary panel
	bool osd_enable;
	bool osd_on;
	bool osd_switch_on_pending;
	bool iris_osd_autorefresh;
	// retain autorefresh in prepare_commit
	bool iris_cur_osd_autorefresh;
	bool iris_osd_autorefresh_enabled;
	bool mipi2_pwr_st;	// secondary mipi power status
	atomic_t osd_irq_cnt;
	atomic_t video_update_wo_osd;

	char display_mode_name[16];
	uint32_t app_version;
	uint8_t app_date[4];
	uint8_t abyp_prev_mode;
	struct clk *ext_clk;

	int32_t panel_pending;
	int32_t panel_delay;
	int32_t panel_level;
	uint8_t *panel_name;

	//struct dsi_regulator_info iris_power_info; // iris pmic power

	uint32_t lut_cmds_cnt;
	uint32_t dtsi_cmds_cnt;
	struct iris_ip_index ip_index_arr[IRIS_PIP_IDX_CNT][IRIS_IP_CNT];
	struct iris_ctrl_seq ctrl_seq[IRIS_CHIP_CNT];
	struct iris_ctrl_seq ctrl_seq_cs[IRIS_CHIP_CNT];
	struct iris_pq_init_val pq_init_val;
	struct iris_out_cmds iris_cmds;
	/* one wire gpio lock */
	spinlock_t iris_1w_lock;
	struct dentry *dbg_root;

	struct work_struct lut_update_work;
	struct work_struct vfr_update_work;
	struct completion frame_ready_completion;

	/* hook for i2c extension */
	iris_i2c_read_cb iris_i2c_read;
	iris_i2c_write_cb iris_i2c_write;
	iris_i2c_burst_write_cb iris_i2c_burst_write;
	bool dual_setting;
	uint32_t dual_test;
	bool osd_layer_empty;
	uint32_t metadata;
	bool mcu_running;
	bool iris_i2c_switch;

	/*iris abypass function*/
	struct iris_abyp_ops abyp_ops;
	struct mutex kickoff_mutex;

	struct drm_panel *panel;
	struct drm_device *drm;
	struct device *dev;
	struct mtk_ddp_comp *mtk_comp;
	struct drm_display_mode *dsp_mode;
	struct mtk_panel_ext *panel_ext;
	struct mipi_dsi_device *dsi_dev;
	struct drm_crtc *crtc;
	struct mutex mutex;
	struct mutex lock_send_pkt;

	struct dsi_mode_info timing;

	u32 esd_chk_val[ESD_CHK_NUM + 1];
	bool is_esd_check_ongoing;
	bool esd_read_flag;
	int esd_check_num;
	int esd_read_index;
};

struct iris_data {
	const uint8_t *buf;
	uint32_t size;
};

struct iris_cfg *iris_get_cfg(void);

int iris_lightup(void);
int iris_lightoff(struct iris_cmd_set *off_cmds);
int32_t iris_send_ipopt_cmds(int32_t ip, int32_t opt_id);
int32_t iris_send_ipopt_cmds_nonlock(int32_t ip, int32_t opt_id);

void iris_update_pq_opt(struct iris_update_ipopt *popt, int len, uint8_t path);
void iris_update_bitmask_regval_nonread(
		struct iris_update_regval *pregval, bool is_commit);

void iris_alloc_seq_space(void);

void iris_init_update_ipopt(struct iris_update_ipopt *popt,
		uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t skip_last);
struct iris_pq_ipopt_val  *iris_get_cur_ipopt_val(uint8_t ip);

int iris_init_update_ipopt_t(struct iris_update_ipopt *popt,  int len,
		uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t skip_last);

/*
 * @description  get assigned position data of ip opt
 * @param ip       ip sign
 * @param opt_id   option id of ip
 * @param pos      the position of option payload
 * @return   fail NULL/success payload data of position
 */
uint32_t  *iris_get_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos);
void iris_set_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);

/*
 *@Description: get current continue splash stage
 first light up panel only
 second pq effect
 */
uint8_t iris_get_cont_splash_type(void);

/*
 *@Description: print continuous splash commands for bootloader
 *@param: pcmd: cmds array  cnt: cmds cound
 */
void iris_print_desc_cmds(struct iris_cmd_desc *pcmd, int cmd_cnt, int state);

int iris_init_cmds(void);
void iris_get_cmds(struct iris_cmd_set *cmds, char **ls_arr);
void iris_get_lightoff_cmds(struct iris_cmd_set *cmds, char **ls_arr);

int32_t iris_attach_cmd_to_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index);

struct iris_ip_index *iris_get_ip_idx(int32_t type);

void iris_change_type_addr(struct iris_ip_opt *dest, struct iris_ip_opt *src);

struct iris_ip_opt *iris_find_specific_ip_opt(uint8_t ip, uint8_t opt_id, int32_t type);
struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id);

int iris_wait_vsync(void);
int iris_set_pending_panel_brightness(int32_t pending, int32_t delay, int32_t level);


void iris_free_ipopt_buf(uint32_t ip_type);
void iris_free_seq_space(void);

void iris_send_assembled_pkt(struct iris_ctrl_opt *arr, int seq_cnt);

int iris_get_vrefresh(void);

int iris_get_vres(void);

bool iris_check_high_res(void);

int iris_get_hres(void);

int iris_get_vtotal(void);

int iris_get_htotal(void);

bool iris_check_dsc_enable(void);

int32_t iris_parse_dtsi_cmd(const struct device_node *lightup_node,
		uint32_t cmd_index);
int32_t iris_parse_optional_seq(struct device_node *np, const uint8_t *key,
		struct iris_ctrl_seq *pseq);

int iris_driver_register(void);
void iris_driver_unregister(void);
int32_t iris_send_ipopt_cmds_i2c(int32_t ip, int32_t opt_id);
int iris_read_chip_id(void);
void iris_load_mcu(void);

void iris_get_panel_params(struct drm_device *drm, struct drm_display_mode **dsp_mode);
void iris_sync_timing(struct dsi_mode_info *ptiming, struct drm_display_mode *dsp_mode);
void iris_sync_cur_timing(void);


#endif // _DSI_IRIS_LIGHTUP_H_
