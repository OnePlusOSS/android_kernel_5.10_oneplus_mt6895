#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <drm/drm_mipi_dsi.h>

#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_crtc.h"
#include "mtk_log.h"

#include "iris_api.h"
#include "iris_lightup.h"
#include "iris_lightup_ocp.h"
#include "iris_mode_switch.h"
#include "iris_lp.h"
#include "iris_pq.h"
#include "iris_gpio.h"
#include "iris_timing_switch.h"
#include "iris_log.h"
#include "iris_frc.h"
#include "iris_mtk_api.h"

#define DEBUG_READ_PMU
#define DEFAULT_ABYP_LP_MODE ABYP_POWER_DOWN_PLL

static int debug_lp_opt = 0;
extern uint8_t iris_pq_update_path;

/* abyp light up option (need panel off/on to take effect)
 * bit[0]: 0 -- light up with PT, 1 -- light up with ABYP
 * bit[1]: 0 -- efuse mode is ABYP, 1 -- efuse mode is PT
 * bit[2]: 0 -- use mipi command to switch, 1 -- use GPIO to switch
 * bit[3]: 0 -- non force, 1 -- force abyp during panel switch
 */
static int debug_on_opt;

static bool iris_lce_power;

static bool iris_bsram_power; /* BSRAM domain power status */

#define IRIS_TRACE_FPS       0x01
#define IRIS_TRACE_CADENCE   0X02
static int debug_trace_opt;
static int debug_abyp_gpio_status = -1;


static void _iris_reset_mipi(void);
static void _iris_plus_reset_mipi(void);
static void _iris_video_abyp_enter(void);
static void _iris_video_abyp_exit(void);
static bool _iris_cmd_abyp_exit(int mode);
static bool _iris_cmd_abyp_enter(int mode);
static bool _iris_plus_cmd_abyp_exit(int mode);
static bool _iris_plus_cmd_abyp_enter(int mode);
static int _iris_abyp_select(int dest_mode);
static int _iris_plus_abyp_select(int dest_mode);
static void _iris_pre_config_for_timing(void);
static void _iris_plus_pre_config_for_timing(void);

int iris_get_default_work_mode(void)
{
	return iris_get_cfg()->abypss_ctrl.default_abyp_mode;
}


void iris_set_default_work_mode(int mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (mode < PASS_THROUGH_MODE)
		mode = PASS_THROUGH_MODE;

	switch (pcfg->chip_id) {
	case IRIS_CHIP_VER_0:
		if (mode > ANALOG_BYPASS_MODE)
			mode = ANALOG_BYPASS_MODE;
		break;
	case IRIS_CHIP_VER_1:
		if (mode > ABP_SLEEP_MODE)
			mode = ABP_SLEEP_MODE;
		break;
	}

	pcfg->abypss_ctrl.default_abyp_mode = mode;
}

void iris_init_abyp_ops(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	enum dsi_op_mode panel_mode  = pcfg->tx_mode;

	iris_set_default_work_mode(ANALOG_BYPASS_MODE);
	switch (pcfg->chip_id) {
	case IRIS_CHIP_VER_0:
		if (panel_mode == DSI_OP_CMD_MODE) {
			pcfg->abyp_ops.abyp_enter = _iris_cmd_abyp_enter;
			pcfg->abyp_ops.abyp_exit = _iris_cmd_abyp_exit;
			pcfg->abyp_ops.mipi_reset = _iris_reset_mipi;
			pcfg->abyp_ops.abyp_select = _iris_abyp_select;
			pcfg->abyp_ops.pre_config_for_timing = _iris_pre_config_for_timing;
			pcfg->abyp_ops.configure_abyp = iris_configure_abyp;
		}
		break;
	case IRIS_CHIP_VER_1:
		if (panel_mode == DSI_OP_CMD_MODE) {
			pcfg->abyp_ops.abyp_enter = _iris_plus_cmd_abyp_enter;
			pcfg->abyp_ops.abyp_exit = _iris_plus_cmd_abyp_exit;
			pcfg->abyp_ops.mipi_reset = _iris_plus_reset_mipi;
			pcfg->abyp_ops.abyp_select = _iris_plus_abyp_select;
			pcfg->abyp_ops.pre_config_for_timing = _iris_plus_pre_config_for_timing;
			pcfg->abyp_ops.configure_abyp = iris_plus_configure_abyp;
		}
		iris_set_default_work_mode(ABP_SLEEP_MODE);
		break;
	default:
		IRIS_LOGE("%s could not chip_id=%d", pcfg->chip_id);
		break;
	}
}

int32_t iris_parse_lp_ctrl(struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 vals[3];

	rc = of_property_read_u8_array(np, "pxlw,low-power", vals, 3);
	if (rc) {
		IRIS_LOGE("%s(), failed to find low power property, return: %d",
				__func__, rc);
		return 0;
	}
	pcfg->lp_ctrl.esd_ctrl = 7;

	pcfg->lp_ctrl.dynamic_power = (bool)vals[0];
	pcfg->lp_ctrl.ulps_lp = (bool)vals[1];
	pcfg->lp_ctrl.abyp_enable = (bool)vals[2];
	IRIS_LOGI("%s(), parse low power info: %d %d %d",
			__func__, vals[0], vals[1], vals[2]);

	return rc;
}

static void _iris_init_abp_mode(void)
{
	static bool once = true;

	if (once) {
		if (iris_read_chip_id())
			IRIS_LOGI("could not read chip id,use default value");

		//according to chip_id to initialize abyp ops
		iris_init_abyp_ops();

		iris_get_cfg()->abypss_ctrl.abypass_mode =
			iris_get_default_work_mode();
		once = false;
	}
}

void iris_lp_preinit(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (/*iris_virtual_display(pcfg->display) || */pcfg->valid < PARAM_PARSED)
		return;

	_iris_init_abp_mode();

	if (debug_on_opt & 0x01)
		pcfg->abypss_ctrl.abypass_mode = iris_get_default_work_mode();

	IRIS_LOGI("%s:%d, pcfg->abypss_ctrl.abypass_mode = %d", __func__, __LINE__, pcfg->abypss_ctrl.abypass_mode);
#if 0
	iris_init_one_wired();
#endif
	pcfg->is_esd_check_ongoing = false;
}

/* clear some pmu domains */
static void iris_clear_pmu(void)
{
	struct iris_update_regval regval;

	iris_bsram_power = false;

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_PMU_CTRL;
	regval.mask = 0x000000b8; /*clear MIPI2, BSRAM, FRC, DSCU */
	regval.value = 0x0;

	iris_update_bitmask_regval_nonread(&regval, true);
}

/* init iris low power */
void iris_lp_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg = iris_get_cfg();
	if (/*iris_virtual_display(pcfg->display) || */pcfg->valid < PARAM_PARSED)
		return;

	IRIS_LOGI("lp dynamic_power:%d, ulps_lp:%d, abyp_lp_enable:%d",
			pcfg->lp_ctrl.dynamic_power, pcfg->lp_ctrl.ulps_lp,
			pcfg->lp_ctrl.abyp_enable);

	if (pcfg->lp_ctrl.dynamic_power) {
		IRIS_LOGD(" [%s, %d] open psr_mif osd first address eco.", __func__, __LINE__);
		iris_psf_mif_dyn_addr_set(true);
		iris_dynamic_power_set(true);
	} else {
		IRIS_LOGD(" [%s, %d] close psr_mif osd first address eco.", __func__, __LINE__);
		iris_psf_mif_dyn_addr_set(false);
	}

	iris_clear_pmu();

	if (pcfg->lp_ctrl.ulps_lp)
		iris_ulps_source_sel(ULPS_MAIN);
	else
		iris_ulps_source_sel(ULPS_NONE);
}

/*== PMU related APIs ==*/

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_PMU_CTRL;
	regval.mask = 0x00000001;
	regval.value = (enable ? 0x1 : 0x0);

	if (enable) {
		/* 0xf0: read; 0xf1: non-read */
		iris_send_ipopt_cmds(IRIS_IP_DMA, 0xf0);

		iris_update_bitmask_regval_nonread(&regval, true);
	} else {
		iris_update_bitmask_regval_nonread(&regval, true);

		/* delay for disabling dynamic power gating take effect */
		usleep_range(1000 * 20, 1000 * 20 + 1);
		/* 0xf0: read; 0xf1: non-read */
		iris_send_ipopt_cmds(IRIS_IP_DMA, 0xf1);
	}

	pcfg->lp_ctrl.dynamic_power = enable;
	IRIS_LOGI("%s: %d", __func__, enable);
}

/* dynamic power gating get */
bool iris_dynamic_power_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.dynamic_power;
}

static int iris_pmu_power_set(enum iris_pmu_domain domain_id, bool on)
{
	struct iris_update_regval regval;
	struct iris_update_ipopt popt;
#ifdef DEBUG_READ_PMU
	uint32_t set_pmu_ctrl, pmu_ctrl;
	uint32_t  *payload = NULL;
	uint32_t reg_pmu_ctrl, top_pmu_status, pmu_status;
	int i;
#endif
	bool is_ulps_enable = 0;
	uint8_t path = iris_pq_update_path;

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_PMU_CTRL;
	regval.mask = domain_id;
	regval.value = (on ? domain_id : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt(&popt, IRIS_IP_SYS, regval.opt_id, regval.opt_id, 0);
	is_ulps_enable = iris_disable_ulps(path);
	iris_update_pq_opt(&popt, 1, path);
	iris_enable_ulps(path, is_ulps_enable);

#ifdef DEBUG_READ_PMU
	if ((debug_lp_opt & 0x100) == 0x100) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_PMU_CTRL, 2);
		set_pmu_ctrl = payload[0];

		reg_pmu_ctrl = iris_ocp_read(REG_ADDR_PMU_CTRL, DSI_CMD_SET_STATE_HS);

		if (reg_pmu_ctrl != set_pmu_ctrl) {
			IRIS_LOGE("Err: read pmu ctrl 0x%08x != set_pmu_ctrl 0x%08x", reg_pmu_ctrl, set_pmu_ctrl);
			return 2;
		}
		pmu_ctrl = (reg_pmu_ctrl >> 2) & 0xff;

		for (i = 0; i < 10; i++) {
			top_pmu_status = iris_ocp_read(REG_ADDR_PMU_STATUS, DSI_CMD_SET_STATE_HS);
			pmu_status = ((top_pmu_status>>8)&0x3) + (((top_pmu_status>>15)&0x1)<<2) +
				(((top_pmu_status>>11)&0x1)<<3) + (((top_pmu_status>>10)&0x1)<<4) +
				(((top_pmu_status>>12)&0x7)<<5);
			IRIS_LOGI("read pmu ctrl 0x%08x top_pmu_status 0x%08x, pmu_status 0x%02x",
					reg_pmu_ctrl, top_pmu_status, pmu_status);

			if (pmu_status == pmu_ctrl)
				break;

			IRIS_LOGE("Err %d: pmu_status: 0x%08x != pmu_ctrl 0x%02x", i, pmu_status, pmu_ctrl);
			usleep_range(1000 * 10, 1000 * 10 + 1);
		}
		if (i == 10) {
			IRIS_LOGE("Err: return!");
			return 3;
		}
	}
#endif

	return 0;
}

static bool iris_pmu_power_get(enum iris_pmu_domain domain_id)
{
	uint32_t pmu_ctrl;
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_PMU_CTRL, 2);
	pmu_ctrl = payload[0];
	return ((pmu_ctrl & domain_id) != 0);
}

void _iris_video_abyp_power(bool on)
{
	struct iris_update_regval regval;
	struct iris_update_ipopt popt;
	struct iris_cfg *pcfg;
	bool is_ulps_enable = 0;
	uint8_t path = iris_pq_update_path;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_PMU_CTRL;
	regval.mask = 0x40800003;
	if (on)
		regval.value = (pcfg->lp_ctrl.dynamic_power ? 0x3 : 0x2);
	else
		regval.value = 0x40800000; /*MIPI0_AUTO_DMA_EN, CORE_DOMAINS_OFF_BY_MIPI_EN*/

	IRIS_LOGE("%s 0x%x 0x%x", __func__, regval.mask, regval.value);

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt(&popt, IRIS_IP_SYS, regval.opt_id, regval.opt_id, 0);
	is_ulps_enable = iris_disable_ulps(path);
	iris_update_pq_opt(&popt, 1, path);
	iris_enable_ulps(path, is_ulps_enable);
}

/* power on & off mipi2 domain */
int iris_pmu_mipi2_set(bool on)
{
	int rt = 0;

	if (((debug_lp_opt & 0x1) == 0x1) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	rt = iris_pmu_power_set(MIPI2_PWR, on);
	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

/* power on & off bulksram domain */
int iris_pmu_bsram_set(bool on)
{
	int rt = 0;
	bool is_ulps_enable = 0;
	uint8_t path = iris_pq_update_path;
	int i = 0;

	if (((debug_lp_opt & 0x2) == 0x2) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	if (on != iris_bsram_power) {
		struct iris_update_regval regval;
		struct iris_update_ipopt popt;

		rt = iris_pmu_power_set(BSRAM_PWR, on);
		iris_bsram_power = on;

		regval.ip = IRIS_IP_SYS;
		regval.opt_id = ID_SYS_MEM_REPAIR;
		regval.mask = 0x330000;
		regval.value = (on ? 0x330000 : 0x0);
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt(&popt, IRIS_IP_SYS, regval.opt_id, regval.opt_id, 0);
		is_ulps_enable = iris_disable_ulps(path);
		iris_update_pq_opt(&popt, 1, path);
		iris_enable_ulps(path, is_ulps_enable);
		// send more packages to ensure memory repair done
		if (on) {
			udelay(100);
			for (i = 0; i < 10; i++)
				iris_pmu_power_set(BSRAM_PWR, on);
		}
	} else {
		IRIS_LOGW("%s: cur %d == on %d", __func__, iris_bsram_power, on);
		return 2;
	}
	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

bool iris_pmu_bsram_get(void)
{
	return iris_bsram_power;
}

/* power on & off frc domain */
int iris_pmu_frc_set(bool on)
{
	int rt = 0;

	if (((debug_lp_opt & 0x4) == 0x4) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	rt = iris_pmu_power_set(FRC_PWR, on);
	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

bool iris_pmu_frc_get(void)
{
	return iris_pmu_power_get(FRC_PWR);
}

/* power on & off dsc unit domain */
int iris_pmu_dscu_set(bool on)
{
	int rt = 0;

	if (((debug_lp_opt & 0x8) == 0x8) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	rt = iris_pmu_power_set(DSCU_PWR, on);
	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

/* power on & off lce domain */
int iris_pmu_lce_set(bool on)
{
	int rt = 0;

	if (((debug_lp_opt & 0x10) == 0x10) && !on) {
		IRIS_LOGI("%s: not power down!", __func__);
		return 0;
	}
	rt = iris_pmu_power_set(LCE_PWR, on);
	iris_lce_power_status_set(on);

	IRIS_LOGI("%s: on - %d, rt - %d", __func__, on, rt);
	return rt;
}

/* lce dynamic pmu mask enable */
void iris_lce_dynamic_pmu_mask_set(bool enable)
{
	//TODO: compile error
	//enable = enable;
	IRIS_LOGI("%s(), %s", __func__, enable ? "true" : "false");
}

void iris_lce_power_status_set(bool enable)
{
	iris_lce_power = enable;

	IRIS_LOGI("%s: %d", __func__, enable);
}

bool iris_lce_power_status_get(void)
{
	return iris_lce_power;
}

/* trigger DMA to load */
void iris_dma_trigger_load(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	/* only effective when dynamic power gating off */
	if (!pcfg->lp_ctrl.dynamic_power) {
		if (iris_lce_power_status_get())
			iris_send_ipopt_cmds(IRIS_IP_DMA, 0xe1);
		else
			iris_send_ipopt_cmds(IRIS_IP_DMA, 0xe5);
	}
}

void iris_ulps_source_sel(enum iris_ulps_sel ulps_sel)
{
	struct iris_update_regval regval;
	struct iris_update_ipopt popt;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if ((debug_lp_opt & 0x200) == 0x200) {
		IRIS_LOGE("not set ulps source sel: %d", ulps_sel);
		return;
	}

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0xf1;
	regval.mask = 0x3;
	regval.value = ulps_sel;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt(&popt, IRIS_IP_SYS, regval.opt_id, regval.opt_id, 0);
	iris_update_pq_opt(&popt, 1, PATH_DSI);
	IRIS_LOGD("ulps source sel: %d", ulps_sel);

	if (ulps_sel == ULPS_NONE)
		pcfg->lp_ctrl.ulps_lp = false;
	else
		pcfg->lp_ctrl.ulps_lp = true;
}

bool iris_ulps_enable_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGI("ulps iris:%d", pcfg->lp_ctrl.ulps_lp);

	if (/*pcfg->display->panel->ulps_feature_enabled &&*/pcfg->lp_ctrl.ulps_lp)
		return true;
	else
		return false;
}

/* TE delay or EVS delay select.
 * 0: TE delay; 1: EVS delay
 */
void iris_te_select(int sel)
{
	struct iris_update_ipopt popt[IP_OPT_MAX];
	int len;
	bool is_ulps_enable = 0;
	uint8_t path = iris_pq_update_path;

	iris_init_ipopt_ip(popt, IP_OPT_MAX);

	iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_SYS,
			sel ? 0xe1 : 0xe0, 0x1);
	iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_TX,
			sel ? 0xe1 : 0xe0, 0x1);
	len = iris_update_ip_opt(popt, IP_OPT_MAX, IRIS_IP_DTG,
			sel ? 0xe1 : 0xe0, 0x0);
	is_ulps_enable = iris_disable_ulps(path);
	iris_update_pq_opt(popt, len, path);
	iris_enable_ulps(path, is_ulps_enable);

	IRIS_LOGD("%s: %s", __func__, (sel ? "EVS delay" : "TE delay"));
}

/*== Analog bypass related APIs ==*/



/* Switch ABYP by mipi commands
   enter_abyp: true -- Enter ABYP;
			   false -- Exit ABYP
*/
static void iris_mipi_abyp_switch(bool enter_abyp, int need_lock)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (enter_abyp) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		IRIS_LOGD("%s, Enter ABYP.", __func__);
	} else {
		if (pcfg && pcfg->iris_i2c_switch) {
			iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 5);
			udelay(1000);
		} else {
			iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
		}
		IRIS_LOGD("%s, Exit ABYP.", __func__);
	}
}

void iris_abyp_register_dump(void)
{
	u32 value = 0;
	u32 index = 0;
	u32 len = 0;
	int ret = 0;
	static u32 iris_register_list[] = {
		0xf0000018,
		0xf0000034,
		0xf0000060,
		0xf0000094,
		0xf00000c0,
		0xf00000c8,
		0xf00000d0,
		0xf000028c,
		0xf0000290,
		0xf1800000,
		0xf1800004,
		0xf180000c,
		0xf180002c,
		0xf1800300,
		0xf1800304,
		0xf1800440,
		0xf1800444,
		0xf1801494,
		0xf1240030,
		0xf125ffe4,
	};

	len =  sizeof(iris_register_list) / sizeof(u32);
	IRIS_LOGI("%s, len -- %d", __func__, len);
	for (index = 0; index < len; index++) {
		value = iris_register_list[index];
		ret = iris_i2c_conver_ocp_read(&value, 1, false);
		if (ret) {
			IRIS_LOGE("%s, i2c read fail, ret = %d", __func__, ret);
			break;
		}
		IRIS_LOGI("[%02d] - %08x : %08x", index, iris_register_list[index], value);
	}
}

static int _iris_dphy_itf_check(void)
{
	int i = 0;
	int ret = 0;
	u32 dphy_itf_ctrl = 0xf1800000;
	u32 arr[2] = {0};
	u32 val = 0;

	while (i < 10) {
		arr[0] = dphy_itf_ctrl;
		ret = iris_i2c_conver_ocp_read(arr, 1, false);
		if (ret) {
			IRIS_LOGE("%s, i2c read fail, ret = %d", __func__, ret);
			return -EINVAL;
		}

		val = arr[0];
		if ((val & 0x10) && (val & 0x10000000)) {
			IRIS_LOGI("%s, ST_DPHY_DSI_FLUSH is 1, value = 0x%08x", __func__, val);
			arr[0] = dphy_itf_ctrl;
			arr[1] = val | 0x01000000;
			iris_i2c_single_conver_ocp_write(arr, 1);
			arr[0] = dphy_itf_ctrl;
			arr[1] = val & (~0x01000000);
			iris_i2c_single_conver_ocp_write(arr, 1);
		} else
			break;
		i++;
		usleep_range(100, 101);
	}

	if (i == 10) {
		ret = -EINVAL;
		IRIS_LOGE("%s INIT_DPHY_RDY failed for %d time", __func__, i);
	}
	return ret;
}

static int _iris_lp_check_gpio_status(int cnt, int target_status)
{
	int i;
	int abyp_status_gpio;

	if (cnt <= 0) {
		IRIS_LOGE("invalid param, cnt is %d", cnt);
		return -EINVAL;
	}

	IRIS_LOGD("%s, cnt = %d, target_status = %d", __func__, cnt, target_status);

	/* check abyp gpio status */
	for (i = 0; i < cnt; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s, %d, ABYP status: %d.", __func__, i, abyp_status_gpio);
		if (abyp_status_gpio == target_status)
			break;
		udelay(3 * 1000);
	}

	return abyp_status_gpio;
}

static bool _iris_plus_cmd_abyp_enter(int mode)
{
	int abyp_status_gpio;
	int toler_cnt = 3;
	struct iris_cfg *pcfg = iris_get_cfg();
	int prev_mode = pcfg->abypss_ctrl.abypass_mode;

	IRIS_LOGI("Enter %d abyp mode start", mode);

	if (prev_mode == PASS_THROUGH_MODE) {
		/* HS enter abyp */
		iris_send_ipopt_cmds(IRIS_IP_SYS, 0x8);
		udelay(100);
enter_abyp_begin:
		/* check abyp gpio status */
		abyp_status_gpio = _iris_lp_check_gpio_status(50, 1);

		if (!abyp_status_gpio) {
			iris_abyp_register_dump();
			if (toler_cnt > 0) {
				IRIS_LOGW("Enter abyp retry %d", toler_cnt);
				iris_reset_chip();
				pcfg->iris_initialized = false;
				toler_cnt--;
				goto enter_abyp_begin;
			} else {
				IRIS_LOGE("Enter abyp mode Failed!");
				return true;
			}
		}
	}

	if (mode == ANALOG_BYPASS_MODE) {
		/*
		 * it would close MIPI domain
		 * and then close PLL and clock of sys and MIPI
		 */
		iris_send_one_wired_cmd(IRIS_POWER_DOWN_MIPI);
	} else if (mode == ABP_SLEEP_MODE) {
		iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);
	}

	pcfg->abypss_ctrl.abypass_mode = mode;
	IRIS_LOGI("Enter abyp done");
	return false;
}

static bool _iris_cmd_abyp_enter(int mode)
{
	struct iris_cfg *pcfg;
	int abyp_status_gpio, toler_cnt;
	ktime_t ktime0 = {0};
	ktime_t ktime1 = {0};
	uint32_t timeus;

	pcfg = iris_get_cfg();
	if (debug_lp_opt & 0x400)
		ktime0 = ktime_get();

	toler_cnt = 3;
	IRIS_LOGI("Enter abyp mode start");
	/* HS enter abyp */
	iris_send_ipopt_cmds(IRIS_IP_SYS, 0x8);
	udelay(100);

enter_abyp_begin:
	/* check abyp gpio status */
	abyp_status_gpio = _iris_lp_check_gpio_status(50, 1);
	if (abyp_status_gpio == 1) {
		if (debug_lp_opt & 0x400) {
			ktime1 = ktime_get();
			timeus = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			ktime0 = ktime1;
			IRIS_LOGI("spend time switch ABYP %d us", timeus);
		}
		if (pcfg->iris_i2c_switch) {
			//power off domains, switch clocks mux
			iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 0x22);
			//power off PLL, gate clocks
			iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 0x23);
		} else {
			//power off domains, switch clocks mux
			iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, 0x22);
			//power off PLL, gate clocks
			iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, 0x23);
		}
		IRIS_LOGD("ABYP enter LP");
		if (debug_lp_opt & 0x400) {
			ktime1 = ktime_get();
			timeus = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			IRIS_LOGI("spend time ABYP LP %d us", timeus);
		}

		pcfg->abypss_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
	}
	if (abyp_status_gpio == 0) {
		iris_abyp_register_dump();
		if (toler_cnt > 0) {
			IRIS_LOGW("Enter abyp retry %d", toler_cnt);
			iris_reset_chip();
			pcfg->iris_initialized = false;
			toler_cnt--;
			goto enter_abyp_begin;
		} else {
			IRIS_LOGE("Enter abyp mode Failed!");
			return true;
		}
	}
	IRIS_LOGI("Enter abyp done");
	return false;
}

static void _iris_reset_mipi(void)
{
	iris_send_one_wired_cmd(IRIS_POWER_DOWN_MIPI);
	usleep_range(3500, 3501);
	iris_send_one_wired_cmd(IRIS_POWER_UP_MIPI);
	usleep_range(3500, 3501);
}

static void _iris_plus_reset_mipi(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int prev_mode = pcfg->abypss_ctrl.abypass_mode;

	if (prev_mode == ABP_SLEEP_MODE) {
		pcfg->iris_initialized = false;
		iris_send_one_wired_cmd(IRIS_POWER_UP_SYS);
		usleep_range(3500, 3501);
	} else {
		iris_send_one_wired_cmd(IRIS_POWER_UP_MIPI);
		usleep_range(3500, 3501);
	}
}

static void _iris_pre_config_for_timing(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_initialized) {
		// it's same timing with last PT mode
		if (iris_is_same_timing_from_last_pt()) {
			if (pcfg->iris_i2c_switch) {
				//ungate clocks & power on PLLs
				iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 0x20);
				udelay(1000);
				//switch clock mux & power on domains
				iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 0x21);
				udelay(1000);
				//configure MIPI and other domains via DMA
				iris_send_ipopt_cmds_i2c(IRIS_IP_DMA, 0xE9);
				udelay(1000);
			} else {
				//ungate clocks & power on PLLs
				iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, 0x20);
				udelay(100);
				//switch clock mux & power on domains
				iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, 0x21);
				//configure MIPI and other domains via DMA
				iris_send_ipopt_cmds_nonlock(IRIS_IP_DMA, 0xE9);
				udelay(100);
			}
			IRIS_LOGD("configure DMA");
			IRIS_LOGI("%s(), same timing with last PT", __func__);
		} else {
			bool use_2nd_timing = iris_belongs_to_2nd_timing(&pcfg->timing);

			if (pcfg->iris_i2c_switch) {
				//ungate clocks && re-program PLL
				iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, use_2nd_timing ? 0x28 : 0x27);
				udelay(1000);
				//switch clock mux & power on domains
				iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 0x21);
				udelay(1000);

				//configure MIPI Rx
				iris_send_ipopt_cmds_i2c(IRIS_IP_RX, use_2nd_timing ? 0xF2 : 0xF1);
				udelay(1000);
				//configure MIPI and other domains via DMA
				iris_send_ipopt_cmds_i2c(IRIS_IP_DMA, 0xE9);
				udelay(1000);
			} else {
				//ungate clocks && re-program PLL
				iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, use_2nd_timing ? 0x28 : 0x27);
				udelay(100);
				//switch clock mux & power on domains
				iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, 0x21);

				//configure MIPI Rx
				iris_send_ipopt_cmds_nonlock(IRIS_IP_RX, use_2nd_timing ? 0xF2 : 0xF1);
			}
			IRIS_LOGI("%s(), different timing with last PT", __func__);
		}
	} else {
		if (pcfg->iris_i2c_switch) {
			//ungate clocks, power on MIPI PLL
			iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 0x24);
			udelay(1000);
			//switch clock mux default
			iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 0x25);
			udelay(1000);
		} else {
			//ungate clocks, power on MIPI PLL
			iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, 0x24);
			//switch clock mux default
			iris_send_ipopt_cmds_nonlock(IRIS_IP_SYS, 0x25);
		}
		IRIS_LOGD("ABYP exit LP default");
		IRIS_LOGI("%s(), enter PT by default", __func__);
	}
}

static void _iris_plus_pre_config_for_timing(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int prev_mode = pcfg->abypss_ctrl.abypass_mode;

	if (prev_mode == ABP_SLEEP_MODE)
		return;

	_iris_pre_config_for_timing();
}

static void _iris_post_config_for_timing(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg->iris_initialized) {
		iris_lightup();
		pcfg->iris_initialized = true;
		IRIS_LOGI("%s(), fully light up iris", __func__);
	} else {
		bool use_2nd_timing = iris_belongs_to_2nd_timing(&pcfg->timing);

		if (iris_is_clk_switched_from_last_pt() && !iris_is_between_main_2nd_from_last_pt()) {
			iris_send_timing_switch_pkt();
			iris_send_ipopt_cmds(IRIS_IP_RX, use_2nd_timing ? 0xE1 : 0xE0);
			iris_set_out_frame_rate(pcfg->timing.refresh_rate);
			iris_send_ipopt_cmds(IRIS_IP_DMA, 0xE9);
			udelay(200);
			IRIS_LOGI("%s(), for clock rate switch", __func__);
		} else if (iris_is_res_switched_from_last_pt()) {
			//resolution change (may have fps change)
			iris_send_timing_switch_pkt();
			iris_send_ipopt_cmds(IRIS_IP_RX, use_2nd_timing ? 0xE1 : 0xE0);
			iris_send_ipopt_cmds(IRIS_IP_DMA, 0xE9);
			udelay(200);
			IRIS_LOGI("%s(), for resolution switch", __func__);
		} else if (iris_is_freq_switched_from_last_pt()) {
			//only fps change
			iris_send_ipopt_cmds(IRIS_IP_RX, use_2nd_timing ? 0xE1 : 0xE0);
			iris_set_out_frame_rate(pcfg->timing.refresh_rate);
			iris_send_ipopt_cmds(IRIS_IP_DMA, 0xE9);
			udelay(200);
			IRIS_LOGI("%s(), for frequency switch", __func__);
		}

		iris_send_ipopt_cmds(IRIS_IP_SYS, ID_SYS_PMU_CTRL);
	}

	iris_update_last_pt_timing_index();
	iris_update_frc_fps(pcfg->timing.refresh_rate & 0xFF);

	pcfg->abypss_ctrl.pending_mode = MAX_MODE;
	pcfg->abypss_ctrl.abypass_mode = PASS_THROUGH_MODE;

	if (0 /*pcfg->panel->qsync_mode > 0*/)
		iris_qsync_set(true);

}

static void _iris_wait_prev_frame_done(void)
{
#if 0

	int i = 0;
	struct drm_encoder *drm_enc = iris_get_drm_encoder_handle();
	struct sde_encoder_virt *sde_enc = NULL;

	if (!drm_enc) {
		IRIS_LOGE("invalid encoder\n");
		return;
	}

	sde_enc = to_sde_encoder_virt(drm_enc);
	for (i = 0; i < sde_enc->num_phys_encs; i++) {
		struct sde_encoder_phys *phys = sde_enc->phys_encs[i];
		int pending_cnt = 0;

		if (phys->split_role != ENC_ROLE_SLAVE) {
			int j = 0;

			pending_cnt = atomic_read(&phys->pending_kickoff_cnt);
			for (j = 0; j < pending_cnt; j++)
				sde_encoder_wait_for_event(phys->parent, MSM_ENC_TX_COMPLETE);

			break;
		}
	}
#endif
}

static bool _iris_plus_cmd_abyp_exit(int mode)
{
	int ret = true;
	int abyp_status_gpio;
	int toler_cnt = 3;
	struct iris_cfg *pcfg = iris_get_cfg();
	int prev_mode = pcfg->abypss_ctrl.abypass_mode;

exit_abyp_loop:
	if (!pcfg->iris_i2c_switch) {
		IRIS_LOGI("Exit abyp mode start");
		pcfg->abyp_ops.mipi_reset();
		pcfg->abyp_ops.pre_config_for_timing();
	}
	if (prev_mode == ABP_SLEEP_MODE) {
		mutex_lock(&pcfg->kickoff_mutex);
		_iris_wait_prev_frame_done();
	}

	/* exit abyp */
	abyp_status_gpio = iris_exit_abyp(false);
	if (abyp_status_gpio == 0) {
		_iris_post_config_for_timing();

		if (prev_mode == ABP_SLEEP_MODE
			&& mode == ANALOG_BYPASS_MODE)
			_iris_plus_cmd_abyp_enter(mode);
	}

	if (prev_mode == ABP_SLEEP_MODE)
		mutex_unlock(&pcfg->kickoff_mutex);

	if (abyp_status_gpio) {
		iris_abyp_register_dump();
		if (toler_cnt > 0) {
			IRIS_LOGW("Exit abyp retry, %d", toler_cnt);
			iris_reset_chip();
			pcfg->iris_initialized = false;
			toler_cnt--;
			goto exit_abyp_loop;
		} else {
			IRIS_LOGE("Exit abyp failed!");
		}
	}
	return ret;
}


static bool _iris_cmd_abyp_exit(int mode)
{
	struct iris_cfg *pcfg;
	int abyp_status_gpio;
	int toler_cnt = 3;
	ktime_t ktime0 = {0};
	ktime_t ktime1 = {0};
	uint32_t timeus;

	pcfg = iris_get_cfg();

	if (debug_lp_opt & 0x400)
		ktime0 = ktime_get();

exit_abyp_loop:
	if (!pcfg->iris_i2c_switch) {
		IRIS_LOGI("Exit abyp mode start");
		_iris_reset_mipi();
		if (debug_lp_opt & 0x400) {
			ktime1 = ktime_get();
			timeus = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			ktime0 = ktime1;
			IRIS_LOGI("spend time MIPI off/on %d us", timeus);
		}

		_iris_pre_config_for_timing();
		if (pcfg->iris_initialized && (debug_lp_opt & 0x400)) {
			ktime1 = ktime_get();
			timeus = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			ktime0 = ktime1;
			IRIS_LOGI("spend time ABP send LP command %d us", timeus);
		}
	}

	/* exit abyp */
	abyp_status_gpio = iris_exit_abyp(false);
	if (debug_lp_opt & 0x400) {
		ktime1 = ktime_get();
		timeus = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
		ktime0 = ktime1;
		IRIS_LOGI("spend time switch to PT %d us", timeus);
	}

	if (abyp_status_gpio == 0) {
		_iris_post_config_for_timing();
		if (pcfg->iris_initialized && debug_lp_opt & 0x400) {
			ktime1 = ktime_get();
			timeus = (u32)ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
			IRIS_LOGI("spend time PT HS commmand %d us", timeus);
		}

		IRIS_LOGI("Exit abyp mode done");
		return true;
	}

	if (abyp_status_gpio != 0) {
		iris_abyp_register_dump();
		if (toler_cnt > 0) {
			IRIS_LOGW("Exit abyp retry, %d", toler_cnt);
			iris_reset_chip();
			pcfg->iris_initialized = false;
			toler_cnt--;
			goto exit_abyp_loop;
		} else {
			IRIS_LOGE("Exit abyp failed!");
		}
	}
	return false;
}

static void _iris_video_abyp_enter(void)
{
	//todo: I3C or i2c to save more power (one-wire power down/up sys)
	struct iris_cfg *pcfg;
	int i;
	int abyp_status_gpio;

	pcfg = iris_get_cfg();

	iris_send_ipopt_cmds(IRIS_IP_RX, ID_RX_ENTER_TTL);
	IRIS_LOGI("enter TTL bypass");

	//disable dynamic power gating, power down other domains
	_iris_video_abyp_power(false);
	IRIS_LOGI("power down other domains");

	iris_send_one_wired_cmd(IRIS_ENTER_ANALOG_BYPASS);
	IRIS_LOGI("enter abyp");

	/* check abyp gpio status */
	for (i = 0; i < 3; i++) {
		mdelay(10);
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s(%d), ABYP status: %d.", __func__, __LINE__, abyp_status_gpio);
		if (abyp_status_gpio == 1) {
			iris_send_one_wired_cmd(IRIS_POWER_DOWN_MIPI);
			//IRIS_LOGI("power down mipi");
			pcfg->abypss_ctrl.abypass_mode = ANALOG_BYPASS_MODE;
			break;
		}
	}
	//usleep_range(1000 * 40, 1000 * 40 + 1);
}

static void _iris_video_abyp_exit(void)
{
	struct iris_cfg *pcfg;
	int i;
	int abyp_status_gpio = 1;

	pcfg = iris_get_cfg();
	IRIS_LOGI("%s", __func__);

	if (!pcfg->iris_i2c_switch) {
		iris_send_one_wired_cmd(IRIS_POWER_UP_MIPI);
		mdelay(10);
	}

	iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
	//IRIS_LOGI("exit abyp");

	/* check abyp gpio status */
	for (i = 0; i < 3; i++) {
		mdelay(10);
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGI("%s(%d), ABYP status: %d.", __func__, __LINE__, abyp_status_gpio);
		if (abyp_status_gpio == 0) {
			pcfg->abypss_ctrl.abypass_mode = PASS_THROUGH_MODE;
			break;
		}
	}

	/*power up other domains manually */
	_iris_video_abyp_power(true);
	IRIS_LOGI("power up other domains");
	usleep_range(1000 * 20, 1000 * 20 + 1);

	/*configure other domains IP via DMA trigger */
	iris_send_ipopt_cmds(IRIS_IP_DMA, 0xe5);
	IRIS_LOGI("write dma to trigger other domains");
	usleep_range(1000 * 40, 1000 * 40 + 1);

	/*enable dynamic power gating if need */
	if (pcfg->lp_ctrl.dynamic_power) {
		IRIS_LOGI(" [%s, %d] open psr_mif osd first address eco.", __func__, __LINE__);
		iris_psf_mif_dyn_addr_set(true);
		iris_dynamic_power_set(true);
	} else {
		IRIS_LOGI(" [%s, %d] close psr_mif osd first address eco.", __func__, __LINE__);
		iris_psf_mif_dyn_addr_set(false);
	}
	//usleep_range(1000 * 40, 1000 * 40 + 1);
	iris_send_ipopt_cmds(IRIS_IP_RX, ID_RX_EXIT_TTL);
	IRIS_LOGI("exit TTL bypass");
	//usleep_range(1000 * 40, 1000 * 40 + 1);

}

static int _iris_plus_abyp_select(int dest_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int prev_mode = pcfg->abypss_ctrl.abypass_mode;

	if ((dest_mode == ANALOG_BYPASS_MODE && prev_mode == PASS_THROUGH_MODE)
		|| (dest_mode == ABP_SLEEP_MODE && prev_mode == PASS_THROUGH_MODE)
		|| (dest_mode == ABP_SLEEP_MODE && prev_mode == ANALOG_BYPASS_MODE))
		return ANALOG_BYPASS_MODE;
	else if ((dest_mode == PASS_THROUGH_MODE && prev_mode == ANALOG_BYPASS_MODE)
		|| (dest_mode == PASS_THROUGH_MODE && prev_mode == ABP_SLEEP_MODE)
		|| (dest_mode == ANALOG_BYPASS_MODE && prev_mode == ABP_SLEEP_MODE))
		return PASS_THROUGH_MODE;
	else
		return MAX_MODE;

}

static int _iris_abyp_select(int dest_mode)
{
	return dest_mode;
}



/* Switch PT and Bypass mode */
/* Return: true is PT, false is Bypass */
bool iris_abypass_switch_proc(int mode, bool pending, bool first)
{
	ktime_t ktime0 = 0;
	int dest_mode = mode & PT_ABP_MASK; //obtain pt or abyp mode
	int dport_status = mode & PQ_SWITCH_MASK;// check pq switch mask
	struct iris_cfg *pcfg = iris_get_cfg();
	int prev_mode = pcfg->abypss_ctrl.abypass_mode;
	bool pt_mode = iris_is_pt_mode();

	if (!pcfg->lp_ctrl.abyp_enable) {
		IRIS_LOGE("abyp is disable!");
		return pt_mode;
	}

	if (pcfg->rx_mode != pcfg->tx_mode) {
		IRIS_LOGE("abyp can't be supported! rx_mode != tx_mode!");
		return pt_mode;
	}

	if (pending) {
		mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
		pcfg->abypss_ctrl.pending_mode = dest_mode;
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
		return pt_mode;
	}
	if (debug_lp_opt & 0x1000)
		ktime0 = ktime_get();

	// Check GPIO or mipi inside abyp_enter, abyp_exit
	if (pcfg->abyp_ops.abyp_select(dest_mode) == ANALOG_BYPASS_MODE) {
		mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);

		if (pcfg->rx_mode == DSI_OP_CMD_MODE) { /* command mode */
			pt_mode = pcfg->abyp_ops.abyp_enter(mode & 0x3);
		} else {
			_iris_video_abyp_enter();
			pt_mode = false;
		}
		iris_clean_frc_status(pcfg);

		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
	} else if (pcfg->abyp_ops.abyp_select(dest_mode) == PASS_THROUGH_MODE) {
		if (pcfg->iris_i2c_switch) {
			IRIS_LOGI("Exit abyp mode start");
			pcfg->abyp_ops.mipi_reset();
			pcfg->abyp_ops.pre_config_for_timing();
		}

		mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
		if (pcfg->rx_mode == DSI_OP_CMD_MODE) {
			pt_mode = pcfg->abyp_ops.abyp_exit(mode & 0x3);
		} else {
			_iris_video_abyp_exit();
			pt_mode = true;
		}
		/* Soft Iris switch iris PQ: Close dport after enter PT imediately,
		 * IrisService will open dport after PQ switch
		 */
		if (dport_status && pt_mode)
			iris_dom_set(0);
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
	} else
		IRIS_LOGE("%s: switch mode: %d not supported!", __func__, mode);

	if (debug_lp_opt & 0x1000) {
		IRIS_LOGI("%s mode: %d -> %d spend time %d", __func__, prev_mode, dest_mode,
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime0));
	}

	return pt_mode;
}

void iris_abyp_lp(int mode)
{
	int abyp_status_gpio;

	abyp_status_gpio = iris_check_abyp_ready();
	IRIS_LOGD("%s(%d), ABYP status: %d, lp_mode: %d",
			__func__, __LINE__, abyp_status_gpio, mode);

	if (abyp_status_gpio == 1) {
		if (mode == ABYP_POWER_DOWN_SYS)
			iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);
		else if (mode == ABYP_POWER_DOWN_MIPI)
			iris_send_one_wired_cmd(IRIS_POWER_DOWN_MIPI);
		else if (mode == ABYP_POWER_DOWN_PLL)
			iris_send_ipopt_cmds(IRIS_IP_SYS, 0x26);
		else
			IRIS_LOGW("[%s:%d] mode: %d error", __func__, __LINE__, mode);
	} else {
		IRIS_LOGW("iris is not in ABYP mode");
	}

}

static int _iris_exit_abyp(bool one_wired, int need_lock)
{
	int i = 0;
	int abyp_status_gpio = 0;

	_iris_dphy_itf_check();
	/* try to exit abyp */
	if (one_wired) {
		iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
		mdelay(2);
	} else {
		iris_mipi_abyp_switch(false, need_lock); /* switch by MIPI command */
		udelay(100);
	}
	IRIS_LOGI("send exit abyp, one_wired:%d.", one_wired);

	/* check abyp gpio status */
	for (i = 0; i < 10; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s, ABYP status: %d.", __func__, abyp_status_gpio);
		if (abyp_status_gpio == 0)
			break;
		udelay(3 * 1000);
	}

	return abyp_status_gpio;
}


int iris_exit_abyp(bool one_wired)
{
	return _iris_exit_abyp(one_wired, 1);
}

int iris_lightup_opt_get(void)
{
	return debug_on_opt;
}

void iris_lp_setting_off(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->abypss_ctrl.pending_mode = MAX_MODE;
}

int iris_esd_ctrl_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.esd_ctrl;
}

void iris_qsync_set(bool enable)
{
	struct iris_cfg *pcfg;
	struct iris_ip_opt *psopt;
	uint32_t *data;
	uint32_t val = 0;
	int32_t ip = 0;
	int32_t opt_id = 0;
	bool use_2nd_timing = false;

	if (!iris_is_chip_supported())
		return;

	pcfg = iris_get_cfg();
	IRIS_LOGD("%s, mode: %d. enable: %d", __func__, pcfg->abypss_ctrl.abypass_mode, enable);

	if (iris_is_pt_mode()) {
		use_2nd_timing = iris_belongs_to_2nd_timing(&iris_get_cfg()->timing);

		ip = IRIS_IP_DTG;
		opt_id = use_2nd_timing ? 0x01 : 0x00;
		psopt = iris_find_ip_opt(ip, opt_id);
		if (psopt == NULL) {
			IRIS_LOGE("%s, can not find ip_ = %02x opt_id = %02x", __func__, ip, opt_id);
			return;
		}
		data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
		val = data[11];

		opt_id = enable ? 0x11 : 0x10;
		psopt = iris_find_ip_opt(ip, opt_id);
		if (psopt == NULL) {
			IRIS_LOGE("%s, can not find ip_ = %02x opt_id = %02x", __func__, ip, opt_id);
			return;
		}
		data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
		data[4] = data[2] = enable ? (val & 0x7fffffff) : val;

		iris_send_ipopt_cmds(IRIS_IP_DTG, enable ? 0x11 : 0x10);
	}
}

#define CHECK_KICKOFF_FPS_CADNENCE
#if defined(CHECK_KICKOFF_FPS_CADNENCE)
int getFrameDiff(long timeDiff)
{
	int frameDiff;

	if (timeDiff < 11) // 16.7-5 ms
		frameDiff = 0;
	else if (timeDiff < 28) // 33.3-5
		frameDiff = 1;
	else if (timeDiff < 45)    // 50-5
		frameDiff = 2;
	else if (timeDiff < 61)    // 66.7-5
		frameDiff = 3;
	else if (timeDiff < 78)    // 83.3-5
		frameDiff = 4;
	else if (timeDiff < 95)    // 100 - 5
		frameDiff = 5;
	else if (timeDiff < 111)   // 116.7 - 5
		frameDiff = 6;
	else
		frameDiff = 7;
	return frameDiff;
}

#define CHECK_KICKOFF_FPS_DURATION      5 /*EVERY 5s*/

void iris_check_kickoff_fps_cadence(void)
{
	static u32 kickoff_cnt;
	u32 timeusDelta = 0;
	static ktime_t ktime_kickoff_start;
	static u32 us_last_kickoff;
	ktime_t ktime_kickoff;
	static u32 cadence[10];
	static int cdIndex;
	u32 us_timediff;

	if (kickoff_cnt == 0) {
		kickoff_cnt++;
		ktime_kickoff_start = ktime_get();
		memset(cadence, 0, sizeof(cadence));
		cdIndex = 0;
		cadence[cdIndex++] = 0;
		us_last_kickoff = (u32)ktime_to_us(ktime_kickoff_start);
	} else {
		kickoff_cnt++;
		ktime_kickoff = ktime_get();
		timeusDelta = (u32)ktime_to_us(ktime_kickoff) - (u32)ktime_to_us(ktime_kickoff_start);
		us_timediff = (u32)ktime_to_us(ktime_kickoff) - us_last_kickoff;
		us_last_kickoff = (u32)ktime_to_us(ktime_kickoff);
		if (cdIndex > 9)
			cdIndex = 0;

		cadence[cdIndex++] = getFrameDiff((us_timediff+500)/1000);//16667
		if (timeusDelta > 1000000*CHECK_KICKOFF_FPS_DURATION) {
			if ((debug_trace_opt&IRIS_TRACE_FPS) == IRIS_TRACE_FPS)
				IRIS_LOGI("iris: kickoff fps % d", kickoff_cnt/CHECK_KICKOFF_FPS_DURATION);
			if ((debug_trace_opt&IRIS_TRACE_CADENCE) == IRIS_TRACE_CADENCE)
				IRIS_LOGI("iris: Latest cadence: %d %d %d %d %d, %d %d %d %d %d",
						cadence[0], cadence[1], cadence[2], cadence[3], cadence[4],
						cadence[5], cadence[6], cadence[7], cadence[8], cadence[9]);
			kickoff_cnt = 0;
		}
	}
}
#endif

void iris_set_metadata(bool panel_lock)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t dport_meta = pcfg->metadata & 0x3;
	uint32_t osd_meta = (pcfg->metadata >> 2) & 0x3;
	uint32_t dpp_meta = (pcfg->metadata >> 4) & 0x3;

	if (pcfg->metadata == 0)
		return;
	if (pcfg->iris_initialized == false) {
		pcfg->metadata = 0;
		IRIS_LOGI("clean metadata when iris not initialized");
		return;
	}
	IRIS_LOGI("dport_meta: %x, osd_meta: %x, dpp_meta: %x", dport_meta, osd_meta, dpp_meta);
	pcfg->metadata = 0;

	IRIS_ATRACE_BEGIN("iris_set_metadata");

	if (panel_lock)
		//mutex_lock(&pcfg->panel->panel_lock);
	// bit: 0x: nothing, 10: disable dport, 11: enable dport
	switch (dport_meta) {
	case 0x2:
		iris_dom_set(0);
		break;
	case 0x3:
		iris_dom_set(2);
		break;
	default:
		break;
	}
	// bit: 0x: nothing, 10: disable dual, 11: enable dual
	switch (osd_meta) {
	case 0x2:
		iris_switch_osd_blending(0);
		break;
	case 0x3:
		iris_switch_osd_blending(1);
		break;
	default:
		break;
	}

	switch (dpp_meta) {
	case 0x2:
		iris_pwil_dpp_en(false);
		break;
	case 0x3:
		iris_pwil_dpp_en(true);
		break;
	default:
		break;
	}

	if (panel_lock)
		;//mutex_unlock(&pcfg->panel->panel_lock);

	IRIS_ATRACE_END("iris_set_metadata");
}

// prepare_commit before prepare_for_kickoff
int iris_prepare_commit(void)
{

	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (0 /*iris_virtual_display(display)*/) {
		// retain iris_osd_autorefresh
		pcfg->iris_cur_osd_autorefresh = pcfg->iris_osd_autorefresh;
	}

	return 0;
}

int iris_prepare_for_kickoff(struct mtk_ddp_comp *comp)
{
	struct iris_cfg *pcfg;
	int mode;

	pcfg = iris_get_cfg();
	if (/*iris_virtual_display(display) || */pcfg->valid < PARAM_PARSED)
		return 0;

#if defined(CHECK_KICKOFF_FPS_CADNENCE)
	if (debug_trace_opt > 0)
		iris_check_kickoff_fps_cadence();
#endif
	mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
	if (pcfg->abypss_ctrl.pending_mode != MAX_MODE) {
		mode = pcfg->abypss_ctrl.pending_mode;
		pcfg->abypss_ctrl.pending_mode = MAX_MODE;
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
		//iris_ddp_mutex_lock();
		iris_abypass_switch_proc(mode, false, false);
		//iris_ddp_mutex_unlock();
	} else
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);

	iris_set_metadata(true);
	return 0;
}

void iris_power_up_mipi(void)
{
#if 0

	struct iris_cfg *pcfg = iris_get_cfg();

	struct dsi_display *display = pcfg->display;

	if (iris_virtual_display(display))
		return;
#endif
	IRIS_LOGI("%s(%d), power up mipi", __func__, __LINE__);
	iris_send_one_wired_cmd(IRIS_POWER_UP_MIPI);
	usleep_range(3500, 3501);
}

void iris_reset_mipi(void)
{
	iris_send_one_wired_cmd(IRIS_POWER_DOWN_MIPI);
	IRIS_LOGI("%s(%d), power down mipi", __func__, __LINE__);
	usleep_range(3500, 3501);

	iris_send_one_wired_cmd(IRIS_POWER_UP_MIPI);
	IRIS_LOGI("%s(%d), power up mipi", __func__, __LINE__);
	usleep_range(3500, 3501);
}

int iris_get_abyp_mode(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGD("%s(%d), abyp mode: %d", __func__, __LINE__,
			pcfg->abypss_ctrl.abypass_mode);

	return pcfg->abypss_ctrl.abypass_mode;
}

bool iris_is_pt_mode(void)
{
	return iris_get_abyp_mode() == PASS_THROUGH_MODE;
}
EXPORT_SYMBOL(iris_is_pt_mode);

/*== Low Power debug related ==*/

static ssize_t _iris_abyp_dbg_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
#if 0
	if (!mutex_trylock(&pcfg->panel->panel_lock))
		return -EFAULT;
#endif

	if (val == 0) {
		if (pcfg->iris_i2c_switch) {
			IRIS_LOGI("Exit abyp mode start");
			pcfg->abyp_ops.mipi_reset();
			pcfg->abyp_ops.pre_config_for_timing();
		}
		mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
		pcfg->abyp_ops.abyp_exit(val);
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
		IRIS_LOGI("abyp->pt");
	} else if (val == 1) {
		mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
		pcfg->abyp_ops.abyp_enter(ANALOG_BYPASS_MODE);
		mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
		IRIS_LOGI("pt->standby abyp");
	} else if (val == 2) {
		if (pcfg->chip_id == IRIS_CHIP_VER_1) {
			mutex_lock(&pcfg->abypss_ctrl.abypass_mutex);
			pcfg->abyp_ops.abyp_enter(ABP_SLEEP_MODE);
			mutex_unlock(&pcfg->abypss_ctrl.abypass_mutex);
			IRIS_LOGI("pt->sleep abyp");
		}
	} else if (val == 3) {
		_iris_video_abyp_enter();
	} else if (val == 4) {
		_iris_video_abyp_exit();
	} else if (val >= 10 && val <= 18) {
		IRIS_LOGI("%s one wired %d", __func__, (int)(val - 10));
		iris_send_one_wired_cmd((int)(val - 10));
	} else if (val == 20) {
		if (pcfg->iris_i2c_switch) {
			iris_send_ipopt_cmds_i2c(IRIS_IP_SYS, 5);
		} else {
			iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
		}
		IRIS_LOGI("miniPMU abyp->pt");
	} else if (val == 21) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		IRIS_LOGI("miniPMU pt->abyp");
	} else if (val == 22) {
		iris_send_ipopt_cmds(IRIS_IP_TX, 4);
		IRIS_LOGI("Enable Tx");
	} else if (val == 23) {
		// mutex_lock(&g_debug_mfd->switch_lock);
		iris_lightup();
		// mutex_unlock(&g_debug_mfd->switch_lock);
		IRIS_LOGI("lightup Iris abyp_panel_cmds");
	} else if (val == 24) {
		iris_abypass_switch_proc(PASS_THROUGH_MODE, false, true);
	} else if (val == 25) {
		iris_abypass_switch_proc(ANALOG_BYPASS_MODE, false, true);
	} else if (val == 100) {
		debug_abyp_gpio_status = iris_check_abyp_ready();
	}
	//mutex_unlock(&pcfg->panel->panel_lock);
	return count;
}

static ssize_t _iris_abyp_dbg_read(struct file *file, char __user *buff,
								  size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	int buf_len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	buf_len = sizeof(bp);

	if (*ppos)
		return 0;

	tot = scnprintf(bp, buf_len,
					"abyp status gpio: %d\n", iris_check_abyp_ready());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp mode: %d\n", pcfg->abypss_ctrl.abypass_mode);
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp on_opt: %d\n", debug_on_opt);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t _iris_lp_dbg_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
#if 0
	if (!mutex_trylock(&pcfg->panel->panel_lock))
		return -EFAULT;
#endif

	if (val == 0) {
		iris_dynamic_power_set(false);
		iris_ulps_source_sel(ULPS_NONE);
		IRIS_LOGE("disable dynamic & ulps low power.");
	} else if (val == 1) {
		iris_dynamic_power_set(true);
		iris_ulps_source_sel(ULPS_MAIN);
		IRIS_LOGE("enable dynamic & ulps low power.");
	} else if (val == 2) {
		IRIS_LOGE("dynamic power: %d", iris_dynamic_power_get());
		IRIS_LOGE("abyp enable: %d", pcfg->lp_ctrl.abyp_enable);
		IRIS_LOGE("ulps enable: %d", iris_ulps_enable_get());
	} else if (val == 3) {
		pcfg->lp_ctrl.abyp_enable = true;
		IRIS_LOGE("enable abyp.");
	} else if (val == 4) {
		pcfg->lp_ctrl.abyp_enable = false;
		IRIS_LOGE("disable abyp.");
	} else if (val == 5) {
		iris_ulps_source_sel(ULPS_MAIN);
		IRIS_LOGE("enable iris ulps lp.");
	} else if (val == 6) {
		iris_ulps_source_sel(ULPS_NONE);
		IRIS_LOGE("disable iris ulps lp.");
	} else if (val == 11) {
		iris_pmu_mipi2_set(true);
	} else if (val == 12) {
		iris_pmu_mipi2_set(false);
	} else if (val == 13) {
		iris_pmu_bsram_set(true);
	} else if (val == 14) {
		iris_pmu_bsram_set(false);
	} else if (val == 15) {
		iris_pmu_frc_set(true);
	} else if (val == 16) {
		iris_pmu_frc_set(false);
	} else if (val == 17) {
		iris_pmu_dscu_set(true);
	} else if (val == 18) {
		iris_pmu_dscu_set(false);
	} else if (val == 19) {
		iris_pmu_lce_set(true);
	} else if (val == 20) {
		iris_pmu_lce_set(false);
	} else if (val == 254) {
		IRIS_LOGI("lp_opt usages:");
		IRIS_LOGI("bit 0 -- MIPI2");
		IRIS_LOGI("bit 1 -- BSRAM");
		IRIS_LOGI("bit 2 -- FRC");
		IRIS_LOGI("bit 3 -- DSCU");
		IRIS_LOGI("bit 4 -- LCE");
	} else if (val == 255) {
		IRIS_LOGI("lp debug usages:");
		IRIS_LOGI("0  -- disable dynamic & ulps low power.");
		IRIS_LOGI("1  -- enable dynamic & ulps low power.");
		IRIS_LOGI("2  -- show low power flag.");
		IRIS_LOGI("3  -- enable abyp.");
		IRIS_LOGI("4  -- disable abyp.");
		IRIS_LOGI("11 -- enable mipi2 power.");
		IRIS_LOGI("12 -- disable mipi2 power.");
		IRIS_LOGI("13 -- enable bsram power.");
		IRIS_LOGI("14 -- disable bram power.");
		IRIS_LOGI("15 -- enable frc power.");
		IRIS_LOGI("16 -- disable frc power.");
		IRIS_LOGI("17 -- enable dsc unit power.");
		IRIS_LOGI("18 -- disable dsc unit power.");
		IRIS_LOGI("19 -- enable lce power.");
		IRIS_LOGI("20 -- disable lce power.");
		IRIS_LOGI("254 -- show lp_opt usages.");
		IRIS_LOGI("255 -- show debug usages.");
	}
	//mutex_unlock(&pcfg->panel->panel_lock);
	return count;
}

static ssize_t _iris_esd_dbg_write(struct file *file,
								 const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
#if 0
	if (!mutex_trylock(&pcfg->panel->panel_lock))
		return -EFAULT;
#endif

	if (val >= 0 && val < 32) {
		pcfg->lp_ctrl.esd_ctrl = val;
	} else if (val > 40 && val < 44) {
		pcfg->lp_ctrl.esd_trigger = val - 40;
		IRIS_LOGI("Set ESD trigger: %d", pcfg->lp_ctrl.esd_trigger);
	} else if (val == 100) {
		IRIS_LOGI("clear ESD count!");
		pcfg->lp_ctrl.esd_cnt_iris = 0;
		pcfg->lp_ctrl.esd_cnt_panel = 0;
		pcfg->lp_ctrl.esd_trigger = 0;
	}
	//mutex_unlock(&pcfg->panel->panel_lock);
	return count;
}

static ssize_t _iris_esd_dbg_read(struct file *file, char __user *buff,
								 size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	int buf_len;
	struct iris_cfg *pcfg;
//	struct drm_panel_esd_config *esd_config;

	pcfg = iris_get_cfg();

	buf_len = sizeof(bp);

	if (*ppos)
		return 0;
#if 0
	esd_config = &(pcfg->panel->esd_config);


	switch (esd_config->status_mode) {
		case ESD_MODE_REG_READ:
			tot = scnprintf(bp, buf_len, "ap esd mode: %s\n", "reg_read");
			break;
		case ESD_MODE_PANEL_TE:
			tot = scnprintf(bp, buf_len, "ap esd mode: %s\n", "te_signal_check");
			break;
		case ESD_MODE_SW_SIM_FAILURE:
			tot = scnprintf(bp, buf_len, "ap esd mode: %s\n", "esd_sw_sim_failure");
			break;
		case ESD_MODE_SW_SIM_SUCCESS:
			tot = scnprintf(bp, buf_len, "ap esd mode: %s\n", "esd_sw_sim_success");
			break;
		default:
			tot = scnprintf(bp, buf_len, "ap esd mode: %s\n", "invalid");
			break;
	}
#endif
	tot += scnprintf(bp + tot, buf_len - tot, "esd ctrl: %d\n", pcfg->lp_ctrl.esd_ctrl);
	tot += scnprintf(bp + tot, buf_len - tot, "iris esd cnt: %d\n", pcfg->lp_ctrl.esd_cnt_iris);
	tot += scnprintf(bp + tot, buf_len - tot, "panel esd cnt: %d\n", pcfg->lp_ctrl.esd_cnt_panel);
	tot += scnprintf(bp + tot, buf_len - tot, "esd trigger: %d\n", pcfg->lp_ctrl.esd_trigger);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

int iris_dbgfs_lp_init(void)
{
	struct iris_cfg *pcfg;
	static const struct file_operations iris_abyp_dbg_fops = {
		.open = simple_open,
		.write = _iris_abyp_dbg_write,
		.read = _iris_abyp_dbg_read,
	};

	static const struct file_operations iris_lp_dbg_fops = {
		.open = simple_open,
		.write = _iris_lp_dbg_write,
	};

	static const struct file_operations iris_esd_dbg_fops = {
		.open = simple_open,
		.write = _iris_esd_dbg_write,
		.read = _iris_esd_dbg_read,
	};

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("lp_opt", 0644, pcfg->dbg_root,
			(u32 *)&debug_lp_opt);

	debugfs_create_u32("abyp_opt", 0644, pcfg->dbg_root,
			(u32 *)&debug_on_opt);

	debugfs_create_u32("abyp_gpio", 0644, pcfg->dbg_root,
			(u32 *)&debug_abyp_gpio_status);

	debugfs_create_u32("trace", 0644, pcfg->dbg_root,
			(u32 *)&debug_trace_opt);

	debugfs_create_u32("dual_test", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->dual_test);

	debugfs_create_u32("default_abyp_mode", 0644, pcfg->dbg_root,
			(u32 *)&(pcfg->abypss_ctrl.default_abyp_mode));

	if (debugfs_create_file("abyp", 0644, pcfg->dbg_root, NULL,
				&iris_abyp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("lp", 0644, pcfg->dbg_root, NULL,
				&iris_lp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("esd", 0644, pcfg->dbg_root, NULL,
				&iris_esd_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}
