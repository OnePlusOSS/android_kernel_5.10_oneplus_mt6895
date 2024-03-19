#include <drm/drm_mipi_dsi.h>

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <video/mipi_display.h>
#include <linux/delay.h>

#include "mtk_drm_mmp.h"
#include "mtk_drm_crtc.h"
#include "mtk_log.h"

#include "iris_api.h"
#include "iris_lightup.h"
#include "iris_lightup_ocp.h"
#include "iris_lp.h"
#include "iris_log.h"
#include "iris_i3c.h"
#include "iris_mtk_api.h"

#define IRIS_TX_HV_PAYLOAD_LEN   80
#define IRIS_TX_PAYLOAD_LEN 84
#define IRIS_RD_PACKET_DATA  0xF189C018
#define IRIS_RD_PACKET_DATA_I3  0xF0C1C018

extern int iris_w_path_select;
extern int iris_r_path_select;
static char iris_read_cmd_rbuf[16];
static struct iris_ocp_cmd ocp_cmd;
static struct iris_ocp_cmd ocp_test_cmd[DSI_CMD_CNT];
static struct iris_cmd_desc iris_test_cmd[DSI_CMD_CNT];
static struct iris_mtk_dsi_op *gp_mtk_dsi_op;
static bool dsi_cmd_log_enable;

#define IRIS_DSI_READ_CMD(type)                                         \
	((type == MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM) ||                    \
	 (type == MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM) ||                    \
	 (type == MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM) ||                    \
	 (type == MIPI_DSI_DCS_READ) ||                                        \
	 (type == MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE))

struct iris_mtk_dsi_op *iris_get_mtk_dsi_op(void)
{
	return gp_mtk_dsi_op;
}

void iris_set_mtk_dsi_op(struct iris_mtk_dsi_op *op)
{
	gp_mtk_dsi_op = op;
}


void iris_print_cmd_desc(const struct mipi_dsi_msg *msg)
{
	char buf[1024];
	int len = 0;
	size_t i;
	char *tx_buf;

	if (!dsi_cmd_log_enable || !msg)
		return;

	tx_buf = (char*)msg->tx_buf;
	/* Packet Info */
	len += snprintf(buf, sizeof(buf) - len,  "%02X ", msg->type);
	/* Low power bit */
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", (msg->flags & MIPI_DSI_MSG_USE_LPM) ? 1 : 0);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", msg->channel);
	len += snprintf(buf + len, sizeof(buf) - len, "%02X ", (unsigned int)msg->flags);
	/* Delay */
	len += snprintf(buf + len, sizeof(buf) - len, "%02X %02X ", msg->tx_len >> 8, msg->tx_len & 0x00FF);

	/* Packet Payload */
	for (i = 0 ; i < msg->tx_len ; i++) {
		len += snprintf(buf + len, sizeof(buf) - len, "%02X ", tx_buf[i]);
		/* Break to prevent show too long command */
		if (i > 250)
			break;
	}
	IRIS_LOGE("%s", buf);
}

extern bool mtk_crtc_is_connector_enable(struct mtk_drm_crtc *mtk_crtc);
int iris_create_cmdq_handle(struct cmdq_pkt **phandle, bool *is_frame_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_crtc *crtc = pcfg->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle = NULL;

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s:%d, crtc is slept\n", __func__,
				__LINE__);
		return -EINVAL;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	*phandle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	cmdq_handle = *phandle;

	*is_frame_mode = mtk_crtc_is_frame_trigger_mode(crtc);

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);

	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
				&& mtk_crtc->panel_ext->params->oplus_cmdq_pkt_set_event) {
		if (*is_frame_mode && mtk_crtc_is_connector_enable(mtk_crtc)) {
			cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
			cmdq_pkt_wfe(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		}
	}
	return 0;
}

int iris_create_conti_cmdq_handle(struct cmdq_pkt **phandle, bool *is_frame_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_crtc *crtc = pcfg->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle = NULL;

	if (!(mtk_crtc->enabled)) {
		DDPPR_ERR("%s:%d, crtc is slept\n", __func__,
				__LINE__);
		return -EINVAL;
	}

	*phandle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	cmdq_handle = *phandle;

	return 0;
}

int iris_destory_cmdq_handle(struct cmdq_pkt **phandle, bool is_frame_mode)
{
	struct cmdq_pkt *cmdq_handle = *phandle;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_crtc *crtc = pcfg->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int ret;

	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
				&& mtk_crtc->panel_ext->params->oplus_cmdq_pkt_set_event) {
		if (is_frame_mode && mtk_crtc_is_connector_enable(mtk_crtc)) {
			cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		}
	}
	ret = cmdq_pkt_flush(cmdq_handle);
	if (ret < 0) {
		DDPPR_ERR("%s:%d, flush error:%d\n", __func__, __LINE__, ret);
		if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
					&& mtk_crtc->panel_ext->params->oplus_cmdq_pkt_set_event) {
			g_mobile_log = 1;
			mtk_drm_crtc_analysis(crtc);
			mtk_drm_crtc_dump(crtc);
			g_mobile_log = 0;
		}
	}
	cmdq_pkt_destroy(cmdq_handle);

	return 0;
}

int iris_destory_conti_cmdq_handle(struct cmdq_pkt **phandle, bool is_frame_mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct drm_crtc *crtc = pcfg->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle = *phandle;

	if (is_frame_mode)
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	return 0;
}

int iris_send_cmdq_cmds(struct iris_cmd_set *pset, int need_lock, struct cmdq_pkt *cmdq_handle)
{
	bool is_frame_mode;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct cmdq_pkt *temp_handle;
	int i = 0;
	int count = pset->count;
	int div_base = 10;
	int div_count = div_base;
	int curr = 0;
	struct iris_cmd_desc *pdesc = NULL;

	if (!cmdq_handle) {
		DDPINFO("%s:%d pset:0x%p, handle:0x%p, need_lock:%d, count:%d\n",
				__func__, __LINE__,
				pset, cmdq_handle,
				need_lock, count);

		if (pset) {
			if (count > div_base)
				div_count = div_base;
			else
				div_count = count;

			while (div_count) {
				if (iris_create_cmdq_handle(&temp_handle, &is_frame_mode))
					return -EINVAL;

				for (i = 0; i < div_count; i++) {
					pdesc = pset->cmds + curr + i;

					iris_get_mtk_dsi_op()->transfer(pcfg->mtk_comp,
							temp_handle, (void *)pdesc->msg.tx_buf, pdesc->msg.tx_len, pset->state);
				}

				iris_destory_cmdq_handle(&temp_handle, is_frame_mode);

				count -= div_count;
				curr += div_count;
				if (count > div_base)
					div_count = div_base;
				else
					div_count = count;
			}

			DDPINFO("%s:%d count:%d, curr:%d, div:%d\n",
					__func__, __LINE__,
					pset->count,
					curr, div_count);
		}
	} else {
		DDPINFO("%s:%d pset:0x%p, handle:0x%p, need_lock:%d, count:%d\n",
				__func__, __LINE__,
				pset, cmdq_handle,
				need_lock, count);

		if (pset) {
			for (i = 0; i < count; i++) {
				pdesc = pset->cmds + i;

				iris_get_mtk_dsi_op()->transfer(pcfg->mtk_comp,
						cmdq_handle, (void *)pdesc->msg.tx_buf, pdesc->msg.tx_len, pset->state);
			}
		}
	}

	return 0;
}

static void _iris_pt_send_cmds(struct iris_cmd_desc *ptx_cmds, u32 cmds_cnt, enum dsi_cmd_set_state state);


bool iris_is_abyp_mode(void)
{
	return iris_get_cfg()->abypss_ctrl.abypass_mode
		 == ANALOG_BYPASS_MODE;
}

static void iris_send_noncmdq_cmds(struct iris_cmd_desc *cmds, u32 cnt,
		enum dsi_cmd_set_state state)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (i = 0; i < cnt; i++) {
		mipi_dsi_generic_write(pcfg->dsi_dev, cmds[i].msg.tx_buf, cmds[i].msg.tx_len);
	}
}

void iris_i2c_send_cmds(struct iris_cmd_set *pset)
{
	u32 header;
	int cmd_indx = 0;
	struct iris_cmd_desc *pdesc = pset->cmds;
	u32 *tx_buf = (u32 *)pdesc->msg.tx_buf;
	u32 len = (pdesc->msg.tx_len >> 2);


	for (cmd_indx = 0; cmd_indx < pset->count; cmd_indx++) {
		memcpy(&header, tx_buf, sizeof(*tx_buf) );
		pr_err("iris_i2c: header = %x, len = %d\n", header, pdesc->msg.tx_len);
		switch (header & 0x0f) {
		case OCP_SINGLE_WRITE_BYTEMASK:
			iris_i2c_ocp_single_write(tx_buf + 1, len -1);
			break;
		case OCP_BURST_WRITE:
			iris_i2c_ocp_burst_write(tx_buf + 1, len - 1);
			break;
		case PXLW_DIRECTBUS_WRITE:
			iris_i2c_direct_write(tx_buf + 1, len -1, header);
			break;
		default:
			break;
		}
		pdesc++;
	}

}

static void _iris_dsi_send_cmds(struct iris_cmd_desc *cmds, u32 cnt,
		enum dsi_cmd_set_state state,
		int need_lock,
		struct cmdq_pkt *cmdq_handle)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_set cmdset = {
		.state = state,
		.count = cnt,
		.cmds = cmds,
	};

	if (state == DSI_CMD_SET_STATE_LP)
		pcfg->dsi_dev->mode_flags |= MIPI_DSI_MODE_LPM;
	else
		pcfg->dsi_dev->mode_flags &= ~(MIPI_DSI_MODE_LPM);

	if (pcfg->valid == PARAM_PARSED)
		iris_send_noncmdq_cmds(cmds, cnt, state);
	else if (pcfg->valid > PARAM_PARSED) {
		if (pcfg->valid == PARAM_PREPARED)
			cmdset.state = DSI_CMD_SET_STATE_LP;
#ifdef IRIS_I2C_ENABLE
		iris_i2c_send_cmds(&cmdset);
#else
		iris_send_cmdq_cmds(&cmdset, need_lock, cmdq_handle);
#endif
	}
}

int32_t iris_send_cmdset(struct iris_cmd_set *pcmd_set)
{
	int ret = -EINVAL;
	if (pcmd_set == NULL)
		return ret;

	ret = 0;
	_iris_pt_send_cmds(pcmd_set->cmds, pcmd_set->count, pcmd_set->state);
	return ret;
}

u8 iris_get_cmd_type(u8 cmd, u32 count)
{
	u8 dtype = 0;

	if (cmd < 0xB0) {
		if (count > 1)
			dtype = MIPI_DSI_DCS_LONG_WRITE;
		else if (count == 1)
			dtype = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		else
			dtype = MIPI_DSI_DCS_SHORT_WRITE;
	} else {
		if (count > 1)
			dtype = MIPI_DSI_GENERIC_LONG_WRITE;
		else if (count == 1)
			dtype = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM;
		else
			dtype = MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM;
	}
	return dtype;
}


void iris_cmd_desc_para_fill(struct iris_cmd_desc *dsi_cmd)
{
	u32 count = dsi_cmd->msg.tx_len;
	u8 cmd = *((u8 *)dsi_cmd->msg.tx_buf);

	dsi_cmd->post_wait_ms = 0;
	dsi_cmd->last_command = 1;

	if (IRIS_DSI_READ_CMD(dsi_cmd->msg.type)) {
		return;
	}

	dsi_cmd->msg.type = iris_get_cmd_type(cmd, count-1);
}

static void _iris_add_cmd_addr_val(struct iris_ocp_cmd *pcmd, u32 addr, u32 val)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(addr);
	*(u32 *)(pcmd->cmd + pcmd->cmd_len + 4) = cpu_to_le32(val);
	pcmd->cmd_len += 8;
}

static void _iris_add_cmd_payload(struct iris_ocp_cmd *pcmd, u32 payload)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(payload);
	pcmd->cmd_len += 4;
}

void iris_ocp_write_val(u32 address, u32 value)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0} };

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, 0xFFFFFFF0 | OCP_SINGLE_WRITE_BYTEMASK);
	_iris_add_cmd_addr_val(&ocp_cmd, address, value);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
	IRIS_LOGD("%s(), addr: 0x%08x, value: 0x%08x", __func__, address, value);

	_iris_pt_send_cmds(iris_ocp_cmd, 1, DSI_CMD_SET_STATE_HS);
}

void iris_ocp_write_vals(u32 header, u32 address, u32 size, u32 *pvalues)
{
	u32 i;
	u32 max_size = CMD_PKT_SIZE / 4 - 2;
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0} };


	while (size > 0) {
		memset(&ocp_cmd, 0, sizeof(ocp_cmd));

		_iris_add_cmd_payload(&ocp_cmd, header);
		_iris_add_cmd_payload(&ocp_cmd, address);
		if (size < max_size) {
			for (i = 0; i < size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			size = 0;
		} else {
			for (i = 0; i < max_size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			address += max_size * 4;
			pvalues += max_size;
			size -= max_size;
		}
		iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
		IRIS_LOGD("%s(), header: 0x%08x, addr: 0x%08x, len: %zu",
				__func__,
				header, address, iris_ocp_cmd[0].msg.tx_len);

		_iris_pt_send_cmds(iris_ocp_cmd,
				1, DSI_CMD_SET_STATE_HS);
	}
}

/*pvalues need to be one address and one value*/
static void _iris_dsi_write_mult_vals(u32 size, u32 *pvalues, int need_lock, struct cmdq_pkt *cmdq_handle)
{
	u32 i;
	/*need to remove one header length*/
#ifdef IRIS_I2C_ENABLE
	u32 max_size = 8; /*(244 -4)/4*/
#else
	u32 max_size = 48; /*(196 -4)/4*/
#endif
	u32 header = 0xFFFFFFF4;
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0} };

	if (size % 2 != 0) {
		IRIS_LOGE("%s(), need to be mult pair of address and value", __func__);
		return;
	}

	while (size > 0) {
		memset(&ocp_cmd, 0, sizeof(ocp_cmd));

		_iris_add_cmd_payload(&ocp_cmd, header);
		if (size < max_size) {
			for (i = 0; i < size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			size = 0;
		} else {
			for (i = 0; i < max_size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			pvalues += max_size;
			size -= max_size;
		}
		iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
		IRIS_LOGD("%s(), header: 0x%08x, len: %zu",
				__func__,
				header, iris_ocp_cmd[0].msg.tx_len);

		_iris_dsi_send_cmds(iris_ocp_cmd, 1, DSI_CMD_SET_STATE_HS, need_lock, cmdq_handle);
	}
}

static void _iris_i2c_write_mult_vals(u32 size, u32 *pvalues)
{
	int ret = 0;
	bool is_burst = 0;
	bool is_ulps_enable = 0;

	if (size % 2 != 0) {
		IRIS_LOGE("%s(), need to be mult pair of address and value", __func__);
		return;
	}

	is_ulps_enable = iris_disable_ulps(PATH_I2C);
	ret = iris_i2c_ocp_write(pvalues, size / 2, is_burst);
	iris_enable_ulps(PATH_I2C, is_ulps_enable);

	if (ret)
		IRIS_LOGE("%s(%d), i2c send fail, return: %d",
				__func__, __LINE__, ret);
}


/*pvalues need to be one address and one value*/
void _iris_ocp_write_mult_vals(u32 size, u32 *pvalues, int need_lock, struct cmdq_pkt *cmdq_handle)
{
	int path = iris_w_path_select;

	if (path == PATH_I2C) {
		IRIS_LOGD("%s(%d), path select i2c", __func__, __LINE__);
		_iris_i2c_write_mult_vals(size, pvalues);
	} else if (path == PATH_DSI) {
		IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
		_iris_dsi_write_mult_vals(size, pvalues, need_lock, cmdq_handle);
	} else {
		IRIS_LOGE("%s(%d), path not i2c or dsi, path = %d",
			__func__, __LINE__, path);
	}
}


void iris_ocp_write_mult_vals(u32 size, u32 *pvalues)
{
	int need_lock = 1;
	struct cmdq_pkt *pcmdq = NULL;

	_iris_ocp_write_mult_vals(size, pvalues, need_lock, pcmdq);
}

void iris_ocp_write_mult_vals_nonelock(u32 size, u32 *pvalues)
{
	int need_lock = 0;
	struct cmdq_pkt *pcmdq = NULL;

	_iris_ocp_write_mult_vals(size, pvalues, need_lock, pcmdq);
}

static void _iris_ocp_write_addr(u32 address, u32 mode)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0} };

	/* Send OCP command.*/
	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	_iris_add_cmd_payload(&ocp_cmd, OCP_SINGLE_READ);
	_iris_add_cmd_payload(&ocp_cmd, address);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	_iris_pt_send_cmds(iris_ocp_cmd, 1, mode);
}

int iris_send_cmdq_rdcmd(struct iris_cmd_set *pset)
{
	bool is_frame_mode;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct cmdq_pkt *cmdq_handle = NULL;
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;
	struct iris_cmd_desc  *cmd = pset->cmds;
	struct mtk_ddp_comp *comp = pcfg->mtk_comp;
	unsigned int slot_index = DISP_SLOT_IRIS_READ_BASE;

	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_len = cmd->msg.tx_len,
		.tx_buf = cmd->msg.tx_buf,
		.rx_len = cmd->msg.rx_len,
		.rx_buf = cmd->msg.rx_buf,
	};

	if (iris_create_cmdq_handle(&cmdq_handle, &is_frame_mode))
		return -EINVAL;
	//transfer rd cmd
	iris_get_mtk_dsi_op()->transfer_rdcmd(comp, cmdq_handle, &msg, slot_index);
	iris_destory_cmdq_handle(&cmdq_handle, is_frame_mode);

	iris_get_mtk_dsi_op()->obtain_rdvalue(comp, &msg, slot_index);

	return 0;
}

int iris_send_noncmdq_rdcmd(struct iris_cmd_set *pcmdset)
{
	struct iris_cmd_desc *cmd = pcmdset->cmds;
	struct iris_cfg *pcfg = iris_get_cfg();
	u8 *tx_buf = (u8 *)cmd->msg.tx_buf;
	u32 tx_len = cmd->msg.tx_len;
	u8 *rx_buf = (u8 *)cmd->msg.rx_buf;
	u32 rx_len = cmd->msg.rx_len;

	if (cmd->msg.type == MIPI_DSI_DCS_READ)
		mipi_dsi_dcs_read(pcfg->dsi_dev, tx_buf[0], rx_buf, rx_len);
	else
		mipi_dsi_generic_read(pcfg->dsi_dev, tx_buf, tx_len, rx_buf, rx_len);
	return 0;
}

int iris_send_rd_cmd(struct iris_cmd_set *pcmdset)
{
	int state;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_cmd_desc *cmd;

	if (!pcmdset || pcmdset->count > 1)
		return -EINVAL;

	cmd = pcmdset->cmds;
	state = pcmdset->state;

	if (state == DSI_CMD_SET_STATE_LP)
		pcfg->dsi_dev->mode_flags |= MIPI_DSI_MODE_LPM;
	else
		pcfg->dsi_dev->mode_flags &= ~(MIPI_DSI_MODE_LPM);

	if (pcfg->valid == PARAM_PARSED)
		iris_send_noncmdq_rdcmd(pcmdset);
	else if (pcfg->valid > PARAM_PARSED)
		iris_send_cmdq_rdcmd(pcmdset);

	return 0;
}

u32 _iris_ocp_read_value(u32 mode)
{
	u32 response_value = 0;
	char pi_read[1] = {0xC0};
	struct iris_cmd_desc pi_read_cmd[] = {
		{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			sizeof(pi_read), pi_read,
			sizeof(iris_read_cmd_rbuf),
			iris_read_cmd_rbuf}, 1, 0}};
	struct iris_cmd_set cmdset = {
		.cmds = pi_read_cmd,
		.state = mode,
		.count = 1,
	};

	/* Read response.*/
	memset(iris_read_cmd_rbuf, 0, sizeof(iris_read_cmd_rbuf));

	iris_send_rd_cmd(&cmdset);
	IRIS_LOGD("read register %02x %02x %02x %02x",
			iris_read_cmd_rbuf[0], iris_read_cmd_rbuf[1],
			iris_read_cmd_rbuf[2], iris_read_cmd_rbuf[3]);

	response_value = iris_read_cmd_rbuf[0] | (iris_read_cmd_rbuf[1] << 8) |
		(iris_read_cmd_rbuf[2] << 16) | (iris_read_cmd_rbuf[3] << 24);

	return response_value;
}

static u32 _iris_i2c_single_read(u32 address)
{
	u32 arr[2] = {0};
	bool is_burst = 0;
	int ret = 0;
	//bool is_ulps_enable = 0;

	arr[0] = address;
	//is_ulps_enable = iris_disable_ulps(PATH_I2C);
	//ret = iris_i2c_ocp_read(arr, 1, is_burst);
	ret = iris_i2c_conver_ocp_read(arr, 1, is_burst);
	//iris_enable_ulps(PATH_I2C, is_ulps_enable);
	if (ret) {
		IRIS_LOGE("%s(%d), i2c ocp single read fail, return: %d",
				__func__, __LINE__, ret);
		return 0;
	}
	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, arr[0]);

	return arr[0];
}

static u32 _iris_dsi_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;

	_iris_ocp_write_addr(address, mode);

	value = _iris_ocp_read_value(mode);
	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, value);

	return value;
}

u32 iris_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;
	int path = iris_r_path_select;

	if (path == PATH_I2C) {
		IRIS_LOGD("%s(%d), path select i2c", __func__, __LINE__);
		value = _iris_i2c_single_read(address);
	} else if (path == PATH_DSI) {
		IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
		value = _iris_dsi_ocp_read(address, mode);
	} else {
		IRIS_LOGE("%s(%d), path not i2c or dsi, path = %d",
				__func__, __LINE__, path);
	}

	return value;
}

static void _iris_dump_packet(u8 *data, int size)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4, data, size, false);
}

void iris_write_test(u32 iris_addr,
		int ocp_type, u32 pkt_size)
{
	union iris_ocp_cmd_header ocp_header;
	struct iris_cmd_desc iris_cmd = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0};

	u32 test_value = 0xFFFF0000;

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	memcpy(ocp_cmd.cmd, &ocp_header.header32, OCP_HEADER);
	ocp_cmd.cmd_len = OCP_HEADER;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:
		for (; ocp_cmd.cmd_len <= (pkt_size - 8); ) {
			_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
			test_value++;
		}
		break;
	case OCP_BURST_WRITE:
		test_value = 0xFFFF0000;
		_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
		if (pkt_size <= ocp_cmd.cmd_len)
			break;
		test_value++;
		for (; ocp_cmd.cmd_len <= pkt_size - 4;) {
			_iris_add_cmd_payload(&ocp_cmd, test_value);
			test_value++;
		}
		break;
	default:
		break;
	}

	IRIS_LOGI("%s(), len: %d, iris addr: %#x, test value: %#x",
			__func__,
			ocp_cmd.cmd_len, iris_addr, test_value);
	iris_cmd.msg.tx_len = ocp_cmd.cmd_len;

	_iris_pt_send_cmds(&iris_cmd, 1, DSI_CMD_SET_STATE_HS);

	if (IRIS_IF_LOGD())
		_iris_dump_packet(ocp_cmd.cmd, ocp_cmd.cmd_len);
}

void iris_write_test_muti_pkt(struct iris_ocp_dsi_tool_input *ocp_input)
{
	union iris_ocp_cmd_header ocp_header;
	u32 test_value = 0xFF000000;
	int cnt = 0;

	u32 iris_addr, ocp_type, pkt_size, total_cnt;

	ocp_type = ocp_input->iris_ocp_type;
	test_value = ocp_input->iris_ocp_value;
	iris_addr = ocp_input->iris_ocp_addr;
	total_cnt = ocp_input->iris_ocp_cnt;
	pkt_size = ocp_input->iris_ocp_size;

	memset(iris_test_cmd, 0, sizeof(iris_test_cmd));
	memset(ocp_test_cmd, 0, sizeof(ocp_test_cmd));

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:
		for (cnt = 0; cnt < total_cnt; cnt++) {
			memcpy(ocp_test_cmd[cnt].cmd,
					&ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;

			test_value = 0xFF000000 | (cnt << 16);
			while (ocp_test_cmd[cnt].cmd_len <= (pkt_size - 8)) {
				_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
						(iris_addr + cnt * 4), test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;
		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;
	case OCP_BURST_WRITE:
		for (cnt = 0; cnt < total_cnt; cnt++) {
			memcpy(ocp_test_cmd[cnt].cmd,
					&ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;
			test_value = 0xFF000000 | (cnt << 16);

			_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
					(iris_addr + cnt * 4), test_value);
			/* if(pkt_size <= ocp_test_cmd[cnt].cmd_len)
			 * break;
			 */
			test_value++;
			while (ocp_test_cmd[cnt].cmd_len <= pkt_size - 4) {
				_iris_add_cmd_payload(&ocp_test_cmd[cnt], test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;
		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;
	default:
		break;
	}

	IRIS_LOGI("%s(), total count: %#x, iris addr: %#x, test value: %#x",
			__func__, total_cnt, iris_addr, test_value);
	_iris_pt_send_cmds(iris_test_cmd, total_cnt, DSI_CMD_SET_STATE_HS);

	if (IRIS_IF_NOT_LOGV())
		return;
	for (cnt = 0; cnt < total_cnt; cnt++)
		_iris_dump_packet(ocp_test_cmd[cnt].cmd,
				ocp_test_cmd[cnt].cmd_len);
}

int iris_dsi_send_cmds(
		struct iris_cmd_desc *cmds,
		u32 count,
		enum dsi_cmd_set_state state)
{
	int need_lock = 1;
	struct cmdq_pkt * cmdq_handle  = NULL;

	_iris_dsi_send_cmds(cmds, count, state, need_lock, cmdq_handle);

	return 0;
}

int  iris_dsi_send_cmds_lockcmdq(
		struct iris_cmd_desc *cmds, u32 count,
		enum dsi_cmd_set_state state, int need_lock, struct cmdq_pkt *cmdq_handle)
{
	_iris_dsi_send_cmds(cmds, count, state, need_lock, cmdq_handle);
	return 0;
}

static u32 _iris_pt_get_split_pkt_cnt(int dlen)
{
	u32 sum = 1;

	if (dlen > IRIS_TX_HV_PAYLOAD_LEN)
		sum = (dlen - IRIS_TX_HV_PAYLOAD_LEN
				+ IRIS_TX_PAYLOAD_LEN - 1) / IRIS_TX_PAYLOAD_LEN + 1;
	return sum;
}

/*
 * @Description: use to do statitics for cmds which should not less than 252
 *      if the payload is out of 252, it will change to more than one cmds
 * the first payload need to be
 *	4 (ocp_header) + 8 (tx_addr_header + tx_val_header)
 *	+ 2* payload_len (TX_payloadaddr + payload_len)<= 252
 * the sequence payloader need to be
 *	4 (ocp_header) + 2* payload_len (TX_payloadaddr + payload_len)<= 252
 *	so the first payload should be no more than 120
 *	the second and sequence need to be no more than 124
 *
 * @Param: cmdset  cmds request
 * @return: the cmds number need to split
 **/
static u32 _iris_pt_calc_cmd_cnt(struct iris_cmd_set *cmdset)
{
	u32 i = 0;
	u32 sum = 0;
	u32 dlen = 0;

	for (i = 0; i < cmdset->count; i++) {
		dlen = cmdset->cmds[i].msg.tx_len;
		sum += _iris_pt_get_split_pkt_cnt(dlen);
	}
	return sum;
}

static int _iris_pt_alloc_cmds(
		struct iris_cmd_set *cmdset,
		struct iris_cmd_desc **ptx_cmds,
		struct iris_ocp_cmd **pocp_cmds)
{
	int cmds_cnt = _iris_pt_calc_cmd_cnt(cmdset);

	IRIS_LOGD("%s(%d), cmds cnt: %d malloc len: %lu",
			__func__, __LINE__,
			cmds_cnt, cmds_cnt * sizeof(**ptx_cmds));
	*ptx_cmds = vmalloc(cmds_cnt * sizeof(**ptx_cmds));
	if (!(*ptx_cmds)) {
		IRIS_LOGE("%s(), failed to malloc buf, len: %lu",
				__func__,
				cmds_cnt * sizeof(**ptx_cmds));
		return -ENOMEM;
	}

	*pocp_cmds = vmalloc(cmds_cnt * sizeof(**pocp_cmds));
	if (!(*pocp_cmds)) {
		IRIS_LOGE("%s(), failed to malloc buf for pocp cmds", __func__);
		vfree(*ptx_cmds);
		*ptx_cmds = NULL;
		return -ENOMEM;
	}
	return cmds_cnt;
}


static void _iris_pt_init_tx_cmd_hdr(
		struct iris_cmd_set *cmdset, struct iris_cmd_desc *dsi_cmd,
		union iris_mipi_tx_cmd_header *header)
{
	u8 dtype = dsi_cmd->msg.type;

	memset(header, 0x00, sizeof(*header));
	header->stHdr.dtype = dtype;
	header->stHdr.linkState = (cmdset->state == DSI_CMD_SET_STATE_LP) ? 1 : 0;
}

static void _iris_pt_set_cmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd, bool is_write)
{
	u32 dlen = 0;
	u8 *ptr = NULL;

	if (!dsi_cmd)
		return;

	dlen = dsi_cmd->msg.tx_len;

	if (is_write)
		pheader->stHdr.writeFlag = 0x01;
	else
		pheader->stHdr.writeFlag = 0x00;

	if (pheader->stHdr.longCmdFlag == 0) {
		ptr = (u8 *)dsi_cmd->msg.tx_buf;
		if (dlen == 1) {
			pheader->stHdr.len[0] = ptr[0];
		} else if (dlen == 2) {
			pheader->stHdr.len[0] = ptr[0];
			pheader->stHdr.len[1] = ptr[1];
		}
	} else {
		pheader->stHdr.len[0] = dlen & 0xff;
		pheader->stHdr.len[1] = (dlen >> 8) & 0xff;
	}
}

static void _iris_pt_set_wrcmd_hdr(union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, true);
}

static void _iris_pt_set_rdcmd_hdr(
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, false);
}

static void _iris_pt_init_ocp_cmd(struct iris_ocp_cmd *pocp_cmd)
{
	union iris_ocp_cmd_header ocp_header;

	if (!pocp_cmd) {
		IRIS_LOGE("%s(), invalid pocp cmd!", __func__);
		return;
	}

	memset(pocp_cmd, 0x00, sizeof(*pocp_cmd));
	ocp_header.header32 = 0xfffffff0 | OCP_SINGLE_WRITE_BYTEMASK;
	memcpy(pocp_cmd->cmd, &ocp_header.header32, OCP_HEADER);
	pocp_cmd->cmd_len = OCP_HEADER;
}

static void _iris_add_tx_cmds(
		struct iris_cmd_desc *ptx_cmd,
		struct iris_ocp_cmd *pocp_cmd, u8 wait)
{
	struct iris_cmd_desc desc_init_val = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			CMD_PKT_SIZE, NULL, 0, NULL}, 1, 0};

	memcpy(ptx_cmd, &desc_init_val, sizeof(struct iris_cmd_desc));
	ptx_cmd->msg.tx_buf = pocp_cmd->cmd;
	ptx_cmd->msg.tx_len = pocp_cmd->cmd_len;
	ptx_cmd->post_wait_ms = wait;
}

static u32 _iris_pt_short_write(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	pheader->stHdr.longCmdFlag = 0x00;

	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, IRIS_MIPI_TX_HEADER_ADDR, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_short_read(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		struct iris_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	pheader->stHdr.longCmdFlag= 0x00;
	_iris_pt_set_rdcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, IRIS_MIPI_TX_HEADER_ADDR, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_get_split_pkt_len(u16 dlen, int sum, int k)
{
	u16 split_len = 0;

	if (k == 0)
		split_len = dlen <  IRIS_TX_HV_PAYLOAD_LEN
			? dlen : IRIS_TX_HV_PAYLOAD_LEN;
	else if (k == sum - 1)
		split_len = dlen - IRIS_TX_HV_PAYLOAD_LEN
			- (k - 1) * IRIS_TX_PAYLOAD_LEN;
	else
		split_len = IRIS_TX_PAYLOAD_LEN;

	return split_len;
}

static void _iris_pt_add_split_pkt_payload(
		struct iris_ocp_cmd *pocp_cmd, u8 *ptr, u16 split_len)
{
	u32 i = 0;
	union iris_mipi_tx_cmd_payload payload;

	memset(&payload, 0x00, sizeof(payload));
	for (i = 0; i < split_len; i += 4, ptr += 4) {
		if (i + 4 > split_len) {
			payload.pld32 = 0;
			memcpy(payload.p, ptr, split_len - i);
		} else
			payload.pld32 = *(u32 *)ptr;

		IRIS_LOGD("%s(), payload: %#x", __func__, payload.pld32);
		_iris_add_cmd_addr_val(pocp_cmd, IRIS_MIPI_TX_PAYLOAD_ADDR, payload.pld32);
	}
}

static u32 _iris_pt_long_write(
		struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader,
		 struct iris_cmd_desc *dsi_cmd)
{
	u8 * ptr = NULL;
	u32 i = 0;
	u32 sum = 0;
	u16 dlen = 0;
	u32 split_len = 0;

	dlen = dsi_cmd->msg.tx_len;

	pheader->stHdr.longCmdFlag = 0x1;
	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: %#x",
			__func__, __LINE__,
			pheader->hdr32);
	_iris_add_cmd_addr_val(pocp_cmd, IRIS_MIPI_TX_HEADER_ADDR,
			pheader->hdr32);

	ptr = (u8 *)dsi_cmd->msg.tx_buf;
	sum = _iris_pt_get_split_pkt_cnt(dlen);

	while (i < sum) {
		ptr += split_len;
		split_len = _iris_pt_get_split_pkt_len(dlen, sum, i);
		_iris_pt_add_split_pkt_payload(pocp_cmd + i, ptr, split_len);

		i++;
		if (i < sum)
			_iris_pt_init_ocp_cmd(pocp_cmd + i);
	}
	return sum;
}

static u32 _iris_pt_add_cmd(
		struct iris_cmd_desc *ptx_cmd, struct iris_ocp_cmd *pocp_cmd,
		struct iris_cmd_desc *dsi_cmd, struct iris_cmd_set *cmdset)
{
	u32 i = 0;
	u16 dtype = 0;
	u32 sum = 0;
	u8 wait = 0;
	union iris_mipi_tx_cmd_header header;

	_iris_pt_init_tx_cmd_hdr(cmdset, dsi_cmd, &header);

	dtype = dsi_cmd->msg.type;
	switch (dtype) {
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		sum = _iris_pt_short_read(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
		//case MIPI_DSI_DCS_COMPRESSION_MODE:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		sum = _iris_pt_short_write(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
		//case MIPI_DSI_PPS_LONG_WRITE:
		sum = _iris_pt_long_write(pocp_cmd, &header, dsi_cmd);
		break;
	default:
		IRIS_LOGE("%s(), invalid type: %#x",
				__func__,
				dsi_cmd->msg.type);
		break;
	}

	for (i = 0; i < sum; i++) {
		wait = (i == sum - 1) ? dsi_cmd->post_wait_ms : 0;
		_iris_add_tx_cmds(ptx_cmd + i, pocp_cmd + i, wait);
	}
	return sum;
}

void _iris_pt_send_cmds(struct iris_cmd_desc *ptx_cmds, u32 cmds_cnt, enum dsi_cmd_set_state state)
{
	int need_lock = 1;
	struct cmdq_pkt *cmdq_handle = NULL;

	_iris_dsi_send_cmds(ptx_cmds, cmds_cnt, state, need_lock, cmdq_handle);
}

static int __iris_pt_write_panel_cmd(struct iris_cmd_set *cmdset,
		struct iris_ocp_cmd **ret_pocp_cmds,
		struct iris_cmd_desc **ret_ptx_cmds)
{
	u32 i = 0;
	u32 j = 0;
	int ret = 0;
	int cmds_cnt = 0;
	u32 offset = 0;

	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct iris_cmd_desc *ptx_cmds = NULL;
	struct iris_cmd_desc *dsi_cmds = NULL;

	if (cmdset->count == 0) {
		IRIS_LOGD("%s(), invalid cmdset count!", __func__);
		return -EINVAL;
	}
	cmds_cnt = _iris_pt_alloc_cmds(cmdset, &ptx_cmds, &pocp_cmds);
	if (cmds_cnt < 0) {
		IRIS_LOGE("%s(), invalide cmds count: %d", __func__, cmds_cnt);
		return -EINVAL;
	}

	for (i = 0; i < cmdset->count; i++) {
		/*initial val*/
		dsi_cmds = cmdset->cmds + i;
		iris_cmd_desc_para_fill(dsi_cmds);
		_iris_pt_init_ocp_cmd(pocp_cmds + j);
		offset = _iris_pt_add_cmd(
				ptx_cmds + j, pocp_cmds + j, dsi_cmds, cmdset);
		j += offset;
	}

	if (j != (u32)cmds_cnt) {
		IRIS_LOGE("%s(), invalid cmd count: %d, j: %d",
				__func__,
				cmds_cnt, j);
		ret = -ENOMEM;
	} else
		ret = cmds_cnt;

	*ret_pocp_cmds = pocp_cmds;
	*ret_ptx_cmds = ptx_cmds;

	return ret;
}

int iris_conver_one_panel_cmd(u8 *dest, u8 *src, int len)
{
	int ret = 0;
	struct iris_cmd_set cmdset;
	struct iris_cmd_desc desc;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct iris_cmd_desc *ptx_cmds = NULL;

	if (!src || !dest) {
		IRIS_LOGE("src or dest is error");
		return -EINVAL;
	}

	//backlight operation
	if (src[0] == MIPI_DCS_SET_DISPLAY_BRIGHTNESS)
		ret = iris_check_hdr_backlight(src+1, len-1);

	//abypass mode
	if (!iris_is_pt_mode() || ret) {
		memcpy(dest, src, len);
		ret = len;
		return ret;
	}

	cmdset.cmds = &desc;
	cmdset.count = 1;
	desc.msg.tx_buf = src;
	desc.msg.tx_len = len;

	ret = __iris_pt_write_panel_cmd(&cmdset, &pocp_cmds, &ptx_cmds);
	if (ret == -EINVAL)
		return ret;
	else if (ret == 1) {
		memcpy(dest, ptx_cmds->msg.tx_buf, ptx_cmds->msg.tx_len);
		ret = ptx_cmds->msg.tx_len;
	} else
		ret = -EINVAL;

	kvfree(pocp_cmds);
	kvfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;

	return ret;
}
EXPORT_SYMBOL(iris_conver_one_panel_cmd);

static void _iris_pt_write_panel_cmd(struct iris_cmd_set *cmdset)
{
	int ret = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct iris_cmd_desc *ptx_cmds = NULL;

	if (!cmdset || !cmdset->count) {
		IRIS_LOGE("cmdset is error!");
		return;
	}

	ret = __iris_pt_write_panel_cmd(cmdset, &pocp_cmds, &ptx_cmds);
	if (ret == -EINVAL)
		return;
	else if (ret > 0)
		iris_send_noncmdq_cmds(ptx_cmds, ret, DSI_CMD_SET_STATE_HS);

	kvfree(pocp_cmds);
	kvfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;
}


static void _iris_pt_switch_cmd(
		struct iris_cmd_set *cmdset,
		struct iris_cmd_desc *dsi_cmd)
{
	if (!cmdset || !dsi_cmd) {
		IRIS_LOGE("%s there have null pointer", __func__);
		return;
	}

	cmdset->cmds = dsi_cmd;
	cmdset->count = 1;
}

static int _iris_pt_write_max_pkt_size(
		struct iris_cmd_set *cmdset)
{
	u32 rlen = 0;
	struct iris_cmd_set local_cmdset;
	static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */
	static struct iris_cmd_desc pkt_size_cmd = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			sizeof(max_pktsize), max_pktsize, 0, NULL}, 1, 0};

	rlen = cmdset->cmds[0].msg.rx_len;
	if (rlen > 128) {
		IRIS_LOGE("%s(), invalid len: %d", __func__, rlen);
		return -EINVAL;
	}

	max_pktsize[0] = rlen > 10 ? 10 : rlen;
	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(&local_cmdset, &pkt_size_cmd);
	_iris_pt_write_panel_cmd(&local_cmdset);

	return 0;
}

static void _iris_pt_send_panel_rdcmd(
		struct iris_cmd_set *cmdset)
{
	struct iris_cmd_set local_cmdset;
	struct iris_cmd_desc *dsi_cmd = cmdset->cmds;

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(&local_cmdset, dsi_cmd);

	/*passthrough write to panel*/
	_iris_pt_write_panel_cmd(&local_cmdset);
}

static int _iris_pt_remove_respond_hdr(char *ptr, int *offset)
{
	int rc = 0;
	char cmd;

	if (!ptr)
		return -EINVAL;

	cmd = ptr[0];
	IRIS_LOGV("%s(), cmd: 0x%02x", __func__, cmd);
	switch (cmd) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		IRIS_LOGD("%s(), rx ACK_ERR_REPORT", __func__);
		rc = -EINVAL;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		*offset = 1;
		rc = 1;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		*offset = 1;
		rc = 2;
		break;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		*offset = 4;
		rc = ptr[1];
		break;
	default:
		rc = 0;
	}

	return rc;
}

static int _iris_pt_read(struct iris_cmd_set *cmdset)
{
	u32 i = 0;
	u32 rlen = 0;
	u32 offset = 0;
	union iris_mipi_tx_cmd_payload val;
	u8 *rbuf = NULL;
	u32 address = IRIS_RD_PACKET_DATA;
	int ret = 1;

	rbuf = (u8 *)cmdset->cmds[0].msg.rx_buf;
	rlen = cmdset->cmds[0].msg.rx_len;

	if (!rbuf || rlen <= 0) {
		IRIS_LOGE("rbuf %p rlen =%d", rbuf, rlen);
		ret = -1;
		return ret;
	}

	//read iris for data
#ifdef IRIS_I2C_ENABLE
	val.pld32 = iris_i2c_read(&address, 1, false);
#else
	val.pld32 = iris_ocp_read(address, cmdset->state);
#endif
	rlen = _iris_pt_remove_respond_hdr(val.p, &offset);
	IRIS_LOGV("%s(), read len: %d", __func__, rlen);

	if (rlen <= 0) {
		IRIS_LOGE("%s(), do not return value", __func__);
		ret = -2;
		return ret;
	}

	if (rlen <= 2) {
		for (i = 0; i < rlen; i++)
			rbuf[i] = val.p[offset + i];
	} else {
		int j = 0;
		int len = 0;
		int num = (rlen + 3) / 4;

		for (i = 0; i < num; i++) {
			len = (i == num -1) ? rlen - 4 * i : 4;
#ifdef IRIS_I2C_ENABLE
			val.pld32= iris_i2c_reg_read(IRIS_RD_PACKET_DATA);
#else
			val.pld32 = iris_ocp_read(address, DSI_CMD_SET_STATE_HS);
#endif
			for (j = 0; j < len; j++)
				rbuf[i * 4 + j] = val.p[j];
		}
	}

	return ret;
}

static u32 _iris_get_panel_frame_ms(void)
{
	u32 frame = iris_get_vrefresh();
	if ((frame < 24) || (frame > 240))
		frame = 24;

	frame = ((1000 / frame) + 1);

	return frame;
}

int iris_pt_read_panel_cmd(struct iris_cmd_set *cmdset)
{
//	struct iris_cfg *pcfg = iris_get_cfg();
	u32 ms = _iris_get_panel_frame_ms();
	int ret = 1;

	IRIS_LOGD("%s(), enter", __func__);
	if (!cmdset || cmdset->count != 1) {
		IRIS_LOGE("%s(), invalid input, cmdset: %p", __func__, cmdset);
		ret = -1;
		return ret;
	}

	/*step1  write max packket size*/
	_iris_pt_write_max_pkt_size(cmdset);

	/*step2 write read cmd to panel*/
	_iris_pt_send_panel_rdcmd(cmdset);

	// step 3: delay
	usleep_range(1000*ms, 1000*ms + 1);

	/*step4 read panel data*/
	ret = _iris_pt_read(cmdset);

	return ret;
}

int iris_pt_send_panel_cmd(
	struct iris_cmd_set *cmdset)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int ret = 1;
	int retry = 3;

	if (!cmdset || cmdset->count == 0) {
		IRIS_LOGE("cmdset = %p", cmdset);
		ret = -1;
		return ret;
	}
	if (!iris_is_pt_mode()) {
		IRIS_LOGD("[%s](%d) is not in pt mode.", __func__, __LINE__);
		ret = -2;
		return ret;
	}

	mutex_lock(&pcfg->mutex);
	if (cmdset->count == 1 && cmdset->cmds[0].msg.type == MIPI_DSI_DCS_READ) {
		do { 
			ret = iris_pt_read_panel_cmd(cmdset);
			IRIS_LOGV("[%s](%d) retry = %d.", __func__, __LINE__, retry);
		} while ((ret <= 0) && (--retry));
	}
	else {
		_iris_pt_write_panel_cmd(cmdset);
	}
	mutex_unlock(&pcfg->mutex);

	return ret;
}


void iris_panel_cmd_passthrough(unsigned int cmd, unsigned char count, unsigned char *para_list,
	u8 *buffer, u8 buffer_size, unsigned char rd_cmd)
{
	u8 buf[512] = {0,};
	u32 sum = 1;
	struct iris_cmd_set cmdset;
	struct iris_cmd_desc desc[1];

	memset(&cmdset, 0x00, sizeof(cmdset));
	cmdset.cmds = desc;

	buf[0] = cmd;
	if (!rd_cmd) {
		memcpy(buf+1, para_list, count);
		sum = count+1;
	} else {
		cmdset.cmds[0].msg.rx_buf = buffer;
		cmdset.cmds[0].msg.rx_len = buffer_size;
	}

	cmdset.cmds[0].msg.tx_buf = buf;
	cmdset.cmds[0].msg.tx_len = sum;
	cmdset.count = 1;
	cmdset.state = DSI_CMD_SET_STATE_LP;

	if (iris_is_abyp_mode())
		iris_send_cmdset(&cmdset);
	else
		iris_pt_send_panel_cmd(&cmdset);
}
EXPORT_SYMBOL(iris_panel_cmd_passthrough);

void iris_set_pwil_mode(u8 mode, bool osd_enable, int state)
{
	char pwil_mode[2] = {0x00, 0x00};
	struct iris_cfg *pcfg = iris_get_cfg();

	if (mode == PT_MODE) {
		pwil_mode[0] = 0x0;
		pwil_mode[1] = 0x81;
	} else if (mode == RFB_MODE) {
		pwil_mode[0] = 0xc;
		pwil_mode[1] = 0x81;
	} else if (mode == FRC_MODE) {
		pwil_mode[0] = 0x4;
		pwil_mode[1] = 0x82;
	}
	if (osd_enable)
		pwil_mode[0] |= 0x80;

	if (pcfg->tx_mode && iris_check_dsc_enable())
		pwil_mode[0] |= 0x10;

	IRIS_LOGI("%s(), set pwil mode: %x, %x", __func__, pwil_mode[0], pwil_mode[1]);

	iris_ocp_write_val(0xf1800080, (pwil_mode[1] << 8) + pwil_mode[0]);
}
#if 0
static int _iris_ctrl_ocp_read_value(struct dsi_display_ctrl *ctrl,
		u32 mode, u32 *val)
{
	char pi_read[1] = {0x00};
	struct dsi_cmd_desc pi_read_cmd[] = {
		{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_REQ_ACK,
			sizeof(pi_read), pi_read, 0, NULL}, 1, 0} };
	u32 response_value;
	u32 flags = 0;
	int rc = 0;

	/* Read response.*/
	memset(iris_read_cmd_rbuf, 0, sizeof(iris_read_cmd_rbuf));
	pi_read_cmd[0].msg.rx_len = 4;
	pi_read_cmd[0].msg.rx_buf = iris_read_cmd_rbuf;

	flags |= (DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ |
			DSI_CTRL_CMD_CUSTOM_DMA_SCHED);
	flags |= DSI_CTRL_CMD_LAST_COMMAND;
	pi_read_cmd->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
	if (mode == DSI_CMD_SET_STATE_LP)
		pi_read_cmd->msg.flags |= MIPI_DSI_MSG_USE_LPM;
	rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &pi_read_cmd->msg, &flags);
	if (rc <= 0) {
		IRIS_LOGE("%s(), rx cmd transfer failed, return: %d", __func__, rc);
		return rc;
	}
	IRIS_LOGD("read register %02x %02x %02x %02x",
			iris_read_cmd_rbuf[0], iris_read_cmd_rbuf[1],
			iris_read_cmd_rbuf[2], iris_read_cmd_rbuf[3]);
	response_value = iris_read_cmd_rbuf[0] | (iris_read_cmd_rbuf[1] << 8) |
		(iris_read_cmd_rbuf[2] << 16) | (iris_read_cmd_rbuf[3] << 24);
	*val = response_value;

	return 4;
}

static void _iris_ctrl_ocp_write_addr(struct dsi_display_ctrl *ctrl,
		u32 address,
		u32 mode)
{
	struct iris_ocp_cmd ocp_cmd;
	struct dsi_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL}, 1, 0} };
	u32 flags = 0;

	/* Send OCP command.*/
	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	_iris_add_cmd_payload(&ocp_cmd, OCP_SINGLE_READ);
	_iris_add_cmd_payload(&ocp_cmd, address);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	flags |= DSI_CTRL_CMD_FETCH_MEMORY;
	flags |= DSI_CTRL_CMD_LAST_COMMAND;
	iris_ocp_cmd->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
	iris_pt_send_cmds(ctrl->ctrl, &iris_ocp_cmd->msg, &flags);
}

static int _iris_ctrl_ocp_read(struct dsi_display_ctrl *ctrl, u32 address,
		u32 mode, u32 *pval)
{
	int rc = 0;

	_iris_ctrl_ocp_write_addr(ctrl, address, DSI_CMD_SET_STATE_HS);
	rc = _iris_ctrl_ocp_read_value(ctrl, mode, pval);
	if (rc != 4)
		return -EINVAL;

	return 0;
}

static int _iris_panel_ctrl_send(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel,
		struct dsi_cmd_desc *ptx_cmds, u32 cmds_cnt)
{
	u32 flags = 0;
	int i;
	int rc = 0;

	flags |= DSI_CTRL_CMD_FETCH_MEMORY;
	for (i = 0; i < cmds_cnt; i++) {
		if (ptx_cmds[i].last_command) {
			ptx_cmds[i].msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
			flags |= DSI_CTRL_CMD_LAST_COMMAND;
		}
		rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &ptx_cmds[i].msg, &flags);
		if (ptx_cmds[i].post_wait_ms) {
			usleep_range(ptx_cmds[i].post_wait_ms * 1000,
					((ptx_cmds[i].post_wait_ms * 1000) + 10));
			IRIS_LOGV("%s(%d), wait: %d ms", __func__, __LINE__,
					ptx_cmds[i].post_wait_ms);
		}
	}
	return rc;
}

static int _iris_panel_ctrl_wr(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel,
		struct iris_cmd_set *cmdset)
{
	u32 i = 0;
	u32 j = 0;
	int cmds_cnt = 0;
	u32 offset = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct dsi_cmd_desc *ptx_cmds = NULL;
	struct dsi_cmd_desc *dsi_cmds = NULL;
	int rc = 0;

	if (!panel || !cmdset) {
		IRIS_LOGE("%s(), invalid input!", __func__);
		return -EINVAL;
	}

	cmds_cnt = _iris_pt_alloc_cmds(cmdset, &ptx_cmds, &pocp_cmds);
	IRIS_LOGV("%s(%d), cmdset.cnt: %d cmds_cnt: %d", __func__, __LINE__,
			cmdset->count, cmds_cnt);
	if (cmds_cnt < 0) {
		IRIS_LOGE("%s(), invalid cmds cnt: %d", __func__, cmds_cnt);
		return -ENOMEM;
	}

	for (i = 0; i < cmdset->count; i++) {
		/*initial val*/
		dsi_cmds = cmdset->cmds + i;
		_iris_pt_init_ocp_cmd(pocp_cmds + j);
		offset = _iris_pt_add_cmd(
				ptx_cmds + j, pocp_cmds + j, dsi_cmds, cmdset);
		j += offset;
	}

	if (j != (u32)cmds_cnt) {
		IRIS_LOGE("%s(), invalid cmds count: %d, j: %d",
				__func__,
				cmds_cnt, j);
	} else {
		rc = _iris_panel_ctrl_send(ctrl, panel, ptx_cmds, (u32)cmds_cnt);
	}

	vfree(pocp_cmds);
	vfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;

	return rc;
}

static int _iris_panel_ctrl_pt_read_data(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u32 rlen = 0;
	u32 offset = 0;
	union iris_mipi_tx_cmd_payload val;
	u8 *rbuf = NULL;
	u32 address = IRIS_RD_PACKET_DATA;
	int rc = 0;

	rbuf = (u8 *)cmdset->cmds[0].msg.rx_buf;
	rlen = cmdset->cmds[0].msg.rx_len;

	if (!rbuf || rlen <= 0) {
		IRIS_LOGE("%s(), rbuf: %p, rlen: %d", __func__, rbuf, rlen);
		return -EINVAL;
	}

	/* read iris for data */
	rc = _iris_ctrl_ocp_read(ctrl, address, cmdset->state, &val.pld32);
	if (rc) {
		IRIS_LOGE("%s(), do not return value", __func__);
		return -EINVAL;
	}

	IRIS_LOGD("%s(%d), rbuf: %p, rlen: %d, pld32: 0x%08x",
			__func__, __LINE__, rbuf, rlen, val.pld32);
	rlen = _iris_pt_remove_respond_hdr(val.p, &offset);

	if (rlen <= 0) {
		IRIS_LOGE("%s(), invalid len: %d", __func__, rlen);
		return -EINVAL;
	}

	if (rlen <= 2) {
		for (i = 0; i < rlen; i++) {
			rbuf[i] = val.p[offset + i];
			IRIS_LOGV("%s(%d), rlen: %d, d: 0x%02x",
					__func__, __LINE__, rlen, rbuf[i]);
		}
	} else {
		int j = 0;
		int len = 0;
		int num = (rlen + 3) / 4;

		for (i = 0; i < num; i++) {
			len = (i == num - 1) ? rlen - 4 * i : 4;

			rc = _iris_ctrl_ocp_read(ctrl, address, cmdset->state, &val.pld32);
			if (rc) {
				IRIS_LOGE("%s(), return: %d", __func__, rc);
				return -EINVAL;
			}
			for (j = 0; j < len; j++) {
				rbuf[i * 4 + j] = val.p[j];
				IRIS_LOGV("%s(%d), rlen: %d, d: 0x%02x",
						__func__, __LINE__, rlen, rbuf[i]);
			}
		}
	}

	return rlen;
}

static void _iris_panel_ctrl_write_rdcmd(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	struct dsi_panel_cmd_set local_cmdset;
	struct dsi_cmd_desc *dsi_cmd = NULL;

	IRIS_LOGD("%s(%d)", __func__, __LINE__);
	dsi_cmd = cmdset->cmds;
	memset(&local_cmdset, 0x00, sizeof(local_cmdset));
	_iris_pt_switch_cmd(&local_cmdset, dsi_cmd);
	_iris_panel_ctrl_wr(ctrl, panel, &local_cmdset);
}

static int _iris_panel_ctrl_set_max_pkt_size(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	int rc = 0;
	size_t rlen;
	struct dsi_panel_cmd_set local_cmdset;
	static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */
	static struct dsi_cmd_desc pkt_size_cmd = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK,
			sizeof(max_pktsize), max_pktsize, 0, NULL}, 1, 0};

	IRIS_LOGV("%s(%d)", __func__, __LINE__);
	rlen = cmdset->cmds[0].msg.rx_len;
	if (rlen > 128) {
		IRIS_LOGE("%s(), invalid len: %d", __func__, rlen);
		return -EINVAL;
	}
	IRIS_LOGD("%s(), len: %d", __func__, rlen);

	max_pktsize[0] = (rlen & 0xFF);

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));
	_iris_pt_switch_cmd(&local_cmdset, &pkt_size_cmd);
	local_cmdset.state = DSI_CMD_SET_STATE_HS;
	_iris_panel_ctrl_wr(ctrl, panel, &local_cmdset);

	return rc;
}

static int _iris_panel_ctrl_read(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 ms = _iris_get_panel_frame_ms();

	if (!pcfg)
		return rc;

	// step 1: max return size
	_iris_panel_ctrl_set_max_pkt_size(ctrl, panel, cmdset);
	// step 2: read command
	_iris_panel_ctrl_write_rdcmd(ctrl, panel, cmdset);
	// step 3: delay one frame
	usleep_range(1000 * ms, 1000 * ms + 1);
	// step 4: read response data
	rc = _iris_panel_ctrl_pt_read_data(ctrl, panel, cmdset);

	return rc;
}

static int _iris_panel_ctrl_read_status(struct dsi_display_ctrl *ctrl,
		struct dsi_panel *panel)
{
	int i, rc = 0, count = 0, start = 0, *lenp;
	struct drm_panel_esd_config *config;
	struct dsi_cmd_desc *cmds;

	config = &panel->esd_config;
	lenp = config->status_valid_params ?: config->status_cmds_rlen;
	count = config->status_cmd.count;
	cmds = config->status_cmd.cmds;

	for (i = 0; i < count; ++i) {
		struct dsi_panel_cmd_set local_cmdset;
		int retry = 3;

		memset(&local_cmdset, 0x00, sizeof(local_cmdset));
		memset(config->status_buf, 0x0, SZ_4K);
		cmds[i].msg.rx_buf = config->status_buf;
		cmds[i].msg.rx_len = config->status_cmds_rlen[i];
		local_cmdset.state = config->status_cmd.state;
		_iris_pt_switch_cmd(&local_cmdset, &cmds[i]);
		do {
			rc = _iris_panel_ctrl_read(ctrl, panel, &local_cmdset);
		} while ((rc <= 0) && (--retry));
		if (rc <= 0) {
			IRIS_LOGE("%s(), failed for panel ctrl read, return: %d", __func__, rc);
			return rc;
		}
		IRIS_LOGV("%s(%d), status[0]: 0x%02x len: 0x%02x",
				__func__, __LINE__,
				config->status_buf[0], lenp[i]);
		memcpy(config->return_buf + start,
				config->status_buf, lenp[i]);
		start += lenp[i];
	}

	return 1;
}

int iris_panel_ctrl_read_reg(struct dsi_display_ctrl *ctrl, struct dsi_panel *panel,
		u8 *rx_buf, int rlen, struct dsi_cmd_desc *cmd)
{
	int rc = 0;
	int retry = 3;
	struct dsi_panel_cmd_set local_cmdset;
	struct dsi_cmd_desc *cmds = cmd;

	if (ctrl == NULL || panel == NULL || rx_buf == NULL || cmds == NULL || rlen <= 0)
		return -EINVAL;

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));
	cmds->msg.rx_buf = rx_buf;
	cmds->msg.rx_len = rlen;
	cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;
	_iris_pt_switch_cmd(&local_cmdset, cmds);
	do {
		rc = _iris_panel_ctrl_read(ctrl, panel, &local_cmdset);
	} while ((rc <= 0) && (--retry));

	if (rc <= 0) {
		IRIS_LOGE("%s(), failed for panel ctrl read, return: %d",
				__func__, rc);
		return rc;
	}

	return 1;
}
#endif

void iris_esd_register_dump(void)
{
	u32 value = 0;
	u32 index = 0;
	u32 len = 0;
	static u32 iris_register_list[] = {
		0xf0000000,
		0xf0000004,
		0xf0000008,
		0xf000001c,
		0xf0000020,
		0xf0000024,
		0xf0000040,
		0xf0000044,
		0xf0000048,
		0xf000004c,
		0xf0000060,
		0xf0000094,
		0xf1800004,
		0xf1800034,
		0xf123ffe4,
		0xf155ffe4,
		0xf1240030,
		0xf125ffe4,
		0xf163ffe4,
		0xf165ffe4,
		0xf169ffe4,
		0xf16bffe4,
		0xf16dffe4,
	};

	IRIS_LOGE("iris esd register dump: ");
	len =  sizeof(iris_register_list) / sizeof(u32);
	for (index = 0; index < len; index++) {
		value = iris_ocp_read(iris_register_list[index],
				DSI_CMD_SET_STATE_HS);
		IRIS_LOGE("%08x : %08x", iris_register_list[index], value);
	}
}

int iris_panel_read_status(int i, unsigned char cmd)
{
	unsigned char get_esd_state[1] = {cmd};
	unsigned char read_cmd_rbuf[1] = {0x0};
	struct iris_cfg *pcfg = iris_get_cfg();
	int ret = 1;

	struct iris_cmd_desc cmds = {
		{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0,
		 0, sizeof(get_esd_state), get_esd_state,
		 sizeof(read_cmd_rbuf), read_cmd_rbuf}, 1, 0};

	struct iris_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_LP,
		.count = 1,
		.cmds = &cmds,
	};

	if ((0 <= i) && (i < ESD_CHK_NUM))
	{
		read_cmd_rbuf[0] = 0;
		ret = iris_pt_send_panel_cmd(&cmdset);

		if (iris_is_pt_mode())
			pcfg->esd_chk_val[i] = read_cmd_rbuf[0];
		else {
			IRIS_LOGD("[%s][%d] is not in pt mode.", __func__, i);
			pcfg->esd_chk_val[i] = ESD_CHK_EXIT;
			ret = 1;
		}

		IRIS_LOGD("[%s]esd_chk_val[%d]: 0x%02x", __func__, i, pcfg->esd_chk_val[i]);
	}

	return ret;
}

int iris_get_status(void)
{
	int rc = 1;
	unsigned int data = 0;
	unsigned int reg_update = 0x00;
	int cnt = 3;
	struct iris_cfg *pcfg;
	u32 ms = _iris_get_panel_frame_ms();

	pcfg = iris_get_cfg();

	data = iris_ocp_read(IRIS_REG_INTSTAT_RAW, DSI_CMD_SET_STATE_HS);
	if (pcfg->lp_ctrl.esd_trigger == 2)
		data = TXFALSE_CONTROL_MASK;
	if (data & TXFALSE_CONTROL_MASK) {
		IRIS_LOGE("INTSTAT_RAW: 0x%x", data);
		cnt = 0;
		rc = -2;
		goto status_check_done;
	}
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("INTSTAT_RAW: 0x%x", data);

	data = iris_ocp_read(IRIS_REG_UPDATE, DSI_CMD_SET_STATE_HS);
	iris_ocp_write_val(IRIS_REG_UPDATE, data | (1 << DISP_CMD_SHAWDOW_EN_SHIFT));
	do {
		data = iris_ocp_read(IRIS_REG_UPDATE, DSI_CMD_SET_STATE_HS);
		reg_update = data & DISP_CMD_SHAWDOW_EN_MASK;
		if (pcfg->lp_ctrl.esd_trigger == 3)
			reg_update = DISP_CMD_SHAWDOW_EN_MASK;
		if (!reg_update) {
			if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
				IRIS_LOGI("esd %d reg_update: 0x%x", cnt, data);
			break;
		}
		IRIS_LOGI("esd %d data: 0x%x reg_update: 0x%x", cnt, data, reg_update);
		usleep_range(1000 * ms, 1000 * ms + 1);
	} while (--cnt);
	if (cnt == 0)
		rc = -3;

status_check_done:
	if (cnt == 0) {
		pcfg->lp_ctrl.esd_cnt_iris++;
		IRIS_LOGI("iris esd err cnt: %d. rc %d", pcfg->lp_ctrl.esd_cnt_iris, rc);
		iris_esd_register_dump();
	} else {
		rc = 1;
	}
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("%s(), iris esd return: %d", __func__, rc);

	return rc;
}

int iris_read_status(int i, unsigned char cmd)
{
	struct iris_cfg *pcfg;
	int rc = 1;

	pcfg = iris_get_cfg();

	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("%s() esd", __func__);

	if (!iris_is_pt_mode()) {
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
			IRIS_LOGI("%s(), in bypass mode %d", __func__, iris_get_abyp_mode());
		return -2;
	}

	if ((pcfg->esd_check_num > 0) && (pcfg->esd_read_index >= pcfg->esd_check_num))
		pcfg->esd_read_index = 0;

	IRIS_LOGV("%s() esd_check_num = %d, read_index = %d, read_flag = %d", __func__, pcfg->esd_check_num, pcfg->esd_read_index, pcfg->esd_read_flag);

	if (0 == i) {
		pcfg->esd_read_flag = false;
		if (iris_esd_ctrl_get() & 0x1) {
			rc = iris_get_status();
			if (rc <= 0) {
				if (iris_esd_ctrl_get() & 0x4)
					pcfg->esd_chk_val[0] = 0xbad; // trigger esd recovery
				goto exit;
			} else {
				/* reset the esd_chk_val value */
				pcfg->esd_chk_val[0] = pcfg->panel_ext->params->lcm_esd_check_table[0].para_list[0];
			}
		}
	}

	if (iris_esd_ctrl_get() & 0x2) {
		if (FRC_MODE != pcfg->pwil_mode) {
			if ((i == pcfg->esd_read_index) && !pcfg->esd_read_flag) {
				rc = iris_panel_read_status(i, cmd);
				pcfg->esd_read_flag = true;

				if (pcfg->esd_check_num > 0)
					pcfg->esd_read_index++;
			}
		}

		if (pcfg->lp_ctrl.esd_trigger == 1)
			rc = -1;
		if (rc <= 0) {
			pcfg->lp_ctrl.esd_cnt_panel++;
			IRIS_LOGI("panel esd err cnt: %d. rc %d", pcfg->lp_ctrl.esd_cnt_panel, rc);
			goto exit;
		}
	}

exit:
	if (rc <= 0) {
		if ((iris_esd_ctrl_get() & 0x4) == 0)
			rc = 1; /* Force not return error */
	}
	if (pcfg->lp_ctrl.esd_trigger)
		pcfg->lp_ctrl.esd_trigger = 0;
	return rc;
}

u32 iris_get_panel_esd_state(int i)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	if ((0 <= i) && (i < ESD_CHK_NUM)){
		if (iris_esd_ctrl_get() & 0x4)
			return pcfg->esd_chk_val[i];
		else
			return ESD_CHK_EXIT;
	}
	else
		return 0;
}

void iris_set_esd_check_ongoing(bool status)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->is_esd_check_ongoing = status;
}

bool iris_get_esd_check_ongoing(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->is_esd_check_ongoing;
}

void iris_set_esd_check_num(u32 n)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (ESD_CHK_NUM < n)
		n = ESD_CHK_NUM;

	pcfg->esd_check_num = n;
}

void iris_set_valid(int step)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->valid = step;
}
EXPORT_SYMBOL(iris_set_valid);
