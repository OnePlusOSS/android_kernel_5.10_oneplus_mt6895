#ifndef _IRIS_MTK_API_
#define _IRIS_MTK_API_

struct cmdq_pkt;
struct drm_panel;
struct mtk_panel_ext;
struct mtk_ddp_comp;
struct drm_crtc;
struct mipi_dsi_msg;
struct msm_iris_operate_value;
struct mipi_dsi_msg;

struct iris_mtk_dsi_op {
    void (*transfer)(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle,
				void *data, unsigned int len, int state);
    int (*transfer_rdcmd)(struct mtk_ddp_comp *comp, void *handle,
		  struct mipi_dsi_msg *msg, unsigned int slot_index);
    int (*obtain_rdvalue)(struct mtk_ddp_comp *comp, struct mipi_dsi_msg *msg,
      unsigned int slot_index);
};

struct iris_mtk_dsi_op *iris_get_mtk_dsi_op(void);
int iris_operate_conf(struct msm_iris_operate_value *argp);

void iris_set_mtk_dsi_op(struct iris_mtk_dsi_op *op);
void iris_dsi_pre_cmd(struct mtk_ddp_comp *comp, struct drm_crtc *crtc);
void iris_dsi_pos_cmd(struct mtk_ddp_comp *comp, struct drm_crtc *crtc);

void iris_prepare(void);
u8 iris_get_cmd_type(u8 cmd, u32 count);
void iris_init(struct drm_panel *panel, struct mtk_panel_ext *panel_ext);
void iris_deinit(struct drm_panel *panel);
int iris_kickoff(struct mtk_ddp_comp *comp);
int iris_prepare_for_kickoff(struct mtk_ddp_comp *comp);
void iris_send_cont_splash(void);
void iris_print_cmd_desc(const struct mipi_dsi_msg *msg);
bool iris_is_chip_supported(void);
bool iris_is_softiris_supported(void);
bool iris_is_dual_supported(void);
void iris_reset(void);
int iris_switch(void *dev, int id);

int iris_read_status(int i, unsigned char cmd);
u32 iris_get_panel_esd_state(int i);
void iris_set_esd_check_ongoing(bool status);
bool iris_get_esd_check_ongoing(void);
void iris_set_esd_check_num(u32 n);

#endif
