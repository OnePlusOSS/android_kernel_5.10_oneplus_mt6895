// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>

#include <lpm_dbg_common_v1.h>
#include <lpm_module.h>
#include <mtk_idle_sysfs.h>
#include <mtk_suspend_sysfs.h>
#include <mtk_spm_sysfs.h>

#include <mt6895_pwr_ctrl.h>
#include <lpm_dbg_fs_common.h>
#include <lpm_spm_comm.h>
#include <mt6895_spm_reg.h>
#include <mt-plat/mtk_ccci_common.h>

/* Determine for node route */
#define MT_LP_RQ_NODE	"/proc/mtk_lpm/spm/spm_resource_req"

#define DEFINE_ATTR_RO(_name)			\
	static struct kobj_attribute _name##_attr = {	\
		.attr	= {				\
			.name = #_name,			\
			.mode = 0444,			\
		},					\
		.show	= _name##_show,			\
	}
#define DEFINE_ATTR_RW(_name)			\
	static struct kobj_attribute _name##_attr = {	\
		.attr	= {				\
			.name = #_name,			\
			.mode = 0644,			\
		},					\
		.show	= _name##_show,			\
		.store	= _name##_store,		\
	}
#define __ATTR_OF(_name)	(&_name##_attr.attr)

#undef mtk_dbg_spm_log
#define mtk_dbg_spm_log(fmt, args...) \
	do { \
		int l = scnprintf(p, sz, fmt, ##args); \
		p += l; \
		sz -= l; \
	} while (0)


#define plat_mmio_read(offset)	__raw_readl(lpm_spm_base + offset)

static char *mt6895_pwr_ctrl_str[PW_MAX_COUNT] = {
	[PW_PCM_FLAGS] = "pcm_flags",
	[PW_PCM_FLAGS_CUST] = "pcm_flags_cust",
	[PW_PCM_FLAGS_CUST_SET] = "pcm_flags_cust_set",
	[PW_PCM_FLAGS_CUST_CLR] = "pcm_flags_cust_clr",
	[PW_PCM_FLAGS1] = "pcm_flags1",
	[PW_PCM_FLAGS1_CUST] = "pcm_flags1_cust",
	[PW_PCM_FLAGS1_CUST_SET] = "pcm_flags1_cust_set",
	[PW_PCM_FLAGS1_CUST_CLR] = "pcm_flags1_cust_clr",
	[PW_TIMER_VAL] = "timer_val",
	[PW_TIMER_VAL_CUST] = "timer_val_cust",
	[PW_TIMER_VAL_RAMP_EN] = "timer_val_ramp_en",
	[PW_TIMER_VAL_RAMP_EN_SEC] = "timer_val_ramp_en_sec",
	[PW_WAKE_SRC] = "wake_src",
	[PW_WAKE_SRC_CUST] = "wake_src_cust",
	[PW_WAKELOCK_TIMER_VAL] = "wakelock_timer_val",
	[PW_WDT_DISABLE] = "wdt_disable",
	/* SPM_CLK_CON */
	[PW_REG_SPM_LOCK_INFRA_DCM_LSB] = "spm_lock_infra_dcm_lsb",
	[PW_REG_MD1_C32RM_EN_LSB] = "md1_c32rm_en_lsb",
	[PW_REG_SPM_LEAVE_SUSPEND_MERGE_MASK_LSB] = "spm_leave_suspend_merge_mask_lsb",
	[PW_REG_SYSCLK0_SRC_MASK_B_LSB] = "sysclk0_src_mask_b_lsb",
	[PW_REG_SYSCLK1_SRC_MASK_B_LSB] = "sysclk1_src_mask_b_lsb",
	[PW_REG_SYSCLK2_SRC_MASK_B_LSB] = "sysclk2_src_mask_b_lsb",

	/* SPM_AP_STANDBY_CON */
	[PW_REG_WFI_OP] = "reg_wfi_op",
	[PW_REG_WFI_TYPE] = "reg_wfi_type",
	[PW_REG_MP0_CPUTOP_IDLE_MASK] = "reg_mp0_cputop_idle_mask",
	[PW_REG_MP1_CPUTOP_IDLE_MASK] = "reg_mp1_cputop_idle_mask",
	[PW_REG_MCUSYS_IDLE_MASK] = "reg_mcusys_idle_mask",
	[PW_REG_CSYSPWRUP_REQ_MASK] = "reg_csyspwrup_req_mask_lsb",
	[PW_REG_WFI_AF_SEL] = "reg_wfi_af_sel",
	[PW_REG_CPU_SLEEP_WFI] = "reg_cpu_sleep_wfi",

	/* SPM_SRC_REQ */
	[PW_REG_SPM_ADSP_MAILBOX_REQ] = "reg_spm_adsp_mailbox_req",
	[PW_REG_SPM_APSRC_REQ] = "reg_spm_apsrc_req",
	[PW_REG_SPM_DDREN_REQ] = "reg_spm_ddren_req",
	[PW_REG_SPM_DVFS_REQ] = "reg_spm_dvfs_req",
	[PW_REG_SPM_F26M_REQ] = "reg_spm_f26m_req",
	[PW_REG_SPM_INFRA_REQ] = "reg_spm_infra_req",
	[PW_REG_SPM_SCP_MAILBOX_REQ] = "reg_spm_scp_mailbox_req",
	[PW_REG_SPM_SSPM_MAILBOX_REQ] = "reg_spm_sspm_mailbox_req",
	[PW_REG_SPM_SW_MAILBOX_REQ] = "reg_spm_sw_mailbox_req",
	[PW_REG_SPM_VCORE_REQ] = "reg_spm_vcore_req",
	[PW_REG_SPM_VRF18_REQ] = "reg_spm_vrf18_req",
	[PW_REG_ADSP_MAILBOX_STATE] = "adsp_mailbox_state",
	[PW_REG_APSRC_STATE] = "apsrc_state",
	[PW_REG_DDREN_STATE] = "ddren_state",
	[PW_REG_DVFS_STATE] = "dvfs_state",
	[PW_REG_F26M_STATE] = "f26m_state",
	[PW_REG_INFRA_STATE] = "infra_state",
	[PW_REG_SCP_MAILBOX_STATE] = "scp_mailbox_state",
	[PW_REG_SSSM_MAILBOX_STATE] = "sspm_mailbox_state",
	[PW_REG_SW_MAILBOX_STATE] = "sw_mailbox_state",
	[PW_REG_VCORE_STATE] = "vcore_state",
	[PW_REG_VRF18_STATE] = "vrf18_state",

	/* SPM_SRC_MASK_0 */
	[PW_REG_AFE_APSRC_REQ_MASK_B] = "reg_afe_apsrc_req_mask_b",
	[PW_REG_AFE_DDREN_REQ_MASK_B] = "reg_afe_ddren_req_mask_b",
	[PW_REG_AFE_INFRA_REQ_MASK_B] = "reg_afe_infra_req_mask_b",
	[PW_REG_AFE_SRCCLKENA_MASK_B] = "reg_afe_srcclkena_mask_b",
	[PW_REG_AFE_VRF18_REQ_MASK_B] = "reg_afe_vrf18_req_mask_b",
	[PW_REG_APU_APSRC_REQ_MASK_B] = "reg_apu_apsrc_req_mask_b",
	[PW_REG_APU_DDREN_REQ_MASK_B] = "reg_apu_ddren_req_mask_b",
	[PW_REG_APU_INFRA_REQ_MASK_B] = "reg_apu_infra_req_mask_b",
	[PW_REG_APU_SRCCLKENA_MASK_B] = "reg_apu_srcclkena_mask_b",
	[PW_REG_APU_VRF18_REQ_MASK_B] = "reg_apu_vrf18_req_mask_b",
	[PW_REG_AUDIO_DSP_APSRC_REQ_MASK_B] = "reg_audio_dsp_apsrc_req_mask_b",
	[PW_REG_AUDIO_DSP_DDREN_REQ_MASK_B] = "reg_audio_dsp_ddren_req_mask_b",
	[PW_REG_AUDIO_DSP_INFRA_REQ_MASK_B] = "reg_audio_dsp_infra_req_mask_b",
	[PW_REG_AUDIO_DSP_SRCCLKENA_MASK_B] = "reg_audio_dsp_srcclkena_mask_b",
	[PW_REG_AUDIO_DSP_VRF18_REQ_MASK_B] = "reg_audio_dsp_vrf18_req_mask_b",
	[PW_REG_CAM_APSRC_REQ_MASK_B] = "reg_cam_apsrc_req_mask_b",
	[PW_REG_CAM_DDREN_REQ_MASK_B] = "reg_cam_ddren_req_mask_b",
	[PW_REG_CCIF_EVENT_APSRC_REQ_MASK_B] = "reg_ccif_apsrc_req_mask_b",

	/* SPM_SRC_MASK_1 */
	[PW_REG_CCIF_EVENT_INFRA_REQ_MASK_B] = "reg_ccif_infra_req_mask_b",
	[PW_REG_CCIF_EVENT_SRCCLKENA_MASK_B] = "reg_ccif_srcclkena_mask_b",

	/* SPM_SRC_MASK_2 */
	[PW_REG_CG_CHECK_APSRC_REQ_MASK_B] = "reg_cg_check_apsrc_req_mask_b",
	[PW_REG_CG_CHECK_DDREN_REQ_MASK_B] = "reg_cg_check_ddren_req_mask_b",
	[PW_REG_CG_CHECK_SRCCLKENA_MASK_B] = "reg_cg_check_srcclkena_mask_b",
	[PW_REG_CG_CHECK_VCORE_REQ_MASK_B] = "reg_cg_check_vcore_req_mask_b",
	[PW_REG_CG_CHECK_VRF18_REQ_MASK_B] = "reg_cg_check_vrf18_req_mask_b",
	[PW_REG_CONN_APSRC_REQ_MASK_B] = "reg_conn_apsrc_req_mask_b",
	[PW_REG_CONN_DDREN_REQ_MASK_B] = "reg_conn_ddren_req_mask_b",
	[PW_REG_CONN_INFRA_REQ_MASK_B] = "reg_conn_infra_req_mask_b",
	[PW_REG_CONN_SRCCLKENA_MASK_B] = "reg_conn_srcclkena_mask_b",
	[PW_REG_CONN_SRCCLKENB_MASK_B] = "reg_conn_srcclkenb_mask_b",
	[PW_REG_CONN_VCORE_REQ_MASK_B] = "reg_conn_vcore_req_mask_b",
	[PW_REG_CONN_VRF18_REQ_MASK_B] = "reg_conn_vrf18_req_mask_b",
	[PW_REG_MCUPM_APSRC_REQ_MASK_B] = "reg_mcupm_apsrc_req_mask_b",
	[PW_REG_MCUPM_DDREN_REQ_MASK_B] = "reg_cmcupm_ddren_req_mask_b",
	[PW_REG_MCUPM_INFRA_REQ_MASK_B] = "reg_mcupm_infra_req_mask_b",
	[PW_REG_MCUPM_SRCCLKENA_REQ_MASK_B] = "reg_mcupm_srcclkena_mask_b",
	[PW_REG_MCUPM_VRF18_REQ_MASK_B] = "reg_mcupm_vrf18_req_mask_b",
	[PW_REG_DISP0_APSRC_REQ_MASK_B] = "reg_disp0_apsrc_req_mask_b",
	[PW_REG_DISP0_DDREN_REQ_MASK_B] = "reg_disp0_ddren_req_mask_b",
	[PW_REG_DISP1_APSRC_REQ_MASK_B] = "reg_disp1_apsrc_req_mask_b",
	[PW_REG_DISP1_DDREN_REQ_MASK_B] = "reg_disp1_ddren_req_mask_b",
	[PW_REG_DPM_APSRC_REQ_MASK_B] = "reg_dpm_apsrc_req_mask_b",
	[PW_REG_DPM_INFRA_REQ_MASK_B] = "reg_dpm_infra_req_mask_b",
	[PW_REG_DPM_VRF18_REQ_MASK_B] = "reg_dpm_vrf18_req_mask_b",
	[PW_REG_DPMAIF_APSRC_REQ_MASK_B] = "reg_dpmaif_apsrc_req_mask_b",
	[PW_REG_DPMAIF_DDREN_REQ_MASK_B] = "reg_dpmaif_ddren_req_mask_b",
	[PW_REG_DPMAIF_INFRA_REQ_MASK_B] = "reg_dpmaif_infra_req_mask_b",
	[PW_REG_DPMAIF_SRCCLKENA_MASK_B] = "reg_dpmaif_srcclkena_mask_b",
	[PW_REG_DPMAIF_VRF18_REQ_MASK_B] = "reg_dpmaif_vrf18_req_mask_b",
	[PW_REG_DVFSRC_LEVEL_REQ_MASK_B] = "reg_dvfsrc_level_req_mask_b",
	[PW_REG_GCE_APSRC_REQ_MASK_B] = "reg_gce_apsrc_req_mask_b",
	[PW_REG_GCE_DDREN_REQ_MASK_B] = "reg_gce_ddren_req_mask_b",
	[PW_REG_GCE_INFRA_REQ_MASK_B] = "reg_gce_infra_req_mask_b",
	[PW_REG_GCE_VRF18_REQ_MASK_B] = "reg_gce_vrf18_req_mask_b",
	[PW_REG_GPUEB_APSRC_REQ_MASK_B] = "reg_gpueb_apsrc_req_mask_b",

	/* SPM_SRC_MASK_3 */
	[PW_REG_GPUEB_DDREN_REQ_MASK_B] = "reg_gpueb_ddren_req_mask_b",
	[PW_REG_GPUEB_INFRA_REQ_MASK_B] = "reg_gpueb_infra_req_mask_b",
	[PW_REG_GPUEB_SRCCLKENA_MASK_B] = "reg_gpueb_srcclkena_mask_b",
	[PW_REG_GPUEB_VRF18_REQ_MASK_B] = "reg_gpueb_vrf18_req_mask_b",
	[PW_REG_IMG_APSRC_REQ_MASK_B] = "reg_img_apsrc_req_mask_b",
	[PW_REG_IMG_DDREN_REQ_MASK_B] = "reg_img_ddren_req_mask_b",
	[PW_REG_INFRASYS_APSRC_REQ_MASK_B] = "reg_infrasys_apsrc_req_mask_b",
	[PW_REG_INFRASYS_DDREN_REQ_MASK_B] = "reg_infrasys_ddren_req_mask_b",
	[PW_REG_IPIC_INFRA_REQ_MASK_B] = "reg_ipic_infra_req_mask_b",
	[PW_REG_IPIC_VRF18_REQ_MASK_B] = "reg_ipic_vrf18_req_mask_b",

	[PW_REG_MCUSYS_MERGE_APSRC_REQ_MASK_B] = "reg_mcusys_apsrc_req_mask_b",
	[PW_REG_MCUSYS_MERGE_DDREN_REQ_MASK_B] = "reg_mcusys_ddren_req_mask_b",
	[PW_REG_MD_APSRC_REQ_MASK_B] = "reg_md_apsrc_req_mask_b",

	/* SPM_SRC_MASK_4 */
	[PW_REG_MD_DDREN_REQ_MASK_B] = "reg_md_ddren_req_mask_b",
	[PW_REG_MD_INFRA_REQ_MASK_B] = "reg_md_infra_req_mask_b",
	[PW_REG_MD_SRCCLKENA_MASK_B] = "reg_md_srcclkena_mask_b",
	[PW_REG_MD_SRCCLKENA1_MASK_B] = "reg_md_srcclkena1_mask_b",
	[PW_REG_MD_VCORE_REQ_MASK_B] = "reg_md_vcore_req_mask_b",
	[PW_REG_MD_VRF18_REQ_MASK_B] = "reg_md_vrf18_req_mask_b",
	[PW_REG_MDP_APSRC_REQ_MASK_B] = "reg_mdp_apsrc_req_mask_b",
	[PW_REG_MDP_DDREN_REQ_MASK_B] = "reg_mdp_ddren_req_mask_b",
	[PW_REG_MM_PROC_APSRC_REQ_MASK_B] = "reg_mm_proc_apsrc_req_mask_b",
	[PW_REG_MM_PROC_DDREN_REQ_MASK_B] = "reg_mm_proc_ddren_req_mask_b",
	[PW_REG_MM_PROC_INFRA_REQ_MASK_B] = "reg_mm_proc_infra_req_mask_b",
	[PW_REG_MM_PROC_SRCCLKENA_REQ_MASK_B] = "reg_mm_proc_srcclkena_mask_b",
	[PW_REG_MM_PROC_VRF18_REQ_MASK_B] = "reg_mm_proc_vrf18_req_mask_b",
	[PW_REG_MMSYS_APSRC_REQ_MASK_B] = "reg_mmsys_apsrc_req_mask_b",
	[PW_REG_MMSYS_DDREN_REQ_MASK_B] = "reg_mmsys_ddren_req_mask_b",
	[PW_REG_MMSYS_VRF18_REQ_MASK_B] = "reg_mmsys_vrf18_req_mask_b",
	[PW_REG_MSDC1_APSRC_REQ_MASK_B] = "reg_msdc1_apsrc_req_mask_b",
	[PW_REG_MSDC1_DDREN_REQ_MASK_B] = "reg_msdc1_ddren_req_mask_b",
	[PW_REG_MSDC1_INFRA_REQ_MASK_B] = "reg_msdc1_infra_req_mask_b",
	[PW_REG_MSDC1_SRCCLKENA_MASK_B] = "reg_msdc1_srcclkena_mask_b",
	[PW_REG_MSDC1_VRF18_REQ_MASK_B] = "reg_msdc1_vrf18_req_mask_b",
	[PW_REG_MSDC2_APSRC_REQ_MASK_B] = "reg_msdc2_apsrc_req_mask_b",
	[PW_REG_MSDC2_DDREN_REQ_MASK_B] = "reg_msdc2_ddren_req_mask_b",
	[PW_REG_MSDC2_INFRA_REQ_MASK_B] = "reg_msdc2_infra_req_mask_b",
	[PW_REG_MSDC2_SRCCLKENA_MASK_B] = "reg_msdc2_srcclkena_mask_b",
	[PW_REG_MSDC2_VRF18_REQ_MASK_B] = "reg_msdc2_vrf18_req_mask_b",
	[PW_REG_PCIE0_APSRC_REQ_MASK_B] = "reg_pcie0_apsrc_req_mask_b",
	[PW_REG_PCIE0_DDREN_REQ_MASK_B] = "reg_pcie0_ddren_req_mask_b",
	[PW_REG_PCIE0_INFRA_REQ_MASK_B] = "reg_pcie0_infra_req_mask_b",
	[PW_REG_PCIE0_SRCCLKENA_MASK_B] = "reg_pcie0_srcclkena_mask_b",
	[PW_REG_PCIE_VRF18_REQ_MASK_B] = "reg_pcie0_vrf18_req_mask_b",
	[PW_REG_PCIE1_APSRC_REQ_MASK_B] = "reg_pcie1_apsrc_req_mask_b",
	[PW_REG_PCIE1_DDREN_REQ_MASK_B] = "reg_pcie1_ddren_req_mask_b",
	[PW_REG_PCIE1_INFRA_REQ_MASK_B] = "reg_pcie1_infra_req_mask_b",
	[PW_REG_PCIE1_SRCCLKENA_MASK_B] = "reg_pcie1_srcclkena_mask_b",
	[PW_REG_PCIE1_VRF18_REQ_MASK_B] = "reg_pcie1_vrf18_req_mask_b",

	[PW_REG_SCP_APSRC_REQ_MASK_B] = "reg_scp_apsrc_req_mask_b",
	[PW_REG_SCP_DDREN_REQ_MASK_B] = "reg_scp_ddren_req_mask_b",
	[PW_REG_SCP_INFRA_REQ_MASK_B] = "reg_scp_infra_req_mask_b",
	[PW_REG_SCP_SRCCLKENA_MASK_B] = "reg_scp_srcclkena_mask_b",

	/* SPM_SRC_MASK_5 */
	[PW_REG_SCP_VRF18_REQ_MASK_B] = "reg_scp_vrf18_req_mask_b",
	[PW_REG_SRCCLKENI_INFRA_REQ_MASK_B] = "reg_srcclkeni_infra_req_mask_b",
	[PW_REG_SRCCLKENI_SRCCLKENA_MASK_B] = "reg_srcclkeni_srcclkena_mask_b",
	[PW_REG_SSPM_APSRC_REQ_MASK_B] = "reg_sspm_apsrc_req_mask_b",
	[PW_REG_SSPM_DDREN_REQ_MASK_B] = "reg_sspm_ddren_req_mask_b",
	[PW_REG_SSPM_INFRA_REQ_MASK_B] = "reg_sspm_infra_req_mask_b",
	[PW_REG_SSPM_SRCCLKENA_MASK_B] = "reg_sspm_srcclkena_mask_b",
	[PW_REG_SSPM_VRF18_REQ_MASK_B] = "reg_sspm_vrf18_req_mask_b",
	[PW_REG_SSUSB0_APSRC_REQ_MASK_B] = "reg_ssusb0_apsrc_req_mask_b",
	[PW_REG_SSUSB0_DDREN_REQ_MASK_B] = "reg_ssusb0_ddren_req_mask_b",
	[PW_REG_SSUSB0_INFRA_REQ_MASK_B] = "reg_ssusb0_infra_req_mask_b",
	[PW_REG_SSUSB0_SRCCLKENA_MASK_B] = "reg_ssusb0_srcclkena_mask_b",
	[PW_REG_SSUSB0_VRF18_REQ_MASK_B] = "reg_ssusb0_vrf18_req_mask_b",
	[PW_REG_SSUSB1_APSRC_REQ_MASK_B] = "reg_ssusb1_apsrc_req_mask_b",
	[PW_REG_SSUSB1_DDREN_REQ_MASK_B] = "reg_ssusb1_ddren_req_mask_b",
	[PW_REG_SSUSB1_INFRA_REQ_MASK_B] = "reg_ssusb1_infra_req_mask_b",
	[PW_REG_SSUSB1_SRCCLKENA_MASK_B] = "reg_ssusb1_srcclkena_mask_b",
	[PW_REG_SSUSB1_VRF18_REQ_MASK_B] = "reg_ssusb1_vrf18_req_mask_b",
	[PW_REG_UFS_APSRC_REQ_MASK_B] = "reg_ufs_apsrc_req_mask_b",
	[PW_REG_UFS_DDREN_REQ_MASK_B] = "reg_ufs_ddren_req_mask_b",
	[PW_REG_UFS_INFRA_REQ_MASK_B] = "reg_ufs_infra_req_mask_b",
	[PW_REG_UFS_SRCCLKENA_MASK_B] = "reg_ufs_srcclkena_mask_b",
	[PW_REG_UFS_VRF18_REQ_MASK_B] = "reg_ufs_vrf18_req_mask_b",

	[PW_REG_VDEC_APSRC_REQ_MASK_B] = "reg_vdec_apsrc_req_mask_b",
	[PW_REG_VDEC_DDREN_REQ_MASK_B] = "reg_vdec_ddren_req_mask_b",
	[PW_REG_VENC_APSRC_REQ_MASK_B] = "reg_venc_apsrc_req_mask_b",
	[PW_REG_VENC_DDREN_REQ_MASK_B] = "reg_venc_ddren_req_mask_b",
	/* SPM_EVENT_CON_MISC */
	[PW_REG_SRCCLKEN_FAST_RESP] = "reg_srcclken_fast_resp",
	[PW_REG_CSYSPWRUP_ACK_MASK] = "reg_csyspwrup_ack_mask",

	/* SPM_WAKEUP_EVENT_MASK */
	[PW_REG_WAKEUP_EVENT_MASK] = "reg_wakeup_event_mask",
	/* SPM_WAKEUP_EVENT_EXT_MASK */
	[PW_REG_EXT_WAKEUP_EVENT_MASK] = "reg_ext_wakeup_event_mask",
};


/**************************************
 * xxx_ctrl_show Function
 **************************************/
/* code gen by spm_pwr_ctrl_atf.pl, need struct pwr_ctrl */
static ssize_t mt6895_show_pwr_ctrl(int id, char *buf, size_t buf_sz)
{
	char *p = buf;
	size_t mSize = 0;
	int i;

	for (i = 0; i < PW_MAX_COUNT; i++) {
		mSize += scnprintf(p + mSize, buf_sz - mSize,
			"%s = 0x%zx\n",
			mt6895_pwr_ctrl_str[i],
				lpm_smc_spm_dbg(id, MT_LPM_SMC_ACT_GET, i, 0));
	}

	WARN_ON(buf_sz - mSize <= 0);

	return mSize;
}

/**************************************
 * xxx_ctrl_store Function
 **************************************/
/* code gen by spm_pwr_ctrl_atf.pl, need struct pwr_ctrl */
static ssize_t mt6895_store_pwr_ctrl(int id,	const char *buf, size_t count)
{
	u32 val;
	char cmd[64];
	int i;

	if (sscanf(buf, "%63s %x", cmd, &val) != 2)
		return -EINVAL;

	pr_info("[SPM] pwr_ctrl: cmd = %s, val = 0x%x\n", cmd, val);

	for (i = 0 ; i < PW_MAX_COUNT; i++) {
		if (!strcmp(cmd, mt6895_pwr_ctrl_str[i])) {
			lpm_smc_spm_dbg(id, MT_LPM_SMC_ACT_SET, i, val);
			break;
		}
	}

	return count;
}

static ssize_t
mt6895_generic_spm_read(char *ToUserBuf, size_t sz, void *priv);

static ssize_t
mt6895_generic_spm_write(char *FromUserBuf, size_t sz, void *priv);

struct mt6895_SPM_ENTERY {
	const char *name;
	int mode;
	struct mtk_lp_sysfs_handle handle;
};

struct mt6895_SPM_NODE {
	const char *name;
	int mode;
	struct mtk_lp_sysfs_handle handle;
	struct mtk_lp_sysfs_op op;
};


struct mt6895_SPM_ENTERY mt6895_spm_root = {
	.name = "power",
	.mode = 0644,
};

struct mt6895_SPM_NODE mt6895_spm_idle = {
	.name = "idle_ctrl",
	.mode = 0644,
	.op = {
		.fs_read = mt6895_generic_spm_read,
		.fs_write = mt6895_generic_spm_write,
		.priv = (void *)&mt6895_spm_idle,
	},
};

struct mt6895_SPM_NODE mt6895_spm_suspend = {
	.name = "suspend_ctrl",
	.mode = 0644,
	.op = {
		.fs_read = mt6895_generic_spm_read,
		.fs_write = mt6895_generic_spm_write,
		.priv = (void *)&mt6895_spm_suspend,
	},
};

static ssize_t
mt6895_generic_spm_read(char *ToUserBuf, size_t sz, void *priv)
{
	int id = MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL;

	if (priv == &mt6895_spm_idle)
		id = MT_SPM_DBG_SMC_UID_IDLE_PWR_CTRL;
	return mt6895_show_pwr_ctrl(id, ToUserBuf, sz);
}

#include <mtk_lpm_sysfs.h>

static ssize_t
mt6895_generic_spm_write(char *FromUserBuf, size_t sz, void *priv)
{
	int id = MT_SPM_DBG_SMC_UID_SUSPEND_PWR_CTRL;

	if (priv == &mt6895_spm_idle)
		id = MT_SPM_DBG_SMC_UID_IDLE_PWR_CTRL;

	return mt6895_store_pwr_ctrl(id, FromUserBuf, sz);
}

static char *mt6895_spm_resource_str[MT_SPM_RES_MAX] = {
	[MT_SPM_RES_XO_FPM] = "XO_FPM",
	[MT_SPM_RES_CK_26M] = "CK_26M",
	[MT_SPM_RES_INFRA] = "INFRA",
	[MT_SPM_RES_SYSPLL] = "SYSPLL",
	[MT_SPM_RES_DRAM_S0] = "DRAM_S0",
	[MT_SPM_RES_DRAM_S1] = "DRAM_S1",
};

static ssize_t mt6895_spm_res_rq_read(char *ToUserBuf, size_t sz, void *priv)
{
	char *p = ToUserBuf;
	int i, s, u;
	unsigned int unum, uvalid, uname_i, uname_t;
	unsigned int rnum, rusage, per_usage;
	char uname[MT_LP_RQ_USER_NAME_LEN+1];

	mtk_dbg_spm_log("resource_num=%d, user_num=%d, user_valid=0x%x\n",
	    rnum = lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_NUM,
				       MT_LPM_SMC_ACT_GET, 0, 0),
	    unum = lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_USER_NUM,
				       MT_LPM_SMC_ACT_GET, 0, 0),
	    uvalid = lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_USER_VALID,
					 MT_LPM_SMC_ACT_GET, 0, 0));
	rusage = lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_USAGE,
				     MT_LPM_SMC_ACT_GET,
				     MT_LP_RQ_ID_ALL_USAGE, 0);
	mtk_dbg_spm_log("\n");
	mtk_dbg_spm_log("user [bit][valid]:\n");
	for (i = 0; i < unum; i++) {
		uname_i = lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_USER_NAME,
					    MT_LPM_SMC_ACT_GET, i, 0);
		for (s = 0, u = 0; s < MT_LP_RQ_USER_NAME_LEN;
		     s++, u += MT_LP_RQ_USER_CHAR_U) {
			uname_t = ((uname_i >> u) & MT_LP_RQ_USER_CHAR_MASK);
			uname[s] = (uname_t) ? (char)uname_t : ' ';
		}
		uname[s] = '\0';
		mtk_dbg_spm_log("%4s [%3d][%3s]\n", uname, i,
		    ((1<<i) & uvalid) ? "yes" : "no");
	}
	mtk_dbg_spm_log("\n");

	if (rnum != MT_SPM_RES_MAX) {
		mtk_dbg_spm_log("Platform resource amount mismatch\n");
		rnum = (rnum > MT_SPM_RES_MAX) ? MT_SPM_RES_MAX : rnum;
	}

	mtk_dbg_spm_log("resource [bit][user_usage][blocking]:\n");
	for (i = 0; i < rnum; i++) {
		mtk_dbg_spm_log("%8s [%3d][0x%08x][%3s]\n",
			mt6895_spm_resource_str[i], i,
			(per_usage =
			lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_USAGE,
					    MT_LPM_SMC_ACT_GET, i, 0)),
			((1<<i) & rusage) ? "yes" : "no"
		   );
	}
	mtk_dbg_spm_log("\n");
	mtk_dbg_spm_log("resource request command help:\n");
	mtk_dbg_spm_log("echo enable ${user_bit} > %s\n", MT_LP_RQ_NODE);
	mtk_dbg_spm_log("echo bypass ${user_bit} > %s\n", MT_LP_RQ_NODE);
	mtk_dbg_spm_log("echo request ${resource_bit} > %s\n", MT_LP_RQ_NODE);
	mtk_dbg_spm_log("echo release > %s\n", MT_LP_RQ_NODE);

	return p - ToUserBuf;
}

static ssize_t mt6895_spm_res_rq_write(char *FromUserBuf, size_t sz, void *priv)
{
	char cmd[128];
	int parm;

	if (sscanf(FromUserBuf, "%127s %x", cmd, &parm) == 2) {
		if (!strcmp(cmd, "bypass"))
			lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_USER_VALID,
					    MT_LPM_SMC_ACT_SET,
					    parm, 0);
		else if (!strcmp(cmd, "enable"))
			lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_USER_VALID,
					    MT_LPM_SMC_ACT_SET,
					    parm, 1);
		else if (!strcmp(cmd, "request"))
			lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_REQ,
					    MT_LPM_SMC_ACT_SET,
					    0, parm);
		return sz;
	} else if (sscanf(FromUserBuf, "%127s", cmd) == 1) {
		if (!strcmp(cmd, "release"))
			lpm_smc_spm_dbg(MT_SPM_DBG_SMC_UID_RES_REQ,
					    MT_LPM_SMC_ACT_CLR,
					    0, 0);
		return sz;
	} else if ((!kstrtoint(FromUserBuf, 10, &parm)) == 1) {
		return sz;
	}

	return -EINVAL;
}

void __attribute__ ((weak))
__iomem *get_smem_start_addr(int md_id, enum SMEM_USER_ID user_id,
	int *size_o)
{
		pr_info("%s not ready\n", __func__);
			return 0;
}


#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
#define MD_GUARD_NUMBER 0x536C702E
#define GET_RECORD_CNT1(n) ((n >> 32) & 0xFFFFFFFF)
#define GET_RECORD_CNT2(n) (n & 0xFFFFFFFF)
#define GET_GUARD_L(n) (n & 0xFFFFFFFF)
#define GET_GUARD_H(n) ((n >> 32) & 0xFFFFFFFF)
static struct md_sleep_status md_data;

static int is_md_sleep_info_valid(struct md_sleep_status *md_data)
{
	u32 guard_l = GET_GUARD_L(md_data->guard_sleep_cnt1);
	u32 guard_h = GET_GUARD_H(md_data->guard_sleep_cnt2);
	u32 cnt1 = GET_RECORD_CNT1(md_data->guard_sleep_cnt1);
	u32 cnt2 = GET_RECORD_CNT2(md_data->guard_sleep_cnt2);

	if ((guard_l != MD_GUARD_NUMBER) || (guard_h != MD_GUARD_NUMBER))
		return 0;

	if (cnt1 != cnt2)
		return 0;

	return 1;
}

#endif

#define MD_SLEEP_INFO_SMEM_OFFEST (0)
ssize_t mt6895_spm_system_stats_read(char *ToUserBuf, size_t sz, void *priv)
{
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	u32 len = 0;
	u32 *share_mem = NULL;
	struct md_sleep_status tmp_md_data;
	int ret;
	u64 of_find;
	struct device_node *mddriver = NULL;

	mddriver = of_find_compatible_node(NULL, NULL, "mediatek,mddriver");
	if (!mddriver) {
		pr_info("mddriver not found in DTS\n");
		return 0;
	}

	ret =  of_property_read_u64(mddriver, "md_low_power_addr", &of_find);
	if (ret) {
		pr_info("address not found in DTS");
		return 0;
	}

	share_mem = (u32 *)ioremap_wc(of_find, 0x200);

	if (share_mem == NULL) {
		pr_info("[name:spm&][%s:%d] - No MD share mem\n", __func__, __LINE__);
		return 0;
	} else {
		share_mem = share_mem + MD_SLEEP_INFO_SMEM_OFFEST;
		memset(&tmp_md_data, 0, sizeof(struct md_sleep_status));
		memcpy(&tmp_md_data, share_mem, sizeof(struct md_sleep_status));
	}

	if (is_md_sleep_info_valid(&tmp_md_data))
		md_data = tmp_md_data;
	else
		memset(&md_data, 0, sizeof(struct md_sleep_status));

	len = snprintf(ToUserBuf, sz,
	       "26M:%lld:%lld.%03lld\nAP:%lld:%lld.%03lld\nMD:%lld:%lld.%03lld\n",
	       spm_26M_off_count,
	       PCM_TICK_TO_SEC(spm_26M_off_duration),
	       PCM_TICK_TO_SEC((spm_26M_off_duration % PCM_32K_TICKS_PER_SEC) * 1000),
	       ap_pd_count,
	       PCM_TICK_TO_SEC(ap_slp_duration),
	       PCM_TICK_TO_SEC((ap_slp_duration % PCM_32K_TICKS_PER_SEC) * 1000),
	       md_data.sleep_cnt,
	       PCM_TICK_TO_SEC(md_data.sleep_time),
	       PCM_TICK_TO_SEC((md_data.sleep_time % PCM_32K_TICKS_PER_SEC) * 1000));
	if (share_mem != NULL) {
		iounmap(share_mem);
	}
	return (len > sz) ? sz : len;
#else
	return 0;
#endif
}

static const struct mtk_lp_sysfs_op mt6895_spm_res_rq_fops = {
	.fs_read = mt6895_spm_res_rq_read,
	.fs_write = mt6895_spm_res_rq_write,
};
static const struct mtk_lp_sysfs_op mt6895_spm_system_stats_fops = {
	.fs_read = mt6895_spm_system_stats_read,
};

int lpm_spm_fs_init(void)
{
	int r;

	mtk_spm_sysfs_root_entry_create();
	mtk_spm_sysfs_entry_node_add("spm_resource_req", 0444
			, &mt6895_spm_res_rq_fops, NULL);
	mtk_spm_sysfs_entry_node_add("system_stats", 0444
			, &mt6895_spm_system_stats_fops, NULL);

	r = mtk_lp_sysfs_entry_func_create(mt6895_spm_root.name,
					   mt6895_spm_root.mode, NULL,
					   &mt6895_spm_root.handle);
	if (!r) {
		mtk_lp_sysfs_entry_func_node_add(mt6895_spm_suspend.name,
						mt6895_spm_suspend.mode,
						&mt6895_spm_suspend.op,
						&mt6895_spm_root.handle,
						&mt6895_spm_suspend.handle);

		mtk_lp_sysfs_entry_func_node_add(mt6895_spm_idle.name,
						 mt6895_spm_idle.mode,
						 &mt6895_spm_idle.op,
						 &mt6895_spm_root.handle,
						 &mt6895_spm_idle.handle);
	}

	return r;
}

int lpm_spm_fs_deinit(void)
{
	return 0;
}

