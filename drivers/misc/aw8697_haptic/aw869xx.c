/*
 * File: aw869xx.c
 *
 * Author: Ethan <renzhiqiang@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/vmalloc.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "haptic_hv.h"
#include "haptic_hv_reg.h"

static uint8_t aw869xx_get_glb_state(struct aw_haptic *aw_haptic);
static void aw869xx_vbat_mode_config(struct aw_haptic *, uint8_t);

static int aw869xx_check_qualify(struct aw_haptic *aw_haptic)
{
#if 0
	uint8_t reg_val = 0;
	int ret = -1;

	aw_dev_info("%s: enter", __func__);
	/* chip qualify */
	ret = i2c_r_bytes(aw_haptic, AW869XX_REG_RDATA_A, &reg_val,
			  AW_I2C_BYTE_ONE);
	if (ret < 0)
		return ret;
	if (!(reg_val & 0x80)) {
		aw_dev_err("%s: unqualified chip!", __func__);
		return -ERANGE;
	}
#endif
	return 0;
}

static void aw869xx_dump_rtp_regs(struct aw_haptic *aw_haptic)
{

}

static void aw869xx_set_pwm(struct aw_haptic *aw_haptic, uint8_t mode)
{
	switch (mode) {
	case AW_PWM_48K:
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
			   AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
			   AW869XX_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW_PWM_24K:
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
			   AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
			   AW869XX_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW_PWM_12K:
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
			   AW869XX_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
			   AW869XX_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
}

static void aw869xx_set_gain(struct aw_haptic *aw_haptic, uint8_t gain)
{
	i2c_w_bytes(aw_haptic, AW869XX_REG_PLAYCFG2, &gain,
		    AW_I2C_BYTE_ONE);
}

static void aw869xx_set_bst_peak_cur(struct aw_haptic *aw_haptic)
{
	switch (aw_haptic->bst_pc) {
	case AW_BST_PC_L1:
		aw_dev_info("%s: bst pc = L1", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_BSTCFG1,
			   AW869XX_BIT_BSTCFG1_BST_PC_MASK,
			   AW869XX_BIT_BSTCFG1_PEAKCUR_2P75A);
		return;
	case AW_BST_PC_L2:
		aw_dev_info("%s: bst pc = L2", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_BSTCFG1,
			   AW869XX_BIT_BSTCFG1_BST_PC_MASK,
			   AW869XX_BIT_BSTCFG1_PEAKCUR_4A);
		return;
	default:
		aw_dev_info("%s: bst pc = L1", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_BSTCFG1,
			   AW869XX_BIT_BSTCFG1_BST_PC_MASK,
			   AW869XX_BIT_BSTCFG1_PEAKCUR_2P75A);
		break;
	}
}

static void aw869xx_set_bst_vol(struct aw_haptic *aw_haptic, uint8_t bst_vol)
{
	if (bst_vol & 0xc0)
		bst_vol = 0x3f;
	i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG1,
		   AW869XX_BIT_PLAYCFG1_BST_VOUT_RDA_MASK, bst_vol);
	aw_dev_info("%s: bst_vol 0x%02X", __func__, bst_vol);
}

static void aw869xx_set_wav_seq(struct aw_haptic *aw_haptic, uint8_t wav,
				uint8_t seq)
{
	i2c_w_bytes(aw_haptic, AW869XX_REG_WAVCFG1 + wav, &seq,
		    AW_I2C_BYTE_ONE);
}

static void aw869xx_set_wav_loop(struct aw_haptic *aw_haptic, uint8_t wav,
				 uint8_t loop)
{
	uint8_t tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		i2c_w_bits(aw_haptic, AW869XX_REG_WAVCFG9 + (wav / 2),
			   AW869XX_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		i2c_w_bits(aw_haptic, AW869XX_REG_WAVCFG9 + (wav / 2),
			   AW869XX_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}
}

static void aw869xx_set_rtp_data(struct aw_haptic *aw_haptic, uint8_t *data,
				 uint32_t len)
{
	i2c_w_bytes(aw_haptic, AW869XX_REG_RTPDATA, data, len);
}

static void aw869xx_set_rtp_aei(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSINTM,
			   AW869XX_BIT_SYSINTM_FF_AEM_MASK,
			   AW869XX_BIT_SYSINTM_FF_AEM_ON);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSINTM,
			   AW869XX_BIT_SYSINTM_FF_AEM_MASK,
			   AW869XX_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static void aw869xx_set_ram_addr(struct aw_haptic *aw_haptic, uint32_t base_addr)
{
	uint8_t ram_addr[2] = {0};

	ram_addr[0] = (uint8_t)AW869XX_RAM_ADDR_H(base_addr);
	ram_addr[1] = (uint8_t)AW869XX_RAM_ADDR_L(base_addr);
	i2c_w_bytes(aw_haptic, AW869XX_REG_RAMADDRH, ram_addr,
		    AW_I2C_BYTE_TWO);
}

static void aw869xx_auto_brake_mode(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			   AW869XX_BIT_PLAYCFG3_BRK_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_BRK_EN_MASK,
			   AW869XX_BIT_PLAYCFG3_BRK_DISABLE);
	}
}

static uint8_t aw869xx_get_glb_state(struct aw_haptic *aw_haptic)
{
	uint8_t state = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_GLBRD5, &state,
		    AW_I2C_BYTE_ONE);
	aw_dev_dbg("%s: glb state value is 0x%02X", __func__, state);
	return state;
}

static uint8_t aw869xx_get_chip_state(struct aw_haptic *aw_haptic)
{
	uint8_t chip_state_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSST, &chip_state_val,
		    AW_I2C_BYTE_ONE);
	return chip_state_val;
}

static uint8_t aw869xx_read_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t irq_state_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSINT, &irq_state_val,
		    AW_I2C_BYTE_ONE);
	return irq_state_val;
}

static void aw869xx_play_go(struct aw_haptic *aw_haptic, bool flag)
{
	uint8_t delay_20us = AW869XX_BIT_START_DLY_20US;
	uint8_t delay_2p5ms = AW869XX_BIT_START_DLY_2P5MS;
	uint8_t go_on = AW869XX_BIT_PLAYCFG4_GO_ON;
	uint8_t stop_on = AW869XX_BIT_PLAYCFG4_STOP_ON;

	aw_dev_info("%s: enter, flag = %d", __func__, flag);
	if (flag) {
		if (aw_haptic->info.is_enabled_one_wire) {
			i2c_w_bytes(aw_haptic, AW869XX_REG_GLBCFG2,
				    &delay_20us, AW_I2C_BYTE_ONE);
			i2c_w_bytes(aw_haptic, AW869XX_REG_PLAYCFG4,
				    &go_on, AW_I2C_BYTE_ONE);
			usleep_range(1000, 1500);
			i2c_w_bytes(aw_haptic, AW869XX_REG_GLBCFG2,
				    &delay_2p5ms, AW_I2C_BYTE_ONE);
		} else {
			i2c_w_bytes(aw_haptic, AW869XX_REG_PLAYCFG4,
				    &go_on, AW_I2C_BYTE_ONE);
		}
	} else {
		i2c_w_bytes(aw_haptic, AW869XX_REG_PLAYCFG4, &stop_on,
			    AW_I2C_BYTE_ONE);
	}
}

static void aw869xx_bst_mode_config(struct aw_haptic *aw_haptic,
				    uint8_t boost_mode)
{
	switch (boost_mode) {
	case AW_BST_BOOST_MODE:
		aw_dev_info("%s: haptic boost mode = boost", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG1,
			   AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
			   AW869XX_BIT_PLAYCFG1_BST_MODE_BOOST);
		break;
	case AW_BST_BYPASS_MODE:
		aw_dev_info("%s: haptic boost mode = bypass", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG1,
			   AW869XX_BIT_PLAYCFG1_BST_MODE_MASK,
			   AW869XX_BIT_PLAYCFG1_BST_MODE_BYPASS);
		break;
	default:
		aw_dev_err("%s: boost_mode = %d error", __func__, boost_mode);
		break;
	}
}

static int aw869xx_wait_enter_standby(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int count = 100;

	while (count--) {
		reg_val = aw869xx_get_glb_state(aw_haptic);
		if ((reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_STANDBY ||
		    (reg_val & 0x0f) == AW869XX_BIT_GLBRD5_STATE_I2S_GO) {
			aw_dev_info("%s: entered standby!", __func__);
			return 0;
		}
		aw_dev_dbg("%s: wait for standby", __func__);
		usleep_range(2000, 2500);
	}
	aw_dev_err("%s: do not enter standby automatically", __func__);
	return -ERANGE;
}

static void aw869xx_stop(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = 0;

	aw_haptic->play_mode = AW_STANDBY_MODE;
	reg_val = AW869XX_BIT_PLAYCFG4_STOP_ON;
	i2c_w_bytes(aw_haptic, AW869XX_REG_PLAYCFG4, &reg_val,
		    AW_I2C_BYTE_ONE);
	ret = aw869xx_wait_enter_standby(aw_haptic);
	if (ret < 0) {
		aw_dev_err("%s: force to enter standby mode!", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
			   AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
			   AW869XX_BIT_SYSCTRL2_STANDBY_ON);
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
			   AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
			   AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
	}
}

static void aw869xx_play_mode(struct aw_haptic *aw_haptic, uint8_t play_mode)
{
	switch (play_mode) {
	case AW_STANDBY_MODE:
		aw_dev_info("%s: enter standby mode", __func__);
		aw_haptic->play_mode = AW_STANDBY_MODE;
		aw869xx_stop(aw_haptic);
		break;
	case AW_RAM_MODE:
		aw_dev_info("%s: enter ram mode", __func__);
		aw_haptic->play_mode = AW_RAM_MODE;
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		aw869xx_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		break;
	case AW_RAM_LOOP_MODE:
		aw_dev_info("%s: enter ram loop mode", __func__);
		aw_haptic->play_mode = AW_RAM_LOOP_MODE;
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW_RTP_MODE:
		aw_dev_info("%s: enter rtp mode", __func__);
		aw_haptic->play_mode = AW_RTP_MODE;
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_RTP);
		aw869xx_bst_mode_config(aw_haptic, AW_BST_BOOST_MODE);
		break;
	case AW_TRIG_MODE:
		aw_dev_info("%s: enter trig mode", __func__);
		aw_haptic->play_mode = AW_TRIG_MODE;
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_RAM);
		break;
	case AW_CONT_MODE:
		aw_dev_info("%s: enter cont mode", __func__);
		aw_haptic->play_mode = AW_CONT_MODE;
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_MASK,
			   AW869XX_BIT_PLAYCFG3_PLAY_MODE_CONT);
		aw869xx_bst_mode_config(aw_haptic, AW_BST_BYPASS_MODE);
		break;
	default:
		aw_dev_err("%s: play mode %d error", __func__, play_mode);
		break;
	}
}

static void aw869xx_ram_init(struct aw_haptic *aw_haptic, bool flag)
{
	if (flag) {
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
			   AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
			   AW869XX_BIT_SYSCTRL1_RAMINIT_ON);
		usleep_range(1000, 1050);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
			   AW869XX_BIT_SYSCTRL1_RAMINIT_MASK,
			   AW869XX_BIT_SYSCTRL1_RAMINIT_OFF);
	}
}

static void aw869xx_upload_lra(struct aw_haptic *aw_haptic, uint32_t flag)
{
	switch (flag) {
	case AW_WRITE_ZERO:
		aw_dev_info("%s: write zero to trim_lra!", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_TRIMCFG3,
			   AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK, 0x00);
		break;
	case AW_F0_CALI_LRA:
		aw_dev_info("%s: write f0_cali_data to trim_lra = 0x%02X",
			    __func__, aw_haptic->f0_cali_data);
		i2c_w_bits(aw_haptic, AW869XX_REG_TRIMCFG3,
			   AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
			   (char)aw_haptic->f0_cali_data);
		break;
	case AW_OSC_CALI_LRA:
		aw_dev_info("%s: write osc_cali_data to trim_lra = 0x%02X",
			    __func__, aw_haptic->osc_cali_data);
		i2c_w_bits(aw_haptic, AW869XX_REG_TRIMCFG3,
			   AW869XX_BIT_TRIMCFG3_TRIM_LRA_MASK,
			   (char)aw_haptic->osc_cali_data);
		break;
	default:
		break;
	}
}

static void aw869xx_vbat_mode_config(struct aw_haptic *aw_haptic, uint8_t flag)
{
	uint8_t reg_val = 0;

	if (flag == AW_CONT_VBAT_HW_COMP_MODE) {
		reg_val = AW869XX_BIT_START_DLY_250US;
		i2c_w_bytes(aw_haptic, AW869XX_REG_GLBCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
			   AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
			   AW869XX_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL1,
			   AW869XX_BIT_SYSCTRL1_VBAT_MODE_MASK,
			   AW869XX_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
}

static void aw869xx_protect_config(struct aw_haptic *aw_haptic, uint8_t prtime,
				   uint8_t prlvl)
{
	uint8_t reg_val = 0;

	i2c_w_bits(aw_haptic, AW869XX_REG_PWMCFG1,
		   AW869XX_BIT_PWMCFG1_PRC_EN_MASK,
		   AW869XX_BIT_PWMCFG1_PRC_DISABLE);
	if (prlvl != 0) {
		/* Enable protection mode */
		aw_dev_info("%s: enable protection mode", __func__);
		reg_val = AW869XX_BIT_PWMCFG3_PR_ENABLE |
			  (prlvl & (~AW869XX_BIT_PWMCFG3_PRLVL_MASK));
		i2c_w_bytes(aw_haptic, AW869XX_REG_PWMCFG3, &reg_val,
			    AW_I2C_BYTE_ONE);
		i2c_w_bytes(aw_haptic, AW869XX_REG_PWMCFG4, &prtime,
			    AW_I2C_BYTE_ONE);
	} else {
		/* Disable */
		aw_dev_info("%s: disable protection mode", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_PWMCFG3,
			   AW869XX_BIT_PWMCFG3_PR_EN_MASK,
			   AW869XX_BIT_PWMCFG3_PR_DISABLE);
	}
}

static void aw869xx_cont_config(struct aw_haptic *aw_haptic)
{
	/* uint8_t drv1_time = 0xFF; */
	uint8_t drv2_time = 0xFF;

	/* work mode */
	aw869xx_play_mode(aw_haptic, AW_CONT_MODE);
	/* cont config */
	/* i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG1,
	 *	      AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
	 *	      AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	 */

	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG6,
		   (AW869XX_BIT_CONTCFG6_TRACK_EN_MASK &
		    AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK),
		   (AW869XX_BIT_CONTCFG6_TRACK_ENABLE |
		    aw_haptic->info.cont_drv1_lvl));

	i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG7,
		    &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	/* DRV1_TIME */
	/* i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG8, &drv1_time,
	 *	       AW_I2C_BYTE_ONE);
	 */
	/* DRV2_TIME */
	i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG9, &drv2_time,
		    AW_I2C_BYTE_ONE);
	/* cont play go */
	aw869xx_play_go(aw_haptic, true);
}

static void aw869xx_one_wire_init(struct aw_haptic *aw_haptic)
{
	uint8_t trig_prio = 0x6c;
	uint8_t delay_2p5ms = AW869XX_BIT_START_DLY_2P5MS;

	aw_dev_info("%s: enter", __func__);
	/*if enable one-wire, trig1 priority must be less than trig2 and trig3*/
	i2c_w_bytes(aw_haptic, AW869XX_REG_GLBCFG4, &trig_prio,
		    AW_I2C_BYTE_ONE);
	i2c_w_bytes(aw_haptic, AW869XX_REG_GLBCFG2, &delay_2p5ms,
		    AW_I2C_BYTE_ONE);
	i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG8,
		   AW869XX_BIT_TRGCFG8_TRG_ONEWIRE_MASK,
		   AW869XX_BIT_TRGCFG8_TRG_ONEWIRE_ENABLE);
}

static void aw869xx_i2s_init(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter", __func__);
	i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
		   AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
		   AW869XX_BIT_SYSCTRL2_I2S_PIN_I2S);
	i2c_w_bits(aw_haptic, AW869XX_REG_IOCFG1,
		   AW869XX_BIT_IOCFG1_IO_FAST_MASK,
		   AW869XX_BIT_IOCFG1_IIS_IO_FAST_ENABLE);
}

static void aw869xx_trig1_param_init(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter\n", __func__);
	aw_haptic->trig[0].trig_level = AW869XX_TRIG1_DUAL_LEVEL;
	aw_haptic->trig[0].trig_polar = AW869XX_TRIG1_DUAL_POLAR;
	aw_haptic->trig[0].pos_enable = AW869XX_TRIG1_POS_DISABLE;
	aw_haptic->trig[0].pos_sequence = AW869XX_TRIG1_POS_SEQ;
	aw_haptic->trig[0].neg_enable = AW869XX_TRIG1_NEG_DISABLE;
	aw_haptic->trig[0].neg_sequence = AW869XX_TRIG1_NEG_SEQ;
	aw_haptic->trig[0].trig_brk = AW869XX_TRIG1_BRK_DISABLE;
	aw_haptic->trig[0].trig_bst = AW869XX_TRIG1_BST_DISABLE;
}

static void aw869xx_trig2_param_init(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter\n", __func__);
	aw_haptic->trig[1].trig_level = AW869XX_TRIG2_DUAL_LEVEL;
	aw_haptic->trig[1].trig_polar = AW869XX_TRIG2_DUAL_POLAR;
	aw_haptic->trig[1].pos_enable = AW869XX_TRIG2_POS_DISABLE;
	aw_haptic->trig[1].pos_sequence = AW869XX_TRIG2_POS_SEQ;
	aw_haptic->trig[1].neg_enable = AW869XX_TRIG2_NEG_DISABLE;
	aw_haptic->trig[1].neg_sequence = AW869XX_TRIG2_NEG_SEQ;
	aw_haptic->trig[1].trig_brk = AW869XX_TRIG2_BRK_DISABLE;
	aw_haptic->trig[1].trig_bst = AW869XX_TRIG2_BST_DISABLE;
}

static void aw869xx_trig3_param_init(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter\n", __func__);
	aw_haptic->trig[2].trig_level = AW869XX_TRIG3_DUAL_LEVEL;
	aw_haptic->trig[2].trig_polar = AW869XX_TRIG3_DUAL_POLAR;
	aw_haptic->trig[2].pos_enable = AW869XX_TRIG3_POS_DISABLE;
	aw_haptic->trig[2].pos_sequence = AW869XX_TRIG3_POS_SEQ;
	aw_haptic->trig[2].neg_enable = AW869XX_TRIG3_NEG_DISABLE;
	aw_haptic->trig[2].neg_sequence = AW869XX_TRIG3_NEG_SEQ;
	aw_haptic->trig[2].trig_brk = AW869XX_TRIG3_BRK_DISABLE;
	aw_haptic->trig[2].trig_bst = AW869XX_TRIG3_BST_DISABLE;
}

static void aw869xx_trig1_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[0].trig_level)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_MODE_LEVEL;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_MODE_EDGE;
	if (aw_haptic->trig[0].trig_polar)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_POLAR_NEG;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_POLAR_POS;
	if (aw_haptic->trig[0].pos_enable) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG1,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG1,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[0].neg_enable) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG4,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG4,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[0].pos_sequence) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG1,
			   AW869XX_BIT_TRG_SEQ_MASK,
			   aw_haptic->trig[0].pos_sequence);
	}
	if (aw_haptic->trig[0].neg_sequence) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG4,
			   AW869XX_BIT_TRG_SEQ_MASK,
			   aw_haptic->trig[0].neg_sequence);
	}
	if (aw_haptic->trig[0].trig_brk)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[0].trig_bst)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_BST_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG1_BST_DISABLE;
	i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG7,
		   (AW869XX_BIT_TRGCFG7_TRG1_MODE_MASK &
		    AW869XX_BIT_TRGCFG7_TRG1_POLAR_MASK &
		    AW869XX_BIT_TRGCFG7_TRG1_AUTO_BRK_MASK &
		    AW869XX_BIT_TRGCFG7_TRG1_BST_MASK),
		   trig_config);
}

static void aw869xx_trig2_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[1].trig_level)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_MODE_LEVEL;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_MODE_EDGE;
	if (aw_haptic->trig[1].trig_polar)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_POLAR_NEG;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_POLAR_POS;
	if (aw_haptic->trig[1].pos_enable) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG2,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG2,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[1].neg_enable) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG5,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG5,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[1].pos_sequence) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG2,
			   AW869XX_BIT_TRG_SEQ_MASK,
			   aw_haptic->trig[1].pos_sequence);
	}
	if (aw_haptic->trig[1].neg_sequence) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG5,
			   AW869XX_BIT_TRG_SEQ_MASK,
			   aw_haptic->trig[1].neg_sequence);
	}
	if (aw_haptic->trig[1].trig_brk)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[1].trig_bst)
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_BST_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG7_TRG2_BST_DISABLE;
	i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG7,
		   (AW869XX_BIT_TRGCFG7_TRG2_MODE_MASK &
		    AW869XX_BIT_TRGCFG7_TRG2_POLAR_MASK &
		    AW869XX_BIT_TRGCFG7_TRG2_AUTO_BRK_MASK &
		    AW869XX_BIT_TRGCFG7_TRG2_BST_MASK),
		   trig_config);
}

static void aw869xx_trig3_param_config(struct aw_haptic *aw_haptic)
{
	uint8_t trig_config = 0;

	if (aw_haptic->trig[2].trig_level)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_MODE_LEVEL;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_MODE_EDGE;
	if (aw_haptic->trig[2].trig_polar)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_POLAR_NEG;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_POLAR_POS;
	if (aw_haptic->trig[2].pos_enable) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG3,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG3,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[2].neg_enable) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG6,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG6,
			   AW869XX_BIT_TRG_ENABLE_MASK,
			   AW869XX_BIT_TRG_DISABLE);
	}
	if (aw_haptic->trig[2].pos_sequence) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG3,
			   AW869XX_BIT_TRG_SEQ_MASK,
			   aw_haptic->trig[2].pos_sequence);
	}
	if (aw_haptic->trig[2].neg_sequence) {
		i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG6,
			   AW869XX_BIT_TRG_SEQ_MASK,
			   aw_haptic->trig[2].neg_sequence);
	}
	if (aw_haptic->trig[2].trig_brk)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_DISABLE;
	if (aw_haptic->trig[2].trig_bst)
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_BST_ENABLE;
	else
		trig_config |= AW869XX_BIT_TRGCFG8_TRG3_BST_DISABLE;
	i2c_w_bits(aw_haptic, AW869XX_REG_TRGCFG8,
		   (AW869XX_BIT_TRGCFG8_TRG3_MODE_MASK &
		    AW869XX_BIT_TRGCFG8_TRG3_POLAR_MASK &
		    AW869XX_BIT_TRGCFG8_TRG3_AUTO_BRK_MASK &
		    AW869XX_BIT_TRGCFG8_TRG3_BST_MASK),
		   trig_config);
}

static void aw869xx_auto_bst_enable(struct aw_haptic *aw_haptic, uint8_t flag)
{
	aw_haptic->auto_boost = flag;
	if (flag) {
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
			   AW869XX_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		i2c_w_bits(aw_haptic, AW869XX_REG_PLAYCFG3,
			   AW869XX_BIT_PLAYCFG3_AUTO_BST_MASK,
			   AW869XX_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
}

static void aw869xx_interrupt_setup(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSINT, &reg_val, AW_I2C_BYTE_ONE);
	aw_dev_info("%s: reg SYSINT=0x%02X", __func__, reg_val);
	/* edge int mode */
	i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL7,
		   (AW869XX_BIT_SYSCTRL7_INT_MODE_MASK &
		    AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_MASK),
		   (AW869XX_BIT_SYSCTRL7_INT_MODE_EDGE |
		    AW869XX_BIT_SYSCTRL7_INT_EDGE_MODE_POS));
	/* int enable */
	i2c_w_bits(aw_haptic, AW869XX_REG_SYSINTM,
		   (AW869XX_BIT_SYSINTM_BST_SCPM_MASK &
		    AW869XX_BIT_SYSINTM_BST_OVPM_MASK &
		    AW869XX_BIT_SYSINTM_UVLM_MASK &
		    AW869XX_BIT_SYSINTM_OCDM_MASK &
		    AW869XX_BIT_SYSINTM_OTM_MASK),
		   (AW869XX_BIT_SYSINTM_BST_SCPM_ON |
		    AW869XX_BIT_SYSINTM_BST_OVPM_OFF |
		    AW869XX_BIT_SYSINTM_UVLM_ON |
		    AW869XX_BIT_SYSINTM_OCDM_ON |
		    AW869XX_BIT_SYSINTM_OTM_ON));
}

static int aw869xx_judge_rtp_going(struct aw_haptic *aw_haptic)
{
	uint8_t glb_state = 0;
	uint8_t rtp_state = 0;

	glb_state = aw869xx_get_glb_state(aw_haptic);
	if (glb_state == AW869XX_BIT_GLBRD5_STATE_RTP_GO) {
		rtp_state = 1;	/*is going on */
		aw_dev_info("%s: rtp_routine_on", __func__);
	}
	return rtp_state;
}

static void aw869xx_get_ram_data(struct aw_haptic *aw_haptic,
				    uint8_t *data, uint32_t size)
{
	i2c_r_bytes(aw_haptic, AW869XX_REG_RAMDATA, data, size);
}

static void aw869xx_get_first_wave_addr(struct aw_haptic *aw_haptic,
					uint8_t *wave_addr)
{
	uint8_t reg_array[3] = {0};

	i2c_r_bytes(aw_haptic, AW869XX_REG_RAMDATA, reg_array,
			    AW_I2C_BYTE_THREE);
	wave_addr[0] = reg_array[1];
	wave_addr[1] = reg_array[2];
}

static void aw869xx_get_wav_seq(struct aw_haptic *aw_haptic, uint8_t *seq,
				uint8_t len)
{
	aw_dev_dbg("%s: enter!\n", __func__);
	i2c_r_bytes(aw_haptic, AW869XX_REG_WAVCFG1, seq, len);
}

static size_t aw869xx_get_wav_loop(struct aw_haptic *aw_haptic, char *buf)
{
	uint8_t i = 0;
	uint8_t reg_val[4] = {0};
	size_t count = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_WAVCFG9, reg_val,
			    AW_I2C_BYTE_FOUR);
	for (i = 0; i < AW_SEQUENCER_LOOP_SIZE; i++) {
		aw_haptic->loop[i * 2 + 0] = (reg_val[i] >> 4) & 0x0F;
		aw_haptic->loop[i * 2 + 1] = (reg_val[i] >> 0) & 0x0F;
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 1,
				  aw_haptic->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 2,
				  aw_haptic->loop[i * 2 + 1]);
	}
	return count;
}

static void aw869xx_irq_clear(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSINT, &reg_val,
		    AW_I2C_BYTE_ONE);
	aw_dev_info("%s: reg SYSINT=0x%02X", __func__, reg_val);
}

static uint8_t aw869xx_get_prctmode(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_PWMCFG3, &reg_val,
		    AW_I2C_BYTE_ONE);
	reg_val >>= 7;
	return reg_val;
}

static int aw869xx_get_irq_state(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	int ret = -1;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSINT, &reg_val,
		    AW_I2C_BYTE_ONE);
	aw_dev_dbg("%s: reg SYSINT=0x%02X", __func__, reg_val);
	if (reg_val & AW869XX_BIT_SYSINT_BST_SCPI)
		aw_dev_err("%s: chip scp int error", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_BST_OVPI)
		aw_dev_err("%s: chip ov int error", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_UVLI)
		aw_dev_err("%s: chip uvlo int error", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_OCDI)
		aw_dev_err("%s: chip over current int error", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_OTI)
		aw_dev_err("%s: chip over temperature int error", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_DONEI)
		aw_dev_info("%s: chip playback done", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_FF_AFI)
		aw_dev_info("%s: aw_haptic rtp mode fifo almost full!", __func__);

	if (reg_val & AW869XX_BIT_SYSINT_FF_AEI)
		ret = 0;

	return ret;
}

static int aw869xx_read_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[2] = {0};
	uint32_t f0_reg = 0;

#ifdef AW_LRA_F0_DEFAULT
	/* lra_f0 */
	i2c_r_bytes(aw_haptic, AW869XX_REG_CONTRD14, reg_val,
		    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | (reg_val[1] << 0);
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_dev_err("%s: lra_f0 is error, f0_reg=0", __func__);
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW869XX_F0_FARMULA(f0_reg);
	aw_dev_info("%s: lra_f0=%d", __func__, aw_haptic->f0);
#else
	/* cont_f0 */
	i2c_r_bytes(aw_haptic, AW869XX_REG_CONTRD16, reg_val,
		    AW_I2C_BYTE_TWO);
	f0_reg = (reg_val[0] << 8) | (reg_val[1] << 0);
	if (!f0_reg) {
		aw_haptic->f0 = 0;
		aw_dev_err("%s: cont_f0 is error, f0_reg=0", __func__);
		return -ERANGE;
	}
	aw_haptic->f0 = (uint32_t)AW869XX_F0_FARMULA(f0_reg);
	aw_dev_info("%s: cont_f0=%d", __func__, aw_haptic->f0);
#endif
	return 0;
}

static int aw869xx_get_f0(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint8_t brk_en_default = 0;
	uint8_t cont_config[3] = {0};
	int drv_width = 0;
	int ret = 0;

	aw_haptic->f0 = aw_haptic->info.f0_pre;
	/* enter standby mode */
	aw869xx_stop(aw_haptic);
	/* f0 calibrate work mode */
	aw869xx_play_mode(aw_haptic, AW_CONT_MODE);
	/* enable f0 detect */
	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG1,
		   AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
		   AW869XX_BIT_CONTCFG1_F0_DET_ENABLE);
	/* cont config */
	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG6,
		   AW869XX_BIT_CONTCFG6_TRACK_EN_MASK,
		   AW869XX_BIT_CONTCFG6_TRACK_ENABLE);
	/* enable auto break */
	i2c_r_bytes(aw_haptic, AW869XX_REG_PLAYCFG3, &reg_val,
		    AW_I2C_BYTE_ONE);
	brk_en_default = 0x04 & reg_val;
	aw869xx_auto_brake_mode(aw_haptic, true);
	/* f0 driver level & time */
	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG6,
		   AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
		   aw_haptic->info.cont_drv1_lvl);
	cont_config[0] = aw_haptic->info.cont_drv2_lvl;
	cont_config[1] = aw_haptic->info.cont_drv1_time;
	cont_config[2] = aw_haptic->info.cont_drv2_time;
	i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG7, cont_config,
		    AW_I2C_BYTE_THREE);
	/* TRACK_MARGIN */
	if (!aw_haptic->info.cont_track_margin) {
		aw_dev_err("%s: aw_haptic->info.cont_track_margin = 0!", __func__);
	} else {
		i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG11,
			    &aw_haptic->info.cont_track_margin,
			    AW_I2C_BYTE_ONE);
	}
	/* DRV_WIDTH */
	if (!aw_haptic->info.f0_pre)
		return -ERANGE;
	drv_width = AW_DRV_WIDTH_FARMULA(aw_haptic->info.f0_pre,
					 aw_haptic->info.cont_brk_gain,
					 aw_haptic->info.cont_track_margin);
	if (drv_width < AW_DRV_WIDTH_MIN)
		drv_width = AW_DRV_WIDTH_MIN;
	else if (drv_width > AW_DRV_WIDTH_MAX)
		drv_width = AW_DRV_WIDTH_MAX;
	cont_config[0] = (uint8_t)drv_width;
	i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG3, &cont_config[0],
		    AW_I2C_BYTE_ONE);
	/* play go */
	aw869xx_play_go(aw_haptic, true);
	usleep_range(20000, 20500);
	aw869xx_wait_enter_standby(aw_haptic);
	ret = aw869xx_read_f0(aw_haptic);
	/* restore default config */
	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG1,
		   AW869XX_BIT_CONTCFG1_EN_F0_DET_MASK,
		   AW869XX_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	if (brk_en_default)
		aw869xx_auto_brake_mode(aw_haptic, true);
	else
		aw869xx_auto_brake_mode(aw_haptic, false);
	return ret;
}

static uint8_t aw869xx_rtp_get_fifo_afs(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSST, &reg_val,
		    AW_I2C_BYTE_ONE);
	reg_val &= AW869XX_BIT_SYSST_FF_AFS;
	reg_val >>= 3;
	return reg_val;
}

static uint8_t aw869xx_rtp_get_fifo_aes(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSST, &reg_val,
		    AW_I2C_BYTE_ONE);
	reg_val &= AW869XX_BIT_SYSST_FF_AES;
	reg_val >>= 4;
	return reg_val;
}

static uint8_t aw869xx_osc_read_status(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSST2, &reg_val,
		    AW_I2C_BYTE_ONE);
	reg_val &= AW869XX_BIT_SYSST2_FF_EMPTY;
	return reg_val;
}

static void aw869xx_get_lra_resistance(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t lra_code = 0;

	aw869xx_ram_init(aw_haptic, true);
	/* enter standby mode */
	aw869xx_stop(aw_haptic);
	usleep_range(2000, 2500);
	i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
		   AW869XX_BIT_SYSCTRL2_STANDBY_MASK,
		   AW869XX_BIT_SYSCTRL2_STANDBY_OFF);
	i2c_w_bits(aw_haptic, AW869XX_REG_DETCFG1,
		   AW869XX_BIT_DETCFG1_RL_OS_MASK,
		   AW869XX_BIT_DETCFG1_RL);
	i2c_w_bits(aw_haptic, AW869XX_REG_DETCFG2,
		   AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
		   AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(30000, 35000);
	i2c_r_bytes(aw_haptic, AW869XX_REG_DET_RL, &reg_val,
		    AW_I2C_BYTE_ONE);
	lra_code = (lra_code | reg_val) << 2;
	i2c_r_bytes(aw_haptic, AW869XX_REG_DET_LO, &reg_val,
		    AW_I2C_BYTE_ONE);
	lra_code = lra_code | (reg_val & 0x03);
	aw_haptic->lra = AW869XX_LRA_FORMULA(lra_code);
	aw869xx_ram_init(aw_haptic, false);
}

static void aw869xx_set_repeat_seq(struct aw_haptic *aw_haptic, uint8_t seq)
{
	aw869xx_set_wav_seq(aw_haptic, 0x00, seq);
	aw869xx_set_wav_loop(aw_haptic, 0x00, AW869XX_BIT_WAVLOOP_INIFINITELY);
}

static void aw869xx_get_vbat(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t vbat_code = 0;
	uint32_t cont = 2000;

	aw869xx_stop(aw_haptic);
	aw869xx_ram_init(aw_haptic, true);
	i2c_w_bits(aw_haptic, AW869XX_REG_DETCFG2,
		   AW869XX_BIT_DETCFG2_VBAT_GO_MASK,
		   AW869XX_BIT_DETCFG2_VABT_GO_ON);

	while (cont--) {
		i2c_r_bytes(aw_haptic, AW869XX_REG_DETCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x02) == 0 || cont == 0)
			break;
	}

	i2c_r_bytes(aw_haptic, AW869XX_REG_DET_VBAT, &reg_val,
		    AW_I2C_BYTE_ONE);
	vbat_code = (vbat_code | reg_val) << 2;
	i2c_r_bytes(aw_haptic, AW869XX_REG_DET_LO, &reg_val,
		    AW_I2C_BYTE_ONE);
	vbat_code = vbat_code | ((reg_val & 0x30) >> 4);
	aw_haptic->vbat = AW869XX_VBAT_FORMULA(vbat_code);
	if (aw_haptic->vbat > AW_VBAT_MAX) {
		aw_haptic->vbat = AW_VBAT_MAX;
		aw_dev_info("%s: vbat max limit = %d", __func__, aw_haptic->vbat);
	}
	if (aw_haptic->vbat < AW_VBAT_MIN) {
		aw_haptic->vbat = AW_VBAT_MIN;
		aw_dev_info("%s: vbat min limit = %d", __func__, aw_haptic->vbat);
	}
	aw_dev_info("%s: aw_haptic->vbat=%dmV, vbat_code=0x%02X",
		    __func__, aw_haptic->vbat, vbat_code);
	aw869xx_ram_init(aw_haptic, false);
}

static ssize_t aw869xx_get_reg(struct aw_haptic *aw_haptic, ssize_t len,
			       char *buf)
{
	uint8_t i = 0;
	uint8_t reg_array[AW869XX_REG_D2SCFG1 + 1] = {0};

	i2c_r_bytes(aw_haptic, AW869XX_REG_ID, reg_array,
		    AW869XX_REG_RTPDATA);
	i2c_r_bytes(aw_haptic, (AW869XX_REG_RTPDATA + 1),
		    &reg_array[AW869XX_REG_RTPDATA + 1],
		    (AW869XX_REG_RAMDATA - AW869XX_REG_RTPDATA - 1));
	i2c_r_bytes(aw_haptic, (AW869XX_REG_RAMDATA + 1),
		    &reg_array[AW869XX_REG_RAMDATA + 1],
		    (AW869XX_REG_D2SCFG1 - AW869XX_REG_RAMDATA));
	for (i = 0; i <= AW869XX_REG_D2SCFG1; i++)
		if ((i != AW869XX_REG_RTPDATA) && (i != AW869XX_REG_RAMDATA))
			len += snprintf(buf + len, PAGE_SIZE - len,
					"reg:0x%02X=0x%02X\n", i, reg_array[i]);
	return len;
}

static void aw869xx_offset_cali(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t cnt = 2000;

	aw869xx_ram_init(aw_haptic, true);

	i2c_w_bits(aw_haptic, AW869XX_REG_DETCFG2,
		   AW869XX_BIT_DETCFG2_DIAG_GO_MASK,
		   AW869XX_BIT_DETCFG2_DIAG_GO_ON);
	while (cnt--) {
		i2c_r_bytes(aw_haptic, AW869XX_REG_DETCFG2, &reg_val,
			    AW_I2C_BYTE_ONE);
		if ((reg_val & 0x01) == 0) {
			aw869xx_ram_init(aw_haptic, false);
			return;
		}
	}
	aw_dev_err("%s: calibration offset failed!", __func__);
	aw869xx_ram_init(aw_haptic, false);
}

static void aw869xx_trig_init(struct aw_haptic *aw_haptic)
{
	aw_dev_info("%s: enter!", __func__);
	/* i2s config */
	if (aw_haptic->info.is_enabled_i2s && aw_haptic->i2s_config) {
		aw_dev_info("%s: i2s is enabled!", __func__);
		aw869xx_i2s_init(aw_haptic);
	} else {
		aw869xx_trig2_param_init(aw_haptic);
		aw869xx_trig3_param_init(aw_haptic);
		aw869xx_trig2_param_config(aw_haptic);
		aw869xx_trig3_param_config(aw_haptic);
	}
	/* one wire config */
	if (aw_haptic->info.is_enabled_one_wire) {
		aw_dev_info("%s: one wire is enabled!", __func__);
		aw869xx_one_wire_init(aw_haptic);
	} else {
		aw869xx_trig1_param_init(aw_haptic);
		aw869xx_trig1_param_config(aw_haptic);
	}
}

static int aw869xx_container_update(struct aw_haptic *aw_haptic,
				    struct aw_haptic_container *awinic_cont)
{
	uint8_t reg_val[3] = {0};
	uint8_t fifo_adr[4] = {0};
	uint32_t temp = 0;
	uint32_t shift = 0;
	int i = 0;
	int ret = 0;
	int len = 0;

	mutex_lock(&aw_haptic->lock);
	aw_haptic->ram.baseaddr_shift = 2;
	aw_haptic->ram.ram_shift = 4;
	/* RAMINIT Enable */
	aw869xx_ram_init(aw_haptic, true);
	/* Enter standby mode */
	aw869xx_stop(aw_haptic);
	/* base addr */
	shift = aw_haptic->ram.baseaddr_shift;
	aw_haptic->ram.base_addr = (uint32_t)((awinic_cont->data[0 + shift] << 8) |
				   (awinic_cont->data[1 + shift]));
	/* fifo_ael */
	fifo_adr[0] = (uint8_t)AW869XX_FIFO_AE_ADDR_L(aw_haptic->ram.base_addr);
	/* fifo_afl */
	fifo_adr[1] = (uint8_t)AW869XX_FIFO_AF_ADDR_L(aw_haptic->ram.base_addr);
	/* fifo_aeh */
	fifo_adr[2] = (uint8_t)AW869XX_FIFO_AE_ADDR_H(aw_haptic->ram.base_addr);
	/* fifo_afh */
	fifo_adr[3] = (uint8_t)AW869XX_FIFO_AF_ADDR_H(aw_haptic->ram.base_addr);

	aw_dev_info("%s: base_addr = %d", __func__, aw_haptic->ram.base_addr);
	i2c_w_bytes(aw_haptic, AW869XX_REG_RTPCFG1, &awinic_cont->data[shift],
		    AW_I2C_BYTE_TWO);
	i2c_w_bits(aw_haptic, AW869XX_REG_RTPCFG3,
		   (AW869XX_BIT_RTPCFG3_FIFO_AEH_MASK &
		    AW869XX_BIT_RTPCFG3_FIFO_AFH_MASK),
		   (fifo_adr[2] | fifo_adr[3]));
	i2c_w_bytes(aw_haptic, AW869XX_REG_RTPCFG4, fifo_adr,
		    AW_I2C_BYTE_TWO);
	i2c_r_bytes(aw_haptic, AW869XX_REG_RTPCFG3, reg_val,
		    AW_I2C_BYTE_THREE);
	temp = ((reg_val[0] & 0x0f) << 24) | ((reg_val[0] & 0xf0) << 4);
	temp = temp | reg_val[1];
	aw_dev_info("%s: almost_empty_threshold = %d", __func__, (uint16_t)temp);
	temp = temp | (reg_val[2] << 16);
	aw_dev_info("%s: almost_full_threshold = %d", __func__, temp >> 16);
	/* ram */
	shift = aw_haptic->ram.baseaddr_shift;
	i2c_w_bytes(aw_haptic, AW869XX_REG_RAMADDRH,
		    &awinic_cont->data[shift], AW_I2C_BYTE_TWO);
	i = aw_haptic->ram.ram_shift;
	while (i < awinic_cont->len) {
		if ((awinic_cont->len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = awinic_cont->len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;
		i2c_w_bytes(aw_haptic, AW869XX_REG_RAMDATA,
			    &awinic_cont->data[i], len);
		i += len;
	}
	/* RAMINIT Disable */
	aw869xx_ram_init(aw_haptic, false);
	mutex_unlock(&aw_haptic->lock);
	return ret;
}

static unsigned long aw869xx_get_theory_time(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val = 0;
	uint32_t fre_val = 0;
	uint64_t theory_time = 0;

	i2c_r_bytes(aw_haptic, AW869XX_REG_SYSCTRL2, &reg_val,
		    AW_I2C_BYTE_ONE);
	fre_val = (reg_val & 0x03) >> 0;

	if (fre_val == 2 || fre_val == 3)
		theory_time = ((uint64_t)aw_haptic->rtp_len / 12000) * 1000000;	/*12K */
	if (fre_val == 0)
		theory_time = ((uint64_t)aw_haptic->rtp_len / 24000) * 1000000;	/*24K */
	if (fre_val == 1)
		theory_time = ((uint64_t)aw_haptic->rtp_len / 48000) * 1000000;	/*48K */

	aw_dev_info("%s: microsecond:%llu theory_time = %llu", __func__,
		    aw_haptic->microsecond, theory_time);

	return theory_time;
}

static void aw869xx_haptic_value_init(struct aw_haptic *aw_haptic)
{
	aw_haptic->info.f0_pre = AW_HAPTIC_F0_PRE;
	aw_haptic->info.f0_cali_percent = AW_HAPTIC_F0_CALI_PERCEN;
	aw_haptic->info.cont_drv1_lvl = AW869XX_CONT_DRV1_LVL;
	aw_haptic->info.cont_lra_vrms = AW869XX_CONT_LRA_VRMS;
	aw_haptic->info.cont_drv1_time = AW869XX_CONT_DRV1_TIME;
	aw_haptic->info.cont_drv2_time = AW869XX_CONT_DRV2_TIME;
	aw_haptic->info.cont_wait_num = AW869XX_CONT_WAIT_NUM;
	aw_haptic->info.cont_brk_time = AW869XX_CONT_BRK_TIME;
	aw_haptic->info.cont_track_margin = AW869XX_CONT_TRACK_MARGIN;
	aw_haptic->info.cont_tset = AW869XX_CONT_TEST;
	aw_haptic->info.brk_bst_md = AW869XX_BRK_BST_MD;
	aw_haptic->info.cont_bemf_set = AW869XX_CONT_BEMF_SET;
	aw_haptic->info.cont_bst_brk_gain = AW869XX_CONT_BST_BRK_GAIN;
	aw_haptic->info.cont_brk_gain = AW869XX_CONT_BRK_GAIN;

	aw_haptic->info.max_bst_vol = AW869XX_MAX_BST_VOL;
	aw_haptic->info.d2s_gain = AW869XX_D2S_GAIN;
	aw_haptic->info.bst_vol_default = AW869XX_BST_VOL_DEFAULT;
	aw_haptic->info.bst_vol_ram = AW869XX_BST_VOL_RAM;
	aw_haptic->info.bst_vol_rtp = AW869XX_BST_VOL_RTP;

	aw_haptic->info.sine_array[0] = AW869XX_SINE_ARRAY1;
	aw_haptic->info.sine_array[1] = AW869XX_SINE_ARRAY2;
	aw_haptic->info.sine_array[2] = AW869XX_SINE_ARRAY3;
	aw_haptic->info.sine_array[3] = AW869XX_SINE_ARRAY4;
	aw_haptic->info.bstcfg[0] = AW869XX_BSTCFG1;
	aw_haptic->info.bstcfg[1] = AW869XX_BSTCFG2;
	aw_haptic->info.bstcfg[2] = AW869XX_BSTCFG3;
	aw_haptic->info.bstcfg[3] = AW869XX_BSTCFG4;
	aw_haptic->info.bstcfg[4] = AW869XX_BSTCFG5;
}

static void aw869xx_misc_para_init(struct aw_haptic *aw_haptic)
{
	uint8_t reg_val[8] = {0};
	uint32_t drv2_lvl = 0;

	/* Cont drv2 lvl */
	drv2_lvl = AW869XX_DRV2_LVL_FARMULA(aw_haptic->info.f0_pre,
					    aw_haptic->info.cont_lra_vrms);
	if (drv2_lvl > AW_DRV2_LVL_MAX)
		aw_haptic->info.cont_drv2_lvl = AW_DRV2_LVL_MAX;
	else
		aw_haptic->info.cont_drv2_lvl = (uint8_t)drv2_lvl;
	/* Get vmax */
	if (aw_haptic->info.bst_vol_default > 0)
		aw_haptic->vmax = aw_haptic->info.bst_vol_default;
	/* Get gain */
	i2c_r_bytes(aw_haptic, AW869XX_REG_PLAYCFG2, reg_val,
		    AW_I2C_BYTE_ONE);
	aw_haptic->gain = reg_val[0];
	/* Get wave_seq */
	i2c_r_bytes(aw_haptic, AW869XX_REG_WAVCFG1, reg_val,
		    AW_I2C_BYTE_EIGHT);
	aw_haptic->index = reg_val[0];
	memcpy(aw_haptic->seq, reg_val, AW_SEQUENCER_SIZE);
	i2c_w_bytes(aw_haptic, AW869XX_REG_BSTCFG1,
		    aw_haptic->info.bstcfg, AW_I2C_BYTE_FIVE);
	i2c_w_bytes(aw_haptic, AW869XX_REG_SYSCTRL3,
		    aw_haptic->info.sine_array, AW_I2C_BYTE_FOUR);
	/* Set gain_bypass */
	i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL7,
		   AW869XX_BIT_SYSCTRL7_GAIN_BYPASS_MASK,
		   AW869XX_BIT_SYSCTRL7_GAIN_CHANGEABLE);

	/* brk_bst_md */
	if (!aw_haptic->info.brk_bst_md)
		aw_dev_err("%s: aw_haptic->info.brk_bst_md = 0!", __func__);
	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG1,
		   AW869XX_BIT_CONTCFG1_BRK_BST_MD_MASK,
		   aw_haptic->info.brk_bst_md << 1);
	/* d2s_gain */
	if (!aw_haptic->info.d2s_gain)
		aw_dev_err("%s: aw_haptic->info.d2s_gain = 0!", __func__);
	i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL7,
		   AW869XX_BIT_SYSCTRL7_D2S_GAIN_MASK,
		   aw_haptic->info.d2s_gain);
	/* cont_tset */
	if (!aw_haptic->info.cont_tset)
		aw_dev_err("%s: aw_haptic->info.cont_tset = 0!", __func__);
	/* cont_bemf_set */
	if (!aw_haptic->info.cont_bemf_set)
		aw_dev_err("%s: aw_haptic->info.cont_bemf_set = 0!", __func__);
	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG13,
		   (AW869XX_BIT_CONTCFG13_TSET_MASK &
		    AW869XX_BIT_CONTCFG13_BEME_SET_MASK),
		   ((aw_haptic->info.cont_tset << 4) |
		    aw_haptic->info.cont_bemf_set));
	/* cont_brk_time */
	if (!aw_haptic->info.cont_brk_time)
		aw_dev_err("%s: aw_haptic->info.cont_brk_time = 0!", __func__);
	i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG10,
		    &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	/* cont_bst_brk_gain */
	if (!aw_haptic->info.cont_bst_brk_gain)
		aw_dev_err("%s: aw_haptic->info.cont_bst_brk_gain = 0!", __func__);
	/* cont_brk_gain */
	if (!aw_haptic->info.cont_brk_gain)
		aw_dev_err("%s: aw_haptic->info.cont_brk_gain = 0!", __func__);
	i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG5,
		   (AW869XX_BIT_CONTCFG5_BST_BRK_GAIN_MASK &
		    AW869XX_BIT_CONTCFG5_BRK_GAIN_MASK),
		   ((aw_haptic->info.cont_bst_brk_gain << 4) |
		    aw_haptic->info.cont_brk_gain));
	/* i2s enbale */
	if (aw_haptic->info.is_enabled_i2s && aw_haptic->i2s_config) {
		aw_dev_info("%s: i2s enabled!", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
			   AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
			   AW869XX_BIT_SYSCTRL2_I2S_PIN_I2S);
	} else {
		aw_dev_info("%s: i2s disabled!", __func__);
		i2c_w_bits(aw_haptic, AW869XX_REG_SYSCTRL2,
			   AW869XX_BIT_SYSCTRL2_I2S_PIN_MASK,
			   AW869XX_BIT_SYSCTRL2_I2S_PIN_TRIG);
	}
	/* set BST_ADJ */
	i2c_w_bits(aw_haptic, AW869XX_REG_BSTCFG5,
		   AW869XX_BIT_BSTCFG5_BST_ADJ_MASK,
		   AW869XX_BIT_BSTCFG5_BST_ADJ_LOW);
	aw869xx_protect_config(aw_haptic, AW869XX_BIT_PWMCFG4_PRTIME_DEFAULT_VALUE,
			       AW869XX_BIT_PWMCFG3_PRLVL_DEFAULT_VALUE);
}

static ssize_t aw869xx_cont_wait_num_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_wait_num = 0x%02X\n",
			aw_haptic->info.cont_wait_num);
	return len;
}

static ssize_t aw869xx_cont_wait_num_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	uint8_t val = 0;
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtou8(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw_haptic->info.cont_wait_num = val;
	i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG4, &val,
		    AW_I2C_BYTE_ONE);

	return count;
}

static ssize_t aw869xx_cont_drv_lvl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw_haptic->info.cont_drv1_lvl,
			aw_haptic->info.cont_drv2_lvl);
	return len;
}

static ssize_t aw869xx_cont_drv_lvl_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	uint32_t databuf[2] = { 0, 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw_haptic->info.cont_drv1_lvl = databuf[0];
		aw_haptic->info.cont_drv2_lvl = databuf[1];
		i2c_w_bits(aw_haptic, AW869XX_REG_CONTCFG6,
			   AW869XX_BIT_CONTCFG6_DRV1_LVL_MASK,
			   aw_haptic->info.cont_drv1_lvl);
		i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG7,
			    &aw_haptic->info.cont_drv2_lvl, AW_I2C_BYTE_ONE);
	}
	return count;
}

static ssize_t aw869xx_cont_drv_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw_haptic->info.cont_drv1_time,
			aw_haptic->info.cont_drv2_time);
	return len;
}

static ssize_t aw869xx_cont_drv_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	uint8_t cont_time[2] = {0};
	uint32_t databuf[2] = { 0, 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		cont_time[0] = aw_haptic->info.cont_drv1_time = databuf[0];
		cont_time[1] = aw_haptic->info.cont_drv2_time = databuf[1];
		i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG8, cont_time,
			    AW_I2C_BYTE_TWO);
	}
	return count;
}

static ssize_t aw869xx_cont_brk_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_brk_time = 0x%02X\n",
			aw_haptic->info.cont_brk_time);
	return len;
}

static ssize_t aw869xx_cont_brk_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int rc = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	rc = kstrtou8(buf, 0, &aw_haptic->info.cont_brk_time);
	if (rc < 0)
		return rc;
	i2c_w_bytes(aw_haptic, AW869XX_REG_CONTCFG10,
		    &aw_haptic->info.cont_brk_time, AW_I2C_BYTE_ONE);
	return count;
}

static ssize_t aw869xx_trig_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	ssize_t len = 0;
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	for (i = 0; i < AW_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: trig_level=%d, trig_polar=%d, pos_enable=%d, pos_sequence=%d, neg_enable=%d, neg_sequence=%d trig_brk=%d, trig_bst=%d\n",
				i + 1,
				aw_haptic->trig[i].trig_level,
				aw_haptic->trig[i].trig_polar,
				aw_haptic->trig[i].pos_enable,
				aw_haptic->trig[i].pos_sequence,
				aw_haptic->trig[i].neg_enable,
				aw_haptic->trig[i].neg_sequence,
				aw_haptic->trig[i].trig_brk,
				aw_haptic->trig[i].trig_bst);
	}
	return len;
}

static ssize_t aw869xx_trig_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	uint32_t databuf[9] = { 0 };
	cdev_t *cdev = dev_get_drvdata(dev);
	struct aw_haptic *aw_haptic = container_of(cdev, struct aw_haptic,
						   vib_dev);

	if (sscanf(buf, "%u %u %u %u %u %u %u %u %u", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5],
		   &databuf[6], &databuf[7], &databuf[8]) == 9) {
		aw_dev_info("%s: %d, %d, %d, %d, %d, %d, %d, %d, %d",
			    databuf[0], databuf[1], databuf[2], databuf[3],
			    databuf[4], databuf[5], databuf[6], databuf[7],
			    databuf[8]);
		if (databuf[0] < 1 || databuf[0] > 3) {
			aw_dev_info("%s: input trig_num out of range!", __func__);
			return count;
		}
		if (databuf[0] == 1 && aw_haptic->info.is_enabled_one_wire) {
			aw_dev_info("%s: trig1 pin used for one wire!", __func__);
			return count;
		}

		if ((databuf[0] == 2 || databuf[0] == 3) &&
		     aw_haptic->info.is_enabled_i2s && aw_haptic->i2s_config) {
			aw_dev_info("%s: trig2 and trig3 pin used for i2s!", __func__);
			return count;
		}
		if (!aw_haptic->ram_init) {
			aw_dev_err("%s: ram init failed, not allow to play!", __func__);
			return count;
		}
		if (databuf[4] > aw_haptic->ram.ram_num ||
		    databuf[6] > aw_haptic->ram.ram_num) {
			aw_dev_err("%s: input seq value out of range!", __func__);
			return count;
		}
		databuf[0] -= 1;

		aw_haptic->trig[databuf[0]].trig_level = databuf[1];
		aw_haptic->trig[databuf[0]].trig_polar = databuf[2];
		aw_haptic->trig[databuf[0]].pos_enable = databuf[3];
		aw_haptic->trig[databuf[0]].pos_sequence = databuf[4];
		aw_haptic->trig[databuf[0]].neg_enable = databuf[5];
		aw_haptic->trig[databuf[0]].neg_sequence = databuf[6];
		aw_haptic->trig[databuf[0]].trig_brk = databuf[7];
		aw_haptic->trig[databuf[0]].trig_bst = databuf[8];
		mutex_lock(&aw_haptic->lock);
		switch (databuf[0]) {
		case 0:
			aw869xx_trig1_param_config(aw_haptic);
			break;
		case 1:
			aw869xx_trig2_param_config(aw_haptic);
			break;
		case 2:
			aw869xx_trig3_param_config(aw_haptic);
			break;
		}
		mutex_unlock(&aw_haptic->lock);
	}
	return count;
}

static DEVICE_ATTR(cont_wait_num, S_IWUSR | S_IRUGO, aw869xx_cont_wait_num_show,
		   aw869xx_cont_wait_num_store);
static DEVICE_ATTR(cont_drv_lvl, S_IWUSR | S_IRUGO, aw869xx_cont_drv_lvl_show,
		   aw869xx_cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, S_IWUSR | S_IRUGO, aw869xx_cont_drv_time_show,
		   aw869xx_cont_drv_time_store);
static DEVICE_ATTR(cont_brk_time, S_IWUSR | S_IRUGO, aw869xx_cont_brk_time_show,
		   aw869xx_cont_brk_time_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw869xx_trig_show,
		   aw869xx_trig_store);

static struct attribute *aw869xx_vibrator_attributes[] = {
	&dev_attr_trig.attr,
	&dev_attr_cont_wait_num.attr,
	&dev_attr_cont_drv_lvl.attr,
	&dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,
	NULL
};

static struct attribute_group aw869xx_vibrator_attribute_group = {
	.attrs = aw869xx_vibrator_attributes
};

static int aw869xx_creat_node(struct aw_haptic *aw_haptic)
{
	int ret = 0;

	ret = sysfs_create_group(&aw_haptic->vib_dev.dev->kobj,
				 &aw869xx_vibrator_attribute_group);
	if (ret < 0)
		aw_dev_err("%s: error creating sysfs attr files", __func__);
	return ret;
}

struct aw_haptic_func aw869xx_func_list = {
	.play_stop = aw869xx_stop,
	.ram_init = aw869xx_ram_init,
	.get_vbat = aw869xx_get_vbat,
	.creat_node = aw869xx_creat_node,
	.get_f0 = aw869xx_get_f0,
	.cont_config = aw869xx_cont_config,
	.offset_cali = aw869xx_offset_cali,
	.check_qualify = aw869xx_check_qualify,
	.get_irq_state = aw869xx_get_irq_state,
	.juge_rtp_going = aw869xx_judge_rtp_going,
	.set_bst_peak_cur = aw869xx_set_bst_peak_cur,
	.get_theory_time = aw869xx_get_theory_time,
	.get_lra_resistance = aw869xx_get_lra_resistance,
	.set_pwm = aw869xx_set_pwm,
	.play_mode = aw869xx_play_mode,
	.set_bst_vol = aw869xx_set_bst_vol,
	.interrupt_setup = aw869xx_interrupt_setup,
	.set_repeat_seq = aw869xx_set_repeat_seq,
	.auto_bst_enable = aw869xx_auto_bst_enable,
	.vbat_mode_config = aw869xx_vbat_mode_config,
	.set_wav_seq = aw869xx_set_wav_seq,
	.set_wav_loop = aw869xx_set_wav_loop,
	.set_ram_addr = aw869xx_set_ram_addr,
	.set_rtp_data = aw869xx_set_rtp_data,
	.container_update = aw869xx_container_update,
	.protect_config = aw869xx_protect_config,
	.trig_init = aw869xx_trig_init,
	.irq_clear = aw869xx_irq_clear,
	.get_wav_loop = aw869xx_get_wav_loop,
	.play_go = aw869xx_play_go,
	.misc_para_init = aw869xx_misc_para_init,
	.set_rtp_aei = aw869xx_set_rtp_aei,
	.set_gain = aw869xx_set_gain,
	.upload_lra = aw869xx_upload_lra,
	.bst_mode_config = aw869xx_bst_mode_config,
	.get_reg = aw869xx_get_reg,
	.get_prctmode = aw869xx_get_prctmode,
	.get_ram_data = aw869xx_get_ram_data,
	.get_first_wave_addr = aw869xx_get_first_wave_addr,
	.get_glb_state = aw869xx_get_glb_state,
	.get_chip_state = aw869xx_get_chip_state,
	.read_irq_state = aw869xx_read_irq_state,
	.get_osc_status = aw869xx_osc_read_status,
	.rtp_get_fifo_afs = aw869xx_rtp_get_fifo_afs,
	.rtp_get_fifo_aes = aw869xx_rtp_get_fifo_aes,
	.get_wav_seq = aw869xx_get_wav_seq,
	.dump_rtp_regs = aw869xx_dump_rtp_regs,
	.haptic_value_init = aw869xx_haptic_value_init,
};
