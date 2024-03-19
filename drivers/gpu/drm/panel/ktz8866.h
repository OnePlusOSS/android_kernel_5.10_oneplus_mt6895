/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _ktz8866_SW_H_
#define _ktz8866_SW_H_

#define KTZ8866_ID	(0x01)
#define AW3750X_ID	(0x02)

extern int g_lcd_bias_id;
extern struct i2c_client *lcd_bl_i2c0_aw3750x;

extern struct i2c_client *lcd_bl_i2c0_client;
extern struct i2c_client *lcd_bl_i2c3_client;

extern int lcd_bl_i2c_write_dual(struct i2c_client *i2c_client0, unsigned char i2c_client0_addr, unsigned char i2c_client0_value,
		struct i2c_client *i2c_client1, unsigned char i2c_client1_addr, unsigned char i2c_client1_value);
extern int lcd_bl_i2c_read_dual(struct i2c_client *i2c_client0, unsigned char i2c_client0_addr, unsigned char *i2c_client0_buf,
		struct i2c_client *i2c_client1, unsigned char i2c_client1_addr, unsigned char *i2c_client1_buf);
extern int lcd_set_bias(int enable);
extern int lcd_set_bl_bias_reg(struct device *pdev, int enable);
#endif
