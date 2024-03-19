#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"
#include "imx709tele_vcm_23021.h"

#define IMX709TELE_VCM_SLAVE_ADDRESS 0x18
#define DW9718AF_INIT_DELAY_US   1000

struct regval_list {
	u16 reg_num;
	u8 value;
	u8 delay;
};

static struct regval_list dw9718af_init_regs[] = {
	{0x02, 0x01, 0},
	{0x02, 0x00, 1},
	{0x02, 0x02, 0},
	{0x06, 0x40, 0},
	{0x07, 0x5F, 0},
};

static bool selective_write_vcm(struct subdrv_ctx *ctx, struct regval_list *vals, u32 len)
{
    unsigned int i = 0;
    int ret = -1;

    for (i = 0; i < len; i++) {
        ret = adaptor_i2c_wr_u8_u8(ctx->i2c_client, IMX709TELE_VCM_SLAVE_ADDRESS >> 1,
                            vals[i].reg_num, vals[i].value);

        if (ret < 0) {
            pr_err("reinit vcm fail");
        }

        usleep_range(vals[i].delay * DW9718AF_INIT_DELAY_US,
					    vals[i].delay * DW9718AF_INIT_DELAY_US + 1000);
    }

    return ret;
}


unsigned int write_imx709tele_vcm_23021_init(struct subdrv_ctx *ctx)
{
    if (!selective_write_vcm(ctx, dw9718af_init_regs,
                            ARRAY_SIZE(dw9718af_init_regs))) {
        return -1;
    }

    return 0;
}

