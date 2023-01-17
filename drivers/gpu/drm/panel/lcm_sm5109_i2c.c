#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#define LCM_I2C_ID_NAME "I2C_LCD_BIAS"
struct i2c_client *_lcm_i2c_client;
int _lcm_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);
int _lcm_i2c_remove(struct i2c_client *client);
struct _lcm_i2c_dev {
	struct i2c_client *client;
};

const struct of_device_id _lcm_i2c_of_match[] = {
	{
	    .compatible = "mediatek,I2C_LCD_BIAS",
	},
	{},
};

const struct i2c_device_id _lcm_i2c_id[] = { { LCM_I2C_ID_NAME, 0 },
						    {} };

struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect		   = _lcm_i2c_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name = LCM_I2C_ID_NAME,
		.of_match_table = _lcm_i2c_of_match,
	},
};

int _lcm_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	pr_info("[LCM][I2C] NT: info==>name=%s addr=0x%x\n", client->name,
		 client->addr);
	_lcm_i2c_client = client;
	return 0;
}

int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_debug("[LCM][I2C] %s\n", __func__);
	_lcm_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _lcm_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_info("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("[LCM][ERROR] _lcm_i2c write data fail !!\n");
    pr_info(" _lcm_i2c_write_bytes %c %c\n",write_data[0],write_data[1]);
	return ret;
}
EXPORT_SYMBOL(_lcm_i2c_write_bytes);
module_i2c_driver(_lcm_i2c_driver);
MODULE_AUTHOR("Elon Hsu <elon.hsu@mediatek.com>");
MODULE_DESCRIPTION("jdi r66451 CMD AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");