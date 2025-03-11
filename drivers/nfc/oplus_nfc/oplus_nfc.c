#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/soc/qcom/smem.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <soc/oplus/system/oplus_project.h>
#include <linux/io.h>
#include <stdbool.h>

#include "oplus_nfc.h"

#define NFC_CHIPSET_VERSION (0x1)
#define MIXED_CHIPSET    "mixed-chipset"
#define MAX_ID_COUNT     5

struct id_entry {
    u32 key;
    const char *chipset;
    const char *manifest_path;
    const char *feature_path;
};

static char current_chipset[32];
static bool support_nfc = false;

bool is_nfc_support()
{
	return support_nfc;
}

bool is_support_chip(chip_type chip)
{
	bool ret = false;
	const char* target_chipset;

	if (!support_nfc)
	{
		pr_err("%s, nfc not supported, or oplus_nfc has not started", __func__);
		return false;
	}

	switch(chip) {
		case NQ310:
			target_chipset = "NQ310|NQ330|PN557";
			break;
		case NQ330:
			target_chipset = "NQ330";
			break;
		case SN100T:
			target_chipset = "SN100T";
			break;
		case SN100F:
			target_chipset = "SN100F";
			break;
		case SN110T:
			target_chipset = "SN100T|SN110T";
			break;
		case ST21H:
			target_chipset = "ST21H|ST54H";
			break;
		case ST54H:
			target_chipset = "ST54H";
			break;
		default:
			target_chipset = "UNKNOWN";
			break;
	}

	if (strstr(target_chipset, current_chipset) != NULL)
	{
		ret = true;
	}

	pr_err("oplus_nfc target_chipset = %s, current_chipset = %s \n", target_chipset, current_chipset);
	return ret;
}
EXPORT_SYMBOL(is_support_chip);

static int nfc_read_func(struct seq_file *s, void *v)
{
	void *p = s->private;

	switch((unsigned long)(p)) {
	case NFC_CHIPSET_VERSION:
		seq_printf(s, "%s", current_chipset);
		break;
	default:
		seq_printf(s, "not support\n");
		break;
	}

	return 0;
}

static int nfc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nfc_read_func, PDE_DATA(inode));
}

static const struct proc_ops nfc_info_fops = {
	.proc_open  = nfc_open,
	.proc_read  = seq_read,
	.proc_release = single_release,
};

static int single_nfc_probe(struct platform_device *pdev)
{
	struct device* dev;
	unsigned int project = 0;
	int operator = 0;
	int readRet = -1;
	char prop_project[32];
	char prop_project_operator[32];
	const char *chipset_node;
	struct proc_dir_entry *p_entry;
	static struct proc_dir_entry *nfc_info = NULL;

	pr_err("enter %s", __func__);
        memset(prop_project, 0, sizeof(prop_project));
        memset(prop_project_operator, 0, sizeof(prop_project_operator));

	dev = &pdev->dev;
	if (!dev)
	{
		pr_err("%s, no device", __func__);
		goto error_init;
	}
	project = get_project();
	operator = get_Operator_Version();

	//project name consists of 5-symbol
	//project contains letters is big then 0x10000 == 65536
	if (project > 0x10000)
                snprintf(prop_project, sizeof(prop_project) , "chipset-%X", project);
	else
                snprintf(prop_project, sizeof(prop_project) , "chipset-%u", project);

	/*
	 *fixed me
	 *if operator is valid
	 *dts still uses by prop_project? not by prop_project_operator
	 */
	if (operator >= 0) {
                snprintf(prop_project_operator, sizeof(prop_project_operator) , "%s-%d", prop_project, operator);
	        pr_err("%s, prop_project_operator to be read = %s", __func__, prop_project_operator);
	        readRet = of_property_read_string(dev->of_node, prop_project_operator, &chipset_node);
	}

	if (readRet != 0){
                pr_err("%s, prop_project to be read = %s", __func__, prop_project);
                readRet = of_property_read_string(dev->of_node, prop_project, &chipset_node);
	}

	if (readRet != 0)
	{
		snprintf(current_chipset, sizeof(current_chipset) , "NULL");
	} else
	{
		pr_err("%s, get chipset_node content = %s", __func__, chipset_node);
		strncpy(current_chipset, chipset_node, sizeof(current_chipset) - 1);
		support_nfc = true;
	}

	nfc_info = proc_mkdir("oplus_nfc", NULL);
	if (!nfc_info)
	{
		pr_err("%s, make oplus_nfc dir fail", __func__);
		goto error_init;
	}

	p_entry = proc_create_data("chipset", S_IRUGO, nfc_info, &nfc_info_fops, (unsigned long *)(NFC_CHIPSET_VERSION));
	if (!p_entry)
	{
		pr_err("%s, make chipset node fail", __func__);
		goto error_init;
	}

	return 0;

error_init:
	pr_err("%s error_init", __func__);
	remove_proc_entry("oplus_nfc", NULL);
	return -ENOENT;
}

static int read_id_properties(struct device_node *np, u32 id_count, struct id_entry *id_entries)
{
    int err;
    u32 i;
    char propname[30];

    for (i = 0; i < id_count; i++) {
        snprintf(propname, sizeof(propname), "id-%u-key", i);
        err = of_property_read_u32(np, propname, &id_entries[i].key);
        if (err) {
          pr_err("%s, Failed to read dts node:%s\n", __func__ ,propname);
          return err;
        }

        snprintf(propname, sizeof(propname), "id-%u-value-chipset", i);
        err = of_property_read_string(np, propname, &id_entries[i].chipset);
        if (err) {
          pr_err("%s, Failed to read dts node:%s\n", __func__ ,propname);
          return err;
        }

        snprintf(propname, sizeof(propname), "id-%u-value-manifest-path", i);
        err = of_property_read_string(np, propname, &id_entries[i].manifest_path);
        if (err) {
          pr_err("%s, Failed to read dts node:%s\n", __func__ ,propname);
          return err;
        }

        snprintf(propname, sizeof(propname), "id-%u-value-feature-path", i);
        err = of_property_read_string(np, propname, &id_entries[i].feature_path);
        if (err) {
          pr_err("%s, Failed to read dts node:%s\n", __func__ ,propname);
          return err;
        }
    }
    pr_info("%s, read_id_properties success", __func__ );

    return 0;
}

static int get_gpio_value(struct device_node *np, int *gpio_value)
{
    int gpio_num = of_get_named_gpio(np, "id-gpio", 0);

    if (!gpio_is_valid(gpio_num)) {
        pr_err("%s, id-gpio is not valid\n", __func__);
        return -EINVAL;
    }

    *gpio_value = gpio_get_value(gpio_num);
    pr_info("%s, id-gpio value is %d", __func__, *gpio_value);
    return 0;
}

static int create_chipset_file_and_symlinks(struct id_entry *entry)
{
    struct proc_dir_entry *p_entry;
    static struct proc_dir_entry *nfc_info = NULL;

    pr_info("%s, entry->chipset:%s entry->manifest_path:%s entry->feature_path:%s", __func__,
        entry->chipset, entry->manifest_path, entry->feature_path);

    nfc_info = proc_mkdir("oplus_nfc", NULL);

    if (!nfc_info)
    {
        pr_err("%s, make oplus_nfc dir fail", __func__);
        remove_proc_entry("oplus_nfc", NULL);
        return -ENOENT;
    }

    if (strcmp("none", entry->chipset) != 0)
    {
        //nfc chip exist
        support_nfc = true;
        strncpy(current_chipset, entry->chipset, sizeof(current_chipset) - 1);
        p_entry = proc_create_data("chipset", S_IRUGO, nfc_info, &nfc_info_fops, (uint32_t *)(NFC_CHIPSET_VERSION));

        if (!p_entry)
        {
            pr_err("%s, make chipset node fail", __func__);
            remove_proc_entry("oplus_nfc", NULL);
            return -ENOENT;
        }
    }
    else
    {
        pr_info("%s, there is no nfc chip", __func__);
    }

    proc_symlink("manifest", nfc_info , entry->manifest_path);
    proc_symlink("feature", nfc_info , entry->feature_path);
    pr_info("%s, create_chipset_file_and_symlinks success", __func__);
    return 0;
}


static int mixed_nfc_probe(struct platform_device *pdev)
{
    struct device_node *np = pdev->dev.of_node;
    u32 id_count;
    int i, gpio_value, err;
    bool found = false;
    struct id_entry *id_entries = NULL;

    if (NULL == np)
    {
        pr_err("%s pdev->dev.of_node is NULL", __func__);
        return -ENOENT;
    }

    err = of_property_read_u32(np, "id_count", &id_count);
    if (err)
    {
      pr_err("%s read dts node id_count failed", __func__);
      return err;
    }

    if (id_count >= MAX_ID_COUNT)
    {
      pr_err("%s error: id_count more than %d", __func__, MAX_ID_COUNT);
      return -ENOENT;
    }

    id_entries = (struct id_entry *)kzalloc(sizeof(struct id_entry) * id_count, GFP_DMA | GFP_KERNEL);
    if(NULL == id_entries)
    {
      pr_err("%s error:can not kzalloc memory for id_entry", __func__);
      return -ENOMEM;
    }

    err = read_id_properties(np, id_count,id_entries);
    if (err)
    {
      pr_err("%s error:read_id_properties failed", __func__);
      goto free_id_entries;
    }

    err = get_gpio_value(np, &gpio_value);
    if (err)
    {
      pr_err("%s error:get_gpio_value failed", __func__);
      goto free_id_entries;
    }

    for (i = 0; i < id_count; i++) {
        if (id_entries[i].key == gpio_value) {
            err = create_chipset_file_and_symlinks(&id_entries[i]);
            if (err)
            {
              pr_err("%s error:create_chipset_file_and_symlinks failed", __func__);
              goto free_id_entries;
            }
            found = true;
            break;
        }
    }

    if (!found) {
        pr_err("%s, No matching key found for GPIO value\n", __func__);
        err = -EINVAL;
        goto free_id_entries;
    }
    pr_info("%s, mixed_nfc_probe success\n", __func__);
    kfree(id_entries);
    return 0;

free_id_entries:
    kfree(id_entries);
    return err;
}


static int oplus_nfc_probe(struct platform_device *pdev)
{
    struct device* dev;
    uint32_t mixed_chipset;

    pr_err("%s, enter", __func__);
    dev = &pdev->dev;
    if (!dev)
    {
        pr_err("%s, no device", __func__);
        return -ENOENT;
    }

    if (of_property_read_u32(dev->of_node, MIXED_CHIPSET, &mixed_chipset))
    {
        pr_info("%s, read dts property mixed-chipset failed", __func__);
        return single_nfc_probe(pdev);
    }
    else
    {
        if (1 == mixed_chipset)
        {
            pr_info("%s, the value of dts property mixed-chipset is 1(true)", __func__);
            return mixed_nfc_probe(pdev);
        }
        else if(0 == mixed_chipset)
        {
            pr_info("%s, the value of dts property mixed-chipset is 0(false)", __func__);
            return single_nfc_probe(pdev);
        }
        else
        {
            pr_err("%s, mixed-chipset's value is wrong,it is neither 1 nor 0", __func__);
            return -ENOENT;
        }
    }
    return 0;
}


static int oplus_nfc_remove(struct platform_device *pdev)
{
	remove_proc_entry("oplus_nfc", NULL);
	return 0;
}

static const struct of_device_id onc[] = {
	{.compatible = "oplus-nfc-chipset", },
	{},
};

MODULE_DEVICE_TABLE(of, onc);

static struct platform_driver oplus_nfc_driver = {
	.probe  = oplus_nfc_probe,
	.remove = oplus_nfc_remove,
	.driver = {
		.name = "oplus-nfc-chipset",
		.of_match_table = of_match_ptr(onc),
	},
};

static int __init oplus_nfc_init(void)
{
	pr_err("enter %s", __func__);
	return platform_driver_register(&oplus_nfc_driver);
}

subsys_initcall(oplus_nfc_init);

static void __exit oplus_nfc_exit(void)
{
	platform_driver_unregister(&oplus_nfc_driver);
}
module_exit(oplus_nfc_exit);

MODULE_DESCRIPTION("OPLUS nfc chipset version");
MODULE_LICENSE("GPL v2");
