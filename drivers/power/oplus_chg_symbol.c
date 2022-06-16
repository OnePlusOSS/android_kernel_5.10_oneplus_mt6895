#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <soc/oplus/system/oplus_chg.h>

#if IS_ENABLED(CONFIG_OPLUS_SM8350_CHARGER) || IS_ENABLED(CONFIG_OPLUS_SM8450_CHARGER)
#if IS_ENABLED(CONFIG_OPLUS_ADSP_CHARGER)
#define USE_ADSP
#endif
#endif

#ifdef USE_ADSP
#include <linux/soc/qcom/battery_charger.h>
#endif

static int oplus_chg_version;

enum {
	OPLUS_CHG_V1 = 0,
	OPLUS_CHG_V2,
};

#ifdef USE_ADSP

#if IS_ENABLED(CONFIG_OPLUS_SM8350_CHARGER)
extern void oplus_turn_off_power_when_adsp_crash_v1(void);
extern void oplus_turn_off_power_when_adsp_crash_v2(void);
void oplus_turn_off_power_when_adsp_crash(void)
{
	switch (oplus_chg_version) {
	case OPLUS_CHG_V2:
		return oplus_turn_off_power_when_adsp_crash_v2();
	case OPLUS_CHG_V1:
	default:
		return oplus_turn_off_power_when_adsp_crash_v1();
	}
}
EXPORT_SYMBOL(oplus_turn_off_power_when_adsp_crash);
#endif

extern bool oplus_is_pd_svooc_v1(void);
extern bool oplus_is_pd_svooc_v2(void);
bool oplus_is_pd_svooc(void)
{
	switch (oplus_chg_version) {
	case OPLUS_CHG_V2:
		return oplus_is_pd_svooc_v2();
	case OPLUS_CHG_V1:
	default:
		return oplus_is_pd_svooc_v1();
	}
}
EXPORT_SYMBOL(oplus_is_pd_svooc);

#if defined(CONFIG_OPLUS_SM8350_CHARGER)
extern void oplus_adsp_crash_recover_work_v1(void);
extern void oplus_adsp_crash_recover_work_v2(void);
void oplus_adsp_crash_recover_work(void)
{
	switch (oplus_chg_version) {
	case OPLUS_CHG_V2:
		return oplus_adsp_crash_recover_work_v2();
	case OPLUS_CHG_V1:
	default:
		return oplus_adsp_crash_recover_work_v1();
	}
}
EXPORT_SYMBOL(oplus_adsp_crash_recover_work);
#endif

extern int qti_battery_charger_get_prop_v1(const char *name,
					   enum battery_charger_prop prop_id,
					   int *val);
extern int qti_battery_charger_get_prop_v2(const char *name,
					   enum battery_charger_prop prop_id,
					   int *val);
int qti_battery_charger_get_prop(const char *name,
				 enum battery_charger_prop prop_id, int *val)
{
	switch (oplus_chg_version) {
	case OPLUS_CHG_V2:
		return qti_battery_charger_get_prop_v2(name, prop_id, val);
	case OPLUS_CHG_V1:
	default:
		return qti_battery_charger_get_prop_v1(name, prop_id, val);
	}
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);
#endif /* USE_ADSP */

static int __init oplus_chg_symbol_init(void)
{
	struct device_node *node;

	node = of_find_node_by_path("/soc/oplus_chg_core");
	if ((node != NULL) && of_property_read_bool(node, "oplus,chg_framework_v2"))
		oplus_chg_version = OPLUS_CHG_V2;
	else
		oplus_chg_version = OPLUS_CHG_V1;

	return 0;
}

static void __exit oplus_chg_symbol_exit(void)
{
}

subsys_initcall(oplus_chg_symbol_init);
module_exit(oplus_chg_symbol_exit);

MODULE_DESCRIPTION("oplus charge symbol");
MODULE_LICENSE("GPL");
