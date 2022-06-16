#ifndef __OPLUS_CHG_SYMBOL_H__
#define __OPLUS_CHG_SYMBOL_H__

#if IS_ENABLED(CONFIG_OPLUS_SM8350_CHARGER) || IS_ENABLED(CONFIG_OPLUS_SM8450_CHARGER)
#if IS_ENABLED(CONFIG_OPLUS_ADSP_CHARGER)
#define USE_ADSP
#endif
#endif

#ifdef USE_ADSP
#define oplus_turn_off_power_when_adsp_crash oplus_turn_off_power_when_adsp_crash_v1
#define oplus_is_pd_svooc oplus_is_pd_svooc_v1
#define oplus_adsp_crash_recover_work oplus_adsp_crash_recover_work_v1
#define qti_battery_charger_get_prop qti_battery_charger_get_prop_v1
#endif /* USE_ADSP */

#endif /* __OPLUS_CHG_SYMBOL_H__ */
