ifeq ($(CONFIG_OPLUS_CHARGER_MTK),y)
subdir-ccflags-y += -Werror
subdir-ccflags-y += -Wno-unused-variable
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6779Q),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif
endif
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6889),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6893),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6877),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6779R),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif
ifeq ($(CONFIG_OPLUS_CHARGER_MTK6765R),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
endif

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include

oplus_chg-y     += gauge_ic/oplus_bq27541.o

oplus_chg-y     += vooc_ic/oplus_n76e.o
oplus_chg-y     += vooc_ic/oplus_vooc_fw.o
oplus_chg-y     += vooc_ic/oplus_stm8s.o
oplus_chg-y     += vooc_ic/oplus_rk826.o
oplus_chg-y     += vooc_ic/oplus_op10.o

oplus_chg-y     += adapter_ic/oplus_stm.o

oplus_chg-y     += charger_ic/oplus_short_ic.o
oplus_chg-y     += charger_ic/oplus_mp2650.o
oplus_chg-y     += charger_ic/oplus_charge_pump.o
oplus_chg-y     += charger_ic/oplus_usbtemp.o
oplus_chg-y     += charger_ic/oplus_hl7227.o
oplus_chg-y     += wireless_ic/oplus_chargepump.o
oplus_chg-y     += wireless_ic/oplus_nu1619a.o

oplus_chg-y     += hal/oplus_chg_ic.o
oplus_chg-y     += hal/wls_chg_fast.o
oplus_chg-y     += hal/wls_chg_normal.o
oplus_chg-y     += hal/wls_chg_rx.o
oplus_chg-y     += oplus_chg_comm.o
oplus_chg-y     += oplus_chg_wls.o

oplus_chg-y     += oplus_charger.o
oplus_chg-y     += oplus_gauge.o
oplus_chg-y     += oplus_vooc.o
oplus_chg-y     += oplus_adapter.o
oplus_chg-y     += oplus_wireless.o
oplus_chg-y     += oplus_short.o
oplus_chg-y     += oplus_configfs.o
oplus_chg-y     += oplus_debug_info.o
oplus_chg-y     += oplus_chg_ops_manager.o
oplus_chg-y	+= oplus_chg_core.o
oplus_chg-$(CONFIG_OPLUS_CHG_DYNAMIC_CONFIG) += oplus_chg_cfg.o
oplus_chg-y	+= oplus_chg_voter.o
oplus_chg-y	+= oplus_chg_strategy.o
oplus_chg-y	+= oplus_pps.o

oplus_chg-$(CONFIG_OPLUS_CHARGER_MTK6983S)     += charger_ic/oplus_battery_mtk6983S.o
oplus_chg-$(CONFIG_OPLUS_CHARGER_MTK6895S)     += charger_ic/oplus_battery_mtk6895S.o

ccflags-y += -DOPLUS_CHG_REG_DUMP_ENABLE
KBUILD_LDS_MODULE_ATTACH = oplus_chg_module.lds

obj-m := oplus_chg.o
