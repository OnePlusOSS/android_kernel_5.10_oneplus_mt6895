# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_POWER_RESET)	+= reset/
obj-$(CONFIG_POWER_SUPPLY)	+= supply/

ifeq ($(CONFIG_OPLUS_CHARGER_MTK6983S),y)
obj-$(CONFIG_OPLUS_CHARGER)	+= oplus/
endif

ifeq ($(CONFIG_OPLUS_CHG_V2),y)
obj-y	+= oplus_v2/
else ifeq ($(CONFIG_OPLUS_CHG_V2),m)
obj-y	+= oplus_v2/
endif
obj-$(CONFIG_OPLUS_CHG_V2)	:= oplus_chg_symbol.o
