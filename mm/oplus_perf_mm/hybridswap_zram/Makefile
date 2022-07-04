# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_HYBRIDSWAP_ZRAM) += oplus_bsp_hybridswap_zram.o

oplus_bsp_hybridswap_zram-y	:=	zcomp.o zram_drv.o
oplus_bsp_hybridswap_zram-$(CONFIG_HYBRIDSWAP) += hybridswap/hybridswap_main.o
oplus_bsp_hybridswap_zram-$(CONFIG_HYBRIDSWAP_SWAPD) += hybridswap/hybridswap_swapd.o
oplus_bsp_hybridswap_zram-$(CONFIG_HYBRIDSWAP_ASYNC_COMPRESS) += hybridswap/hybridswap_akcompress.o
oplus_bsp_hybridswap_zram-$(CONFIG_HYBRIDSWAP_CORE) += hybridswap/hybridswap_eswap.o
