# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Kernel SOC specific device drivers.
#

obj-$(CONFIG_ARCH_ACTIONS)	+= actions/
obj-$(CONFIG_SOC_ASPEED)	+= aspeed/
obj-$(CONFIG_ARCH_AT91)		+= atmel/
obj-y				+= bcm/
obj-$(CONFIG_ARCH_DOVE)		+= dove/
obj-$(CONFIG_MACH_DOVE)		+= dove/
obj-y				+= fsl/
obj-$(CONFIG_ARCH_GEMINI)	+= gemini/
obj-y				+= imx/
obj-$(CONFIG_ARCH_IXP4XX)	+= ixp4xx/
obj-$(CONFIG_SOC_XWAY)		+= lantiq/
obj-y				+= mediatek/
obj-y				+= amlogic/
obj-y				+= qcom/
obj-y				+= renesas/
obj-$(CONFIG_ARCH_ROCKCHIP)	+= rockchip/
obj-$(CONFIG_SOC_SAMSUNG)	+= samsung/
obj-$(CONFIG_SOC_SIFIVE)	+= sifive/
obj-y				+= sunxi/
obj-$(CONFIG_ARCH_TEGRA)	+= tegra/
obj-y				+= ti/
obj-$(CONFIG_ARCH_U8500)	+= ux500/
obj-$(CONFIG_PLAT_VERSATILE)	+= versatile/
obj-y				+= xilinx/
obj-$(CONFIG_ARCH_ZX)		+= zte/
obj-$(CONFIG_SOC_KENDRYTE)	+= kendryte/
#ifdef OPLUS_BUG_STABILITY
obj-$(CONFIG_OPLUS_SYSTEM_KERNEL)	+= oplus/system/
#endif /* OPLUS_BUG_STABILITY */

#ifdef OPLUS_DFR_FEATURE
obj-y += oplus/dfr/
#endif /* OPLUS_DFR_FEATURE */

#ifdef OPLUS_BOOT_FEATURE
obj-y += oplus/boot/
#endif /* OPLUS_BOOT_FEATURE */

#ifdef OPLUS_DFT_FEATURE
obj-y += oplus/dft/
#endif /* OPLUS_DFT_FEATURE */

#ifdef OPLUS_FEATURE_HANS
obj-$(CONFIG_OPLUS_FEATURE_HANS_GKI)	+= oplus/hans/
#endif /* OPLUS_FEATURE_HANS */

#ifdef OPLUS_TP_BASIC
obj-$(CONFIG_OPLUS_DEVICE_IFNO)		+= oplus/device_info/

#ifdef OPLUS_FEATURE_MIDAS_GKI
obj-$(CONFIG_OPLUS_FEATURE_MIDAS_GKI)	+= oplus/midas/
#endif /* OPLUS_FEATURE_MIDAS_GKI */

#ifdef OPLUS_FEATURE_MM_FEEDBACK
obj-$(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)	+= oplus/oplus_mm_fb/
#endif /* OPLUS_FEATURE_MM_FEEDBACK */
