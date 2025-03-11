#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#


obj-y += isp_6s/
obj-y += isp_4/

ifeq ($(strip $(CONFIG_MTK_ISP_IRQ_CONTROLLER)), y)
$(warning "CONFIG_MTK_ISP_IRQ_CONTROLLER")
$(warning $(CONFIG_MTK_ISP_IRQ_CONTROLLER))
LOCAL_CFLAGS += -DISP_IRQ_CONTROLLER
endif
