# SPDX-License-Identifier: BSD-3-Clause */

# Copyright (c) 2022 Rockchip Electronics Co., Ltd.

PROJECT		:= RK3506
SOC		:= RK3506
CPU		:= -mcpu=cortex-m0

# BUS M0
CFLAGS := -O0
CFLAGS += -MMD -MP
LDFLAGS := --specs=nosys.specs
LINKER_SCRIPT := gcc_bus_m0.ld

-include ../../../middleware/benchmark/benchmark.mk
-include ../../../middleware/rpmsg-lite/rpmsg-lite.mk
-include ../../../middleware/simple_console/simple_console.mk


INCLUDES += -I ../bsp/bsp_uart \
            -I ../bsp/bsp_can \
            -I ../bsp/bsp_rpmsg \
            -I ../modules \
            -I ../modules/rpmsg_frame \
            -I ../modules/protocol \
            -I ../modules/algorithm \
            -I ../modules/motor \
            -I ../modules/motor/DJImotor



SRC_DIRS += ../bsp/bsp_uart \
            ../bsp/bsp_can \
            ../bsp/bsp_rpmsg \
            ../modules/rpmsg_frame \
            ../modules/algorithm \
            ../modules/motor/DJImotor \


DEPS += $(OBJS:.o=.d)
-include $(DEPS)

include ../../common/GCC/Cortex-M.mk