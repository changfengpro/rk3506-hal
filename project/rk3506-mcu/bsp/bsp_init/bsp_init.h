/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 */

#ifndef __BSP_INIT_H
#define __BSP_INIT_H

#include "hal_base.h"

/**
 * @brief BSP层统一初始化入口。
 *
 * 仅初始化当前业务所需的基础外设服务，供RobotInit统一调用。
 */
void BSPInit(void);

#endif /* __BSP_INIT_H */
