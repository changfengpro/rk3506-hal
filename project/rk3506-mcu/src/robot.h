/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 */

#ifndef __ROBOT_H
#define __ROBOT_H

#include "hal_base.h"

/**
 * @brief Robot module feature switches.
 */
typedef struct {
    uint8_t enable_rpmsg_test;
    uint8_t enable_can_test;
} Robot_Feature_s;

/**
 * @brief Initialize module-level robot business.
 *
 * This function only initializes module/business objects and does not touch
 * board-level BSP bring-up.
 */
uint8_t Robot_Init(const Robot_Feature_s *feature);

/**
 * @brief SysTick hook for module-level periodic work.
 */
void Robot_SysTickHandler(void);

/* Debug counters for CAN test path. */
extern volatile uint32_t g_appCanTxCnt;
extern volatile uint32_t g_appCanTxErrCnt;
extern volatile uint32_t g_appCanRxCnt;

#endif /* __ROBOT_H */
