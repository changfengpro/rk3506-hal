/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 *
 * Top-level entry:
 * 1) bring up BSP services;
 * 2) initialize robot business module;
 * 3) keep MCU in WFI and drive periodic work in SysTick.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_rpmsg.h"
#include "robot.h"

#define APP_ENABLE_RPMSG_TEST  1U
#define APP_ENABLE_CAN_TEST    1U

/**
 * @brief Initialize board-level services required by the robot module.
 */
static void BSPInit(void)
{
    uint8_t rpmsgInitOk = 1U;

    BSP_UART_Init();
    BSP_CAN_Init();

#if (APP_ENABLE_RPMSG_TEST != 0U)
    rpmsgInitOk = BSP_RPMSG_Init();
    HAL_ASSERT(rpmsgInitOk == 1U);
    if (rpmsgInitOk == 0U) {
        HAL_DBG_ERR("bsp rpmsg init failed\n");
    }
#endif

    __enable_irq();
}

/**
 * @brief Initialize module-level robot business logic.
 */
static void RobotInit(void)
{
    Robot_Feature_s feature = { 0 };
    uint8_t robotInitOk;

    feature.enable_rpmsg_test = APP_ENABLE_RPMSG_TEST;
    feature.enable_can_test = APP_ENABLE_CAN_TEST;

    robotInitOk = Robot_Init(&feature);
    HAL_ASSERT(robotInitOk == 1U);
    if (robotInitOk == 0U) {
        HAL_DBG_ERR("robot init failed\n");
    }
}

/**
 * @brief SysTick IRQ entry; delegates periodic work to robot module.
 */
void SysTick_Handler(void)
{
    HAL_SYSTICK_IRQHandler();
    Robot_SysTickHandler();
}

int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();

    BSPInit();
    RobotInit();

    while (1) {
        __WFI();
    }
}

int entry(void)
{
    return main();
}
