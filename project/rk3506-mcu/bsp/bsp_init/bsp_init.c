/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 */

#include "bsp_init.h"

#include "bsp_can.h"
#include "bsp_rpmsg.h"
#include "bsp_uart.h"

#define BSP_SYSTICK_IRQ_PRIO 1U
#define BSP_RPMSG_IRQ_PRIO   3U

void BSPInit(void)
{
    uint8_t rpmsgInitOk;

    BSP_UART_Init();
    BSP_CAN_Init();

    rpmsgInitOk = BSP_RPMSG_Init();
    HAL_ASSERT(rpmsgInitOk == 1U);
    if (rpmsgInitOk == 0U) {
        HAL_DBG_ERR("bsp rpmsg init failed\n");
    }

    HAL_SYSTICK_Init();
    HAL_NVIC_SetPriority(SysTick_IRQn, BSP_SYSTICK_IRQ_PRIO, 0U);
    HAL_NVIC_SetPriority(INTMUX_OUT3_IRQn, BSP_RPMSG_IRQ_PRIO, 0U);
}
