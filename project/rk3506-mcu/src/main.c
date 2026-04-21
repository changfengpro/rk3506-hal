/*
 * @Description: 
 * @Author: changfengpro
 * @brief: 
 * @version: 
 * @Date: 2026-04-20 12:10:53
 * @LastEditors:  
 * @LastEditTime: 2026-04-21 21:05:58
 */
/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_can.h"

#include <string.h>


int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();

    __enable_irq();

    while (1) {
        
    }
}

int entry(void)
{
    return main();
}
