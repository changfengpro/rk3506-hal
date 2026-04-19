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
#include "robot.h"

/**
 * @brief SysTick IRQ entry; delegates periodic work to robot module.
 */
void SysTick_Handler(void)
{
    HAL_SYSTICK_IRQHandler();
}

int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();

    RobotInit();

    while (1) {
        RobotTask();
        __WFI();
    }
}

int entry(void)
{
    return main();
}
