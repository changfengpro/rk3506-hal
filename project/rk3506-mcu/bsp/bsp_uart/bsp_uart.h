#pragma once

#include <stdint.h>
#include <stdio.h>
#include "hal_base.h"
#include "hal_bsp.h"

/**
 * @brief  初始化调试串口 (UART3)
 * @note   包含引脚复用(RMIO)、波特率配置以及HAL库初始化
 */
void BSP_UART_Init(void);