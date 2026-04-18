/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 *
 * UART BSP for debug console initialization and stdio redirection.
 */

#include "bsp_uart.h"

// 内部使用的 UART 寄存器基址指针
static struct UART_REG *pUart = UART3;

void BSP_UART_Init(void)
{
    struct HAL_UART_CONFIG hal_uart_config = {
        .baudRate = UART_BR_115200,
        .dataBit  = UART_DATA_8B,
        .stopBit  = UART_ONE_STOPBIT,
        .parity   = UART_PARITY_DISABLE,
    };

    /* 1. 将物理引脚复用为 RMIO 功能
     * RMIO_14 对应物理引脚 GPIO0_B6
     * RMIO_15 对应物理引脚 GPIO0_B7
     */
    HAL_PINCTRL_SetIOMUX(GPIO_BANK0, GPIO_PIN_B6, PIN_CONFIG_MUX_FUNC7); 
    HAL_PINCTRL_SetIOMUX(GPIO_BANK0, GPIO_PIN_B7, PIN_CONFIG_MUX_FUNC7);

    /* 2. 配置 RMIO 路由矩阵 
     * 将 UART3_TX 路由到 RMIO_14 (GPIO0_B6)
     * 将 UART3_RX 路由到 RMIO_15 (GPIO0_B7)
     */
    HAL_PINCTRL_SetRMIO(GPIO_BANK0, GPIO_PIN_B6, RMIO_UART3_TX);
    HAL_PINCTRL_SetRMIO(GPIO_BANK0, GPIO_PIN_B7, RMIO_UART3_RX);
    
    /* 3. 调用 HAL 库初始化 UART 硬件 */
    HAL_UART_Init(&g_uart3Dev, &hal_uart_config);
}

/* ==================================================================
 * 标准 I/O 重定向
 * ================================================================== */
#ifdef __GNUC__
__USED int _write(int fd, char *ptr, int len)
{
    int i = 0;

    /* Only work for STDOUT, STDIN, and STDERR */
    if (fd > 2) {
        return -1;
    }

    while (*ptr && (i < len)) {
        if (*ptr == '\n') {
            HAL_UART_SerialOutChar(pUart, '\r');
        }
        HAL_UART_SerialOutChar(pUart, *ptr);

        i++;
        ptr++;
    }

    return i;
}
#else
int fputc(int ch, FILE *f)
{
    if (ch == '\n') {
        HAL_UART_SerialOutChar(pUart, '\r');
    }

    HAL_UART_SerialOutChar(pUart, (char)ch);

    return 0;
}
#endif