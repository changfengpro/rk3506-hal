/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_can.h"

/********************* Private MACRO Definition ******************************/
// #define TEST_DEMO
// #define DEBUG_FLAG_ADDR  0x03b00000
// #define WRITE_FLAG(val)  (*((volatile uint32_t *)DEBUG_FLAG_ADDR) = (val))
/********************* Private Structure Definition **************************/
/********************* Private Variable Definition ***************************/
static uint32_t count;
static uint8_t can_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
CANInstance *test_can0 = NULL;
CANInstance *test_can1 = NULL;
static CANInstance *s_can0 = NULL;
/********************* Private Function Definition ***************************/


// extern void app_rpmsg_send_loop_task(void);

/********************* Public Function Definition ****************************/


int main(void)
{
    /* HAL BASE Init */
    HAL_Init();

    /* BSP Init */
    BSP_Init();

    /* INTMUX Init */
    HAL_INTMUX_Init();

    /* UART Init */
    BSP_UART_Init();

    HAL_DBG("g_can0Dev.irqNum = %d\n", g_can0Dev.irqNum);

    CAN_Service_Init();

    __enable_irq();

    /* 回环自测时 rx_id 需要和 tx_id 一致，才能命中回调过滤 */
    CAN_Init_Config_s cfg0 = { .can_handle = g_can0Dev.pReg, .tx_id = 0x200, .rx_id = 0x201 };
    s_can0 = CAN_Register(&cfg0);
    if (s_can0 == NULL) {
        HAL_DBG_ERR("CAN_Register failed, stop.\n");
        while (1) {
            ;
        }
    }

    HAL_DBG("RK3506 CAN System Ready...\n");

    HAL_DBG("\n=======================================\n");
    HAL_DBG("Hello RK3506 MCU - UART3 is ALIVE!!!\n");
    HAL_DBG("CAN Build Tag: 2026-04-09-CAN-EXT-1M-HARDIRQ-NOSELFTEST-NOFILTER\n");
    HAL_DBG("=======================================\n");


#ifdef TEST_DEMO
    test_demo();
#endif

    while (1) {
    
        HAL_DBG("MCU is running, tick: %d\n", count++);
        
        uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
        memcpy(s_can0->tx_buff, data, 8);
        s_can0->tx_len = 8;
        CAN_Transmit(s_can0, 100);

        HAL_DelayMs(1000);

    }
}

int entry(void)
{
    return main();
}
