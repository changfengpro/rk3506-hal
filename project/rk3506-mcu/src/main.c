/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_rpmsg.h"
#include "bsp_can.h"

#include <string.h>

#define RPMSG_TEST
#define CAN_TEST

#ifdef RPMSG_TEST

#define APP_RPMSG_LOCAL_EPT     0x4003U
#define APP_RPMSG_SERVICE_NAME  "rpmsg-mcu0-test"
#define APP_RPMSG_REPLY_MSG     "mcu ack"

static RPMsgInstance *g_rpmsgIns;

static void App_RPMsgCallback(RPMsgInstance *ins)
{
    uint32_t txLen;

    if (ins == NULL) {
        return;
    }

    txLen = (uint32_t)strlen(APP_RPMSG_REPLY_MSG) + 1U;
    memcpy(ins->tx_buff, APP_RPMSG_REPLY_MSG, txLen);
    RPMsg_SetTxLen(ins, txLen);
    (void)RPMsg_Transmit(ins, RL_BLOCK);
}

static void App_RPMsgInit(void)
{
    RPMsg_Init_Config_s config;

    memset(&config, 0, sizeof(config));
    config.local_ept = APP_RPMSG_LOCAL_EPT;
    config.remote_ept = RPMSG_REMOTE_EPT_DYNAMIC;
    config.ept_name = APP_RPMSG_SERVICE_NAME;
    config.rpmsg_module_callback = App_RPMsgCallback;

    g_rpmsgIns = RPMsg_Register(&config);
    HAL_ASSERT(g_rpmsgIns != NULL);
}

#endif

#ifdef CAN_TEST

#define APP_CAN_TX_ID  0x200U
#define APP_CAN_RX_ID  0x201U

static CANInstance *g_canIns;

static void App_CanCallback(CANInstance *ins)
{
    (void)ins;
    HAL_DBG("can rx ok\n");
}

static void App_CanInit(void)
{
    CAN_Init_Config_s config;

    memset(&config, 0, sizeof(config));
    config.can_handle = g_can0Dev.pReg;
    config.tx_id = APP_CAN_TX_ID;
    config.rx_id = APP_CAN_RX_ID;
    config.can_module_callback = App_CanCallback;

    CANSetInterruptEnable(CAN_INTERRUPT_RX);
    g_canIns = CANRegister(&config);
    HAL_ASSERT(g_canIns != NULL);
    CANSetDLC(g_canIns, 8U);
}

static void App_CanSendTest(void)
{
    static uint8_t counter = 0U;

    if (g_canIns == NULL) {
        return;
    }

    g_canIns->tx_buff[0] = 0xA5U;
    g_canIns->tx_buff[1] = 0x5AU;
    g_canIns->tx_buff[2] = counter;
    g_canIns->tx_buff[3] = (uint8_t)(~counter);
    g_canIns->tx_buff[4] = 0x01U;
    g_canIns->tx_buff[5] = 0x02U;
    g_canIns->tx_buff[6] = 0x03U;
    g_canIns->tx_buff[7] = 0x04U;

    if (CANTransmit(g_canIns, 1000U) != 0U) {
        HAL_DBG("can tx ok\n");
    } else {
        HAL_DBG_ERR("can tx err\n");
    }

    counter++;
}
#endif

int main(void)
{
    HAL_Init();
    BSP_Init();
    BSP_UART_Init();

#ifdef RPMSG_TEST
    RPMsg_Service_Init();
    App_RPMsgInit();
#endif

#ifdef CAN_TEST
    BSP_CAN_Init();
    App_CanInit();
#endif
    __enable_irq();

    while (1) {
#ifdef CAN_TEST
        App_CanSendTest();
        HAL_DelayMs(1000);
#endif
    }
}

int entry(void)
{
    return main();
}
