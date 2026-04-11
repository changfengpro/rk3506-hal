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

int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();

#ifdef RPMSG_TEST
    RPMsg_Service_Init();
    App_RPMsgInit();
#endif

    while (1) {

        
    }
}

int entry(void)
{
    return main();
}
