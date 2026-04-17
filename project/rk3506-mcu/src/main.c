/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "rpmsg.h"
#include "bsp_can.h"

#include <string.h>

#define RPMSG_TEST
#define CAN_TEST

#ifdef RPMSG_TEST

#define APP_RPMSG_LOCAL_EPT     0x4003U
#define APP_RPMSG_SERVICE_NAME  "rpmsg-mcu0-test"
#define APP_RPMSG_MOTOR_CNT     1U
#define APP_RPMSG_TEMP_C        35

static RPMsgFrameInstance *g_rpmsgIns;


static void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    __disable_irq();

    BSP_UART_Init();
    BSP_CAN_Init();
    BSP_RPMSG_Init();
    
    __enable_irq();
}

static void App_RPMsgCallback(RPMsgFrameInstance *ins)
{
    FrameCommand_t command;
    MotorState_t state;

    if ((ins == NULL) || (RPMsgFrameGetCommand(ins, &command, 1U) == 0U)) {
        return;
    }

    memset(&state, 0, sizeof(state));
    state.temperature_c = APP_RPMSG_TEMP_C;
    if (command.motor_count > 0U) {
        state.motor_id = command.motors[0].motor_id;
        state.position_q16 = command.motors[0].target_position_q16;
        state.velocity_q16 = command.motors[0].target_velocity_q16;
        state.torque_q8 = command.motors[0].target_torque_q8;
        state.status_flags = command.motors[0].ctrl_flags;
    }

    (void)RPMsgFrameSetTelemetryMotorState(ins, 0U, &state);
    (void)RPMsgFrameTransmitTelemetry(ins, HAL_GetTick(), RL_BLOCK);
}

static void App_RPMsgInit(void)
{
    RPMsgFrame_Init_Config_s config;

    memset(&config, 0, sizeof(config));
    config.local_ept = APP_RPMSG_LOCAL_EPT;
    config.remote_ept = RPMSG_REMOTE_EPT_DYNAMIC;
    config.ept_name = APP_RPMSG_SERVICE_NAME;
    config.telemetry_motor_count = APP_RPMSG_MOTOR_CNT;
    config.rpmsg_frame_callback = App_RPMsgCallback;

    g_rpmsgIns = RPMsgFrameInit(&config);
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

    if (CANTransmit(g_canIns, 1000U) == 0U) {
        HAL_DBG_ERR("can tx err\n");
    }

    counter++;
}
#endif

int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    
    RobotInit();

#ifdef RPMSG_TEST
    App_RPMsgInit();
#endif

#ifdef CAN_TEST

    App_CanInit();
#endif

    while (1) {
#ifdef CAN_TEST
        App_CanSendTest();
        HAL_DelayMs(1);
#endif
    }
}

int entry(void)
{
    return main();
}
