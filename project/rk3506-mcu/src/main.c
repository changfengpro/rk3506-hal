/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "rpmsg_frame.h"
#include "bsp_can.h"
#include "bsp_rpmsg.h"

#include <string.h>

#define RPMSG_TEST
#define CAN_TEST

#ifdef RPMSG_TEST

#define APP_RPMSG_LOCAL_EPT     0x4003U
#define APP_RPMSG_SERVICE_NAME  "rpmsg-mcu0-test"
#define APP_RPMSG_MOTOR_CNT     1U
#define APP_RPMSG_TEMP_C        35

static RPMsgFrameInstance *g_rpmsgIns;
volatile uint32_t g_appRpmsgCmdCnt;
volatile uint32_t g_appRpmsgTxCnt;
volatile uint32_t g_appRpmsgTxErrCnt;
volatile RPMsgFrameInstance *g_dbgRpmsgIns;
volatile FrameCommand_t g_dbgLastRpmsgCommand;
volatile FrameTelemetry_t g_dbgLastRpmsgState;


static void RobotInit()
{
    uint8_t rpmsgInitOk;

    BSP_UART_Init();
    BSP_CAN_Init();
    rpmsgInitOk = BSP_RPMSG_Init();
    HAL_ASSERT(rpmsgInitOk == 1U);
    if (rpmsgInitOk == 0U) {
        HAL_DBG_ERR("bsp rpmsg init failed\n");
    }


    __enable_irq();
}

static void App_RPMsgBuildStateFrame(RPMsgFrameInstance *ins, const FrameCommand_t *command)
{
    FrameTelemetry_t *stateFrame;
    uint8_t motorIdx;
    uint8_t motorCount;

    if ((ins == NULL) || (command == NULL)) {
        return;
    }

    stateFrame = RPMsgFrameGetStateFrame(ins);
    if (stateFrame == NULL) {
        return;
    }

    RPMsgFrameResetStateFrame(ins);

    motorCount = command->motor_count;
    if (motorCount > RPMSG_FRAME_MAX_MOTOR_CNT) {
        motorCount = RPMSG_FRAME_MAX_MOTOR_CNT;
    }

    stateFrame->motor_count = motorCount;

    for (motorIdx = 0U; motorIdx < motorCount; motorIdx++) {
        const MotorCmd_t *motorCmd = &command->motors[motorIdx];
        MotorState_t *motorState = &stateFrame->motors[motorIdx];

        motorState->motor_id = motorCmd->motor_id;
        motorState->position_q16 = motorCmd->target_position_q16;
        motorState->velocity_q16 = motorCmd->target_velocity_q16;
        motorState->torque_q8 = motorCmd->target_torque_q8;
        motorState->temperature_c = APP_RPMSG_TEMP_C;
        motorState->status_flags = motorCmd->ctrl_flags;
    }
}

static void App_RPMsgCallback(RPMsgFrameInstance *ins)
{
    FrameCommand_t command;

    if ((ins == NULL) || (RPMsgFrameGetCommandFrame(ins, &command, 1U) == 0U)) {
        return;
    }

    g_dbgRpmsgIns = ins;
    memcpy((void *)&g_dbgLastRpmsgCommand, &command, sizeof(command));
    g_appRpmsgCmdCnt++;
    App_RPMsgBuildStateFrame(ins, &command);
    if (RPMsgFrameTransmitStateFrame(ins, HAL_GetTick(), RL_BLOCK) == 0U) {
        g_appRpmsgTxErrCnt++;
        return;
    }

    memcpy((void *)&g_dbgLastRpmsgState, &ins->state_frame, sizeof(ins->state_frame));
    g_appRpmsgTxCnt++;
}

static uint8_t App_RPMsgInit(void)
{
    RPMsgFrame_Init_Config_s config;

    memset(&config, 0, sizeof(config));
    config.local_ept = APP_RPMSG_LOCAL_EPT;
    config.remote_ept = RPMSG_REMOTE_EPT_DYNAMIC;
    config.ept_name = APP_RPMSG_SERVICE_NAME;
    config.state_motor_count = APP_RPMSG_MOTOR_CNT;
    config.command_callback = App_RPMsgCallback;

    g_rpmsgIns = RPMsgFrameInit(&config);
    g_dbgRpmsgIns = g_rpmsgIns;
    if (g_rpmsgIns == NULL) {
        return 0U;
    }

    HAL_DBG("app rpmsg frame ready\n");
    return 1U;
}



#endif

#ifdef CAN_TEST

#define APP_CAN_TX_ID  0x200U
#define APP_CAN_RX_ID  0x201U

static CANInstance *g_canIns;
volatile uint32_t g_appCanTxCnt;
volatile uint32_t g_appCanTxErrCnt;
volatile uint32_t g_appCanRxCnt;

static void App_CanCallback(CANInstance *ins)
{
    (void)ins;
    g_appCanRxCnt++;
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
        g_appCanTxErrCnt++;
        HAL_DBG_ERR("can tx err\n");
        return;
    }

    g_appCanTxCnt++;
    counter++;
}
#endif

int main(void)
{
    uint8_t rpmsgModuleOk;

    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    
    RobotInit();

#ifdef RPMSG_TEST
    rpmsgModuleOk = App_RPMsgInit();
    HAL_ASSERT(rpmsgModuleOk == 1U);
    if (rpmsgModuleOk == 0U) {
        HAL_DBG_ERR("app rpmsg module init failed\n");
    }
#endif

#ifdef CAN_TEST

    App_CanInit();
#endif

    while (1) {
#ifdef CAN_TEST
        App_CanSendTest();
#endif
        HAL_DelayMs(1);
    }
}

int entry(void)
{
    return main();
}
