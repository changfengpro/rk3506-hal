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

RPMsgFrameInstance *g_rpmsgIns;


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
        motorState->status_flags = 0U;
    }
}

static void App_RPMsgCallback(RPMsgFrameInstance *ins)
{
    FrameCommand_t command;

    if ((ins == NULL) || (RPMsgFrameGetCommandFrame(ins, &command, 1U) == 0U)) {
        return;
    }

    App_RPMsgBuildStateFrame(ins, &command);
    if (RPMsgFrameTransmitStateFrame(ins, HAL_GetTick(), RL_BLOCK) == 0U) {
        return;
    }
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
    if (g_rpmsgIns == NULL) {
        return 0U;
    }

    HAL_NVIC_SetPriority(INTMUX_OUT3_IRQn, 1U, 0U);
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

static void App_CanSwapBytesPerWord(uint8_t *dst, const uint8_t *src, uint8_t len)
{
    uint8_t base;

    for (base = 0U; base < len; base = (uint8_t)(base + 4U)) {
        uint8_t i;
        uint8_t remain = (uint8_t)(len - base);
        uint8_t chunk = (remain < 4U) ? remain : 4U;

        for (i = 0U; i < chunk; i++) {
            dst[base + i] = src[base + (chunk - 1U - i)];
        }
    }
}

static void App_CanSendTestISR(void)
{
    static uint8_t counter = 0U;
    struct CANFD_MSG tx_msg = { 0 };
    uint8_t raw_payload[8];
    uint32_t cmd_state;

    if (g_canIns == NULL) {
        return;
    }

    cmd_state = READ_REG(g_canIns->can_handle->CMD);
    if ((cmd_state & (CAN_CMD_TX0_REQ_MASK | CAN_CMD_TX1_REQ_MASK)) ==
        (CAN_CMD_TX0_REQ_MASK | CAN_CMD_TX1_REQ_MASK)) {
        g_appCanTxErrCnt++;
        return;
    }

    raw_payload[0] = 0xA5U;
    raw_payload[1] = 0x5AU;
    raw_payload[2] = counter;
    raw_payload[3] = (uint8_t)(~counter);
    raw_payload[4] = 0x01U;
    raw_payload[5] = 0x02U;
    raw_payload[6] = 0x03U;
    raw_payload[7] = 0x04U;

    memcpy(g_canIns->tx_buff, raw_payload, sizeof(raw_payload));

    tx_msg.stdId = g_canIns->tx_id;
    tx_msg.ide = CANFD_ID_STANDARD;
    tx_msg.rtr = CANFD_RTR_DATA;
    tx_msg.fdf = CANFD_FORMAT;
    tx_msg.dlc = 8U;
    App_CanSwapBytesPerWord(tx_msg.data, raw_payload, tx_msg.dlc);

    if (HAL_CANFD_Transmit(g_canIns->can_handle, &tx_msg) != HAL_OK) {
        g_appCanTxErrCnt++;
        return;
    }

    g_appCanTxCnt++;
    counter++;
}

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

    CANSetInterruptEnable(CAN_INTERRUPT_ALL);
    g_canIns = CANRegister(&config);
    HAL_ASSERT(g_canIns != NULL);
    CANSetDLC(g_canIns, 8U);

    HAL_SYSTICK_Init();
    HAL_NVIC_SetPriority(SysTick_IRQn, 2U, 0U);
}
#endif

void SysTick_Handler(void)
{
    HAL_SYSTICK_IRQHandler();

#ifdef CAN_TEST
    App_CanSendTestISR();
#endif
}

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
        __WFI();
    }
}

int entry(void)
{
    return main();
}
