/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 *
 * Robot business module:
 * - RPMsg command/telemetry bridge;
 * - CAN test TX/RX workflow;
 * - SysTick-driven periodic sending.
 */

#include "robot.h"

#include <string.h>

#include "bsp_can.h"
#include "hal_bsp.h"
#include "rpmsg_frame.h"

#define APP_RPMSG_LOCAL_EPT     0x4003U
#define APP_RPMSG_SERVICE_NAME  "rpmsg-mcu0-test"
#define APP_RPMSG_MOTOR_CNT     1U
#define APP_RPMSG_TEMP_C        35

#define APP_CAN_TX_ID  0x200U
#define APP_CAN_RX_ID  0x201U

static RPMsgFrameInstance *s_rpmsgIns;
static CANInstance *s_canIns;
static Robot_Feature_s s_feature;

volatile uint32_t g_appCanTxCnt;
volatile uint32_t g_appCanTxErrCnt;
volatile uint32_t g_appCanRxCnt;

/**
 * @brief Build MCU->Linux telemetry frame from latest command frame.
 */
static void Robot_RPMsgBuildStateFrame(RPMsgFrameInstance *ins,
                                       const FrameCommand_t *command)
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

/**
 * @brief RPMsg command callback: decode command and reply telemetry.
 */
static void Robot_RPMsgCallback(RPMsgFrameInstance *ins)
{
    FrameCommand_t command;

    if ((ins == NULL) || (RPMsgFrameGetCommandFrame(ins, &command, 1U) == 0U)) {
        return;
    }

    Robot_RPMsgBuildStateFrame(ins, &command);
    (void)RPMsgFrameTransmitStateFrame(ins, HAL_GetTick(), RL_BLOCK);
}

/**
 * @brief Initialize RPMsg frame module for robot command channel.
 */
static uint8_t Robot_RPMsgInit(void)
{
    RPMsgFrame_Init_Config_s config;

    if (s_rpmsgIns != NULL) {
        return 1U;
    }

    memset(&config, 0, sizeof(config));
    config.local_ept = APP_RPMSG_LOCAL_EPT;
    config.remote_ept = RPMSG_REMOTE_EPT_DYNAMIC;
    config.ept_name = APP_RPMSG_SERVICE_NAME;
    config.state_motor_count = APP_RPMSG_MOTOR_CNT;
    config.command_callback = Robot_RPMsgCallback;

    s_rpmsgIns = RPMsgFrameInit(&config);
    if (s_rpmsgIns == NULL) {
        return 0U;
    }

    HAL_NVIC_SetPriority(INTMUX_OUT3_IRQn, 1U, 0U);
    HAL_DBG("app rpmsg frame ready\n");

    return 1U;
}

/**
 * @brief Swap bytes in 32-bit chunks to match CAN controller payload order.
 */
static void Robot_CanSwapBytesPerWord(uint8_t *dst, const uint8_t *src, uint8_t len)
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

/**
 * @brief Periodic CAN test sender, called from SysTick context.
 */
static void Robot_CanSendTestISR(void)
{
    static uint8_t counter = 0U;
    struct CANFD_MSG txMsg = { 0 };
    uint8_t rawPayload[8];
    uint32_t cmdState;

    if (s_canIns == NULL) {
        return;
    }

    cmdState = READ_REG(s_canIns->can_handle->CMD);
    if ((cmdState & (CAN_CMD_TX0_REQ_MASK | CAN_CMD_TX1_REQ_MASK)) ==
        (CAN_CMD_TX0_REQ_MASK | CAN_CMD_TX1_REQ_MASK)) {
        g_appCanTxErrCnt++;
        return;
    }

    rawPayload[0] = 0xA5U;
    rawPayload[1] = 0x5AU;
    rawPayload[2] = counter;
    rawPayload[3] = (uint8_t)(~counter);
    rawPayload[4] = 0x01U;
    rawPayload[5] = 0x02U;
    rawPayload[6] = 0x03U;
    rawPayload[7] = 0x04U;

    memcpy(s_canIns->tx_buff, rawPayload, sizeof(rawPayload));

    txMsg.stdId = s_canIns->tx_id;
    txMsg.ide = CANFD_ID_STANDARD;
    txMsg.rtr = CANFD_RTR_DATA;
    txMsg.fdf = CANFD_FORMAT;
    txMsg.dlc = 8U;
    Robot_CanSwapBytesPerWord(txMsg.data, rawPayload, txMsg.dlc);

    if (HAL_CANFD_Transmit(s_canIns->can_handle, &txMsg) != HAL_OK) {
        g_appCanTxErrCnt++;
        return;
    }

    g_appCanTxCnt++;
    counter++;
}

/**
 * @brief CAN RX callback for test statistics and debug output.
 */
static void Robot_CanCallback(CANInstance *ins)
{
    (void)ins;
    g_appCanRxCnt++;
    HAL_DBG("can rx ok\n");
}

/**
 * @brief Initialize CAN test channel and SysTick tick source.
 */
static uint8_t Robot_CanInit(void)
{
    CAN_Init_Config_s config;

    if (s_canIns != NULL) {
        return 1U;
    }

    memset(&config, 0, sizeof(config));
    config.can_handle = g_can0Dev.pReg;
    config.tx_id = APP_CAN_TX_ID;
    config.rx_id = APP_CAN_RX_ID;
    config.can_module_callback = Robot_CanCallback;

    CANSetInterruptEnable(CAN_INTERRUPT_ALL);
    s_canIns = CANRegister(&config);
    if (s_canIns == NULL) {
        return 0U;
    }

    CANSetDLC(s_canIns, 8U);

    HAL_SYSTICK_Init();
    HAL_NVIC_SetPriority(SysTick_IRQn, 2U, 0U);

    return 1U;
}

/**
 * @brief Initialize robot module by enabled feature switches.
 */
uint8_t Robot_Init(const Robot_Feature_s *feature)
{
    if (feature == NULL) {
        return 0U;
    }

    s_feature = *feature;

    if ((s_feature.enable_rpmsg_test != 0U) && (Robot_RPMsgInit() == 0U)) {
        HAL_DBG_ERR("robot rpmsg init failed\n");
        return 0U;
    }

    if ((s_feature.enable_can_test != 0U) && (Robot_CanInit() == 0U)) {
        HAL_DBG_ERR("robot can init failed\n");
        return 0U;
    }

    return 1U;
}

/**
 * @brief Robot periodic hook called by SysTick_Handler.
 */
void Robot_SysTickHandler(void)
{
    if (s_feature.enable_can_test == 0U) {
        return;
    }

    Robot_CanSendTestISR();
}
