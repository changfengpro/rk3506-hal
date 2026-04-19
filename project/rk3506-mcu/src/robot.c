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
#define APP_CAN_TX_TIMEOUT_MS 0U

static RPMsgFrameInstance *s_rpmsgIns;
static CANInstance *s_canIns;
static Robot_Feature_s s_feature;



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
 * @brief Periodic CAN test sender, called from SysTick context.
 */
static void Robot_CanSendTestISR(void)
{
    static uint8_t counter = 0U;
    uint8_t rawPayload[8];

    if (s_canIns == NULL) {
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

    if (CANTransmit(s_canIns, APP_CAN_TX_TIMEOUT_MS) == 0U) {
        return;
    }

    counter++;
}

/**
 * @brief CAN RX callback for test statistics and debug output.
 */
static void Robot_CanCallback(CANInstance *ins)
{
    (void)ins;

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
