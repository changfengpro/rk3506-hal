/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 */

#include "robot.h"

#include <string.h>

#include "bsp_init.h"
#include "djimotor.h"
#include "rpmsg_frame.h"

#define ROBOT_CMD_LOCAL_EPT        0x4003U
#define ROBOT_LINUX_LOCAL_EPT      1024U
#define ROBOT_CMD_SERVICE_NAME     "rpmsg-mcu0-test"
#define ROBOT_TELEMETRY_PERIOD_MS  2U
#define ROBOT_RPMSG_TX_TIMEOUT_MS  2U
#define ROBOT_CMD_PEER_TIMEOUT_MS  20U
#define ROBOT_CMD_ACTIVE_WINDOW_MS 200U
#define ROBOT_CMD_ACTIVE_STREAK    2U

extern const struct HAL_CANFD_DEV g_can0Dev;

static RPMsgFrameInstance *robot_cmd_frame_instance;
static DJIMotorInstance *robot_motor_instance[RPMSG_FRAME_MAX_MOTOR_CNT];
static uint8_t robot_motor_count;
static uint32_t robot_last_telemetry_tick;
static uint32_t robot_last_command_tick;
static uint32_t robot_prev_command_tick;
static uint8_t robot_cmd_peer_active;
static uint8_t robot_cmd_valid_streak;
static uint8_t robot_cmd_need_telemetry;
static uint8_t robot_cmd_pending;
static uint8_t robot_init_finished;

static void RobotCMDTask(void);
static void RobotMotorControlTask(void);
static void RobotTelemetryTask(void);

static uint8_t RobotMotorTypeSupported(uint8_t motor_type)
{
    switch ((Motor_Type_e)motor_type) {
    case GM6020:
    case M3508:
    case M2006:
        return 1U;
    default:
        return 0U;
    }
}

static void RobotDeactivateCmdPeer(void)
{
    if ((robot_cmd_frame_instance == NULL) ||
        (robot_cmd_frame_instance->rpmsg_ins == NULL)) {
        robot_cmd_peer_active = 0U;
        return;
    }

    RPMsg_SetRemoteEndpoint(robot_cmd_frame_instance->rpmsg_ins,
                            RPMSG_REMOTE_EPT_DYNAMIC);
    robot_cmd_frame_instance->rpmsg_ins->last_rx_src = RPMSG_REMOTE_EPT_DYNAMIC;
    robot_cmd_peer_active = 0U;
    robot_cmd_valid_streak = 0U;
    robot_cmd_need_telemetry = 0U;
    robot_cmd_pending = 0U;
}

static float RobotQ16ToFloat(int32_t raw)
{
    return (float)raw / (float)RPMSG_FRAME_Q16_SCALE;
}

static float RobotQ8ToFloat(int16_t raw)
{
    return (float)raw / (float)RPMSG_FRAME_Q8_SCALE;
}

static int32_t RobotFloatToQ16(float value)
{
    float scaled = value * (float)RPMSG_FRAME_Q16_SCALE;

    if (scaled >= 2147483647.0f) {
        return 2147483647;
    }
    if (scaled <= -2147483648.0f) {
        return (-2147483647 - 1);
    }

    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int32_t)scaled;
}

static int16_t RobotFloatToQ8(float value)
{
    float scaled = value * (float)RPMSG_FRAME_Q8_SCALE;

    if (scaled >= 32767.0f) {
        return 32767;
    }
    if (scaled <= -32768.0f) {
        return -32768;
    }

    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)scaled;
}

static void RobotFillPIDConfig(PID_Init_Config_s *pid, float max_out)
{
    if (pid == NULL) {
        return;
    }

    memset(pid, 0, sizeof(*pid));
    pid->Kp = 1.0f;
    pid->MaxOut = max_out;
    pid->IntegralLimit = max_out;
    pid->Improve = PID_IMPROVE_NONE;
}

static DJIMotorInstance *RobotFindMotor(uint8_t motor_id, uint8_t motor_type)
{
    uint8_t i;

    for (i = 0U; i < robot_motor_count; i++) {
        DJIMotorInstance *motor = robot_motor_instance[i];

        if ((motor == NULL) || (motor->motor_can_instance == NULL)) {
            continue;
        }
        if ((motor->motor_type == (Motor_Type_e)motor_type) &&
            (motor->motor_can_instance->tx_id == motor_id)) {
            return motor;
        }
    }

    return NULL;
}

static DJIMotorInstance *RobotRegisterMotor(uint8_t motor_id, uint8_t motor_type)
{
    Motor_Init_Config_s config;
    DJIMotorInstance *motor;

    if ((motor_id == 0U) || (motor_id > 8U)) {
        return NULL;
    }
    if (robot_motor_count >= RPMSG_FRAME_MAX_MOTOR_CNT) {
        return NULL;
    }

    memset(&config, 0, sizeof(config));

    config.motor_type = (Motor_Type_e)motor_type;
    config.motor_close_type = TOTAL_ANGLE;

    config.controller_setting_init_config.outer_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    config.controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE;
    config.controller_setting_init_config.power_limit_flag = POWET_LIMIT_OFF;

    RobotFillPIDConfig(&config.controller_param_init_config.current_PID, 16384.0f);
    RobotFillPIDConfig(&config.controller_param_init_config.speed_PID, 16384.0f);
    RobotFillPIDConfig(&config.controller_param_init_config.angle_PID, 16384.0f);

    config.can_init_config.can_handle = g_can0Dev.pReg;
    config.can_init_config.tx_id = motor_id;

    motor = DJIMotorInit(&config);
    if (motor == NULL) {
        return NULL;
    }

    robot_motor_instance[robot_motor_count++] = motor;
    return motor;
}

static void RobotApplyControlMode(DJIMotorInstance *motor, uint8_t control_mode)
{
    if (motor == NULL) {
        return;
    }

    switch ((Motor_Control_Mode_e)control_mode) {
    case MOTOR_CONTROL_MODE_POSITION:
        motor->motor_settings.close_loop_type = ANGLE_AND_SPEED_LOOP;
        DJIMotorOuterLoop(motor, ANGLE_LOOP);
        break;
    case MOTOR_CONTROL_MODE_VELOCITY:
        motor->motor_settings.close_loop_type = SPEED_LOOP;
        DJIMotorOuterLoop(motor, SPEED_LOOP);
        break;
    case MOTOR_CONTROL_MODE_TORQUE:
    case MOTOR_CONTROL_MODE_NONE:
    default:
        motor->motor_settings.close_loop_type = OPEN_LOOP;
        DJIMotorOuterLoop(motor, OPEN_LOOP);
        break;
    }
}

static void RobotCMDCallback(RPMsgFrameInstance *instance)
{
    (void)instance;
    robot_cmd_pending = 1U;
}

static uint8_t RobotCMDInit(void)
{
    RPMsgFrame_Init_Config_s frameConfig;

    if (robot_cmd_frame_instance != NULL) {
        return 1U;
    }

    memset(&frameConfig, 0, sizeof(frameConfig));
    frameConfig.local_ept = ROBOT_CMD_LOCAL_EPT;
    frameConfig.remote_ept = ROBOT_LINUX_LOCAL_EPT;
    frameConfig.ept_name = ROBOT_CMD_SERVICE_NAME;
    frameConfig.state_motor_count = 0U;
    frameConfig.command_callback = RobotCMDCallback;

    robot_cmd_frame_instance = RPMsgFrameInit(&frameConfig);
    if (robot_cmd_frame_instance == NULL) {
        return 0U;
    }

    return 1U;
}

static void RobotCMDTask(void)
{
    FrameCommand_t command;
    uint8_t i;
    uint8_t motorCount;
    uint32_t now;

    if (robot_cmd_frame_instance == NULL) {
        return;
    }

    if (robot_cmd_pending == 0U) {
        return;
    }

    if (RPMsgFrameGetCommandFrame(robot_cmd_frame_instance, &command, 1U) == 0U) {
        return;
    }
    robot_cmd_pending = 0U;

    now = HAL_GetTick();

    if ((robot_prev_command_tick == 0U) ||
        ((now - robot_prev_command_tick) > ROBOT_CMD_ACTIVE_WINDOW_MS)) {
        robot_cmd_valid_streak = 1U;
    } else if (robot_cmd_valid_streak < 0xFFU) {
        robot_cmd_valid_streak++;
    }

    robot_prev_command_tick = now;
    robot_last_command_tick = now;
    if (robot_cmd_valid_streak >= ROBOT_CMD_ACTIVE_STREAK) {
        robot_cmd_peer_active = 1U;
        robot_cmd_need_telemetry = 1U;
    }

    motorCount = command.motor_count;
    if (motorCount > RPMSG_FRAME_MAX_MOTOR_CNT) {
        motorCount = RPMSG_FRAME_MAX_MOTOR_CNT;
    }

    for (i = 0U; i < motorCount; i++) {
        const MotorCmd_t *cmd = &command.motors[i];
        DJIMotorInstance *motor;
        float ref = 0.0f;

        if (RobotMotorTypeSupported(cmd->motor_type) == 0U) {
            static uint32_t last_type_warn_tick;

            if ((now - last_type_warn_tick) >= 1000U) {
                HAL_DBG_ERR("[robot] unsupported motor_type=%u id=%u, skip\n",
                            (unsigned int)cmd->motor_type,
                            (unsigned int)cmd->motor_id);
                last_type_warn_tick = now;
            }
            continue;
        }

        motor = RobotFindMotor(cmd->motor_id, cmd->motor_type);
        if (motor == NULL) {
            motor = RobotRegisterMotor(cmd->motor_id, cmd->motor_type);
        }
        if (motor == NULL) {
            continue;
        }

        RobotApplyControlMode(motor, cmd->control_mode);
        switch ((Motor_Control_Mode_e)cmd->control_mode) {
        case MOTOR_CONTROL_MODE_POSITION:
            ref = RobotQ16ToFloat(cmd->target_position_q16);
            break;
        case MOTOR_CONTROL_MODE_VELOCITY:
            ref = RobotQ16ToFloat(cmd->target_velocity_q16);
            break;
        case MOTOR_CONTROL_MODE_TORQUE:
            ref = RobotQ8ToFloat(cmd->target_torque_q8);
            break;
        case MOTOR_CONTROL_MODE_NONE:
        default:
            ref = 0.0f;
            break;
        }

        DJIMotorSetRef(motor, ref);
    }
}

static void RobotMotorControlTask(void)
{
    DJIMotorControl();
}

static void RobotTelemetryTask(void)
{
    FrameTelemetry_t *stateFrame;
    uint8_t count = 0U;
    uint8_t i;
    uint32_t now;

    if (robot_cmd_frame_instance == NULL) {
        return;
    }

    now = HAL_GetTick();
    if (robot_cmd_peer_active == 0U) {
        return;
    }

    if ((now - robot_last_command_tick) > ROBOT_CMD_PEER_TIMEOUT_MS) {
        RobotDeactivateCmdPeer();
        return;
    }

    if (robot_cmd_need_telemetry == 0U) {
        return;
    }

    if (RPMsg_IsLinkUp() == 0U) {
        RobotDeactivateCmdPeer();
        return;
    }

    if ((now - robot_last_telemetry_tick) < ROBOT_TELEMETRY_PERIOD_MS) {
        return;
    }
    robot_last_telemetry_tick = now;

    RPMsgFrameResetStateFrame(robot_cmd_frame_instance);
    stateFrame = RPMsgFrameGetStateFrame(robot_cmd_frame_instance);
    if (stateFrame == NULL) {
        return;
    }

    for (i = 0U; i < robot_motor_count; i++) {
        DJIMotorInstance *motor = robot_motor_instance[i];

        if (motor == NULL || motor->motor_can_instance == NULL) {
            continue;
        }
        if (count >= RPMSG_FRAME_MAX_MOTOR_CNT) {
            break;
        }

        stateFrame->motors[count].motor_id = (uint8_t)motor->motor_can_instance->tx_id;
        stateFrame->motors[count].position_q16 = RobotFloatToQ16(motor->measure.total_angle);
        stateFrame->motors[count].velocity_q16 = RobotFloatToQ16(motor->measure.speed_aps);
        stateFrame->motors[count].torque_q8 = RobotFloatToQ8((float)motor->measure.real_current);
        stateFrame->motors[count].temperature_c = (int16_t)motor->measure.temperature;
        stateFrame->motors[count].status_flags =
            (motor->stop_flag == MOTOR_STOP) ? 1U : 0U;
        count++;
    }

    stateFrame->motor_count = count;

    if (RPMsgFrameTransmitStateFrame(robot_cmd_frame_instance,
                                     now,
                                     ROBOT_RPMSG_TX_TIMEOUT_MS) == 0U) {
        RobotDeactivateCmdPeer();
        return;
    }

    robot_cmd_need_telemetry = 0U;
}

void RobotInit(void)
{
    uint8_t cmdInitOk;

    __disable_irq();

    BSPInit();

    memset(robot_motor_instance, 0, sizeof(robot_motor_instance));
    robot_motor_count = 0U;

    cmdInitOk = RobotCMDInit();
    HAL_ASSERT(cmdInitOk == 1U);
    if (cmdInitOk == 0U) {
        HAL_DBG_ERR("RobotCMDInit failed\n");
    }

    robot_last_telemetry_tick = HAL_GetTick();
    robot_last_command_tick = robot_last_telemetry_tick;
    robot_prev_command_tick = 0U;
    robot_cmd_peer_active = 0U;
    robot_cmd_valid_streak = 0U;
    robot_cmd_need_telemetry = 0U;
    robot_cmd_pending = 0U;
    robot_init_finished = 1U;

    __enable_irq();
}

void RobotTask(void)
{
    if (robot_init_finished == 0U) {
        return;
    }

    RobotCMDTask();
    RobotMotorControlTask();
    RobotTelemetryTask();
}
