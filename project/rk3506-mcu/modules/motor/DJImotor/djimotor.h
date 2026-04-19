/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 *
 * DJI motor module.
 *
 * Expected information flow:
 * 1) BSP CAN receives feedback and dispatches callbacks to this module.
 * 2) This module maintains target/feedback states and sends grouped DJI frames.
 * 3) Robot layer only feeds command frames and fetches telemetry from module APIs.
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "rpmsg_frame.h"

#define DJI_MOTOR_CNT                  4U
#define DJI_MOTOR_CMD_LIMIT            16384
#define DJI_MOTOR_DEFAULT_TX_TIMEOUT_MS 1U
#define DJI_MOTOR_DEFAULT_FB_TIMEOUT_MS 100U

#define DJI_MOTOR_STATUS_REGISTERED       (1U << 0)
#define DJI_MOTOR_STATUS_RX_VALID         (1U << 1)
#define DJI_MOTOR_STATUS_TIMEOUT          (1U << 2)
#define DJI_MOTOR_STATUS_MODE_UNSUPPORTED (1U << 3)
#define DJI_MOTOR_STATUS_TX_FAIL          (1U << 4)

typedef struct {
	struct CAN_REG *can_handle;
	uint32_t tx_timeout_ms;
	uint32_t feedback_timeout_ms;
} DJIMotor_Init_Config_s;

/**
 * @brief Initialize DJI motor module service.
 *
 * @param config Initialization parameters. If NULL, defaults are used.
 * @return 1 on success, 0 on failure.
 */
uint8_t DJIMotorInit(const DJIMotor_Init_Config_s *config);

/**
 * @brief Feed the latest RPMsg command frame into DJI module.
 *
 * The module stores target values per motor and creates RX registrations
 * on demand according to motor ID/type.
 */
void DJIMotorApplyCommandFrame(const FrameCommand_t *command);

/**
 * @brief Run one control step: pack and send grouped DJI CAN command frames.
 */
void DJIMotorControl(void);

/**
 * @brief Fill telemetry motors[] from module feedback cache.
 *
 * @param stateFrame Output telemetry frame buffer.
 * @return Number of motors written into stateFrame->motors.
 */
uint8_t DJIMotorFillTelemetry(FrameTelemetry_t *stateFrame);

#endif /* DJI_MOTOR_H */
