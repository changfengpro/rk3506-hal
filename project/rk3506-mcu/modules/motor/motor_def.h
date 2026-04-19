/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 *
 * Motor common definitions.
 *
 * This header follows the naming style of the reference project
 * while keeping only dependency-free definitions for current MCU project.
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include <stdint.h>

#define LIMIT_MIN_MAX(x, min, max) ((x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))))

typedef enum {
	OPEN_LOOP = 0b0000,
	CURRENT_LOOP = 0b0001,
	SPEED_LOOP = 0b0010,
	ANGLE_LOOP = 0b0100,

	SPEED_AND_CURRENT_LOOP = 0b0011,
	ANGLE_AND_SPEED_LOOP = 0b0110,
	ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;

typedef enum {
	FEEDFORWARD_NONE = 0b00,
	CURRENT_FEEDFORWARD = 0b01,
	SPEED_FEEDFORWARD = 0b10,
	CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

typedef enum {
	MOTOR_FEED = 0,
	OTHER_FEED,
} Feedback_Source_e;

typedef enum {
	MOTOR_DIRECTION_NORMAL = 0,
	MOTOR_DIRECTION_REVERSE = 1,
} Motor_Reverse_Flag_e;

typedef enum {
	FEEDBACK_DIRECTION_NORMAL = 0,
	FEEDBACK_DIRECTION_REVERSE = 1,
} Feedback_Reverse_Flag_e;

typedef enum {
	POWET_LIMIT_OFF = 0,
	POWER_LIMIT_ON = 1,
} Power_Limit_Flag_e;

typedef enum {
	MOTOR_STOP = 0,
	MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

typedef enum {
	MOTOR_TYPE_NONE = 0,
	GM6020,
	M3508,
	M2006,
	LK9025,
	HT04,
} Motor_Type_e;

typedef enum {
	SINGLE_ANGLE = 0,
	TOTAL_ANGLE = 1,
} Motor_Close_Type;

#endif /* MOTOR_DEF_H */