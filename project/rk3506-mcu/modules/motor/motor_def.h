/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include <stdint.h>

#include "bsp_can.h"
#include "controller.h"

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

typedef struct {
	Closeloop_Type_e outer_loop_type;
	Closeloop_Type_e close_loop_type;
	Motor_Reverse_Flag_e motor_reverse_flag;
	Feedback_Reverse_Flag_e feedback_reverse_flag;
	Feedback_Source_e angle_feedback_source;
	Feedback_Source_e speed_feedback_source;
	Feedfoward_Type_e feedforward_flag;
	Power_Limit_Flag_e power_limit_flag;
} Motor_Control_Setting_s;

typedef struct {
	float *other_angle_feedback_ptr;
	float *other_speed_feedback_ptr;
	float *speed_feedforward_ptr;
	float *current_feedforward_ptr;

	PIDInstance current_PID;
	PIDInstance speed_PID;
	PIDInstance angle_PID;

	float pid_ref;
	float pid_output;
	float pid_output_limit;
} Motor_Controller_s;

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

typedef struct {
	float *other_angle_feedback_ptr;
	float *other_speed_feedback_ptr;

	float *speed_feedforward_ptr;
	float *current_feedforward_ptr;

	PID_Init_Config_s current_PID;
	PID_Init_Config_s speed_PID;
	PID_Init_Config_s angle_PID;
} Motor_Controller_Init_s;

typedef struct {
	Motor_Controller_Init_s controller_param_init_config;
	Motor_Control_Setting_s controller_setting_init_config;
	Motor_Type_e motor_type;
	CAN_Init_Config_s can_init_config;
	Motor_Close_Type motor_close_type;
} Motor_Init_Config_s;

#endif /* MOTOR_DEF_H */