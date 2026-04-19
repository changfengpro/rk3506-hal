/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>

#include "bsp_can.h"
#include "controller.h"
#include "../motor_def.h"

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF   0.85f
#define CURRENT_SMOOTH_COEF 0.9f
#define ECD_ANGLE_COEF_DJI  0.043945f

/* DJI电机CAN反馈信息 */
typedef struct {
	uint16_t last_ecd;
	uint16_t ecd;
	float angle_single_round;
	float speed_aps;
	int16_t real_current;
	uint8_t temperature;
	float speed_rpm;

	float total_angle;
	int32_t total_round;
} DJI_Motor_Measure_s;

typedef struct {
	DJI_Motor_Measure_s measure;
	Motor_Control_Setting_s motor_settings;
	Motor_Controller_s motor_controller;

	CANInstance *motor_can_instance;
	uint8_t sender_group;
	uint8_t message_num;

	Motor_Type_e motor_type;
	Motor_Working_Type_e stop_flag;

	void *daemon;
	Motor_Close_Type motor_close_type;
	uint32_t feed_cnt;
	float dt;
} DJIMotorInstance;

DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void DJIMotorControl(void);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);
void DJIMotorSetOutputLimit(DJIMotorInstance *motor, float output_limit);

#endif /* DJI_MOTOR_H */
