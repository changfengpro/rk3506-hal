/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 */

#include "djimotor.h"

#include <stdlib.h>
#include <string.h>

#include "hal_bsp.h"

#define RPM_2_ANGLE_PER_SEC 6.0f

extern const struct HAL_CANFD_DEV g_can0Dev;
extern const struct HAL_CANFD_DEV g_can1Dev;

static uint8_t idx = 0;
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = { NULL };

static CANInstance sender_assignment[6] = {
	[0] = { 0 },
	[1] = { 0 },
	[2] = { 0 },
	[3] = { 0 },
	[4] = { 0 },
	[5] = { 0 },
};

static uint8_t sender_enable_flag[9] = { 0 };
static uint8_t sender_assignment_inited = 0;

static float DJIMotorGetDeltaT(uint32_t *last_tick)
{
	uint32_t now;
	uint32_t delta_ms;

	if (last_tick == NULL) {
		return 0.001f;
	}

	now = HAL_GetTick();
	if (*last_tick == 0U) {
		*last_tick = now;
		return 0.001f;
	}

	delta_ms = now - *last_tick;
	*last_tick = now;

	if (delta_ms == 0U) {
		return 0.001f;
	}

	return (float)delta_ms * 0.001f;
}

static void DJIMotorInitSenderAssignment(void)
{
	if (sender_assignment_inited != 0U) {
		return;
	}

	sender_assignment[0].can_handle = g_can0Dev.pReg;
	sender_assignment[0].tx_id = 0x1ffU;
	sender_assignment[0].tx_len = 8U;

	sender_assignment[1].can_handle = g_can0Dev.pReg;
	sender_assignment[1].tx_id = 0x200U;
	sender_assignment[1].tx_len = 8U;

	sender_assignment[2].can_handle = g_can0Dev.pReg;
	sender_assignment[2].tx_id = 0x2ffU;
	sender_assignment[2].tx_len = 8U;

	sender_assignment[3].can_handle = g_can1Dev.pReg;
	sender_assignment[3].tx_id = 0x1ffU;
	sender_assignment[3].tx_len = 8U;

	sender_assignment[4].can_handle = g_can1Dev.pReg;
	sender_assignment[4].tx_id = 0x200U;
	sender_assignment[4].tx_len = 8U;

	sender_assignment[5].can_handle = g_can1Dev.pReg;
	sender_assignment[5].tx_id = 0x2ffU;
	sender_assignment[5].tx_len = 8U;

	sender_assignment_inited = 1U;
}

static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
	uint8_t motor_id;
	uint8_t motor_send_num;
	uint8_t motor_grouping;
	uint8_t grouping_offset;

	if ((motor == NULL) || (config == NULL) || (config->tx_id == 0U)) {
		return;
	}

	motor_id = (uint8_t)(config->tx_id - 1U);
	if (config->can_handle == g_can0Dev.pReg) {
		grouping_offset = 0U;
	} else if (config->can_handle == g_can1Dev.pReg) {
		grouping_offset = 3U;
	} else {
		grouping_offset = 6U;
	}

	switch (motor->motor_type) {
	case M2006:
	case M3508:
		if (motor_id < 4U) {
			motor_send_num = motor_id;
			motor_grouping = (uint8_t)(grouping_offset + 1U);
		} else {
			motor_send_num = (uint8_t)(motor_id - 4U);
			motor_grouping = (uint8_t)(grouping_offset + 0U);
		}

		config->rx_id = (uint32_t)(0x200U + motor_id + 1U);
		sender_enable_flag[motor_grouping] = 1U;
		motor->message_num = motor_send_num;
		motor->sender_group = motor_grouping;

		for (size_t i = 0; i < idx; ++i) {
			if ((dji_motor_instance[i] != NULL) &&
				(dji_motor_instance[i]->motor_can_instance != NULL) &&
				(dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle) &&
				(dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)) {
				HAL_DBG_ERR("[dji_motor] ID crash, rx_id=%lu\n", (unsigned long)config->rx_id);
				while (1) {
				}
			}
		}
		break;

	case GM6020:
		if (motor_id < 4U) {
			motor_send_num = motor_id;
			motor_grouping = (uint8_t)(grouping_offset + 0U);
		} else {
			motor_send_num = (uint8_t)(motor_id - 4U);
			motor_grouping = (uint8_t)(grouping_offset + 2U);
		}

		config->rx_id = (uint32_t)(0x204U + motor_id + 1U);
		sender_enable_flag[motor_grouping] = 1U;
		motor->message_num = motor_send_num;
		motor->sender_group = motor_grouping;

		for (size_t i = 0; i < idx; ++i) {
			if ((dji_motor_instance[i] != NULL) &&
				(dji_motor_instance[i]->motor_can_instance != NULL) &&
				(dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle) &&
				(dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)) {
				HAL_DBG_ERR("[dji_motor] ID crash, rx_id=%lu\n", (unsigned long)config->rx_id);
				while (1) {
				}
			}
		}
		break;

	default:
		HAL_DBG_ERR("[dji_motor] unsupported motor type=%d\n", (int)motor->motor_type);
		while (1) {
		}
	}
}

static void DecodeDJIMotor(CANInstance *instance)
{
	uint8_t *rxbuff;
	DJIMotorInstance *motor;
	DJI_Motor_Measure_s *measure;

	if ((instance == NULL) || (instance->id == NULL) || (instance->rx_len < 7U)) {
		return;
	}

	rxbuff = instance->rx_buff;
	motor = (DJIMotorInstance *)instance->id;
	measure = &motor->measure;

	motor->dt = DJIMotorGetDeltaT(&motor->feed_cnt);

	measure->last_ecd = measure->ecd;
	measure->ecd = (uint16_t)(((uint16_t)rxbuff[0] << 8) | rxbuff[1]);
	measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
	measure->speed_rpm = (float)((int16_t)(((uint16_t)rxbuff[2] << 8) | rxbuff[3]));
	measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
		RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * measure->speed_rpm;
	measure->real_current = (int16_t)((1.0f - CURRENT_SMOOTH_COEF) * (float)measure->real_current +
		CURRENT_SMOOTH_COEF * (float)((int16_t)(((uint16_t)rxbuff[4] << 8) | rxbuff[5])));
	measure->temperature = rxbuff[6];

	if ((int32_t)measure->ecd - (int32_t)measure->last_ecd > 4096) {
		measure->total_round--;
	} else if ((int32_t)measure->ecd - (int32_t)measure->last_ecd < -4096) {
		measure->total_round++;
	}
	measure->total_angle = (float)measure->total_round * 360.0f + measure->angle_single_round;
}

DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
	DJIMotorInstance *instance;

	if ((config == NULL) || (idx >= DJI_MOTOR_CNT)) {
		return NULL;
	}

	DJIMotorInitSenderAssignment();

	instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
	if (instance == NULL) {
		return NULL;
	}
	memset(instance, 0, sizeof(DJIMotorInstance));

	instance->motor_type = config->motor_type;
	instance->motor_settings = config->controller_setting_init_config;
	instance->motor_close_type = config->motor_close_type;

	PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
	PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
	PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
	instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
	instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
	instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
	instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

	MotorSenderGrouping(instance, &config->can_init_config);

	config->can_init_config.can_module_callback = DecodeDJIMotor;
	config->can_init_config.id = instance;
	instance->motor_can_instance = CANRegister(&config->can_init_config);
	if (instance->motor_can_instance == NULL) {
		free(instance);
		return NULL;
	}

	DJIMotorEnable(instance);
	dji_motor_instance[idx++] = instance;
	return instance;
}

void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
	if (motor == NULL) {
		return;
	}

	if (loop == ANGLE_LOOP) {
		motor->motor_settings.angle_feedback_source = type;
	} else if (loop == SPEED_LOOP) {
		motor->motor_settings.speed_feedback_source = type;
	} else {
		HAL_DBG_ERR("[dji_motor] loop type error\n");
	}
}

void DJIMotorStop(DJIMotorInstance *motor)
{
	if (motor == NULL) {
		return;
	}

	motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
	if (motor == NULL) {
		return;
	}

	motor->stop_flag = MOTOR_ENALBED;
}

void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
	if (motor == NULL) {
		return;
	}

	motor->motor_settings.outer_loop_type = outer_loop;
}

void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
	if (motor == NULL) {
		return;
	}

	motor->motor_controller.pid_ref = ref;
}

void DJIMotorSetOutputLimit(DJIMotorInstance *motor, float output_limit)
{
	if (motor == NULL) {
		return;
	}

	motor->motor_controller.pid_output_limit = output_limit;
}

void DJIMotorControl(void)
{
	uint8_t group;
	uint8_t num;
	int16_t set;
	DJIMotorInstance *motor;
	Motor_Control_Setting_s *motor_setting;
	Motor_Controller_s *motor_controller;
	DJI_Motor_Measure_s *measure;
	float pid_measure;
	float pid_ref;

	for (size_t i = 0; i < idx; ++i) {
		motor = dji_motor_instance[i];
		if (motor == NULL) {
			continue;
		}

		motor_setting = &motor->motor_settings;
		motor_controller = &motor->motor_controller;
		measure = &motor->measure;
		pid_ref = motor_controller->pid_ref;

		if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
			pid_ref *= -1.0f;
		}

		if ((motor_setting->close_loop_type & ANGLE_LOOP) &&
			(motor_setting->outer_loop_type == ANGLE_LOOP)) {
			if ((motor_setting->angle_feedback_source == OTHER_FEED) &&
				(motor_controller->other_angle_feedback_ptr != NULL)) {
				pid_measure = *motor_controller->other_angle_feedback_ptr;
			} else {
				if (motor->motor_close_type == SINGLE_ANGLE) {
					pid_measure = measure->angle_single_round;
				} else {
					pid_measure = measure->total_angle;
				}
			}

			pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
		}

		if ((motor_setting->close_loop_type & SPEED_LOOP) &&
			(motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
			if ((motor_setting->feedforward_flag & SPEED_FEEDFORWARD) &&
				(motor_controller->speed_feedforward_ptr != NULL)) {
				pid_ref += *motor_controller->speed_feedforward_ptr;
			}

			if ((motor_setting->speed_feedback_source == OTHER_FEED) &&
				(motor_controller->other_speed_feedback_ptr != NULL)) {
				pid_measure = *motor_controller->other_speed_feedback_ptr;
			} else {
				pid_measure = measure->speed_aps;
			}

			pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
		}

		if ((motor_setting->feedforward_flag & CURRENT_FEEDFORWARD) &&
			(motor_controller->current_feedforward_ptr != NULL)) {
			pid_ref += *motor_controller->current_feedforward_ptr;
		}
		if (motor_setting->close_loop_type & CURRENT_LOOP) {
			pid_ref = PIDCalculate(&motor_controller->current_PID, (float)measure->real_current, pid_ref);
		}

		if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
			pid_ref *= -1.0f;
		}

		motor_controller->pid_output = pid_ref;
		set = (int16_t)pid_ref;

		if (motor_setting->power_limit_flag == POWER_LIMIT_ON) {
			set = (int16_t)motor_controller->pid_output_limit;
		}

		group = motor->sender_group;
		num = motor->message_num;
		if (group >= 6U || num >= 4U) {
			continue;
		}

		sender_assignment[group].tx_buff[2U * num] = (uint8_t)(((uint16_t)set) >> 8);
		sender_assignment[group].tx_buff[2U * num + 1U] = (uint8_t)(((uint16_t)set) & 0x00ffU);

		if (motor->stop_flag == MOTOR_STOP) {
			memset(sender_assignment[group].tx_buff + 2U * num, 0, 2U);
		}
	}

	for (size_t i = 0; i < 6U; ++i) {
		if (sender_enable_flag[i] != 0U) {
			(void)CANTransmit(&sender_assignment[i], 1U);
		}
	}
}
