/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022-2026 changfengpro
 */

#include "djimotor.h"

#include <string.h>

#include "hal_bsp.h"

#define DJI_TX_GROUP_1FF 0x1FFU
#define DJI_TX_GROUP_200 0x200U
#define DJI_TX_GROUP_2FF 0x2FFU

typedef struct {
	uint8_t used;
	uint8_t motor_id;
	uint8_t motor_type;
	uint8_t control_mode;
	uint8_t tx_group_slot;
	uint32_t rx_id;
	uint32_t tx_group_id;

	CANInstance *rx_can;

	int32_t target_position_q16;
	int32_t target_velocity_q16;
	int16_t target_torque_q8;

	int32_t feedback_position_q16;
	int32_t feedback_velocity_q16;
	int16_t feedback_torque_q8;
	int16_t temperature_c;
	uint32_t last_rx_tick;
	uint16_t status_flags;
} DJIMotorInstance;

typedef struct {
	uint32_t tx_id;
	uint8_t tx_buff[8];
	uint8_t active;
} DJIMotorSender;

static DJIMotorInstance dji_motor_instance[DJI_MOTOR_CNT];
static DJIMotorSender sender_assignment[3] = {
	{ DJI_TX_GROUP_1FF, { 0 }, 0U },
	{ DJI_TX_GROUP_200, { 0 }, 0U },
	{ DJI_TX_GROUP_2FF, { 0 }, 0U },
};

static struct CAN_REG *dji_motor_can_handle;
static uint32_t dji_motor_tx_timeout_ms = DJI_MOTOR_DEFAULT_TX_TIMEOUT_MS;
static uint32_t dji_motor_feedback_timeout_ms = DJI_MOTOR_DEFAULT_FB_TIMEOUT_MS;
static uint8_t dji_motor_init_flag;

static int16_t DJIMotorClampInt16(int32_t value, int16_t min, int16_t max)
{
	if (value < (int32_t)min) {
		return min;
	}
	if (value > (int32_t)max) {
		return max;
	}

	return (int16_t)value;
}

static uint8_t DJIMotorGetTxGroupIndex(uint32_t txGroupId)
{
	if (txGroupId == DJI_TX_GROUP_1FF) {
		return 0U;
	}
	if (txGroupId == DJI_TX_GROUP_200) {
		return 1U;
	}
	if (txGroupId == DJI_TX_GROUP_2FF) {
		return 2U;
	}

	return 0xFFU;
}

static uint8_t MotorSenderGrouping(uint8_t motorType,
						   uint8_t motorId,
						   uint32_t *rxId,
						   uint32_t *txGroupId,
						   uint8_t *txGroupSlot)
{
	if ((rxId == NULL) || (txGroupId == NULL) || (txGroupSlot == NULL)) {
		return 0U;
	}
	if ((motorId == 0U) || (motorId > 8U)) {
		return 0U;
	}

	switch (motorType) {
	case M2006:
	case M3508:
		if (motorId <= 4U) {
			*txGroupId = DJI_TX_GROUP_200;
			*txGroupSlot = (uint8_t)(motorId - 1U);
		} else {
			*txGroupId = DJI_TX_GROUP_1FF;
			*txGroupSlot = (uint8_t)(motorId - 5U);
		}
		*rxId = (uint32_t)(DJI_TX_GROUP_200 + motorId);
		return 1U;

	case GM6020:
		if (motorId <= 4U) {
			*txGroupId = DJI_TX_GROUP_1FF;
			*txGroupSlot = (uint8_t)(motorId - 1U);
		} else {
			*txGroupId = DJI_TX_GROUP_2FF;
			*txGroupSlot = (uint8_t)(motorId - 5U);
		}
		*rxId = (uint32_t)(0x204U + motorId);
		return 1U;

	default:
		return 0U;
	}
}

static void DecodeDJIMotor(CANInstance *ins)
{
	DJIMotorInstance *slot;
	uint16_t ecd;
	int16_t speedRpm;
	int16_t currentRaw;

	if ((ins == NULL) || (ins->id == NULL)) {
		return;
	}
	if (ins->rx_len < 7U) {
		return;
	}

	slot = (DJIMotorInstance *)ins->id;
	ecd = (uint16_t)(((uint16_t)ins->rx_buff[0] << 8) | ins->rx_buff[1]);
	speedRpm = (int16_t)(((uint16_t)ins->rx_buff[2] << 8) | ins->rx_buff[3]);
	currentRaw = (int16_t)(((uint16_t)ins->rx_buff[4] << 8) | ins->rx_buff[5]);

	slot->feedback_position_q16 = ((int32_t)ecd * 360 * (int32_t)RPMSG_FRAME_Q16_SCALE) / 8192;
	slot->feedback_velocity_q16 = (int32_t)speedRpm * 6 * (int32_t)RPMSG_FRAME_Q16_SCALE;
	slot->feedback_torque_q8 = currentRaw;
	slot->temperature_c = (int16_t)ins->rx_buff[6];
	slot->last_rx_tick = HAL_GetTick();
	slot->status_flags |= DJI_MOTOR_STATUS_RX_VALID;
	slot->status_flags &= (uint16_t)(~DJI_MOTOR_STATUS_TIMEOUT);
}

static DJIMotorInstance *DJIMotorFindSlot(uint8_t motorId, uint8_t motorType)
{
	uint8_t i;

	for (i = 0U; i < DJI_MOTOR_CNT; i++) {
		if ((dji_motor_instance[i].used != 0U) &&
			(dji_motor_instance[i].motor_id == motorId) &&
			(dji_motor_instance[i].motor_type == motorType)) {
			return &dji_motor_instance[i];
		}
	}

	return NULL;
}

static DJIMotorInstance *DJIMotorAllocInstance(void)
{
	uint8_t i;

	for (i = 0U; i < DJI_MOTOR_CNT; i++) {
		if (dji_motor_instance[i].used == 0U) {
			memset(&dji_motor_instance[i], 0, sizeof(dji_motor_instance[i]));
			dji_motor_instance[i].used = 1U;
			return &dji_motor_instance[i];
		}
	}

	return NULL;
}

static DJIMotorInstance *DJIMotorRegister(uint8_t motorId, uint8_t motorType)
{
	DJIMotorInstance *slot;
	CAN_Init_Config_s canCfg;
	uint32_t rxId;
	uint32_t txGroupId;
	uint8_t txGroupSlot;

	slot = DJIMotorFindSlot(motorId, motorType);
	if (slot != NULL) {
		return slot;
	}

	if (MotorSenderGrouping(motorType, motorId, &rxId, &txGroupId, &txGroupSlot) == 0U) {
		return NULL;
	}

	slot = DJIMotorAllocInstance();
	if (slot == NULL) {
		return NULL;
	}

	slot->motor_id = motorId;
	slot->motor_type = motorType;
	slot->rx_id = rxId;
	slot->tx_group_id = txGroupId;
	slot->tx_group_slot = txGroupSlot;

	memset(&canCfg, 0, sizeof(canCfg));
	canCfg.can_handle = dji_motor_can_handle;
	canCfg.tx_id = rxId;
	canCfg.rx_id = rxId;
	canCfg.can_module_callback = DecodeDJIMotor;
	canCfg.id = slot;

	slot->rx_can = CANRegister(&canCfg);
	if (slot->rx_can == NULL) {
		slot->used = 0U;
		return NULL;
	}

	slot->status_flags |= DJI_MOTOR_STATUS_REGISTERED;
	return slot;
}

static int16_t DJIMotorResolveOutput(const DJIMotorInstance *slot, uint16_t *statusFlags)
{
	int32_t raw = 0;

	if ((slot == NULL) || (statusFlags == NULL)) {
		return 0;
	}

	if (slot->control_mode == MOTOR_CONTROL_MODE_TORQUE) {
		raw = slot->target_torque_q8;
		*statusFlags = (uint16_t)(*statusFlags & (~DJI_MOTOR_STATUS_MODE_UNSUPPORTED));
	} else {
		raw = 0;
		*statusFlags = (uint16_t)(*statusFlags | DJI_MOTOR_STATUS_MODE_UNSUPPORTED);
	}

	return DJIMotorClampInt16(raw, -DJI_MOTOR_CMD_LIMIT, DJI_MOTOR_CMD_LIMIT);
}

uint8_t DJIMotorInit(const DJIMotor_Init_Config_s *config)
{
	memset(dji_motor_instance, 0, sizeof(dji_motor_instance));

	if ((config != NULL) && (config->can_handle != NULL)) {
		dji_motor_can_handle = config->can_handle;
	} else {
		dji_motor_can_handle = g_can0Dev.pReg;
	}

	if (dji_motor_can_handle == NULL) {
		return 0U;
	}

	if ((config != NULL) && (config->tx_timeout_ms > 0U)) {
		dji_motor_tx_timeout_ms = config->tx_timeout_ms;
	} else {
		dji_motor_tx_timeout_ms = DJI_MOTOR_DEFAULT_TX_TIMEOUT_MS;
	}

	if ((config != NULL) && (config->feedback_timeout_ms > 0U)) {
		dji_motor_feedback_timeout_ms = config->feedback_timeout_ms;
	} else {
		dji_motor_feedback_timeout_ms = DJI_MOTOR_DEFAULT_FB_TIMEOUT_MS;
	}
	dji_motor_init_flag = 1U;

	return 1U;
}

void DJIMotorApplyCommandFrame(const FrameCommand_t *command)
{
	uint8_t motorCount;
	uint8_t i;

	if ((dji_motor_init_flag == 0U) || (command == NULL)) {
		return;
	}

	motorCount = command->motor_count;
	if (motorCount > RPMSG_FRAME_MAX_MOTOR_CNT) {
		motorCount = RPMSG_FRAME_MAX_MOTOR_CNT;
	}

	for (i = 0U; i < motorCount; i++) {
		const MotorCmd_t *cmd = &command->motors[i];
		DJIMotorInstance *slot;

		slot = DJIMotorRegister(cmd->motor_id, cmd->motor_type);
		if (slot == NULL) {
			continue;
		}

		slot->control_mode = cmd->control_mode;
		slot->target_position_q16 = cmd->target_position_q16;
		slot->target_velocity_q16 = cmd->target_velocity_q16;
		slot->target_torque_q8 = cmd->target_torque_q8;
	}
}

void DJIMotorControl(void)
{
	uint8_t i;

	if (dji_motor_init_flag == 0U) {
		return;
	}

	for (i = 0U; i < 3U; i++) {
		memset(sender_assignment[i].tx_buff, 0, sizeof(sender_assignment[i].tx_buff));
		sender_assignment[i].active = 0U;
	}

	for (i = 0U; i < DJI_MOTOR_CNT; i++) {
		uint8_t groupIdx;
		DJIMotorInstance *slot = &dji_motor_instance[i];
		int16_t out;

		if (slot->used == 0U) {
			continue;
		}

		out = DJIMotorResolveOutput(slot, &slot->status_flags);
		groupIdx = DJIMotorGetTxGroupIndex(slot->tx_group_id);
		if (groupIdx >= 3U) {
			continue;
		}

		sender_assignment[groupIdx].active = 1U;
		sender_assignment[groupIdx].tx_buff[2U * slot->tx_group_slot] = (uint8_t)((uint16_t)out >> 8);
		sender_assignment[groupIdx].tx_buff[2U * slot->tx_group_slot + 1U] = (uint8_t)((uint16_t)out & 0xFFU);
	}

	for (i = 0U; i < 3U; i++) {
		CANInstance txIns;
		uint8_t j;

		if (sender_assignment[i].active == 0U) {
			continue;
		}

		memset(&txIns, 0, sizeof(txIns));
		txIns.can_handle = dji_motor_can_handle;
		txIns.tx_id = sender_assignment[i].tx_id;
		txIns.tx_len = 8U;
		memcpy(txIns.tx_buff, sender_assignment[i].tx_buff, sizeof(sender_assignment[i].tx_buff));

		if (CANTransmit(&txIns, dji_motor_tx_timeout_ms) == 0U) {
			for (j = 0U; j < DJI_MOTOR_CNT; j++) {
				if ((dji_motor_instance[j].used != 0U) && (dji_motor_instance[j].tx_group_id == sender_assignment[i].tx_id)) {
					dji_motor_instance[j].status_flags |= DJI_MOTOR_STATUS_TX_FAIL;
				}
			}
		} else {
			for (j = 0U; j < DJI_MOTOR_CNT; j++) {
				if ((dji_motor_instance[j].used != 0U) && (dji_motor_instance[j].tx_group_id == sender_assignment[i].tx_id)) {
					dji_motor_instance[j].status_flags &= (uint16_t)(~DJI_MOTOR_STATUS_TX_FAIL);
				}
			}
		}
	}
}

uint8_t DJIMotorFillTelemetry(FrameTelemetry_t *stateFrame)
{
	uint8_t i;
	uint8_t count = 0U;

	if ((dji_motor_init_flag == 0U) || (stateFrame == NULL)) {
		return 0U;
	}

	for (i = 0U; i < DJI_MOTOR_CNT; i++) {
		DJIMotorInstance *slot = &dji_motor_instance[i];
		MotorState_t *dst;
		uint16_t flags;

		if (slot->used == 0U) {
			continue;
		}
		if (count >= RPMSG_FRAME_MAX_MOTOR_CNT) {
			break;
		}

		dst = &stateFrame->motors[count++];
		flags = slot->status_flags;

		if ((HAL_GetTick() - slot->last_rx_tick) > dji_motor_feedback_timeout_ms) {
			flags |= DJI_MOTOR_STATUS_TIMEOUT;
		} else {
			flags &= (uint16_t)(~DJI_MOTOR_STATUS_TIMEOUT);
		}

		dst->motor_id = slot->motor_id;
		dst->position_q16 = slot->feedback_position_q16;
		dst->velocity_q16 = slot->feedback_velocity_q16;
		dst->torque_q8 = slot->feedback_torque_q8;
		dst->temperature_c = slot->temperature_c;
		dst->status_flags = flags;
	}

	stateFrame->motor_count = count;
	return count;
}
