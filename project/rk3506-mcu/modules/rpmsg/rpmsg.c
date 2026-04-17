/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "rpmsg.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static uint8_t idx = 0U; /* register idx，模块全局实例索引，在注册时使用 */
/* RPMsg 实例池，此处仅保存指针，实例内存在 RPMsgFrameInit 时通过 malloc 分配 */
static RPMsgFrameInstance *rpmsg_frame_instance[RPMSG_FRAME_MX_REGISTER_CNT] = { NULL };


/**
 * @brief 计算 CRC16-CCITT(FALSE) 校验值。
 * @param data 输入数据。
 * @param len 数据长度。
 * @return CRC16 校验值。
 */
static uint16_t RPMsgFrameCalcCrc16(const uint8_t *data, uint32_t len)
{
	uint16_t crc = 0xFFFFU;
	uint32_t i;

	if (data == NULL) {
		return 0U;
	}

	for (i = 0U; i < len; i++) {
		uint8_t bit;

		crc ^= (uint16_t)((uint16_t)data[i] << 8);
		for (bit = 0U; bit < 8U; bit++) {
			if ((crc & 0x8000U) != 0U) {
				crc = (uint16_t)((crc << 1) ^ 0x1021U);
			} else {
				crc = (uint16_t)(crc << 1);
			}
		}
	}

	return crc;
}

/**
 * @brief 组装并计算遥测帧 CRC。
 * @param instance RPMsg 业务实例。
 * @param timestamp_ms 时间戳。
 */
static void PackRPMsgTelemetryFrame(RPMsgFrameInstance *instance, uint32_t timestamp_ms)
{
	FrameTelemetry_t *frame;

	if (instance == NULL) {
		return;
	}

	frame = &instance->tx_frame;

	frame->seq = instance->tx_seq++;
	frame->timestamp_ms = timestamp_ms;
	if (frame->motor_count > RPMSG_FRAME_MAX_MOTOR_CNT) {
		frame->motor_count = RPMSG_FRAME_MAX_MOTOR_CNT;
	}
	frame->crc16 = RPMsgFrameCalcCrc16((const uint8_t *)frame,
										(uint32_t)offsetof(FrameTelemetry_t, crc16));
}

/**
 * @brief 解码并校验命令帧。
 * @param instance RPMsg 业务实例。
 * @param payload 输入负载。
 * @param payload_len 输入长度。
 * @return 校验并更新成功返回 1，否则返回 0。
 */
static uint8_t DecodeRPMsgCommandFrame(RPMsgFrameInstance *instance,
									   const uint8_t *payload,
									   uint32_t payload_len)
{
	FrameCommand_t frame;
	uint16_t crcExpect;
	uint16_t crcCalc;

	if ((instance == NULL) || (payload == NULL)) {
		return 0U;
	}

	if (payload_len != sizeof(FrameCommand_t)) {
		return 0U;
	}

	memcpy(&frame, payload, sizeof(frame));

	if (frame.motor_count > RPMSG_FRAME_MAX_MOTOR_CNT) {
		return 0U;
	}

	crcExpect = frame.crc16;
	crcCalc = RPMsgFrameCalcCrc16((const uint8_t *)&frame,
								   (uint32_t)offsetof(FrameCommand_t, crc16));
	if (crcCalc != crcExpect) {
		return 0U;
	}

	instance->rx_frame = frame;
	instance->last_rx_seq = frame.seq;

	return 1U;
}

/**
 * @brief RPMsg 接收回调适配，负责命令帧解包与分发。
 * @param ins BSP RPMsg 实例。
 */
static void DecodeRPMsgFrame(RPMsgInstance *_instance)
{
	RPMsgFrameInstance *instance;

	if ((_instance == NULL) || (_instance->id == NULL)) {
		return;
	}

	instance = (RPMsgFrameInstance *)_instance->id;

	if (DecodeRPMsgCommandFrame(instance, _instance->rx_buff, _instance->rx_len) == 0U) {
		instance->rx_drop_cnt++;
		return;
	}

	instance->rx_ok_cnt++;
	instance->new_command_ready = 1U;

	if (instance->rpmsg_frame_callback != NULL) {
		instance->rpmsg_frame_callback(instance);
	}
}

/**
 * @brief 初始化并注册一个 RPMsg 业务实例。
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
RPMsgFrameInstance *RPMsgFrameInit(RPMsgFrame_Init_Config_s *config)
{
	RPMsgFrameInstance *instance;
	RPMsg_Init_Config_s rpmsgConfig;

	if (config == NULL) {
		return NULL;
	}

	if (idx >= RPMSG_FRAME_MX_REGISTER_CNT) {
		HAL_DBG_ERR("RPMsg Frame instance pool exhausted.\n");
		return NULL;
	}

	instance = (RPMsgFrameInstance *)malloc(sizeof(RPMsgFrameInstance));
	if (instance == NULL) {
		HAL_DBG_ERR("RPMsg Frame malloc instance failed.\n");
		return NULL;
	}
	memset(instance, 0, sizeof(RPMsgFrameInstance));

	instance->rpmsg_frame_callback = config->rpmsg_frame_callback;
	instance->id = config->id;
	instance->tx_frame.motor_count = config->telemetry_motor_count;
	if (instance->tx_frame.motor_count > RPMSG_FRAME_MAX_MOTOR_CNT) {
		instance->tx_frame.motor_count = RPMSG_FRAME_MAX_MOTOR_CNT;
	}

	memset(&rpmsgConfig, 0, sizeof(rpmsgConfig));
	rpmsgConfig.local_ept = config->local_ept;
	rpmsgConfig.remote_ept = config->remote_ept;
	rpmsgConfig.ept_name = config->ept_name;
	rpmsgConfig.rpmsg_module_callback = DecodeRPMsgFrame;
	rpmsgConfig.id = instance;

	instance->rpmsg_ins = RPMsg_Register(&rpmsgConfig);
	if (instance->rpmsg_ins == NULL) {
		free(instance);
		return NULL;
	}

	rpmsg_frame_instance[idx++] = instance;

	return instance;
}

/**
 * @brief 设置遥测帧中的电机数量。
 * @param instance RPMsg 业务实例。
 * @param motor_count 电机数量，范围 0~RPMSG_FRAME_MAX_MOTOR_CNT。
 */
void RPMsgFrameSetTelemetryMotorCount(RPMsgFrameInstance *instance, uint8_t motor_count)
{
	if (instance == NULL) {
		return;
	}

	if (motor_count > RPMSG_FRAME_MAX_MOTOR_CNT) {
		motor_count = RPMSG_FRAME_MAX_MOTOR_CNT;
	}

	instance->tx_frame.motor_count = motor_count;
}

/**
 * @brief 设置遥测帧中某个电机状态。
 * @param instance RPMsg 业务实例。
 * @param motor_idx 电机索引。
 * @param state 电机状态。
 * @return 设置成功返回 1，失败返回 0。
 */
uint8_t RPMsgFrameSetTelemetryMotorState(RPMsgFrameInstance *instance,
										  uint8_t motor_idx,
										  const MotorState_t *state)
{
	if ((instance == NULL) || (state == NULL)) {
		return 0U;
	}

	if (motor_idx >= RPMSG_FRAME_MAX_MOTOR_CNT) {
		return 0U;
	}

	instance->tx_frame.motors[motor_idx] = *state;
	if (instance->tx_frame.motor_count <= motor_idx) {
		instance->tx_frame.motor_count = (uint8_t)(motor_idx + 1U);
	}

	return 1U;
}

/**
 * @brief 发送遥测帧。
 * @param instance RPMsg 业务实例。
 * @param timestamp_ms 帧时间戳。
 * @param timeout_ms 发送等待超时时间。
 * @return 发送成功返回 1，失败返回 0。
 */
uint8_t RPMsgFrameTransmitTelemetry(RPMsgFrameInstance *instance,
									 uint32_t timestamp_ms,
									 uint32_t timeout_ms)
{
	if ((instance == NULL) || (instance->rpmsg_ins == NULL)) {
		return 0U;
	}

	PackRPMsgTelemetryFrame(instance, timestamp_ms);

	memcpy(instance->rpmsg_ins->tx_buff, &instance->tx_frame, sizeof(FrameTelemetry_t));
	RPMsg_SetTxLen(instance->rpmsg_ins, sizeof(FrameTelemetry_t));

	if (RPMsg_Transmit(instance->rpmsg_ins, timeout_ms) == 0U) {
		instance->tx_err_cnt++;
		return 0U;
	}

	instance->tx_ok_cnt++;
	return 1U;
}

/**
 * @brief 查询是否收到新命令帧。
 * @param instance RPMsg 业务实例。
 * @return 有新命令返回 1，否则返回 0。
 */
uint8_t RPMsgFrameHasNewCommand(RPMsgFrameInstance *instance)
{
	if (instance == NULL) {
		return 0U;
	}

	return instance->new_command_ready;
}

/**
 * @brief 获取最近一次通过 CRC 校验的命令帧。
 * @param instance RPMsg 业务实例。
 * @param command 输出命令帧缓冲区。
 * @param clear_new_flag 是否在读取后清除 new_command_ready 标记。
 * @return 获取成功返回 1，失败返回 0。
 */
uint8_t RPMsgFrameGetCommand(RPMsgFrameInstance *instance,
							  FrameCommand_t *command,
							  uint8_t clear_new_flag)
{
	if ((instance == NULL) || (command == NULL)) {
		return 0U;
	}

	if (instance->new_command_ready == 0U) {
		return 0U;
	}

	*command = instance->rx_frame;

	if (clear_new_flag != 0U) {
		instance->new_command_ready = 0U;
	}

	return 1U;
}
