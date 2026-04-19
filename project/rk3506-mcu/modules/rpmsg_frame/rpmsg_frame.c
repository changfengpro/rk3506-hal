/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 *
 * RPMsg frame encode/decode, CRC check and transport integration.
 */

#include "rpmsg_frame.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief 计算 CRC16-CCITT(FALSE) 校验值。
 * @param data 输入数据。
 * @param len 数据长度。
 * @return CRC16 校验值。
 */
static uint16_t RPMsgFrameCalcCrc16(const uint8_t *data, uint32_t len)
{
	/* Nibble table for CRC16-CCITT(FALSE), poly=0x1021. */
	static const uint16_t kCrcNibbleTable[16] = {
		0x0000U, 0x1021U, 0x2042U, 0x3063U,
		0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
		0x8108U, 0x9129U, 0xA14AU, 0xB16BU,
		0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
	};
	uint16_t crc = 0xFFFFU;
	uint32_t i;

	if (data == NULL) {
		return 0U;
	}

	for (i = 0U; i < len; i++) {
		uint8_t byte = data[i];
		uint8_t idx;

		idx = (uint8_t)(((uint8_t)(crc >> 12U)) ^ (uint8_t)(byte >> 4U));
		crc = (uint16_t)((uint16_t)(crc << 4U) ^ kCrcNibbleTable[idx & 0x0FU]);

		idx = (uint8_t)(((uint8_t)(crc >> 12U)) ^ (byte & 0x0FU));
		crc = (uint16_t)((uint16_t)(crc << 4U) ^ kCrcNibbleTable[idx & 0x0FU]);
	}

	return crc;
}

/**
 * @brief 组装并计算遥测帧 CRC。
 * @param instance RPMsg 业务实例。
 * @param timestamp_ms 时间戳。
 */
static void PackRPMsgStateFrame(RPMsgFrameInstance *instance, uint32_t timestamp_ms)
{
	FrameTelemetry_t *frame;

	if (instance == NULL) {
		return;
	}

	frame = &instance->state_frame;

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

	instance->command_frame = frame;
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
	instance->has_new_command = 1U;

	if (instance->command_callback != NULL) {
		instance->command_callback(instance);
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

	instance = (RPMsgFrameInstance *)malloc(sizeof(RPMsgFrameInstance));
	if (instance == NULL) {
		HAL_DBG_ERR("RPMsg Frame malloc instance failed.\n");
		return NULL;
	}
	memset(instance, 0, sizeof(RPMsgFrameInstance));

	instance->command_callback = config->command_callback;
	instance->id = config->id;
	instance->state_frame.motor_count = config->state_motor_count;
	if (instance->state_frame.motor_count > RPMSG_FRAME_MAX_MOTOR_CNT) {
		instance->state_frame.motor_count = RPMSG_FRAME_MAX_MOTOR_CNT;
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

	return instance;
}

/**
 * @brief 清空当前状态帧内容，但保留电机数量配置。
 * @param instance RPMsg 业务实例。
 */
void RPMsgFrameResetStateFrame(RPMsgFrameInstance *instance)
{
	uint8_t motorCount;

	if (instance == NULL) {
		return;
	}

	motorCount = instance->state_frame.motor_count;
	memset(&instance->state_frame, 0, sizeof(instance->state_frame));
	instance->state_frame.motor_count = motorCount;
}

/**
 * @brief 获取可由 App 层填充的状态帧缓冲区。
 * @param instance RPMsg 业务实例。
 * @return 返回当前状态帧指针，失败返回 NULL。
 */
FrameTelemetry_t *RPMsgFrameGetStateFrame(RPMsgFrameInstance *instance)
{
	if (instance == NULL) {
		return NULL;
	}

	return &instance->state_frame;
}

/**
 * @brief 发送状态帧。
 * @param instance RPMsg 业务实例。
 * @param timestamp_ms 帧时间戳。
 * @param timeout_ms 发送等待超时时间。
 * @return 发送成功返回 1，失败返回 0。
 */
uint8_t RPMsgFrameTransmitStateFrame(RPMsgFrameInstance *instance,
									 uint32_t timestamp_ms,
									 uint32_t timeout_ms)
{
	if ((instance == NULL) || (instance->rpmsg_ins == NULL)) {
		return 0U;
	}

	PackRPMsgStateFrame(instance, timestamp_ms);

	memcpy(instance->rpmsg_ins->tx_buff, &instance->state_frame, sizeof(FrameTelemetry_t));
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

	return instance->has_new_command;
}

/**
 * @brief 获取最近一次通过 CRC 校验的命令帧。
 * @param instance RPMsg 业务实例。
 * @param command 输出命令帧缓冲区。
 * @param clear_new_flag 是否在读取后清除 has_new_command 标记。
 * @return 获取成功返回 1，失败返回 0。
 */
uint8_t RPMsgFrameGetCommandFrame(RPMsgFrameInstance *instance,
								  FrameCommand_t *command,
								  uint8_t clear_new_flag)
{
	if ((instance == NULL) || (command == NULL)) {
		return 0U;
	}

	if (instance->has_new_command == 0U) {
		return 0U;
	}

	*command = instance->command_frame;

	if (clear_new_flag != 0U) {
		instance->has_new_command = 0U;
	}

	return 1U;
}
