/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#ifndef __FRAME_RPMSG_H
#define __FRAME_RPMSG_H

#include "bsp_rpmsg.h"

#define RPMSG_FRAME_MAX_MOTOR_CNT 4U

typedef struct __attribute__((packed)) {
	uint8_t motor_id;
	int32_t position_q16;
	int32_t velocity_q16;
	int16_t torque_q8;
	int16_t temperature_c;
	uint16_t status_flags;
} MotorState_t;

typedef struct __attribute__((packed)) {
	uint8_t motor_id;
	int32_t target_position_q16;
	int32_t target_velocity_q16;
	int16_t target_torque_q8;
	int16_t kp_q8;
	int16_t kd_q8;
	uint16_t ctrl_flags;
} MotorCmd_t;

typedef struct __attribute__((packed)) {
	uint16_t seq;
	uint32_t timestamp_ms;
	uint8_t motor_count;
	MotorState_t motors[RPMSG_FRAME_MAX_MOTOR_CNT];
	uint16_t crc16;
} FrameTelemetry_t;

typedef struct __attribute__((packed)) {
	uint16_t seq;
	uint32_t timestamp_ms;
	uint8_t motor_count;
	MotorCmd_t motors[RPMSG_FRAME_MAX_MOTOR_CNT];
	uint16_t crc16;
} FrameCommand_t;

typedef struct RPMsgFrameInstance RPMsgFrameInstance;
typedef void (*RPMsgFrame_Callback)(RPMsgFrameInstance *ins);

struct RPMsgFrameInstance {
	RPMsgInstance *rpmsg_ins;
	uint16_t tx_seq;
	uint16_t last_rx_seq;
	uint8_t has_new_command;
	FrameTelemetry_t state_frame;
	FrameCommand_t command_frame;
	uint32_t tx_ok_cnt;
	uint32_t tx_err_cnt;
	uint32_t rx_ok_cnt;
	uint32_t rx_drop_cnt;
	RPMsgFrame_Callback command_callback;
	void *id;
};

typedef struct {
	uint32_t local_ept;
	uint32_t remote_ept;
	const char *ept_name;
	uint8_t state_motor_count;
	RPMsgFrame_Callback command_callback;
	void *id;
} RPMsgFrame_Init_Config_s;

/**
 * @brief 初始化并注册一个 RPMsg 业务实例。
 *
 * 该接口仅负责注册 RPMsg 业务实例，
 * 底层 RPMsg 服务初始化由 App 层统一管理。
 *
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
RPMsgFrameInstance *RPMsgFrameInit(RPMsgFrame_Init_Config_s *config);

/**
 * @brief 清空当前状态帧内容，但保留电机数量配置。
 * @param instance RPMsg 业务实例。
 */
void RPMsgFrameResetStateFrame(RPMsgFrameInstance *instance);

/**
 * @brief 获取可由 App 层填充的状态帧缓冲区。
 * @param instance RPMsg 业务实例。
 * @return 返回当前状态帧指针，失败返回 NULL。
 */
FrameTelemetry_t *RPMsgFrameGetStateFrame(RPMsgFrameInstance *instance);

/**
 * @brief 发送状态帧。
 * @param instance RPMsg 业务实例。
 * @param timestamp_ms 帧时间戳。
 * @param timeout_ms 发送等待超时时间。
 * @return 发送成功返回 1，失败返回 0。
 */
uint8_t RPMsgFrameTransmitStateFrame(RPMsgFrameInstance *instance,
									 uint32_t timestamp_ms,
									 uint32_t timeout_ms);

/**
 * @brief 查询是否收到新命令帧。
 * @param instance RPMsg 业务实例。
 * @return 有新命令返回 1，否则返回 0。
 */
uint8_t RPMsgFrameHasNewCommand(RPMsgFrameInstance *instance);

/**
 * @brief 获取最近一次通过 CRC 校验的命令帧。
 * @param instance RPMsg 业务实例。
 * @param command 输出命令帧缓冲区。
 * @param clear_new_flag 是否在读取后清除 has_new_command 标记。
 * @return 获取成功返回 1，失败返回 0。
 */
uint8_t RPMsgFrameGetCommandFrame(RPMsgFrameInstance *instance,
								  FrameCommand_t *command,
								  uint8_t clear_new_flag);

#endif /* __FRAME_RPMSG_H */
