/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 *
 * RPMsg frame protocol definitions for robot command and telemetry exchange.
 */

#ifndef __FRAME_RPMSG_H
#define __FRAME_RPMSG_H

#include "bsp_rpmsg.h"
#include "../motor/motor_def.h"

/* 单帧里最多携带的电机对象数量，MCU/Linux 两侧必须保持一致。 */
#define RPMSG_FRAME_MAX_MOTOR_CNT 20U
/* Q8 定点缩放系数，真实值 = 原始值 / 256。 */
#define RPMSG_FRAME_Q8_SCALE 256L
/* Q16 定点缩放系数，真实值 = 原始值 / 65536。 */
#define RPMSG_FRAME_Q16_SCALE 65536L

/**
 * @brief 电机控制模式枚举。
 *
 * 当前建议至少保留以下几种模式：
 * - TORQUE：力矩控制
 * - VELOCITY：速度控制
 * - POSITION：位置控制
 *
 * 在线上传输时使用 uint8_t 保存。
 */
typedef enum {
	MOTOR_CONTROL_MODE_NONE = 0,
	MOTOR_CONTROL_MODE_TORQUE,
	MOTOR_CONTROL_MODE_VELOCITY,
	MOTOR_CONTROL_MODE_POSITION,
} Motor_Control_Mode_e;

/**
 * @brief 单个电机的状态数据。
 *
 * 该结构直接参与线上传输，字段顺序、类型和 packed 属性
 * 必须与 Linux 侧保持完全一致。
 */
typedef struct __attribute__((packed)) {
	uint8_t motor_id;        /* 电机 ID。 */
	int32_t position_q16;    /* 位置，Q16 定点。 */
	int32_t velocity_q16;    /* 速度，Q16 定点。 */
	int16_t torque_q8;       /* 力矩，Q8 定点。 */
	int16_t temperature_c;   /* 温度，单位摄氏度。 */
	uint16_t status_flags;   /* 状态标志位。 */
} MotorState_t;

/**
 * @brief 单个电机的目标命令数据。
 *
 * 该结构由 Linux 下发，MCU 解码后供应用层读取。
 * 若后续扩展控制量，需要同步修改 Linux 侧镜像定义。
 *
 * 定点说明：
 * - `target_position_q16` 的真实值 = 原始值 / 65536
 * - `target_velocity_q16` 的真实值 = 原始值 / 65536
 * - `target_torque_q8` 的真实值 = 原始值 / 256
 *
 * 例如：
 * - 位置 1.5 -> `1.5 * 65536 = 98304`
 * - 力矩 2.25 -> `2.25 * 256 = 576`
 */
typedef struct __attribute__((packed)) {
	uint8_t motor_id;                /* 电机 ID。 */
	uint8_t motor_type;              /* 协议字段，取值见 Motor_Type_e，业务侧建议转成 enum 使用。 */
	uint8_t control_mode;            /* 协议字段，取值见 Motor_Control_Mode_e，业务侧建议转成 enum 使用。 */
	int32_t target_position_q16;     /* 目标位置，Q16 定点。 */
	int32_t target_velocity_q16;     /* 目标速度，Q16 定点。 */
	int16_t target_torque_q8;        /* 目标力矩，Q8 定点。 */
} MotorCmd_t;

/**
 * @brief MCU -> Linux 的状态帧。
 *
 * crc16 覆盖范围为从 seq 起到 crc16 前一个字节为止。
 */
typedef struct __attribute__((packed)) {
	uint16_t seq;                                      /* 发送序号。 */
	uint32_t timestamp_ms;                             /* 时间戳，单位 ms。 */
	uint8_t motor_count;                               /* 当前有效电机数量。 */
	MotorState_t motors[RPMSG_FRAME_MAX_MOTOR_CNT];    /* 电机状态数组。 */
	uint16_t crc16;                                    /* CRC16-CCITT(FALSE)。 */
} FrameTelemetry_t;

/**
 * @brief Linux -> MCU 的命令帧。
 *
 * crc16 覆盖范围为从 seq 起到 crc16 前一个字节为止。
 */
typedef struct __attribute__((packed)) {
	uint16_t seq;                                   /* 发送序号。 */
	uint32_t timestamp_ms;                          /* 时间戳，单位 ms。 */
	uint8_t motor_count;                            /* 当前有效电机数量。 */
	MotorCmd_t motors[RPMSG_FRAME_MAX_MOTOR_CNT];   /* 电机命令数组。 */
	uint16_t crc16;                                 /* CRC16-CCITT(FALSE)。 */
} FrameCommand_t;

typedef struct RPMsgFrameInstance RPMsgFrameInstance;
typedef void (*RPMsgFrame_Callback)(RPMsgFrameInstance *ins);

/**
 * @brief RPMsg 帧业务实例。
 *
 * 该实例将 transport 层的 RPMsg endpoint 与业务层帧缓存绑定起来。
 * App 层通常只需要：
 * - 在回调中读取 command_frame
 * - 填写 state_frame
 * - 调用发送接口回传状态
 */
struct RPMsgFrameInstance {
	RPMsgInstance *rpmsg_ins;                /* 底层 transport 层 RPMsg 实例。 */
	uint16_t tx_seq;                         /* 状态帧发送序号。 */
	uint16_t last_rx_seq;                    /* 最近一次成功接收的命令序号。 */
	uint8_t has_new_command;                 /* 是否存在尚未取走的新命令。 */
	FrameTelemetry_t state_frame;            /* 当前待发送的状态帧缓冲。 */
	FrameCommand_t command_frame;            /* 最近一次成功校验的命令帧。 */
	uint32_t tx_ok_cnt;                      /* 状态帧发送成功计数。 */
	uint32_t tx_err_cnt;                     /* 状态帧发送失败计数。 */
	uint32_t rx_ok_cnt;                      /* 命令帧接收成功计数。 */
	uint32_t rx_drop_cnt;                    /* 命令帧丢弃计数（长度/CRC等错误）。 */
	RPMsgFrame_Callback command_callback;    /* 命令帧回调。 */
	void *id;                                /* 业务私有指针。 */
};

/**
 * @brief RPMsg 帧实例初始化参数。
 */
typedef struct {
	uint32_t local_ept;                    /* MCU 本地 endpoint 地址。 */
	uint32_t remote_ept;                   /* Linux 远端 endpoint 地址，动态时填 RPMSG_REMOTE_EPT_DYNAMIC。 */
	const char *ept_name;                  /* nameservice 服务名。 */
	uint8_t state_motor_count;             /* 初始状态帧电机数量。 */
	RPMsgFrame_Callback command_callback;  /* 收到新命令后的业务回调。 */
	void *id;                              /* 业务私有指针。 */
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
