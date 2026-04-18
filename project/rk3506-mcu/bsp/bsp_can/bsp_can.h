/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 */

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "hal_base.h"
#include "hal_bsp.h"

#define CAN_MX_REGISTER_CNT 2U
#define DEVICE_CAN_CNT      2U

#define CAN_INTERRUPT_RX    0x01U
#define CAN_INTERRUPT_TX    0x02U
#define CAN_INTERRUPT_ALL   (CAN_INTERRUPT_RX | CAN_INTERRUPT_TX)

typedef struct CANInstance CANInstance;
typedef void (*CAN_Callback)(CANInstance *ins);

struct CANInstance {
    struct CAN_REG *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t tx_buff[64];
    uint8_t rx_buff[64];
    uint8_t tx_len;
    uint8_t rx_len;
    CAN_Callback can_module_callback;
    void *id;
};

typedef struct {
    struct CAN_REG *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    CAN_Callback can_module_callback;
    void *id;
} CAN_Init_Config_s;

/**
 * @brief 初始化 CAN 控制器、引脚和中断路由。
 */
void BSP_CAN_Init(void);

/**
 * @brief 配置 CAN TX/RX 中断开关。
 * @param interrupt_mask 中断使能位组合，支持 CAN_INTERRUPT_RX/CAN_INTERRUPT_TX。
 */
void CANSetInterruptEnable(uint32_t interrupt_mask);

/**
 * @brief 注册一个 CAN 业务实例。
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改 CAN 发送帧长度。
 * @param instance 目标 CAN 实例。
 * @param length 发送长度，经典 CAN 模式下支持 1~8。
 */
void CANSetDLC(CANInstance *instance, uint8_t length);

/**
 * @brief 发送一帧标准 CAN 数据帧。
 * @param instance 已注册的 CAN 实例。
 * @param timeout_ms 发送邮箱等待超时时间，单位毫秒。
 * @return 1 表示成功写入发送邮箱，0 表示邮箱超时不可用或参数非法。
 */
uint8_t CANTransmit(CANInstance *instance, uint32_t timeout_ms);



#endif /* __BSP_CAN_H */
