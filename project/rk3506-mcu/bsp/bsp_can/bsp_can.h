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
void CAN_Service_Init(void);

/**
 * @brief 配置 CAN TX/RX 中断开关。
 * @param interrupt_mask 中断使能位组合，支持 CAN_INTERRUPT_RX/CAN_INTERRUPT_TX。
 *
 * 默认值为 CAN_INTERRUPT_ALL，即同时打开 TX 和 RX 中断。
 * 该接口可在 CAN_Service_Init() 前后调用；若在初始化后调用，会立即更新硬件屏蔽寄存器。
 */
void CAN_SetInterruptEnable(uint32_t interrupt_mask);

/**
 * @brief 注册一个 CAN 业务实例。
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
CANInstance *CAN_Register(CAN_Init_Config_s *config);

/**
 * @brief 修改 CAN 发送帧长度。
 * @param instance 目标 CAN 实例。
 * @param length 发送长度，经典 CAN 模式下支持 1~8。
 */
void CAN_SetDLC(CANInstance *instance, uint8_t length);

/**
 * @brief 发送一帧标准 CAN 数据帧。
 * @param instance 已注册的 CAN 实例。
 * @param timeout_ms 发送等待超时时间，单位毫秒。
 * @return 1 表示发送成功，0 表示发送失败或超时。
 */
uint8_t CAN_Transmit(CANInstance *instance, uint32_t timeout_ms);


#endif /* __BSP_CAN_H */
