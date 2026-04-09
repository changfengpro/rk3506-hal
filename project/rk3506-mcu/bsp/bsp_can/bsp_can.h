#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "hal_base.h"
#include "hal_bsp.h" // 必须包含，以获取 HAL_CANFD_DEV 定义

// 外部声明，供 main.c 使用
extern volatile uint32_t g_isr_hit_count;
extern volatile uint32_t g_can_adapter_hit_count;
extern volatile uint32_t g_can_common_hit_count;
extern volatile uint32_t g_can_last_isr;
extern volatile uint32_t g_can_rx_dispatch_count;

#define CAN_MX_REGISTER_CNT 2

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
};

typedef struct {
    struct CAN_REG *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    CAN_Callback can_module_callback;
} CAN_Init_Config_s;

/* 函数声明 */
void CAN_Service_Init(void);
CANInstance* CAN_Register(CAN_Init_Config_s *config);
uint8_t CAN_Transmit(CANInstance *_instance, uint32_t timeout_ms);

#endif /* __BSP_CAN_H */
