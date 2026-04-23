/*
 * @Description: Motor Control IPC Protocol (Linux A7 <-> Cortex-M0)
 * @Author: changfengpro
 * @brief: Optimized for Cortex-M0 32-bit natural alignment and performance
 */
#ifndef IPC_MOTOR_PROTOCOL_H
#define IPC_MOTOR_PROTOCOL_H

#include <stdint.h>

#define MAX_MOTOR_COUNT (20U) 

/* 电机类型枚举 */
typedef enum {
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
} Motor_Type_e;

/* 控制模式枚举 */
typedef enum {
    MOTOR_CONTROL_MODE_NONE = 0,
    MOTOR_CONTROL_MODE_TORQUE,
    MOTOR_CONTROL_MODE_VELOCITY,
    MOTOR_CONTROL_MODE_POSITION,
} Motor_Control_Mode_e;

/* ==========================================================
 * 上行：M0 -> Linux (遥测数据)
 * ========================================================== */
typedef struct {
    // 4 字节变量放在最前面
    int32_t  total_round;  // 4 bytes: 累计圈数
    
    // 2 字节变量
    uint16_t motor_id;       // 2 bytes: 当前motor can id
    uint16_t ecd;          // 2 bytes: 当前编码器原始值
    int16_t  speed_raw;    // 2 bytes: 原始速度反馈
    int16_t  real_current; // 2 bytes: 实际电流
    
    // 1 字节变量
    uint8_t  type;         // 1 byte:  电机类型 (Motor_Type_e)
    uint8_t  control_mode; // 1 byte:  控制模式 (Motor_Control_Mode_e)
    uint8_t  temperature;  // 1 byte:  温度
    
    // 手动填充 3 字节，确保整个结构体大小为 16 字节的整数倍
    uint8_t  reserved;  // 1 bytes: 保留对齐
} Motor_IPC_Payload_s;     // 总大小严格为 16 bytes

typedef struct {
    uint32_t timestamp;                              // 4 bytes: M0 系统时间戳
    Motor_IPC_Payload_s motors[MAX_MOTOR_COUNT];     // 16 * 20 = 320 bytes
} System_Telemetry_s;                                // 整体总计: 324 bytes

/* ==========================================================
 * 下行：Linux -> M0 (控制指令)
 * ========================================================== */
typedef struct {
    // 4 字节变量
    int32_t position_q16;     // 4 bytes: 位置目标 Q16 (真实值 = raw / 65536.0)
    int32_t velocity_q16;     // 4 bytes: 速度目标 Q16 (真实值 = raw / 65536.0)
    int32_t torque_q16;       // 4 bytes: 力矩目标 Q16 (修复：改为 int32_t)
    
    // 1 字节变量
    uint8_t motor_id;         // 1 byte:  电机CAN ID 
    uint8_t motor_type;       // 1 byte:  电机类型
    uint8_t control_mode;     // 1 byte:  控制模式
    
    // 手动填充 1 字节，确保整个结构体大小为 16 字节的整数倍
    uint8_t reserved;         // 1 byte:  保留对齐
} Motor_IPC_Cmd_s;            // 总大小严格为 16 bytes

typedef struct {
    uint32_t timestamp;                              // 4 bytes: Linux 系统时间戳 (用于 M0 识别通讯超时和看门狗)
    Motor_IPC_Cmd_s cmds[MAX_MOTOR_COUNT];           // 16 * 20 = 320 bytes
} System_ControlCmd_s;                               // 整体总计: 324 bytes

#endif