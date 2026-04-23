/*
 * @Description: 
 * @Author: changfengpro
 * @brief: 
 * @version: 
 * @Date: 2026-04-20 12:10:53
 * @LastEditors:  
 * @LastEditTime: 2026-04-23 16:23:40
 */
/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */
#include <string.h>
#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "rpmsg_frame.h"

#define RPMSG_FRAME_TEST

#ifdef RPMSG_FRAME_TEST
System_Telemetry_s mcu_telemetry;

/**
 * @brief 初始化测试用虚拟遥测数据 (Mock Data)
 * @param telemetry 指向遥测结构体的指针
 */
void Init_Dummy_Telemetry_Data(System_Telemetry_s *telemetry)
{
    // 1. 模拟系统运行时间戳 (假设系统已经运行了 123456 毫秒)
    telemetry->timestamp = 123456; 

    // 2. 遍历 20 个电机的数组进行赋值
    for (int i = 0; i < MAX_MOTOR_COUNT; i++)
    {
        if (i < 4) 
        {
            // 模拟 1~4 号：M3508 底盘电机 (高频运行态)
            telemetry->motors[i].can_id       = 0x205;
            telemetry->motors[i].type         = M3508;
            telemetry->motors[i].control_mode = MOTOR_CONTROL_MODE_VELOCITY;
            telemetry->motors[i].total_round  = 1000 + i * 15;        // 已经转了 1000 多圈
            telemetry->motors[i].ecd          = (4096 + i * 500) % 8192; // 编码器读数 (0~8191)
            telemetry->motors[i].speed_raw    = 3500 + i * 50;        // 3500 RPM 左右的转速
            telemetry->motors[i].real_current = 1500 + i * 100;       // 1500 的实际反馈电流
            telemetry->motors[i].temperature  = 45 + i;               // 温度 45~48 度
        } 
        else if (i < 8) 
        {
            // 模拟 5~8 号：GM6020 云台/关节电机 (低速维稳态)
            telemetry->motors[i].can_id       = 0x209;
            telemetry->motors[i].type         = GM6020;
            telemetry->motors[i].control_mode = MOTOR_CONTROL_MODE_POSITION;
            telemetry->motors[i].total_round  = 5 + i;                // 转的圈数很少
            telemetry->motors[i].ecd          = (1024 + i * 800) % 8192;
            telemetry->motors[i].speed_raw    = 15 - i * 2;           // 几乎处于静止维持状态
            telemetry->motors[i].real_current = 800 - i * 50;         // 维持力矩产生的电流
            telemetry->motors[i].temperature  = 38 + i;               // 温度较低
        }
        else 
        {
            // 模拟 9~20 号：空槽位，未接入电机
            telemetry->motors[i].type         = MOTOR_TYPE_NONE;
            telemetry->motors[i].control_mode = MOTOR_CONTROL_MODE_NONE;
            telemetry->motors[i].total_round  = 0;
            telemetry->motors[i].ecd          = 0;
            telemetry->motors[i].speed_raw    = 0;
            telemetry->motors[i].real_current = 0;
            telemetry->motors[i].temperature  = 0;
        }
        
        // 3. 内存对齐填充字段清零 (避免传输脏数据)
        telemetry->motors[i].reserved = 0;

    }
}

#endif


int main(void)
{
    uint8_t rpmsg_ready = 0U;

    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();

    
    __enable_irq();

    RPMsg_Frame_Init(); 

#ifdef RPMSG_FRAME_TEST
    Init_Dummy_Telemetry_Data(&mcu_telemetry);
#endif

    while (1) {

        
#ifdef RPMSG_FRAME_TEST
            mcu_telemetry.timestamp += 20;
            RPMsg_Frame_Send_Telemetry(&mcu_telemetry);
#endif
        HAL_DelayMs(2);

    }
}

int entry(void)
{
    return main();
}
