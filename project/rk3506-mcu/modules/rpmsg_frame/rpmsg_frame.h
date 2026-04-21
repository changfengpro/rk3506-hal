/*
 * @Description: 
 * @Author: changfengpro
 * @brief: 
 * @version: 
 * @Date: 2026-04-20 12:25:55
 * @LastEditors:  
 * @LastEditTime: 2026-04-21 22:30:52
 */


#ifndef RPMSG_FRAME_H
#define RPMSG_FRAME_H

#include "ipc_motor_protocol.h"


/**
 * @brief: 
 * @return {*}
 */
void RPMsg_Frame_Init(void);

/**
 * @brief: 
 * @param {System_Telemetry_s} *telemetry_data
 * @return {*}
 */
void RPMsg_Frame_Send_Telemetry(System_Telemetry_s *telemetry_data);


#endif /* RPMSG_FRAME_H */