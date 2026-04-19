/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 */

#ifndef __ROBOT_H
#define __ROBOT_H

/**
 * @brief 机器人初始化统一入口。
 *
 * 该函数是业务侧唯一需要在启动阶段调用的入口。
 */
void RobotInit(void);

/**
 * @brief 机器人周期任务。
 *
 * 无RTOS场景下可在SysTick中调用；有RTOS场景下可由任务调度调用。
 */
void RobotTask(void);



#endif /* __ROBOT_H */
