#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

// 初始化控制层需要用到的电机和传感器
void Control_Init(void);

// 小车前进
void Control_Forward(uint16_t speed);
// 小车后退
void Control_Backward(uint16_t speed);

// 小车原地左转
void Control_TurnLeft(uint16_t speed);
// 小车原地右转
void Control_TurnRight(uint16_t speed);

// 小车停止
void Control_Stop(void);

// 重置自动避障状态机
void Control_AvoidanceReset(void);
// 自动避障任务，需要在while循环中反复调用
void Control_AvoidanceTask(void);

#endif // CONTROL_H
