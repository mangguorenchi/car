#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef enum {
  // M1：电机A
  MOTOR_M1 = 1,
  // M2：电机D
  MOTOR_M2 = 2,
  // M3：电机B
  MOTOR_M3 = 3,
  // M4：电机C
  MOTOR_M4 = 4,
} Motor_ID;

// 初始化电机PWM，并停止所有电机
void Motor_Init(void);
// 设置单个电机速度，正数正转，负数反转，0停止
void Motor_SetSpeed(Motor_ID motor, int16_t speed);
// 停止全部电机
void Motor_StopAll(void);

#endif /* MOTOR_H */
