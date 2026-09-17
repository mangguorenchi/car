#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef enum {
  MOTOR_M1 = 1,
  MOTOR_M2 = 2,
  MOTOR_M3 = 3,
  MOTOR_M4 = 4,
} Motor_ID;

void Motor_Init(void);
void Motor_SetSpeed(Motor_ID motor, int16_t speed);
void Motor_StopAll(void);

#endif /* MOTOR_H */
