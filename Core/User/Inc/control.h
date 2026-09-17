#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"

void Control_Init(void);

void Control_Forward(uint16_t speed);
void Control_Backward(uint16_t speed);

void Control_TurnLeft(uint16_t speed);
void Control_TurnRight(uint16_t speed);

void Control_Stop(void);

void Control_AvoidanceReset(void);
void Control_AvoidanceTask(void);

#endif // CONTROL_H
