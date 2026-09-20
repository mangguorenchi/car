#ifndef SERVO_H
#define SERVO_H

#include "main.h"

void Servo_Init(void);
void Servo_Task(void);
void Servo_Stop(uint8_t servo_id);
void Servo_SetAngle(uint8_t servo_id, uint16_t angle);
void Servo_AddAngle(uint8_t servo_id);
void Servo_SubAngle(uint8_t servo_id);
void Servo_TurnForward(uint8_t servo_id, uint8_t time_ms);
void Servo_TurnBackward(uint8_t servo_id, uint8_t time_ms);

#endif /* SERVO_H */
