#ifndef SERVO_H
#define SERVO_H

#include "main.h"

// 初始化舵机PWM
void Servo_Init(void);
// 舵机自动停止任务，需要在while循环中反复调用
void Servo_Task(void);
// 停止指定舵机
void Servo_Stop(uint8_t servo_id);
// 指定舵机正方向转动一段时间
void Servo_TurnForward(uint8_t servo_id, uint8_t time_ms);
// 指定舵机反方向转动一段时间
void Servo_TurnBackward(uint8_t servo_id, uint8_t time_ms);

#endif /* SERVO_H */
