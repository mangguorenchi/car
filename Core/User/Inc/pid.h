#ifndef PID_H
#define PID_H

#include "main.h"

typedef enum {
  encoder1 = 1,
  encoder2,
  encoder3,
  encoder4,
  
}Encoder_ID_t;

void Encoder_Init(void);

int Encoder_GetCount(Encoder_ID_t encoder);

void Encoder_ClearCount(Encoder_ID_t encoder);

void Encoder_ClearAllCount(void);

void Encoder_Task(void);

int16_t Encoder_GetSpeed(Encoder_ID_t encoder);

#endif // PID_H