#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

void Sensor_Init(void);

float HCSR04_FrontRead(void);

float HCSR04_RightRead(void);

float HCSR04_LeftRead(void);

#endif // SENSOR_H