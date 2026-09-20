#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

// 初始化超声波测距需要用到的定时器和Trig引脚
void Sensor_Init(void);

// 读取前方超声波距离，单位mm，返回-1表示测距失败
float HCSR04_FrontRead(void);

// 读取右侧超声波距离，单位mm，返回-1表示测距失败
float HCSR04_RightRead(void);

// 读取左侧超声波距离，单位mm，返回-1表示测距失败
float HCSR04_LeftRead(void);

#endif // SENSOR_H
