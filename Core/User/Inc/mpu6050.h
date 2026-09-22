#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

// 初始化MPU6050并唤醒芯片
void MPU6050_Init(void);
// 读取WHO_AM_I，用来测试I2C连接是否正常
void MPU6050_Test(void);
// 打印Z轴陀螺仪原始值和角速度
void MPU6050_TestGyro(void);
// 校准陀螺仪零漂，调用时小车必须静止
void MPU6050_Calibrate(void);
// 更新当前Yaw角度，需要在while循环中反复调用
void MPU6050_Update(void);
// 清零Yaw角度，一般在开始转弯前调用
void MPU6050_ResetYaw(void);
// 获取当前Yaw角度
float MPU6050_GetYaw(void);

#endif /* MPU6050_H */
