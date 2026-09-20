#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

void MPU6050_Init(void);
void MPU6050_Test(void);
void MPU6050_TestGyro(void);
void MPU6050_Calibrate(void);
void MPU6050_Update(void);
void MPU6050_ResetYaw(void);
float MPU6050_GetYaw(void);

#endif /* MPU6050_H */
