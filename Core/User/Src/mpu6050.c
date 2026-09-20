#include "mpu6050.h"
#include "i2c.h"
#include <stdio.h>

#define MPU6050_ADDR (0x68 << 1)
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_SCALE 131.0f
#define MPU6050_CALIBRATION_SAMPLES 200

static float gyro_z_bias;
static float yaw;
static uint32_t last_update_tick;

static int MPU6050_ReadGyroZ(int16_t *gyro_z_raw) {
  uint8_t buffer[2];

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_ZOUT_H,
                       I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) != HAL_OK) {
    return 0;
  }

  *gyro_z_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
  return 1;
}

void MPU6050_Init(void) {
  uint8_t data = 0x00;

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3, 100) != HAL_OK) {
    printf("MPU6050 not found\r\n");
    return;
  }

  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1,
                        I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
    printf("MPU6050 wake failed\r\n");
    return;
  }

  gyro_z_bias = 0.0f;
  yaw = 0.0f;
  last_update_tick = HAL_GetTick();

  printf("MPU6050 init ok\r\n");
}

void MPU6050_Test(void) {
  uint8_t who_am_i = 0;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) != HAL_OK) {
    printf("MPU6050 read failed\r\n");
    return;
  }

  printf("MPU6050 WHO_AM_I = 0x%02X\r\n", who_am_i);
}

void MPU6050_TestGyro(void) {
  int16_t gyro_z_raw;
  float gyro_z_dps;

  if (!MPU6050_ReadGyroZ(&gyro_z_raw)) {
    printf("MPU6050 gyro read failed\r\n");
    return;
  }

  gyro_z_dps = gyro_z_raw / MPU6050_GYRO_SCALE;

  printf("Gyro Z raw=%d, speed=%.2f deg/s\r\n", gyro_z_raw, gyro_z_dps);
}

void MPU6050_Calibrate(void) {
  int16_t gyro_z_raw;
  int32_t sum = 0;
  uint16_t success_count = 0;

  printf("MPU6050 calibration, keep still...\r\n");

  for (uint16_t i = 0; i < MPU6050_CALIBRATION_SAMPLES; i++) {
    if (MPU6050_ReadGyroZ(&gyro_z_raw)) {
      sum += gyro_z_raw;
      success_count++;
    }
    HAL_Delay(5);
  }

  if (success_count == 0) {
    printf("MPU6050 calibration failed\r\n");
    return;
  }

  gyro_z_bias = (float)sum / success_count;
  yaw = 0.0f;
  last_update_tick = HAL_GetTick();

  printf("MPU6050 bias=%.2f\r\n", gyro_z_bias);
  printf("MPU6050 calibration done\r\n");
}

void MPU6050_Update(void) {
  int16_t gyro_z_raw;
  uint32_t now = HAL_GetTick();
  float delta_time;
  float gyro_z_dps;

  if (!MPU6050_ReadGyroZ(&gyro_z_raw)) {
    return;
  }

  delta_time = (now - last_update_tick) / 1000.0f;
  last_update_tick = now;
  gyro_z_dps = ((float)gyro_z_raw - gyro_z_bias) / MPU6050_GYRO_SCALE;
  yaw += gyro_z_dps * delta_time;
}

void MPU6050_ResetYaw(void) {
  yaw = 0.0f;
  last_update_tick = HAL_GetTick();
}

float MPU6050_GetYaw(void) {
  return yaw;
}
