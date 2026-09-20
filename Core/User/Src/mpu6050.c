#include "mpu6050.h"
#include "i2c.h"
#include <stdio.h>

// MPU6050的7位地址是0x68，HAL库需要左移1位
#define MPU6050_ADDR (0x68 << 1)
// WHO_AM_I寄存器，用来确认I2C是否能读到芯片
#define MPU6050_REG_WHO_AM_I 0x75
// 电源管理寄存器，写0可以唤醒MPU6050
#define MPU6050_REG_PWR_MGMT_1 0x6B
// Z轴陀螺仪高8位寄存器地址
#define MPU6050_REG_GYRO_ZOUT_H 0x47
// 陀螺仪默认量程下，131个原始值约等于1度每秒
#define MPU6050_GYRO_SCALE 131.0f
// 校准时采样次数，次数越多越稳定，但等待时间也越长
#define MPU6050_CALIBRATION_SAMPLES 200

// Z轴零漂，静止时陀螺仪也可能有一点读数，需要减掉
static float gyro_z_bias;
// 当前累计角度，单位是度
static float yaw;
// 上一次更新角度的时间
static uint32_t last_update_tick;

static int MPU6050_ReadGyroZ(int16_t *gyro_z_raw) {
  uint8_t buffer[2];

  // 连续读取Z轴陀螺仪高8位和低8位
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_GYRO_ZOUT_H,
                       I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) != HAL_OK) {
    return 0;
  }

  // 两个8位数据合成一个16位有符号数
  *gyro_z_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
  return 1;
}

void MPU6050_Init(void) {
  uint8_t data = 0x00;

  // 检查I2C总线上能不能找到MPU6050
  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3, 100) != HAL_OK) {
    printf("MPU6050 not found\r\n");
    return;
  }

  // 唤醒MPU6050，默认上电后它可能处于睡眠状态
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

  // 读取WHO_AM_I，正常一般能读到0x68
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

  // 读取Z轴角速度原始值
  if (!MPU6050_ReadGyroZ(&gyro_z_raw)) {
    printf("MPU6050 gyro read failed\r\n");
    return;
  }

  // 原始值转换为度每秒
  gyro_z_dps = gyro_z_raw / MPU6050_GYRO_SCALE;

  printf("Gyro Z raw=%d, speed=%.2f deg/s\r\n", gyro_z_raw, gyro_z_dps);
}

void MPU6050_Calibrate(void) {
  int16_t gyro_z_raw;
  int32_t sum = 0;
  uint16_t success_count = 0;

  printf("MPU6050 calibration, keep still...\r\n");

  // 校准时小车必须静止，读取多次静止数据求平均值
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

  // 平均值就是零漂，后面每次读数都减掉它
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
    // 读取失败时本次不更新角度
    return;
  }

  // 计算距离上次更新过了多少秒
  delta_time = (now - last_update_tick) / 1000.0f;
  last_update_tick = now;
  // 减掉零漂，再换算成角速度
  gyro_z_dps = ((float)gyro_z_raw - gyro_z_bias) / MPU6050_GYRO_SCALE;
  // 角度 = 角速度 * 时间，把每次的小角度累加起来就是yaw
  yaw += gyro_z_dps * delta_time;
}

void MPU6050_ResetYaw(void) {
  // 开始一次新的转弯前，把当前角度清零
  yaw = 0.0f;
  last_update_tick = HAL_GetTick();
}

float MPU6050_GetYaw(void) {
  // 返回当前累计角度
  return yaw;
}
