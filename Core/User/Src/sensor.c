#include "sensor.h"
#include "gpio.h"
#include "tim.h"

static void Delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  while (__HAL_TIM_GET_COUNTER(&htim3) < us) {
  }
}

void Sensor_Init(void) {
  HAL_TIM_Base_Start(&htim3);

  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
}

float HCSR04_FrontRead(void) {
  uint32_t start_tick;
  uint16_t time_us;
  // 确保初始是低电平
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  Delay_us(2);

  // 发送10us触发脉冲
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);

  Delay_us(10);

  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  // 等待ECHO上升
  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_FRONT_ECHO_GPIO_Port, US_FRONT_ECHO_Pin) ==
         GPIO_PIN_RESET) {
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  // 开始计时
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_FRONT_ECHO_GPIO_Port, US_FRONT_ECHO_Pin) ==
         GPIO_PIN_SET) {
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  time_us = __HAL_TIM_GET_COUNTER(&htim3);
//计算距离（mm）
  return (float)time_us * 0.343f / 2.0f;
}

float HCSR04_RightRead(void) {
  uint32_t start_tick;
  uint16_t time_us;
  // 确保初始是低电平
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  Delay_us(2);

  // 发送10us触发脉冲
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);

  Delay_us(10);

  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  // 等待ECHO上升
  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_RIGHT_ECHO_GPIO_Port, US_RIGHT_ECHO_Pin) ==
         GPIO_PIN_RESET) {
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  // 开始计时
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_RIGHT_ECHO_GPIO_Port, US_RIGHT_ECHO_Pin) ==
         GPIO_PIN_SET) {
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  time_us = __HAL_TIM_GET_COUNTER(&htim3);
  // 计算距离（mm）
  return (float)time_us * 0.343f / 2.0f;
}

float HCSR04_LeftRead(void) {
  uint32_t start_tick;
  uint16_t time_us;
  // 确保初始是低电平
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  Delay_us(2);

  // 发送10us触发脉冲
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);

  Delay_us(10);

  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  // 等待ECHO上升
  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_LEFT_ECHO_GPIO_Port, US_LEFT_ECHO_Pin) ==
         GPIO_PIN_RESET) {
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  // 开始计时
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_LEFT_ECHO_GPIO_Port, US_LEFT_ECHO_Pin) ==
         GPIO_PIN_SET) {
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  time_us = __HAL_TIM_GET_COUNTER(&htim3);
  // 计算距离（mm）
  return (float)time_us * 0.343f / 2.0f;
}