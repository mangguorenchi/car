#include "sensor.h"
#include "gpio.h"
#include "tim.h"

static void Delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  while (__HAL_TIM_GET_COUNTER(&htim3) < us) {
  }
}

static float HCSR04_Read(GPIO_TypeDef *trig_port,
                         uint16_t trig_pin,
                         GPIO_TypeDef *echo_port,
                         uint16_t echo_pin) {
  uint32_t start_tick;
  uint32_t time_us;

  // 发送触发脉冲
  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
  Delay_us(2);

  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
  Delay_us(10);
  HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

  // 等待当前传感器的 Echo 变高
  start_tick = HAL_GetTick();
  while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET) {
    if (HAL_GetTick() - start_tick > 100U) {
      return -1.0f;
    }
  }

  // Echo 高电平持续时间就是超声波往返时间
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET) {
    if (HAL_GetTick() - start_tick > 100U) {
      return -1.0f;
    }
  }

  time_us = __HAL_TIM_GET_COUNTER(&htim3);

  // 声速约为 0.343 mm/us，除以 2 得到单程距离
  return (float)time_us * 0.343f / 2.0f;
}

void Sensor_Init(void) {
  HAL_TIM_Base_Start(&htim3);

  // 前方 Trig 保持低电平
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
}

float HCSR04_FrontRead(void) {
  return HCSR04_Read(US_TRIG_GPIO_Port,
                     US_TRIG_Pin,
                     US_FRONT_ECHO_GPIO_Port,
                     US_FRONT_ECHO_Pin);
}
