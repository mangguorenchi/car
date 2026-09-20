#include "sensor.h"
#include "gpio.h"
#include "tim.h"

static void Delay_us(uint16_t us) {
  // TIM3配置成1MHz计数时，计数1次约等于1us
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  // 一直等到计数值达到目标微秒数
  while (__HAL_TIM_GET_COUNTER(&htim3) < us) {
  }
}

void Sensor_Init(void) {
  // 启动TIM3，用来做微秒级延时和测量Echo高电平时间
  HAL_TIM_Base_Start(&htim3);

  // 超声波Trig默认拉低，避免上电误触发
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

  // 等待前方Echo变高，表示超声波开始返回
  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_FRONT_ECHO_GPIO_Port, US_FRONT_ECHO_Pin) ==
         GPIO_PIN_RESET) {
    // 等太久还没变高，说明超声波没响应
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  // Echo变高后开始计时
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_FRONT_ECHO_GPIO_Port, US_FRONT_ECHO_Pin) ==
         GPIO_PIN_SET) {
    // Echo一直不变低，也认为本次测距失败
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  time_us = __HAL_TIM_GET_COUNTER(&htim3);
  // 距离mm = 时间us * 声速0.343mm/us / 2，除2是因为声音走了来回
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

  // 等待右侧Echo变高
  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_RIGHT_ECHO_GPIO_Port, US_RIGHT_ECHO_Pin) ==
         GPIO_PIN_RESET) {
    // 超时返回-1，外层代码会把它当作错误
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  // Echo高电平持续时间就是超声波往返时间
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_RIGHT_ECHO_GPIO_Port, US_RIGHT_ECHO_Pin) ==
         GPIO_PIN_SET) {
    // 防止传感器异常时卡在while里
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

  // 等待左侧Echo变高
  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_LEFT_ECHO_GPIO_Port, US_LEFT_ECHO_Pin) ==
         GPIO_PIN_RESET) {
    // 没收到回波时返回错误值
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  // 开始测量Echo高电平时间
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  start_tick = HAL_GetTick();

  while (HAL_GPIO_ReadPin(US_LEFT_ECHO_GPIO_Port, US_LEFT_ECHO_Pin) ==
         GPIO_PIN_SET) {
    // Echo高电平异常过长时退出
    if (HAL_GetTick() - start_tick > 100) {
      return -1.0f;
    }
  }

  time_us = __HAL_TIM_GET_COUNTER(&htim3);
  // 计算距离（mm）
  return (float)time_us * 0.343f / 2.0f;
}
