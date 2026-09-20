#include "servo.h"
#include "tim.h"

// 360度连续旋转舵机：1500us附近表示停止
#define SERVO_STOP_PULSE 1500
// 大于1500us向一个方向转，数值越大速度越快
#define SERVO_FORWARD_PULSE 1700
// 小于1500us向另一个方向转，数值越小速度越快
#define SERVO_BACKWARD_PULSE 1300

// 每个舵机需要停止的时间点，0表示当前不需要自动停止
static uint32_t servo_stop_tick[4];

static void Servo_SetPulse(uint8_t servo_id, uint16_t pulse) {
  // 根据舵机编号选择TIM1对应的PWM通道
  if (servo_id == 1) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
  } else if (servo_id == 2) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
  } else if (servo_id == 3) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
  } else if (servo_id == 4) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
  }
}

void Servo_Init(void) {
  // 启动TIM1四路PWM，用来控制四个舵机
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // 上电后先让全部舵机停止
  Servo_Stop(1);
  Servo_Stop(2);
  Servo_Stop(3);
  Servo_Stop(4);
}

void Servo_Task(void) {
  uint32_t now = HAL_GetTick();

  // 周期检查每个舵机是否到了停止时间
  for (uint8_t i = 0; i < 4; i++) {
    if (servo_stop_tick[i] != 0 && now >= servo_stop_tick[i]) {
      // i从0开始，舵机编号从1开始，所以这里要加1
      Servo_Stop(i + 1);
      servo_stop_tick[i] = 0;
    }
  }
}

void Servo_Stop(uint8_t servo_id) {
  // 防止传入不存在的舵机编号
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  // 连续旋转舵机给1500us就是停止
  Servo_SetPulse(servo_id, SERVO_STOP_PULSE);
  servo_stop_tick[servo_id - 1] = 0;
}

void Servo_TurnForward(uint8_t servo_id, uint8_t time_ms) {
  // 防止数组越界
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  // 先让舵机正方向转
  Servo_SetPulse(servo_id, SERVO_FORWARD_PULSE);
  // 记录停止时间，后面由Servo_Task自动停止
  servo_stop_tick[servo_id - 1] = HAL_GetTick() + time_ms;
}

void Servo_TurnBackward(uint8_t servo_id, uint8_t time_ms) {
  // 防止数组越界
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  // 先让舵机反方向转
  Servo_SetPulse(servo_id, SERVO_BACKWARD_PULSE);
  // 记录停止时间，后面由Servo_Task自动停止
  servo_stop_tick[servo_id - 1] = HAL_GetTick() + time_ms;
}
