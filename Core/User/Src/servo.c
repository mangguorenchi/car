#include "servo.h"
#include "tim.h"

#define SERVO_STOP_PULSE 1500
#define SERVO_FORWARD_PULSE 1700
#define SERVO_BACKWARD_PULSE 1300

static uint32_t servo_stop_tick[4];

static void Servo_SetPulse(uint8_t servo_id, uint16_t pulse) {
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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  Servo_Stop(1);
  Servo_Stop(2);
  Servo_Stop(3);
  Servo_Stop(4);
}

void Servo_Task(void) {
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0; i < 4; i++) {
    if (servo_stop_tick[i] != 0 && now >= servo_stop_tick[i]) {
      Servo_Stop(i + 1);
      servo_stop_tick[i] = 0;
    }
  }
}

void Servo_Stop(uint8_t servo_id) {
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  Servo_SetPulse(servo_id, SERVO_STOP_PULSE);
  servo_stop_tick[servo_id - 1] = 0;
}

void Servo_TurnForward(uint8_t servo_id, uint8_t time_ms) {
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  Servo_SetPulse(servo_id, SERVO_FORWARD_PULSE);
  servo_stop_tick[servo_id - 1] = HAL_GetTick() + time_ms;
}

void Servo_TurnBackward(uint8_t servo_id, uint8_t time_ms) {
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  Servo_SetPulse(servo_id, SERVO_BACKWARD_PULSE);
  servo_stop_tick[servo_id - 1] = HAL_GetTick() + time_ms;
}
