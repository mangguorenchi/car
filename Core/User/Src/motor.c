#include "motor.h"
#include "gpio.h"
#include "tim.h"

#define MAX_Speed 7199
// max speed=MAX_Speed

void Motor_Init(void) {
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  Motor_StopAll();
}

void Motor_SetSpeed(Motor_ID motor, int16_t speed) {
  uint16_t pwm;

  // 限速
  if (speed > MAX_Speed)
    speed = MAX_Speed;

  if (speed < -MAX_Speed)
    speed = -MAX_Speed;

  if (speed == 0) {
    pwm = 0;
  } else {
    pwm = (speed > 0) ? speed : -speed;
  }

  switch (motor) {
  case MOTOR_M1:
    if (speed > 0) {
      // 控制M1
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (speed < 0) {

      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }

    // pwma= TIM2_CH1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);

    break;

  case MOTOR_M2:
    if (speed > 0) {
      // 控制M2
      HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);

      HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
    } else if (speed < 0) {

      HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);

    break;

  case MOTOR_M3:
    if (speed > 0) {
      // 控制M3
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else if (speed < 0) {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);

      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);

    break;

  case MOTOR_M4:
    if (speed > 0) {
      // 控制M4
      HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
    } else if (speed < 0) {

      HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);

      HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);

      HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);

    break;

  default:
    break;
  }
}

void Motor_StopAll(void) {
  Motor_SetSpeed(MOTOR_M1, 0);
  Motor_SetSpeed(MOTOR_M2, 0);
  Motor_SetSpeed(MOTOR_M3, 0);
  Motor_SetSpeed(MOTOR_M4, 0);
}