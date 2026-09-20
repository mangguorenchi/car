#include "motor.h"
#include "gpio.h"
#include "tim.h"

// TIM2的ARR是7199，所以PWM占空比最大也用7199
#define MAX_Speed 7199

void Motor_Init(void) {
  // 启动TIM2的4路PWM，分别控制4个电机的速度
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // 初始化完成后先全部停止，防止上电后电机突然转动
  Motor_StopAll();
}

void Motor_SetSpeed(Motor_ID motor, int16_t speed) {
  uint16_t pwm;

  // 限制速度范围，防止PWM值超过定时器最大值
  if (speed > MAX_Speed)
    speed = MAX_Speed;

  if (speed < -MAX_Speed)
    speed = -MAX_Speed;

  // speed的正负表示方向，PWM只需要速度大小，所以要取绝对值
  if (speed == 0) {
    pwm = 0;
  } else {
    pwm = (speed > 0) ? speed : -speed;
  }

  // 根据电机编号，分别设置方向引脚和PWM通道
  switch (motor) {
  case MOTOR_M1:
    if (speed > 0) {
      // M1正转
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
      // M1反转
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else {
      // 两个方向脚都拉低，让M1停止
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }

    // M1的PWM接在TIM2_CH1
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);

    break;

  case MOTOR_M2:
    if (speed > 0) {
      // M2正转
      HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
      // M2反转
      HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_SET);
    } else {
      // M2停止
      HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, GPIO_PIN_RESET);
    }

    // M2的PWM接在TIM2_CH4
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);

    break;

  case MOTOR_M3:
    if (speed > 0) {
      // M3正转
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else if (speed < 0) {
      // M3反转
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else {
      // M3停止
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }

    // M3的PWM接在TIM2_CH2
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);

    break;

  case MOTOR_M4:
    if (speed > 0) {
      // M4正转
      HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_SET);
    } else if (speed < 0) {
      // M4反转
      HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
    } else {
      // M4停止
      HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);
    }

    // M4的PWM接在TIM2_CH3
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);

    break;

  default:
    break;
  }
}

void Motor_StopAll(void) {
  // 调用统一的设置速度函数，速度给0就是停止
  Motor_SetSpeed(MOTOR_M1, 0);
  Motor_SetSpeed(MOTOR_M2, 0);
  Motor_SetSpeed(MOTOR_M3, 0);
  Motor_SetSpeed(MOTOR_M4, 0);
}
