#include "servo.h"
#include "tim.h"

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_STEP_ANGLE 10
#define SERVO_360_MAX_ANGLE 360
#define SERVO_180_MAX_ANGLE 180

static uint16_t servo_angle[4];

static uint16_t Servo_GetMaxAngle(uint8_t servo_id) {
  if (servo_id == 1 || servo_id == 2 || servo_id == 3) {
    return SERVO_360_MAX_ANGLE;
  }

  return SERVO_180_MAX_ANGLE;
}

static uint16_t Servo_AngleToPulse(uint8_t servo_id, uint16_t angle) {
  uint16_t max_angle = Servo_GetMaxAngle(servo_id);

  if (angle > max_angle) {
    angle = max_angle;
  }

  return SERVO_MIN_PULSE +
         (uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / max_angle;
}

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
}

void Servo_Task(void) {
}

void Servo_Stop(uint8_t servo_id) {
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  Servo_SetAngle(servo_id, servo_angle[servo_id - 1]);
}

void Servo_SetAngle(uint8_t servo_id, uint16_t angle) {
  uint16_t max_angle;

  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  max_angle = Servo_GetMaxAngle(servo_id);

  if (angle > max_angle) {
    angle = max_angle;
  }

  servo_angle[servo_id - 1] = angle;
  Servo_SetPulse(servo_id, Servo_AngleToPulse(servo_id, angle));
}

void Servo_AddAngle(uint8_t servo_id) {
  uint16_t max_angle;
  uint16_t angle;

  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  max_angle = Servo_GetMaxAngle(servo_id);
  angle = servo_angle[servo_id - 1];

  if (angle + SERVO_STEP_ANGLE > max_angle) {
    Servo_SetAngle(servo_id, max_angle);
  } else {
    Servo_SetAngle(servo_id, angle + SERVO_STEP_ANGLE);
  }
}

void Servo_SubAngle(uint8_t servo_id) {
  uint16_t angle;

  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  angle = servo_angle[servo_id - 1];

  if (angle < SERVO_STEP_ANGLE) {
    Servo_SetAngle(servo_id, 0);
  } else {
    Servo_SetAngle(servo_id, angle - SERVO_STEP_ANGLE);
  }
}

void Servo_TurnForward(uint8_t servo_id, uint8_t time_ms) {
  (void)time_ms;
  Servo_AddAngle(servo_id);
}

void Servo_TurnBackward(uint8_t servo_id, uint8_t time_ms) {
  (void)time_ms;
  Servo_SubAngle(servo_id);
}
