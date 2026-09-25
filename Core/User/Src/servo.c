#include "servo.h"
#include "tim.h"

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_STEP_ANGLE 10
#define SERVO_360_MAX_ANGLE 360
#define SERVO_180_MAX_ANGLE 180

// 舵机1是360度连续旋转舵机
// 1500us附近停止，1700us和1300us分别向两个方向转动
#define SERVO1_STOP_PULSE 1500
#define SERVO1_FORWARD_PULSE 1700
#define SERVO1_BACKWARD_PULSE 1300

// 第五个舵机是360度连续旋转舵机，用于带动超声波旋转
#define SERVO_US_STOP_PULSE 1500
#define SERVO_US_FORWARD_PULSE 1700
#define SERVO_US_BACKWARD_PULSE 1300

// 舵机1使用连续旋转控制，舵机2、3、4使用角度控制
// 舵机2默认从170度开始，舵机3默认从90度开始
static uint16_t servo_angle[4] = {0, 170, 90, 160};

// 舵机1自动停止的时间点
static uint32_t servo1_stop_tick;

static uint16_t Servo_GetMaxAngle(uint8_t servo_id) {
  if (servo_id == 1) {
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
  } else if (servo_id == 5) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
  }
}

void Servo_Init(void) {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // 舵机1上电后先保持停止
  Servo_SetPulse(1, SERVO1_STOP_PULSE);
  // 第五个舵机上电后保持停止，避免超声波支架自行旋转
  Servo_US_Stop();

  // 舵机2是180度角度舵机，上电后转到预设位置
  Servo_SetAngle(2, servo_angle[1]);

  // 舵机3不上电主动复位，保持CubeMX设置的初始脉宽
  // 它的内部角度从90度开始记录

  // 舵机4是180度角度舵机，上电后转到预设位置
  Servo_SetAngle(4, servo_angle[3]);
}

void Servo_Task(void) {
  // 舵机1运行到设定时间后自动停止
  if (servo1_stop_tick != 0 && HAL_GetTick() >= servo1_stop_tick) {
    Servo_SetPulse(1, SERVO1_STOP_PULSE);
    servo1_stop_tick = 0;
  }

}

void Servo_Stop(uint8_t servo_id) {
  if (servo_id < 1 || servo_id > 4) {
    return;
  }

  // 舵机1是360度舵机，停止时使用停止脉宽
  if (servo_id == 1) {
    Servo_SetPulse(1, SERVO1_STOP_PULSE);
    servo1_stop_tick = 0;
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
  // 舵机1使用连续旋转控制，转动一小段时间后自动停止
  if (servo_id == 1) {
    Servo_SetPulse(1, SERVO1_FORWARD_PULSE);
    servo1_stop_tick = HAL_GetTick() + time_ms;
    return;
  }

  // 舵机2、3、4使用角度控制，每次增加10度
  (void)time_ms;
  Servo_AddAngle(servo_id);
}

void Servo_TurnBackward(uint8_t servo_id, uint8_t time_ms) {
  // 舵机1使用连续旋转控制，反方向转动一小段时间后自动停止
  if (servo_id == 1) {
    Servo_SetPulse(1, SERVO1_BACKWARD_PULSE);
    servo1_stop_tick = HAL_GetTick() + time_ms;
    return;
  }

  // 舵机2、3、4使用角度控制，每次减少10度
  (void)time_ms;
  Servo_SubAngle(servo_id);
}

void Servo_US_TurnLeft(void) {
  // 超声波倒装后，舵机正方向对应物理左侧
  Servo_SetPulse(5, SERVO_US_FORWARD_PULSE);
}

void Servo_US_TurnRight(void) {
  Servo_SetPulse(5, SERVO_US_BACKWARD_PULSE);
}

void Servo_US_Stop(void) {
  Servo_SetPulse(5, SERVO_US_STOP_PULSE);
}
