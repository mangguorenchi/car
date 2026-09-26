#include "servo.h"
#include "tim.h"

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_STEP_ANGLE 10
#define SERVO4_STEP_ANGLE 1
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
// 舵机2、3不设置上电初始角度，首次收到指令时再建立软件位置
static uint16_t servo_angle[4] = {0, 0, 0, 160};
static uint8_t servo_position_known[4] = {0, 0, 0, 1};

// 舵机1自动停止的时间点
static uint32_t servo1_stop_tick;

// 舵机2、3只在收到指令时输出PWM，时间到后释放保持力矩
static uint32_t servo2_stop_tick;
static uint32_t servo3_stop_tick;

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

static uint16_t Servo_PulseToAngle(uint16_t pulse) {
  if (pulse < SERVO_MIN_PULSE) {
    pulse = SERVO_MIN_PULSE;
  }

  if (pulse > SERVO_MAX_PULSE) {
    pulse = SERVO_MAX_PULSE;
  }

  return (uint32_t)(pulse - SERVO_MIN_PULSE) * SERVO_180_MAX_ANGLE /
         (SERVO_MAX_PULSE - SERVO_MIN_PULSE);
}

static uint8_t Servo_PrepareRelativeMove(uint8_t servo_id) {
  uint32_t pulse;

  if (servo_id != 2 && servo_id != 3) {
    return 1;
  }

  if (servo_position_known[servo_id - 1]) {
    return 1;
  }

  pulse = (servo_id == 2) ? __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2)
                          : __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
  servo_angle[servo_id - 1] = Servo_PulseToAngle((uint16_t)pulse);
  servo_position_known[servo_id - 1] = 1;
  return 0;
}

void Servo_Init(void) {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // 舵机1上电后先保持停止
  Servo_SetPulse(1, SERVO1_STOP_PULSE);
  // 第五个舵机上电后保持停止，避免超声波支架自行旋转
  Servo_US_Stop();

  // 舵机2、3机械联动，为避免互相顶住，不启动PWM输出，不固定角度

  // 舵机4是180度角度舵机，上电后转到预设位置
  Servo_SetAngle(4, servo_angle[3]);
}

void Servo_Task(void) {
  // 舵机1运行到设定时间后自动停止
  if (servo1_stop_tick != 0 && HAL_GetTick() >= servo1_stop_tick) {
    Servo_SetPulse(1, SERVO1_STOP_PULSE);
    servo1_stop_tick = 0;
  }

  // 舵机2、3运行到设定时间后停止PWM，避免机械联动时持续顶住
  if (servo2_stop_tick != 0 && HAL_GetTick() >= servo2_stop_tick) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    servo2_stop_tick = 0;
  }

  if (servo3_stop_tick != 0 && HAL_GetTick() >= servo3_stop_tick) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    servo3_stop_tick = 0;
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

  if (servo_id == 2) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    servo2_stop_tick = 0;
    return;
  }

  if (servo_id == 3) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    servo3_stop_tick = 0;
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

  // 舵机2、3临时输出PWM，转动后释放保持力矩
  if (servo_id == 2 || servo_id == 3) {
    if (!Servo_PrepareRelativeMove(servo_id)) {
      return;
    }

    Servo_AddAngle(servo_id);
    if (servo_id == 2) {
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      servo2_stop_tick = HAL_GetTick() + time_ms;
    } else {
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      servo3_stop_tick = HAL_GetTick() + time_ms;
    }
    return;
  }

  // 舵机4每次增加1度
  if (servo_angle[3] + SERVO4_STEP_ANGLE > SERVO_180_MAX_ANGLE) {
    Servo_SetAngle(4, SERVO_180_MAX_ANGLE);
  } else {
    Servo_SetAngle(4, servo_angle[3] + SERVO4_STEP_ANGLE);
  }
}

void Servo_TurnBackward(uint8_t servo_id, uint8_t time_ms) {
  // 舵机1使用连续旋转控制，反方向转动一小段时间后自动停止
  if (servo_id == 1) {
    Servo_SetPulse(1, SERVO1_BACKWARD_PULSE);
    servo1_stop_tick = HAL_GetTick() + time_ms;
    return;
  }

  // 舵机2、3临时输出PWM，转动后释放保持力矩
  if (servo_id == 2 || servo_id == 3) {
    if (!Servo_PrepareRelativeMove(servo_id)) {
      return;
    }

    Servo_SubAngle(servo_id);
    if (servo_id == 2) {
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      servo2_stop_tick = HAL_GetTick() + time_ms;
    } else {
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      servo3_stop_tick = HAL_GetTick() + time_ms;
    }
    return;
  }

  // 舵机4每次减少1度
  if (servo_angle[3] < SERVO4_STEP_ANGLE) {
    Servo_SetAngle(4, 0);
  } else {
    Servo_SetAngle(4, servo_angle[3] - SERVO4_STEP_ANGLE);
  }
}

void Servo_US_TurnLeft(void) {
  // 超声波倒装后，舵机正方向对应物理左侧
  Servo_SetPulse(5, SERVO_US_FORWARD_PULSE);
}

void Servo_US_TurnRight(void) { Servo_SetPulse(5, SERVO_US_BACKWARD_PULSE); }

void Servo_US_Stop(void) { Servo_SetPulse(5, SERVO_US_STOP_PULSE); }
