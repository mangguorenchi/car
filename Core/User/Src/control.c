#include "control.h"
#include "motor.h"
#include "mpu6050.h"
#include "sensor.h"
#include <stdio.h>

// 自动行驶速度，比之前3600降低一半
#define CONTROL_SPEED 1800

// 90度转弯速度
#define TURN_SPEED 3000

// 左右探测时的低速摇摆速度
#define SCAN_TURN_SPEED 1800

// 前方距离小于这个值时，认为遇到障碍，单位mm
#define FRONT_OBSTACLE_DISTANCE 425.0f

// 左右探测角度
#define SCAN_TURN_TARGET 30.0f

// 从左30度转到右30度，一共60度
#define SCAN_TOTAL_TURN_TARGET 60.0f

// 最终选择方向后，转90度进入下一段路
#define TURN_90_TARGET 90.0f

// 每次停车稳定时间
#define AVOID_STOP_TIME 1000U

// 转弯最大等待时间，防止陀螺仪异常时卡死
#define AVOID_MAX_TURN_TIME 3500U

// 两侧距离差超过这个值，才认为一边明显更空
#define SCAN_DIFF_DISTANCE 100.0f

typedef enum {
  // 正常向前行驶
  AVOID_STATE_RUN = 0,

  // 第一次遇障碍后先停车
  AVOID_STATE_STOP_WAIT,

  // 左转30度探测
  AVOID_STATE_SCAN_LEFT,

  // 左侧探测前停车等待
  AVOID_STATE_SCAN_LEFT_WAIT,

  // 右转60度到右侧探测位置
  AVOID_STATE_SCAN_RIGHT,

  // 右侧探测前停车等待
  AVOID_STATE_SCAN_RIGHT_WAIT,

  // 从右侧30度回到正前方
  AVOID_STATE_RETURN_CENTER,

  // 回正后判断左右距离
  AVOID_STATE_CHOOSE_DIRECTION,

  // 探测后左转90度
  AVOID_STATE_TURN_LEFT_90,

  // 探测后右转90度
  AVOID_STATE_TURN_RIGHT_90,

  // 下一次遇障碍时，直接左转90度
  AVOID_STATE_DIRECT_LEFT_90,

  // 下一次遇障碍时，直接右转90度
  AVOID_STATE_DIRECT_RIGHT_90
} Avoid_State_t;

static Avoid_State_t avoid_state;
static uint32_t avoid_state_tick;
static float avoid_turn_target;
static float scan_left_distance;
static float scan_right_distance;

// 0表示下一次遇障碍需要先左右探测
// 1表示下一次遇障碍直接左转90度
// 2表示下一次遇障碍直接右转90度
static uint8_t next_direct_turn_direction;

static uint8_t Control_TurnFinished(uint8_t direction) {
  float yaw = MPU6050_GetYaw();

  // 左转时Yaw变成负数
  if (direction == 1 && yaw <= -avoid_turn_target) {
    return 1;
  }

  // 右转时Yaw变成正数
  if (direction == 2 && yaw >= avoid_turn_target) {
    return 1;
  }

  // 超时也结束，避免一直卡在转弯状态
  if (HAL_GetTick() - avoid_state_tick >= AVOID_MAX_TURN_TIME) {
    printf("Gyro turn timeout, yaw=%.2f\r\n", yaw);
    return 1;
  }

  return 0;
}

static void Control_StartTurnLeft(float target, uint16_t speed,
                                  Avoid_State_t next_state,
                                  uint32_t now) {
  MPU6050_ResetYaw();
  avoid_turn_target = target;
  Control_TurnLeft(speed);
  avoid_state = next_state;
  avoid_state_tick = now;
}

static void Control_StartTurnRight(float target, uint16_t speed,
                                   Avoid_State_t next_state,
                                   uint32_t now) {
  MPU6050_ResetYaw();
  avoid_turn_target = target;
  Control_TurnRight(speed);
  avoid_state = next_state;
  avoid_state_tick = now;
}

void Control_Init(void) {
  Motor_Init();
  Sensor_Init();
}

void Control_Forward(uint16_t speed) {
  Motor_SetSpeed(MOTOR_M1, speed);
  Motor_SetSpeed(MOTOR_M2, speed);
  Motor_SetSpeed(MOTOR_M3, speed);
  Motor_SetSpeed(MOTOR_M4, speed);
}

void Control_Backward(uint16_t speed) {
  Motor_SetSpeed(MOTOR_M1, -speed);
  Motor_SetSpeed(MOTOR_M2, -speed);
  Motor_SetSpeed(MOTOR_M3, -speed);
  Motor_SetSpeed(MOTOR_M4, -speed);
}

void Control_TurnLeft(uint16_t speed) {
  Motor_SetSpeed(MOTOR_M1, speed);
  Motor_SetSpeed(MOTOR_M2, -speed);
  Motor_SetSpeed(MOTOR_M3, speed);
  Motor_SetSpeed(MOTOR_M4, -speed);
}

void Control_TurnRight(uint16_t speed) {
  Motor_SetSpeed(MOTOR_M1, -speed);
  Motor_SetSpeed(MOTOR_M2, speed);
  Motor_SetSpeed(MOTOR_M3, -speed);
  Motor_SetSpeed(MOTOR_M4, speed);
}

void Control_Stop(void) {
  Motor_StopAll();
}

void Control_AvoidanceReset(void) {
  avoid_state = AVOID_STATE_RUN;
  avoid_state_tick = HAL_GetTick();
  avoid_turn_target = TURN_90_TARGET;
  scan_left_distance = -1.0f;
  scan_right_distance = -1.0f;
  next_direct_turn_direction = 0;
}

void Control_AvoidanceTask(void) {
  uint32_t now = HAL_GetTick();
  float front_distance;

  switch (avoid_state) {
  case AVOID_STATE_RUN:
    front_distance = HCSR04_FrontRead();

    if (front_distance < 0) {
      Control_Stop();
      printf("Ultrasonic ERROR\r\n");
      return;
    }

    if (front_distance <= FRONT_OBSTACLE_DISTANCE) {
      // 遇到障碍马上停下，不再继续向前顶
      Control_Stop();
      avoid_state_tick = now;

      if (next_direct_turn_direction == 1) {
        Control_StartTurnLeft(TURN_90_TARGET, TURN_SPEED,
                              AVOID_STATE_DIRECT_LEFT_90, now);
      } else if (next_direct_turn_direction == 2) {
        Control_StartTurnRight(TURN_90_TARGET, TURN_SPEED,
                               AVOID_STATE_DIRECT_RIGHT_90, now);
      } else {
        avoid_state = AVOID_STATE_STOP_WAIT;
      }
    } else {
      Control_Forward(CONTROL_SPEED);
    }
    break;

  case AVOID_STATE_STOP_WAIT:
    // 第一次遇障碍，先停车1秒，再开始左右探测
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      scan_left_distance = -1.0f;
      scan_right_distance = -1.0f;
      Control_StartTurnLeft(SCAN_TURN_TARGET, SCAN_TURN_SPEED,
                            AVOID_STATE_SCAN_LEFT, now);
    }
    break;

  case AVOID_STATE_SCAN_LEFT:
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_SCAN_LEFT_WAIT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_SCAN_LEFT_WAIT:
    // 左侧位置停稳后读取距离
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      scan_left_distance = HCSR04_FrontRead();

      if (scan_left_distance < 0) {
        Control_Stop();
        printf("Ultrasonic ERROR\r\n");
        avoid_state_tick = now;
        break;
      }

      printf("Scan left 30 deg: %.1f mm\r\n", scan_left_distance);

      Control_StartTurnRight(SCAN_TOTAL_TURN_TARGET, SCAN_TURN_SPEED,
                             AVOID_STATE_SCAN_RIGHT, now);
    }
    break;

  case AVOID_STATE_SCAN_RIGHT:
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_SCAN_RIGHT_WAIT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_SCAN_RIGHT_WAIT:
    // 右侧位置停稳后读取距离
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      scan_right_distance = HCSR04_FrontRead();

      if (scan_right_distance < 0) {
        Control_Stop();
        printf("Ultrasonic ERROR\r\n");
        avoid_state_tick = now;
        break;
      }

      printf("Scan right 30 deg: %.1f mm\r\n", scan_right_distance);

      // 读取右侧后，先回正
      Control_StartTurnLeft(SCAN_TURN_TARGET, SCAN_TURN_SPEED,
                            AVOID_STATE_RETURN_CENTER, now);
    }
    break;

  case AVOID_STATE_RETURN_CENTER:
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_CHOOSE_DIRECTION;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_CHOOSE_DIRECTION:
    // 回正后，再按照左右距离选择90度转弯方向
    if (scan_left_distance > scan_right_distance + SCAN_DIFF_DISTANCE) {
      Control_StartTurnLeft(TURN_90_TARGET, TURN_SPEED,
                            AVOID_STATE_TURN_LEFT_90, now);
    } else {
      Control_StartTurnRight(TURN_90_TARGET, TURN_SPEED,
                             AVOID_STATE_TURN_RIGHT_90, now);
    }
    break;

  case AVOID_STATE_TURN_LEFT_90:
    if (Control_TurnFinished(1)) {
      Control_Stop();
      // 这次左转了，下一次遇障碍直接右转90度
      next_direct_turn_direction = 2;
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_RIGHT_90:
    if (Control_TurnFinished(2)) {
      Control_Stop();
      // 这次右转了，下一次遇障碍直接左转90度
      next_direct_turn_direction = 1;
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_DIRECT_LEFT_90:
    if (Control_TurnFinished(1)) {
      Control_Stop();
      // 直接转弯完成后清除记录，回到第一步
      next_direct_turn_direction = 0;
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_DIRECT_RIGHT_90:
    if (Control_TurnFinished(2)) {
      Control_Stop();
      // 直接转弯完成后清除记录，回到第一步
      next_direct_turn_direction = 0;
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
    }
    break;

  default:
    Control_Stop();
    Control_AvoidanceReset();
    break;
  }
}
