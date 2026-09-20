#include "control.h"
#include "motor.h"
#include "mpu6050.h"
#include "sensor.h"
#include <stdio.h>

#define Control_Speed 2400
#define Turn_Speed 3600
#define FRONT_OBSTACLE_DISTANCE 450.0f
#define SIDE_OBSTACLE_DISTANCE 300.0f
#define CORNER_TURN_TARGET 90.0f
#define OBSTACLE_TURN_TARGET 60.0f
#define SIDE_OPEN_DISTANCE 550.0f
#define AVOID_MAX_TURN_TIME 1500
#define AVOID_FORWARD_TIME 700
#define AVOID_BACKWARD_TIME 1000

typedef enum {
  AVOID_STATE_RUN = 0,
  AVOID_STATE_STOP_WAIT,
  AVOID_STATE_CHECK_SIDE,
  AVOID_STATE_TURN_LEFT,
  AVOID_STATE_TURN_RIGHT,
  AVOID_STATE_BACKWARD,
  AVOID_STATE_FORWARD_AFTER_TURN,
  AVOID_STATE_RETURN_LEFT,
  AVOID_STATE_RETURN_RIGHT,
  AVOID_STATE_TURN_STOP
} Avoid_State_t;

static Avoid_State_t avoid_state;
static uint32_t avoid_state_tick;
static uint8_t avoid_turn_direction;
static uint8_t avoid_need_return;
static float avoid_turn_target;

static uint8_t Control_TurnFinished(uint8_t direction) {
  float yaw = MPU6050_GetYaw();

  if (direction == 1 && yaw <= -avoid_turn_target) {
    return 1;
  }

  if (direction == 2 && yaw >= avoid_turn_target) {
    return 1;
  }

  if (HAL_GetTick() - avoid_state_tick >= AVOID_MAX_TURN_TIME) {
    printf("Gyro turn timeout, yaw=%.2f\r\n", yaw);
    return 1;
  }

  return 0;
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
  Motor_SetSpeed(MOTOR_M1, -speed);
  Motor_SetSpeed(MOTOR_M2, speed);
  Motor_SetSpeed(MOTOR_M3, -speed);
  Motor_SetSpeed(MOTOR_M4, speed);
}
void Control_TurnRight(uint16_t speed) {
  Motor_SetSpeed(MOTOR_M1, speed);
  Motor_SetSpeed(MOTOR_M2, -speed);
  Motor_SetSpeed(MOTOR_M3, speed);
  Motor_SetSpeed(MOTOR_M4, -speed);
}

void Control_Stop(void) { Motor_StopAll(); }

void Control_AvoidanceReset(void) {
  avoid_state = AVOID_STATE_RUN;
  avoid_state_tick = HAL_GetTick();
  avoid_turn_direction = 0;
  avoid_need_return = 0;
  avoid_turn_target = OBSTACLE_TURN_TARGET;
}
// 在ai建议下利用定时器改为了非阻塞状态 9/13日
void Control_AvoidanceTask(void) {
  uint32_t now = HAL_GetTick();
  float front_distance;
  float left_distance;
  float right_distance;

  switch (avoid_state) {
  case AVOID_STATE_RUN:
    front_distance = HCSR04_FrontRead();

    if (front_distance < 0) {
      Control_Stop();
      printf("Ultrasonic ERROR\r\n");
      return;
    }

    if (front_distance <= FRONT_OBSTACLE_DISTANCE) {
      Control_Stop();
      avoid_state = AVOID_STATE_STOP_WAIT;
      avoid_state_tick = now;
    } else {
      Control_Forward(Control_Speed);
    }
    break;

  case AVOID_STATE_STOP_WAIT:
    if (now - avoid_state_tick >= 200) {
      avoid_state = AVOID_STATE_CHECK_SIDE;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_CHECK_SIDE:
    left_distance = HCSR04_LeftRead();
    right_distance = HCSR04_RightRead();

    if (left_distance < 0 || right_distance < 0) {
      Control_Stop();
      printf("Ultrasonic ERROR\r\n");
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
      break;
    }

    if (right_distance >= SIDE_OPEN_DISTANCE) {
      avoid_turn_direction = 2;
      avoid_need_return = 0;
      avoid_turn_target = CORNER_TURN_TARGET;
      MPU6050_ResetYaw();
      Control_TurnRight(Turn_Speed);
      avoid_state = AVOID_STATE_TURN_RIGHT;
      avoid_state_tick = now;
    } else if (left_distance >= SIDE_OPEN_DISTANCE) {
      avoid_turn_direction = 1;
      avoid_need_return = 0;
      avoid_turn_target = CORNER_TURN_TARGET;
      MPU6050_ResetYaw();
      Control_TurnLeft(Turn_Speed);
      avoid_state = AVOID_STATE_TURN_LEFT;
      avoid_state_tick = now;
    } else if (left_distance <= SIDE_OBSTACLE_DISTANCE &&
               right_distance <= SIDE_OBSTACLE_DISTANCE) {
      if (left_distance > right_distance) {
        avoid_turn_direction = 1;
      } else {
        avoid_turn_direction = 2;
      }
      avoid_need_return = 1;
      avoid_turn_target = OBSTACLE_TURN_TARGET;
      Control_Backward(Control_Speed);
      avoid_state = AVOID_STATE_BACKWARD;
      avoid_state_tick = now;
    } else if (left_distance > right_distance) {
      avoid_turn_direction = 1;
      avoid_need_return = 1;
      avoid_turn_target = OBSTACLE_TURN_TARGET;
      MPU6050_ResetYaw();
      Control_TurnLeft(Turn_Speed);
      avoid_state = AVOID_STATE_TURN_LEFT;
      avoid_state_tick = now;
    } else {
      avoid_turn_direction = 2;
      avoid_need_return = 1;
      avoid_turn_target = OBSTACLE_TURN_TARGET;
      MPU6050_ResetYaw();
      Control_TurnRight(Turn_Speed);
      avoid_state = AVOID_STATE_TURN_RIGHT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_LEFT:
    if (Control_TurnFinished(1)) {
      if (avoid_need_return) {
        Control_Forward(Control_Speed);
        avoid_state = AVOID_STATE_FORWARD_AFTER_TURN;
      } else {
        Control_Stop();
        avoid_state = AVOID_STATE_TURN_STOP;
      }
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_RIGHT:
    if (Control_TurnFinished(2)) {
      if (avoid_need_return) {
        Control_Forward(Control_Speed);
        avoid_state = AVOID_STATE_FORWARD_AFTER_TURN;
      } else {
        Control_Stop();
        avoid_state = AVOID_STATE_TURN_STOP;
      }
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_BACKWARD:
    if (now - avoid_state_tick >= AVOID_BACKWARD_TIME) {
      if (avoid_turn_direction == 1) {
        MPU6050_ResetYaw();
        Control_TurnLeft(Turn_Speed);
        avoid_state = AVOID_STATE_TURN_LEFT;
      } else {
        MPU6050_ResetYaw();
        Control_TurnRight(Turn_Speed);
        avoid_state = AVOID_STATE_TURN_RIGHT;
      }
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_FORWARD_AFTER_TURN:
    if (now - avoid_state_tick >= AVOID_FORWARD_TIME) {
      avoid_turn_target = OBSTACLE_TURN_TARGET;
      if (avoid_turn_direction == 1) {
        MPU6050_ResetYaw();
        Control_TurnRight(Turn_Speed);
        avoid_state = AVOID_STATE_RETURN_RIGHT;
      } else {
        MPU6050_ResetYaw();
        Control_TurnLeft(Turn_Speed);
        avoid_state = AVOID_STATE_RETURN_LEFT;
      }
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_RETURN_LEFT:
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_TURN_STOP;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_RETURN_RIGHT:
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_TURN_STOP;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_STOP:
    if (now - avoid_state_tick >= 100) {
      Control_Forward(Control_Speed);
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
    }
    break;

  default:
    Control_AvoidanceReset();
    break;
  }
}
