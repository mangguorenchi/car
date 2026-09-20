#include "control.h"
#include "motor.h"
#include "mpu6050.h"
#include "sensor.h"
#include <stdio.h>

#define CONTROL_SPEED 3600
#define TURN_SPEED 3000
#define FRONT_OBSTACLE_DISTANCE 425.0f
#define FRONT_CLEAR_DISTANCE 450.0f
#define FRONT_CONFIRM_COUNT 2
#define SCAN_TURN_TARGET 30.0f
#define SCAN_TOTAL_TURN_TARGET 60.0f
#define RETURN_TURN_TARGET 30.0f
#define AVOID_MAX_TURN_TIME 1500
#define AVOID_FORWARD_TIME 700
#define AVOID_STOP_TIME 100
#define SCAN_DIFF_DISTANCE 100.0f

typedef enum {
  AVOID_STATE_RUN = 0,
  AVOID_STATE_STOP_WAIT,
  AVOID_STATE_SCAN_LEFT,
  AVOID_STATE_SCAN_LEFT_WAIT,
  AVOID_STATE_SCAN_RIGHT,
  AVOID_STATE_SCAN_RIGHT_WAIT,
  AVOID_STATE_CHOOSE_DIRECTION,
  AVOID_STATE_TURN_TO_LEFT,
  AVOID_STATE_FORWARD,
  AVOID_STATE_RETURN_LEFT,
  AVOID_STATE_RETURN_RIGHT
} Avoid_State_t;

static Avoid_State_t avoid_state;
static uint32_t avoid_state_tick;
static float avoid_turn_target;
static float scan_left_distance;
static float scan_right_distance;
static uint8_t avoid_direction;
static uint8_t front_obstacle_count;

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
  avoid_turn_target = SCAN_TURN_TARGET;
  scan_left_distance = -1.0f;
  scan_right_distance = -1.0f;
  avoid_direction = 0;
  front_obstacle_count = 0;
}

void Control_AvoidanceTask(void) {
  uint32_t now = HAL_GetTick();
  float front_distance;

  switch (avoid_state) {
  case AVOID_STATE_RUN:
    front_distance = HCSR04_FrontRead();

    if (front_distance < 0) {
      Control_Stop();
      front_obstacle_count = 0;
      printf("Ultrasonic ERROR\r\n");
      return;
    }

    if (front_distance <= FRONT_OBSTACLE_DISTANCE) {
      if (front_obstacle_count < FRONT_CONFIRM_COUNT) {
        front_obstacle_count++;
      }

      if (front_obstacle_count >= FRONT_CONFIRM_COUNT) {
        Control_Stop();
        avoid_state = AVOID_STATE_STOP_WAIT;
        avoid_state_tick = now;
        front_obstacle_count = 0;
      } else {
        Control_Forward(CONTROL_SPEED);
      }
    } else {
      if (front_distance >= FRONT_CLEAR_DISTANCE) {
        front_obstacle_count = 0;
      }
      Control_Forward(CONTROL_SPEED);
    }
    break;

  case AVOID_STATE_STOP_WAIT:
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      MPU6050_ResetYaw();
      avoid_turn_target = SCAN_TURN_TARGET;
      Control_TurnLeft(TURN_SPEED);
      avoid_state = AVOID_STATE_SCAN_LEFT;
      avoid_state_tick = now;
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
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      scan_left_distance = HCSR04_FrontRead();

      if (scan_left_distance < 0) {
        Control_Stop();
        printf("Ultrasonic ERROR\r\n");
        avoid_state = AVOID_STATE_RUN;
        avoid_state_tick = now;
        break;
      }

      printf("Scan left 30 deg: %.1f mm\r\n", scan_left_distance);

      MPU6050_ResetYaw();
      avoid_turn_target = SCAN_TOTAL_TURN_TARGET;
      Control_TurnRight(TURN_SPEED);
      avoid_state = AVOID_STATE_SCAN_RIGHT;
      avoid_state_tick = now;
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
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      scan_right_distance = HCSR04_FrontRead();

      if (scan_right_distance < 0) {
        Control_Stop();
        printf("Ultrasonic ERROR\r\n");
        avoid_state = AVOID_STATE_RUN;
        avoid_state_tick = now;
        break;
      }

      printf("Scan right 30 deg: %.1f mm\r\n", scan_right_distance);
      avoid_state = AVOID_STATE_CHOOSE_DIRECTION;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_CHOOSE_DIRECTION:
    if (scan_left_distance > scan_right_distance + SCAN_DIFF_DISTANCE) {
      avoid_direction = 1;
      MPU6050_ResetYaw();
      avoid_turn_target = SCAN_TOTAL_TURN_TARGET;
      Control_TurnLeft(TURN_SPEED);
      avoid_state = AVOID_STATE_TURN_TO_LEFT;
      avoid_state_tick = now;
    } else {
      avoid_direction = 2;
      Control_Forward(CONTROL_SPEED);
      avoid_state = AVOID_STATE_FORWARD;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_TO_LEFT:
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_FORWARD;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_FORWARD:
    Control_Forward(CONTROL_SPEED);
    if (now - avoid_state_tick >= AVOID_FORWARD_TIME) {
      Control_Stop();
      MPU6050_ResetYaw();
      avoid_turn_target = RETURN_TURN_TARGET;

      if (avoid_direction == 1) {
        Control_TurnRight(TURN_SPEED);
        avoid_state = AVOID_STATE_RETURN_RIGHT;
      } else if (avoid_direction == 2) {
        Control_TurnLeft(TURN_SPEED);
        avoid_state = AVOID_STATE_RETURN_LEFT;
      } else {
        avoid_state = AVOID_STATE_RUN;
      }

      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_RETURN_LEFT:
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
      front_obstacle_count = 0;
    }
    break;

  case AVOID_STATE_RETURN_RIGHT:
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
      front_obstacle_count = 0;
    }
    break;

  default:
    Control_AvoidanceReset();
    break;
  }
}
