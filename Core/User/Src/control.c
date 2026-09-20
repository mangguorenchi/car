#include "control.h"
#include "motor.h"
#include "mpu6050.h"
#include "sensor.h"
#include <stdio.h>

// 小车直行速度，数值越大PWM占空比越大
#define Control_Speed 2400
// 小车原地转弯速度
#define Turn_Speed 3600
// 前方距离小于这个值时，认为前面有障碍，单位是mm
#define FRONT_OBSTACLE_DISTANCE 450.0f
// 左右两侧距离都小于这个值时，认为车身两边都比较窄
#define SIDE_OBSTACLE_DISTANCE 300.0f
// 遇到弯道或开口时使用的转弯角度
#define CORNER_TURN_TARGET 90.0f
// 普通避障时使用的转弯角度
#define OBSTACLE_TURN_TARGET 60.0f
// 侧边距离大于这个值时，认为这一侧比较空，可以当作路口
#define SIDE_OPEN_DISTANCE 550.0f
// 陀螺仪转弯最大等待时间，防止陀螺仪异常时一直转
#define AVOID_MAX_TURN_TIME 1500
// 避障转过去后向前走一小段的时间
#define AVOID_FORWARD_TIME 700
// 两边都很窄时，先后退一点再转弯
#define AVOID_BACKWARD_TIME 1000

typedef enum {
  // 正常前进，并不断检测前方距离
  AVOID_STATE_RUN = 0,
  // 检测到障碍后先停一下，让车身稳定
  AVOID_STATE_STOP_WAIT,
  // 读取左右距离，决定往哪边走
  AVOID_STATE_CHECK_SIDE,
  // 正在左转
  AVOID_STATE_TURN_LEFT,
  // 正在右转
  AVOID_STATE_TURN_RIGHT,
  // 两边太窄时先后退
  AVOID_STATE_BACKWARD,
  // 避开障碍后先向前走一小段
  AVOID_STATE_FORWARD_AFTER_TURN,
  // 避障后向左回正
  AVOID_STATE_RETURN_LEFT,
  // 避障后向右回正
  AVOID_STATE_RETURN_RIGHT,
  // 转弯结束后短暂停车
  AVOID_STATE_TURN_STOP
} Avoid_State_t;

// 当前避障状态
static Avoid_State_t avoid_state;
// 进入当前状态的时间，用来做非阻塞延时
static uint32_t avoid_state_tick;
// 记录这次避障选择的方向：1左转，2右转
static uint8_t avoid_turn_direction;
// 是否需要回正：普通避障需要回正，真正路口转弯不需要回正
static uint8_t avoid_need_return;
// 本次转弯目标角度
static float avoid_turn_target;

static uint8_t Control_TurnFinished(uint8_t direction) {
  // 读取陀螺仪积分出来的当前转角
  float yaw = MPU6050_GetYaw();

  // direction=1表示左转，当前代码中左转yaw会变成负数
  if (direction == 1 && yaw <= -avoid_turn_target) {
    return 1;
  }

  // direction=2表示右转，当前代码中右转yaw会变成正数
  if (direction == 2 && yaw >= avoid_turn_target) {
    return 1;
  }

  // 如果一直没有达到目标角度，也强制结束，防止卡死
  if (HAL_GetTick() - avoid_state_tick >= AVOID_MAX_TURN_TIME) {
    printf("Gyro turn timeout, yaw=%.2f\r\n", yaw);
    return 1;
  }

  return 0;
}

void Control_Init(void) {
  // 控制层初始化底层电机和传感器
  Motor_Init();
  Sensor_Init();
}

void Control_Forward(uint16_t speed) {
  // 四个轮子都给正速度，小车前进
  Motor_SetSpeed(MOTOR_M1, speed);
  Motor_SetSpeed(MOTOR_M2, speed);
  Motor_SetSpeed(MOTOR_M3, speed);
  Motor_SetSpeed(MOTOR_M4, speed);
}
void Control_Backward(uint16_t speed) {
  // 四个轮子都给负速度，小车后退
  Motor_SetSpeed(MOTOR_M1, -speed);
  Motor_SetSpeed(MOTOR_M2, -speed);
  Motor_SetSpeed(MOTOR_M3, -speed);
  Motor_SetSpeed(MOTOR_M4, -speed);
}

void Control_TurnLeft(uint16_t speed) {
  // 左边轮子后退，右边轮子前进，实现原地左转
  Motor_SetSpeed(MOTOR_M1, -speed);
  Motor_SetSpeed(MOTOR_M2, speed);
  Motor_SetSpeed(MOTOR_M3, -speed);
  Motor_SetSpeed(MOTOR_M4, speed);
}
void Control_TurnRight(uint16_t speed) {
  // 左边轮子前进，右边轮子后退，实现原地右转
  Motor_SetSpeed(MOTOR_M1, speed);
  Motor_SetSpeed(MOTOR_M2, -speed);
  Motor_SetSpeed(MOTOR_M3, speed);
  Motor_SetSpeed(MOTOR_M4, -speed);
}

void Control_Stop(void) { Motor_StopAll(); }

void Control_AvoidanceReset(void) {
  // 把自动避障流程重新回到初始状态
  avoid_state = AVOID_STATE_RUN;
  avoid_state_tick = HAL_GetTick();
  avoid_turn_direction = 0;
  avoid_need_return = 0;
  avoid_turn_target = OBSTACLE_TURN_TARGET;
}

// 自动避障任务，需要在while循环里反复调用
// 这里不用HAL_Delay长时间等待，而是用状态机和HAL_GetTick做非阻塞流程
void Control_AvoidanceTask(void) {
  uint32_t now = HAL_GetTick();
  float front_distance;
  float left_distance;
  float right_distance;

  switch (avoid_state) {
  case AVOID_STATE_RUN:
    // 正常运行时，只检测前方距离
    front_distance = HCSR04_FrontRead();

    // 读到负数表示超声波超时或异常
    if (front_distance < 0) {
      Control_Stop();
      printf("Ultrasonic ERROR\r\n");
      return;
    }

    // 前面太近，先停车，再进入下一步判断
    if (front_distance <= FRONT_OBSTACLE_DISTANCE) {
      Control_Stop();
      avoid_state = AVOID_STATE_STOP_WAIT;
      avoid_state_tick = now;
    } else {
      // 前方安全就继续前进
      Control_Forward(Control_Speed);
    }
    break;

  case AVOID_STATE_STOP_WAIT:
    // 停200ms，让车不再晃动，超声波读数也会稳定一点
    if (now - avoid_state_tick >= 200) {
      avoid_state = AVOID_STATE_CHECK_SIDE;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_CHECK_SIDE:
    // 读取左右距离，用来判断哪边更空
    left_distance = HCSR04_LeftRead();
    right_distance = HCSR04_RightRead();

    // 左右任意一个读数异常，就停止并重新开始流程
    if (left_distance < 0 || right_distance < 0) {
      Control_Stop();
      printf("Ultrasonic ERROR\r\n");
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
      break;
    }

    // 右侧很空，认为右边可能是路口，直接右转90度，不再回正
    if (right_distance >= SIDE_OPEN_DISTANCE) {
      avoid_turn_direction = 2;
      avoid_need_return = 0;
      avoid_turn_target = CORNER_TURN_TARGET;
      MPU6050_ResetYaw();
      Control_TurnRight(Turn_Speed);
      avoid_state = AVOID_STATE_TURN_RIGHT;
      avoid_state_tick = now;
    // 左侧很空，认为左边可能是路口，直接左转90度，不再回正
    } else if (left_distance >= SIDE_OPEN_DISTANCE) {
      avoid_turn_direction = 1;
      avoid_need_return = 0;
      avoid_turn_target = CORNER_TURN_TARGET;
      MPU6050_ResetYaw();
      Control_TurnLeft(Turn_Speed);
      avoid_state = AVOID_STATE_TURN_LEFT;
      avoid_state_tick = now;
    // 两边都很近，空间太窄，先后退再选择较空的一侧转
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
    // 左边比右边远，说明左边更空，向左避障
    } else if (left_distance > right_distance) {
      avoid_turn_direction = 1;
      avoid_need_return = 1;
      avoid_turn_target = OBSTACLE_TURN_TARGET;
      MPU6050_ResetYaw();
      Control_TurnLeft(Turn_Speed);
      avoid_state = AVOID_STATE_TURN_LEFT;
      avoid_state_tick = now;
    } else {
      // 右边更空，向右避障
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
    // 左转直到陀螺仪角度达到目标
    if (Control_TurnFinished(1)) {
      if (avoid_need_return) {
        // 普通避障：转过去后先向前走一小段，绕开障碍物
        Control_Forward(Control_Speed);
        avoid_state = AVOID_STATE_FORWARD_AFTER_TURN;
      } else {
        // 路口转弯：不用回正，转完先停一下
        Control_Stop();
        avoid_state = AVOID_STATE_TURN_STOP;
      }
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_RIGHT:
    // 右转直到陀螺仪角度达到目标
    if (Control_TurnFinished(2)) {
      if (avoid_need_return) {
        // 普通避障：转过去后先向前走一小段
        Control_Forward(Control_Speed);
        avoid_state = AVOID_STATE_FORWARD_AFTER_TURN;
      } else {
        // 路口转弯：不用回正
        Control_Stop();
        avoid_state = AVOID_STATE_TURN_STOP;
      }
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_BACKWARD:
    // 后退一段时间后，再按刚才选好的方向转弯
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
    // 绕过障碍后走一小段，再向相反方向回正
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
    // 向左回正完成后停车
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_TURN_STOP;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_RETURN_RIGHT:
    // 向右回正完成后停车
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_TURN_STOP;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_STOP:
    // 转弯或回正后短暂停一下，然后回到正常巡航
    if (now - avoid_state_tick >= 100) {
      Control_Forward(Control_Speed);
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
    }
    break;

  default:
    // 如果状态值异常，直接重置避障状态机
    Control_AvoidanceReset();
    break;
  }
}
