#include "control.h"
#include "motor.h"
#include "mpu6050.h"
#include "sensor.h"
#include <stdio.h>

// 小车正常前进速度
#define CONTROL_SPEED 3600

// 普通避障转弯速度
#define TURN_SPEED 3000

// 左右摇摆扫描时使用较慢的速度，避免测距时转得太快
#define SCAN_TURN_SPEED 1800

// 前方距离小于这个值时，认为前方有障碍，单位mm
#define FRONT_OBSTACLE_DISTANCE 425.0f

// 前方距离恢复到这个值以上后，清除连续障碍计数
#define FRONT_CLEAR_DISTANCE 450.0f

// 连续检测到两次前方障碍后才开始避障，减少误触发
#define FRONT_CONFIRM_COUNT 2

// 左右扫描各转30度
#define SCAN_TURN_TARGET 30.0f

// 从左侧30度转到右侧30度，一共转60度
#define SCAN_TOTAL_TURN_TARGET 60.0f

// 普通避障回到直线方向时转回30度
#define RETURN_TURN_TARGET 30.0f

// 墙面转弯角度
#define WALL_TURN_TARGET 90.0f

// 超声波读数小于这个值时，认为可能正对墙面
#define WALL_DISTANCE 180.0f

// 连续小距离检测时间超过这个值，触发防卡死流程
#define STUCK_DISTANCE 120.0f
#define STUCK_CONFIRM_TIME 1000U

// 自动避障时两次动作之间的最大允许时间
#define AVOID_MAX_TURN_TIME 2500U

// 避障转弯后直行一小段的时间
#define AVOID_FORWARD_TIME 700U

// 每次测距前后停车稳定的时间
#define AVOID_STOP_TIME 1000U

// 回到车身正方向后的短暂稳定时间
#define CENTER_SETTLE_TIME 300U

// 避障时先后退的时间
#define RECOVERY_BACKWARD_TIME 700U

// 防卡死或遇墙时后退的速度
#define RECOVERY_BACKWARD_SPEED 2000

// 两个方向距离差超过这个值，才认为一边明显更空
#define SCAN_DIFF_DISTANCE 100.0f

typedef enum {
  // 正常前进，持续检测前方
  AVOID_STATE_RUN = 0,

  // 检测到障碍后停车，准备开始扫描
  AVOID_STATE_STOP_WAIT,

  // 向左转30度
  AVOID_STATE_SCAN_LEFT,

  // 左转结束后停车1秒，再读取左侧距离
  AVOID_STATE_SCAN_LEFT_WAIT,

  // 从左侧30度转到右侧30度
  AVOID_STATE_SCAN_RIGHT,

  // 右转结束后停车1秒，再读取右侧距离
  AVOID_STATE_SCAN_RIGHT_WAIT,

  // 右侧读取完成后回到车身正方向
  AVOID_STATE_RETURN_CENTER,

  // 回正后短暂停车，再决定往哪边避障
  AVOID_STATE_RETURN_CENTER_WAIT,

  // 根据左右距离决定方向
  AVOID_STATE_CHOOSE_DIRECTION,

  // 向左侧空旷方向转30度
  AVOID_STATE_TURN_AVOID_LEFT,

  // 向右侧空旷方向转30度
  AVOID_STATE_TURN_AVOID_RIGHT,

  // 避开障碍后向前行驶
  AVOID_STATE_FORWARD,

  // 向左回正
  AVOID_STATE_RETURN_LEFT,

  // 向右回正
  AVOID_STATE_RETURN_RIGHT,

  // 防卡死或遇墙时后退
  AVOID_STATE_RECOVERY_BACKWARD,

  // 遇墙后向右转90度
  AVOID_STATE_WALL_TURN_RIGHT
} Avoid_State_t;

// 当前自动避障状态
static Avoid_State_t avoid_state;

// 进入当前状态的时间
static uint32_t avoid_state_tick;

// 当前转弯目标角度
static float avoid_turn_target;

// 左转30度后测得的距离
static float scan_left_distance;

// 右转30度后测得的距离
static float scan_right_distance;

// 这次避障选择的方向：1表示左，2表示右
static uint8_t avoid_direction;

// 前方障碍连续确认次数
static uint8_t front_obstacle_count;

// 是否在扫描过程中发现了墙
static uint8_t wall_detected;

// 记录开始检测到很近距离的时间
static uint32_t near_start_tick;

// 读取陀螺仪角度，判断目标转弯是否完成
static uint8_t Control_TurnFinished(uint8_t direction) {
  float yaw = MPU6050_GetYaw();

  // 左转时Yaw应该变成负数
  if (direction == 1 && yaw <= -avoid_turn_target) {
    return 1;
  }

  // 右转时Yaw应该变成正数
  if (direction == 2 && yaw >= avoid_turn_target) {
    return 1;
  }

  // 陀螺仪异常或电机不转时，超时退出，防止状态机卡死
  if (HAL_GetTick() - avoid_state_tick >= AVOID_MAX_TURN_TIME) {
    printf("Gyro turn timeout, yaw=%.2f\r\n", yaw);
    return 1;
  }

  return 0;
}

// 开始向左转30度进行扫描
static void Control_StartScanLeft(uint32_t now) {
  MPU6050_ResetYaw();
  avoid_turn_target = SCAN_TURN_TARGET;
  Control_TurnLeft(SCAN_TURN_SPEED);
  avoid_state = AVOID_STATE_SCAN_LEFT;
  avoid_state_tick = now;
}

// 扫描完成后回到车身正方向
static void Control_StartReturnCenter(uint32_t now) {
  MPU6050_ResetYaw();
  avoid_turn_target = SCAN_TURN_TARGET;
  Control_TurnLeft(SCAN_TURN_SPEED);
  avoid_state = AVOID_STATE_RETURN_CENTER;
  avoid_state_tick = now;
}

// 开始遇墙后的后退和右转流程
static void Control_StartWallRecovery(uint32_t now) {
  Control_Stop();
  Control_Backward(RECOVERY_BACKWARD_SPEED);
  avoid_state = AVOID_STATE_RECOVERY_BACKWARD;
  avoid_state_tick = now;
}

void Control_Init(void) {
  // 初始化电机和超声波
  Motor_Init();
  Sensor_Init();
}

void Control_Forward(uint16_t speed) {
  // 四个电机同时前进
  Motor_SetSpeed(MOTOR_M1, speed);
  Motor_SetSpeed(MOTOR_M2, speed);
  Motor_SetSpeed(MOTOR_M3, speed);
  Motor_SetSpeed(MOTOR_M4, speed);
}

void Control_Backward(uint16_t speed) {
  // 四个电机同时后退
  Motor_SetSpeed(MOTOR_M1, -speed);
  Motor_SetSpeed(MOTOR_M2, -speed);
  Motor_SetSpeed(MOTOR_M3, -speed);
  Motor_SetSpeed(MOTOR_M4, -speed);
}

void Control_TurnLeft(uint16_t speed) {
  // 根据当前电机方向定义，执行原地左转
  Motor_SetSpeed(MOTOR_M1, speed);
  Motor_SetSpeed(MOTOR_M2, -speed);
  Motor_SetSpeed(MOTOR_M3, speed);
  Motor_SetSpeed(MOTOR_M4, -speed);
}

void Control_TurnRight(uint16_t speed) {
  // 根据当前电机方向定义，执行原地右转
  Motor_SetSpeed(MOTOR_M1, -speed);
  Motor_SetSpeed(MOTOR_M2, speed);
  Motor_SetSpeed(MOTOR_M3, -speed);
  Motor_SetSpeed(MOTOR_M4, speed);
}

void Control_Stop(void) {
  // 停止全部电机
  Motor_StopAll();
}

void Control_AvoidanceReset(void) {
  // 自动模式开始或重新开始时，清空所有避障数据
  avoid_state = AVOID_STATE_RUN;
  avoid_state_tick = HAL_GetTick();
  avoid_turn_target = SCAN_TURN_TARGET;
  scan_left_distance = -1.0f;
  scan_right_distance = -1.0f;
  avoid_direction = 0;
  front_obstacle_count = 0;
  wall_detected = 0;
  near_start_tick = 0;
}

// 自动避障任务，需要在main的while循环中反复调用
// 所有等待都使用HAL_GetTick，避免长时间HAL_Delay阻塞主循环
void Control_AvoidanceTask(void) {
  uint32_t now = HAL_GetTick();
  float front_distance;

  switch (avoid_state) {
  case AVOID_STATE_RUN:
    // 正常前进时读取前方距离
    front_distance = HCSR04_FrontRead();

    if (front_distance < 0) {
      // 超声波异常时立即停车，保留当前状态等待下一次重试
      Control_Stop();
      front_obstacle_count = 0;
      near_start_tick = 0;
      printf("Ultrasonic ERROR\r\n");
      return;
    }

    // 距离非常小时，先停车确认，防止小车继续顶墙
    if (front_distance <= STUCK_DISTANCE) {
      if (near_start_tick == 0) {
        near_start_tick = now;
      }

      Control_Stop();

      // 小距离持续一段时间，进入防卡死后退流程
      if (now - near_start_tick >= STUCK_CONFIRM_TIME) {
        front_obstacle_count = 0;
        wall_detected = 0;
        scan_left_distance = -1.0f;
        scan_right_distance = -1.0f;
        Control_StartWallRecovery(now);
      }
      break;
    }

    // 距离恢复后，清除小距离计时
    near_start_tick = 0;

    // 普通障碍需要连续检测到两次才触发，减少超声波误报
    if (front_distance <= FRONT_OBSTACLE_DISTANCE) {
      // 一检测到障碍就停车，确认期间不再继续向前顶
      Control_Stop();

      if (front_obstacle_count < FRONT_CONFIRM_COUNT) {
        front_obstacle_count++;
      }

      if (front_obstacle_count >= FRONT_CONFIRM_COUNT) {
        Control_Stop();
        avoid_state = AVOID_STATE_STOP_WAIT;
        avoid_state_tick = now;
        front_obstacle_count = 0;
      }
    } else {
      if (front_distance >= FRONT_CLEAR_DISTANCE) {
        front_obstacle_count = 0;
      }
      Control_Forward(CONTROL_SPEED);
    }
    break;

  case AVOID_STATE_STOP_WAIT:
    // 遇到普通障碍后先停1秒，再开始左侧扫描
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      wall_detected = 0;
      scan_left_distance = -1.0f;
      scan_right_distance = -1.0f;
      Control_StartScanLeft(now);
    }
    break;

  case AVOID_STATE_SCAN_LEFT:
    // 用较慢速度左转30度
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_SCAN_LEFT_WAIT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_SCAN_LEFT_WAIT:
    // 停住1秒后再读取左侧距离
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      scan_left_distance = HCSR04_FrontRead();

      if (scan_left_distance < 0) {
        Control_Stop();
        printf("Ultrasonic ERROR\r\n");
        avoid_state_tick = now;
        break;
      }

      printf("Scan left 30 deg: %.1f mm\r\n", scan_left_distance);

      // 左侧读数很小，认为这一侧可能是墙
      if (scan_left_distance <= WALL_DISTANCE) {
        wall_detected = 1;
      }

      // 从左侧30度转到右侧30度
      MPU6050_ResetYaw();
      avoid_turn_target = SCAN_TOTAL_TURN_TARGET;
      Control_TurnRight(SCAN_TURN_SPEED);
      avoid_state = AVOID_STATE_SCAN_RIGHT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_SCAN_RIGHT:
    // 用较慢速度右转60度
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_SCAN_RIGHT_WAIT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_SCAN_RIGHT_WAIT:
    // 停住1秒后再读取右侧距离
    if (now - avoid_state_tick >= AVOID_STOP_TIME) {
      scan_right_distance = HCSR04_FrontRead();

      if (scan_right_distance < 0) {
        Control_Stop();
        printf("Ultrasonic ERROR\r\n");
        avoid_state_tick = now;
        break;
      }

      printf("Scan right 30 deg: %.1f mm\r\n", scan_right_distance);

      // 右侧读数很小，也认为可能遇到墙
      if (scan_right_distance <= WALL_DISTANCE) {
        wall_detected = 1;
      }

      // 当前车头在右侧30度，先回到正前方
      Control_StartReturnCenter(now);
    }
    break;

  case AVOID_STATE_RETURN_CENTER:
    // 从右侧30度回到中间方向
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_RETURN_CENTER_WAIT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_RETURN_CENTER_WAIT:
    // 回正后短暂停车，让车身和测距结果稳定
    if (now - avoid_state_tick >= CENTER_SETTLE_TIME) {
      avoid_state = AVOID_STATE_CHOOSE_DIRECTION;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_CHOOSE_DIRECTION:
    // 扫描时发现墙，执行：回正 -> 后退 -> 右转90度
    if (wall_detected) {
      Control_StartWallRecovery(now);
      break;
    }

    // 左边明显更空，选择左边
    if (scan_left_distance > scan_right_distance + SCAN_DIFF_DISTANCE) {
      avoid_direction = 1;
      MPU6050_ResetYaw();
      avoid_turn_target = RETURN_TURN_TARGET;
      Control_TurnLeft(SCAN_TURN_SPEED);
      avoid_state = AVOID_STATE_TURN_AVOID_LEFT;
      avoid_state_tick = now;
    } else {
      // 右边更空，或者两边差距不大时默认选择右边
      avoid_direction = 2;
      MPU6050_ResetYaw();
      avoid_turn_target = RETURN_TURN_TARGET;
      Control_TurnRight(SCAN_TURN_SPEED);
      avoid_state = AVOID_STATE_TURN_AVOID_RIGHT;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_AVOID_LEFT:
    // 向左侧空旷方向转30度
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_FORWARD;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_TURN_AVOID_RIGHT:
    // 向右侧空旷方向转30度
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_FORWARD;
      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_FORWARD:
    // 沿着选择的方向前进一小段，绕过障碍
    Control_Forward(CONTROL_SPEED);

    if (now - avoid_state_tick >= AVOID_FORWARD_TIME) {
      Control_Stop();
      MPU6050_ResetYaw();
      avoid_turn_target = RETURN_TURN_TARGET;

      // 左绕后向右回30度
      if (avoid_direction == 1) {
        Control_TurnRight(SCAN_TURN_SPEED);
        avoid_state = AVOID_STATE_RETURN_RIGHT;
      } else {
        // 右绕后向左回30度
        Control_TurnLeft(SCAN_TURN_SPEED);
        avoid_state = AVOID_STATE_RETURN_LEFT;
      }

      avoid_state_tick = now;
    }
    break;

  case AVOID_STATE_RETURN_LEFT:
    // 右绕后向左回正
    if (Control_TurnFinished(1)) {
      Control_Stop();
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
      front_obstacle_count = 0;
      near_start_tick = 0;
    }
    break;

  case AVOID_STATE_RETURN_RIGHT:
    // 左绕后向右回正
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
      front_obstacle_count = 0;
      near_start_tick = 0;
    }
    break;

  case AVOID_STATE_RECOVERY_BACKWARD:
    // 防卡死或遇墙时先后退
    if (now - avoid_state_tick >= RECOVERY_BACKWARD_TIME) {
      Control_Stop();

      // 如果是扫描发现墙，后退后右转90度
      if (wall_detected) {
        MPU6050_ResetYaw();
        avoid_turn_target = WALL_TURN_TARGET;
        Control_TurnRight(TURN_SPEED);
        avoid_state = AVOID_STATE_WALL_TURN_RIGHT;
        avoid_state_tick = now;
      } else {
        // 防卡死流程后重新左右扫描，继续普通避障
        scan_left_distance = -1.0f;
        scan_right_distance = -1.0f;
        Control_StartScanLeft(now);
      }
    }
    break;

  case AVOID_STATE_WALL_TURN_RIGHT:
    // 遇墙后右转90度，完成后继续前进
    if (Control_TurnFinished(2)) {
      Control_Stop();
      avoid_state = AVOID_STATE_RUN;
      avoid_state_tick = now;
      front_obstacle_count = 0;
      wall_detected = 0;
      near_start_tick = 0;
    }
    break;

  default:
    // 状态异常时停车并重新初始化避障流程
    Control_Stop();
    Control_AvoidanceReset();
    break;
  }
}
