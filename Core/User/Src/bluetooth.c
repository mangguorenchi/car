#include "bluetooth.h"
#include "control.h"
#include "mode.h"
#include "servo.h"
#include "usart.h"

// 蓝牙数据包包头和包尾，用来判断一包数据的开始和结束
#define BT_HEAD 0xA5
#define BT_TAIL 0x5A

// 小车运动命令
#define CMD_FORWARD 0x01
#define CMD_BACKWARD 0x02
#define CMD_LEFT 0x03
#define CMD_RIGHT 0x04
#define CMD_STOP 0x05
// 模式切换命令
#define CMD_MODE_MANUAL 0x06
#define CMD_MODE_AUTO 0x07
// 舵机命令：每个舵机分正转和反转
#define CMD_SERVO1_FORWARD 0x10
#define CMD_SERVO1_BACKWARD 0x11
#define CMD_SERVO2_FORWARD 0x12
#define CMD_SERVO2_BACKWARD 0x13
#define CMD_SERVO3_FORWARD 0x14
#define CMD_SERVO3_BACKWARD 0x15
#define CMD_SERVO4_FORWARD 0x16
#define CMD_SERVO4_BACKWARD 0x17

// 速度命令：直线和转弯用不同速度
#define SPEED_STRAIGHT 0x01
#define SPEED_TURN 0x02

// 实际PWM速度值，可以在这里调
#define CAR_STRAIGHT_SPEED 2800
#define CAR_TURN_SPEED 4500
// 舵机每次收到命令后转动的时间，时间到后自动停止
#define SERVO_RUN_TIME 100

// DMA每次只接收1个字节，收到后进入回调继续解析
static uint8_t rx_byte;
// 接收状态：0等包头，1等命令，2等速度，3等校验，4等包尾
static uint8_t rx_state;
// 暂存收到的命令字节
static uint8_t rx_cmd;
// 暂存收到的速度字节
static uint8_t rx_speed_cmd;
// 收到一包完整且校验正确的数据后置1
static volatile uint8_t new_cmd;

static uint16_t Bluetooth_SpeedValue(uint8_t speed_cmd) {
  // 转弯命令使用转弯速度
  if (speed_cmd == SPEED_TURN) {
    return CAR_TURN_SPEED;
  }

  // 前进后退使用直线速度
  if (speed_cmd == SPEED_STRAIGHT) {
    return CAR_STRAIGHT_SPEED;
  }

  // 未知速度命令给0，防止误动作
  return 0;
}

// 蓝牙接收状态机，每次只处理1个字节
// 数据包格式：A5 + 命令 + 速度 + 校验和 + 5A
static void Bluetooth_ParseByte(uint8_t byte) {
  switch (rx_state) {
  case 0:
    // 状态0：等待包头
    if (byte == BT_HEAD) {
      rx_state = 1;
    }
    break;

  case 1:
    // 状态1：保存命令字节
    rx_cmd = byte;
    rx_state = 2;
    break;

  case 2:
    // 状态2：保存速度字节
    rx_speed_cmd = byte;
    rx_state = 3;
    break;

  case 3:
    // 状态3：检查校验和，校验=命令+速度，超出255自动截断
    if (byte == (uint8_t)(rx_cmd + rx_speed_cmd)) {
      rx_state = 4;
    } else {
      // 校验失败，丢弃这一包，从头重新等包头
      rx_state = 0;
    }
    break;

  case 4:
    // 状态4：检查包尾，正确才认为收到了一包完整命令
    rx_state = 0;
    if (byte == BT_TAIL) {
      new_cmd = 1;
    }
    break;

  default:
    rx_state = 0;
    break;
  }
}

void Bluetooth_Init(void) {
  // 清空接收变量
  rx_byte = 0;
  rx_state = 0;
  rx_cmd = 0;
  rx_speed_cmd = 0;
  new_cmd = 0;

  // 开启USART3 DMA接收，每次接收1个字节
  HAL_UART_Receive_DMA(&huart3, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // 只处理蓝牙使用的USART3，避免影响其他串口
  if (huart != &huart3) {
    return;
  }

  // DMA收到1个字节后，交给状态机解析
  Bluetooth_ParseByte(rx_byte);
  // 重新开启下一次1字节DMA接收
  HAL_UART_Receive_DMA(&huart3, &rx_byte, 1);
}

// 串口出错时进入这里，比如噪声、帧错误、溢出等
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) {
    return;
  }

  // 出错后丢弃当前半包数据，重新开始接收
  rx_state = 0;
  new_cmd = 0;
  HAL_UART_Receive_DMA(&huart3, &rx_byte, 1);
}

void Bluetooth_Task(void) {
  uint8_t cmd;
  uint8_t speed_cmd;
  uint16_t speed;

  // 没有新命令就直接返回，不影响主循环其他任务
  if (new_cmd == 0) {
    return;
  }

  // 复制命令时短暂关中断，防止中断刚好改写rx_cmd
  __disable_irq();
  cmd = rx_cmd;
  speed_cmd = rx_speed_cmd;
  new_cmd = 0;
  __enable_irq();

  switch (cmd) {
    case CMD_MODE_MANUAL:
      // 切换到蓝牙手动模式
      carmode_SetMode(mode_manual);
      Control_Stop();
      Control_AvoidanceReset();
      break;
    case CMD_MODE_AUTO:
      // 切换到自动避障模式
      carmode_SetMode(mode_auto);
      Control_Stop();
      Control_AvoidanceReset();
      break;
    default:
      if (carmode_Getmode() == mode_auto) {
        // 自动模式下忽略普通遥控运动和舵机命令
        return;
      }
  }

  // 把速度命令转换成真正的PWM速度值
  speed = Bluetooth_SpeedValue(speed_cmd);

  switch (cmd) {
  case CMD_FORWARD:
    // 小车前进
    Control_Forward(speed);
    break;

  case CMD_BACKWARD:
    // 小车后退
    Control_Backward(speed);
    break;

  case CMD_LEFT:
    // 小车左转
    Control_TurnLeft(speed);
    break;

  case CMD_RIGHT:
    // 小车右转
    Control_TurnRight(speed);
    break;

  case CMD_STOP:
    // 小车停止
    Control_Stop();
    break;

  case CMD_SERVO1_FORWARD:
    // 舵机1正方向转动一小段时间
    Servo_TurnForward(1, SERVO_RUN_TIME);
    break;

  case CMD_SERVO1_BACKWARD:
    // 舵机1反方向转动一小段时间
    Servo_TurnBackward(1, SERVO_RUN_TIME);
    break;

  case CMD_SERVO2_FORWARD:
    // 舵机2和舵机3控制同一个结构，所以一起正转
    Servo_TurnForward(2, SERVO_RUN_TIME);
    Servo_TurnForward(3, SERVO_RUN_TIME);
    break;

  case CMD_SERVO2_BACKWARD:
    // 舵机2和舵机3一起反转
    Servo_TurnBackward(2, SERVO_RUN_TIME);
    Servo_TurnBackward(3, SERVO_RUN_TIME);
    break;

  case CMD_SERVO3_FORWARD:
    // 舵机3命令也让2和3一起动，方便手机端按钮单独映射
    Servo_TurnForward(2, SERVO_RUN_TIME);
    Servo_TurnForward(3, SERVO_RUN_TIME);
    break;

  case CMD_SERVO3_BACKWARD:
    // 舵机3反转命令，同样让2和3一起动
    Servo_TurnBackward(2, SERVO_RUN_TIME);
    Servo_TurnBackward(3, SERVO_RUN_TIME);
    break;

  case CMD_SERVO4_FORWARD:
    // 舵机4正方向转动一小段时间
    Servo_TurnForward(4, SERVO_RUN_TIME);
    break;

  case CMD_SERVO4_BACKWARD:
    // 舵机4反方向转动一小段时间
    Servo_TurnBackward(4, SERVO_RUN_TIME);
    break;

  default:
    break;
  }
}
