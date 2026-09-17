#include "bluetooth.h"
#include "control.h"
#include "mode.h"
#include "servo.h"
#include "usart.h"

#define BT_HEAD 0xA5
#define BT_TAIL 0x5A

#define CMD_FORWARD 0x01
#define CMD_BACKWARD 0x02
#define CMD_LEFT 0x03
#define CMD_RIGHT 0x04
#define CMD_STOP 0x05
#define CMD_MODE_MANUAL 0x06
#define CMD_MODE_AUTO 0x07
#define CMD_SERVO1_FORWARD 0x10
#define CMD_SERVO1_BACKWARD 0x11
#define CMD_SERVO2_FORWARD 0x12
#define CMD_SERVO2_BACKWARD 0x13
#define CMD_SERVO3_FORWARD 0x14
#define CMD_SERVO3_BACKWARD 0x15
#define CMD_SERVO4_FORWARD 0x16
#define CMD_SERVO4_BACKWARD 0x17

#define SPEED_STRAIGHT 0x01
#define SPEED_TURN 0x02

#define CAR_STRAIGHT_SPEED 2800
#define CAR_TURN_SPEED 4500
#define SERVO_RUN_TIME 100

static uint8_t rx_byte;
static uint8_t rx_state;
static uint8_t rx_cmd;
static uint8_t rx_speed_cmd;
static volatile uint8_t new_cmd;

static uint16_t Bluetooth_SpeedValue(uint8_t speed_cmd) {
  if (speed_cmd == SPEED_TURN) {
    return CAR_TURN_SPEED;
  }

  if (speed_cmd == SPEED_STRAIGHT) {
    return CAR_STRAIGHT_SPEED;
  }

  return 0;
}

//状态机
static void Bluetooth_ParseByte(uint8_t byte) {
  switch (rx_state) {
  case 0:
    if (byte == BT_HEAD) {
      rx_state = 1;
    }
    break;

  case 1:
    rx_cmd = byte;
    rx_state = 2;
    break;

  case 2:
    rx_speed_cmd = byte;
    rx_state = 3;
    break;

  case 3:
    if (byte == (uint8_t)(rx_cmd + rx_speed_cmd)) {
      rx_state = 4;
    } else {
      rx_state = 0;
    }
    break;

  case 4:
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
  rx_byte = 0;
  rx_state = 0;
  rx_cmd = 0;
  rx_speed_cmd = 0;
  new_cmd = 0;

  HAL_UART_Receive_DMA(&huart3, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) {
    return;
  }

  Bluetooth_ParseByte(rx_byte);
  HAL_UART_Receive_DMA(&huart3, &rx_byte, 1);
}
//错误数据处理
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart != &huart3) {
    return;
  }

  rx_state = 0;
  new_cmd = 0;
  HAL_UART_Receive_DMA(&huart3, &rx_byte, 1);
}

void Bluetooth_Task(void) {
  uint8_t cmd;
  uint8_t speed_cmd;
  uint16_t speed;

  if (new_cmd == 0) {
    return;
  }
//暂时退出中断
  __disable_irq();
  cmd = rx_cmd;
  speed_cmd = rx_speed_cmd;
  new_cmd = 0;
  __enable_irq();

  switch (cmd) {
    case CMD_MODE_MANUAL:
      carmode_SetMode(mode_manual);
      Control_Stop();
      Control_AvoidanceReset();
      break;
    case CMD_MODE_AUTO:
      carmode_SetMode(mode_auto);
      Control_Stop();
      Control_AvoidanceReset();
      break;
    default:
      if (carmode_Getmode() == mode_auto) {
        // 如果是自动模式，忽略手动控制命令
        return;
      }
  }

  speed = Bluetooth_SpeedValue(speed_cmd);

  switch (cmd) {
  case CMD_FORWARD:
    Control_Forward(speed);
    break;

  case CMD_BACKWARD:
    Control_Backward(speed);
    break;

  case CMD_LEFT:
    Control_TurnLeft(speed);
    break;

  case CMD_RIGHT:
    Control_TurnRight(speed);
    break;

  case CMD_STOP:
    Control_Stop();
    break;

  case CMD_SERVO1_FORWARD:
    Servo_TurnForward(1, SERVO_RUN_TIME);
    break;

  case CMD_SERVO1_BACKWARD:
    Servo_TurnBackward(1, SERVO_RUN_TIME);
    break;

  case CMD_SERVO2_FORWARD:
    Servo_TurnForward(2, SERVO_RUN_TIME);
    Servo_TurnForward(3, SERVO_RUN_TIME);
    break;

  case CMD_SERVO2_BACKWARD:
    Servo_TurnBackward(2, SERVO_RUN_TIME);
    Servo_TurnBackward(3, SERVO_RUN_TIME);
    break;



  case CMD_SERVO3_FORWARD:
    Servo_TurnForward(2, SERVO_RUN_TIME);
    Servo_TurnForward(3, SERVO_RUN_TIME);
    break;

  case CMD_SERVO3_BACKWARD:
    Servo_TurnBackward(2, SERVO_RUN_TIME);
    Servo_TurnBackward(3, SERVO_RUN_TIME);
    break;

  case CMD_SERVO4_FORWARD:
    Servo_TurnForward(4, SERVO_RUN_TIME);
    break;

  case CMD_SERVO4_BACKWARD:
    Servo_TurnBackward(4, SERVO_RUN_TIME);
    break;

  default:
    break;
  }
}
