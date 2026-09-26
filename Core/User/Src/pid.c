#include "pid.h"
#include <stdio.h>

#define ENCODER_COUNT_PER_REVOLUTION 390 // 编码器每转的计数值
#define ENCODER_SAMPLE_time_ms 100       // 编码器采样时间，单位毫秒

static volatile int32_t encoder_count[4];
static int32_t encoder_last_count[4];
static int16_t encoder_speed[4]; // 每100ms多少脉冲
static uint32_t encoder_last_time;

void Encoder_ClearCount(Encoder_ID_t encoder) {
  if (encoder >= encoder1 && encoder <= encoder4) {

    encoder_count[encoder - 1] = 0;
  }
}

void Encoder_ClearAllCount(void) {
  for (uint8_t i = 0; i < 4; i++) {
    encoder_count[i] = 0;
  }
}

void Encoder_Init(void) {
  Encoder_ClearAllCount();
  for (uint8_t i = 0; i < 4; i++) {
    encoder_last_count[i] = 0;
    encoder_speed[i] = 0;
  }
  encoder_last_time = HAL_GetTick();
}

int Encoder_GetCount(Encoder_ID_t encoder) {
  if (encoder >= encoder1 && encoder <= encoder4) {
    return encoder_count[encoder - 1];
  }
  return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  GPIO_PinState b_state;
  if (GPIO_Pin == ENC1_A_Pin) {
    b_state = HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin);
    if (b_state == GPIO_PIN_SET) {
      encoder_count[encoder1 - 1]++;
    } else {
      encoder_count[encoder1 - 1]--;
    }
  } else if (GPIO_Pin == ENC2_A_Pin) {
    b_state = HAL_GPIO_ReadPin(ENC2_B_GPIO_Port, ENC2_B_Pin);
    if (b_state == GPIO_PIN_SET) {
      encoder_count[encoder2 - 1]++;
    } else {
      encoder_count[encoder2 - 1]--;
    }
  } else if (GPIO_Pin == ENC3_A_Pin) {
    b_state = HAL_GPIO_ReadPin(ENC3_B_GPIO_Port, ENC3_B_Pin);
    if (b_state == GPIO_PIN_SET) {
      encoder_count[encoder3 - 1]++;
    } else {
      encoder_count[encoder3 - 1]--;
    }
  } else if (GPIO_Pin == ENC4_A_Pin) {
    b_state = HAL_GPIO_ReadPin(ENC4_B_GPIO_Port, ENC4_B_Pin);
    if (b_state == GPIO_PIN_SET) {
      encoder_count[encoder4 - 1]++;
    } else {
      encoder_count[encoder4 - 1]--;
    }
  }
}

void Encoder_Task(void) {
  uint32_t now = HAL_GetTick();
  if (now - encoder_last_time >= ENCODER_SAMPLE_time_ms) {
    encoder_last_time = now;

    for (uint8_t i = 0; i < 4; i++) {
      int32_t now_count = encoder_count[i];
      encoder_speed[i] = now_count - encoder_last_count[i];
      encoder_last_count[i] = now_count;
    }

    printf("ENC count: %ld, %ld, %ld, %ld | speed: %d, %d, %d, %d\r\n",
           (long)encoder_count[0], (long)encoder_count[1],
           (long)encoder_count[2], (long)encoder_count[3],
           encoder_speed[0], encoder_speed[1],
           encoder_speed[2], encoder_speed[3]);
  }
}

int16_t Encoder_GetSpeed(Encoder_ID_t encoder) {
  if (encoder >= encoder1 && encoder <= encoder4) {

    return encoder_speed[encoder - 1];
  }
  return 0;
}
