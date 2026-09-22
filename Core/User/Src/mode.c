#include "mode.h"

// 保存当前小车模式，默认在初始化时设为手动模式
static mode_t current_mode;

void Carmode_Init(void){
    // 上电默认手动模式，防止小车自动乱跑
    current_mode=mode_manual;
  
}


void carmode_SetMode(mode_t mode) {
    // 蓝牙命令会调用这里切换手动/自动模式
    current_mode = mode;
  
}


mode_t carmode_Getmode(void) {
    // 主循环通过这里判断当前是否进入自动避障模式
    return current_mode;
  
}


