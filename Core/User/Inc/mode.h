#ifndef MODE_H
#define MODE_H

typedef enum {
  // 手动模式：蓝牙直接控制小车和舵机
  mode_manual = 0,
  // 自动模式：执行自动避障流程
  mode_auto=1
}mode_t;

// 初始化模式，默认进入手动模式
void Carmode_Init(void);
// 设置当前模式
void carmode_SetMode(mode_t mode);
// 获取当前模式
mode_t carmode_Getmode(void);


#endif // MODE_H
