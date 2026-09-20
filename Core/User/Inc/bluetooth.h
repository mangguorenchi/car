#ifndef BLUETOOTH_H
#define BLUETOOTH_H

// 初始化蓝牙串口DMA接收
void Bluetooth_Init(void);
// 处理收到的蓝牙命令，需要在while循环中反复调用
void Bluetooth_Task(void);

#endif /* BLUETOOTH_H */
