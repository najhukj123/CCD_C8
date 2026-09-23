#ifndef CAR_APP_H
#define CAR_APP_H

// 小车应用负责行驶决策和周期调度；main.c 只负责 CubeMX 初始化与主循环。
// GPIO、SPI、定时器和串口硬件配置均来自当前 CubeMX 工程。
void CarApp_Init(void);
void CarApp_Poll(void);

#endif
