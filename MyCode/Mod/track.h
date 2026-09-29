#ifndef TRACK_H
#define TRACK_H

// 初始化后等待有效 CCD 窄线，再以代码中的固定参数自动循迹。
void Track_Init(void);
// 主循环调用：20 ms 电机控制、CCD 采集、循迹。
void Track_Poll(void);

#endif
