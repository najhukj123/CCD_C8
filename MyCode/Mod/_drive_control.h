#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "_motor_controller.h"

#define DRIVE_MAX_RPM 360.0f

// CubeMX 完成 TIM1、TIM2、TIM3 和 GPIO 初始化后，启用 PWM 与编码器。
void DriveControl_Init(void);

// 立即清除两轮目标和 PID 记忆，并将 PWM 置零。
void DriveControl_Stop(void);

// 设置两轮的请求转速；实际目标会在 Update 时乘补偿与缓启动系数。
void DriveControl_SetRequested(float right_rpm, float left_rpm);

// 每 20 ms 调用一次，读取编码器并更新左右轮速度闭环。
void DriveControl_Update(float ramp);

#endif
