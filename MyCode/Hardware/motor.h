#ifndef MOTOR_H
#define MOTOR_H

// 轮子编号：0 右轮，1 左轮。只负责方向引脚和 PWM，不包含控制算法。
void Motor_Init(void);
void Motor_Write(unsigned wheel_index, float percent);

#endif
