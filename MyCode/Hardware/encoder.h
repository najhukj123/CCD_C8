#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// 轮子编号：0 右轮，1 左轮。返回原始计数，增量与测速由 Mod 计算。
void Encoder_Init(void);
uint16_t Encoder_Read(unsigned wheel_index);

#endif
