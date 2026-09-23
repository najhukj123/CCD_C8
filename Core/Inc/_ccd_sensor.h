#ifndef CCD_SENSOR_H
#define CCD_SENSOR_H

#include <stdint.h>

#define CCD_SENSOR_PIXEL_COUNT 1000U

// 一帧的像素指向模块内部缓冲区；下一次读取开始前有效，不能长期保存。
typedef struct
{
  uint16_t sequence;
  const uint8_t *pixels;
} CcdFrame;

// 初始化 CCD 接收状态。CubeMX 必须先完成 GPIO 和 SPI1 初始化。
void CcdSensor_Init(void);

// DR 信号到来后读取一帧；没有新帧或 SPI 失败时返回空指针。
const CcdFrame *CcdSensor_ReadFrame(void);

#endif
