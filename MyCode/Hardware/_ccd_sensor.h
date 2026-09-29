#ifndef CCD_SENSOR_H
#define CCD_SENSOR_H

#include <stdint.h>

#define CCD_SENSOR_PIXEL_COUNT 1500U

// 初始化 CCD 接收状态。CubeMX 必须先完成 GPIO 和 SPI1 初始化。
void CcdSensor_Init(void);

// DR 信号到来后读取像素；没有新帧或 SPI 失败时返回空指针。
// 返回的像素保存在内部缓冲区，下一次读取时会被覆盖。
const uint8_t *CcdSensor_ReadPixels(void);

#endif
