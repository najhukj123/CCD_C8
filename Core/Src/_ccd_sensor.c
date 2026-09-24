#include "_ccd_sensor.h"
#include "main.h"
#include "spi.h"
#include <string.h>

enum
{
  CCD_PREFIX_BYTES = 10,
  // 模块在原生软件中的“使用长度”为 1500；一帧共 10 + 1500 字节。
  CCD_MODULE_PIXEL_COUNT = 1500,
  CCD_FRAME_BYTES = CCD_PREFIX_BYTES + CCD_MODULE_PIXEL_COUNT,
  CCD_SPI_CHUNK = 64
};

// DR 中断只设置标志，SPI 传输和像素翻转都在主循环完成。
typedef struct
{
  volatile uint8_t ready;
  volatile uint8_t reading;
  uint16_t sequence;
  uint8_t dummy[CCD_SPI_CHUNK];
  uint8_t bytes[CCD_FRAME_BYTES];
  CcdFrame frame;
} CcdSensorState;

static CcdSensorState sensor;

void CcdSensor_Init(void)
{
  uint16_t index;

  memset(&sensor, 0, sizeof(sensor));
  for (index = 0U; index < CCD_SPI_CHUNK; ++index)
    sensor.dummy[index] = 0xFFU;

  // 指针会在每帧读取后指向完整像素区中央的 1000 点。
  sensor.frame.pixels = sensor.bytes + CCD_PREFIX_BYTES +
                        (CCD_MODULE_PIXEL_COUNT - CCD_SENSOR_PIXEL_COUNT) / 2U;

  // 初始化前若 DR 已经拉低，也要读取这帧，避免只等下一次下降沿。
  if (HAL_GPIO_ReadPin(CCD_DR_GPIO_Port, CCD_DR_Pin) == GPIO_PIN_RESET)
    sensor.ready = 1U;
}

const CcdFrame *CcdSensor_ReadFrame(void)
{
  uint16_t received = 0U;
  uint16_t module_pixels = CCD_MODULE_PIXEL_COUNT;
  uint16_t frame_bytes;
  uint16_t index;
  uint8_t *pixels;
  uint8_t success = 1U;

  if (!sensor.ready) return 0;

  sensor.reading = 1U;
  sensor.ready = 0U;
  HAL_GPIO_WritePin(CCD_CS_GPIO_Port, CCD_CS_Pin, GPIO_PIN_RESET);

  // 先取 10 字节帧头。原生软件的串口帧头用 07 21 ... 21 07，
  // 第 2、3 字节是实际像素数。SPI 若不提供同样的头，则按当前
  // 模块配置的 1500 点读取，避免把原先未解析的帧头当作硬性依赖。
  if (HAL_SPI_TransmitReceive(&hspi1, sensor.dummy, sensor.bytes,
                              CCD_PREFIX_BYTES, 100U) != HAL_OK)
    success = 0U;
  else
  {
    received = CCD_PREFIX_BYTES;
    if (sensor.bytes[0] == 0x07U && sensor.bytes[1] == 0x21U &&
        sensor.bytes[8] == 0x21U && sensor.bytes[9] == 0x07U)
    {
      uint16_t reported = (uint16_t)sensor.bytes[2] |
                          ((uint16_t)sensor.bytes[3] << 8);
      if (reported >= CCD_SENSOR_PIXEL_COUNT &&
          reported <= CCD_MODULE_PIXEL_COUNT)
        module_pixels = reported;
    }
  }

  frame_bytes = CCD_PREFIX_BYTES + module_pixels;
  // SPI 是全双工：发送 0xFF 才能同步读取模块输出的数据。
  // 分段读取，避免在栈上准备整帧发送缓冲区。
  while (success && received < frame_bytes)
  {
    uint16_t count = frame_bytes - received;
    if (count > CCD_SPI_CHUNK) count = CCD_SPI_CHUNK;
    if (HAL_SPI_TransmitReceive(&hspi1, sensor.dummy,
                                sensor.bytes + received, count, 100U) != HAL_OK)
    {
      success = 0U;
      break;
    }
    received += count;
  }

  HAL_GPIO_WritePin(CCD_CS_GPIO_Port, CCD_CS_Pin, GPIO_PIN_SET);
  sensor.reading = 0U;
  if (!success) return 0;

  // 取完整视野的中央 1000 点，再按车体左右方向镜像。
  // 原先只取前 1000 点：1500 点的中央线(约第 750 点)会显示在第 249 点。
  pixels = sensor.bytes + CCD_PREFIX_BYTES +
           (module_pixels - CCD_SENSOR_PIXEL_COUNT) / 2U;
  // COM18 实测偶发 1000 个像素全部为 7。这样的整帧不是正常灰度图，
  // 不交给循迹算法，也不上传给上位机；下一次有效帧仍会正常处理。
  if (pixels[0] == 7U)
  {
    for (index = 1U; index < CCD_SENSOR_PIXEL_COUNT; ++index)
      if (pixels[index] != 7U) break;
    if (index == CCD_SENSOR_PIXEL_COUNT) return 0;
  }
  sensor.frame.pixels = pixels;
  for (index = 0U; index < CCD_SENSOR_PIXEL_COUNT / 2U; ++index)
  {
    uint8_t value = pixels[index];
    pixels[index] = pixels[CCD_SENSOR_PIXEL_COUNT - 1U - index];
    pixels[CCD_SENSOR_PIXEL_COUNT - 1U - index] = value;
  }

  sensor.frame.sequence = ++sensor.sequence;
  return &sensor.frame;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == CCD_DR_Pin && !sensor.reading)
    sensor.ready = 1U;
}
