#ifndef CAR_SERIAL_H
#define CAR_SERIAL_H

#include "_ccd_sensor.h"
#include "_drive_control.h"
#include "_line_tracker.h"
#include <stdint.h>

#define CAR_SERIAL_COMMAND_BYTES 48U

// 数据流模式的数值按位组合：1 为电机，2 为 CCD。
typedef enum
{
  CAR_STREAM_OFF = 0,
  CAR_STREAM_MOTOR = 1,
  CAR_STREAM_CCD = 2,
  CAR_STREAM_BOTH = 3
} CarStreamMode;

// 发送一包 MTR2 遥测时临时填写此结构体；它不拥有指针指向的状态。
// drive 直接装入 DriveControl_GetStatus() 的结果，避免再逐项复制轮子数据。
// 应用层提供数值，串口模块负责按协议排列字节并计算 CRC。
typedef struct
{
  uint8_t drive_mode;
  DriveStatus drive;
  const LineTracker *line;
  float max_steer_rpm;
  float steering_rpm;
  float base_rpm;
  float steering_kp;
  float steering_kd;
} CarMotorTelemetry;

// CubeMX 完成 USART1、DMA 初始化后，启动逐字节接收。
void CarSerial_Init(void);

// STOP 在接收中断里获得最高优先级；主循环调用本函数后执行停车。
uint8_t CarSerial_TakeEmergencyStop(void);
uint8_t CarSerial_StopPending(void);

// 取出一条已收齐的命令；输出缓冲区必须容纳 CAR_SERIAL_COMMAND_BYTES 字节。
uint8_t CarSerial_ReadCommand(char command[CAR_SERIAL_COMMAND_BYTES]);

void CarSerial_SetStream(CarStreamMode mode);
uint8_t CarSerial_StreamEnabled(CarStreamMode stream);

// 两种报文共用 USART1 TX DMA；DMA 忙时直接跳过本次上传。
void CarSerial_SendCcd(const CcdFrame *frame);
void CarSerial_SendMotor(const CarMotorTelemetry *telemetry);

#endif
