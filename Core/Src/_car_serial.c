#include "_car_serial.h"
#include "usart.h"
#include <string.h>

enum
{
  COMMAND_QUEUE_SIZE = 8,
  CCD_HEADER_BYTES = 8,
  CCD_PACKET_BYTES = CCD_HEADER_BYTES + CCD_SENSOR_PIXEL_COUNT + 2,
  MOTOR_PACKET_BYTES = 90
};

// 接收缓冲区由 USART1 中断写入，由主循环读取；发送缓冲区由 DMA 使用。
typedef struct
{
  CarStreamMode stream;
  volatile uint8_t tx_busy;
  uint8_t rx_byte;
  char receiving[CAR_SERIAL_COMMAND_BYTES];
  uint8_t rx_length;
  uint8_t discard_until_newline;
  char commands[COMMAND_QUEUE_SIZE][CAR_SERIAL_COMMAND_BYTES];
  volatile uint8_t queue_head;
  volatile uint8_t queue_tail;
  volatile uint8_t queue_count;
  volatile uint8_t stop_requested;
  uint16_t motor_sequence;
  uint8_t ccd_packet[CCD_PACKET_BYTES];
  uint8_t motor_packet[MOTOR_PACKET_BYTES];
} CarSerialState;

static CarSerialState serial_state;

// 串口协议固定为小端序；STM32F103 上的 float 是 4 字节 IEEE 754。
static void PutU16(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8);
}

static void PutFloat(uint8_t *destination, float value)
{
  union { float number; uint8_t bytes[4]; } encoded;
  encoded.number = value;
  memcpy(destination, encoded.bytes, 4U);
}

// CRC-16/CCITT-FALSE：初值 FFFF，多项式 1021，不反射。
static uint16_t Crc16(const uint8_t *bytes, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;
  uint8_t bit;

  for (index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)bytes[index] << 8;
    for (bit = 0U; bit < 8U; ++bit)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                            : (uint16_t)(crc << 1);
  }
  return crc;
}

void CarSerial_Init(void)
{
  memset(&serial_state, 0, sizeof(serial_state));
  serial_state.stream = CAR_STREAM_MOTOR;
  HAL_UART_Receive_IT(&huart1, &serial_state.rx_byte, 1U);
}

uint8_t CarSerial_TakeEmergencyStop(void)
{
  uint8_t requested;

  // 队列和急停标志也会在 USART1 中断中修改，取出时保持原子性。
  __disable_irq();
  requested = serial_state.stop_requested;
  if (requested)
  {
    serial_state.stop_requested = 0U;
    serial_state.queue_count = 0U;
    serial_state.queue_head = serial_state.queue_tail;
  }
  __enable_irq();
  return requested;
}

uint8_t CarSerial_StopPending(void)
{
  return serial_state.stop_requested;
}

uint8_t CarSerial_ReadCommand(char command[CAR_SERIAL_COMMAND_BYTES])
{
  uint8_t found = 0U;

  if (!serial_state.queue_count) return 0U;

  __disable_irq();
  if (serial_state.queue_count)
  {
    memcpy(command, serial_state.commands[serial_state.queue_head],
           CAR_SERIAL_COMMAND_BYTES);
    serial_state.queue_head =
        (uint8_t)((serial_state.queue_head + 1U) % COMMAND_QUEUE_SIZE);
    --serial_state.queue_count;
    found = 1U;
  }
  __enable_irq();
  return found;
}

void CarSerial_SetStream(CarStreamMode mode)
{
  serial_state.stream = mode;
}

uint8_t CarSerial_StreamEnabled(CarStreamMode stream)
{
  return ((uint8_t)serial_state.stream & (uint8_t)stream) != 0U;
}

void CarSerial_SendCcd(const CcdFrame *frame)
{
  uint8_t *packet = serial_state.ccd_packet;

  if (serial_state.tx_busy) return;

  // CCD1：标识 4 字节、像素数 2 字节、序号 2 字节、像素 1000 字节、CRC 2 字节。
  memcpy(packet, "CCD1", 4U);
  PutU16(packet + 4, CCD_SENSOR_PIXEL_COUNT);
  PutU16(packet + 6, frame->sequence);
  memcpy(packet + CCD_HEADER_BYTES, frame->pixels, CCD_SENSOR_PIXEL_COUNT);
  PutU16(packet + CCD_HEADER_BYTES + CCD_SENSOR_PIXEL_COUNT,
         Crc16(packet + CCD_HEADER_BYTES, CCD_SENSOR_PIXEL_COUNT));

  // DMA 发送完成前不能再次改写这个缓冲区，也不能启动另一种报文。
  serial_state.tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart1, packet, CCD_PACKET_BYTES) != HAL_OK)
    serial_state.tx_busy = 0U;
}

void CarSerial_SendMotor(const CarMotorTelemetry *telemetry)
{
  const MotorController *right = telemetry->drive.right_motor;
  const MotorController *left = telemetry->drive.left_motor;
  const LineTracker *line = telemetry->line;
  uint8_t *packet = serial_state.motor_packet;

  if (serial_state.tx_busy) return;

  // MTR2 的字段和位置保持不变，现有 Python 上位机仍可直接解析。
  memcpy(packet, "MTR2", 4U);
  PutU16(packet + 4, serial_state.motor_sequence++);
  packet[6] = telemetry->drive_mode;
  packet[7] = 0x03U;  // TIM2 和 TIM3 编码器均已配置。

  PutFloat(packet + 8, right->target_rpm);
  PutFloat(packet + 12, right->actual_rpm);
  PutFloat(packet + 16, right->raw_rpm);
  PutFloat(packet + 20, right->output_percent);
  PutFloat(packet + 24, left->target_rpm);
  PutFloat(packet + 28, left->actual_rpm);
  PutFloat(packet + 32, left->raw_rpm);
  PutFloat(packet + 36, left->output_percent);

  packet[40] = (uint8_t)line->state;
  packet[41] = line->config.dark_line;
  packet[42] = line->threshold_used;
  packet[43] = line->lost_frames;
  PutU16(packet + 44, line->left);
  PutU16(packet + 46, line->right);
  PutU16(packet + 48, line->width);
  PutU16(packet + 50, (uint16_t)(telemetry->max_steer_rpm * 10.0f));

  PutFloat(packet + 52, line->raw_center);
  PutFloat(packet + 56, line->filtered_center);
  PutFloat(packet + 60, line->error);
  PutFloat(packet + 64, telemetry->steering_rpm);
  PutFloat(packet + 68, telemetry->base_rpm);
  PutFloat(packet + 72, telemetry->drive.right_trim);
  PutFloat(packet + 76, telemetry->drive.left_trim);
  PutFloat(packet + 80, telemetry->steering_kp);
  PutFloat(packet + 84, telemetry->steering_kd);
  PutU16(packet + 88, Crc16(packet + 4, 84U));

  serial_state.tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart1, packet, MOTOR_PACKET_BYTES) != HAL_OK)
    serial_state.tx_busy = 0U;
}

// 接收回调只负责拼接 ASCII 命令；浮点解析和电机操作留给主循环。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;

  if (serial_state.rx_byte == '\r' || serial_state.rx_byte == '\n')
  {
    if (!serial_state.discard_until_newline && serial_state.rx_length)
    {
      serial_state.receiving[serial_state.rx_length] = '\0';
      if (strcmp(serial_state.receiving, "STOP") == 0)
      {
        // 急停立即清空排队的行驶命令，防止主循环停车后又执行旧命令。
        serial_state.queue_count = 0U;
        serial_state.queue_head = serial_state.queue_tail;
        serial_state.stop_requested = 1U;
      }
      else if (serial_state.queue_count < COMMAND_QUEUE_SIZE &&
               !serial_state.stop_requested)
      {
        memcpy(serial_state.commands[serial_state.queue_tail],
               serial_state.receiving, serial_state.rx_length + 1U);
        serial_state.queue_tail =
            (uint8_t)((serial_state.queue_tail + 1U) % COMMAND_QUEUE_SIZE);
        ++serial_state.queue_count;
      }
    }
    serial_state.rx_length = 0U;
    serial_state.discard_until_newline = 0U;
  }
  else if (serial_state.discard_until_newline)
  {
    // 命令过长时整行丢弃，到换行后再恢复接收。
  }
  else if (serial_state.rx_length < CAR_SERIAL_COMMAND_BYTES - 1U)
  {
    serial_state.receiving[serial_state.rx_length++] =
        (char)serial_state.rx_byte;
  }
  else
  {
    serial_state.rx_length = 0U;
    serial_state.discard_until_newline = 1U;
  }

  HAL_UART_Receive_IT(&huart1, &serial_state.rx_byte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1) serial_state.tx_busy = 0U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart != &huart1) return;

  // 接收出错时发送 DMA 可能仍在运行，不能过早复用发送缓冲区。
  if (huart1.gState == HAL_UART_STATE_READY)
    serial_state.tx_busy = 0U;
  HAL_UART_Receive_IT(&huart1, &serial_state.rx_byte, 1U);
}
