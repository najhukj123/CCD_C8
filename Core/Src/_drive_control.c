#include "_drive_control.h"
#include "main.h"
#include "tim.h"
#include <string.h>

enum { RIGHT_WHEEL = 0, LEFT_WHEEL = 1, WHEEL_COUNT = 2 };

typedef struct
{
  MotorController controller;  // 单轮速度 PID 及测速结果。
  float requested_rpm;         // 应用层希望这个轮子达到的 RPM。
  float trim;                  // 左右轮差异补偿，默认接近 1。
  uint16_t previous_count;    // 上一次读取的编码器计数。
} WheelState;

// 只有两个轮子；不再额外套一层仅含 wheels 的结构体。
static WheelState wheels[WHEEL_COUNT];

// 参数顺序：输出轴每圈计数、采样秒数、速度滤波系数、Kp、Ki、Kd、
// 积分启用误差范围、最大 PWM 百分比、最大目标转速。
static const MotorConfig motor_configs[WHEEL_COUNT] = {
  { // 右轮：顺序与 MotorConfig 的字段顺序一致。
    1456.0f,       // counts_per_turn：输出轴转一圈的编码器计数。
    0.020f,        // period_s：每 20 ms 更新一次。
    0.293f,        // filter_alpha：测速低通滤波系数。
    0.81f,         // kp：比例系数。
    0.059f,        // ki：积分系数。
    0.15f,         // kd：微分系数。
    140.0f,        // integral_gate_rpm：误差过大时暂停积分。
    100.0f,        // max_output_percent：PWM 百分比上限。
    DRIVE_MAX_RPM  // max_target_rpm：目标速度上限。
  },
  { // 左轮。
    1509.0f,       // counts_per_turn
    0.020f,        // period_s
    0.293f,        // filter_alpha
    0.85f,         // kp
    0.050f,        // ki
    0.00f,         // kd
    140.0f,        // integral_gate_rpm
    100.0f,        // max_output_percent
    DRIVE_MAX_RPM  // max_target_rpm
  }
};

// 把浮点值限制在上下限之间。
static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

// 返回浮点数的绝对值。
static float AbsFloat(float value)
{
  if (value < 0.0f) return -value;
  return value;
}

static uint16_t ReadEncoder(unsigned wheel_index)
{
  TIM_HandleTypeDef *timer = wheel_index == RIGHT_WHEEL ? &htim2 : &htim3;
  return (uint16_t)__HAL_TIM_GET_COUNTER(timer);
}

static void WriteWheel(unsigned wheel_index, float percent)
{
  uint32_t channel = wheel_index == RIGHT_WHEEL ? TIM_CHANNEL_1 : TIM_CHANNEL_4;
  GPIO_TypeDef *port = wheel_index == RIGHT_WHEEL ?
      MOTOR1_PH_GPIO_Port : MOTOR2_PH_GPIO_Port;
  uint16_t pin = wheel_index == RIGHT_WHEEL ? MOTOR1_PH_Pin : MOTOR2_PH_Pin;
  uint32_t compare;

  percent = ClampFloat(percent, -100.0f, 100.0f);

  // 换向前先关闭 PWM，避免 PH 切换时仍向旧方向输出。
  __HAL_TIM_SET_COMPARE(&htim1, channel, 0U);
  if (percent == 0.0f) return;

  // CubeMX：TIM1 CH1/CH4 分别驱动右/左轮；PH 低电平表示前进。
  HAL_GPIO_WritePin(port, pin,
                    percent > 0.0f ? GPIO_PIN_RESET : GPIO_PIN_SET);
  compare = (uint32_t)(AbsFloat(percent) *
                       __HAL_TIM_GET_AUTORELOAD(&htim1) / 100.0f);
  __HAL_TIM_SET_COMPARE(&htim1, channel, compare);
}

void DriveControl_Init(void)
{
  unsigned index;

  memset(wheels, 0, sizeof(wheels));
  for (index = 0U; index < WHEEL_COUNT; ++index)
    MotorController_Init(&wheels[index].controller, &motor_configs[index]);

  wheels[RIGHT_WHEEL].trim = 0.996f;
  wheels[LEFT_WHEEL].trim = 1.004f;

  // 外设已由 CubeMX 初始化；此处只启动控制所需的通道。
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  for (index = 0U; index < WHEEL_COUNT; ++index)
    wheels[index].previous_count = ReadEncoder(index);

  DriveControl_Stop();
}

void DriveControl_Stop(void)
{
  unsigned index;

  for (index = 0U; index < WHEEL_COUNT; ++index)
  {
    WheelState *wheel = &wheels[index];
    wheel->requested_rpm = 0.0f;
    MotorController_ClearDrive(&wheel->controller);
    WriteWheel(index, 0.0f);
  }
}

void DriveControl_SetRequested(float right_rpm, float left_rpm)
{
  wheels[RIGHT_WHEEL].requested_rpm = right_rpm;
  wheels[LEFT_WHEEL].requested_rpm = left_rpm;
}

void DriveControl_SetTrim(float right_trim, float left_trim)
{
  wheels[RIGHT_WHEEL].trim = right_trim;
  wheels[LEFT_WHEEL].trim = left_trim;
}

void DriveControl_Update(float ramp)
{
  unsigned index;

  // 每个轮子走同一套流程：读编码器 → 算目标 RPM → PID → 写 PWM。
  for (index = 0U; index < WHEEL_COUNT; ++index)
  {
    WheelState *wheel = &wheels[index];
    uint16_t current_count = ReadEncoder(index);
    int16_t count_delta = (int16_t)(current_count - wheel->previous_count);

    wheel->previous_count = current_count;
    MotorController_SetTarget(&wheel->controller,
        wheel->requested_rpm * wheel->trim * ramp);
    MotorController_Update(&wheel->controller, count_delta);
    WriteWheel(index, wheel->controller.output_percent);
  }
}

void DriveControl_GetStatus(DriveStatus *status)
{
  status->right_motor = &wheels[RIGHT_WHEEL].controller;
  status->left_motor = &wheels[LEFT_WHEEL].controller;
  status->right_trim = wheels[RIGHT_WHEEL].trim;
  status->left_trim = wheels[LEFT_WHEEL].trim;
}
