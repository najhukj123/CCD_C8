#include "motor.h"
#include "main.h"
#include "tim.h"

enum { RIGHT_WHEEL = 0 };

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

void Motor_Write(unsigned wheel_index, float percent)
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

void Motor_Init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
