#include "encoder.h"
#include "tim.h"

enum { RIGHT_WHEEL = 0 };

uint16_t Encoder_Read(unsigned wheel_index)
{
  TIM_HandleTypeDef *timer = wheel_index == RIGHT_WHEEL ? &htim2 : &htim3;
  return (uint16_t)__HAL_TIM_GET_COUNTER(timer);
}

void Encoder_Init(void)
{
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}
