#include "pid.h"

static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static float AbsFloat(float value)
{
  return value < 0.0f ? -value : value;
}

void PID_Reset(PID *pid)
{
  pid->error = 0.0f;
  pid->previous_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

void PID_Init(PID *pid, const PID_Config *config)
{
  pid->config = *config;
  PID_Reset(pid);
}

float PID_Update(PID *pid, float error)
{
  const PID_Config *config = &pid->config;
  float integral_limit;

  pid->previous_error = pid->error;
  pid->error = error;

  // 起步或堵转造成大误差时不积分，防止 PWM 饱和后积累过多误差。
  if (config->ki > 0.0f &&
      AbsFloat(pid->error) < config->integral_gate)
  {
    pid->integral += pid->error;
    integral_limit = config->max_output / config->ki;
    pid->integral = ClampFloat(pid->integral,
                                 -integral_limit, integral_limit);
  }
  else
  {
    pid->integral = 0.0f;
  }

  pid->output = config->kp * pid->error +
      config->ki * pid->integral +
      config->kd * (pid->error - pid->previous_error);
  pid->output = ClampFloat(pid->output,
      -config->max_output, config->max_output);
  return pid->output;
}
