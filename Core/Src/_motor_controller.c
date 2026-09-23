#include "_motor_controller.h"

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

void MotorController_Init(MotorController *motor, const MotorConfig *config)
{
  motor->config = *config;
  motor->raw_rpm = 0.0f;
  motor->actual_rpm = 0.0f;
  MotorController_ClearDrive(motor);
}

void MotorController_SetTarget(MotorController *motor, float rpm)
{
  const MotorConfig *config = &motor->config;
  motor->target_rpm = ClampFloat(rpm, -config->max_target_rpm,
                               config->max_target_rpm);
}

void MotorController_ClearDrive(MotorController *motor)
{
  // 停车时清掉 PID 记忆，避免再次启动沿用旧积分。
  // 实际转速继续保留，遥测仍能看到车轮减速的过程。
  motor->target_rpm = 0.0f;
  motor->error = 0.0f;
  motor->previous_error = 0.0f;
  motor->integral = 0.0f;
  motor->output_percent = 0.0f;
}

float MotorController_Update(MotorController *motor, int16_t encoder_delta)
{
  const MotorConfig *config = &motor->config;
  float integral_limit;

  // 编码器增量换算成输出轴 RPM，并用一阶低通滤掉计数抖动。
  motor->raw_rpm = (float)encoder_delta * 60.0f /
      (config->counts_per_turn * config->period_s);
  motor->actual_rpm += config->filter_alpha *
      (motor->raw_rpm - motor->actual_rpm);

  if (motor->target_rpm == 0.0f)
  {
    MotorController_ClearDrive(motor);
    return 0.0f;
  }

  motor->previous_error = motor->error;
  motor->error = motor->target_rpm - motor->actual_rpm;

  // 起步或堵转造成大误差时不积分，防止 PWM 饱和后积累过多误差。
  if (config->ki > 0.0f &&
      AbsFloat(motor->error) < config->integral_gate_rpm)
  {
    motor->integral += motor->error;
    integral_limit = config->max_output_percent / config->ki;
    motor->integral = ClampFloat(motor->integral,
                                 -integral_limit, integral_limit);
  }
  else
  {
    motor->integral = 0.0f;
  }

  motor->output_percent = config->kp * motor->error +
      config->ki * motor->integral +
      config->kd * (motor->error - motor->previous_error);
  motor->output_percent = ClampFloat(motor->output_percent,
      -config->max_output_percent, config->max_output_percent);
  return motor->output_percent;
}
