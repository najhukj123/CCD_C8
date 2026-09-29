#include "_motor_controller.h"

static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void MotorController_Init(MotorController *motor, const MotorConfig *config)
{
  PID_Config pid_config;

  motor->config = *config;
  pid_config.kp = config->kp;
  pid_config.ki = config->ki;
  pid_config.kd = config->kd;
  pid_config.integral_gate = config->integral_gate_rpm;
  pid_config.max_output = config->max_output_percent;
  PID_Init(&motor->pid, &pid_config);
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
  // 实际转速继续保留，调试时仍可观察车轮减速过程。
  motor->target_rpm = 0.0f;
  PID_Reset(&motor->pid);
  motor->output_percent = 0.0f;
}

float MotorController_Update(MotorController *motor, int16_t encoder_delta)
{
  const MotorConfig *config = &motor->config;

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

  motor->output_percent = PID_Update(&motor->pid,
      motor->target_rpm - motor->actual_rpm);
  return motor->output_percent;
}
