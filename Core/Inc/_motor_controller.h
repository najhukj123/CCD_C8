#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

// 一个电机的速度控制参数。每圈计数指减速后的输出轴，不是电机裸轴。
typedef struct
{
  float counts_per_turn;        // 输出轴每转一圈的编码器计数。
  float period_s;               // Update 调用间隔，单位秒。
  float filter_alpha;           // 实际转速的一阶滤波系数。
  float kp;                     // 比例系数。
  float ki;                     // 积分系数。
  float kd;                     // 微分系数。
  float integral_gate_rpm;      // 误差绝对值小于此值才积分。
  float max_output_percent;     // PWM 输出百分比上限。
  float max_target_rpm;         // 目标转速绝对值上限。
} MotorConfig;

// 每个车轮持有独立状态，不能共享积分量或滤波结果。
typedef struct
{
  MotorConfig config;
  float target_rpm;             // 限幅后的目标转速。
  float raw_rpm;                // 本周期编码器计算得到的未滤波转速。
  float actual_rpm;             // 滤波后的实际转速。
  float error;                  // 当前速度误差。
  float previous_error;         // 上周期速度误差。
  float integral;               // 累计误差。
  float output_percent;         // 输出给驱动器的有符号 PWM 百分比。
} MotorController;

// 复制配置并清空状态。
void MotorController_Init(MotorController *motor, const MotorConfig *config);
// 设置目标转速，并按配置限幅。
void MotorController_SetTarget(MotorController *motor, float rpm);
// 清除驱动和 PID 记忆；保留实际转速供停车后的遥测观察。
void MotorController_ClearDrive(MotorController *motor);
// 输入本周期编码器增量，返回有符号 PWM 百分比。
float MotorController_Update(MotorController *motor, int16_t encoder_delta);

#endif
