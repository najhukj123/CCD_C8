#ifndef PID_H
#define PID_H

// 离散位置式 PID：Ki/Kd 已包含原控制周期，不另乘或除 dt。
typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_gate;
  float max_output;
} PID_Config;

typedef struct
{
  PID_Config config;
  float error;
  float previous_error;
  float integral;
  float output;
} PID;

void PID_Init(PID *pid, const PID_Config *config);
void PID_Reset(PID *pid);
float PID_Update(PID *pid, float error);

#endif
