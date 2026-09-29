#include "track.h"
#include "_ccd_sensor.h"
#include "_line_tracker.h"
#include "_drive_control.h"
#include "main.h"
#include <string.h>

enum
{
  CONTROL_PERIOD_MS = 20,
  CCD_TIMEOUT_MS = 300,
  START_RAMP_MS = 600
};

// 固定运行参数：转速单位为 RPM；方向环沿用 PD，无积分项。
#define TRACK_BASE_RPM      260.0f
#define TRACK_STEERING_KP   0.22f
#define TRACK_STEERING_KD   0.21f
#define TRACK_MAX_STEER_RPM 330.0f
#define TRACK_LINE_THRESHOLD 40U

// 这是 Mod 控制层唯一的整车运行状态。电机、CCD的内部状态仍由各自模块保存。
// 把常用字段放在同一层，读代码时可以直接写 car.line、car.steering_rpm 等。
typedef struct
{
  uint8_t started;             // 首次有效找线后启动缓启动计时。

  // CCD 找线结果和外层转向 PD。LineTracker 自己保存线的位置与丢线状态。
  LineTracker line;
  float previous_line_error;
  float steering_rpm;

  // HAL_GetTick() 的毫秒时间戳，用来安排周期任务和检测超时。
  uint32_t last_frame_ms;
  uint32_t control_ms;
  uint32_t launch_ms;
} TrackStatus;

static TrackStatus car;

// 1500 像素；两端各排除 30 像素；有效线宽 20～350；默认识别黑线。
// 修改 TRACK_LINE_THRESHOLD 即可调整黑线识别阈值。
static const LineConfig line_config = {
  CCD_SENSOR_PIXEL_COUNT, // pixel_count：一帧有 1500 个像素。
  30U,                    // roi_margin：左右两端各忽略 30 个像素。
  20U,                    // min_width：有效线段最小宽度。
  350U,                   // max_width：有效线段最大宽度。
  300.0f,                 // max_center_offset：线中心距传感器中点的上限。
  180.0f,                 // max_jump：连续两帧允许的最大中心跳变。
  0.28f,                  // filter_alpha：中心位置的低通滤波系数。
  1U,                     // dark_line：1 识别黑线，0 识别白线。
  TRACK_LINE_THRESHOLD    // threshold：低于此值的像素识别为黑线。
};

// 把浮点值限制在上下限之间。
static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static void UpdateMotors(uint32_t now_ms)
{
  float ramp = 1.0f;
  uint32_t elapsed_ms = (uint32_t)(now_ms - car.launch_ms);

  // 自动循迹时 CCD 过期，只清零目标速度；新帧到来后可以继续循迹。
  if ((uint32_t)(now_ms - car.last_frame_ms) > CCD_TIMEOUT_MS)
    DriveControl_SetRequested(0.0f, 0.0f);

  // 每次启动后的 600 ms 逐渐增加目标转速，减少突然起步。
  if (elapsed_ms < START_RAMP_MS)
    ramp = (float)elapsed_ms / (float)START_RAMP_MS;
  DriveControl_Update(ramp);
}

static void UpdateTracking(const uint8_t *pixels, uint32_t now_ms)
{
  LineTracker *line = &car.line;
  float steering;
  float right_rpm;
  float left_rpm;

  LineTracker_Update(line, pixels);

  // 当前帧未找到有效线时，立即停止两轮。
  if (line->state == LINE_LOST)
  {
    DriveControl_SetRequested(0.0f, 0.0f);
    car.steering_rpm = 0.0f;
    car.previous_line_error = 0.0f;
    return;
  }

  // 首次看到有效窄线才开始 600 ms 缓启动，等待 CCD 不消耗缓启动时间。
  if (!car.started)
  {
    car.started = 1U;
    car.launch_ms = now_ms;
  }

  // 外层是 PD：当前偏差决定转向强度，偏差变化抑制转向过冲。
  steering = TRACK_STEERING_KP * line->error +
             TRACK_STEERING_KD * (line->error - car.previous_line_error);
  car.previous_line_error = line->error;
  car.steering_rpm = ClampFloat(steering,
      -TRACK_MAX_STEER_RPM, TRACK_MAX_STEER_RPM);

  // CCD 像素已按车体方向镜像；右转时右轮减速、左轮加速。
  right_rpm = ClampFloat(TRACK_BASE_RPM - car.steering_rpm,
                         -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
  left_rpm = ClampFloat(TRACK_BASE_RPM + car.steering_rpm,
                        -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
  DriveControl_SetRequested(right_rpm, left_rpm);
}

void Track_Init(void)
{
  uint32_t now_ms = HAL_GetTick();

  memset(&car, 0, sizeof(car));
  LineTracker_Init(&car.line, &line_config);
  car.last_frame_ms = now_ms;

  car.control_ms = now_ms;
  car.launch_ms = now_ms;

  DriveControl_Init();
  CcdSensor_Init();
}

void Track_Poll(void)
{
  const uint8_t *pixels;
  uint32_t now_ms = HAL_GetTick();

  // 保留原先 20 ms 主循环调度与顺序，阻塞式 SPI 不放进中断。
  if ((uint32_t)(now_ms - car.control_ms) >= CONTROL_PERIOD_MS)
  {
    car.control_ms += CONTROL_PERIOD_MS;
    UpdateMotors(now_ms);
  }
  pixels = CcdSensor_ReadPixels();
  if (pixels != 0)
  {
    now_ms = HAL_GetTick();
    car.last_frame_ms = now_ms;
    UpdateTracking(pixels, now_ms);
  }
}
