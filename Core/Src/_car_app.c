#include "_car_app.h"
#include "_car_serial.h"
#include "_ccd_sensor.h"
#include "_drive_control.h"
#include "_line_tracker.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

enum
{
  CONTROL_PERIOD_MS = 20,
  MOTOR_REPORT_PERIOD_MS = 50,
  CCD_REPORT_PERIOD_MS = 80,
  CCD_TIMEOUT_MS = 300,
  LINK_TIMEOUT_MS = 750,
  START_RAMP_MS = 600,
  PARKING_DARK_PERCENT = 70,
  PARKING_SIDE_DARK_PERCENT = 60,
  PARKING_CONFIRM_FRAMES = 2,
  PARKING_CLEAR_FRAMES = 3,
  PARKING_MIN_LAP_MS = 3000
};

// 模式编号是 MTR2 报文的一部分，必须与现有上位机保持一致。
typedef enum
{
  DRIVE_STOPPED = 0,
  DRIVE_TRACKING = 3,
  DRIVE_MANUAL = 4,
  DRIVE_PARKED = 7
} DriveMode;

// 第一次宽黑线是起点；离开它后，再次看到宽黑线才是终点。
typedef enum
{
  PARKING_WAIT_START = 0,
  PARKING_WAIT_CLEAR = 1,
  PARKING_WAIT_FINISH = 2
} ParkingState;

// 这是应用层唯一的运行状态。电机、CCD、串口的内部状态仍由各自模块保存。
// 把常用字段放在同一层，读代码时可以直接写 car.line、car.base_rpm 等。
typedef struct
{
  DriveMode mode;

  // CCD 找线结果和外层转向 PD。LineTracker 自己保存线的位置与丢线状态。
  LineTracker line;
  float base_rpm;
  float steering_kp;
  float steering_kd;
  float max_steer_rpm;
  float previous_line_error;
  float steering_rpm;

  ParkingState parking_state;
  uint8_t parking_dark_frames;
  uint8_t parking_clear_frames;
  uint8_t parking_threshold;  // 最近一次正常循迹的阈值；0 表示还没有参照。
  uint32_t first_parking_ms;

  // HAL_GetTick() 的毫秒时间戳，用来安排周期任务和检测超时。
  uint32_t last_frame_ms;
  uint32_t control_ms;
  uint32_t motor_report_ms;
  uint32_t ccd_report_ms;
  uint32_t last_command_ms;
  uint32_t launch_ms;
} CarState;

static CarState car;

// 1000 像素；两端各排除 30 像素；有效线宽 20～350；默认识别黑线。
// 阈值为 0 时按当前帧灰度分布自动计算。
static const LineConfig line_config = {
  CCD_SENSOR_PIXEL_COUNT, // pixel_count：一帧有 1000 个像素。
  30U,                    // roi_margin：左右两端各忽略 30 个像素。
  20U,                    // min_width：有效线段最小宽度。
  350U,                   // max_width：有效线段最大宽度。
  250.0f,                 // max_center_offset：线中心距传感器中点的上限。
  180.0f,                 // max_jump：连续两帧允许的最大中心跳变。
  0.28f,                  // filter_alpha：中心位置的低通滤波系数。
  3U,                     // hold_frames：短暂丢线时保持上次偏差的帧数。
  1U,                     // dark_line：1 识别黑线，0 识别白线。
  0U                      // threshold：0 表示使用自动阈值。
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

// 横向停车线会让 CCD 的大部分视野变黑，包括左右两侧。
// 使用上一次正常循迹的阈值：整帧都黑时，本帧自动阈值无法区分黑线与背景。
static uint8_t ParkingLineVisible(const uint8_t *pixels, uint8_t threshold)
{
  uint16_t first = car.line.config.roi_margin;
  uint16_t last = car.line.config.pixel_count - first - 1U;
  uint16_t width = last - first + 1U;
  uint16_t side_width = width / 5U;
  uint16_t index;
  uint16_t dark = 0U;
  uint16_t left_dark = 0U;
  uint16_t right_dark = 0U;

  if (threshold == 0U || side_width == 0U) return 0U;
  for (index = first; index <= last; ++index)
  {
    if (pixels[index] >= threshold) continue;
    ++dark;
    if (index < first + side_width) ++left_dark;
    if (index > last - side_width) ++right_dark;
  }

  return (uint32_t)dark * 100U >=
             (uint32_t)width * PARKING_DARK_PERCENT &&
         (uint32_t)left_dark * 100U >=
             (uint32_t)side_width * PARKING_SIDE_DARK_PERCENT &&
         (uint32_t)right_dark * 100U >=
             (uint32_t)side_width * PARKING_SIDE_DARK_PERCENT;
}

// 停车：清除循迹转向记忆，并让电机控制器清目标和 PWM。
static void StopVehicle(void)
{
  car.mode = DRIVE_STOPPED;
  car.steering_rpm = 0.0f;
  car.previous_line_error = 0.0f;
  DriveControl_Stop();
}

static void ApplyEmergencyStop(void)
{
  if (CarSerial_TakeEmergencyStop())
  {
    StopVehicle();
    car.last_command_ms = HAL_GetTick();
  }
}

static void UpdateMotors(uint32_t now_ms)
{
  float ramp = 1.0f;
  uint32_t elapsed_ms = (uint32_t)(now_ms - car.launch_ms);

  // PC 端超过 750 ms 未发 KEEP 或行驶命令，就进入停车模式。
  if ((car.mode == DRIVE_TRACKING || car.mode == DRIVE_MANUAL) &&
      (uint32_t)(now_ms - car.last_command_ms) > LINK_TIMEOUT_MS)
    StopVehicle();

  // 自动循迹时 CCD 过期，只清零目标速度；新帧到来后可以继续循迹。
  if (car.mode == DRIVE_TRACKING &&
      (uint32_t)(now_ms - car.last_frame_ms) > CCD_TIMEOUT_MS)
    DriveControl_SetRequested(0.0f, 0.0f);

  // 每次启动后的 600 ms 逐渐增加目标转速，减少突然起步。
  if (elapsed_ms < START_RAMP_MS)
    ramp = (float)elapsed_ms / (float)START_RAMP_MS;
  DriveControl_Update(ramp);
}

// 返回 1 表示当前帧是宽黑线，本帧不用再按普通窄线计算转向。
static uint8_t HandleParkingLine(const uint8_t *pixels, uint32_t now_ms)
{
  uint8_t threshold = car.line.config.threshold != 0U ?
      car.line.config.threshold : car.parking_threshold;

  if (car.mode != DRIVE_TRACKING || !car.line.config.dark_line)
    return 0U;

  if (ParkingLineVisible(pixels, threshold))
  {
    // 停车线不是普通窄线，告诉上位机本帧正在暂时保持上次位置。
    if (car.line.state != LINE_LOST)
    {
      car.line.state = LINE_HOLD;
      car.line.width = 0U;
    }
    car.parking_clear_frames = 0U;
    if (car.parking_dark_frames < PARKING_CONFIRM_FRAMES)
      ++car.parking_dark_frames;

    if (car.parking_dark_frames >= PARKING_CONFIRM_FRAMES)
    {
      if (car.parking_state == PARKING_WAIT_START)
      {
        car.parking_state = PARKING_WAIT_CLEAR;
        car.first_parking_ms = now_ms;
      }
      else if (car.parking_state == PARKING_WAIT_FINISH &&
               (uint32_t)(now_ms - car.first_parking_ms) >= PARKING_MIN_LAP_MS)
      {
        StopVehicle();
        car.mode = DRIVE_PARKED;
        return 1U;
      }
    }

    // 起点宽黑线遮住了原来的窄线，暂时沿上一帧方向驶过它。
    DriveControl_SetRequested(
        ClampFloat(car.base_rpm - car.steering_rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM),
        ClampFloat(car.base_rpm + car.steering_rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM));
    return 1U;
  }

  car.parking_dark_frames = 0U;
  if (car.parking_state == PARKING_WAIT_CLEAR)
  {
    if (car.parking_clear_frames < PARKING_CLEAR_FRAMES)
      ++car.parking_clear_frames;
    if (car.parking_clear_frames >= PARKING_CLEAR_FRAMES)
      car.parking_state = PARKING_WAIT_FINISH;
  }
  return 0U;
}

static void UpdateTracking(const uint8_t *pixels, uint32_t now_ms)
{
  LineTracker *line = &car.line;
  float steering;
  float steering_limit;
  float right_rpm;
  float left_rpm;

  if (HandleParkingLine(pixels, now_ms)) return;

  // 停车和手动模式也更新识别结果，便于上位机观察与中心标定。
  LineTracker_Update(line, pixels);
  if (line->state == LINE_TRACKING && line->config.dark_line)
    car.parking_threshold = line->threshold_used;
  if (car.mode != DRIVE_TRACKING)
  {
    car.steering_rpm = 0.0f;
    return;
  }

  // 短暂丢线时 LineTracker 保留上一次偏差；确认丢线后才停止两轮。
  if (line->state == LINE_LOST)
  {
    DriveControl_SetRequested(0.0f, 0.0f);
    car.steering_rpm = 0.0f;
    car.previous_line_error = 0.0f;
    return;
  }

  // 外层是 PD：当前偏差决定转向强度，偏差变化抑制转向过冲。
  steering = car.steering_kp * line->error +
             car.steering_kd * (line->error - car.previous_line_error);
  car.previous_line_error = line->error;
  steering_limit = car.max_steer_rpm;
  if (steering_limit > AbsFloat(car.base_rpm))
    steering_limit = AbsFloat(car.base_rpm);
  car.steering_rpm = ClampFloat(steering, -steering_limit, steering_limit);

  // CCD 像素已按车体方向镜像；右转时右轮减速、左轮加速。
  right_rpm = ClampFloat(car.base_rpm - car.steering_rpm,
                         -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
  left_rpm = ClampFloat(car.base_rpm + car.steering_rpm,
                        -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
  DriveControl_SetRequested(right_rpm, left_rpm);
}

static void SendMotorTelemetry(void)
{
  CarMotorTelemetry telemetry;

  // 遥测只借用当前状态，串口会在本次调用中完成打包。
  DriveControl_GetStatus(&telemetry.drive);
  telemetry.drive_mode = (uint8_t)car.mode;
  telemetry.parking_state = (uint8_t)car.parking_state;
  telemetry.line = &car.line;
  telemetry.max_steer_rpm = car.max_steer_rpm;
  telemetry.steering_rpm = car.steering_rpm;
  telemetry.base_rpm = car.base_rpm;
  telemetry.steering_kp = car.steering_kp;
  telemetry.steering_kd = car.steering_kd;
  CarSerial_SendMotor(&telemetry);
}

static void ParseCommand(char *command)
{
  char *separator;
  uint32_t now_ms = HAL_GetTick();

  // 串口模块只负责收齐命令；参数含义和行驶决策都由应用层处理。
  if (strcmp(command, "KEEP") == 0)
  {
    car.last_command_ms = now_ms;
    return;
  }
  if (strcmp(command, "STOP") == 0)
  {
    StopVehicle();
    car.last_command_ms = now_ms;
    return;
  }

  // 数据流开关只影响上传，不改变当前行驶状态。
  if (strcmp(command, "STREAM,OFF") == 0)
    CarSerial_SetStream(CAR_STREAM_OFF);
  else if (strcmp(command, "STREAM,MOTOR") == 0)
    CarSerial_SetStream(CAR_STREAM_MOTOR);
  else if (strcmp(command, "STREAM,CCD") == 0)
    CarSerial_SetStream(CAR_STREAM_CCD);
  else if (strcmp(command, "STREAM,BOTH") == 0)
    CarSerial_SetStream(CAR_STREAM_BOTH);
  else if (strncmp(command, "DUAL,", 5U) == 0)
  {
    float right_rpm;
    float left_rpm;

    separator = strchr(command + 5, ',');
    if (separator == 0) return;
    *separator = '\0';
    right_rpm = ClampFloat((float)atof(command + 5),
                           -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
    left_rpm = ClampFloat((float)atof(separator + 1),
                          -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
    // 手动双轮命令先清除旧 PID 状态，再按新的目标速度缓启动。
    StopVehicle();
    DriveControl_SetRequested(right_rpm, left_rpm);
    if (right_rpm != 0.0f || left_rpm != 0.0f)
      car.mode = DRIVE_MANUAL;
    car.last_command_ms = car.launch_ms = now_ms;
  }
  // 以下循迹参数只保存在 RAM 中，本次上电期间有效。
  else if (strncmp(command, "TH,", 3U) == 0)
    car.line.config.threshold = (uint8_t)ClampFloat(
        (float)atof(command + 3), 0.0f, 255.0f);
  else if (strncmp(command, "DARK,", 5U) == 0)
    car.line.config.dark_line = atoi(command + 5) ? 1U : 0U;
  else if (strncmp(command, "STEERKP,", 8U) == 0)
    car.steering_kp = ClampFloat((float)atof(command + 8), 0.0f, 5.0f);
  else if (strncmp(command, "STEERKD,", 8U) == 0)
    car.steering_kd = ClampFloat((float)atof(command + 8), 0.0f, 5.0f);
  else if (strncmp(command, "MAXSTEER,", 9U) == 0)
    car.max_steer_rpm = ClampFloat(
        (float)atof(command + 9), 0.0f, DRIVE_MAX_RPM);
  else if (strncmp(command, "TRIM,", 5U) == 0)
  {
    float right_trim;
    float left_trim;

    separator = strchr(command + 5, ',');
    if (separator == 0) return;
    *separator = '\0';
    right_trim = ClampFloat((float)atof(command + 5), 0.8f, 1.2f);
    left_trim = ClampFloat((float)atof(separator + 1), 0.8f, 1.2f);
    DriveControl_SetTrim(right_trim, left_trim);
  }
  else if (strcmp(command, "CENTER") == 0)
  {
    // 以当前识别到的赛道中心为零点，同时清除转向微分记忆。
    LineTracker_CenterHere(&car.line);
    car.previous_line_error = 0.0f;
    car.steering_rpm = 0.0f;
  }
  else if (strncmp(command, "AUTO,", 5U) == 0)
  {
    float base_rpm = ClampFloat((float)atof(command + 5),
                                -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
    // 自动循迹启动时先停旧模式，下一帧 CCD 再决定左右轮速度。
    StopVehicle();
    car.base_rpm = base_rpm;
    car.parking_state = PARKING_WAIT_START;
    car.parking_dark_frames = 0U;
    car.parking_clear_frames = 0U;
    car.first_parking_ms = 0U;
    car.mode = DRIVE_TRACKING;
    car.last_command_ms = car.launch_ms = now_ms;
    // 等下一帧有效 CCD 数据到来，UpdateTracking 才会写入两轮目标转速。
  }
}

void CarApp_Init(void)
{
  uint32_t now_ms = HAL_GetTick();

  memset(&car, 0, sizeof(car));
  LineTracker_Init(&car.line, &line_config);
  car.steering_kp = 0.35f;
  car.steering_kd = 0.20f;
  car.max_steer_rpm = 120.0f;
  car.last_frame_ms = now_ms;

  car.control_ms = now_ms;
  car.motor_report_ms = now_ms;
  car.ccd_report_ms = now_ms;
  car.last_command_ms = now_ms;
  car.launch_ms = now_ms;

  DriveControl_Init();
  CcdSensor_Init();
  CarSerial_Init();
}

void CarApp_Poll(void)
{
  char command[CAR_SERIAL_COMMAND_BYTES];
  const CcdFrame *frame;
  uint32_t now_ms;

  // STOP 优先于已经排队的控制命令；解析期间收到 STOP 也要再次处理。
  ApplyEmergencyStop();
  if (CarSerial_ReadCommand(command) && !CarSerial_StopPending())
    ParseCommand(command);
  ApplyEmergencyStop();

  // 按固定间隔更新电机控制。
  now_ms = HAL_GetTick();
  if ((uint32_t)(now_ms - car.control_ms) >= CONTROL_PERIOD_MS)
  {
    car.control_ms += CONTROL_PERIOD_MS;
    UpdateMotors(now_ms);
  }

  // 每帧都参与循迹，只按 80 ms 上传一帧，给电机遥测留出带宽。
  frame = CcdSensor_ReadFrame();
  if (frame != 0)
  {
    now_ms = HAL_GetTick();
    car.last_frame_ms = now_ms;
    UpdateTracking(frame->pixels, now_ms);
    if (CarSerial_StreamEnabled(CAR_STREAM_CCD) &&
        (uint32_t)(now_ms - car.ccd_report_ms) >= CCD_REPORT_PERIOD_MS)
    {
      car.ccd_report_ms = now_ms;
      CarSerial_SendCcd(frame);
    }
  }

  now_ms = HAL_GetTick();
  if (CarSerial_StreamEnabled(CAR_STREAM_MOTOR) &&
      (uint32_t)(now_ms - car.motor_report_ms) >= MOTOR_REPORT_PERIOD_MS)
  {
    car.motor_report_ms += MOTOR_REPORT_PERIOD_MS;
    SendMotorTelemetry();
  }
}
