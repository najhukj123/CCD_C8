#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdint.h>

typedef enum { LINE_LOST = 0, LINE_HOLD = 1, LINE_TRACKING = 2 } LineState;

// 线识别参数。像素位置均为镜像后的车体坐标系。
typedef struct
{
  uint16_t pixel_count;       // 每帧像素数。
  uint16_t roi_margin;        // 左右两端各排除的像素数。
  uint16_t min_width;         // 有效赛道的最小宽度。
  uint16_t max_width;         // 有效赛道的最大宽度。
  float max_center_offset;    // 候选中心到物理中点的最大距离。
  float max_jump;             // 相邻有效帧的中心最大跳变。
  float filter_alpha;         // 中心位置一阶滤波系数。
  uint8_t hold_frames;        // 临时丢线时允许沿用旧偏差的帧数。
  uint8_t dark_line;          // 1 识别黑线，0 识别白线。
  uint8_t threshold;          // 0 表示由本帧灰度直方图自动计算阈值。
} LineConfig;

// 每次输入新帧后更新；HOLD 时保留上次误差，LOST 时由应用层停止循迹。
typedef struct
{
  LineConfig config;
  LineState state;
  uint16_t left;              // 最近一次有效线的左边界。
  uint16_t right;             // 最近一次有效线的右边界。
  uint16_t width;             // 当前有效线宽；没有识别到时为 0。
  float zero_center;          // 标定零点，初始为传感器物理中点。
  float raw_center;           // 最近一次有效线的原始中心。
  float filtered_center;      // 滤波后的中心。
  float error;                // 滤波中心减去标定零点。
  uint8_t threshold_used;     // 当前帧实际采用的阈值。
  uint8_t lost_frames;        // 连续未识别到有效线的帧数。
} LineTracker;

// 初始化识别器并复制参数，之后参数可通过 tracker->config 调整。
void LineTracker_Init(LineTracker *tracker, const LineConfig *config);
// 处理一帧 pixel_count 个像素，更新状态和边界。
void LineTracker_Update(LineTracker *tracker, const uint8_t *pixels);
// 当前有有效线或处于短暂保持状态时，把滤波中心设为零点。
void LineTracker_CenterHere(LineTracker *tracker);

#endif
