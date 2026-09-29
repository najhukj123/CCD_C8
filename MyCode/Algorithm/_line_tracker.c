#include "_line_tracker.h"

static float AbsFloat(float value)
{
  return value < 0.0f ? -value : value;
}

void LineTracker_Init(LineTracker *tracker, const LineConfig *config)
{
  tracker->config = *config;
  tracker->state = LINE_LOST;
  tracker->left = 0U;
  tracker->right = 0U;
  tracker->width = 0U;
  tracker->zero_center = ((float)config->pixel_count - 1.0f) * 0.5f;
  tracker->raw_center = tracker->zero_center;
  tracker->filtered_center = tracker->zero_center;
  tracker->error = 0.0f;
}

void LineTracker_Update(LineTracker *tracker, const uint8_t *pixels)
{
  const LineConfig *config = &tracker->config;
  uint16_t first = config->roi_margin;
  uint16_t last = config->pixel_count - first - 1U;
  uint16_t index;
  uint16_t best_left = 0U;
  uint16_t best_right = 0U;
  uint8_t left_found = 0U;
  uint8_t right_found = 0U;
  uint8_t background_pixels = 0U;

  // 从左向右找第一个跨过阈值的线像素，作为左边界。
  // 边缘暗区先跳过；至少看到 3 个背景像素后才接受边界。
  for (index = first; index <= last; ++index)
  {
    uint8_t active = config->dark_line ?
        (pixels[index] < config->threshold) :
        (pixels[index] >= config->threshold);
    if (!active)
    {
      if (background_pixels < 3U) ++background_pixels;
    }
    else if (background_pixels >= 3U)
    {
      best_left = index;
      left_found = 1U;
      break;
    }
  }

  // 从右向左同样找第一个线像素，作为右边界。
  background_pixels = 0U;
  for (index = last; index > first; --index)
  {
    //小于阈值为0，大于阈值为1
    uint8_t active = config->dark_line ?
        (pixels[index] < config->threshold) :
        (pixels[index] >= config->threshold);
    if (active == 0U)
    {
      if (background_pixels < 3U) ++background_pixels;
    }
    else if (background_pixels >= 3U)
    {
      best_right = index;
      right_found = 1U;
      break;
    }
  }

  if (!left_found || !right_found || best_right <= best_left)
  {
    tracker->width = 0U;
    tracker->state = LINE_LOST;
    return;
  }

  // 连续跟踪期间，突然跳到远处的阴影属于可疑帧，不更新赛道中心。
  {
    uint16_t width = best_right - best_left + 1U;
    //赛道中心为左右相加取中值
    float center = ((float)best_left + best_right) * 0.5f;
    //线太宽，太窄，离画面中心太远都不行
    if (width < config->min_width || width > config->max_width ||
        AbsFloat(center - tracker->zero_center) > config->max_center_offset)
    {
      tracker->width = 0U;
      tracker->state = LINE_LOST;
      return;
    }

    // 检查连续帧中心跳变是否过大，属于可疑帧。
    if (tracker->state == LINE_TRACKING &&
        AbsFloat(center - tracker->filtered_center) > config->max_jump)
    {
      tracker->width = 0U;
      tracker->state = LINE_LOST;
      return;
    }

    tracker->left = best_left;
    tracker->right = best_right;
    tracker->width = width;
    tracker->raw_center = center;
    //丢线了直接采用新的
    if (tracker->state == LINE_LOST)
      tracker->filtered_center = center;
    else
    //没丢线就使用滤波进行平滑的更新
      tracker->filtered_center += config->filter_alpha *
                                  (center - tracker->filtered_center);
  }

  tracker->error = tracker->filtered_center - tracker->zero_center;
  tracker->state = LINE_TRACKING;
}
