#include "_line_tracker.h"

static float AbsFloat(float value)
{
  return value < 0.0f ? -value : value;
}

static void MarkLineMissing(LineTracker *tracker)
{
  // HOLD 保留上一次位置和误差；连续丢线超过允许帧数才进入 LOST。
  tracker->width = 0U;
  if (tracker->lost_frames < 255U)
    ++tracker->lost_frames;
  tracker->state = tracker->state != LINE_LOST &&
      tracker->lost_frames <= tracker->config.hold_frames ?
      LINE_HOLD : LINE_LOST;
}

static uint8_t AutomaticThreshold(const uint8_t *pixels,
                                  uint16_t first, uint16_t last,
                                  uint8_t *contrast_ok)
{
  uint16_t histogram[256] = {0};
  uint16_t index;
  uint32_t cumulative = 0U;
  uint32_t pixel_count = (uint32_t)last - first + 1U;
  uint8_t low = 0U;
  uint8_t high = 255U;

  for (index = first; index <= last; ++index)
    ++histogram[pixels[index]];

  // 用灰度的 2% 和 80% 分位数估计黑、白背景，避开孤立噪点。
  for (index = 0U; index < 256U; ++index)
  {
    cumulative += histogram[index];
    if (cumulative >= pixel_count * 2U / 100U)
    {
      low = (uint8_t)index;
      break;
    }
  }

  cumulative = 0U;
  for (index = 0U; index < 256U; ++index)
  {
    cumulative += histogram[index];
    if (cumulative >= pixel_count * 80U / 100U)
    {
      high = (uint8_t)index;
      break;
    }
  }

  // 整帧灰度近似一致时，不能把地面或曝光不足误判为赛道。
  *contrast_ok = high >= (uint16_t)low + 8U;
  return (uint8_t)(((uint16_t)low + high) / 2U);
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
  tracker->threshold_used = config->threshold;
  tracker->lost_frames = 0U;
}

void LineTracker_CenterHere(LineTracker *tracker)
{
  if (tracker->state != LINE_LOST)
  {
    tracker->zero_center = tracker->filtered_center;
    tracker->error = 0.0f;
  }
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
  uint8_t contrast_ok = 1U;
  // 标定后，搜索范围要跟着零点走；固定在像素中点会丢掉偏装的真实黑线。
  float search_center = tracker->zero_center;

  if (config->threshold == 0U)
    tracker->threshold_used = AutomaticThreshold(pixels, first, last,
                                                  &contrast_ok);
  else
    tracker->threshold_used = config->threshold;

  if (!contrast_ok)
  {
    MarkLineMissing(tracker);
    return;
  }

  // 从左向右找第一个跨过阈值的线像素，作为左边界。
  // 边缘暗区先跳过；至少看到 3 个背景像素后才接受边界。
  for (index = first; index <= last; ++index)
  {
    uint8_t active = config->dark_line ?
        (pixels[index] < tracker->threshold_used) :
        (pixels[index] >= tracker->threshold_used);
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
    uint8_t active = config->dark_line ?
        (pixels[index] < tracker->threshold_used) :
        (pixels[index] >= tracker->threshold_used);
    if (!active)
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
    MarkLineMissing(tracker);
    return;
  }

  // 连续跟踪期间，突然跳到远处的阴影属于可疑帧，不更新赛道中心。
  {
    uint16_t width = best_right - best_left + 1U;
    float center = ((float)best_left + best_right) * 0.5f;
    if (width < config->min_width || width > config->max_width ||
        AbsFloat(center - search_center) > config->max_center_offset)
    {
      MarkLineMissing(tracker);
      return;
    }
    if (tracker->state != LINE_LOST &&
        AbsFloat(center - tracker->filtered_center) > config->max_jump)
    {
      MarkLineMissing(tracker);
      return;
    }

    tracker->left = best_left;
    tracker->right = best_right;
    tracker->width = width;
    tracker->raw_center = center;
    if (tracker->state == LINE_LOST)
      tracker->filtered_center = center;
    else
      tracker->filtered_center += config->filter_alpha *
                                  (center - tracker->filtered_center);
  }

  tracker->error = tracker->filtered_center - tracker->zero_center;
  tracker->lost_frames = 0U;
  tracker->state = LINE_TRACKING;
}
