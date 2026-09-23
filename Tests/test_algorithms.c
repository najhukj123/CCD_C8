#include "_line_tracker.h"
#include "_motor_controller.h"
#include <assert.h>
#include <stdio.h>

static void TestLine(void)
{
  uint8_t pixels[1000];
  LineTracker line;
  LineConfig cfg = {1000U, 30U, 20U, 350U, 250.0f, 180.0f,
                    0.28f, 3U, 1U, 0U};
  unsigned i;
  LineTracker_Init(&line, &cfg);
  for (i = 0U; i < 1000U; ++i) pixels[i] = 60U;
  for (i = 440U; i <= 559U; ++i) pixels[i] = 18U;
  /* 两端“黑色区域”靠近视野外沿，不应覆盖真正的中央黑线。 */
  for (i = 0U; i <= 100U; ++i) pixels[i] = 18U;
  for (i = 900U; i < 1000U; ++i) pixels[i] = 18U;
  LineTracker_Update(&line, pixels);
  assert(line.state == LINE_TRACKING);
  assert(line.left == 440U && line.right == 559U);
  assert(line.error == 0.0f);
  assert(line.threshold_used > 18U && line.threshold_used < 60U);
  /* 连续丢线 3 帧 HOLD，第四帧 LOST，必须停止自动循迹。 */
  for (i = 0U; i < 1000U; ++i) pixels[i] = 60U;
  for (i = 0U; i < 3U; ++i)
  {
    LineTracker_Update(&line, pixels);
    assert(line.state == LINE_HOLD && line.width == 0U);
  }
  LineTracker_Update(&line, pixels);
  assert(line.state == LINE_LOST);
}

static void TestMotor(void)
{
  MotorController motor;
  MotorConfig cfg = {1456.0f, 0.020f, 0.293f, 0.81f, 0.059f,
                     0.15f, 140.0f, 100.0f, 360.0f};
  MotorController_Init(&motor, &cfg);
  MotorController_SetTarget(&motor, 60.0f);
  assert(MotorController_Update(&motor, 0) > 0.0f);
  MotorController_SetTarget(&motor, 0.0f);
  assert(MotorController_Update(&motor, 0) == 0.0f);
  assert(motor.integral == 0.0f);
}

int main(void)
{
  TestLine();
  TestMotor();
  puts("algorithm tests passed");
  return 0;
}
