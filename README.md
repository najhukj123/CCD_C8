# CCD 小车：从哪里开始读代码

这是一个 STM32F103C8 小车工程。线性 CCD 提供 1000 个灰度像素；固件从中找赛道，计算左右轮目标转速，再用编码器反馈调节两路 PWM。电脑端通过串口发送命令并显示数据。当前固件和电脑端统一使用 **921600 8N1**。

## 建议阅读顺序

1. `Core/Src/main.c`：上电后初始化硬件，然后不断调用 `CarApp_Poll()`。
2. `Core/Src/_car_app.c`：整车调度、命令、自动循迹转向。先读 `CarApp_Init()` 和 `CarApp_Poll()`，再读 `UpdateTracking()`。
3. `Core/Src/_ccd_sensor.c`：CCD 数据就绪后，从 SPI 读取一帧像素。
4. `Core/Src/_line_tracker.c`：从像素里找线，得到中心和偏差。
5. `Core/Src/_drive_control.c`：管理左右轮，连接编码器、单轮控制器和 PWM。
6. `Core/Src/_motor_controller.c`：一个轮子的测速、滤波和速度 PID。
7. `Core/Src/_car_serial.c`：接收文字命令，发送 CCD 和电机二进制报文。

`Core/Inc/` 中同名的 `.h` 文件说明各模块对外提供的函数和数据类型。`gpio.c`、`tim.c`、`spi.c`、`usart.c` 等文件主要是 CubeMX 生成的硬件配置。

## 一帧数据怎样让小车转弯

```text
CCD 模块告知“新帧到了”
  → CcdSensor_ReadFrame() 从 SPI 读取像素
  → LineTracker_Update() 找左右边界和线中心
  → UpdateTracking() 用中心偏差计算转向量
  → DriveControl_SetRequested() 设置左右轮目标 RPM
  → DriveControl_Update() 读编码器并更新两轮 PID
  → WriteWheel() 把 PID 输出写到方向引脚和 PWM
```

找线时，固件选出合适的连续黑色或白色区域，中心取 `(左边界 + 右边界) / 2`。转向使用 **PD**：`转向量 = Kp × 当前偏差 + Kd × (当前偏差 - 上次偏差)`。右轮目标是 `基础RPM - 转向量`，左轮目标是 `基础RPM + 转向量`。每个车轮再单独用速度 **PID** 跟随自己的目标 RPM。

## 椭圆赛道的停车线

横向黑色停车线会让 CCD 同时看到左右两侧和大部分中间区域变黑，原来的窄线识别会把它当作无效宽线。现在 `_car_app.c` 单独检查这种宽黑带：有效视野至少 70% 为黑色，左右各 1/5 区域至少 60% 为黑色，连续出现 2 帧才确认。阈值沿用最近一次正常循迹的结果，因为全黑画面无法单独计算可靠的自动阈值。

每次发送 `AUTO,速度` 后，第一次经过停车线记为起点；小车沿进入停车线前的方向继续驶过。离开停车线连续 3 帧后，第二次经过同一条线，且距第一次至少 3 秒，才清零电机目标并停车。综合调试台依次显示“等起点”“驶离起点”“等终点”，停车后显示“已到停车线”。发送 `STOP` 仍可随时停车，再次发送 `AUTO,速度` 会重新开始计数。

**摆车方法：**把车放在停车线前方、CCD 能先看到普通窄循迹线的位置，再启动自动循迹。这样第一次经过停车线是起跑，绕一圈返回时才停车。停车线在车速下必须至少连续出现在 2 帧 CCD 中；电机断电后还会滑行，车身未必恰好停在线上。若从停车线后方起跑，程序无法凭单个 CCD 判断已经绕了几圈。黑色区域覆盖整个视野的阴影或遮挡也可能被误认为停车线；正式跑前请根据实测 CSV 调整 `_car_app.c` 中的 70%、60% 和连续帧数。

思路参考：[CCD 巡线小车的灰度边缘识别研究](https://yhjcjs.spacejournal.cn/article/doi/10.12060/j.issn.1000-7202.2019.05.13)和[卡内基梅隆大学的停车线跨帧跟踪研究](https://publications.ri.cmu.edu/a-vision-system-for-detection-and-tracking-of-stop-lines)。本车只有一维 CCD，因此“宽黑带”与遮挡的区分能力有限；70%、60% 和帧数是待实测调整的工程参数，不是文献给出的固定值。

## 哪些结构体真正保存状态

| 名称 | 作用 | 保存在哪里 |
| --- | --- | --- |
| `CarState car` | 行驶模式、循迹参数和任务时间 | `_car_app.c` |
| `LineTracker` | 当前线的位置、偏差与丢线状态 | `car.line` |
| `WheelState wheels[2]` | 右轮、左轮各自的目标、补偿和编码器上次读数 | `_drive_control.c` |
| `MotorController` | 单轮实际速度与 PID 记忆 | 每个 `WheelState` 内 |
| `CcdSensorState sensor` | CCD 接收缓冲和新帧标志 | `_ccd_sensor.c` |
| `CarSerialState serial_state` | 串口收发缓冲和命令队列 | `_car_serial.c` |

`DriveStatus` 和 `CarMotorTelemetry` 只在准备遥测报文时临时使用。应用层把 `DriveStatus` 填入 `CarMotorTelemetry.drive`，然后交给串口打包；它们借用上述状态的指针，并没有再创建一套电机控制器。

## 读 C 代码时会遇到的符号

- `car.line`：取结构体 `car` 中的 `line` 字段。
- `&car.line`：取得这个字段的地址，交给需要修改它的函数。
- `line->error`：`line` 是指针，读取它指向的结构体中的 `error`；含义相当于 `(*line).error`。
- `static` 放在文件级变量或函数前：只有当前 `.c` 文件能直接访问它。
- `HAL_GPIO_EXTI_Callback()`、`HAL_UART_RxCpltCallback()`：硬件事件经 HAL 转交给这些回调。回调只记事件或收字节，主要工作在主循环进行。

## 常用命令和参数位置

| 命令 | 作用 |
| --- | --- |
| `AUTO,60` | 以 60 RPM 基础速度启动自动循迹 |
| `DUAL,60,60` | 手动要求右轮、左轮各 60 RPM |
| `STOP` | 停车并清除排队的旧行驶命令 |
| `KEEP` | 告诉固件电脑端仍在线；运行时超过 750 ms 未收到会停车 |
| `CENTER` | 将当前识别到的赛道中心设为零点 |
| `TH,0` | 设置自动阈值；非零值表示手动阈值 |
| `DARK,1` | 识别黑线；`DARK,0` 识别白线 |
| `STEERKP,...`、`STEERKD,...` | 修改循迹转向 PD 参数 |
| `MAXSTEER,...`、`TRIM,...` | 修改最大转向量、左右轮补偿 |
| `STREAM,BOTH` | 同时上传 CCD 和电机数据 |

找线的默认参数在 `_car_app.c` 的 `line_config`，转向 PD 默认参数在 `CarApp_Init()`，单轮速度 PID 参数在 `_drive_control.c` 的 `motor_configs`。电脑端综合调试界面入口是 `PC_App/run_car_debug.bat`。
