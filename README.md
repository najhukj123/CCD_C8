# CCD_C8

STM32F103C8 线性 CCD 循迹小车固件。

## 工程文件

- `MDK-ARM/CCD.uvprojx`：Keil 工程。
- `CCD.ioc`：STM32CubeMX 外设配置。
- `Core/`：启动入口和外设初始化。
- `MyCode/Hardware/`：CCD、编码器和电机的硬件接口。
- `MyCode/Algorithm/`：黑线识别和单轮 PID。
- `MyCode/Mod/`：整车循迹与双轮速度控制。

当前 CCD 读取 10 字节帧头后的全部 1500 个像素，并将像素左右反转。黑线识别使用 `MyCode/Mod/track.c` 中的 `TRACK_LINE_THRESHOLD` 固定阈值。主循环处理 CCD 新帧，并约每 20 ms 更新电机控制。
