"""Bluetooth dashboard for CCD waveform, tracking state and car controls."""

from __future__ import annotations

import csv
import math
import queue
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少 pyserial，请先运行：py -m pip install -r requirements.txt") from exc

from protocol import BAUD_RATE, Frame, MotorPacketParser, MotorTelemetry, PacketParser
from plotjuggler_bridge import TelemetryRecorder, find_plotjuggler, launch_plotjuggler
from track import automatic_threshold, detect_track


TRACK_STATES = {0: "LOST", 1: "HOLD", 2: "TRACKING"}
MODE_NAMES = {0: "停止", 1: "右轮闭环", 2: "左轮闭环", 3: "自动循迹", 4: "双轮闭环", 5: "右轮开环", 6: "左轮开环", 7: "已到停车线"}


@dataclass
class EventRecord:
    timestamp: str
    kind: str
    detail: str
    ccd_sequence: int | None
    pixels: bytes | None


@dataclass
class PendingCenterCalibration:
    sequence: int
    sensor_center: float
    prior_error: float
    started_at: float
    observed_packets: int = 0


class BluetoothLink(threading.Thread):
    def __init__(
        self,
        port: str,
        frames: queue.Queue[Frame],
        telemetry: queue.Queue[MotorTelemetry],
        errors: queue.Queue[str],
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.frames = frames
        self.telemetry = telemetry
        self.errors = errors
        self.stop_event = threading.Event()
        self.serial_port: serial.Serial | None = None
        self.frame_parser = PacketParser()
        self.motor_parser = MotorPacketParser()
        self.write_lock = threading.Lock()

    @staticmethod
    def _put_latest(output: queue.Queue, value: object) -> None:
        try:
            output.put_nowait(value)
        except queue.Full:
            try:
                output.get_nowait()
            except queue.Empty:
                pass
            output.put_nowait(value)

    def run(self) -> None:
        try:
            self.serial_port = serial.Serial(self.port, BAUD_RATE, timeout=0.05)
            self.serial_port.reset_input_buffer()
            self.send("STOP")
            self.send("STREAM,BOTH")
            while not self.stop_event.is_set():
                chunk = self.serial_port.read(4096)
                if not chunk:
                    continue
                for frame in self.frame_parser.feed(chunk):
                    self._put_latest(self.frames, frame)
                for packet in self.motor_parser.feed(chunk):
                    self._put_latest(self.telemetry, packet)
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.errors.put(str(exc))
        finally:
            if self.serial_port is not None and self.serial_port.is_open:
                self.serial_port.close()

    def send(self, command: str) -> bool:
        port = self.serial_port
        if port is None or not port.is_open:
            return False
        try:
            with self.write_lock:
                port.write((command.strip() + "\n").encode("ascii"))
                port.flush()
            return True
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.errors.put(str(exc))
            return False

    def stop(self) -> None:
        self.send("STOP")
        self.stop_event.set()


class CarDebugConsole(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("线性 CCD 蓝牙循迹调试台")
        self.geometry("1420x900")
        self.minsize(1080, 720)

        self.frames: queue.Queue[Frame] = queue.Queue(maxsize=5)
        self.telemetry: queue.Queue[MotorTelemetry] = queue.Queue(maxsize=30)
        self.errors: queue.Queue[str] = queue.Queue()
        self.link: BluetoothLink | None = None
        self.port_lookup: dict[str, str] = {}
        self.current_frame: Frame | None = None
        self.current_motor: MotorTelemetry | None = None
        self.last_motor_received_at = 0.0
        self.pending_center_calibration: PendingCenterCalibration | None = None
        self.frozen_frame: Frame | None = None
        self.frozen_motor: MotorTelemetry | None = None
        self.events: list[EventRecord] = []
        self.previous_track_state: int | None = None
        self.last_large_error_log = 0.0
        self.last_keepalive = 0.0
        self.frame_count = 0
        self.frame_rate_count = 0
        self.frame_rate_started = time.monotonic()
        self.last_frame_sequence: int | None = None
        self.dropped_frames = 0
        self.recorder: TelemetryRecorder | None = None
        self.last_recording: Path | None = None

        self.port_var = tk.StringVar()
        self.stream_var = tk.StringVar(value="同时")
        self.threshold_var = tk.StringVar(value="0")
        self.line_mode_var = tk.StringVar(value="黑线")
        self.base_rpm_var = tk.StringVar(value="60")
        self.steer_kp_var = tk.StringVar(value="0.35")
        self.steer_kd_var = tk.StringVar(value="0.20")
        self.max_steer_var = tk.StringVar(value="120")
        self.right_trim_var = tk.StringVar(value="0.996")
        self.left_trim_var = tk.StringVar(value="1.004")
        self.freeze_var = tk.BooleanVar(value=False)
        self.calibration_var = tk.StringVar(value="")
        self.connection_var = tk.StringVar(value="未连接")
        self.track_var = tk.StringVar(value="等待循迹状态")
        self.motor_var = tk.StringVar(value="等待电机遥测")
        self.stats_var = tk.StringVar(value="等待 CCD 波形")
        self.recording_var = tk.StringVar(
            value="PlotJuggler 4 已就绪" if find_plotjuggler() else "未找到 PlotJuggler 4"
        )

        self.raw_size: tuple[int, int] | None = None
        self.binary_size: tuple[int, int] | None = None
        self.raw_wave: int | None = None
        self.raw_threshold: int | None = None
        self.raw_info: int | None = None
        self.raw_sensor_center: int | None = None
        self.raw_calibrated_center: int | None = None
        self.raw_left: int | None = None
        self.raw_right: int | None = None
        self.raw_center: int | None = None
        self.binary_wave: int | None = None

        self._build_ui()
        self.refresh_ports()
        self.after(25, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=8)
        toolbar.pack(fill=tk.X)
        ttk.Label(toolbar, text="蓝牙串口").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(toolbar, textvariable=self.port_var, width=28, state="readonly")
        self.port_box.pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(toolbar, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="急停", command=self.stop_car).pack(side=tk.LEFT, padx=(8, 16))
        ttk.Label(toolbar, text="数据流").pack(side=tk.LEFT)
        stream_box = ttk.Combobox(
            toolbar, textvariable=self.stream_var, values=("电机", "CCD", "同时", "关闭"), width=7, state="readonly"
        )
        stream_box.pack(side=tk.LEFT, padx=5)
        stream_box.bind("<<ComboboxSelected>>", lambda _event: self.apply_stream())
        ttk.Checkbutton(toolbar, text="丢线时冻结波形", variable=self.freeze_var, command=self.on_freeze_toggle).pack(side=tk.LEFT, padx=12)
        ttk.Button(toolbar, text="继续实时", command=self.resume_live).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="保存当前帧", command=self.save_current_frame).pack(side=tk.LEFT, padx=6)
        ttk.Label(toolbar, textvariable=self.connection_var).pack(side=tk.RIGHT)

        analysis = ttk.Frame(self, padding=(8, 0, 8, 6))
        analysis.pack(fill=tk.X)
        ttk.Label(analysis, text="整圈数据分析").pack(side=tk.LEFT)
        self.record_button = ttk.Button(analysis, text="开始整圈记录", command=self.toggle_recording)
        self.record_button.pack(side=tk.LEFT, padx=(8, 5))
        ttk.Button(analysis, text="打开最近记录", command=self.open_last_recording).pack(side=tk.LEFT)
        ttk.Label(analysis, textvariable=self.recording_var).pack(side=tk.LEFT, padx=12)

        parameters = ttk.LabelFrame(self, text="直接写入 STM32（本次上电有效）", padding=8)
        parameters.pack(fill=tk.X, padx=8, pady=(0, 6))
        fields = (
            ("阈值(0自动)", self.threshold_var, 8),
            ("基础RPM", self.base_rpm_var, 7),
            ("转向Kp", self.steer_kp_var, 7),
            ("转向Kd", self.steer_kd_var, 7),
            ("最大转向RPM", self.max_steer_var, 7),
            ("右轮补偿", self.right_trim_var, 7),
            ("左轮补偿", self.left_trim_var, 7),
        )
        for column, (label, variable, width) in enumerate(fields):
            ttk.Label(parameters, text=label).grid(row=0, column=column, padx=3, sticky="w")
            ttk.Entry(parameters, textvariable=variable, width=width).grid(row=1, column=column, padx=3)
        ttk.Label(parameters, text="赛道").grid(row=0, column=7, padx=3, sticky="w")
        ttk.Combobox(
            parameters, textvariable=self.line_mode_var, values=("黑线", "白线"), width=5, state="readonly"
        ).grid(row=1, column=7, padx=3)
        ttk.Button(parameters, text="应用全部参数", command=self.apply_parameters).grid(row=1, column=8, padx=8)
        ttk.Button(parameters, text="当前位置设为中心", command=self.calibrate_center).grid(row=1, column=9, padx=4)
        ttk.Button(parameters, text="启动自动循迹", command=self.start_auto).grid(row=1, column=10, padx=4)
        ttk.Button(parameters, text="直行测试", command=self.start_straight).grid(row=1, column=11, padx=4)

        state = ttk.Frame(self, padding=(10, 2, 10, 5))
        state.pack(fill=tk.X)
        ttk.Label(state, textvariable=self.track_var).pack(side=tk.LEFT)
        ttk.Label(state, textvariable=self.motor_var).pack(side=tk.RIGHT)
        ttk.Label(self, textvariable=self.calibration_var, foreground="#7c3aed", padding=(10, 0, 10, 4)).pack(anchor=tk.W)

        plots = ttk.Frame(self, padding=(8, 0, 8, 6))
        plots.pack(fill=tk.BOTH, expand=True)
        ttk.Label(plots, text="CCD 原始灰度（蓝） · 阈值（红虚线） · STM32边界（绿） · 本地候选（橙） · 像素中点（白虚线） · 校准零点（紫虚线）").pack(anchor=tk.W)
        self.raw_canvas = tk.Canvas(plots, background="#101820", highlightthickness=1, highlightbackground="#65727d")
        self.raw_canvas.pack(fill=tk.BOTH, expand=True, pady=(3, 8))
        ttk.Label(plots, text="有效赛道（二值化筛选后；两端阴影不计入）").pack(anchor=tk.W)
        self.binary_canvas = tk.Canvas(plots, height=125, background="#101820", highlightthickness=1, highlightbackground="#65727d")
        self.binary_canvas.pack(fill=tk.X, pady=(3, 0))
        self.raw_canvas.bind("<Configure>", lambda _event: self.redraw())
        self.binary_canvas.bind("<Configure>", lambda _event: self.redraw())

        lower = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        lower.pack(fill=tk.BOTH, padx=8, pady=(0, 8))
        event_box = ttk.LabelFrame(lower, text="识别事件记录", padding=5)
        controls = ttk.Frame(event_box)
        controls.pack(fill=tk.X)
        ttk.Label(controls, textvariable=self.stats_var).pack(side=tk.LEFT)
        ttk.Button(controls, text="保存事件与异常帧", command=self.save_events).pack(side=tk.RIGHT)
        ttk.Button(controls, text="清空", command=self.clear_events).pack(side=tk.RIGHT, padx=5)
        self.event_tree = ttk.Treeview(event_box, columns=("time", "kind", "detail"), show="headings", height=6)
        self.event_tree.heading("time", text="时间")
        self.event_tree.heading("kind", text="事件")
        self.event_tree.heading("detail", text="详情")
        self.event_tree.column("time", width=90, stretch=False)
        self.event_tree.column("kind", width=100, stretch=False)
        self.event_tree.column("detail", width=800)
        self.event_tree.pack(fill=tk.BOTH, expand=True)
        lower.add(event_box, weight=1)

    def refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda p: ("BLUETOOTH" not in (p.description or "").upper(), p.device))
        self.port_lookup = {f"{p.device} — {p.description}": p.device for p in ports}
        labels = list(self.port_lookup)
        self.port_box["values"] = labels
        if labels and self.port_var.get() not in labels:
            self.port_var.set(labels[0])

    def toggle_connection(self) -> None:
        if self.link is not None:
            self.disconnect()
            return
        port = self.port_lookup.get(self.port_var.get(), "")
        if not port:
            messagebox.showwarning("没有串口", "请先配对 HC-05，并选择它的输出 COM 口。")
            return
        self.link = BluetoothLink(port, self.frames, self.telemetry, self.errors)
        self.link.start()
        self.connect_button.configure(text="断开")
        self.port_box.configure(state="disabled")
        self.connection_var.set(f"正在连接 {port} · {BAUD_RATE} 8N1")
        self.last_keepalive = time.monotonic()
        self.previous_track_state = None

    def disconnect(self) -> None:
        if self.recorder is not None:
            self._finish_recording(open_plotter=False)
        link = self.link
        self.link = None
        self.pending_center_calibration = None
        self.last_motor_received_at = 0.0
        self.current_motor = None
        self.calibration_var.set("")
        if link is not None:
            link.stop()
            link.join(timeout=0.4)
        self.connect_button.configure(text="连接")
        self.port_box.configure(state="readonly")
        self.connection_var.set("未连接（已请求急停）")

    def send(self, command: str, *, quiet: bool = False) -> bool:
        if self.link is None or not self.link.send(command):
            if not quiet:
                messagebox.showwarning("未连接", "请先连接蓝牙串口。")
            return False
        if not quiet:
            self.connection_var.set(f"已发送：{command}")
        return True

    def calibrate_center(self) -> None:
        """Send CENTER only with live tracking data; verify using later telemetry."""
        if self.link is None:
            self.calibration_var.set("未连接：无法设置中心")
            return
        if self.stream_var.get() != "同时":
            if self.send("STREAM,BOTH", quiet=True):
                self.stream_var.set("同时")
                self.calibration_var.set("已切换为同时接收；等 TRACKING 更新后再点一次设置中心")
            else:
                self.calibration_var.set("无法开启电机遥测；中心未设置")
            return
        packet = self.current_motor
        now = time.monotonic()
        if (packet is None or not packet.tracking_available or
            packet.track_state != 2 or now - self.last_motor_received_at > 1.0):
            self.calibration_var.set("需要新鲜的 TRACKING 遥测；中心未设置")
            return
        if not self.send("CENTER", quiet=True):
            self.calibration_var.set("CENTER 发送失败；中心未设置")
            return
        self.pending_center_calibration = PendingCenterCalibration(
            packet.sequence, packet.filtered_center, packet.track_error, now
        )
        self.calibration_var.set("CENTER 已发出，等待 STM32 后续遥测确认…")

    def _observe_center_calibration(self, packet: MotorTelemetry, now: float) -> None:
        pending = self.pending_center_calibration
        if pending is None:
            return
        advance = (packet.sequence - pending.sequence) & 0xFFFF
        if advance == 0 or advance >= 0x8000:
            return
        if not packet.tracking_available or packet.track_state != 2:
            return
        pending.observed_packets += 1
        zero = packet.filtered_center - packet.track_error
        if (math.isfinite(zero) and abs(packet.track_error) <= 5.0 and
            abs(zero - pending.sensor_center) <= 15.0):
            if abs(pending.prior_error) <= 5.0:
                self.calibration_var.set("当前偏差接近 0；遥测无法单独证明本次 CENTER 指令已执行")
            else:
                self.calibration_var.set(
                    f"校准已生效（遥测确认）：零点 {zero:.1f}，当前偏差 {packet.track_error:+.1f}"
                )
            self.pending_center_calibration = None

    def _check_center_calibration_timeout(self, now: float) -> None:
        pending = self.pending_center_calibration
        if pending is None or now - pending.started_at < 2.0:
            return
        if pending.observed_packets:
            packet = self.current_motor
            error = packet.track_error if packet is not None else float("nan")
            self.calibration_var.set(f"中心未确认：新遥测的偏差仍为 {error:+.1f}；检查固件是否执行 CENTER")
        else:
            self.calibration_var.set("中心未确认：没有收到新的 TRACKING 遥测")
        self.pending_center_calibration = None

    def apply_stream(self) -> None:
        commands = {"电机": "STREAM,MOTOR", "CCD": "STREAM,CCD", "同时": "STREAM,BOTH", "关闭": "STREAM,OFF"}
        self.send(commands[self.stream_var.get()])

    @staticmethod
    def _float(variable: tk.Variable, label: str) -> float:
        try:
            return float(variable.get())
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label}不是有效数字") from exc

    def apply_parameters(self) -> bool:
        try:
            threshold = int(float(self.threshold_var.get()))
            base = self._float(self.base_rpm_var, "基础RPM")
            kp = self._float(self.steer_kp_var, "转向Kp")
            kd = self._float(self.steer_kd_var, "转向Kd")
            maximum = self._float(self.max_steer_var, "最大转向RPM")
            right_trim = self._float(self.right_trim_var, "右轮补偿")
            left_trim = self._float(self.left_trim_var, "左轮补偿")
            if not 0 <= threshold <= 255:
                raise ValueError("阈值必须在0到255之间")
            if not 0 <= abs(base) <= 360 or not 0 <= maximum <= 360:
                raise ValueError("转速必须在0到360rpm之间")
            if min(kp, kd) < 0 or not (0.8 <= right_trim <= 1.2 and 0.8 <= left_trim <= 1.2):
                raise ValueError("控制参数超出安全范围")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return False
        commands = (
            f"TH,{threshold}",
            f"DARK,{1 if self.line_mode_var.get() == '黑线' else 0}",
            f"STEERKP,{kp:g}",
            f"STEERKD,{kd:g}",
            f"MAXSTEER,{maximum:g}",
            f"TRIM,{right_trim:g},{left_trim:g}",
        )
        for command in commands:
            if not self.send(command, quiet=True):
                return False
        self.connection_var.set("参数已写入 STM32（本次上电有效）")
        self.redraw()
        return True

    def start_auto(self) -> None:
        if not self.apply_parameters():
            return
        try:
            base = self._float(self.base_rpm_var, "基础RPM")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        self.send(f"AUTO,{base:g}")

    def start_straight(self) -> None:
        if not self.apply_parameters():
            return
        try:
            base = self._float(self.base_rpm_var, "基础RPM")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        # STM32 applies rightTrim/leftTrim inside its motor-control loop.
        self.send(f"DUAL,{base:g},{base:g}")

    def stop_car(self) -> None:
        self.send("STOP")

    def resume_live(self) -> None:
        self.frozen_frame = None
        self.frozen_motor = None
        self.redraw()

    def on_freeze_toggle(self) -> None:
        if not self.freeze_var.get():
            self.resume_live()

    def toggle_recording(self) -> None:
        if self.recorder is not None:
            self._finish_recording(open_plotter=True)
            return
        if self.link is None:
            messagebox.showwarning("未连接", "请先连接蓝牙串口，再开始整圈记录。")
            return
        log_folder = Path(__file__).resolve().parent / "logs"
        self.recorder = TelemetryRecorder.start_in(log_folder)
        self.last_recording = self.recorder.path
        self.record_button.configure(text="停止记录并分析")
        self.recording_var.set(f"正在记录：{self.recorder.path.name}")
        if self.stream_var.get() != "同时":
            self.stream_var.set("同时")
            self.apply_stream()

    def _finish_recording(self, *, open_plotter: bool) -> None:
        recorder = self.recorder
        if recorder is None:
            return
        self.recorder = None
        path = recorder.finish()
        self.last_recording = path
        self.record_button.configure(text="开始整圈记录")
        self.recording_var.set(f"已保存 {recorder.samples} 点：{path.name}")
        if not open_plotter:
            return
        if recorder.samples == 0:
            messagebox.showinfo("没有遥测", "本次记录没有收到 MTR2 数据，CSV 已保留但不打开分析。")
            return
        self._launch_plotjuggler(path)

    def open_last_recording(self) -> None:
        path = self.last_recording
        if path is None or not path.is_file():
            logs = Path(__file__).resolve().parent / "logs"
            recordings = sorted(logs.glob("ccd_car_run_*.csv"), key=lambda item: item.stat().st_mtime, reverse=True)
            path = recordings[0] if recordings else None
        if path is None:
            messagebox.showinfo("没有记录", "先完成一次整圈记录。")
            return
        self._launch_plotjuggler(path)

    def _launch_plotjuggler(self, path: Path) -> None:
        try:
            launch_plotjuggler(path)
            self.recording_var.set(f"已交给 PlotJuggler：{path.name}")
        except (FileNotFoundError, OSError) as exc:
            messagebox.showerror("无法打开 PlotJuggler", str(exc))

    def _poll(self) -> None:
        now = time.monotonic()
        if self.link is not None and now - self.last_keepalive >= 0.25:
            self.send("KEEP", quiet=True)
            self.last_keepalive = now

        latest_frame: Frame | None = None
        motor_packets: list[MotorTelemetry] = []
        try:
            while True:
                latest_frame = self.frames.get_nowait()
        except queue.Empty:
            pass
        try:
            while True:
                motor_packets.append(self.telemetry.get_nowait())
        except queue.Empty:
            pass

        if latest_frame is not None:
            if self.last_frame_sequence is not None:
                gap = (latest_frame.sequence - self.last_frame_sequence - 1) & 0xFFFF
                if gap < 0x8000:
                    self.dropped_frames += gap
            self.last_frame_sequence = latest_frame.sequence
            self.current_frame = latest_frame
            self.frame_count += 1
            self.frame_rate_count += 1
            if self.frozen_frame is None:
                self.redraw()

        if self.recorder is not None:
            frame_crc = self.link.frame_parser.crc_errors if self.link else 0
            motor_crc = self.link.motor_parser.crc_errors if self.link else 0
            for packet in motor_packets:
                self.recorder.append(
                    packet,
                    self.current_frame,
                    dropped_frames=self.dropped_frames,
                    ccd_crc_errors=frame_crc,
                    motor_crc_errors=motor_crc,
                )

        latest_motor = motor_packets[-1] if motor_packets else None
        if latest_motor is not None:
            self.current_motor = latest_motor
            self.last_motor_received_at = now
            for packet in motor_packets:
                self._observe_center_calibration(packet, now)
            self._update_telemetry(latest_motor)
            self._check_events(latest_motor)
            self.redraw()
        self._check_center_calibration_timeout(now)

        try:
            error = self.errors.get_nowait()
        except queue.Empty:
            error = None
        if error is not None:
            self.disconnect()
            messagebox.showerror("蓝牙串口错误", error)

        elapsed = now - self.frame_rate_started
        if elapsed >= 1.0:
            fps = self.frame_rate_count / elapsed
            self.frame_rate_count = 0
            self.frame_rate_started = now
            frame_crc = self.link.frame_parser.crc_errors if self.link else 0
            motor_crc = self.link.motor_parser.crc_errors if self.link else 0
            freeze = " · 已冻结异常帧" if self.frozen_frame is not None else ""
            self.stats_var.set(
                f"CCD {fps:.1f}帧/s · 收到{self.frame_count} · 序号跳过{self.dropped_frames} · CRC {frame_crc + motor_crc}{freeze}"
            )
            if self.recorder is not None:
                self.recording_var.set(f"正在记录：{self.recorder.samples} 点 · {self.recorder.path.name}")
        self.after(25, self._poll)

    def _update_telemetry(self, packet: MotorTelemetry) -> None:
        state = TRACK_STATES.get(packet.track_state, str(packet.track_state))
        mode = MODE_NAMES.get(packet.mode, str(packet.mode))
        self.connection_var.set(f"蓝牙正常 · {BAUD_RATE} 8N1 · 模式：{mode}")
        if packet.tracking_available:
            zero = packet.filtered_center - packet.track_error
            parking = ""
            if packet.mode == 3 and packet.ready_flags & 0x80:
                parking_state = (packet.ready_flags >> 2) & 0x03
                parking = ("等起点", "驶离起点", "等终点", "未知状态")[parking_state] + " · "
            wave_note = " · 下方波形已冻结" if self.frozen_frame is not None else ""
            self.track_var.set(
                f"{parking}{state}{wave_note}  边界 {packet.track_left}~{packet.track_right}  线宽 {packet.track_width}  "
                f"中心 {packet.filtered_center:.1f}  零点 {zero:.1f}  "
                f"偏差 {packet.track_error:+.1f}  转向 {packet.steering:+.1f}rpm"
            )
        else:
            self.track_var.set("当前固件未回传循迹字段 · 绿色边界由上位机本地计算")
        self.motor_var.set(
            f"右轮 {packet.motor1_actual:.1f}/{packet.motor1_target:.1f}rpm PWM {packet.motor1_output:.1f}%  ·  "
            f"左轮 {packet.motor2_actual:.1f}/{packet.motor2_target:.1f}rpm PWM {packet.motor2_output:.1f}%"
        )

    def _check_events(self, packet: MotorTelemetry) -> None:
        if not packet.tracking_available:
            return
        state = packet.track_state
        previous = self.previous_track_state
        if previous is not None and state != previous:
            name = f"{TRACK_STATES.get(previous, previous)}→{TRACK_STATES.get(state, state)}"
            detail = f"误差{packet.track_error:+.1f} 线宽{packet.track_width} 丢线{packet.lost_frames}帧"
            self._add_event(name, detail, freeze=state in (0, 1))
        self.previous_track_state = state

        now = time.monotonic()
        if abs(packet.track_error) >= 200 and now - self.last_large_error_log >= 1.0:
            self.last_large_error_log = now
            self._add_event("大偏差", f"偏差{packet.track_error:+.1f} 转向{packet.steering:+.1f}rpm", freeze=False)
        if state == 2 and (packet.track_width < 20 or packet.track_width > 350) and now - self.last_large_error_log >= 1.0:
            self.last_large_error_log = now
            self._add_event("异常线宽", f"线宽{packet.track_width} 边界{packet.track_left}~{packet.track_right}", freeze=False)

    def _add_event(self, kind: str, detail: str, *, freeze: bool) -> None:
        timestamp = time.strftime("%H:%M:%S")
        frame = self.current_frame
        record = EventRecord(timestamp, kind, detail, frame.sequence if frame else None, frame.pixels if frame else None)
        self.events.append(record)
        self.event_tree.insert("", 0, values=(timestamp, kind, detail))
        if freeze and self.freeze_var.get() and frame is not None:
            self.frozen_frame = frame
            self.frozen_motor = self.current_motor

    def clear_events(self) -> None:
        self.events.clear()
        for item in self.event_tree.get_children():
            self.event_tree.delete(item)

    def save_events(self) -> None:
        if not self.events:
            messagebox.showinfo("没有事件", "尚未记录丢线或识别异常。")
            return
        folder = filedialog.askdirectory(title="选择事件记录保存目录")
        if not folder:
            return
        base = Path(folder)
        with (base / "track_events.csv").open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(("time", "event", "detail", "ccd_sequence"))
            for event in self.events:
                writer.writerow((event.timestamp, event.kind, event.detail, event.ccd_sequence))
        for index, event in enumerate(self.events):
            if event.pixels is None:
                continue
            safe_time = event.timestamp.replace(":", "-")
            with (base / f"event_{index:03d}_{safe_time}.csv").open("w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.writer(handle)
                writer.writerow(("pixel", "gray"))
                writer.writerows(enumerate(event.pixels))
        messagebox.showinfo("保存完成", f"已保存{len(self.events)}条事件及对应波形。")

    def save_current_frame(self) -> None:
        frame = self.current_frame
        if frame is None:
            messagebox.showinfo("没有数据", "收到 CCD 波形后才能保存。")
            return
        path = filedialog.asksaveasfilename(
            title="保存当前 CCD 波形",
            defaultextension=".csv",
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
        )
        if not path:
            return
        threshold = self._effective_threshold(frame.pixels, live=True)
        boundary = self._visible_boundary(frame.pixels, threshold, live=True)
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(("pixel", "gray", "active"))
            for index, value in enumerate(frame.pixels):
                active = boundary is not None and boundary[0] <= index <= boundary[1]
                writer.writerow((index, value, int(active)))

    def redraw(self) -> None:
        frame = self.frozen_frame or self.current_frame
        if frame is None:
            return
        self._draw_raw(frame.pixels)
        self._draw_binary(frame.pixels)

    def _effective_threshold(self, pixels: bytes, *, live: bool = False) -> int:
        """Use STM32's actual threshold when its telemetry is available."""
        packet = self.current_motor if live else self.frozen_motor or self.current_motor
        if packet is not None and packet.tracking_available and packet.threshold > 0:
            return packet.threshold
        try:
            configured = int(float(self.threshold_var.get()))
        except (TypeError, ValueError):
            configured = 0
        if configured > 0:
            return max(0, min(255, configured))
        return automatic_threshold(pixels)

    def _visible_boundary(self, pixels: bytes, threshold: int, *, live: bool = False):
        """Show STM32's result, or label a local candidate when STM32 is LOST."""
        packet = self.current_motor if live else self.frozen_motor or self.current_motor
        if packet is not None and packet.tracking_available:
            if (packet.track_state == 2 and
                0 <= packet.track_left < packet.track_right < len(pixels)):
                return packet.track_left, packet.track_right, packet.filtered_center, "STM32"
        reference_center = None
        if packet is not None and packet.tracking_available:
            reference_center = packet.filtered_center - packet.track_error
        detection = detect_track(
            pixels,
            threshold,
            dark_line=self.line_mode_var.get() == "黑线",
            minimum_width=20,
            maximum_width=350,
            reference_center=reference_center,
            roi_start=30,
            roi_end=969,
        )
        if detection is None:
            return None
        source = "本地候选（STM32未识别）" if packet is not None and packet.tracking_available else "本地"
        return detection.left, detection.right, detection.center, source

    def _draw_raw(self, pixels: bytes) -> None:
        canvas = self.raw_canvas
        width, height = max(canvas.winfo_width(), 2), max(canvas.winfo_height(), 2)
        left, top, right, bottom = 45, 12, width - 12, height - 25
        if self.raw_size != (width, height) or self.raw_wave is None:
            canvas.delete("all")
            for value in (0, 64, 128, 192, 255):
                y = bottom - value / 255 * (bottom - top)
                canvas.create_line(left, y, right, y, fill="#293640")
                canvas.create_text(left - 7, y, text=str(value), fill="#9aa7b2", anchor=tk.E)
            self.raw_threshold = canvas.create_line(left, top, right, top, fill="#ff6b6b", dash=(6, 4))
            self.raw_wave = canvas.create_line(left, bottom, right, bottom, fill="#42a5f5", width=1.5)
            self.raw_info = canvas.create_text(right, top, text="", fill="#dbe5ee", anchor=tk.NE)
            mid = left + (right - left) / 2
            self.raw_sensor_center = canvas.create_line(mid, top, mid, bottom, fill="#dbe5ee", dash=(3, 5))
            self.raw_calibrated_center = canvas.create_line(
                mid, top, mid, bottom, fill="#bf86ff", dash=(7, 4), width=2, state=tk.HIDDEN
            )
            self.raw_left = canvas.create_line(left, top, left, bottom, fill="#66d17a", width=2)
            self.raw_right = canvas.create_line(right, top, right, bottom, fill="#66d17a", width=2)
            self.raw_center = canvas.create_line(mid, top, mid, bottom, fill="#ffd166", width=3)
            self.raw_size = (width, height)
        threshold = self._effective_threshold(pixels)
        threshold_y = bottom - threshold / 255 * (bottom - top)
        canvas.coords(self.raw_threshold, left, threshold_y, right, threshold_y)
        points: list[float] = []
        for index, value in enumerate(pixels):
            points.extend((left + index / 999 * (right - left), bottom - value / 255 * (bottom - top)))
        canvas.coords(self.raw_wave, *points)
        packet = self.frozen_motor or self.current_motor
        if packet is not None and packet.tracking_available:
            calibrated_zero = packet.filtered_center - packet.track_error
            if math.isfinite(calibrated_zero) and 0 <= calibrated_zero < len(pixels):
                zero_x = left + calibrated_zero / 999 * (right - left)
                canvas.coords(self.raw_calibrated_center, zero_x, top, zero_x, bottom)
                canvas.itemconfigure(self.raw_calibrated_center, state=tk.NORMAL)
            else:
                canvas.itemconfigure(self.raw_calibrated_center, state=tk.HIDDEN)
        else:
            canvas.itemconfigure(self.raw_calibrated_center, state=tk.HIDDEN)
        boundary = self._visible_boundary(pixels, threshold)
        if boundary is not None:
            color = "#ffae42" if boundary[3] != "STM32" else "#66d17a"
            for item, position in zip(
                (self.raw_left, self.raw_right, self.raw_center), boundary[:3]
            ):
                x = left + position / 999 * (right - left)
                canvas.coords(item, x, top, x, bottom)
                canvas.itemconfigure(item, fill=color, state=tk.NORMAL)
        else:
            for item in (self.raw_left, self.raw_right, self.raw_center):
                canvas.itemconfigure(item, state=tk.HIDDEN)
        boundary_text = "未识别" if boundary is None else f"{boundary[3]}边界 {boundary[0]}~{boundary[1]}"
        canvas.itemconfigure(
            self.raw_info,
            text=(
                f"min {min(pixels)}  max {max(pixels)}  mean {sum(pixels)/len(pixels):.1f}  "
                f"阈值 {threshold}  {boundary_text}"
            ),
        )

    def _draw_binary(self, pixels: bytes) -> None:
        canvas = self.binary_canvas
        width, height = max(canvas.winfo_width(), 2), max(canvas.winfo_height(), 2)
        left, top, right, bottom = 45, 10, width - 12, height - 20
        if self.binary_size != (width, height) or self.binary_wave is None:
            canvas.delete("all")
            canvas.create_line(left, top, left, bottom, fill="#66727c")
            canvas.create_line(left, bottom, right, bottom, fill="#66727c")
            self.binary_wave = canvas.create_line(left, bottom, right, bottom, fill="#66d17a", width=1.5)
            self.binary_size = (width, height)
        boundary = self._visible_boundary(pixels, self._effective_threshold(pixels))
        canvas.itemconfigure(self.binary_wave, fill="#ffae42" if boundary is not None and boundary[3] != "STM32" else "#66d17a")
        points: list[float] = []
        for index in range(len(pixels)):
            active = boundary is not None and boundary[0] <= index <= boundary[1]
            points.extend((left + index / 999 * (right - left), top if active else bottom))
        canvas.coords(self.binary_wave, *points)

    def _on_close(self) -> None:
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = CarDebugConsole()
    if len(sys.argv) > 1:
        requested_port = sys.argv[1].upper()
        for label, port in app.port_lookup.items():
            if port.upper() == requested_port:
                app.port_var.set(label)
                app.after(100, app.toggle_connection)
                break
    app.mainloop()
