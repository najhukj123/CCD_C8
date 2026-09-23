"""Linear CCD serial viewer for the STM32 firmware in this project."""

from __future__ import annotations

import csv
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from protocol import BAUD_RATE, Frame, PacketParser
from track import TrackFollower, TrackingResult

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - shown to the end user
    raise SystemExit("缺少 pyserial，请先运行：py -m pip install -r requirements.txt") from exc


class SerialReader(threading.Thread):
    def __init__(self, port: str, output: queue.Queue[Frame], errors: queue.Queue[str]) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.output = output
        self.errors = errors
        self.stop_event = threading.Event()
        self.serial_port: serial.Serial | None = None
        self.parser = PacketParser()

    def run(self) -> None:
        try:
            self.serial_port = serial.Serial(self.port, BAUD_RATE, timeout=0.1)
            self.serial_port.reset_input_buffer()
            self.serial_port.write(b"STREAM,CCD\n")
            self.serial_port.flush()
            while not self.stop_event.is_set():
                chunk = self.serial_port.read(4096)
                if not chunk:
                    continue
                for frame in self.parser.feed(chunk):
                    try:
                        self.output.put_nowait(frame)
                    except queue.Full:
                        try:
                            self.output.get_nowait()
                        except queue.Empty:
                            pass
                        self.output.put_nowait(frame)
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.errors.put(str(exc))
        finally:
            if self.serial_port is not None and self.serial_port.is_open:
                self.serial_port.close()

    def stop(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                self.serial_port.write(b"STREAM,MOTOR\n")
                self.serial_port.flush()
            except (serial.SerialException, OSError):
                pass
        self.stop_event.set()


class CCDViewer(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("线性 CCD 波形与二值化")
        self.geometry("1180x760")
        self.minsize(820, 560)

        self.frames: queue.Queue[Frame] = queue.Queue(maxsize=4)
        self.errors: queue.Queue[str] = queue.Queue()
        self.reader: SerialReader | None = None
        self.port_lookup: dict[str, str] = {}
        self.current_pixels: bytes | None = None
        self.current_tracking: TrackingResult | None = None
        self.tracker = TrackFollower()
        self.last_sequence: int | None = None
        self.received_count = 0
        self.dropped_count = 0
        self.rate_count = 0
        self.rate_started = time.monotonic()
        self.raw_plot_size: tuple[int, int] | None = None
        self.binary_plot_size: tuple[int, int] | None = None
        self.raw_wave_item: int | None = None
        self.raw_threshold_item: int | None = None
        self.raw_stats_item: int | None = None
        self.raw_sensor_center_item: int | None = None
        self.raw_track_left_item: int | None = None
        self.raw_track_right_item: int | None = None
        self.raw_track_center_item: int | None = None
        self.binary_wave_item: int | None = None

        self.port_var = tk.StringVar()
        self.threshold_var = tk.IntVar(value=128)
        self.invert_var = tk.BooleanVar(value=False)
        self.line_mode_var = tk.StringVar(value="黑线")
        self.status_var = tk.StringVar(value="未连接")
        self.stats_var = tk.StringVar(value="等待数据")

        self._build_ui()
        self.refresh_ports()
        self.after(25, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=10)
        toolbar.pack(fill=tk.X)

        ttk.Label(toolbar, text="串口").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(toolbar, textvariable=self.port_var, width=18, state="readonly")
        self.port_box.pack(side=tk.LEFT, padx=(6, 8))
        ttk.Button(toolbar, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(toolbar, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=8)
        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(toolbar, text="二值化阈值").pack(side=tk.LEFT)
        threshold = ttk.Scale(
            toolbar,
            from_=0,
            to=255,
            variable=self.threshold_var,
            command=self._threshold_changed,
            length=220,
        )
        threshold.pack(side=tk.LEFT, padx=(8, 4))
        self.threshold_label = ttk.Label(toolbar, text="128", width=4)
        self.threshold_label.pack(side=tk.LEFT)
        ttk.Checkbutton(toolbar, text="反相", variable=self.invert_var, command=self.redraw).pack(side=tk.LEFT, padx=8)
        ttk.Label(toolbar, text="赛道").pack(side=tk.LEFT, padx=(6, 4))
        line_mode = ttk.Combobox(
            toolbar,
            textvariable=self.line_mode_var,
            values=("黑线", "白线"),
            width=5,
            state="readonly",
        )
        line_mode.pack(side=tk.LEFT)
        line_mode.bind("<<ComboboxSelected>>", lambda _event: self._line_mode_changed())
        ttk.Button(toolbar, text="当前设为中心", command=self.calibrate_center).pack(side=tk.LEFT, padx=8)
        ttk.Button(toolbar, text="保存当前帧 CSV", command=self.save_csv).pack(side=tk.RIGHT)

        plots = ttk.Frame(self, padding=(10, 0, 10, 8))
        plots.pack(fill=tk.BOTH, expand=True)
        ttk.Label(plots, text="原始灰度波形  0 黑  255 白").pack(anchor=tk.W)
        self.raw_canvas = tk.Canvas(plots, background="#101820", highlightthickness=1, highlightbackground="#6f7882")
        self.raw_canvas.pack(fill=tk.BOTH, expand=True, pady=(4, 12))
        ttk.Label(plots, text="二值化结果").pack(anchor=tk.W)
        self.binary_canvas = tk.Canvas(plots, height=150, background="#101820", highlightthickness=1, highlightbackground="#6f7882")
        self.binary_canvas.pack(fill=tk.X, pady=(4, 0))

        status = ttk.Frame(self, padding=(10, 6, 10, 10))
        status.pack(fill=tk.X)
        ttk.Label(status, textvariable=self.status_var).pack(side=tk.LEFT)
        ttk.Label(status, textvariable=self.stats_var).pack(side=tk.RIGHT)

        self.raw_canvas.bind("<Configure>", lambda _event: self.redraw())
        self.binary_canvas.bind("<Configure>", lambda _event: self.redraw())

    def refresh_ports(self) -> None:
        ports = sorted(
            list_ports.comports(),
            key=lambda item: (
                0
                if any(
                    token in f"{item.description} {item.hwid}".upper()
                    for token in ("CH340", "CP210", "FTDI", "USB-SERIAL")
                )
                else 1
                if "USB" in item.hwid.upper()
                else 2,
                item.device,
            ),
        )
        self.port_lookup = {
            f"{port.device} — {port.description}": port.device
            for port in ports
        }
        labels = list(self.port_lookup)
        self.port_box["values"] = labels
        if labels and self.port_var.get() not in labels:
            self.port_var.set(labels[0])
        if not labels:
            self.port_var.set("")

    def toggle_connection(self) -> None:
        if self.reader is not None:
            self.disconnect()
            return
        port = self.port_lookup.get(self.port_var.get(), "")
        if not port:
            messagebox.showwarning("没有串口", "请连接 USB-TTL 后点击刷新。")
            return
        self.reader = SerialReader(port, self.frames, self.errors)
        self.reader.start()
        self.connect_button.configure(text="断开")
        self.port_box.configure(state="disabled")
        self.status_var.set(f"已连接 {port}  ·  {BAUD_RATE} 8N1")
        self.received_count = self.dropped_count = self.rate_count = 0
        self.last_sequence = None
        self.current_tracking = None
        self.tracker.reset_tracking()
        self.rate_started = time.monotonic()

    def disconnect(self) -> None:
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.connect_button.configure(text="连接")
        self.port_box.configure(state="readonly")
        self.status_var.set("未连接")

    def _poll(self) -> None:
        latest: Frame | None = None
        try:
            while True:
                latest = self.frames.get_nowait()
        except queue.Empty:
            pass

        if latest is not None:
            if self.last_sequence is not None:
                gap = (latest.sequence - self.last_sequence - 1) & 0xFFFF
                if gap < 0x8000:
                    self.dropped_count += gap
            self.last_sequence = latest.sequence
            self.current_pixels = latest.pixels
            self.current_tracking = self.tracker.update(
                latest.pixels,
                self.threshold_var.get(),
                dark_line=self.line_mode_var.get() == "黑线",
            )
            self.received_count += 1
            self.rate_count += 1
            self.redraw()

        try:
            error = self.errors.get_nowait()
        except queue.Empty:
            error = None
        if error is not None:
            self.disconnect()
            messagebox.showerror("串口错误", error)

        elapsed = time.monotonic() - self.rate_started
        if elapsed >= 1.0:
            fps = self.rate_count / elapsed
            self.rate_count = 0
            self.rate_started = time.monotonic()
            crc_errors = self.reader.parser.crc_errors if self.reader is not None else 0
            self.stats_var.set(
                f"{fps:.1f} 帧/秒  ·  已收 {self.received_count}  ·  丢帧 {self.dropped_count}  ·  CRC错误 {crc_errors}"
            )

        self.after(25, self._poll)

    def _threshold_changed(self, value: str) -> None:
        threshold = int(float(value))
        self.threshold_var.set(threshold)
        self.threshold_label.configure(text=str(threshold))
        self._restart_tracking()
        self.redraw()

    def _line_mode_changed(self) -> None:
        self._restart_tracking()
        self.redraw()

    def _restart_tracking(self) -> None:
        self.tracker.reset_tracking()
        if self.current_pixels is not None:
            self.current_tracking = self.tracker.update(
                self.current_pixels,
                self.threshold_var.get(),
                dark_line=self.line_mode_var.get() == "黑线",
            )

    def calibrate_center(self) -> None:
        if not self.tracker.calibrate_zero():
            messagebox.showinfo("无法标定", "请先让程序稳定识别到赛道。")
            return
        if self.current_pixels is not None:
            self.current_tracking = self.tracker.update(
                self.current_pixels,
                self.threshold_var.get(),
                dark_line=self.line_mode_var.get() == "黑线",
            )
        self.redraw()

    def redraw(self) -> None:
        if self.current_pixels is None:
            self._draw_empty(self.raw_canvas, "等待 STM32 数据")
            self._draw_empty(self.binary_canvas, "等待 STM32 数据")
            return
        self._draw_raw(self.current_pixels)
        self._draw_binary(self.current_pixels)

    @staticmethod
    def _draw_empty(canvas: tk.Canvas, text: str) -> None:
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        existing = canvas.find_withtag("empty_message")
        if existing:
            canvas.coords(existing[0], width / 2, height / 2)
            canvas.itemconfigure(existing[0], text=text)
        else:
            canvas.create_text(
                width / 2,
                height / 2,
                text=text,
                fill="#aab4be",
                font=("Microsoft YaHei UI", 12),
                tags=("empty_message",),
            )

    def _draw_raw(self, pixels: bytes) -> None:
        canvas = self.raw_canvas
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        left, top, right, bottom = 48, 12, width - 12, height - 28

        size = (width, height)
        if self.raw_plot_size != size or self.raw_wave_item is None:
            canvas.delete("all")
            for value in (0, 64, 128, 192, 255):
                y = bottom - value / 255 * (bottom - top)
                canvas.create_line(left, y, right, y, fill="#293640")
                canvas.create_text(left - 8, y, text=str(value), fill="#9aa7b2", anchor=tk.E)
            for value in (0, 250, 500, 750, 999):
                x = left + value / 999 * (right - left)
                canvas.create_line(x, top, x, bottom, fill="#293640")
                canvas.create_text(x, bottom + 14, text=str(value), fill="#9aa7b2")
            self.raw_threshold_item = canvas.create_line(
                left, top, right, top, fill="#ff6b6b", dash=(5, 4)
            )
            self.raw_wave_item = canvas.create_line(
                left, bottom, right, bottom, fill="#42a5f5", width=1.5
            )
            self.raw_stats_item = canvas.create_text(
                right, top, text="", fill="#dbe5ee", anchor=tk.NE
            )
            sensor_x = left + 0.5 * (right - left)
            self.raw_sensor_center_item = canvas.create_line(
                sensor_x, top, sensor_x, bottom, fill="#dbe5ee", dash=(3, 5)
            )
            self.raw_track_left_item = canvas.create_line(
                left, top, left, bottom, fill="#66d17a", width=2, state=tk.HIDDEN
            )
            self.raw_track_right_item = canvas.create_line(
                right, top, right, bottom, fill="#66d17a", width=2, state=tk.HIDDEN
            )
            self.raw_track_center_item = canvas.create_line(
                sensor_x, top, sensor_x, bottom, fill="#ffd166", width=3, state=tk.HIDDEN
            )
            self.raw_plot_size = size

        threshold = self.threshold_var.get()
        threshold_y = bottom - threshold / 255 * (bottom - top)
        canvas.coords(self.raw_threshold_item, left, threshold_y, right, threshold_y)

        points: list[float] = []
        for index, value in enumerate(pixels):
            points.extend((left + index / 999 * (right - left), bottom - value / 255 * (bottom - top)))
        canvas.coords(self.raw_wave_item, *points)

        minimum, maximum = min(pixels), max(pixels)
        mean = sum(pixels) / len(pixels)
        tracking = self.current_tracking
        if tracking is None:
            tracking = self.tracker.update(
                pixels,
                threshold,
                dark_line=self.line_mode_var.get() == "黑线",
            )
            self.current_tracking = tracking
        detection = tracking.detection
        if tracking.state == "LOST":
            track_text = f"LOST  丢线 {tracking.lost_frames} 帧"
            for item in (self.raw_track_left_item, self.raw_track_right_item, self.raw_track_center_item):
                canvas.itemconfigure(item, state=tk.HIDDEN)
        else:
            if detection is None:
                canvas.itemconfigure(self.raw_track_left_item, state=tk.HIDDEN)
                canvas.itemconfigure(self.raw_track_right_item, state=tk.HIDDEN)
                positions = ((self.raw_track_center_item, tracking.filtered_center),)
                track_text = f"HOLD {tracking.lost_frames}/{self.tracker.hold_frames}   偏差 {tracking.error:+.1f}"
            else:
                positions = (
                    (self.raw_track_left_item, detection.left),
                    (self.raw_track_right_item, detection.right),
                    (self.raw_track_center_item, tracking.filtered_center),
                )
                track_text = (
                    f"TRACKING  线宽 {detection.width}   原始 {detection.center:.1f}   "
                    f"滤波 {tracking.filtered_center:.1f}   偏差 {tracking.error:+.1f}"
                )
            for item, position in positions:
                x = left + position / 999 * (right - left)
                canvas.coords(item, x, top, x, bottom)
                canvas.itemconfigure(item, state=tk.NORMAL)
        canvas.coords(self.raw_stats_item, right, top)
        canvas.itemconfigure(
            self.raw_stats_item,
            text=f"min {minimum}   max {maximum}   mean {mean:.1f}   {track_text}",
        )

    def _draw_binary(self, pixels: bytes) -> None:
        canvas = self.binary_canvas
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        left, top, right, bottom = 48, 10, width - 12, height - 24
        threshold = self.threshold_var.get()
        invert = self.invert_var.get()

        size = (width, height)
        if self.binary_plot_size != size or self.binary_wave_item is None:
            canvas.delete("all")
            self.binary_wave_item = canvas.create_line(
                left, bottom, right, bottom, fill="#66d17a", width=1.5
            )
            canvas.create_text(left - 8, top, text="1", fill="#9aa7b2", anchor=tk.E)
            canvas.create_text(left - 8, bottom, text="0", fill="#9aa7b2", anchor=tk.E)
            canvas.create_line(left, top, left, bottom, fill="#66727c")
            canvas.create_line(left, bottom, right, bottom, fill="#66727c")
            self.binary_plot_size = size

        points: list[float] = []
        for index, value in enumerate(pixels):
            state = value >= threshold
            if invert:
                state = not state
            y = top if state else bottom
            points.extend((left + index / 999 * (right - left), y))
        canvas.coords(self.binary_wave_item, *points)

    def save_csv(self) -> None:
        if self.current_pixels is None:
            messagebox.showinfo("没有数据", "收到一帧数据后才能保存。")
            return
        path = filedialog.asksaveasfilename(
            title="保存当前 CCD 帧",
            defaultextension=".csv",
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
        )
        if not path:
            return
        threshold = self.threshold_var.get()
        invert = self.invert_var.get()
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(("pixel", "gray", "binary"))
            for index, value in enumerate(self.current_pixels):
                state = int(value >= threshold)
                if invert:
                    state = 1 - state
                writer.writerow((index, value, state))

    def _on_close(self) -> None:
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    CCDViewer().mainloop()
