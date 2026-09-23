"""Two-motor PID tuning console for the STM32 CCD car firmware."""

from __future__ import annotations

import csv
import queue
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少 pyserial，请先运行：py -m pip install -r requirements.txt") from exc

from protocol import BAUD_RATE, MotorPacketParser, MotorTelemetry


MODE_NAMES = {
    0: "停止",
    1: "电机1闭环",
    2: "电机2闭环",
    3: "循迹",
    4: "双电机闭环",
    5: "电机1开环",
    6: "电机2开环",
}


class SerialLink(threading.Thread):
    def __init__(
        self,
        port: str,
        output: queue.Queue[MotorTelemetry],
        errors: queue.Queue[str],
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.output = output
        self.errors = errors
        self.stop_event = threading.Event()
        self.serial_port: serial.Serial | None = None
        self.parser = MotorPacketParser()
        self.write_lock = threading.Lock()

    def run(self) -> None:
        try:
            self.serial_port = serial.Serial(self.port, BAUD_RATE, timeout=0.05)
            self.serial_port.reset_input_buffer()
            self.serial_port.write(b"STREAM,MOTOR\n")
            self.serial_port.flush()
            while not self.stop_event.is_set():
                chunk = self.serial_port.read(4096)
                for packet in self.parser.feed(chunk):
                    try:
                        self.output.put_nowait(packet)
                    except queue.Full:
                        try:
                            self.output.get_nowait()
                        except queue.Empty:
                            pass
                        self.output.put_nowait(packet)
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
            self.errors.put(str(exc))
            return False

    def stop(self) -> None:
        self.send("STOP")
        self.stop_event.set()


@dataclass
class MotorFields:
    cpr: tk.StringVar
    kp: tk.StringVar
    ki: tk.StringVar
    kd: tk.StringVar
    target: tk.StringVar
    live: tk.StringVar


class MotorPidTuner(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("双电机 PID 调试")
        self.geometry("1120x760")
        self.minsize(900, 650)

        self.telemetry: queue.Queue[MotorTelemetry] = queue.Queue(maxsize=100)
        self.errors: queue.Queue[str] = queue.Queue()
        self.link: SerialLink | None = None
        self.port_lookup: dict[str, str] = {}
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="未连接")
        self.mode_var = tk.StringVar(value="模式：未知")
        self.last_packet_time = 0.0
        self.last_keepalive_time = 0.0
        self.history: deque[tuple[float, MotorTelemetry]] = deque(maxlen=600)
        self.started_at = time.monotonic()

        self.motor1 = MotorFields(
            tk.StringVar(value="1456"),
            tk.StringVar(value="0.81"),
            tk.StringVar(value="0.059"),
            tk.StringVar(value="0.15"),
            tk.StringVar(value="80"),
            tk.StringVar(value="等待遥测"),
        )
        self.motor2 = MotorFields(
            tk.StringVar(value="1509"),
            tk.StringVar(value="0.85"),
            tk.StringVar(value="0.05"),
            tk.StringVar(value="0.00"),
            tk.StringVar(value="80"),
            tk.StringVar(value="等待遥测"),
        )

        self._build_ui()
        self.refresh_ports()
        self.after(25, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        toolbar = ttk.Frame(self, padding=10)
        toolbar.pack(fill=tk.X)
        ttk.Label(toolbar, text="串口").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(toolbar, textvariable=self.port_var, width=26, state="readonly")
        self.port_box.pack(side=tk.LEFT, padx=6)
        ttk.Button(toolbar, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_button = ttk.Button(toolbar, text="连接", command=self.toggle_connection)
        self.connect_button.pack(side=tk.LEFT, padx=8)
        ttk.Button(toolbar, text="急停", command=self.stop_motors, style="Emergency.TButton").pack(side=tk.LEFT, padx=12)
        ttk.Button(toolbar, text="保存记录 CSV", command=self.save_csv).pack(side=tk.RIGHT)
        ttk.Label(toolbar, textvariable=self.mode_var).pack(side=tk.RIGHT, padx=15)
        style = ttk.Style(self)
        style.configure("Emergency.TButton", foreground="#b00020")

        controls = ttk.Frame(self, padding=(10, 0, 10, 8))
        controls.pack(fill=tk.X)
        self._motor_panel(controls, 1, self.motor1).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self._motor_panel(controls, 2, self.motor2).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))

        dual = ttk.LabelFrame(self, text="双轮同时闭环", padding=8)
        dual.pack(fill=tk.X, padx=10, pady=(0, 8))
        ttk.Label(dual, text="使用上面两侧目标转速").pack(side=tk.LEFT)
        ttk.Button(dual, text="同时启动", command=self.start_dual).pack(side=tk.LEFT, padx=12)
        ttk.Button(dual, text="停止", command=self.stop_motors).pack(side=tk.LEFT)
        ttk.Label(
            dual,
            text="顺序：先分别用10%开环确认方向/编码器符号，再分别闭环，最后同时启动",
        ).pack(side=tk.RIGHT)

        graph_frame = ttk.Frame(self, padding=(10, 0, 10, 0))
        graph_frame.pack(fill=tk.BOTH, expand=True)
        self.canvas1 = tk.Canvas(graph_frame, background="#101820", highlightthickness=1, highlightbackground="#6f7882")
        self.canvas2 = tk.Canvas(graph_frame, background="#101820", highlightthickness=1, highlightbackground="#6f7882")
        self.canvas1.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        self.canvas2.pack(fill=tk.BOTH, expand=True)
        self.canvas1.bind("<Configure>", lambda _event: self.redraw())
        self.canvas2.bind("<Configure>", lambda _event: self.redraw())

        status = ttk.Frame(self, padding=10)
        status.pack(fill=tk.X)
        ttk.Label(status, textvariable=self.status_var).pack(side=tk.LEFT)
        ttk.Label(status, text="蓝=目标RPM  绿=实际RPM  黄=PWM%（按±360刻度显示）").pack(side=tk.RIGHT)

    def _motor_panel(self, parent: ttk.Frame, index: int, fields: MotorFields) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent, text=f"电机{index}", padding=8)
        names = (("CPR", fields.cpr), ("Kp", fields.kp), ("Ki", fields.ki), ("Kd", fields.kd), ("目标RPM", fields.target))
        for column, (label, variable) in enumerate(names):
            ttk.Label(frame, text=label).grid(row=0, column=column, padx=3, sticky="w")
            ttk.Entry(frame, textvariable=variable, width=9).grid(row=1, column=column, padx=3)
        ttk.Button(frame, text="应用参数", command=lambda: self.apply_parameters(index, fields)).grid(row=1, column=5, padx=7)
        ttk.Button(frame, text="10%开环", command=lambda: self.open_loop(index, 10)).grid(row=1, column=6, padx=3)
        ttk.Button(frame, text="单独闭环", command=lambda: self.start_single(index, fields)).grid(row=1, column=7, padx=3)
        ttk.Label(frame, textvariable=fields.live).grid(row=2, column=0, columnspan=8, pady=(8, 0), sticky="w")
        return frame

    def refresh_ports(self) -> None:
        ports = sorted(list_ports.comports(), key=lambda p: ("USB-SERIAL" not in p.description.upper(), p.device))
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
            messagebox.showwarning("没有串口", "请选择 USB-TTL 串口。")
            return
        self.link = SerialLink(port, self.telemetry, self.errors)
        self.link.start()
        self.last_keepalive_time = time.monotonic()
        self.connect_button.configure(text="断开")
        self.port_box.configure(state="disabled")
        self.status_var.set(f"已连接 {port} · {BAUD_RATE} 8N1，等待电机遥测")

    def disconnect(self) -> None:
        if self.link is not None:
            self.link.stop()
            self.link.join(timeout=0.3)
            self.link = None
        self.connect_button.configure(text="连接")
        self.port_box.configure(state="readonly")
        self.status_var.set("未连接（已发送急停）")

    def send(self, command: str) -> bool:
        if self.link is None or not self.link.send(command):
            messagebox.showwarning("未连接", "请先连接 USB-TTL 串口。")
            return False
        self.status_var.set(f"已发送：{command}")
        return True

    @staticmethod
    def _number(value: tk.StringVar, name: str) -> float:
        try:
            return float(value.get())
        except ValueError as exc:
            raise ValueError(f"{name} 不是有效数字") from exc

    def apply_parameters(self, index: int, fields: MotorFields) -> None:
        try:
            cpr = self._number(fields.cpr, "CPR")
            kp = self._number(fields.kp, "Kp")
            ki = self._number(fields.ki, "Ki")
            kd = self._number(fields.kd, "Kd")
            if cpr < 1 or min(kp, ki, kd) < 0:
                raise ValueError("CPR必须大于0，PID参数不能为负数")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        for command in (f"M{index}CPR,{cpr:g}", f"M{index}KP,{kp:g}", f"M{index}KI,{ki:g}", f"M{index}KD,{kd:g}"):
            if not self.send(command):
                break

    def open_loop(self, index: int, pwm: float) -> None:
        self.send(f"M{index}PWM,{pwm:g}")

    def start_single(self, index: int, fields: MotorFields) -> None:
        try:
            target = self._number(fields.target, "目标转速")
            if abs(target) > 360:
                raise ValueError("目标转速必须在 -360 到 360 rpm 之间")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        self.send(f"M{index}S,{target:g}")

    def start_dual(self) -> None:
        try:
            target1 = self._number(self.motor1.target, "电机1目标转速")
            target2 = self._number(self.motor2.target, "电机2目标转速")
            if max(abs(target1), abs(target2)) > 360:
                raise ValueError("目标转速必须在 -360 到 360 rpm 之间")
        except ValueError as exc:
            messagebox.showerror("参数错误", str(exc))
            return
        self.send(f"DUAL,{target1:g},{target2:g}")

    def stop_motors(self) -> None:
        self.send("STOP")

    def _poll(self) -> None:
        now = time.monotonic()
        if self.link is not None and now - self.last_keepalive_time >= 0.25:
            self.link.send("KEEP")
            self.last_keepalive_time = now

        latest: MotorTelemetry | None = None
        try:
            while True:
                latest = self.telemetry.get_nowait()
                self.history.append((time.monotonic() - self.started_at, latest))
        except queue.Empty:
            pass
        if latest is not None:
            self.last_packet_time = time.monotonic()
            self.mode_var.set(f"模式：{MODE_NAMES.get(latest.mode, str(latest.mode))}")
            self.motor1.live.set(
                f"目标 {latest.motor1_target:+.1f}  实际 {latest.motor1_actual:+.1f}  "
                f"原始 {latest.motor1_raw:+.1f} rpm  PWM {latest.motor1_output:+.1f}%"
            )
            self.motor2.live.set(
                f"目标 {latest.motor2_target:+.1f}  实际 {latest.motor2_actual:+.1f}  "
                f"原始 {latest.motor2_raw:+.1f} rpm  PWM {latest.motor2_output:+.1f}%"
            )
            self.status_var.set(
                f"遥测正常 · 序号 {latest.sequence} · 电机1 {'就绪' if latest.ready_flags & 1 else '未就绪'} · "
                f"电机2 {'就绪' if latest.ready_flags & 2 else '未就绪'}"
            )
            self.redraw()
        try:
            error = self.errors.get_nowait()
        except queue.Empty:
            error = None
        if error is not None:
            self.disconnect()
            messagebox.showerror("串口错误", error)
        if self.link is not None and self.last_packet_time and time.monotonic() - self.last_packet_time > 1.0:
            self.status_var.set("串口已连接，但超过1秒没有收到电机遥测；请确认已烧录新版固件")
        self.after(25, self._poll)

    def redraw(self) -> None:
        self._draw_motor(self.canvas1, 1)
        self._draw_motor(self.canvas2, 2)

    def _draw_motor(self, canvas: tk.Canvas, index: int) -> None:
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 2), max(canvas.winfo_height(), 2)
        left, top, right, bottom = 55, 22, width - 12, height - 24
        canvas.create_text(left, 5, text=f"电机{index}", fill="#dbe5ee", anchor=tk.NW)
        for rpm in (-360, -180, 0, 180, 360):
            y = bottom - (rpm + 360) / 720 * (bottom - top)
            canvas.create_line(left, y, right, y, fill="#293640")
            canvas.create_text(left - 8, y, text=str(rpm), fill="#9aa7b2", anchor=tk.E)
        if len(self.history) < 2:
            canvas.create_text(width / 2, height / 2, text="等待遥测", fill="#9aa7b2")
            return
        rows = list(self.history)
        t0, t1 = rows[0][0], rows[-1][0]
        span = max(t1 - t0, 0.1)

        def points(selector) -> list[float]:
            result: list[float] = []
            for timestamp, packet in rows:
                value = max(-360.0, min(360.0, selector(packet)))
                x = left + (timestamp - t0) / span * (right - left)
                y = bottom - (value + 360.0) / 720.0 * (bottom - top)
                result.extend((x, y))
            return result

        if index == 1:
            target = lambda p: p.motor1_target
            actual = lambda p: p.motor1_actual
            output = lambda p: p.motor1_output * 3.6
        else:
            target = lambda p: p.motor2_target
            actual = lambda p: p.motor2_actual
            output = lambda p: p.motor2_output * 3.6
        canvas.create_line(*points(target), fill="#42a5f5", width=2)
        canvas.create_line(*points(actual), fill="#66d17a", width=2)
        canvas.create_line(*points(output), fill="#ffd166", width=1)

    def save_csv(self) -> None:
        if not self.history:
            messagebox.showinfo("没有数据", "收到遥测后才能保存。")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=(("CSV", "*.csv"),))
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(("time_s", "m1_target", "m1_actual", "m1_raw", "m1_pwm", "m2_target", "m2_actual", "m2_raw", "m2_pwm", "mode"))
            for timestamp, p in self.history:
                writer.writerow((timestamp, p.motor1_target, p.motor1_actual, p.motor1_raw, p.motor1_output, p.motor2_target, p.motor2_actual, p.motor2_raw, p.motor2_output, p.mode))

    def _on_close(self) -> None:
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    MotorPidTuner().mainloop()
