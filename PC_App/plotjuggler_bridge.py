"""Record car telemetry and open it in PlotJuggler 4 with a ready layout."""

from __future__ import annotations

import csv
import html
import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Callable

from protocol import Frame, MotorTelemetry


CSV_COLUMNS = (
    "time_s",
    "wall_time_unix_s",
    "mode",
    "track_state",
    "tracking_ok",
    "holding_line",
    "line_lost",
    "threshold",
    "dark_line",
    "lost_frames",
    "line_left_px",
    "line_right_px",
    "line_width_px",
    "line_raw_center_px",
    "line_filtered_center_px",
    "track_error_px",
    "steering_rpm",
    "base_rpm",
    "right_target_rpm",
    "right_actual_rpm",
    "right_raw_rpm",
    "right_pwm_pct",
    "left_target_rpm",
    "left_actual_rpm",
    "left_raw_rpm",
    "left_pwm_pct",
    "right_trim",
    "left_trim",
    "steer_kp",
    "steer_kd",
    "max_steer_rpm",
    "ccd_valid",
    "ccd_sequence",
    "ccd_min",
    "ccd_max",
    "ccd_range",
    "ccd_mean",
    "ccd_dropped_frames",
    "ccd_crc_errors",
    "motor_crc_errors",
)


DEFAULT_PLOTS = (
    ("track_error_px", "steering_rpm"),
    ("right_target_rpm", "right_actual_rpm", "left_target_rpm", "left_actual_rpm"),
    ("right_pwm_pct", "left_pwm_pct"),
    ("tracking_ok", "holding_line", "line_lost"),
)


class TelemetryRecorder:
    """A line-buffered PlotJuggler CSV recording behind a three-method interface."""

    def __init__(self, path: Path, *, clock: Callable[[], float] = time.monotonic) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._clock = clock
        self._started = clock()
        self._last_time = -1.0
        self._handle = self.path.open("w", newline="", encoding="utf-8", buffering=1)
        self._writer = csv.DictWriter(self._handle, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self.samples = 0
        self._closed = False

    @classmethod
    def start_in(cls, folder: Path, *, clock: Callable[[], float] = time.monotonic) -> "TelemetryRecorder":
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = Path(folder) / f"ccd_car_run_{stamp}.csv"
        suffix = 1
        while path.exists():
            path = Path(folder) / f"ccd_car_run_{stamp}_{suffix}.csv"
            suffix += 1
        return cls(path, clock=clock)

    def append(
        self,
        packet: MotorTelemetry,
        frame: Frame | None,
        *,
        dropped_frames: int = 0,
        ccd_crc_errors: int = 0,
        motor_crc_errors: int = 0,
    ) -> None:
        if self._closed:
            raise RuntimeError("recording is already closed")
        elapsed = self._clock() - self._started
        if elapsed <= self._last_time:
            elapsed = self._last_time + 0.000001
        self._last_time = elapsed

        pixels = frame.pixels if frame is not None and frame.pixels else None
        ccd_min = min(pixels) if pixels else -1
        ccd_max = max(pixels) if pixels else -1
        row = {
            "time_s": f"{elapsed:.6f}",
            "wall_time_unix_s": f"{time.time():.6f}",
            "mode": packet.mode,
            "track_state": packet.track_state,
            "tracking_ok": int(packet.track_state == 2),
            "holding_line": int(packet.track_state == 1),
            "line_lost": int(packet.track_state == 0),
            "threshold": packet.threshold,
            "dark_line": packet.dark_line,
            "lost_frames": packet.lost_frames,
            "line_left_px": packet.track_left,
            "line_right_px": packet.track_right,
            "line_width_px": packet.track_width,
            "line_raw_center_px": packet.raw_center,
            "line_filtered_center_px": packet.filtered_center,
            "track_error_px": packet.track_error,
            "steering_rpm": packet.steering,
            "base_rpm": packet.base_rpm,
            "right_target_rpm": packet.motor1_target,
            "right_actual_rpm": packet.motor1_actual,
            "right_raw_rpm": packet.motor1_raw,
            "right_pwm_pct": packet.motor1_output,
            "left_target_rpm": packet.motor2_target,
            "left_actual_rpm": packet.motor2_actual,
            "left_raw_rpm": packet.motor2_raw,
            "left_pwm_pct": packet.motor2_output,
            "right_trim": packet.right_trim,
            "left_trim": packet.left_trim,
            "steer_kp": packet.steering_kp,
            "steer_kd": packet.steering_kd,
            "max_steer_rpm": packet.max_steer,
            "ccd_valid": int(pixels is not None),
            "ccd_sequence": frame.sequence if frame is not None else -1,
            "ccd_min": ccd_min,
            "ccd_max": ccd_max,
            "ccd_range": ccd_max - ccd_min if pixels else -1,
            "ccd_mean": sum(pixels) / len(pixels) if pixels else -1,
            "ccd_dropped_frames": dropped_frames,
            "ccd_crc_errors": ccd_crc_errors,
            "motor_crc_errors": motor_crc_errors,
        }
        self._writer.writerow(row)
        self.samples += 1

    def finish(self) -> Path:
        if not self._closed:
            self._handle.flush()
            self._handle.close()
            self._closed = True
            write_plotjuggler_layout(self.path)
        return self.path


def find_plotjuggler() -> Path | None:
    """Find the installed PlotJuggler 4 executable without relying on PATH."""
    candidates: list[Path] = []
    command = shutil.which("PlotJuggler4") or shutil.which("PlotJuggler4.exe")
    if command:
        candidates.append(Path(command))
    local = os.environ.get("LOCALAPPDATA")
    program_files = os.environ.get("ProgramFiles")
    if local:
        candidates.append(Path(local) / "Programs" / "PlotJuggler" / "bin" / "PlotJuggler4.exe")
    if program_files:
        candidates.extend(
            (
                Path(program_files) / "PlotJuggler" / "bin" / "PlotJuggler4.exe",
                Path(program_files) / "PlotJuggler" / "PlotJuggler4.exe",
            )
        )
    return next((candidate for candidate in candidates if candidate.is_file()), None)


def write_plotjuggler_layout(csv_path: Path) -> Path:
    """Create a source-bound PJ4 layout that loads the CSV with no setup dialog."""
    csv_path = Path(csv_path).resolve()
    layout_path = csv_path.with_suffix(".pj4.xml")
    source = csv_path.as_posix()
    topic = csv_path.stem
    config = json.dumps(
        {
            "filepath": source,
            "delimiter": ",",
            "time_mode": "column",
            "time_column_index": 0,
            "combined_date_index": -1,
            "combined_time_index": -1,
            "custom_time_format": "",
            "use_custom_format": False,
            "detect_delimiter": False,
            "column_history": ["time_s"],
        },
        ensure_ascii=False,
        separators=(",", ":"),
    )
    tabs: list[str] = []
    tab_names = ("循迹控制", "轮速跟随", "电机输出", "识别状态")
    for index, (tab_name, fields) in enumerate(zip(tab_names, DEFAULT_PLOTS), start=1):
        curves = "\n".join(
            f'            <curve topic="{html.escape(topic, quote=True)}" field="{html.escape(field, quote=True)}"/>'
            for field in fields
        )
        tabs.append(
            f'''    <Tab id="analysis_{index}" name="{tab_name}" containers="1">
      <Container>
        <DockArea id="analysis_area_{index}" name="{tab_name}">
          <plot id="plot_{index}" mode="TimeSeries">
{curves}
          </plot>
        </DockArea>
      </Container>
    </Tab>'''
        )
    layout = f'''<?xml version="1.0" encoding="UTF-8"?>
<root format="PlotJuggler" pj4_version="4" binding="source">
  <tabbed_widget parent="main_window">
{os.linesep.join(tabs)}
  </tabbed_widget>
  <data_processors/>
  <previouslyLoaded_Datafiles>
    <fileInfo filename="{html.escape(source, quote=True)}" prefix="">
      <dataset source_index="0" display_offset_ns="0"/>
      <plugin ID="CSV Loader" manifest_id="csv-loader" filepath_mode="source"><![CDATA[{config}]]></plugin>
    </fileInfo>
  </previouslyLoaded_Datafiles>
</root>
'''
    layout_path.write_text(layout, encoding="utf-8")
    return layout_path


def launch_plotjuggler(csv_path: Path) -> subprocess.Popen[bytes]:
    """Open one recording through a generated source-bound PlotJuggler layout."""
    executable = find_plotjuggler()
    if executable is None:
        raise FileNotFoundError("没有找到 PlotJuggler 4，请先安装 Windows x64 版本。")
    layout_path = write_plotjuggler_layout(Path(csv_path))
    return subprocess.Popen(
        [str(executable), "--nosplash", "--layout", str(layout_path)],
        cwd=str(executable.parent),
        close_fds=True,
    )
