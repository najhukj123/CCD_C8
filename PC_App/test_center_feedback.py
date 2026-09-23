import unittest
from unittest.mock import patch

import tkinter as tk

from car_debug_console import CarDebugConsole
from protocol import MotorTelemetry
from test_boundary_overlay import FakeCanvas


class Value:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class Link:
    def __init__(self):
        self.commands = []

    def send(self, command):
        self.commands.append(command)
        return True


def telemetry(sequence, center=380.0, error=-119.5):
    return MotorTelemetry(
        sequence, 0, 0, *([0.0] * 8),
        track_state=2, track_left=350, track_right=410,
        filtered_center=center, track_error=error,
        tracking_available=True,
    )


class CenterFeedbackTests(unittest.TestCase):
    def make_app(self):
        app = CarDebugConsole.__new__(CarDebugConsole)
        app.link = Link()
        app.current_motor = telemetry(10)
        app.last_motor_received_at = 100.0
        app.calibration_var = Value()
        app.connection_var = Value()
        app.stream_var = Value("同时")
        return app

    def test_center_requires_fresh_telemetry_to_confirm(self):
        app = self.make_app()
        with patch("car_debug_console.time.monotonic", return_value=100.1):
            app.calibrate_center()
        self.assertEqual(app.link.commands, ["CENTER"])
        self.assertIn("等待", app.calibration_var.get())

        app._observe_center_calibration(telemetry(10, error=0.0), 100.2)
        self.assertIn("等待", app.calibration_var.get())
        app._observe_center_calibration(telemetry(11, error=0.0), 100.3)
        self.assertIn("生效", app.calibration_var.get())
        self.assertIsNone(app.pending_center_calibration)

    def test_center_reports_unconfirmed_error(self):
        app = self.make_app()
        with patch("car_debug_console.time.monotonic", return_value=100.1):
            app.calibrate_center()
        app._observe_center_calibration(telemetry(11), 100.3)
        app._check_center_calibration_timeout(102.2)
        self.assertIn("未确认", app.calibration_var.get())
        self.assertIsNone(app.pending_center_calibration)

    def test_center_does_not_send_with_stale_telemetry(self):
        app = self.make_app()
        with patch("car_debug_console.time.monotonic", return_value=103.0):
            app.calibrate_center()
        self.assertEqual(app.link.commands, [])
        self.assertIn("未设置", app.calibration_var.get())

    def test_raw_plot_shows_calibrated_zero_without_moving_raw_pixels(self):
        app = self.make_app()
        app.raw_canvas = FakeCanvas()
        app.raw_size = None
        app.raw_wave = None
        app.raw_threshold = None
        app.raw_info = None
        app.raw_sensor_center = None
        app.raw_left = None
        app.raw_right = None
        app.raw_center = None
        app.raw_calibrated_center = None
        app.threshold_var = Value("30")
        app.line_mode_var = Value("黑线")
        app.frozen_motor = None
        pixels = bytes([50] * 1000)

        app._draw_raw(pixels)

        canvas = app.raw_canvas
        zero_x = canvas.coordinates[app.raw_calibrated_center][0]
        sensor_x = canvas.coordinates[app.raw_sensor_center][0]
        detected_x = canvas.coordinates[app.raw_center][0]
        self.assertAlmostEqual(zero_x, 45 + 499.5 / 999 * 988, delta=2)
        self.assertAlmostEqual(sensor_x, zero_x, delta=0.1)
        self.assertAlmostEqual(detected_x, 45 + 380 / 999 * 988, delta=2)
        self.assertEqual(canvas.options[app.raw_calibrated_center].get("state"), tk.NORMAL)

        app.current_motor = telemetry(11, error=0.0)
        app._draw_raw(pixels)
        moved_zero_x = canvas.coordinates[app.raw_calibrated_center][0]
        self.assertAlmostEqual(moved_zero_x, detected_x, delta=0.1)
        self.assertAlmostEqual(canvas.coordinates[app.raw_sensor_center][0], sensor_x, delta=0.1)


if __name__ == "__main__":
    unittest.main()
