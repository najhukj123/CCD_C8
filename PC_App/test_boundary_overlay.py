import unittest

import tkinter as tk

from car_debug_console import CarDebugConsole
from protocol import MotorTelemetry


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


class FakeCanvas:
    def __init__(self):
        self.next_id = 1
        self.coordinates = {}
        self.options = {}

    def winfo_width(self):
        return 1045

    def winfo_height(self):
        return 300

    def delete(self, _tag):
        self.coordinates.clear()
        self.options.clear()

    def _create(self, *coordinates, **options):
        item = self.next_id
        self.next_id += 1
        self.coordinates[item] = coordinates
        self.options[item] = options
        return item

    def create_line(self, *coordinates, **options):
        return self._create(*coordinates, **options)

    def create_text(self, *coordinates, **options):
        return self._create(*coordinates, **options)

    def coords(self, item, *coordinates):
        self.coordinates[item] = coordinates

    def itemconfigure(self, item, **options):
        self.options[item].update(options)


class BoundaryOverlayTests(unittest.TestCase):
    def test_binary_plot_shows_only_stm32_selected_line(self):
        app = CarDebugConsole.__new__(CarDebugConsole)
        app.binary_canvas = FakeCanvas()
        app.binary_size = None
        app.binary_wave = None
        app.threshold_var = Value("30")
        app.line_mode_var = Value("黑线")
        app.frozen_motor = None
        app.current_motor = MotorTelemetry(
            1, 0, 0, *([0.0] * 8),
            track_state=2, track_left=450, track_right=510,
            filtered_center=480.0, tracking_available=True,
        )
        pixels = bytearray([50] * 1000)
        pixels[0:60] = bytes([5] * 60)
        pixels[450:511] = bytes([5] * 61)
        pixels[940:1000] = bytes([5] * 60)

        app._draw_binary(bytes(pixels))

        points = app.binary_canvas.coordinates[app.binary_wave]
        top, bottom = 10, 280
        self.assertEqual(points[20 * 2 + 1], bottom)
        self.assertEqual(points[480 * 2 + 1], top)
        self.assertEqual(points[970 * 2 + 1], bottom)

    def test_local_ccd_valley_draws_green_edges_without_motor_telemetry(self):
        app = CarDebugConsole.__new__(CarDebugConsole)
        app.raw_canvas = FakeCanvas()
        app.raw_size = None
        app.raw_wave = None
        app.raw_threshold = None
        app.raw_info = None
        app.raw_sensor_center = None
        app.raw_left = None
        app.raw_right = None
        app.raw_center = None
        app.threshold_var = Value("0")
        app.line_mode_var = Value("黑线")
        app.current_motor = None
        app.frozen_motor = None

        pixels = bytearray([50] * 1000)
        pixels[400:601] = bytes([5] * 201)
        app._draw_raw(bytes(pixels))

        canvas = app.raw_canvas
        self.assertEqual(canvas.options[app.raw_left].get("state"), tk.NORMAL)
        self.assertEqual(canvas.options[app.raw_right].get("state"), tk.NORMAL)
        left_x = canvas.coordinates[app.raw_left][0]
        right_x = canvas.coordinates[app.raw_right][0]
        self.assertAlmostEqual(left_x, 45 + 400 / 999 * (1045 - 12 - 45), delta=2)
        self.assertAlmostEqual(right_x, 45 + 600 / 999 * (1045 - 12 - 45), delta=2)


if __name__ == "__main__":
    unittest.main()
