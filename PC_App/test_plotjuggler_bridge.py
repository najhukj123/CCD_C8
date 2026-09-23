import csv
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from plotjuggler_bridge import CSV_COLUMNS, TelemetryRecorder, write_plotjuggler_layout
from protocol import Frame, MotorTelemetry


class FakeClock:
    def __init__(self) -> None:
        self.value = 10.0

    def __call__(self) -> float:
        self.value += 0.05
        return self.value


class PlotJugglerBridgeTests(unittest.TestCase):
    def test_recording_writes_numeric_telemetry_and_layout(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "run.csv"
            recorder = TelemetryRecorder(path, clock=FakeClock())
            packet = MotorTelemetry(
                sequence=7,
                mode=3,
                ready_flags=3,
                motor1_target=80.0,
                motor1_actual=79.0,
                motor1_raw=78.0,
                motor1_output=35.0,
                motor2_target=82.0,
                motor2_actual=81.0,
                motor2_raw=80.0,
                motor2_output=36.0,
                track_state=2,
                dark_line=1,
                threshold=97,
                lost_frames=0,
                track_left=400,
                track_right=600,
                track_width=200,
                raw_center=500.0,
                filtered_center=499.0,
                track_error=-0.5,
                steering=-0.175,
                base_rpm=80.0,
                right_trim=0.996,
                left_trim=1.004,
                steering_kp=0.35,
                steering_kd=0.2,
                max_steer=120.0,
            )
            recorder.append(
                packet,
                Frame(12, bytes((10, 20, 30))),
                dropped_frames=4,
                ccd_crc_errors=1,
                motor_crc_errors=2,
            )
            result = recorder.finish()

            self.assertEqual(result, path)
            self.assertEqual(recorder.samples, 1)
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(tuple(rows[0]), CSV_COLUMNS)
            self.assertEqual(rows[0]["tracking_ok"], "1")
            self.assertEqual(rows[0]["right_actual_rpm"], "79.0")
            self.assertEqual(rows[0]["ccd_range"], "20")
            self.assertEqual(rows[0]["ccd_dropped_frames"], "4")

            layout = path.with_suffix(".pj4.xml")
            self.assertTrue(layout.is_file())
            root = ET.parse(layout).getroot()
            self.assertEqual(root.attrib["pj4_version"], "4")
            plugin = root.find("./previouslyLoaded_Datafiles/fileInfo/plugin")
            self.assertIsNotNone(plugin)
            self.assertEqual(plugin.attrib["manifest_id"], "csv-loader")
            self.assertIn('"time_column_index":0', plugin.text)

    def test_layout_contains_ready_made_analysis_curves(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            csv_path = Path(folder) / "lap.csv"
            csv_path.write_text("time_s,track_error_px\n0,0\n", encoding="utf-8")
            layout = write_plotjuggler_layout(csv_path)
            root = ET.parse(layout).getroot()
            fields = {curve.attrib["field"] for curve in root.findall(".//curve")}
            self.assertIn("track_error_px", fields)
            self.assertIn("steering_rpm", fields)
            self.assertIn("right_actual_rpm", fields)
            self.assertIn("left_actual_rpm", fields)


if __name__ == "__main__":
    unittest.main()
