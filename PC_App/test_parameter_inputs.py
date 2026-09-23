import unittest

from car_debug_console import CarDebugConsole
from protocol import MotorTelemetry


class FakeVariable:
    def __init__(self, value: str) -> None:
        self.value = value

    def get(self) -> str:
        return self.value

    def set(self, value: object) -> None:
        self.value = str(value)


class ParameterInputTests(unittest.TestCase):
    def test_zero_telemetry_does_not_erase_parameter_inputs(self) -> None:
        app = CarDebugConsole.__new__(CarDebugConsole)
        app.connection_var = FakeVariable("")
        app.track_var = FakeVariable("")
        app.motor_var = FakeVariable("")

        expected = {
            "threshold_var": "97",
            "line_mode_var": "黑线",
            "base_rpm_var": "60",
            "steer_kp_var": "0.35",
            "steer_kd_var": "0.20",
            "max_steer_var": "120",
            "right_trim_var": "0.996",
            "left_trim_var": "1.004",
        }
        for name, value in expected.items():
            setattr(app, name, FakeVariable(value))

        app._update_telemetry(MotorTelemetry(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

        self.assertEqual(
            {name: getattr(app, name).get() for name in expected},
            expected,
        )

    def test_straight_command_does_not_apply_trim_twice(self) -> None:
        app = CarDebugConsole.__new__(CarDebugConsole)
        app.base_rpm_var = FakeVariable("60")
        app.right_trim_var = FakeVariable("0.996")
        app.left_trim_var = FakeVariable("1.004")
        app.apply_parameters = lambda: True
        commands = []
        app.send = lambda command, **_kwargs: commands.append(command) or True

        app.start_straight()

        self.assertEqual(commands, ["DUAL,60,60"])


if __name__ == "__main__":
    unittest.main()
