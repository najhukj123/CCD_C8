import unittest

from protocol import BAUD_RATE, PIXEL_COUNT


class ThroughputTests(unittest.TestCase):
    def test_bluetooth_uart_can_carry_both_streams_with_margin(self) -> None:
        ccd_packet_bytes = 8 + PIXEL_COUNT + 2
        motor_packet_bytes = 90
        required_bytes_per_second = ccd_packet_bytes * (1000 / 80) + motor_packet_bytes * 20
        available_bytes_per_second = BAUD_RATE / 10
        self.assertGreaterEqual(
            available_bytes_per_second,
            required_bytes_per_second * 1.2,
            "UART cannot carry 12.5 FPS CCD plus 20 Hz motor telemetry with 20% margin",
        )
        self.assertLessEqual(required_bytes_per_second, 20_000 * 0.8)


if __name__ == "__main__":
    unittest.main()
