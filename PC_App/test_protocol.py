import struct
import unittest

from protocol import MAGIC, PIXEL_COUNT, MotorPacketParser, PacketParser, crc16_ccitt


def make_packet(sequence: int, pixels: bytes) -> bytes:
    return MAGIC + struct.pack("<HH", len(pixels), sequence) + pixels + struct.pack("<H", crc16_ccitt(pixels))


class PacketParserTests(unittest.TestCase):
    def test_split_packet_and_noise(self) -> None:
        pixels = bytes((index * 7) & 0xFF for index in range(PIXEL_COUNT))
        packet = b"noise" + make_packet(42, pixels)
        parser = PacketParser()
        self.assertEqual(parser.feed(packet[:93]), [])
        frames = parser.feed(packet[93:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 42)
        self.assertEqual(frames[0].pixels, pixels)

    def test_bad_crc_recovers_at_next_packet(self) -> None:
        pixels = bytes(range(250)) * 4
        damaged = bytearray(make_packet(1, pixels))
        damaged[20] ^= 1
        parser = PacketParser()
        frames = parser.feed(bytes(damaged) + make_packet(2, pixels))
        self.assertEqual([frame.sequence for frame in frames], [2])
        self.assertEqual(parser.crc_errors, 1)

    def test_motor_telemetry_split_packet(self) -> None:
        body = struct.pack("<HBB8f", 7, 4, 3, 80.0, 79.5, 81.0, 22.0, 80.0, 78.0, 77.5, 24.0)
        packet = b"MTR1" + body + struct.pack("<H", crc16_ccitt(body))
        parser = MotorPacketParser()
        self.assertEqual(parser.feed(packet[:11]), [])
        result = parser.feed(packet[11:])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].sequence, 7)
        self.assertEqual(result[0].mode, 4)
        self.assertAlmostEqual(result[0].motor2_actual, 78.0)

    def test_motor_parser_recovers_from_bad_crc(self) -> None:
        body = struct.pack("<HBB8f", 8, 0, 3, *([0.0] * 8))
        bad = b"MTR1" + body + b"\x00\x00"
        good = b"MTR1" + body + struct.pack("<H", crc16_ccitt(body))
        parser = MotorPacketParser()
        result = parser.feed(b"noise" + bad + good)
        self.assertEqual(len(result), 1)
        self.assertEqual(parser.crc_errors, 1)

    def test_mtr2_contains_tracking_and_runtime_parameters(self) -> None:
        motor_values = (60.0, 59.8, 60.1, 18.0, 60.0, 60.2, 59.9, 19.0)
        tracking_values = (480.0, 482.5, -17.0, -6.2, 60.0, 0.996, 1.004, 0.35, 0.2)
        body = (
            struct.pack("<HBB8f", 9, 3, 3, *motor_values)
            + struct.pack("<BBBB4H9f", 2, 1, 97, 0, 450, 510, 61, 1200, *tracking_values)
        )
        packet = b"MTR2" + body + struct.pack("<H", crc16_ccitt(body))
        parser = MotorPacketParser()
        result = parser.feed(packet)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].track_state, 2)
        self.assertEqual(result[0].track_width, 61)
        self.assertAlmostEqual(result[0].track_error, -17.0)
        self.assertAlmostEqual(result[0].right_trim, 0.996)
        self.assertAlmostEqual(result[0].max_steer, 120.0)
        self.assertTrue(result[0].tracking_available)


if __name__ == "__main__":
    unittest.main()
