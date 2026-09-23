"""Wire protocol shared by the CCD viewer and its tests."""

from __future__ import annotations

import struct
from dataclasses import dataclass


MAGIC = b"CCD1"
MOTOR_MAGIC = b"MTR2"
LEGACY_MOTOR_MAGIC = b"MTR1"
PIXEL_COUNT = 1000
BAUD_RATE = 921600


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    sequence: int
    pixels: bytes


@dataclass(frozen=True)
class MotorTelemetry:
    sequence: int
    mode: int
    ready_flags: int
    motor1_target: float
    motor1_actual: float
    motor1_raw: float
    motor1_output: float
    motor2_target: float
    motor2_actual: float
    motor2_raw: float
    motor2_output: float
    track_state: int = 0
    dark_line: int = 1
    threshold: int = 97
    lost_frames: int = 0
    track_left: int = 0
    track_right: int = 0
    track_width: int = 0
    raw_center: float = 499.5
    filtered_center: float = 499.5
    track_error: float = 0.0
    steering: float = 0.0
    base_rpm: float = 0.0
    right_trim: float = 1.0
    left_trim: float = 1.0
    steering_kp: float = 0.0
    steering_kd: float = 0.0
    max_steer: float = 120.0
    tracking_available: bool = False


class MotorPacketParser:
    """Incremental parser for MTR2 telemetry, with MTR1 compatibility."""

    PACKET_SIZE = 90
    LEGACY_PACKET_SIZE = 42

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, chunk: bytes) -> list[MotorTelemetry]:
        self.buffer.extend(chunk)
        packets: list[MotorTelemetry] = []

        while True:
            marker_v2 = self.buffer.find(MOTOR_MAGIC)
            marker_v1 = self.buffer.find(LEGACY_MOTOR_MAGIC)
            markers = [value for value in (marker_v2, marker_v1) if value >= 0]
            if not markers:
                if len(self.buffer) > len(MOTOR_MAGIC) - 1:
                    del self.buffer[: -(len(MOTOR_MAGIC) - 1)]
                break
            marker = min(markers)
            if marker:
                del self.buffer[:marker]
            is_v2 = self.buffer.startswith(MOTOR_MAGIC)
            packet_size = self.PACKET_SIZE if is_v2 else self.LEGACY_PACKET_SIZE
            crc_offset = 88 if is_v2 else 40
            crc_length = 84 if is_v2 else 36
            if len(self.buffer) < packet_size:
                break

            received_crc = struct.unpack_from("<H", self.buffer, crc_offset)[0]
            if received_crc != crc16_ccitt(bytes(self.buffer[4 : 4 + crc_length])):
                self.crc_errors += 1
                del self.buffer[0]
                continue

            sequence, mode, flags = struct.unpack_from("<HBB", self.buffer, 4)
            values = struct.unpack_from("<8f", self.buffer, 8)
            if is_v2:
                state, dark_line, threshold, lost_frames = struct.unpack_from(
                    "<BBBB", self.buffer, 40
                )
                left, right, width, max_steer_tenths = struct.unpack_from("<4H", self.buffer, 44)
                tracking = struct.unpack_from("<9f", self.buffer, 52)
                packets.append(
                    MotorTelemetry(
                        sequence,
                        mode,
                        flags,
                        *values,
                        state,
                        dark_line,
                        threshold,
                        lost_frames,
                        left,
                        right,
                        width,
                        *tracking,
                        max_steer_tenths / 10.0,
                        0.8 <= tracking[5] <= 1.2 and 0.8 <= tracking[6] <= 1.2,
                    )
                )
            else:
                packets.append(MotorTelemetry(sequence, mode, flags, *values))
            del self.buffer[:packet_size]

        return packets


class PacketParser:
    """Incremental parser that can recover after noise or a partial packet."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, chunk: bytes) -> list[Frame]:
        self.buffer.extend(chunk)
        frames: list[Frame] = []

        while True:
            marker = self.buffer.find(MAGIC)
            if marker < 0:
                if len(self.buffer) > len(MAGIC) - 1:
                    del self.buffer[: -(len(MAGIC) - 1)]
                break
            if marker:
                del self.buffer[:marker]
            if len(self.buffer) < 8:
                break

            length, sequence = struct.unpack_from("<HH", self.buffer, 4)
            if length != PIXEL_COUNT:
                del self.buffer[0]
                continue

            packet_size = 8 + length + 2
            if len(self.buffer) < packet_size:
                break

            pixels = bytes(self.buffer[8 : 8 + length])
            received_crc = struct.unpack_from("<H", self.buffer, 8 + length)[0]
            if received_crc != crc16_ccitt(pixels):
                self.crc_errors += 1
                del self.buffer[0]
                continue

            frames.append(Frame(sequence, pixels))
            del self.buffer[:packet_size]

        return frames
