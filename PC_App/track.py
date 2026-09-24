"""Track-centre detection for a one-dimensional CCD image."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TrackDetection:
    left: int
    right: int
    center: float
    error: float
    width: int


@dataclass(frozen=True)
class TrackingResult:
    detection: TrackDetection | None
    filtered_center: float | None
    error: float | None
    state: str
    lost_frames: int


def automatic_threshold(pixels: bytes, roi_start: int = 30, roi_end: int = 969) -> int:
    """Match the STM32 p2/p80 threshold, including narrow dark lines."""
    if not pixels:
        return 0
    roi_start = max(0, roi_start)
    roi_end = min(len(pixels) - 1, roi_end)
    if roi_start > roi_end:
        return 0
    histogram = [0] * 256
    for value in pixels[roi_start : roi_end + 1]:
        histogram[value] += 1
    total = roi_end - roi_start + 1

    def percentile(target: int) -> int:
        accumulated = 0
        for value, count in enumerate(histogram):
            accumulated += count
            if accumulated >= target:
                return value
        return 255

    low = percentile(max(1, total * 2 // 100))
    high = percentile(total * 8 // 10)
    return (low + high) // 2


class TrackFollower:
    """Stateful line tracker that produces a stable steering error."""

    def __init__(
        self,
        pixel_count: int = 1000,
        *,
        filter_alpha: float = 0.28,
        hold_frames: int = 3,
        maximum_jump: float = 180.0,
        minimum_width: int = 20,
        maximum_width: int = 350,
        roi_margin: int = 30,
    ) -> None:
        self.pixel_count = pixel_count
        self.filter_alpha = filter_alpha
        self.hold_frames = hold_frames
        self.maximum_jump = maximum_jump
        self.minimum_width = minimum_width
        self.maximum_width = maximum_width
        self.roi_margin = roi_margin
        self.zero_center = (pixel_count - 1) / 2.0
        self.previous_raw_center: float | None = None
        self.filtered_center: float | None = None
        self.lost_frames = 0

    def reset_tracking(self) -> None:
        self.previous_raw_center = None
        self.filtered_center = None
        self.lost_frames = 0

    def reset_zero(self) -> None:
        self.zero_center = (self.pixel_count - 1) / 2.0

    def calibrate_zero(self) -> bool:
        if self.filtered_center is None:
            return False
        self.zero_center = self.filtered_center
        return True

    def update(self, pixels: bytes, threshold: int, *, dark_line: bool = True) -> TrackingResult:
        detection = detect_track(
            pixels,
            threshold,
            dark_line=dark_line,
            previous_center=self.previous_raw_center,
            minimum_width=self.minimum_width,
            maximum_width=self.maximum_width,
            reference_center=self.zero_center,
            roi_start=self.roi_margin,
            roi_end=len(pixels) - 1 - self.roi_margin,
        )

        if (
            detection is not None
            and self.previous_raw_center is not None
            and abs(detection.center - self.previous_raw_center) > self.maximum_jump
        ):
            detection = None

        if detection is not None:
            self.previous_raw_center = detection.center
            if self.filtered_center is None:
                self.filtered_center = detection.center
            else:
                self.filtered_center += self.filter_alpha * (
                    detection.center - self.filtered_center
                )
            self.lost_frames = 0
            return TrackingResult(
                detection=detection,
                filtered_center=self.filtered_center,
                error=self.filtered_center - self.zero_center,
                state="TRACKING",
                lost_frames=0,
            )

        self.lost_frames += 1
        if self.filtered_center is not None and self.lost_frames <= self.hold_frames:
            return TrackingResult(
                detection=None,
                filtered_center=self.filtered_center,
                error=self.filtered_center - self.zero_center,
                state="HOLD",
                lost_frames=self.lost_frames,
            )

        # 丢线后重新找线时，不再拿很久以前的位置限制跳变。
        self.previous_raw_center = None
        self.filtered_center = None
        return TrackingResult(
            detection=None,
            filtered_center=None,
            error=None,
            state="LOST",
            lost_frames=self.lost_frames,
        )


def detect_track(
    pixels: bytes,
    threshold: int,
    *,
    dark_line: bool = True,
    previous_center: float | None = None,
    minimum_width: int = 4,
    maximum_width: int = 400,
    maximum_center_offset: float = 300.0,
    reference_center: float | None = None,
    roi_start: int = 0,
    roi_end: int | None = None,
) -> TrackDetection | None:
    """Find a plausible continuous line segment and return its centre error."""
    if not pixels:
        return None
    if roi_end is None:
        roi_end = len(pixels) - 1
    roi_start = max(0, roi_start)
    roi_end = min(len(pixels) - 1, roi_end)
    if roi_start > roi_end:
        return None

    sensor_center = (len(pixels) - 1) / 2.0
    if reference_center is None:
        reference_center = sensor_center
    runs: list[tuple[int, int]] = []
    run_start: int | None = None
    for index in range(roi_start, roi_end + 2):
        active = False
        if index <= roi_end:
            active = pixels[index] < threshold if dark_line else pixels[index] >= threshold
        if active and run_start is None:
            run_start = index
        elif not active and run_start is not None:
            right = index - 1
            width = right - run_start + 1
            # A real valley must have background on both sides.  This rejects
            # the dark unused areas commonly visible at both sensor ends.
            has_two_edges = run_start > roi_start + 2 and right < roi_end - 2
            center = (run_start + right) / 2.0
            inside_centre = abs(center - reference_center) <= maximum_center_offset
            if minimum_width <= width <= maximum_width and has_two_edges and inside_centre:
                runs.append((run_start, right))
            run_start = None

    if not runs:
        return None

    target = previous_center if previous_center is not None else reference_center
    left, right = min(
        runs,
        key=lambda run: (abs(((run[0] + run[1]) / 2.0) - target), -(run[1] - run[0] + 1)),
    )
    center = (left + right) / 2.0
    return TrackDetection(
        left=left,
        right=right,
        center=center,
        error=center - sensor_center,
        width=right - left + 1,
    )
