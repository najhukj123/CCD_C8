import unittest

from track import TrackFollower, automatic_threshold, detect_track


class TrackDetectionTests(unittest.TestCase):
    def test_auto_threshold_keeps_narrow_dark_line_below_dim_background(self) -> None:
        pixels = bytearray([55] * 1000)
        pixels[700:1000] = bytes([42] * 300)
        pixels[470:531] = bytes([20] * 61)
        self.assertLess(automatic_threshold(bytes(pixels)), 42)

    def test_detects_dark_line_and_ignores_narrow_noise(self) -> None:
        pixels = bytearray([220] * 1000)
        pixels[100:102] = b"\x00\x00"
        pixels[430:571] = bytes([25] * 141)
        result = detect_track(bytes(pixels), 128, dark_line=True)
        self.assertIsNotNone(result)
        self.assertEqual((result.left, result.right), (430, 570))
        self.assertAlmostEqual(result.center, 500.0)
        self.assertAlmostEqual(result.error, 0.5)

    def test_detects_white_line(self) -> None:
        pixels = bytearray([20] * 1000)
        pixels[700:751] = bytes([230] * 51)
        result = detect_track(bytes(pixels), 128, dark_line=False)
        self.assertIsNotNone(result)
        self.assertEqual((result.left, result.right), (700, 750))

    def test_rejects_a_background_sized_region(self) -> None:
        pixels = bytes([20] * 600 + [220] * 400)
        self.assertIsNone(detect_track(pixels, 128, dark_line=True, maximum_width=300))

    def test_rejects_dark_sensor_edges_and_chooses_middle_valley(self) -> None:
        pixels = bytearray([220] * 1000)
        pixels[30:120] = bytes([20] * 90)
        pixels[420:581] = bytes([20] * 161)
        pixels[880:970] = bytes([20] * 90)
        result = detect_track(
            bytes(pixels), 128, dark_line=True,
            minimum_width=20, maximum_width=350, roi_start=30, roi_end=969,
        )
        self.assertIsNotNone(result)
        self.assertEqual((result.left, result.right), (420, 580))

    def test_rejects_far_shadow_when_centre_line_is_missing(self) -> None:
        pixels = bytearray([50] * 1000)
        pixels[790:851] = bytes([5] * 61)
        result = detect_track(
            bytes(pixels), 30, dark_line=True,
            minimum_width=20, maximum_width=350, roi_start=30, roi_end=969,
        )
        self.assertIsNone(result)

    @staticmethod
    def dark_line(center: int, width: int = 100) -> bytes:
        pixels = bytearray([220] * 1000)
        left = center - width // 2
        pixels[left : left + width] = bytes([20] * width)
        return bytes(pixels)

    def test_follower_low_pass_filters_center(self) -> None:
        follower = TrackFollower(filter_alpha=0.25)
        first = follower.update(self.dark_line(500), 128)
        second = follower.update(self.dark_line(540), 128)
        self.assertEqual(first.state, "TRACKING")
        self.assertAlmostEqual(first.filtered_center, 499.5)
        self.assertAlmostEqual(second.filtered_center, 509.5)
        self.assertAlmostEqual(second.error, 10.0)

    def test_follower_holds_brief_loss_then_reports_lost(self) -> None:
        follower = TrackFollower(hold_frames=2)
        follower.update(self.dark_line(500), 128)
        blank = bytes([220] * 1000)
        self.assertEqual(follower.update(blank, 128).state, "HOLD")
        self.assertEqual(follower.update(blank, 128).state, "HOLD")
        lost = follower.update(blank, 128)
        self.assertEqual(lost.state, "LOST")
        self.assertIsNone(lost.error)

    def test_zero_calibration_removes_static_offset(self) -> None:
        follower = TrackFollower()
        follower.update(self.dark_line(470), 128)
        self.assertTrue(follower.calibrate_zero())
        result = follower.update(self.dark_line(470), 128)
        self.assertAlmostEqual(result.error, 0.0)


if __name__ == "__main__":
    unittest.main()
