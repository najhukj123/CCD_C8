import unittest

from ccd_viewer import CCDViewer


class RenderStabilityTests(unittest.TestCase):
    def test_repeated_frames_update_existing_canvas_items(self) -> None:
        app = CCDViewer()
        try:
            app.update_idletasks()
            app.current_pixels = bytes((index * 7) & 0xFF for index in range(1000))
            app.redraw()
            raw_ids = app.raw_canvas.find_all()
            binary_ids = app.binary_canvas.find_all()

            app.current_pixels = bytes((255 - index) & 0xFF for index in range(1000))
            app.redraw()

            self.assertEqual(app.raw_canvas.find_all(), raw_ids)
            self.assertEqual(app.binary_canvas.find_all(), binary_ids)
        finally:
            app.destroy()


if __name__ == "__main__":
    unittest.main()
