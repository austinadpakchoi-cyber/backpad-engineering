"""Regression for a false click whose release clears the final HID button mask."""
import unittest

from reproduce import check_release_tail_buttons


class ReleaseTailButtonsTest(unittest.TestCase):
    def test_press_then_release_is_rejected(self):
        row = {"cadence": "5", "tail_um": "500", "tail_ms": "60", "left": "1", "final_buttons": "0"}
        with self.assertRaisesRegex(RuntimeError, r"r2 .*unexpected left-button press"):
            check_release_tail_buttons("r2", [row])


if __name__ == "__main__":
    unittest.main()
