import pathlib
import re
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from terminal_view import Box, Frame, grid_size, render

ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def visible(text):
    return ANSI.sub("", text).split("\r\n")


class TerminalViewTest(unittest.TestCase):
    def test_24_rows_and_aspect_ratio(self):
        self.assertEqual(grid_size(72, 24, 48, 640, 480), (48, 18))
        text = render(Frame(), 72, 24)
        lines = visible(text)
        self.assertEqual(len(lines), 24)
        self.assertTrue(all(len(line) == 50 for line in lines))
        self.assertFalse(text.endswith("\n"))
        self.assertLess(len(text.encode("ascii")), 4096)

    def test_centers_filled_boxes_clipping_and_overlap(self):
        frame = Frame(filters_current=True, boxes=[
            Box(3, -20, -20, 1000, 1000),  # fills entire image after clipping
            Box(1, 0, 0, 80, 80),
            Box(0, 0, 0, 2, 2),
            Box(0, 638, 478, 2, 2),
            Box(2, 650, 500, 20, 20),  # outside image, must not accumulate at edge
            Box(4, 100, 100, 100, 100, False)])
        lines = visible(render(frame, 72, 24, color=False))
        self.assertEqual(lines[1][1], ".")
        self.assertEqual(lines[18][48], ".")
        self.assertIn("B", lines[2])
        self.assertIn("D", lines[10])
        self.assertNotIn("G", "".join(lines[1:19]))
        self.assertNotIn("H", "".join(lines[1:19]))

    def test_pause_raw_and_stale_do_not_reuse_old_boxes(self):
        frame = Frame(run="PAUSED", filters_current=False,
                      boxes=[Box(1, 100, 100, 50, 50, False)])
        self.assertIn("B", "".join(visible(render(frame, 72, 24))[1:19]))
        stale = visible(render(frame, 72, 24, age=3))
        self.assertIn("STALE", stale[20])
        self.assertTrue(all(line == "|" + " " * 48 + "|" for line in stale[1:19]))

    def test_strict_majority_and_forced_center(self):
        def painted(box):
            # 480x360 -> 48x18: a cell corresponds to 10x20 pixels.
            frame = Frame(width=480, height=360, boxes=[box])
            lines = visible(render(frame, 72, 24, color=False))[1:19]
            return {(x, y) for y, line in enumerate(lines)
                    for x, char in enumerate(line[1:-1]) if char == "B"}

        self.assertEqual(painted(Box(1, 5, 0, 20, 20)), {(1, 0)})
        self.assertEqual(painted(Box(1, 4.9, 0, 20, 20)), {(0, 0), (1, 0)})
        # Coverage uses area, not independent >50% tests on each axis.
        self.assertEqual(painted(Box(1, 4, 8, 20, 40)), {(1, 0), (0, 1), (1, 1)})
        # Tiny objects spanning four cells still get exactly their center cell.
        self.assertEqual(painted(Box(1, 9, 19, 2, 2)), {(1, 1)})
        # A visible edge fragment whose original center is off-image survives.
        self.assertEqual(painted(Box(1, -20, 1, 21, 1)), {(0, 0)})
        self.assertEqual(painted(Box(1, -20, 1, 10, 1)), set())

    def test_waiting_small_terminal_and_hostile_text(self):
        self.assertIn("NO DATA", render(None, 72, 24))
        for columns, rows in [(40, 15), (19, 9), (100, 24)]:
            lines = visible(render(Frame(), columns, rows))
            self.assertLessEqual(len(lines), rows)
            self.assertTrue(all(len(line) < columns for line in lines))
        text = render(Frame(run="\x1b[2J\n악성"), 72, 24, color=False)
        self.assertNotIn("\x1b[2J", text)
        text.encode("ascii")
        self.assertIn("Resize terminal", render(None, 18, 8))


if __name__ == "__main__":
    unittest.main()
