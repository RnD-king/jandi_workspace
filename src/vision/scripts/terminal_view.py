"""Pure ASCII renderer for the SSH viewer; no ROS/image dependencies."""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field

SYMBOLS = {0: ".", 1: "B", 2: "G", 3: "D", 4: "H"}
COLORS = {0: "92", 1: "93", 2: "95", 3: "96", 4: "91"}


@dataclass(frozen=True)
class Box:
    kind: int
    x: float
    y: float
    width: float
    height: float
    filtered: bool = True


@dataclass
class Frame:
    width: int = 640
    height: int = 480
    run: str = "WAITING FOR DATA"
    mission: str = "-"
    phase: str = "-"
    action: str = "-"
    action_id: int = 0
    camera: str = "-"
    settled: bool = True
    filters_current: bool = False
    boxes: list[Box] = field(default_factory=list)


def ascii_text(value: object) -> str:
    # One byte/cell. Also prevents topic strings from injecting terminal escapes.
    return "".join(c if 32 <= ord(c) <= 126 else "?" for c in str(value))


def grid_size(columns: int, rows: int, desired: int, width: int, height: int) -> tuple[int, int]:
    """Two border rows + four status rows. Reserve the terminal's last column."""
    if columns < 19 or rows < 9:
        return 0, 0
    width, height = max(1, width), max(1, height)
    grid_columns = min(desired, columns - 3)
    grid_rows = max(1, round(grid_columns * height / (2 * width)))
    if grid_rows > rows - 6:
        grid_rows = rows - 6
        grid_columns = min(grid_columns, max(1, round(grid_rows * 2 * width / height)))
    return grid_columns, grid_rows


def render(frame: Frame | None, columns: int, rows: int, *, desired: int = 48,
           age: float = 0, stale_after: float = 2, rx_hz: float = 0,
           output_kbps: float = 0, color: bool = True) -> str:
    """Full cursor-home redraw, no trailing newline and no terminal scroll."""
    source = frame or Frame()
    w, h = grid_size(columns, rows, desired, source.width, source.height)
    if not w or not h:
        text = "Resize terminal (>=19x9)"
        return "\x1b[H" + text[:max(0, columns - 1)] + "\x1b[0m\x1b[J"
    stale = frame is not None and age >= stale_after
    cells = [[-1] * w for _ in range(h)]
    counts: Counter = Counter()
    # Larger objects go behind smaller ones; line points remain visible on top.
    boxes = [] if frame is None or stale else sorted(
        source.boxes, key=lambda b: (b.kind == 0, -b.width * b.height))
    for box in boxes:
        if box.kind not in SYMBOLS or not all(math.isfinite(v) for v in
                (box.x, box.y, box.width, box.height)) or box.width <= 0 or box.height <= 0:
            continue
        if source.filters_current and not box.filtered:
            continue
        if box.kind == 0:
            x, y = box.x + box.width / 2, box.y + box.height / 2
            if 0 <= x < source.width and 0 <= y < source.height:
                cells[min(h-1, int(y*h/source.height))][min(w-1, int(x*w/source.width))] = 0
                counts[0] += 1
            continue
        left, top = max(0.0, box.x), max(0.0, box.y)
        right = min(float(source.width), box.x + box.width)
        bottom = min(float(source.height), box.y + box.height)
        if left >= right or top >= bottom:
            continue
        x0, y0 = int(left*w/source.width), int(top*h/source.height)
        x1 = min(w, math.ceil(right*w/source.width))
        y1 = min(h, math.ceil(bottom*h/source.height))
        # Work in cell coordinates: each complete cell has area 1. Exactly
        # half coverage is excluded; the object's center cell is forced below.
        gx0, gy0 = left*w/source.width, top*h/source.height
        gx1, gy1 = right*w/source.width, bottom*h/source.height
        for y in range(y0, y1):
            overlap_y = max(0.0, min(gy1, y+1) - max(gy0, y))
            for x in range(x0, x1):
                overlap_x = max(0.0, min(gx1, x+1) - max(gx0, x))
                if overlap_x * overlap_y > 0.5:
                    cells[y][x] = box.kind
        center_x, center_y = box.x + box.width/2, box.y + box.height/2
        # If the raw center lies outside the image, use the visible clipped
        # rectangle's center so even a small visible fragment gets one cell.
        if not 0 <= center_x < source.width or not 0 <= center_y < source.height:
            center_x, center_y = (left+right)/2, (top+bottom)/2
        cells[min(h-1, int(center_y*h/source.height))][min(w-1, int(center_x*w/source.width))] = box.kind
        counts[box.kind] += 1

    def draw_row(row: list[int]) -> str:
        text, previous = "|", -1
        for kind in row:
            if color and kind != previous:
                text += "\x1b[0m" if kind < 0 else f"\x1b[{COLORS[kind]}m"
            text += SYMBOLS.get(kind, " ")
            previous = kind
        return text + ("\x1b[0m" if color else "") + "|"

    mode = "FILTERED" if source.filters_current else "RAW"
    run = "NO DATA: terminal_node:=true?" if frame is None else (
        f"STALE {age:.1f}s: no new frames" if stale else f"{source.run} {mode} RX {rx_hz:.1f}Hz")
    status = [run,
              f"{source.mission} {source.phase}",
              f"{source.action} ID:{source.action_id} CAM:{source.camera}{'' if source.settled else '*'}",
              f"L{counts[0]} B{counts[1]} G{counts[2]} D{counts[3]} H{counts[4]} "
              f"age:{age:.1f}s OUT:{output_kbps:.1f}KB/s"]
    if frame is None:
        status[1] = "Enable publisher in line_perception_node"
        status[2] = "B:ball G:goal D:backboard H:hurdle .:line"
    border = "+" + "-" * w + "+"
    lines = [border] + [draw_row(row) for row in cells] + [border]
    lines += [ascii_text(text)[:w+2].ljust(w+2) for text in status]
    # Erase each row's tail and any unused rows when terminal size shrinks.
    return "\x1b[H" + "\r\n".join(line + "\x1b[K" for line in lines) + "\x1b[0m\x1b[J"
