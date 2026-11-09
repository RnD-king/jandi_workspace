"""Lossless log journal and incremental, paged console reader. No ROS/control."""
from array import array
import json
from pathlib import Path
import re
import threading
import time


FILTERS = {
    "1": (None, None, "전체 모든 로그"),
    "2": (None, {"WARN", "ERROR", "FATAL"}, "전체 경고·오류"),
    "3": ("camera", {"ERROR", "FATAL", "WARN"}, "카메라 오류·경고"),
    "4": ("motor", {"ERROR", "FATAL", "WARN"}, "모터 오류·경고"),
    "5": ("vision", {"ERROR", "FATAL", "WARN"}, "비전 오류·경고"),
    "6": ("camera", None, "카메라 모든 로그"),
    "7": ("motor", None, "모터 모든 로그"),
    "8": ("vision", None, "비전 모든 로그"),
}


class LogClassifier:
    def __init__(self):
        self.exception = False

    def classify(self, text):
        tag = re.search(r"\[(DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL)\]", text, re.I)
        if tag:
            level = tag[1].upper().replace("WARNING", "WARN")
            self.exception = level in ("ERROR", "FATAL")
            return level
        if re.search(r"Traceback \(|terminate called|uncaught|\b(?:\w*Error|\w*Exception)\s*:|"
                     r"Segmentation fault|core dumped|process has died|what\(\)", text, re.I):
            self.exception = True
            return "ERROR"
        # Preserve traceback/context lines until the next explicit log level.
        if self.exception:
            return "ERROR"
        if re.search(r"\b(?:WARNING|WARN)\b", text):
            return "WARN"
        return "INFO"


class LogJournal:
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()

    def write(self, node, level, text):
        record = {"time": time.strftime("%H:%M:%S"), "node": node,
                  "level": level, "text": text}
        # Full original text is retained; the screen sanitizes only when drawing.
        try:
            with self.lock, self.path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError:
            pass  # A diagnostic write failure must never stop/control a child.


class LogView:
    def __init__(self, directory, selection):
        self.path = Path(directory) / "all.jsonl"
        self.selection = selection
        self.offset = 0
        self.matches = array("Q")  # Index offsets, not entire log contents in RAM.
        self.end = None

    def poll(self):
        if not self.path.exists():
            return
        if self.path.stat().st_size < self.offset:
            self.offset = 0
            self.matches = array("Q")
            self.end = None
        node, levels, _ = FILTERS[self.selection]
        with self.path.open("rb") as stream:
            stream.seek(self.offset)
            # Bound each UI refresh; subsequent refreshes continue from here.
            budget = 256 * 1024
            while budget > 0:
                position = stream.tell()
                line = stream.readline()
                if not line.endswith(b"\n"):
                    break
                self.offset = stream.tell()
                budget -= len(line)
                try:
                    record = json.loads(line)
                except (ValueError, UnicodeError):
                    continue
                if (node is None or record["node"] == node) and (
                        levels is None or record["level"] in levels):
                    self.matches.append(position)

    def move(self, direction, page_size=14):
        current = len(self.matches) if self.end is None else self.end
        if direction == "p":
            self.end = max(min(page_size, len(self.matches)), current - page_size)
        elif direction == "n":
            end = min(len(self.matches), current + page_size)
            self.end = None if end == len(self.matches) else end
        elif direction == "f":
            self.end = None

    def lines(self, page_size=14):
        self.poll()
        end = len(self.matches) if self.end is None else min(self.end, len(self.matches))
        start = max(0, end - page_size)
        records = []
        if self.path.exists():
            with self.path.open("rb") as stream:
                for offset in self.matches[start:end]:
                    stream.seek(offset)
                    records.append(json.loads(stream.readline()))
        return records, f"{start + 1 if end else 0}–{end} / {len(self.matches)}행"
