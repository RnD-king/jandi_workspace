#!/usr/bin/env python3
"""Run on Jetson inside SSH; only ASCII/ANSI output crosses the SSH link."""
from __future__ import annotations

import math
import shutil
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from vision.msg import TerminalFrame
from terminal_view import Box, Frame, render


class VisionTerminalNode(Node):
    def __init__(self):
        super().__init__("vision_terminal_node")
        descriptor = ParameterDescriptor(read_only=True)

        def parameter(name, default):
            return self.declare_parameter(name, default, descriptor).value

        topic = parameter("terminal_topic", "/jandi_vision/terminal")
        self.hz = parameter("refresh_hz", 5.0)
        self.stale_after = parameter("stale_timeout_sec", 2.0)
        self.grid_columns = parameter("grid_columns", 48)
        self.color = parameter("color", True)
        if not math.isfinite(self.hz) or not 0 < self.hz <= 60:
            raise ValueError("refresh_hz must be in (0,60]")
        if not math.isfinite(self.stale_after) or self.stale_after <= 0:
            raise ValueError("stale_timeout_sec must be positive")
        if not 8 <= self.grid_columns <= 300:
            raise ValueError("grid_columns must be in [8,300]")
        self.frame = None
        self.received = 0.0
        self.arrivals = deque(maxlen=600)
        self.output = deque(maxlen=600)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(TerminalFrame, topic, self.on_frame, qos)

    def on_frame(self, message):
        if message.image_width <= 0 or message.image_height <= 0:
            return
        self.received = time.monotonic()
        self.arrivals.append(self.received)
        self.frame = Frame(
            width=message.image_width, height=message.image_height,
            run=message.control_state, mission=message.mission, phase=message.phase,
            action=message.action_name, action_id=message.waiting_action_id,
            camera=message.camera, settled=message.camera_settled,
            filters_current=message.filters_current,
            boxes=[Box(b.kind, b.x, b.y, b.width, b.height, b.filtered)
                   for b in message.detections])

    def draw(self):
        now = time.monotonic()
        while self.arrivals and self.arrivals[0] < now - 2:
            self.arrivals.popleft()
        while self.output and self.output[0][0] < now - 2:
            self.output.popleft()
        rx_hz = ((len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])
                 if len(self.arrivals) > 1 and self.arrivals[-1] > self.arrivals[0] else 0)
        output_rate = (sum(size for _, size in self.output) /
                       max(0.2, now - self.output[0][0]) / 1000 if self.output else 0)
        size = shutil.get_terminal_size(fallback=(80, 24))
        text = render(self.frame, size.columns, size.lines, desired=self.grid_columns,
                      age=now-self.received if self.frame else 0,
                      stale_after=self.stale_after, rx_hz=rx_hz,
                      output_kbps=output_rate, color=self.color)
        sys.stdout.write(text)
        sys.stdout.flush()
        self.output.append((now, len(text.encode("ascii"))))


def main(args=None):
    if not sys.stdout.isatty():
        print("vision_terminal_node requires an interactive terminal (SSH: use ssh -t).", file=sys.stderr)
        return 1
    node = None
    screen_entered = False
    try:
        rclpy.init(args=args)
        node = VisionTerminalNode()
        sys.stdout.write("\x1b[?1049h\x1b[?25l\x1b[2J\x1b[H")
        sys.stdout.flush()
        screen_entered = True
        next_draw = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            if now >= next_draw:
                node.draw()
                next_draw = time.monotonic() + 1.0 / node.hz
            rclpy.spin_once(node, timeout_sec=max(0, min(.05, next_draw-time.monotonic())))
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as error:
        print(f"vision_terminal_node: {error}", file=sys.stderr)
        return 1
    finally:
        if screen_entered:
            sys.stdout.write("\x1b[0m\x1b[?25h\x1b[?1049l")
            sys.stdout.flush()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
