#!/usr/bin/python3
"""Actual viewer transport/render loop, synthetic RGB-D, fake YOLO and executor."""
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision.msg import CameraCommand, CommandStatus


class InspectionTest(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[])
        self.ros = Node("synthetic_yolo_inspection_camera_executor")
        self.rgb = self.ros.create_publisher(Image, "/camera/camera/color/image_raw", qos_profile_sensor_data)
        self.depth = self.ros.create_publisher(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", qos_profile_sensor_data)
        self.status = self.ros.create_publisher(CommandStatus, "/jandi_vision/camera_status", 10)
        self.commands = []
        self.subscription = self.ros.create_subscription(
            CameraCommand, "/jandi_vision/camera_cmd", self.commands.append, 10)
        self.depth_offset = 0.
        self.depth_enabled = True
        self.rgb_enabled = True
        self.frame = 0
        self.timer = self.ros.create_timer(.05, self.publish)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()
        self.proc = None
        self.stream = None
        self.http_thread = None
        self.latest_frame = None
        self.last_read_frame = 0
        self.stream_error = None
        self.log = tempfile.TemporaryFile()

    def spin(self):
        while not self.stop.is_set():
            rclpy.spin_once(self.ros, timeout_sec=.02)

    def publish(self):
        if not self.rgb_enabled:
            return
        now = self.ros.get_clock().now()
        color = Image(width=640, height=480, encoding="bgr8", step=1920)
        pixels = np.full((480, 640, 3), 70, dtype=np.uint8)
        pixels[420:440, 550:570] = 90 + self.frame % 120
        color.data = pixels.tobytes()
        color.header.stamp = now.to_msg()
        color.header.frame_id = "synthetic_color_optical_frame"
        if self.depth_enabled:
            depth = Image(width=640, height=480, encoding="16UC1", step=1280)
            depth.data = np.full((480, 640), 1500, dtype=np.uint16).tobytes()
            depth.header.stamp = rclpy.time.Time(
                nanoseconds=now.nanoseconds + int(self.depth_offset * 1e9)).to_msg()
            depth.header.frame_id = color.header.frame_id
            self.depth.publish(depth)
        self.rgb.publish(color)
        self.frame += 1

    def start(self, executable):
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            self.port = sock.getsockname()[1]
        env = dict(os.environ)
        env.pop("DISPLAY", None)
        env.pop("WAYLAND_DISPLAY", None)
        # Omit stream/show flags to verify stream ON and debug OFF defaults.
        self.proc = subprocess.Popen([executable, "--ros-args", "-p", f"stream_debug_port:={self.port}"],
                                     env=env, stdout=self.log, stderr=subprocess.STDOUT)
        def connect():
            try:
                self.stream = urllib.request.urlopen(
                    f"http://127.0.0.1:{self.port}/stream.mjpg", timeout=3)
                return True
            except OSError:
                return False
        self.wait(connect, "HTTP stream failed to start")
        # Drain continuously like a browser. Pausing test assertions must not
        # trigger the server's intentional slow-client disconnection deadline.
        self.http_thread = threading.Thread(target=self.read_stream, daemon=True)
        self.http_thread.start()

    def wait(self, condition, message, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if condition():
                return
            if self.proc and self.proc.poll() is not None:
                self.log.seek(0)
                self.fail(self.log.read().decode(errors="replace"))
            time.sleep(.05)
        self.fail(message)

    def receive_image(self):
        headers = {}
        while True:
            line = self.stream.readline()
            if not line:
                raise ConnectionError("stream disconnected")
            if line == b"\r\n":
                break
            if b":" in line:
                key, value = line.decode().split(":", 1)
                headers[key.lower()] = value.strip()
        data = self.stream.read(int(headers["content-length"]))
        self.stream.read(2)
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        return image

    def read_stream(self):
        frame_id = 0
        try:
            while not self.stop.is_set():
                image = self.receive_image()
                frame_id += 1
                self.latest_frame = (frame_id, image)
        except Exception as error:
            if not self.stop.is_set():
                self.stream_error = error

    def image(self):
        def fresh():
            if self.stream_error:
                raise self.stream_error
            return self.latest_frame and self.latest_frame[0] > self.last_read_frame
        self.wait(fresh, "no new streamed frame")
        self.last_read_frame, image = self.latest_frame
        self.assertEqual(image.shape, (480, 640, 3))
        return image

    def wait_image(self, condition):
        result = [None]
        def check():
            result[0] = self.image()
            return condition(result[0])
        self.wait(check, "expected live image did not arrive")
        return result[0]

    @staticmethod
    def has_boxes(image):
        # A red board edge and orange ball edge, in original image coordinates.
        return image[240, 419, 2] > 140 and image[230, 30, 2] > 140

    @staticmethod
    def band_text_pixels(image):
        # Depth/ratio text occupies this area inside the board, away from borders.
        region = image[145:184, 225:375].astype(np.int16)
        return np.count_nonzero(np.max(region, axis=2) - np.min(region, axis=2) > 65)

    def feedback(self, command_id, status):
        message = CommandStatus(command_id=command_id, status=status)
        for _ in range(3):
            self.status.publish(message)
            time.sleep(.05)

    def assert_no_body_or_start(self, node_name):
        topics = self.ros.get_publisher_names_and_types_by_node(node_name, "/")
        self.assertNotIn("/jandi_vision/action_cmd", [name for name, _ in topics])
        services = self.ros.get_service_names_and_types_by_node(node_name, "/")
        self.assertFalse(any(name.endswith(("/start", "/commit", "/preview")) for name, _ in services))
        return topics

    def test_forward_preserves_camera_and_streams_all_boxes(self):
        self.start(sys.argv[1])
        self.wait_image(self.has_boxes)
        topics = self.assert_no_body_or_start("forward_yolo_node")
        self.assertNotIn("/jandi_vision/camera_cmd", [name for name, _ in topics])
        time.sleep(.7)
        self.assertEqual(self.commands, [])

    def test_goal_waits_for_real_done_and_rejects_stale_depth(self):
        self.start(sys.argv[2])
        self.wait(lambda: len(self.commands) >= 2, "GOAL camera was not retried")
        command_id = self.commands[0].command_id
        self.assertNotEqual(command_id, 0)
        self.assertTrue(all(command.request == CameraCommand.GOAL for command in self.commands))
        self.feedback(command_id, CommandStatus.ACK)
        self.feedback(command_id + 1, CommandStatus.DONE)
        before = self.wait_image(self.has_boxes)
        self.assertLess(self.band_text_pixels(before), 10, "bands appeared before matching DONE")
        count = len(self.commands)
        self.wait(lambda: len(self.commands) > count, "ACK incorrectly stopped DONE recovery")
        self.assertTrue(all(command.command_id == command_id for command in self.commands))
        self.feedback(command_id, CommandStatus.DONE)
        valid = self.wait_image(lambda image: self.has_boxes(image) and self.band_text_pixels(image) > 100)
        self.assert_no_body_or_start("goal_yolo_node")
        time.sleep(.2)
        count = len(self.commands)
        time.sleep(.7)
        self.assertEqual(len(self.commands), count, "camera commands continued after DONE")
        # Images keep updating, but depth older by 1 second must lose its numbers.
        self.depth_offset = -1.
        self.wait_image(lambda image: self.has_boxes(image) and self.band_text_pixels(image) < 10)
        self.depth_offset = 0.
        self.wait_image(lambda image: self.band_text_pixels(image) > 100)
        self.depth_enabled = False
        missing = self.wait_image(lambda image: self.has_boxes(image) and self.band_text_pixels(image) < 10)
        self.assertFalse(np.array_equal(valid, missing), "missing depth retained stale depth text")

    def tearDown(self):
        self.stop.set()
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
                self.fail("viewer shutdown hung")
        if self.http_thread:
            self.http_thread.join(timeout=4)
        if self.stream:
            self.stream.close()
        self.thread.join(timeout=2)
        self.ros.destroy_node()
        rclpy.shutdown()
        self.log.close()


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
