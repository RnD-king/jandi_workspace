#!/usr/bin/python3
"""Actual ROS graph/Trigger/message tests with synthetic peers, no hardware."""
from pathlib import Path
import sys
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from jandi_ros import RosObserver
from jandi_runtime import Plan, SERVICES
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from vision.msg import ActionCommand, CameraCommand, CommandStatus


class RosTest(unittest.TestCase):
    def test_discovery_manual_services_and_real_done_tracking(self):
        observer = RosObserver(str(ROOT / "config/vision_params.yaml"))
        params = Plan.load(ROOT).commands()["vision"]
        remaps = []
        for index, arg in enumerate(params):
            if arg == "-r":
                remaps += ["-r", params[index + 1]]
        vision = Node("line_perception_node", cli_args=["--ros-args"] + remaps)
        motor = Node("main_node")
        camera = Node("synthetic_camera")
        executor = MultiThreadedExecutor(num_threads=2)
        for node in (vision, motor, camera):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        calls = []
        references = []
        try:
            for name, path in SERVICES.items():
                def callback(request, response, n=name):
                    calls.append(n)
                    response.success = True
                    response.message = "synthetic " + n
                    return response
                references.append(vision.create_service(Trigger, name, callback))
                self.assertEqual(vision.resolve_service_name(name), path)
            action = vision.create_publisher(ActionCommand, observer.topics["action_cmd_topic"], 10)
            status = motor.create_publisher(CommandStatus, observer.topics["action_status_topic"], 10)
            cam_command = vision.create_publisher(CameraCommand, observer.topics["camera_cmd_topic"], 10)
            cam_status = motor.create_publisher(CommandStatus, observer.topics["camera_status_topic"], 10)
            references.append(motor.create_subscription(ActionCommand, observer.topics["action_cmd_topic"], lambda msg: None, 10))
            info = camera.create_publisher(CameraInfo, observer.topics["camera_info_topic"], qos_profile_sensor_data)
            references += [camera.create_publisher(Image, observer.topics[key], qos_profile_sensor_data)
                           for key in ("image_topic", "depth_topic")]
            observer.log("motor", "Startup WALK_MODE pose reached and held; ready for command topics")
            observer.log("vision", "vision node DISARMED. Services:")
            deadline = time.monotonic() + 8
            while not all(observer.graph.values()) or not observer.info_at:
                info.publish(CameraInfo())
                observer.spin()
                if time.monotonic() > deadline:
                    self.fail(f"synthetic ROS discovery timeout: {observer.graph}")
                time.sleep(.02)
            self.assertEqual(calls, [])  # readiness alone is read-only
            self.assertTrue(observer.call("start")[0])
            self.assertEqual(calls, ["start"])
            self.assertTrue(observer.may_issue_commands)
            message = ActionCommand(action_id=42, mission=1, action=33)
            action.publish(message)
            cam_command.publish(CameraCommand(command_id=12, request=1))
            deadline = time.monotonic() + 3
            while observer.pending != {"action": {42}, "camera": {12}}:
                observer.spin()
                if time.monotonic() > deadline:
                    self.fail(f"command observation timeout: {observer.pending}")
                time.sleep(.01)
            status.publish(CommandStatus(command_id=42, status=CommandStatus.READY))
            observer.spin()
            self.assertIn(42, observer.pending["action"])
            status.publish(CommandStatus(command_id=42, status=CommandStatus.DONE))
            cam_status.publish(CommandStatus(command_id=12, status=CommandStatus.DONE))
            deadline = time.monotonic() + 3
            while any(observer.pending.values()):
                observer.spin()
                if time.monotonic() > deadline:
                    self.fail("DONE observation timeout")
                time.sleep(.01)
            self.assertEqual(calls, ["start"])
        finally:
            executor.shutdown(timeout_sec=2)
            thread.join(timeout=2)
            for node in (vision, motor, camera, observer.node):
                node.destroy_node()
            observer.rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
