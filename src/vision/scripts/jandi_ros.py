"""Read-only ROS observation plus four explicitly requested Trigger calls."""
import re
import time

from jandi_runtime import SERVICES


class RosObserver:
    def __init__(self, params_path):
        import rclpy
        import yaml
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo
        from std_srvs.srv import Trigger
        from vision.msg import ActionCommand, CameraCommand, CommandStatus
        self.rclpy = rclpy
        self.trigger = Trigger
        rclpy.init(args=[])
        self.node = rclpy.create_node("jandi_runtime_monitor")
        self.clients = {name: self.node.create_client(Trigger, path)
                        for name, path in SERVICES.items()}
        with open(params_path) as stream:
            params = yaml.safe_load(stream)["line_perception_node"]["ros__parameters"]
        self.topics = {key: params[key] for key in (
            "image_topic", "depth_topic", "camera_info_topic", "action_cmd_topic",
            "action_status_topic", "camera_cmd_topic", "camera_status_topic")}
        self.pending = {"action": set(), "camera": set()}
        self.done = {"action": set(), "camera": set()}
        self.run_state = "UNKNOWN"
        self.may_issue_commands = False
        self.motor_initialized = False
        self.info_at = 0.0
        self.graph = {"camera": False, "motor": False, "vision": False}
        self.next_graph = 0.0
        self.services = {name: False for name in SERVICES}
        self.subscriptions = []
        for kind, cmd_type, topic_key in (("action", ActionCommand, "action_cmd_topic"),
                                          ("camera", CameraCommand, "camera_cmd_topic")):
            self.subscriptions.append(self.node.create_subscription(
                cmd_type, self.topics[topic_key],
                lambda msg, k=kind: self.command(k, msg), 100))
            self.subscriptions.append(self.node.create_subscription(
                CommandStatus, self.topics[f"{kind}_status_topic"],
                lambda msg, k=kind: self.status(k, msg), 100))
        # CameraInfo is small. No extra RGB/depth subscriptions or inference.
        self.subscriptions.append(self.node.create_subscription(
            CameraInfo, self.topics["camera_info_topic"],
            lambda msg: setattr(self, "info_at", time.monotonic()), qos_profile_sensor_data))

    def starting(self, name):
        if name == "vision":
            self.run_state = "UNKNOWN"
            self.may_issue_commands = False
        if name == "motor":
            self.motor_initialized = False
            # A user-started new executor owns a new completed-ID history.
            # Never discard unfinished IDs or manufacture a completion.
            if not any(self.pending.values()):
                self.done = {"action": set(), "camera": set()}
        self.next_graph = 0.0

    def command(self, kind, msg):
        value = msg.action_id if kind == "action" else msg.command_id
        # SCAN_HOLD/RESUME/RETURN share the original camera transaction ID.
        if value and value not in self.done[kind]:
            self.pending[kind].add(value)

    def status(self, kind, msg):
        if msg.status == msg.DONE and msg.command_id:
            self.pending[kind].discard(msg.command_id)
            self.done[kind].add(msg.command_id)
            if len(self.done[kind]) > 1024:
                # IDs are monotonically assigned across physical RESET.
                self.done[kind] = set(sorted(self.done[kind])[-512:])

    def log(self, name, text):
        if name == "motor" and "ready for command topics" in text:
            self.motor_initialized = True
        if name == "vision":
            state = re.search(r"\[STATE\].*\brun=(\w+)", text)
            if state:
                self.run_state = state.group(1)
            if "vision node DISARMED" in text or "-> WAIT_START" in text:
                self.run_state = "WAIT_START"
            elif "[CONTROL] PAUSED:" in text:
                self.run_state = "PAUSED"
                self.may_issue_commands = False
            elif "[CONTROL] PAUSING" in text:
                self.run_state = "PAUSING"
            elif "[CONTROL] RESUME -> OBSERVING" in text:
                self.run_state = "OBSERVING"
            elif "[CONTROL] START accepted" in text:
                self.run_state = "OBSERVING"
            if "[RESET]" in text and "DONE -> WAIT_START" in text:
                self.may_issue_commands = False

    def spin(self):
        # Drain command/status traffic rather than processing just one callback
        # per process-manager tick. No camera image subscriptions are added.
        for _ in range(32):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
        if time.monotonic() < self.next_graph:
            return
        self.next_graph = time.monotonic() + 1.0
        nodes = {ns.rstrip("/") + "/" + name
                 for name, ns in self.node.get_node_names_and_namespaces()}
        self.services = {name: client.service_is_ready() for name, client in self.clients.items()}
        self.graph = {
            "camera": all(self.node.count_publishers(self.topics[key]) > 0
                          for key in ("image_topic", "depth_topic", "camera_info_topic")),
            "motor": "/main_node" in nodes and self.motor_initialized and any(
                endpoint.node_name == "main_node" for endpoint in
                self.node.get_subscriptions_info_by_topic(self.topics["action_cmd_topic"])),
            "vision": "/line_perception_node" in nodes and all(self.services.values()),
        }

    def snapshot(self, nodes):
        ready = {name: value and nodes[name]["state"] == "RUNNING"
                 for name, value in self.graph.items()}
        ready["camera"] = ready["camera"] and bool(self.info_at) and time.monotonic() - self.info_at < 2.0
        return {"ros_ready": ready, "services": dict(self.services),
                "run_state": self.run_state,
                "pending_actions": sorted(self.pending["action"]),
                "pending_cameras": sorted(self.pending["camera"]),
                "camera_info_age": round(time.monotonic() - self.info_at, 1) if self.info_at else None}

    def can_stop(self, name, nodes):
        if self.pending["action"] or self.pending["camera"]:
            return False, "아직 DONE을 받지 못한 몸/카메라 명령이 있습니다"
        if name == "motor" and not self.motor_initialized:
            return False, "모터 초기 자세 완료를 확인하지 못했습니다"
        vision = nodes["vision"]["process"]
        if vision is not None and vision.poll() is None and (
                self.may_issue_commands or self.run_state not in ("WAIT_START", "PAUSED")):
            return False, f"비전 상태 {self.run_state}: 새 명령이 나올 수 있습니다"
        return True, ""

    def call(self, name):
        client = self.clients[name]
        if not client.service_is_ready():
            return False, f"서비스 준비 안 됨: {SERVICES[name]}"
        previous = self.run_state, self.may_issue_commands
        self.may_issue_commands = True
        self.run_state = {"start": "OBSERVING", "resume": "OBSERVING",
                          "pause": "PAUSING", "reset": "RESET_FEET_CAMERA"}[name]
        future = client.call_async(self.trigger.Request())
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        if not future.done():
            # Do not resend: the server might have accepted this one request.
            client.remove_pending_request(future)
            self.may_issue_commands = True
            self.run_state = "UNKNOWN"
            return False, "응답 대기 시간 초과. 실행 여부 불명, 자동 재전송하지 않습니다."
        try:
            response = future.result()
            if not response.success:
                self.run_state, self.may_issue_commands = previous
            return response.success, response.message
        except Exception as exc:
            return False, f"서비스 응답 오류: {exc}"
