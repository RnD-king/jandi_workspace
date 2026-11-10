#!/usr/bin/python3
"""Hardware-free lifecycle tests. Child processes below are Python stubs only."""
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import jandi_runtime as runtime
import jandi_ros
import jandi as console


class Observer:
    def __init__(self):
        self.calls = []
        self.allowed = True
        self.logs = []

    def starting(self, name):
        pass

    def log(self, name, text):
        self.logs.append((name, text))

    def spin(self):
        pass

    def can_stop(self, name, nodes):
        return self.allowed, "동작 중"

    def snapshot(self, nodes):
        return {"ros_ready": dict.fromkeys(runtime.NAMES, False), "services": {},
                "run_state": "UNKNOWN", "pending_actions": [], "pending_cameras": []}

    def call(self, name):
        self.calls.append(name)
        return True, name


class Child:
    counter = 700000

    def __init__(self, command, **kwargs):
        Child.counter += 1
        self.pid = Child.counter
        self.command = command
        self.kwargs = kwargs
        self.code = None
        self.stdout = io.StringIO("")

    def poll(self):
        return self.code


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.now = 100.0
        self.observer = Observer()
        self.created = []

        def spawn(command, **kwargs):
            child = Child(command, **kwargs)
            self.created.append(child)
            return child

        self.manager = runtime.ProcessManager(
            {name: [name] for name in runtime.NAMES}, self.temp.name,
            self.observer, clock=lambda: self.now, popen=spawn)
        self.addCleanup(lambda: [setattr(child, "code", 0) for child in self.created])

    def advance(self, seconds):
        self.now += seconds
        self.manager.tick()

    def boot(self):
        self.advance(0)
        self.advance(5)
        self.advance(5)

    def test_exact_start_order_and_service_gate(self):
        self.advance(0)
        self.assertEqual([x.command[0] for x in self.created], ["camera"])
        self.advance(4.999)
        self.assertEqual(len(self.created), 1)
        self.advance(.001)
        self.assertEqual([x.command[0] for x in self.created], ["camera", "motor"])
        self.advance(5)
        self.assertEqual([x.command[0] for x in self.created], list(runtime.NAMES))
        self.assertFalse(self.manager.request({"op": "service", "name": "start"})["ok"])
        self.advance(5)
        self.assertEqual(self.observer.calls, [])
        self.assertTrue(self.manager.request({"op": "service", "name": "start"})["ok"])
        self.assertEqual(self.observer.calls, ["start"])
        for child in self.created:
            self.assertTrue(child.kwargs["start_new_session"])
            self.assertEqual(child.kwargs["stdin"], subprocess.DEVNULL)

    def test_failure_does_not_stop_restart_or_control_other_nodes(self):
        self.advance(0)
        self.created[0].code = 42
        self.advance(5)
        self.assertEqual(self.manager.nodes["camera"]["state"], "EXITED")
        self.advance(5)
        self.advance(100)
        self.assertEqual([x.command[0] for x in self.created], list(runtime.NAMES))
        self.assertEqual(self.observer.calls, [])
        self.assertTrue(any(e["level"] == "ERROR" for e in self.manager.events))
        self.assertEqual(self.manager.nodes["motor"]["state"], "RUNNING")

    def test_error_log_only_notifies(self):
        self.boot()
        self.manager.observe_line("vision", 1, "[ERROR] YOLO failed")
        self.manager.observe_line("vision", 1, "[FATAL] lost device")
        self.advance(50)
        self.assertEqual(self.observer.calls, [])
        self.assertEqual(len(self.created), 3)
        self.assertFalse(self.manager.shutdown_requested)
        self.assertTrue(any("YOLO failed" in e["text"] for e in self.manager.events))

    @patch("jandi_runtime.os.killpg")
    def test_manual_restart_affects_only_selected_node(self, kill):
        self.boot()
        self.manager.request({"op": "restart", "node": "motor"})
        kill.assert_called_once_with(self.created[1].pid, signal.SIGINT)
        self.advance(30)
        self.assertEqual(len(self.created), 3)  # no automatic kill or replacement while alive
        kill.assert_called_once()
        self.created[1].code = 0
        self.advance(0)
        self.assertEqual([x.command[0] for x in self.created], ["camera", "motor", "vision", "motor"])
        self.assertEqual(self.observer.calls, [])

    @patch("jandi_runtime.os.killpg")
    def test_busy_shutdown_and_restart_never_pause_or_signal(self, kill):
        self.boot()
        self.observer.allowed = False
        self.manager.request({"op": "restart", "node": "motor"})
        self.assertFalse(self.manager.request({"op": "shutdown"})["ok"])
        self.advance(100)
        kill.assert_not_called()
        self.assertEqual(self.observer.calls, [])

    @patch("jandi_runtime.os.killpg")
    def test_manual_vision_stop_does_not_interrupt_motor(self, kill):
        self.boot()
        self.observer.allowed = False
        self.assertTrue(self.manager.request({"op": "stop", "node": "vision"})["ok"])
        kill.assert_called_once_with(self.created[2].pid, signal.SIGINT)
        self.assertEqual(self.manager.nodes["motor"]["state"], "RUNNING")
        self.assertEqual(self.observer.calls, [])

    @patch("jandi_runtime.os.killpg")
    def test_manual_stop_cancels_pending_initial_start(self, kill):
        self.advance(0)
        self.manager.request({"op": "stop", "node": "motor"})
        self.advance(50)
        self.assertEqual([x.command[0] for x in self.created], ["camera", "vision"])
        kill.assert_not_called()

    @patch("jandi_runtime.os.killpg")
    def test_emergency_key_stops_all_groups_even_when_busy(self, kill):
        self.boot()
        self.observer.allowed = False
        for page in ("main", "logs", "vision", "motor", "shutdown"):
            confirm,request=console.key_request(page,"K")
            self.assertEqual((confirm,request),("emergency:"+page,None))
            self.assertEqual(console.key_request(confirm,"K"),(confirm,None))
            for cancel in ("0","\x1b"):
                self.assertEqual(console.key_request(confirm,cancel),(page,None))
            self.assertEqual(console.key_request(confirm,"1"),("main",{"op":"emergency_stop"}))
        result = self.manager.request({"op": "emergency_stop"})
        self.assertTrue(result["ok"])
        self.assertTrue(self.manager.shutdown_requested)
        self.assertTrue(self.manager.emergency_stop_requested)
        self.assertEqual(
            kill.call_args_list[:3],
            [unittest.mock.call(self.created[i].pid, signal.SIGINT) for i in (2, 1, 0)])
        self.assertEqual(self.observer.calls, [])
        self.assertFalse(self.manager.request({"op": "service", "name": "resume"})["ok"])
        self.advance(.25)
        self.assertEqual(
            kill.call_args_list[3:6],
            [unittest.mock.call(self.created[i].pid, signal.SIGTERM) for i in (2, 1, 0)])
        self.advance(.25)
        self.assertEqual(
            kill.call_args_list[6:9],
            [unittest.mock.call(self.created[i].pid, signal.SIGKILL) for i in (2, 1, 0)])
        for child in self.created:
            child.code = -signal.SIGKILL
        self.advance(.25)
        self.assertTrue(self.manager.finished)

    @patch("jandi_runtime.os.killpg")
    def test_manual_full_shutdown_orders_stops_without_services(self, kill):
        self.boot()
        self.assertTrue(self.manager.request({"op": "shutdown"})["ok"])
        for index in (2, 1, 0):
            self.advance(0)
            self.assertEqual(kill.call_args.args, (self.created[index].pid, signal.SIGINT))
            self.created[index].code = 0
        self.advance(0)
        self.assertTrue(self.manager.finished)
        self.assertEqual(self.observer.calls, [])

    def test_restart_vision_requires_its_fresh_five_second_gate(self):
        self.boot()
        self.advance(20)
        self.created[2].code = 1
        self.advance(0)
        self.manager.request({"op": "start", "node": "vision"})
        self.assertFalse(self.manager.request({"op": "service", "name": "resume"})["ok"])
        self.advance(5)
        self.assertTrue(self.manager.request({"op": "service", "name": "resume"})["ok"])

    def test_lost_console_and_status_polling_have_no_control_effect(self):
        self.boot()
        for _ in range(30):
            self.manager.request({"op": "status"})
            self.advance(1)
        self.assertEqual(self.observer.calls, [])
        self.assertEqual(len(self.created), 3)
        for page in ("main", "logs", "camera", "motor", "vision"):
            self.assertIsNone(console.key_request(page, "")[1])
        self.assertEqual(console.key_request("main", "9"), ("detach", None))
        self.assertIsNone(console.key_request("main", "0")[1])
        self.assertEqual(console.key_request("shutdown", "1")[1], {"op": "shutdown"})

    def test_plan_uses_explicit_options_and_shared_launch_without_yaml(self):
        with patch.dict(os.environ, {}, clear=True):
            plan = runtime.Plan.load(SCRIPTS.parent)
            self.assertEqual(plan.workspace, "/home/rnd/jandi")
            self.assertEqual(plan.engine, "/home/rnd/jandi/src/dataset/best_1029.engine")
            self.assertEqual(plan.environment()["ROS_DOMAIN_ID"], "22")
        a = plan.commands()
        self.assertIn("terminal_node:=false",a["vision"])
        enabled = runtime.Plan.load(SCRIPTS.parent,terminal_node=True)
        self.assertIn("terminal_node:=true",enabled.commands()["vision"])
        self.assertEqual(a["camera"],["ros2","launch",plan.camera_launch])
        self.assertNotIn("config_file:=", " ".join(a["camera"]))
        self.assertNotIn("mode",plan.__dict__)
        for service,path in runtime.SERVICES.items():
            self.assertIn(f"{service}:={path}",a["vision"])

    def test_survivor_pid_reuse_and_private_directory(self):
        path = Path(self.temp.name) / "state.json"
        identity = runtime.process_identity(os.getpid())
        path.write_text(json.dumps({"nodes": {"motor": {"pid": os.getpid(), "identity": identity}}}))
        with self.assertRaises(RuntimeError):
            runtime.require_no_survivors(path)
        path.write_text(json.dumps({"nodes": {"motor": {"pid": os.getpid(), "identity": "not-this-process"}}}))
        runtime.require_no_survivors(path)
        link = Path(self.temp.name) / "link"
        link.symlink_to(self.temp.name)
        with self.assertRaises(RuntimeError):
            runtime.private_directory(link)

    def test_existing_operator_paths_are_preserved(self):
        with patch.dict(os.environ, {"YOLO26_ENGINE_PATH": "/models/custom.engine",
                                    "VISION_CORE_ROOT": "/opt/custom-core",
                                    "VISION_CORE_ALGORITHM_CONFIG": "/config/operator.yaml"}):
            plan = runtime.Plan.load(SCRIPTS.parent)
        self.assertEqual(plan.engine, "/models/custom.engine")
        self.assertEqual(plan.workspace, "/home/rnd/jandi")
        self.assertEqual(plan.core_prefix, "/opt/custom-core")
        self.assertEqual(plan.algorithm, "/config/operator.yaml")

    def test_real_fake_child_survives_console_disconnect_and_error_message(self):
        # Run a daemon with a pure fake observer and Python stubs, never ROS/hardware.
        stub = "import signal,time; signal.signal(signal.SIGINT,lambda *_:exit(0)); print('[ERROR] simulated',flush=True); time.sleep(30)"
        driver = f"""
import sys
sys.path[:0] = [{str(SCRIPTS)!r}, {str(Path(__file__).parent)!r}]
import jandi_runtime as r
from jandi_runtime_test import Observer
class FakePlan:
    workspace={self.temp.name!r}
    def __init__(self): self.mode='practice'
    def commands(self): return {{n:[sys.executable,'-c',{stub!r}] for n in r.NAMES}}
    def environment(self): return dict(__import__('os').environ)
r.serve(FakePlan(), {self.temp.name!r}, Observer())
"""
        proc = subprocess.Popen([sys.executable, "-c", driver], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                start_new_session=True)
        try:
            deadline = time.monotonic() + 5
            while True:
                try:
                    status = runtime.rpc(self.temp.name, {"op": "status"})["status"]
                    break
                except OSError:
                    if time.monotonic() > deadline:
                        self.fail("fake server did not start")
                    time.sleep(.05)
            pid = status["nodes"]["camera"]["pid"]
            self.assertIsNotNone(pid)
            # A dropped client, HUP and terminal error output must not kill this child.
            import socket
            client = socket.socket(socket.AF_UNIX)
            client.connect(str(Path(self.temp.name) / "manager.sock"))
            client.close()
            for sig in (signal.SIGHUP,signal.SIGINT,signal.SIGTERM):
                proc.send_signal(sig)
                time.sleep(.1)
            status = runtime.rpc(self.temp.name, {"op": "status"})["status"]
            self.assertEqual(status["nodes"]["camera"]["pid"], pid)
            self.assertEqual(status["nodes"]["camera"]["state"], "RUNNING")
            self.assertTrue(any(e["level"] == "ERROR" for e in status["events"]))
            # Exercise the actual menu under a PTY: one key, no Enter, then detach.
            import pty
            import select
            master, slave = pty.openpty()
            menu = subprocess.Popen([sys.executable, str(SCRIPTS / "jandi.py"), "attach",
                                     "--state-dir", self.temp.name],
                                    stdin=slave, stdout=slave, stderr=slave)
            os.close(slave)
            try:
                output = b""
                deadline = time.monotonic() + 3
                while b"JANDI | VERSION=" not in output:
                    if select.select([master], [], [], .1)[0]:
                        output += os.read(master, 65536)
                    if time.monotonic() > deadline:
                        self.fail(f"numeric menu did not render: {output!r}")
                os.write(master, b"9")
                menu.wait(timeout=3)
                self.assertEqual(menu.returncode, 0)
            finally:
                if menu.poll() is None:
                    menu.kill()
                    menu.wait()
                os.close(master)
            status = runtime.rpc(self.temp.name, {"op": "status"})["status"]
            self.assertEqual(status["nodes"]["camera"]["pid"], pid)
            # A console killed without cleanup must also leave the detached
            # backend and its fake child alive.
            master,slave=pty.openpty()
            lost=subprocess.Popen([sys.executable,str(SCRIPTS/"jandi.py"),"attach",
                                   "--state-dir",self.temp.name],stdin=slave,stdout=slave,stderr=slave)
            os.close(slave)
            try:
                time.sleep(.15)
                lost.kill()
                lost.wait(timeout=3)
            finally:
                os.close(master)
                if lost.poll() is None:
                    lost.kill()
                    lost.wait()
            self.assertEqual(runtime.rpc(self.temp.name,{"op":"status"})["status"]["nodes"]["camera"]["pid"],pid)
            runtime.rpc(self.temp.name, {"op": "shutdown"})
            proc.wait(timeout=5)
            self.assertEqual(proc.returncode, 0, proc.stderr.read().decode())
        finally:
            if proc.poll() is None:
                # Test cleanup only: the target is an isolated Python stub.
                state = Path(self.temp.name) / "state.json"
                if state.exists():
                    for item in json.loads(state.read_text()).get("nodes", {}).values():
                        if item.get("pid") and runtime.process_identity(item["pid"]) == item.get("identity"):
                            os.killpg(item["pid"], signal.SIGKILL)
                proc.kill()
                proc.wait()
            proc.stdout.close()
            proc.stderr.close()


class ObservationTest(unittest.TestCase):
    def test_service_timeout_is_reported_without_resend(self):
        observer = jandi_ros.RosObserver.__new__(jandi_ros.RosObserver)
        observer.run_state = "RUNNING"
        observer.may_issue_commands = True
        observer.rclpy = Mock()
        observer.node = Mock()
        observer.trigger = Mock()
        client = Mock()
        client.service_is_ready.return_value = True
        future = client.call_async.return_value
        future.done.return_value = False
        observer.clients = {"pause": client}
        ok, message = observer.call("pause")
        self.assertFalse(ok)
        self.assertIn("시간 초과", message)
        self.assertEqual(observer.run_state, "UNKNOWN")
        client.call_async.assert_called_once()
        client.remove_pending_request.assert_called_once_with(future)

    def test_pause_completion_received_during_call_is_not_overwritten(self):
        observer = jandi_ros.RosObserver.__new__(jandi_ros.RosObserver)
        observer.run_state = "RUNNING"
        observer.may_issue_commands = True
        observer.node = Mock()
        observer.trigger = Mock()
        observer.rclpy = Mock()
        observer.rclpy.spin_until_future_complete.side_effect = lambda *a, **k: observer.log(
            "vision", "[CONTROL] PAUSED: all published motions and camera commands completed")
        client = Mock()
        client.service_is_ready.return_value = True
        client.call_async.return_value.done.return_value = True
        response = client.call_async.return_value.result.return_value
        response.success, response.message = True, "PAUSING"
        observer.clients = {"pause": client}
        self.assertTrue(observer.call("pause")[0])
        self.assertEqual(observer.run_state, "PAUSED")
        self.assertFalse(observer.may_issue_commands)

    def test_busy_and_unknown_are_not_safe_to_terminate(self):
        observer = jandi_ros.RosObserver.__new__(jandi_ros.RosObserver)
        observer.pending = {"action": {4}, "camera": set()}
        observer.done = {"action": set(), "camera": set()}
        observer.motor_initialized = True
        observer.run_state = "PAUSED"
        observer.may_issue_commands = False
        nodes = {"vision": {"process": Child(["vision"])}}
        self.assertFalse(observer.can_stop("motor", nodes)[0])
        observer.pending["action"].clear()
        self.assertTrue(observer.can_stop("motor", nodes)[0])
        observer.may_issue_commands = True
        self.assertFalse(observer.can_stop("motor", nodes)[0])
        observer.log("vision", "[CONTROL] PAUSED: all published motions and camera commands completed")
        self.assertTrue(observer.can_stop("motor", nodes)[0])
        observer.run_state = "UNKNOWN"
        self.assertFalse(observer.can_stop("vision", nodes)[0])


if __name__ == "__main__":
    unittest.main()
