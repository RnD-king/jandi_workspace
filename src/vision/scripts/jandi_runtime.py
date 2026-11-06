"""Process ownership for JANDI. No automatic mission service calls or recovery."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time


NAMES = ("camera", "motor", "vision")
MODES = ("practice", "IRC")
INTERVAL = 5.0
SERVICES = {name: f"/line_perception_node/{name}"
            for name in ("start", "pause", "resume", "reset")}


def clean_text(text):
    text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", text)
    return "".join(c for c in text if c == "\t" or c.isprintable())[:4096]


@dataclass(frozen=True)
class Plan:
    mode: str
    workspace: str
    core_prefix: str
    share: str
    engine: str
    algorithm: str

    @classmethod
    def load(cls, mode, share, workspace=None, core_prefix=None):
        import yaml
        if mode not in MODES:
            raise ValueError("mode must be practice or IRC (case sensitive)")
        share = Path(share).resolve()
        runtime = yaml.safe_load((share / "config/yolo26_runtime.yaml").read_text())
        configured_engine = Path(runtime["yolo26_runtime"]["ros__parameters"]["jetson_engine_path"])
        engine = Path(os.environ.get("YOLO26_ENGINE_PATH") or configured_engine).expanduser()
        # The existing engine setting is /home/rnd/jandi/src/dataset/*.engine.
        default_workspace = configured_engine.parent.parent.parent
        workspace = Path(workspace or default_workspace).expanduser().resolve()
        core = Path(core_prefix or os.environ.get("VISION_CORE_ROOT", "~/vision_core/install"))
        core = core.expanduser().resolve()
        algorithm = Path(os.environ.get("VISION_CORE_ALGORITHM_CONFIG") or
                         core / "share/shared_vision_core/config/vision_algorithm.yaml")
        return cls(mode, str(workspace), str(core), str(share), str(engine),
                   str(algorithm.expanduser().resolve()))

    def commands(self):
        params = str(Path(self.share) / "config/vision_params.yaml")
        vision = ["ros2", "run", "vision", "line_perception_node", "--ros-args",
                  "--params-file", params, "-p", f"engine_path:={self.engine}",
                  "-p", "show_debug_view:=false", "-p", "show_yolo_debug_view:=false",
                  "-p", "rosbag:=false", "-p", "simulate_camera_feedback:=false",
                  "-p", "simulate_decision:=false", "-p", "enable_command_transport:=true",
                  "-p", f"terminal_node:={'true' if self.mode == 'practice' else 'false'}"]
        # These services are relative names in the C++ node, not ~/private names.
        for name, destination in SERVICES.items():
            vision += ["-r", f"{name}:={destination}"]
        return {
            "camera": ["ros2", "launch", "realsense2_camera", "rs_launch.py"],
            "motor": ["ros2", "run", "forward_walk", "main_node"],
            "vision": vision,
        }

    def environment(self):
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = "22"
        env["VISION_CORE_ROOT"] = self.core_prefix
        env["VISION_CORE_ALGORITHM_CONFIG"] = self.algorithm
        env["LD_LIBRARY_PATH"] = str(Path(self.core_prefix) / "lib") + (
            ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
        env["PYTHONUNBUFFERED"] = "1"
        env["RCUTILS_COLORIZED_OUTPUT"] = "0"
        return env


def process_identity(pid):
    """Linux start ticks prevent stale state from addressing a reused PID."""
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return None if fields[0] == "Z" else fields[19]
    except (OSError, IndexError):
        return None


def require_no_survivors(path):
    if not path.exists():
        return
    state = json.loads(path.read_text())
    survivors = [name for name, item in state.get("nodes", {}).items()
                 if item.get("pid") and item.get("identity") is not None and
                 process_identity(item["pid"]) == item["identity"]]
    if survivors:
        raise RuntimeError("기존 관리자가 종료됐지만 프로세스가 남아 있습니다: " +
                           ", ".join(survivors) + "; 중복 실행하지 않습니다. state.json을 확인하세요.")


class ProcessManager:
    def __init__(self, commands, directory, observer, env=None, cwd=None,
                 clock=time.monotonic, popen=subprocess.Popen):
        self.commands = commands
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.observer = observer
        self.env = env
        self.cwd = cwd
        self.clock = clock
        self.popen = popen
        self.begun = clock()
        self.schedule = dict(zip(NAMES, (0.0, INTERVAL, 2 * INTERVAL)))
        self.nodes = {name: {"process": None, "pid": None, "identity": None,
                             "state": "WAITING", "exit_code": None,
                             "restart": False, "started": None, "generation": 0}
                      for name in NAMES}
        self.events = deque(maxlen=80)
        self.event_id = 0
        self.lock = threading.RLock()
        self.shutdown_requested = False
        self.finished = False
        self.event("manager", "INFO", "자동 중지/서비스 발송/재시작 없음. SSH 종료 시 실행 유지.")

    def event(self, name, level, text):
        with self.lock:
            self.event_id += 1
            self.events.append({"id": self.event_id, "time": time.strftime("%H:%M:%S"),
                                "node": name, "level": level, "text": clean_text(text)})

    def observe_line(self, name, generation, line):
        text = clean_text(line.rstrip())
        with self.lock:
            if generation != self.nodes[name]["generation"]:
                return
            self.observer.log(name, text)
            if re.search(r"\[(?:ERROR|FATAL)\]|Traceback \(most recent", text, re.IGNORECASE):
                self.event(name, "ERROR", text)
            elif re.search(r"\[WARN(?:ING)?\]", text, re.IGNORECASE):
                self.event(name, "WARN", text)

    def follow_output(self, name, generation, path, offset, proc):
        # Children write directly to regular files. If the manager itself
        # crashes, no closed stdout PIPE can kill a robot node with SIGPIPE.
        try:
            with path.open(errors="replace") as log:
                log.seek(offset)
                while generation == self.nodes[name]["generation"]:
                    line = log.readline()
                    if line:
                        self.observe_line(name, generation, line)
                    elif proc.poll() is not None:
                        break
                    else:
                        time.sleep(0.1)
        except OSError as exc:
            self.event(name, "ERROR", f"로그 읽기 오류: {exc}")

    def start(self, name):
        item = self.nodes[name]
        self.schedule.pop(name, None)
        if item["process"] is not None and item["process"].poll() is None:
            return f"{name}: 이미 실행 중"
        self.observer.starting(name)
        item.update(state="STARTING", exit_code=None, restart=False)
        item["generation"] += 1
        try:
            path = self.directory / f"{name}.log"
            with path.open("a") as log:
                offset = log.tell()
                proc = self.popen(self.commands[name], stdin=subprocess.DEVNULL,
                                  stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True, env=self.env, cwd=self.cwd)
        except OSError as exc:
            item.update(state="FAILED", process=None, pid=None, identity=None)
            self.event(name, "ERROR", f"실행 실패: {exc}")
            return f"{name}: 실행 실패"
        item.update(process=proc, pid=proc.pid, identity=process_identity(proc.pid),
                    state="RUNNING", started=self.clock())
        threading.Thread(target=self.follow_output,
                         args=(name, item["generation"], path, offset, proc),
                         daemon=True).start()
        self.event(name, "INFO", f"실행 PID={proc.pid}")
        return f"{name}: 실행 요청 완료"

    def stop(self, name, restart=False):
        self.schedule.pop(name, None)
        item = self.nodes[name]
        proc = item["process"]
        if proc is None or proc.poll() is not None:
            item["state"] = "STOPPED"
            return self.start(name) if restart else f"{name}: 종료 상태"
        if name == "motor":
            safe, reason = self.observer.can_stop(name, self.nodes)
            if not safe:
                return f"종료 보류: {reason}. 필요하면 5번 PAUSE를 직접 요청하세요."
        if item["state"] == "STOPPING":
            return f"{name}: 종료 대기 중 (강제 종료하지 않음)"
        # User-requested graceful SIGINT only. No timeout escalation to TERM/KILL.
        os.killpg(proc.pid, signal.SIGINT)
        item.update(state="STOPPING", restart=restart)
        self.event(name, "INFO", "사용자 재시작 요청" if restart else "사용자 종료 요청")
        return f"{name}: 정상 종료 대기"

    def tick(self):
        with self.lock:
            for name, item in self.nodes.items():
                proc = item["process"]
                if proc is None:
                    continue
                code = proc.poll()
                if code is None:
                    continue
                expected = item["state"] == "STOPPING"
                restart = item["restart"]
                item.update(process=None, state="STOPPED" if expected else "EXITED",
                            exit_code=code, restart=False)
                self.event(name, "INFO" if expected else "ERROR",
                           f"프로세스 종료 code={code}" + (" (자동 재시작 없음)" if not expected else ""))
                if restart and not self.shutdown_requested:
                    self.start(name)
            if self.shutdown_requested:
                # The user explicitly requested all three stops. Drain process
                # termination in order; never use mission services here.
                for name in ("vision", "motor", "camera"):
                    item = self.nodes[name]
                    if item["process"] is not None:
                        if item["state"] != "STOPPING":
                            self.stop(name)
                        return
                self.finished = True
                return
            for name, due in list(self.schedule.items()):
                if self.clock() - self.begun >= due:
                    self.start(name)

    def snapshot(self):
        with self.lock:
            nodes = {name: {k: v for k, v in item.items() if k != "process"}
                     for name, item in self.nodes.items()}
            telemetry = self.observer.snapshot(self.nodes)
            return {"nodes": nodes, "events": list(self.events), "telemetry": telemetry,
                    "services_in": round(self.services_remaining(), 1),
                    "shutdown_requested": self.shutdown_requested, "finished": self.finished,
                    "log_directory": str(self.directory)}

    def services_remaining(self):
        vision = self.nodes["vision"]["started"]
        return max(0.0, self.begun + 3 * INTERVAL - self.clock(),
                   (vision + INTERVAL - self.clock()) if vision is not None else 0.0)

    def request(self, request):
        with self.lock:
            op = request.get("op")
            if op == "status":
                return {"ok": True, "status": self.snapshot()}
            if self.shutdown_requested:
                return {"ok": False, "message": "전체 종료 진행 중"}
            if op in ("start", "stop", "restart"):
                name = request.get("node")
                if name not in NAMES:
                    raise ValueError("unknown node")
                result = self.start(name) if op == "start" else self.stop(name, op == "restart")
                return {"ok": not result.startswith("종료 보류:"), "message": result}
            if op == "service":
                name = request.get("name")
                if name not in SERVICES:
                    raise ValueError("unknown service")
                if self.services_remaining() > 0:
                    return {"ok": False, "message": "5초 실행 간격 대기 중"}
                proc = self.nodes["vision"]["process"]
                if proc is None or proc.poll() is not None:
                    return {"ok": False, "message": "비전 프로세스가 실행 중이 아닙니다"}
                # The ONLY service call site: an explicit socket/menu request.
                ok, message = self.observer.call(name)
                self.event("vision", "INFO" if ok else "ERROR", f"사용자 {name}: {message}")
                return {"ok": ok, "message": message}
            if op == "shutdown":
                for name in ("motor",):
                    if self.nodes[name]["process"] is not None:
                        safe, reason = self.observer.can_stop(name, self.nodes)
                        if not safe:
                            return {"ok": False, "message": f"전체 종료 보류: {reason}. 자동 PAUSE 없음."}
                self.schedule.clear()
                self.shutdown_requested = True
                for item in self.nodes.values():
                    item["restart"] = False
                self.event("manager", "INFO", "사용자 전체 종료 요청")
                return {"ok": True, "message": "비전 → 모터 → 카메라 정상 종료 대기"}
            raise ValueError("unknown operation")


def private_directory(path):
    path = Path(path)
    if path.is_symlink():
        raise RuntimeError("관리 디렉터리는 심볼릭 링크일 수 없습니다")
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    if path.stat().st_uid != os.getuid():
        raise RuntimeError("관리 디렉터리 소유자가 다릅니다")
    path.chmod(0o700)
    return path


def rpc(directory, request, timeout=5.0):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout)
        client.connect(str(Path(directory) / "manager.sock"))
        client.sendall(json.dumps(request).encode() + b"\n")
        data = bytearray()
        while not data.endswith(b"\n"):
            part = client.recv(65536)
            if not part:
                raise ConnectionError("관리자 연결이 종료됐습니다")
            data.extend(part)
            if len(data) > 1024 * 1024:
                raise ValueError("response too large")
        return json.loads(data)


def serve(plan, directory, observer):
    import fcntl
    directory = private_directory(directory)
    with (directory / "manager.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("JANDI 관리자가 이미 실행 중입니다")
        state_path = directory / "state.json"
        require_no_survivors(state_path)
        manager = ProcessManager(plan.commands(), directory, observer,
                                 env=plan.environment(), cwd=plan.workspace)
        address = directory / "manager.sock"
        address.unlink(missing_ok=True)
        # Closing a console/SSH or interrupting ros2 launch does not stop children.
        for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda number, frame: manager.event(
                "manager", "WARN", f"signal {number}: 실행 유지. 종료는 숫자 메뉴 0번 사용."))
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(address))
            server.listen(4)
            server.settimeout(0.1)
            saved_event = -1
            finished_at = None
            try:
                while True:
                    if manager.finished:
                        finished_at = finished_at or time.monotonic()
                        if time.monotonic() - finished_at >= 1.0:
                            break
                    else:
                        observer.spin()
                        manager.tick()
                    # Persist process ownership only on lifecycle/log events.
                    # UI refreshes do not continuously rewrite Jetson storage.
                    if saved_event != manager.event_id:
                        state = {"nodes": manager.snapshot()["nodes"], "plan": plan.__dict__}
                        temp = directory / "state.tmp"
                        temp.write_text(json.dumps(state, ensure_ascii=False))
                        temp.replace(state_path)
                        saved_event = manager.event_id
                    try:
                        client, _ = server.accept()
                    except socket.timeout:
                        continue
                    with client:
                        client.settimeout(1.0)
                        try:
                            data = bytearray()
                            while not data.endswith(b"\n"):
                                part = client.recv(4096)
                                if not part:
                                    raise ValueError("empty request")
                                data.extend(part)
                                if len(data) > 4096:
                                    raise ValueError("request too large")
                            response = manager.request(json.loads(data))
                            response["plan"] = plan.__dict__
                        except (ValueError, OSError) as exc:
                            response = {"ok": False, "message": str(exc)}
                        try:
                            client.sendall(json.dumps(response, ensure_ascii=False).encode() + b"\n")
                        except OSError:
                            pass  # A lost client never changes process/service state.
            finally:
                address.unlink(missing_ok=True)
