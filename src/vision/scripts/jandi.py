#!/usr/bin/python3
"""SSH-independent JANDI launcher and numeric console. No automatic control."""
import argparse
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import termios
import time
import tty

from jandi_runtime import MODES, NAMES, Plan, private_directory, rpc, serve
from jandi_logs import FILTERS, LogView


def default_directory():
    # Short AF_UNIX path, per-user permissions; independent of an SSH session.
    return Path(f"/tmp/jandi-{os.getuid()}")


def background(plan, directory):
    try:
        response = rpc(directory, {"op": "status"})
        if response.get("plan") != plan.__dict__:
            raise RuntimeError("다른 모드/경로의 JANDI가 실행 중입니다. 기존 콘솔에서 0번으로 종료하세요.")
        return
    except (ConnectionError, FileNotFoundError, ConnectionRefusedError):
        pass
    log = directory / "manager.log"
    cmd = [sys.executable, str(Path(__file__).resolve()), "--serve", plan.mode,
           "--workspace", plan.workspace, "--core-prefix", plan.core_prefix,
           "--state-dir", str(directory)]
    for name in ("show_debug_view", "terminal_node", "stream_debug_view"):
        cmd += ["--" + name.replace("_", "-"), str(getattr(plan, name)).lower()]
    with log.open("a") as stream:
        subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                         start_new_session=True, env=plan.environment())
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        try:
            response = rpc(directory, {"op": "status"})
            if response.get("plan") != plan.__dict__:
                raise RuntimeError("다른 설정의 관리자가 이미 실행 중입니다")
            return
        except (ConnectionError, FileNotFoundError, ConnectionRefusedError):
            time.sleep(0.1)
    raise RuntimeError(f"실행 관리자 준비 실패. {log} 확인 (다른 노드 자동 종료 없음)")


def screen(status, plan, page, message, log_view=None):
    lines = [f"JANDI {plan['mode']} | VERSION={plan.get('version','legacy').upper()} | ROS_DOMAIN_ID=22 | SSH 종료 시 계속 실행", ""]
    lines.append("옵션: " + " / ".join(
        f"{label}={'ON' if plan.get(name, False) else 'OFF'}" for name, label in (
            ("show_debug_view", "OpenCV"), ("terminal_node", "터미널"),
            ("stream_debug_view", "HTTP :8080"))))
    telemetry = status["telemetry"]
    labels = {"camera": "RealSense", "motor": "Dynamixel", "vision": "Vision"}
    for name, item in status["nodes"].items():
        ready = "ROS READY" if telemetry["ros_ready"].get(name) else "ROS WAIT/UNKNOWN"
        code = "" if item["exit_code"] is None else f" exit={item['exit_code']}"
        lines.append(f"{labels[name]:10} {item['state']:9} PID={item['pid'] or '-'} {ready}{code}")
    lines += [f"MISSION: {telemetry['run_state']} | 미완료 몸={telemetry['pending_actions']} "
              f"카메라={telemetry['pending_cameras']}",
              f"서비스: {status['services_in']:.1f}초 대기" if status['services_in'] else
              "서비스: " + " ".join(f"{k}={'READY' if v else 'WAIT'}"
                                     for k, v in telemetry["services"].items()), ""]
    age = telemetry.get("camera_info_age")
    lines.append("CameraInfo: " + (f"{age:.1f}초 전 수신" if age is not None else "수신 대기"))
    if page in NAMES:
        lines += [f"[{labels[page]}] 1. 켜기  2. 끄기  3. 재시작  0. 뒤로",
                  "모터 종료 시 Torque OFF / 재시작 시 초기 자세 이동"]
    elif page == "logs":
        lines += [f"로그: {status['log_directory']}"]
        lines += [f"{key}. {entry[2]}" for key, entry in FILTERS.items()]
        lines += ["0. 이전 화면"]
    elif page.startswith("logs:"):
        lines += [FILTERS[page[-1]][2], "p. 이전 페이지  n. 다음 페이지  f. 실시간 끝  0. 로그 메뉴"]
        if log_view is not None:
            records, position = log_view.lines()
            lines.append(position)
            for record in records:
                text = "".join(c for c in record["text"] if c == '\t' or c.isprintable())
                lines.append(f"{record['time']} {record['level']} {record['node']}: {text}")
    elif page.startswith("emergency:"):
        lines += ["EMERGENCY STOP", "정말 긴급종료하시겠습니까?",
                  "1. 긴급종료 실행", "0. 취소", "Esc: 취소"]
    elif page.startswith("confirm:"):
        lines += [f"{page.split(':')[1].upper()} 서비스 전송 대기",
                  "Enter: 한 번 전송 / Esc 또는 0: 취소"]
    elif page == "shutdown":
        lines += ["전체 종료: 로봇을 지지했는지 확인하세요 (모터 Torque OFF).",
                  "1. 정상 종료 요청  0. 취소", "자동 PAUSE 없음. 동작 중이면 종료를 보류합니다."]
    else:
        lines += ["1. RealSense   2. 모터   3. 비전 (켜기/끄기/재시작)",
                  "4. START   5. PAUSE   6. RESUME   7. RESET",
                  "8. 오류·로그   9. 콘솔 나가기(실행 유지)   0. 전체 종료",
                  "K. 긴급종료 확인 (1: 실행 / 0·Esc: 취소)"]
        events = [e for e in status["events"] if e["level"] in ("ERROR", "FATAL", "WARN")]
        if events:
            e = events[-1]
            lines += [f"최근 알림 [{e['time']}] {e['node']}: {e['text']}"]
    if status.get("emergency_stop_requested"):
        lines.append("EMERGENCY STOP: 전체 노드 종료 진행 중, 자동 재시작 금지")
    return "\n".join(lines + ["", message, "번호를 누르세요. Ctrl+C는 콘솔만 나갑니다."])


def key_request(page, key):
    """Pure UI mapping: polling/redrawing/disconnection cannot send a service."""
    if len(key) != 1:
        return page, None
    if key in ("K", "k"):
        return (page if page.startswith("emergency:") else "emergency:"+page), None
    if page.startswith("emergency:"):
        if key=="1":
            return "main", {"op":"emergency_stop"}
        return (page[len("emergency:"):] if key in ("0","\x1b") else page), None
    if page in NAMES:
        return ("main", {"op": {"1": "start", "2": "stop", "3": "restart"}[key], "node": page}) \
            if key in "123" else ("main" if key == "0" else page, None)
    if page == "shutdown":
        return ("main", {"op": "shutdown"}) if key == "1" else ("main", None)
    if page == "logs":
        return ("logs:" + key if key in FILTERS else "main" if key == "0" else page, None)
    if page.startswith("logs:"):
        return ("logs" if key in ("0", "\x1b") else page, None)
    if page.startswith("confirm:"):
        if key in ("\r", "\n"):
            return "main", {"op": "service", "name": page.split(":")[1]}
        return ("main" if key in ("0", "\x1b") else page, None)
    if key in "123":
        return NAMES[int(key) - 1], None
    if key in "4567":
        return "confirm:" + ("start", "pause", "resume", "reset")[int(key) - 4], None
    return {"8": "logs", "9": "detach", "0": "shutdown"}.get(key, "main"), None


def console(directory):
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise RuntimeError("숫자 메뉴는 SSH PTY가 필요합니다. ssh -t 로 연결하세요. 백엔드는 계속 실행됩니다.")
    original = termios.tcgetattr(sys.stdin)
    page, message = "main", "미션은 사용자 START 전까지 WAIT_START입니다."
    log_view = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        sys.stdout.write("\x1b[?1049h\x1b[?25l")
        while page != "detach":
            try:
                response = rpc(directory, {"op": "status"})
            except (OSError, ConnectionError) as exc:
                message = f"관리자 연결 종료: {exc} (자동 제어 없음)"
                break
            if response["status"]["finished"]:
                message = "전체 정상 종료를 완료했습니다."
                break
            if page.startswith("logs:"):
                if log_view is None or log_view.selection != page[-1]:
                    log_view = LogView(directory, page[-1])
            else:
                log_view = None
            view = screen(response["status"], response["plan"], page, message, log_view)
            sys.stdout.write("\x1b[H\x1b[2J" + view)
            sys.stdout.flush()
            if not select.select([sys.stdin], [], [], 0.5)[0]:
                continue
            key = sys.stdin.read(1)
            if not key:  # SSH EOF: no cleanup of processes or mission state.
                break
            if log_view is not None:
                log_view.move(key)
            page, request = key_request(page, key)
            if request is not None and not (request.get("op")=="emergency_stop" and
                                             response["status"].get("emergency_stop_requested")):
                try:
                    reply = rpc(directory, request)
                    message = reply.get("message", "")
                except (OSError, ConnectionError) as exc:
                    message = f"요청 응답 불명: {exc}. 자동 재전송 없음."
    except KeyboardInterrupt:
        message = "콘솔 연결 해제. 노드/미션은 계속 실행됩니다."
    finally:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
            sys.stdout.write("\x1b[?25h\x1b[?1049l")
            sys.stdout.flush()
        except (OSError, termios.error):
            pass  # A lost PTY never causes backend cleanup.
    print(message)


def choose_options(initial=None):
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise RuntimeError("처음 실행은 SSH 터미널에서 하세요. 비대화형 실행은 --background 사용.")
    values = list(initial) if initial is not None else [False, False, True]
    original = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            lines = ["JANDI 실행 옵션 — 아직 노드를 시작하지 않았습니다.", ""]
            for index, label in enumerate(("OpenCV 디버그 화면", "터미널 디버그", "HTTP 영상 스트리밍")):
                lines.append(f"{index + 1}. {label}: {'ON' if values[index] else 'OFF'}")
            lines += ["", "1/2/3: 변경 / Enter: 실행 / Esc: 취소"]
            sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(lines) + "\n")
            sys.stdout.flush()
            key = sys.stdin.read(1)
            if key in ("\r", "\n"):
                return values
            if not key or key == "\x1b":
                return None
            if key in "123":
                values[int(key) - 1] = not values[int(key) - 1]
    except KeyboardInterrupt:
        return None
    finally:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        except (OSError, termios.error):
            pass


def boolean(value):
    if value.lower() not in ("true", "false"):
        raise argparse.ArgumentTypeError("true 또는 false를 지정하세요")
    return value.lower() == "true"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", nargs="?", choices=(*MODES, "attach"))
    parser.add_argument("--show-debug-view", type=boolean, default=False)
    parser.add_argument("--terminal-node", type=boolean, default=None)
    parser.add_argument("--stream-debug-view", type=boolean, default=True)
    parser.add_argument("--workspace")
    parser.add_argument("--core-prefix")
    parser.add_argument("--state-dir", default=str(default_directory()))
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--serve", action="store_true", help=argparse.SUPPRESS)
    group.add_argument("--background", action="store_true", help="start backend without a console")
    args = parser.parse_args(argv)
    directory = private_directory(args.state_dir)
    if args.mode is None and not args.serve and not args.background:
        try:
            existing = rpc(directory, {"op": "status"})
        except (ConnectionError, OSError):
            existing = None
        if existing is not None:
            console(directory)
            return
        options = choose_options((args.show_debug_view, bool(args.terminal_node), args.stream_debug_view))
        if options is None:
            print("취소했습니다. 노드를 시작하지 않았습니다.")
            return
        args.show_debug_view, args.terminal_node, args.stream_debug_view = options
    args.mode = args.mode or "custom"
    if args.mode == "attach":
        if args.serve or args.background:
            parser.error("attach cannot start a backend")
        console(directory)
        return
    from ament_index_python.packages import get_package_share_directory
    plan = Plan.load(args.mode, get_package_share_directory("vision"), args.workspace, args.core_prefix,
                     show_debug_view=args.show_debug_view, terminal_node=args.terminal_node,
                     stream_debug_view=args.stream_debug_view)
    if args.serve:
        from jandi_ros import RosObserver
        os.environ.update(plan.environment())
        observer = RosObserver(str(Path(plan.share) / "config/vision_params.yaml"))
        try:
            serve(plan, directory, observer)
        finally:
            observer.node.destroy_node()
            observer.rclpy.shutdown()
    else:
        background(plan, directory)
        if args.background:
            print("JANDI 실행 관리자 시작/연결 완료. 0/5/10초 실행, 15초 이후 수동 서비스.")
            print("숫자 메뉴: run_jandi (SSH가 끊겨도 계속 실행)")
        else:
            console(directory)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(f"JANDI [ERROR]: {error}", file=sys.stderr)
        sys.exit(1)
