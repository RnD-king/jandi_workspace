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


def screen(status, plan, page, message):
    lines = [f"JANDI {plan['mode']} | ROS_DOMAIN_ID=22 | SSH 종료 시 계속 실행", ""]
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
        lines += [f"로그: {status['log_directory']}", "0. 뒤로"]
        lines += [f"{e['time']} {e['level']} {e['node']}: {e['text']}" for e in status["events"][-12:]]
    elif page == "shutdown":
        lines += ["전체 종료: 로봇을 지지했는지 확인하세요 (모터 Torque OFF).",
                  "1. 정상 종료 요청  0. 취소", "자동 PAUSE 없음. 동작 중이면 종료를 보류합니다."]
    else:
        lines += ["1. RealSense   2. 모터   3. 비전 (켜기/끄기/재시작)",
                  "4. START   5. PAUSE   6. RESUME   7. RESET",
                  "8. 오류·로그   9. 콘솔 나가기(실행 유지)   0. 전체 종료"]
        events = [e for e in status["events"] if e["level"] in ("ERROR", "WARN")]
        if events:
            e = events[-1]
            lines += [f"최근 알림 [{e['time']}] {e['node']}: {e['text']}"]
    return "\n".join(lines + ["", message, "번호를 누르세요. Ctrl+C는 콘솔만 나갑니다."])


def key_request(page, key):
    """Pure UI mapping: polling/redrawing/disconnection cannot send a service."""
    if len(key) != 1:
        return page, None
    if page in NAMES:
        return ("main", {"op": {"1": "start", "2": "stop", "3": "restart"}[key], "node": page}) \
            if key in "123" else ("main" if key == "0" else page, None)
    if page == "shutdown":
        return ("main", {"op": "shutdown"}) if key == "1" else ("main", None)
    if page == "logs":
        return ("main" if key == "0" else page, None)
    if key in "123":
        return NAMES[int(key) - 1], None
    if key in "4567":
        return "main", {"op": "service", "name": ("start", "pause", "resume", "reset")[int(key) - 4]}
    return {"8": "logs", "9": "detach", "0": "shutdown"}.get(key, "main"), None


def console(directory):
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise RuntimeError("숫자 메뉴는 SSH PTY가 필요합니다. ssh -t 로 연결하세요. 백엔드는 계속 실행됩니다.")
    original = termios.tcgetattr(sys.stdin)
    page, message = "main", "미션은 사용자 START 전까지 WAIT_START입니다."
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
            view = screen(response["status"], response["plan"], page, message)
            sys.stdout.write("\x1b[H\x1b[2J" + view)
            sys.stdout.flush()
            if not select.select([sys.stdin], [], [], 0.5)[0]:
                continue
            key = sys.stdin.read(1)
            if not key:  # SSH EOF: no cleanup of processes or mission state.
                break
            page, request = key_request(page, key)
            if request is not None:
                try:
                    reply = rpc(directory, request)
                    message = reply.get("message", "")
                except (OSError, ConnectionError) as exc:
                    message = f"요청 응답 불명: {exc}. 자동 재전송 없음."
    except KeyboardInterrupt:
        message = "콘솔 연결 해제. 노드/미션은 계속 실행됩니다."
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        sys.stdout.write("\x1b[?25h\x1b[?1049l")
        sys.stdout.flush()
    print(message)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=(*MODES, "attach"))
    parser.add_argument("--workspace")
    parser.add_argument("--core-prefix")
    parser.add_argument("--state-dir", default=str(default_directory()))
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--serve", action="store_true", help=argparse.SUPPRESS)
    group.add_argument("--background", action="store_true", help="start backend without a console")
    args = parser.parse_args(argv)
    directory = private_directory(args.state_dir)
    if args.mode == "attach":
        if args.serve or args.background:
            parser.error("attach cannot start a backend")
        console(directory)
        return
    from ament_index_python.packages import get_package_share_directory
    plan = Plan.load(args.mode, get_package_share_directory("vision"), args.workspace, args.core_prefix)
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
            print("숫자 메뉴: jandi " + args.mode + " (SSH가 끊겨도 계속 실행)")
        else:
            console(directory)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(f"JANDI: {error}", file=sys.stderr)
        sys.exit(1)
