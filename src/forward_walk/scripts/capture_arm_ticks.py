#!/usr/bin/env python3
"""손으로 움직인 ID 15/16의 기준 자세와 바깥 자세 tick을 비교한다."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import tempfile
import time

MODE, TORQUE, GOAL, POSITION = 11, 64, 116, 132


class ArmCapture:
    def __init__(self, port, packet, success=0):
        self.port, self.packet, self.success = port, packet, success

    def check(self, comm, error, label):
        if comm != self.success:
            raise RuntimeError(f"{label}: {self.packet.getTxRxResult(comm)}")
        if error:
            raise RuntimeError(f"{label}: {self.packet.getRxPacketError(error)}")

    def read_byte(self, motor, address):
        value, comm, error = self.packet.read1ByteTxRx(self.port, motor, address)
        self.check(comm, error, f"ID {motor}, 주소 {address} 읽기")
        return value

    def position(self, motor):
        value, comm, error = self.packet.read4ByteTxRx(self.port, motor, POSITION)
        self.check(comm, error, f"ID {motor} tick 읽기")
        return value - (1 << 32) if value & (1 << 31) else value

    def torque(self, motor, enabled):
        comm, error = self.packet.write1ByteTxRx(self.port, motor, TORQUE, int(enabled))
        self.check(comm, error, f"ID {motor} torque={enabled}")

    def restore(self, motor, originally_enabled):
        tick = self.position(motor)
        if not 0 <= tick <= 4095:
            raise RuntimeError(f"ID {motor} tick={tick}: 범위 밖이라 토크를 다시 켜지 않았습니다.")
        comm, error = self.packet.write4ByteTxRx(self.port, motor, GOAL, tick)
        self.check(comm, error, f"ID {motor} 현재 위치를 Goal Position에 기록")
        self.torque(motor, originally_enabled)
        print(f"\nID {motor}: 현재 {tick} tick을 목표로 저장, 원래 토크 상태 "
              f"{'ON' if originally_enabled else 'OFF'} 복구 완료.", flush=True)

    def validate(self, motor):
        if self.read_byte(motor, MODE) != 3:
            raise RuntimeError(f"ID {motor}: Position Control Mode(3)가 아닙니다. 모드는 변경하지 않습니다.")
        if self.read_byte(motor, TORQUE) not in (0, 1):
            raise RuntimeError(f"ID {motor}: 토크 상태를 확인할 수 없습니다.")
        self.position(motor)

    def capture_pair(self, motors=(15, 16), confirm=input):
        for motor in motors:
            self.validate(motor)
            if self.read_byte(motor, TORQUE) != 1:
                raise RuntimeError(f"ID {motor}: main 초기화 후에도 토크 OFF입니다.")
        confirm("\n[1/3] 초기 자세 도착. Enter를 누르면 기준 tick을 측정합니다: ")
        baseline = {motor: self.position(motor) for motor in motors}
        print("기준 tick:", baseline, flush=True)
        confirm("\n[2/3] 팔을 받치세요. Enter를 누르면 15·16번만 토크를 풉니다: ")
        released = []
        try:
            for motor in motors:
                # 통신 실패여도 OFF 패킷이 도달했을 수 있으므로 복구 대상으로 먼저 등록한다.
                released.append(motor)
                self.torque(motor, False)
            confirm("\n[3/3] 팔을 원하는 바깥 자세로 움직이세요. Enter를 누르면 tick을 측정합니다: ")
            results = {}
            for motor in motors:
                tick = self.position(motor)
                if not 0 <= tick <= 4095:
                    raise RuntimeError(f"ID {motor}: tick={tick}, 정상 위치 범위 밖입니다.")
                delta = tick - baseline[motor]
                results[str(motor)] = {
                    "baseline_tick": baseline[motor], "outward_tick": tick,
                    "delta_tick": delta, "delta_deg": round(delta * 360 / 4096, 3),
                    "outward_direction": 1 if delta >= 8 else -1 if delta <= -8 else None,
                }
                print(f"ID {motor}: 기준={baseline[motor]}, 이동 후={tick}, 차이={delta:+d} tick", flush=True)
            return results
        finally:
            failures = []
            for motor in released:
                try:
                    self.restore(motor, True)
                except Exception as error:
                    failures.append(f"ID {motor}: {error}")
            if failures:
                raise RuntimeError("팔을 받치세요. 토크 복구 실패: " + "; ".join(failures))


def ensure_bus_free(device, allowed_pid=None):
    target = os.path.realpath(device)
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name) in (os.getpid(), allowed_pid):
            continue
        try:
            name = (process / 'comm').read_text().strip()
            if name == 'main_node':
                raise RuntimeError(f"main_node(PID {process.name})를 종료한 뒤 실행하세요.")
            for fd in (process / 'fd').iterdir():
                try:
                    if os.path.realpath(fd) == target:
                        raise RuntimeError(f"{device}를 {name}(PID {process.name})가 사용 중입니다.")
                except OSError:
                    continue
        except (OSError, UnicodeError):
            continue


def save(path, results):
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False,
                                     encoding='utf-8') as stream:
        temp = Path(stream.name)
        json.dump(results, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    try:
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def start_main(device, log_path):
    executable = Path(__file__).resolve().parents[3] / 'install/forward_walk/lib/forward_walk/main_node'
    if not executable.is_file():
        raise RuntimeError(f'main 실행 파일이 없습니다: {executable}')
    log = log_path.open('w', buffering=1)
    try:
        child = subprocess.Popen([
            str(executable), '--ros-args', '-p', f'dynamixel_device:={device}',
            '-r', '__node:=arm_capture_pose_holder',
            '-r', '/jandi_vision/action_cmd:=/arm_tick_capture/disabled_action_cmd',
            '-r', '/jandi_vision/camera_cmd:=/arm_tick_capture/disabled_camera_cmd',
            '-r', '/jandi_vision/action_status:=/arm_tick_capture/action_status',
            '-r', '/jandi_vision/camera_status:=/arm_tick_capture/camera_status',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    finally:
        log.close()
    return child


def wait_startup(child, log_path, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        content = log_path.read_text(errors='replace')
        if child.poll() is not None:
            raise RuntimeError(f'main 종료 코드 {child.returncode}:\n{content[-5000:]}')
        if 'Startup WALK_MODE pose reached and held; ready for command topics' in content:
            # 같은 callback 끝에서 idle timer가 취소된 뒤에 측정 포트를 연다.
            time.sleep(.1)
            return
        if 'Startup WALK_MODE pose failed' in content or 'Playback stopped without DONE' in content:
            raise RuntimeError(f'main 초기 자세 실패:\n{content[-5000:]}')
        time.sleep(.1)
    raise RuntimeError(f'main 초기 자세 대기 시간 초과. 로그: {log_path}')


def main():
    parser = argparse.ArgumentParser(description='실제 main 시작 자세 → Enter 기준 측정 → Enter 팔 토크 OFF → Enter 이동 후 측정')
    parser.add_argument('--port', default='/dev/jandi_dxl' if Path('/dev/jandi_dxl').exists() else '/dev/ttyUSB0')
    parser.add_argument('--output', type=Path, default=Path('/tmp/arm_ticks.json'))
    parser.add_argument('--existing-main-pid', type=int, help='이미 초기 자세를 완료한 측정 전용 main을 유지하며 측정')
    args = parser.parse_args()
    def interrupted(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    port = child = None
    status = 0
    try:
        if not sys.stdin.isatty():
            raise RuntimeError('Enter를 입력할 수 있는 터미널에서 실행하세요.')
        if not Path(args.port).exists():
            raise RuntimeError(f'장치가 없습니다: {args.port}')
        if args.existing_main_pid is not None:
            command = (Path('/proc') / str(args.existing_main_pid) / 'cmdline').read_bytes().split(b'\0')
            required = (
                b'__node:=arm_capture_pose_holder',
                b'/jandi_vision/action_cmd:=/arm_tick_capture/disabled_action_cmd',
                b'/jandi_vision/camera_cmd:=/arm_tick_capture/disabled_camera_cmd',
            )
            if not all(argument in command for argument in required):
                raise RuntimeError('지정한 PID는 비전 명령을 차단한 측정 전용 main이 아닙니다.')
            ready_log = Path('/tmp/arm_capture_main.log').read_text(errors='replace')
            if 'Startup WALK_MODE pose reached and held; ready for command topics' not in ready_log:
                raise RuntimeError('기존 main의 초기 자세 완료 기록이 없습니다.')
        ensure_bus_free(args.port, args.existing_main_pid)
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        output = args.output.expanduser().resolve()
        if not output.parent.is_dir():
            raise RuntimeError(f'저장 폴더가 없습니다: {output.parent}')
        log_path = Path('/tmp/arm_capture_main.log')
        if args.existing_main_pid is None:
            print('main으로 초기 자세로 이동합니다. 로봇을 받쳐주세요.', flush=True)
            print('비전 명령은 측정용 토픽으로 분리합니다. main 로그:', log_path, flush=True)
            child = start_main(args.port, log_path)
            wait_startup(child, log_path)
            ensure_bus_free(args.port, child.pid)
        else:
            print(f'기존 main(PID {args.existing_main_pid})과 몸 토크를 유지합니다. 측정 대상: 15·16번.', flush=True)
        print('main 초기 자세 완료. 몸과 팔 토크 ON.', flush=True)
        port = PortHandler(args.port)
        if not port.openPort() or not port.setBaudRate(4000000):
            raise RuntimeError(f'포트/baudrate 설정 실패: {args.port}')
        capture = ArmCapture(port, PacketHandler(2.0), COMM_SUCCESS)
        results = capture.capture_pair()
        save(output, results)
        print('\n측정 완료:\n' + json.dumps(results, ensure_ascii=False, indent=2), flush=True)
        print(f'저장: {output}', flush=True)
        if args.existing_main_pid is not None:
            print('기존 main은 계속 실행되며 몸과 팔의 토크를 유지합니다.', flush=True)
    except (KeyboardInterrupt, EOFError):
        print('\n측정을 취소했습니다.', flush=True)
        status = 130
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(f'ERROR: {error}', file=sys.stderr, flush=True)
        status = 1
    finally:
        if port is not None:
            port.closePort()
        if child is not None and child.poll() is None:
            # 측정 완료/취소만으로 main을 종료하면 다리까지 토크가 풀린다.
            # 몸을 계속 유지하고, 사용자가 로봇을 받친 뒤 명시적으로 종료할 때만 정리한다.
            try:
                input('\nmain을 유지합니다. 로봇을 받친 뒤 Enter를 누르면 전체 토크를 끄고 종료합니다: ')
            except (KeyboardInterrupt, EOFError):
                pass
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print('main이 종료되지 않았습니다. 강제 종료하지 않습니다. PID:', child.pid, flush=True)
    return status


if __name__ == '__main__':
    sys.exit(main())
