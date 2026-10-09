#!/usr/bin/env python3
"""Capture camera poses using only Dynamixel IDs 21 (yaw) and 22 (pitch)."""

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile


MOTOR_IDS = (21, 22)
POSE_NAMES = ("FORWARD", "DOWN", "GOAL")
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132


class CameraCapture:
    def __init__(self, port, packet, comm_success=0):
        self.port = port
        self.packet = packet
        self.comm_success = comm_success

    def check(self, comm, error, label):
        if comm != self.comm_success:
            raise RuntimeError(f"{label}: {self.packet.getTxRxResult(comm)}")
        if error:
            raise RuntimeError(f"{label}: {self.packet.getRxPacketError(error)}")

    def torque(self, motor_id, enabled):
        comm, error = self.packet.write1ByteTxRx(
            self.port, motor_id, ADDR_TORQUE_ENABLE, int(enabled))
        self.check(comm, error, f"Torque {'ON' if enabled else 'OFF'} ID {motor_id}")

    def read_position(self, motor_id):
        raw, comm, error = self.packet.read4ByteTxRx(
            self.port, motor_id, ADDR_PRESENT_POSITION)
        self.check(comm, error, f"Read position ID {motor_id}")
        # Protocol 2.0 Present Position is signed, including while torque is OFF.
        return raw - (1 << 32) if raw & (1 << 31) else raw

    def validate_modes(self):
        for motor_id in MOTOR_IDS:
            mode, comm, error = self.packet.read1ByteTxRx(
                self.port, motor_id, ADDR_OPERATING_MODE)
            self.check(comm, error, f"Read operating mode ID {motor_id}")
            if mode != 3:
                raise RuntimeError(
                    f"ID {motor_id}: Operating Mode={mode}. "
                    "Position Control Mode(3)에서 실행하세요. 모드는 자동 변경하지 않습니다.")

    def restore_current_pose(self, motor_ids):
        positions = {}
        errors = []
        # Set both goals before enabling either motor. Never reuse an old pose.
        for motor_id in motor_ids:
            try:
                position = self.read_position(motor_id)
                if not 0 <= position <= 4095:
                    raise RuntimeError(
                        f"ID {motor_id}: tick={position}, 0..4095 범위 밖입니다. "
                        "자동 토크 ON을 중단합니다.")
                comm, error = self.packet.write4ByteTxRx(
                    self.port, motor_id, ADDR_GOAL_POSITION, position)
                self.check(comm, error, f"Write current goal ID {motor_id}")
                positions[motor_id] = position
            except RuntimeError as exc:
                errors.append(str(exc))
        for motor_id in positions:
            try:
                self.torque(motor_id, True)
            except RuntimeError as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError(
                "현재 자세 복구를 완료하지 못했습니다. 머리를 지지하고 토크 상태를 확인하세요.\n"
                + "\n".join(errors))
        return positions

    def capture(self, pose, prompt=input):
        released = []
        try:
            for motor_id in MOTOR_IDS:
                print(f"현재 ID {motor_id}: {self.read_position(motor_id)} tick")
            for motor_id in MOTOR_IDS:
                # Include even a failed OFF request: the motor may have received it.
                released.append(motor_id)
                self.torque(motor_id, False)
            print(f"\n[{pose}] ID 21/22 토크 OFF. 머리를 지지하며 손으로 자세를 맞추세요.")
            if pose == "FORWARD":
                print("FORWARD: 평상시 주행용, 약 45도 아래를 보는 자세")
            prompt("준비되면 Enter (취소: Ctrl+C): ")
        finally:
            # On Enter, Ctrl+C, EOF, or an error, read the current pose and restore.
            positions = self.restore_current_pose(released) if released else {}
        print(f"[{pose}] 21(Yaw)={positions[21]}, 22(Pitch)={positions[22]}")
        print("현재 위치를 Goal Position으로 설정하고 ID 21/22 토크 ON 완료.\n")
        return positions


def check_main_stopped():
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            name = (proc / "comm").read_text().strip()
        except (OSError, UnicodeError):
            continue
        if name == "main_node":
            raise RuntimeError(
                f"main_node(PID {proc.name})가 실행 중입니다. "
                "forward_walk와 자동 재시작 launch를 종료한 뒤 실행하세요.")


def save_result(path, results):
    # Atomic replacement avoids losing previous poses on an interrupted write.
    with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
        temporary_path = Path(stream.name)
        try:
            json.dump(results, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        except BaseException:
            temporary_path.unlink(missing_ok=True)
            raise
    try:
        temporary_path.replace(path)
    finally:
        temporary_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description="ID 21/22 카메라 자세 tick 캡처")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=4000000)
    parser.add_argument("--pose", choices=POSE_NAMES, help="생략하면 세 자세를 순서대로 캡처")
    parser.add_argument("--output", type=Path, default=Path("/tmp/camera_ticks.json"))
    args = parser.parse_args()
    port = None
    try:
        check_main_stopped()
        if not Path(args.port).exists():
            raise RuntimeError(f"장치를 찾을 수 없습니다: {args.port}")
        try:
            from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        except ImportError as exc:
            raise RuntimeError(
                "dynamixel_sdk를 불러올 수 없습니다. ROS와 jandi의 install/setup.bash를 "
                "source한 터미널에서 python3로 실행하세요.") from exc
        output = args.output.expanduser().resolve()
        results = json.loads(output.read_text()) if output.exists() else {}
        if not isinstance(results, dict):
            raise RuntimeError(f"저장 파일이 JSON 객체가 아닙니다: {output}")
        if not output.parent.is_dir():
            raise RuntimeError(f"저장 디렉터리가 없습니다: {output.parent}")
        port = PortHandler(args.port)
        if not port.openPort():
            raise RuntimeError(f"포트를 열 수 없습니다: {args.port}")
        if not port.setBaudRate(args.baudrate):
            raise RuntimeError(f"Baudrate 설정 실패: {args.baudrate}")
        capture = CameraCapture(port, PacketHandler(2.0), COMM_SUCCESS)
        capture.validate_modes()
        print("forward_walk와 P2P 에디터 등 같은 U2D2를 사용하는 프로그램은 종료되어야 합니다.")
        print(f"연결: {args.port}, {args.baudrate} baud. ID 21/22만 제어합니다.\n")
        for pose in (args.pose,) if args.pose else POSE_NAMES:
            positions = capture.capture(pose)
            results[pose] = {str(k): v for k, v in positions.items()}
            save_result(output, results)
            print(f"저장: {output}\n")
        print(json.dumps(results, ensure_ascii=False, indent=2))
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\n캡처를 취소했습니다. 이전에 저장한 자세는 유지됩니다.")
        return 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if port is not None:
            port.closePort()


if __name__ == "__main__":
    sys.exit(main())
