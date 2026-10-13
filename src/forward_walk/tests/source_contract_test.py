"""Check preserved source contracts; runtime behavior is tested in C++."""
from pathlib import Path
import json
import re
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
main = (root / "src/main.cpp").read_text()
player = (root / "src/p2p_motion_player.cpp").read_text()
dxl = (root / "src/dynamixel.cpp").read_text()
header = (root / "include/dynamixel.hpp").read_text()
config = (root / "CMakeLists.txt").read_text()
checks = {
    "protocol 2": '#define PROTOCOL_VERSION         2.0' in header,
    "4 Mbps": '#define BAUDRATE                 4000000' in header,
    "23 motors": '#define NUMBER_OF_DYNAMIXELS     23' in header,
    "stable serial device": '#define DEVICE_NAME              "/dev/jandi_dxl"' in header,
    "position mode": 'int16_t Mode = 1' in header,
    "P850 I0 D0": 'PID_Gain << 850, 0, 0;' in dxl,
    "50 Hz fixed step": 'effective_duration_sec_ * 50.0' in player and
        'static_cast<double>(completed_steps_ + 1) / total_steps_' in player,
    "startup 3sec final-only": '"startup_pose_duration_sec", 3.0' in main and 'options.final_keyframe_only = true;' in main,
    "eigen dependency": 'eigen' in [n.text for n in ET.parse(root / 'package.xml').findall('depend')],
    "Eigen3 CMake retained": 'find_package(Eigen3 REQUIRED)' in config,
    "single executor Goal writer": main.count('dxl_->SyncWriteRawPositions(')==1 and 'SyncWriteRawPositions(' not in player,
    "no CONTACT_WALK mapping": 'case vision::msg::ActionCommand::CONTACT_WALK:' not in main,
    "open/baud failure throws": 'throw std::runtime_error("Failed to open Dynamixel port:' in dxl and
        'portHandler->closePort();\n        throw std::runtime_error("Failed to set Dynamixel baudrate:' in dxl,
    "register writes reject communication and device errors":
        'result != COMM_SUCCESS || error != 0' in dxl and 'throw std::runtime_error(std::string(operation)' in dxl,
    "critical register writes checked": all(
        f'CheckRegisterWrite(packetHandler, {result}, dxl_error, dxl_id[i], "{operation}")' in dxl
        for result, operation in [
            ('dxl_comm_result', 'Current operating mode'), ('dxl_comm_result', 'Position operating mode'),
            ('dxl_comm_result', 'Torque enable'), ('result', 'Position P gain'),
            ('result', 'Position I gain'), ('result', 'Position D gain')]),
    "startup failure port cleanup": 'catch (...) {\n        portHandler->closePort();\n        throw;' in dxl,
    "failure IDs logged before reset": main.index('action_id=%llu queued_id=%llu camera_id=%llu') < main.index('action_transactions_.Abort();'),
}
for action, filename in {
    'STEP_FORWARD_LEFT_THREE': 'missions/직진_좌회전_3번.json',
    'STEP_FORWARD_RIGHT_THREE': 'missions/직진_우회전_2번.json',
    'STEP_FORWARD_TWO': 'missions/직진_2걸음.json',
    'STEP_FORWARD_THREE': 'missions/직진_3걸음.json',
    'STEP_FORWARD_FOUR': 'missions/직진_4걸음.json',
    'STEP_FORWARD_LEFT_TWO': 'missions/직진_좌회전_2번.json',
    'STEP_FORWARD_RIGHT_TWO': 'missions/직진_우회전_2번_ball.json',
    'STEP_FORWARD_RIGHT_ONE': 'missions/직진_우회전_1번.json',
    'STEP_FORWARD_LEFT_ONE': 'missions/직진_좌회전_1번.json',
    'STEP_FORWARD_RIGHT_ONE_HUDDLE': 'missions/직진_우회전_1번_huddle.json',
    'PICK_BALL': 'motions/공_줍고일어나기_최종.json',
    'PICK_BALL_RE': 'motions/공_줍고일어나기_최종_재그립제외.json',
    'PICK_BALL_RE_STAND': 'motions/공_줍고일어나기_최종_재그립제외_일어나기.json',
    'STEP_FORWARD_FIVE': 'missions/연속걷기.json',
    'DEFAULT_POSITION': 'motions/초기자세_다소곳_.json',
}.items():
    match = re.search(r'case vision::msg::ActionCommand::'+action+r':\s*return "([^"]+)";', main)
    checks[action+' mapping'] = match is not None and match.group(1)==filename and (root / filename).is_file()
# Validate single-turn missions and preserve the specific hurdle-right variant.
for manifest_name, expected_motion in {
    '직진_좌회전_1번.json': '보행_좌회전전진_20도_최종_초기X.json',
    '직진_우회전_1번_huddle.json': '보행_우회전전진_20도_최종_초기X.json',
}.items():
    manifest = root / 'missions' / manifest_name
    entries = json.loads(manifest.read_text()).get('motions', [])
    checks[manifest_name + ' single-turn reference'] = (
        [entry.get('filename') for entry in entries] == [expected_motion]
        and (root / 'motions' / expected_motion).is_file()
    )

pickup = json.loads((root / 'motions/공_줍고일어나기_최종.json').read_text())
checks['pickup regrip opening speed 80 deg/s'] = (
    pickup['keyframes'][7].get('name') == '재그립_벌리기'
    and pickup['keyframes'][7].get('max_speed_deg_s') == 80
)
gripper_switch = main.split('static bool UsesFixedGripperPose', 1)[1].split('default:', 1)[0]
checks['re-pick actions keep JSON gripper'] = all(
    'case Command::' + action + ':' in gripper_switch
    for action in ('PICK_BALL_RE', 'PICK_BALL_RE_STAND')
)

# Validate both 2x and 3x turn-forward manifests and real motion references.
for side, motion in {
    '좌': '보행_좌회전전진_20도_최종_초기X.json',
    '우': '보행_우회전전진_20도_최종_초기X.json',
}.items():
    for count in ((2,3) if side=='좌' else (1,2,3)):
        manifest = root / f'missions/직진_{side}회전_{count}번.json'
        entries = json.loads(manifest.read_text()).get('motions', [])
        expected = [motion]*count if side=='좌' else [motion,'보행_1걸음_초기X.json']*count
        checks[f'{side}회전 {count}x references'] = (
            [entry.get('filename') for entry in entries] == expected
            and all((root / 'motions' / file).is_file() for file in expected)
        )
state = (root / 'include/motion_executor_state.hpp').read_text()
checks['all-mission READY'] = 'IsReadyWalkAction(active_action)' in state
checks['LINE-only queue'] = 'IsLongLineAction(active_mission, active_action)' in state
checks['preserved camera calibration'] = all(
    needle in main for needle in (
        '"camera_forward_yaw_tick", 2077', '"camera_forward_pitch_tick", 1537',
        '"camera_down_yaw_tick", 2036', '"camera_down_pitch_tick", 2013',
        '"camera_goal_yaw_tick", 2054', '"camera_goal_pitch_tick", 966'
    )
)
for label, passed in checks.items():
    print(('PASS ' if passed else 'FAIL ')+label)
if not all(checks.values()):
    raise SystemExit(1)
