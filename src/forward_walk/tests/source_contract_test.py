"""Check preserved source contracts; runtime behavior is tested in C++."""
from pathlib import Path
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
}
for action, filename in {
    'STEP_FORWARD_LEFT': 'motions/보행_좌회전전진_20도_최종_초기X.json',
    'STEP_FORWARD_RIGHT': 'motions/보행_우회전전진_20도_최종_초기X.json',
    'STEP_FORWARD_FIVE': 'missions/연속걷기.json',
    'DEFAULT_POSITION': 'motions/초기자세_다소곳_.json',
}.items():
    match = re.search(r'case vision::msg::ActionCommand::'+action+r':\s*return "([^"]+)";', main)
    checks[action+' mapping'] = match is not None and match.group(1)==filename and (root / filename).is_file()
for label, passed in checks.items():
    print(('PASS ' if passed else 'FAIL ')+label)
if not all(checks.values()):
    raise SystemExit(1)
