# JANDI 실전 1회차(main) / 2회차(fast) 전환 절차

> **중요**: 1회차에서 성공하기 전에는 2회차 브랜치를 Jetson에 설치하지 않습니다.
> `main`과 `race-round2-fast-20261109`는 별개의 Git 브랜치이며, 자동 병합하지 않습니다.
> 이 문서는 별도 `~/my_cv` 및 `~/vision_core` 설치 환경과
> `~/jandi_workspace` 소스 체크아웃을 기준으로 합니다.

## 0. Jetson 소스 체크아웃 준비 (최초 1회)

```bash
# 현재 사용하는 Jetson: ROS 2 Humble, /home/rnd/my_cv, /home/rnd/vision_core
test -d "$HOME/my_cv/install/dynamixel_sdk" || echo "[확인 필요] dynamixel_sdk 설치 위치"
test -d "$HOME/vision_core" || echo "[확인 필요] vision_core 작업 공간"

# 저장소가 ~/jandi처럼 다른 곳에 있다면 먼저 실제 위치를 확인합니다.
if [ ! -d "$HOME/jandi_workspace/.git" ]; then
  git clone https://github.com/RnD-king/jandi_workspace.git "$HOME/jandi_workspace"
fi
```

`scripts/sync_jandi_to_my_cv.sh`는 `~/jandi_workspace`, `~/my_cv`, `~/vision_core`을
사용하며 시작 전 다음을 확인합니다: 소스 Git 작업 트리 깨끗함, 현재 브랜치 일치,
Dynamixel SDK ROS 설치, 대상 소스 존재. 작업 중 `rsync --delete`로 파일이
삭제될 수 있으므로 **백업 확인 메시지에서 변경 목록과 백업 위치를 검토**합니다.

## 1. 대회 1회차: main 빌드 및 설치

로봇 모터/카메라 노드를 모두 종료하고 **새 SSH 터미널**에서:

```bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/my_cv/install/setup.bash"
cd "$HOME/jandi_workspace"
git fetch origin
git switch main
git pull --ff-only origin main
echo "[1회차 MAIN] $(git rev-parse --short HEAD)"
JANDI_BRANCH=main bash scripts/sync_jandi_to_my_cv.sh
```

동기화 스크립트가 **이전 소스를 백업한 뒤 Debug 빌드 → CTest →
Release vision_core 설치 → ROS vision/forward_walk 빌드 → colcon test**를
순서대로 수행합니다. 단계별 실패가 있으면 실행을 중단합니다.

검증:

```bash
git -C "$HOME/jandi_workspace" branch --show-current
git -C "$HOME/jandi_workspace" rev-parse --short HEAD
source "$HOME/my_cv/install/setup.bash"
ros2 pkg prefix vision
ros2 pkg prefix forward_walk
```

1회차 경기는 여기서 설치한 `main`으로만 진행합니다.

## 2. 1회차 성공 후: 2회차 FAST 브랜치 설치

1회차 동작이 완전히 끝나고 모든 ROS/모터 노드를 종료한 다음,
**새 SSH 터미널**에서:

```bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/my_cv/install/setup.bash"
cd "$HOME/jandi_workspace"
test -z "$(git status --porcelain)" || {
  echo "로컬 소스 변경사항이 있습니다. 먼저 백업·커밋하세요."
  exit 1
}
git fetch origin race-round2-fast-20261109
git switch race-round2-fast-20261109 2>/dev/null ||
  git switch --track origin/race-round2-fast-20261109
git pull --ff-only origin race-round2-fast-20261109
echo "[2회차 FAST] $(git rev-parse --short HEAD)"
JANDI_BRANCH=race-round2-fast-20261109 bash scripts/sync_jandi_to_my_cv.sh
```

이 스크립트도 기존 설치 소스를 별도 타임스탬프 폴더에 백업하며,
`vision_core` Debug/CTest, Release 설치와 `my_cv` 패키지 빌드·테스트를
다시 수행합니다. **코드를 pull하는 것만으로 바이너리가 교체되는 것이
아니므로 반드시 전체 설치·빌드를 완료해야 합니다.**

FAST 설치 검증:

```bash
git -C "$HOME/jandi_workspace" branch --show-current
git -C "$HOME/jandi_workspace" rev-parse --short HEAD
grep -n "race_round2_fast_enabled" \
  "$HOME/vision_core/install/share/shared_vision_core/config/vision_algorithm.yaml"
grep -A1 'case vision::msg::ActionCommand::STEP_FORWARD_ONE:' \
  "$HOME/my_cv/src/forward_walk/src/main.cpp"
source "$HOME/my_cv/install/setup.bash"
ros2 pkg prefix vision
ros2 pkg prefix forward_walk
```

예상: FAST 실전 YAML에는 `race_round2_fast_enabled: true`가 있고,
액션 **10번**은 `보행_1걸음_초기X_수정3.json`에 연결되어 있습니다.
**35번은** `수정5.json` 그대로입니다.

## 3. 2회차 FAST 설치 실패 또는 중단 시: main으로 재설치

이미 FAST 소스 복사가 시작되었다면 설치 라이브러리와 ROS 바이너리가
섞여 있을 수 있으므로 **이전 실행 파일을 그대로 실행하지 말고**
다음 전체 재설치 과정을 수행합니다.

```bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/my_cv/install/setup.bash"
cd "$HOME/jandi_workspace"
git switch main
git pull --ff-only origin main
JANDI_BRANCH=main bash scripts/sync_jandi_to_my_cv.sh
```

## 4. 운영상 주의

- **`main`에 FAST 커밋을 merge/cherry-pick하지 않습니다.** Draft PR은 테스트용입니다.
- `rsync` 복사 후 `vision_core` 설치 YAML/라이브러리와
  `vision`/ `forward_walk`를 반드시 **같은 브랜치의 소스**로 다시 빌드합니다.
- FAST는 CTest/CI 통과가 실기 보행 균형, 10번 연속 실행, 0.4초 카메라
  도착, READY/ACK/DONE 네트워크 지연까지 보장하지 않습니다.
- 경기 직전에는 실행 중인 로봇 노드가 없도록 확인하고, 업데이트 후
  카메라·모터 ID 설정과 `ROS_DOMAIN_ID=22` 등 기존 실전 환경을 확인합니다.
- FAST에서 10번 모션 시간은 JSON 설정상 0.59초이며,
  BALL/HURDLE 미검출 탐색의 추가 간격은 0.3초입니다.
