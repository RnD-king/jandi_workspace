#!/usr/bin/env bash
set -Eeuo pipefail

trap 'echo "[ERROR] line $LINENO: $BASH_COMMAND" >&2' ERR

# ==========================================
# 기본 경로 및 브랜치
# ==========================================
JANDI="$HOME/jandi_workspace"
MY_CV="$HOME/my_cv"
CORE="$HOME/vision_core"

BRANCH="${JANDI_BRANCH:-main}"

SDK_DIR="$MY_CV/install/dynamixel_sdk/share/dynamixel_sdk/cmake"

BACKUP="$HOME/jandi_sync_backups/$(date +%Y%m%d_%H%M%S)"

# ==========================================
# 사전 점검
# ==========================================
for cmd in git rsync cmake ctest colcon ros2; do
    command -v "$cmd" >/dev/null || {
        echo "[ERROR] $cmd 명령어가 없습니다."
        exit 1
    }
done

for path in \
    "$JANDI/.git" \
    "$JANDI/src/vision" \
    "$JANDI/src/forward_walk" \
    "$JANDI/src/vision_core" \
    "$MY_CV/src/vision" \
    "$MY_CV/src/forward_walk" \
    "$CORE/CMakeLists.txt" \
    "$SDK_DIR/dynamixel_sdkConfig.cmake"
do
    [[ -e "$path" ]] || {
        echo "[ERROR] 경로 없음: $path"
        exit 1
    }
done

for path in "$MY_CV/src/vision" "$MY_CV/src/forward_walk" "$CORE"; do
    [[ ! -L "$path" ]] || {
        echo "[ERROR] 대상이 심볼릭 링크입니다: $path"
        exit 1
    }
done

if [[ "${AMENT_PREFIX_PATH:-}" != *"$MY_CV/install/dynamixel_sdk"* ]]; then
    echo "[ERROR] 새 터미널에서 my_cv를 먼저 실행하세요."
    exit 1
fi

if [[ "${AMENT_PREFIX_PATH:-}" == *"$JANDI/install"* ]]; then
    echo "[ERROR] jandi_workspace 환경이 활성화되어 있습니다."
    echo "새 터미널에서 my_cv만 실행한 뒤 재시도하세요."
    exit 1
fi

# ==========================================
# STEP 1. GitHub 최신 코드 받기
# ==========================================
echo
echo "========== STEP 1: GIT PULL =========="

CURRENT_BRANCH="$(git -C "$JANDI" branch --show-current)"

if [[ "$CURRENT_BRANCH" != "$BRANCH" ]]; then
    echo "[ERROR] 현재 브랜치: $CURRENT_BRANCH"
    echo "필요한 브랜치: $BRANCH"
    exit 1
fi

if [[ -n "$(git -C "$JANDI" status --porcelain)" ]]; then
    echo "[ERROR] jandi_workspace에 미저장 변경 사항이 있습니다."
    echo "먼저 커밋하거나 stash하세요."
    exit 1
fi

git -C "$JANDI" pull --ff-only origin "$BRANCH"

COMMIT="$(git -C "$JANDI" rev-parse --short HEAD)"
echo "최신 커밋: $COMMIT"

# ==========================================
# STEP 2. 기존 코드 백업 및 동기화
# ==========================================
echo
echo "========== STEP 2: SYNC CODE =========="

SOURCES=(
    "$JANDI/src/vision"
    "$JANDI/src/forward_walk"
    "$JANDI/src/vision_core"
)

DESTINATIONS=(
    "$MY_CV/src/vision"
    "$MY_CV/src/forward_walk"
    "$CORE"
)

NAMES=(vision forward_walk vision_core)

EXCLUDES=(
    --exclude='.git'
    --exclude='build*/'
    --exclude='install/'
    --exclude='log/'
    --exclude='__pycache__/'
    --exclude='*.pyc'
)

mkdir -p "$BACKUP"
echo "$COMMIT" > "$BACKUP/jandi_commit.txt"

# 복사 전 변경 사항 미리 보기
for i in 0 1 2; do
    NAME="${NAMES[$i]}"
    SRC="${SOURCES[$i]}"
    DST="${DESTINATIONS[$i]}"

    echo
    echo "[$NAME] 변경 사항"

    if [[ -e "$DST/.git" ]]; then
        git -C "$DST" status --short \
            > "$BACKUP/${NAME}_git_status_before.txt" || true
    fi

    rsync -anic --delete "${EXCLUDES[@]}" "$SRC/" "$DST/" \
        > "$BACKUP/${NAME}_changes.txt"

    sed -n '1,60p' "$BACKUP/${NAME}_changes.txt"

    echo "전체 변경 목록: $BACKUP/${NAME}_changes.txt"
done

echo
read -r -p "기존 파일을 백업하고 최신화할까요? [y/N]: " CONFIRM

if [[ "$CONFIRM" != "y" && "$CONFIRM" != "Y" ]]; then
    echo "동기화 취소. 기존 파일은 변경하지 않았습니다."
    exit 0
fi

for i in 0 1 2; do
    NAME="${NAMES[$i]}"
    SRC="${SOURCES[$i]}"
    DST="${DESTINATIONS[$i]}"

    echo "[$NAME] 백업 및 복사 중..."

    mkdir -p "$BACKUP/$NAME"

    # 기존 코드 전체 백업
    rsync -a "${EXCLUDES[@]}" \
        "$DST/" "$BACKUP/$NAME/"

    # 최신 코드 복사 (.git 및 빌드 파일 제외)
    rsync -a --checksum --delete-delay "${EXCLUDES[@]}" \
        "$SRC/" "$DST/"
done

echo "백업 완료: $BACKUP"

# ==========================================
# STEP 3. vision_core 빌드 및 테스트
# ==========================================
echo
echo "========== STEP 3: VISION CORE =========="

# Debug 빌드에서 assert 기반 테스트 실행
cmake -S "$CORE" \
    -B "$CORE/build_jandi_debug" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DBUILD_TESTING=ON

cmake --build "$CORE/build_jandi_debug" -j4

ctest --test-dir "$CORE/build_jandi_debug" \
    --output-on-failure -j4

# 테스트 통과 후 Release 라이브러리 설치
cmake -S "$CORE" \
    -B "$CORE/build_jandi_release" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DCMAKE_INSTALL_PREFIX="$CORE/install"

cmake --build "$CORE/build_jandi_release" -j4

cmake --install "$CORE/build_jandi_release"

echo "vision_core 설치 완료"

# ==========================================
# STEP 4. my_cv ROS 패키지 빌드
# ==========================================
echo
echo "========== STEP 4: MY_CV BUILD =========="

cd "$MY_CV"

export VISION_CORE_ROOT="$CORE/install"

colcon build \
    --packages-select vision forward_walk \
    --symlink-install \
    --parallel-workers 2 \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DSHARED_VISION_CORE_ROOT="$VISION_CORE_ROOT" \
        -Ddynamixel_sdk_DIR="$SDK_DIR"

echo "ROS 패키지 빌드 완료"

# ==========================================
# STEP 5. ROS 2 테스트
# ==========================================
echo
echo "========== STEP 5: TEST =========="

colcon test \
    --packages-select vision forward_walk \
    --event-handlers console_direct+

colcon test-result --verbose

echo
echo "=========================================="
echo "동기화 및 빌드 절차 완료"
echo "적용한 GitHub 커밋: $COMMIT"
echo "백업 위치: $BACKUP"
echo "테스트 결과에 실패 항목이 없는지 확인하세요."
echo "=========================================="
