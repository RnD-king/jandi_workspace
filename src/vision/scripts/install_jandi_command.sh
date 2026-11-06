#!/usr/bin/env bash
# Installs only a shell entrypoint. Never builds or starts a robot node.
set -euo pipefail
workspace="${1:-/home/rnd/jandi}"
core_prefix="${2:-$HOME/vision_core/install}"
bin_dir="${3:-$HOME/.local/bin}"
[[ -f "$workspace/install/setup.bash" ]] || { echo "설치 환경 없음: $workspace/install/setup.bash" >&2; exit 1; }
mkdir -p "$bin_dir"
if [[ -e "$bin_dir/run_jandi" || -L "$bin_dir/run_jandi" ]]; then
    if ! grep -q 'JANDI managed entrypoint' "$bin_dir/run_jandi" 2>/dev/null; then
        echo "기존 명령을 덮어쓰지 않습니다: $bin_dir/run_jandi" >&2
        exit 1
    fi
fi
{
    printf '%s\n' '#!/usr/bin/env bash' '# JANDI managed entrypoint' 'set -eo pipefail'
    printf 'source %q\n' /opt/ros/humble/setup.bash
    printf 'source %q\n' "$workspace/install/setup.bash"
    printf 'export ROS_DOMAIN_ID=22\nexport VISION_CORE_ROOT=%q\n' "$core_prefix"
    printf 'exec ros2 run vision run_jandi "$@" --workspace %q --core-prefix %q\n' "$workspace" "$core_prefix"
} > "$bin_dir/run_jandi"
chmod 755 "$bin_dir/run_jandi"
printf '등록 완료: %s/run_jandi\nPATH에 없다면: export PATH="%s:$PATH"\n' "$bin_dir" "$bin_dir"
