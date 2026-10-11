#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
rule_source="$script_dir/../udev/99-jandi-dxl.rules"
rule_target=/etc/udev/rules.d/99-jandi-dxl.rules
# 관리자 암호는 sudo가 터미널에서 직접 받는다.
sudo -v
if [[ -e "$rule_target" ]] && ! cmp -s "$rule_source" "$rule_target"; then
    echo "기존 $rule_target 내용이 다릅니다. 덮어쓰지 않았습니다." >&2
    exit 1
fi
sudo -n install -m 0644 "$rule_source" "$rule_target"
sudo -n udevadm control --reload-rules
# 확인된 FTDI 어댑터만 다시 평가한다. 모터 명령이나 전체 USB 재연결을 하지 않는다.
for task_device in /sys/class/tty/ttyUSB*; do
    [[ -e "$task_device" ]] || continue
    if udevadm info --query=property --path="$task_device" | rg -q '^ID_SERIAL_SHORT=FT9BTGLH$'; then
        sudo -n udevadm trigger --action=add "$task_device"
    fi
done
udevadm settle --timeout=10
if [[ ! -e /dev/jandi_dxl ]]; then
    echo '규칙 설치 완료. 해당 어댑터를 다시 연결한 뒤 /dev/jandi_dxl을 확인하세요.' >&2
    exit 2
fi
udevadm info --query=property --name=/dev/jandi_dxl | rg '^ID_SERIAL_SHORT=FT9BTGLH$'
ls -l /dev/jandi_dxl
printf '\n설정 완료. 이제 추가 포트 옵션 없이 실행할 수 있습니다:\nros2 run forward_walk main_node\n'
