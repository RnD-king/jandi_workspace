# 모터 상태 로그

main 실행 중 ID 0~22를 하나씩 읽어 Torque Enable(64), Hardware Error Status(70),
Present Input Voltage(144), Present Temperature(146)를 확인한다.
레지스터 해석은 ROBOTIS MX Protocol 2.0 문서를 따른다:
https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#hardware-error-status70

오류/토크 OFF/응답 실패 시 콘솔 ERROR와 `/home/hyeongil/jandi/src/forward_walk/log/motor_health_*.log`에 기록한다.
최초 발견과 상태 변경은 즉시, 같은 오류 지속은 5초마다 기록하고 정상 복구도 기록한다.
로그의 mission/action/action_id는 **진단 응답을 받은 시점**의 활성 명령이다.
phase=startup/action/camera/idle로 구분하며 idle일 때 mission/action_id는 0이다.
last_mission/last_action은 마지막 접수 동작으로 현재 실행 중이라는 뜻은 아니다.
queued_id와 camera_id, 토크 상태, 오류 비트, 전압, 온도, 연속 읽기 실패 횟수도 함께 남긴다.
응답이 없으면 상태 값은 -1이고, 전원/케이블/USB 지연 중 무엇이 원인인지는 확정하지 않는다.
토크 OFF인데 하드웨어 오류가 없을 때도 원인을 추측으로 단정하지 않는다.

기본은 50ms에 한 ID, 전체 23개 한 바퀴 약 1.15초이다.
움직이는 동안 다음 목표 전송까지 timeout+2ms 여유가 없으면 진단을 미뤄 탐지 시간이 늘 수 있다.
짧은 이상이나 명령 경계에서 발생한 문제를 정확한 발생 순간/명령에 귀속시키는 기능은 아니다.
읽기는 같은 executor에서 직렬화하며 timeout은 기본 5ms이다.
OS scheduling과 USB 지연까지 실시간으로 보장하지는 않으며, 진단 지연도 WARN으로 기록한다.
USB latency_timer가 길면 실제로 정상인 모터도 timeout으로 표시될 수 있다.
모터 재부팅, 토크 자동 재활성화, 자동 정지는 하지 않는다. P2P 목표와 프로토콜은 유지한다.

빌드한 forward_walk 소스 패키지의 `log/` 폴더를 자동 생성하여 저장한다.
현재 PC 기본 경로는 `/home/hyeongil/jandi/src/forward_walk/log/`이다.
Jetson에서 빌드하면 해당 Jetson 소스 패키지 아래에 저장한다.
파일 저장에 실패하더라도 터미널 ERROR 출력은 계속된다.

ROS 파라미터:
- motor_health_log_directory: 빌드한 소스 패키지의 log 폴더 (필요하면 경로 지정 가능)
- motor_health_monitor_enabled: true
- motor_health_poll_interval_sec: 0.05 (최소 0.02)
- motor_health_timeout_ms: 5.0 (0 초과, 10 이하)

최신 파일 확인: `ls -t /home/hyeongil/jandi/src/forward_walk/log/motor_health_*.log`
추적: `tail -f /home/hyeongil/jandi/src/forward_walk/log/motor_health_<실행시각>.log`
