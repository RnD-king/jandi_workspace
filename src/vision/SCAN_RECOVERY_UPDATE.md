# LINE 복구·목 탐색·PICK 전 재검증 변경

PC 원본만 수정했다. MAIN은 `~/my_cv/src/{vision,forward_walk}` 및
`~/vision_core`, FAST는 `~/my_cv/variants/fast/src/{vision,forward_walk}` 및
`~/vision_core_fast`다. 업로드용 staging 저장소·Jetson·백업·JSON 모션은 변경하지 않았다.

## 확정한 동작

- LINE 정상 검출 실패 후 기존 1초 정지 관측을 수행한다.
  0~1점은 목 탐색, 2점은 마지막 정상 방향으로 최대 1회,
  3점 이상은 유효한 현재 복구 O/H 우선으로 최대 2회 회전전진한다.
  현재 O/H가 무효이면 안정 검출한 실제 바닥 LINE O/H로 대체한다.
  방향을 정할 수 없으면 임의 우회전 없이 목 탐색한다.
- 복구 횟수는 유실 구간 전체의 누적 횟수다. 점이 줄면 상한을 줄이고,
  점이 늘어도 상한을 늘리거나 실행 횟수를 초기화하지 않는다.
  복구 1회는 실제 LINE 단위 동작 1개이며 READY에서 후속 복구를 예약하지 않는다.
  실제 DONE 후 새 1초 관측을 거쳐 다음 결정을 한다.
- 마지막 정상 LINE 정보와 유실 시작에 고정한 복구용 정보를 분리했다.
  객체 가상선·목 이동 영상·복구 임시 가이드가 정상 방향을 덮어쓰지 않는다.
- 목 도착 후 해당 정지 위치에서 촬영한 고유 프레임만 사용한다.
  미검출도 관측창에 포함하며 MAIN 10중5, FAST 8중4 및 최소 0.5초가 필요하다.
  최대 2초다. 몸 회전 후 재관측의 시간 제한은 RGB 없이도 기존 transport timer가 확인한다.
  시간 확인은 검출·동작 DONE을 만들거나 새 몸 동작을 발행하지 않는다.
  목 정지 위치의 프레임 단절은 기존 실행기 watchdog이 위치당 최대 관측 시간으로 제한한다.
- 공·골대는 정지 관측의 유효 u 평균과 실제 CameraInfo 내부 파라미터로
  대략적인 수평 보정각을 계산한다. pitch 보정은 이번 합의대로 생략했다.
  6도 이내는 0회, 30도 이하는 2회, 초과는 3회다. 좌우는 계산한 각도의 부호로 정한다.
- LINE은 중앙 밖이어도 유효한 가이드가 안정 검출되면 발견으로 인정한다.
  좌/우 목 탐색 위치에서 발견하면 해당 방향으로 3회, 정면이면 0회 회전한다.
  O/H를 실제 몸 회전각으로 환산하지 않는다. 몸 회전 후 정상 LINE을 다시 확인한다.
- LINE/GOAL의 최초 목 방향은 마지막 유효 위치를 우선한다. 재관측에 실패하면
  직전에 발견한 목 방향을 우선한다. 오른쪽부터 시작해도 오른쪽→왼쪽→정면을 모두 방문한다.
- BALL 커브는 기존 코스별 허용 방향·팔 정책을 우선한다. 현재 커브별 단일 허용 방향과
  충돌하는 화면 위치로 반대쪽을 탐색하지 않는다. 일반 BALL 유실 진입 조건은 변경하지 않았다.
- LINE·커브 BALL·GOAL은 발견과 보정 후 재관측 실패를 포함하여 최대 3회 탐색한다.
  중간 발견으로 탐색 횟수를 초기화하지 않는다. 실제 몸 동작 DONE을 하나씩 확인한다.
  커브 탐색 사이의 기존 전진 probe와 발견 후 회전전진은 새로운 회전·재관측 절차로 대체했다.
  소진 후 LINE FINAL HOLD, BALL 코스 잠금/인덱스, GOAL 기존 복귀/실패 경로를 사용한다.
- 몸 회전 후 GOAL은 목 탐색의 10중5/8중4 대신 정상 10중7/8중6과 유효 Depth가 필요하다.
  새 GOAL ID는 즉시 채택하지만 진행 중 보정·재관측 절차를 건너뛰지 않는다.
  Depth만 없는 경우의 기존 접근 복구 진입은 유지한다.
- PICK 후보는 즉시 집지 않는다. 과거 좌표·관측 기록을 버리고 새 안정 관측을
  최소 1초 수집한 후 현재 유효 픽업 다각형으로 다시 판단한다.
  밖이면 FINE, 유실이면 기존 복구, 내부이면 PICK이다.
  후보·관측·복사 PREVIEW는 실제 controller의 픽업 횟수를 소모하지 않는다.
  coordinator가 실제 PICK ActionCommand를 발행한 시점에만 1회 기록하고 재전송은 중복 기록하지 않는다.
  수동 PREVIEW/COMMIT도 실제 controller에서 재검증을 진행하며 사람의 대기 시간을 관측으로 세지 않는다.
- MAIN 허들 FINE 우회전 실측값은 8→11도로 변경했다. FAST는 기존 11도를 유지했다.

실행부에 요청하는 기존 15도와 `turn_step_deg: 15.0`은 바꾸지 않았다.
12도는 탐색 보정 계산용 실측값이고 허들 FINE의 11도와 별개다.
FAST의 8프레임·빠른 카메라 전환/보행 코드, GOAL FINE 자세 후 관측 및
투척 전 1초 검증, 기존 READY 평균·1슬롯 예약·실제 ACK/READY/DONE은 유지한다.

## 새 파라미터

각 버전의 `vision_algorithm.yaml`에서 지정한다.

| YAML 경로 | 기본값 | 의미 |
|---|---:|---|
| `ball.control.pickup_verification_sec` | 1.0 | PICK 후보 이후 추가 정지 관측 시간. 양수만 허용 |
| `common.scan_correction.measured_turn_deg` | 12.0 | 계산용 실측값. 절반 이내 정면 예외 |
| `common.scan_correction.near_angle_deg` | 30.0 | 2회/3회 구분각 |
| `common.scan_correction.max_attempts` | 3 | 탐색 묶음 상한, 1~3 허용 |

PICK 지연의 ROS 파라미터는 `ball_pickup_verification_sec`다.
과거 YAML/직접 호출 fixture의 새 항목 생략에는 같은 기본값을 사용한다.
기존 `line.recovery.body_max_turns`, `scan_body_turn_deg`, GOAL의
`head_scan_body_turn_*_threshold_deg`, 커브 `body_search_max_attempts`는
호환을 위해 남아 있지만 새 AUTO 목 탐색의 횟수 계산에는 사용하지 않는다.
`scan_center_half_width_px`/`head_scan_center_half_width_px`는 최초 탐색 방향의
중앙 근접 판단에 쓰며 LINE 목 탐색 발견을 중앙으로 제한하지 않는다.

## 수정 파일

두 core 원본: `mission_controller.hpp/.cpp`, `line_mission_entry.cpp`,
`continuous_object_approach.cpp`, `ball_controller.hpp/.cpp`, `types.hpp`,
`stationary_scan_observation.hpp`, 새 `scan_body_correction.hpp`, `config_loader.cpp`,
`config/vision_algorithm.yaml`, `CMakeLists.txt` 및 관련 tests/fixture.

두 vision 원본: `line_perception_node.cpp`, `perception_tuning_node.cpp`,
`mission_parameters.hpp`, `mission_debug_view.hpp`, `mission_parameters_test.cpp`,
이 문서·`TUNING_HTTP_GUIDE.md`·문서 설치 CMake.

두 forward_walk 원본: `motion_executor_state.hpp`, `scan_motion_test.cpp`.

## 빌드 및 사용자 검사

빌드·설치·테스트·장비 실행은 수행하지 않았다. 결과는 아직 미확인이다.
[TUNING_HTTP_GUIDE.md](TUNING_HTTP_GUIDE.md)의 **PC 미전송 소스 빌드·설치**를
MAIN/FAST 각각 실행한다. 이 단계에서 GitHub pull/update를 먼저 하지 않는다.
그 문서의 검사 명령에 이번 회귀 검사와 실행기 목 탐색 검사를 추가했다.

추가/변경한 검사는 0/1/2/3점 복구·퇴화 피팅·고정 기억값·횟수 감소,
중앙 밖 LINE, 정지 프레임 중복/최소 시간/시간 초과,
좌표 보정과 2/3회 경계, 실제 DONE와 중복 DONE, RGB 단절 시 시간 초과,
3회 탐색 소진, GOAL 일반 안정 조건/Depth,
PICK 전 독립 관측/수동 승인/유실/정렬 재개/픽업 후 검증,
ROS 파라미터 및 오른쪽 우선 목 순서를 포함한다.

실제 회전량 12도, pitch를 생략한 대략 방향 보정의 적합성,
Jetson 실행·카메라/로봇의 실제 동작은 장비에서 확인해야 한다.
기존 GOAL 투척 전 검증에 관련된 과거 테스트 기대값 문제 등
이번 범위 밖의 실패가 있으면 별도 원인 확인이 필요하다.

작업 전 원격 HEAD와 staging HEAD는 각각 일치했다.
커밋·push를 하지 않았으므로 작업 후 HEAD도 동일하며 PC 원본 수정만 존재한다.

- MAIN: `f0340a88a15e165ba24fb35b6248489830453c53`
- FAST: `7c9fa835088d194a8e233fc23d2c626c58086010`
