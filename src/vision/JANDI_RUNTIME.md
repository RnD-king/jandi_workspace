# JANDI 실행

버전별 빌드·설치와 최초 Jetson 설정은 [JANDI_DUAL_INSTALL.md](JANDI_DUAL_INSTALL.md)를 따른다.

```bash
~/jandi_update.sh main
~/jandi_update.sh fast
~/run_jandi.sh
```

업데이트는 선택 버전의 GitHub 수신·Release 빌드·설치다. 자동 테스트나 노드 실행을 하지 않는다.
실행 메뉴는 MAIN/FAST와 OpenCV·터미널·HTTP ON/OFF만 제공한다. PRACTICE/IRC 구분은 없다.
기본 옵션은 OpenCV OFF / 터미널 OFF / HTTP ON이다.

카메라, 모터, 비전은 기존 0/5/10초 시작을 따르며 미션 START는 수동이다.
콘솔 4/5/6/7에서 START/PAUSE/RESUME/RESET을 선택하고 Enter로 보낸다.
9 또는 Ctrl+C는 콘솔만 나간다. 다시 run_jandi.sh로 기존 콘솔에 연결한다.
0 → 1은 전체 정상 종료, K → 1은 기존 긴급종료다. 0/Esc로 확인을 취소할 수 있다.
실행 중인 버전의 업데이트와 MAIN/FAST 동시 실행은 거부한다.

알고리즘 파라미터는 선택 버전의 소스 YAML을 직접 편집한다.
PC MAIN은 `~/vision_core/config/vision_algorithm.yaml`, FAST는
`~/vision_core_fast/config/vision_algorithm.yaml`이다. Jetson MAIN은
`~/jandi_main/src/vision_core/config/vision_algorithm.yaml`, FAST는
`~/jandi_fast/src/vision_core/config/vision_algorithm.yaml`이다.
저장 후 JANDI 전체 종료·재실행으로 반영하며 `cp`/재빌드는 필요 없다.
PC에서 push하고 Jetson에서 update하면 YAML 값도 그대로 공유된다.
기존 `~/.config/jandi/main·fast/vision_algorithm.yaml`은 보존하지만 실행에 사용하지 않는다.
잘못된 소스 설정은 실제 C++ 로더로 시작 전에 검사하여 카메라·모터 시작을 막는다.
이전 홈 튜닝 방식에서 전환할 때는 JANDI_DUAL_INSTALL.md의 일회 도구 갱신 명령을 따른다.

엔진 경로는 선택 버전의 소스 `vision/config/yolo26_runtime.yaml`에서만 관리한다.
PC는 pc_engine_path, Jetson은 jetson_engine_path다. deployment.json의 engine 값은 제거됐다.
경로 변경 후 전체 종료하고 다시 실행하면 반영되며 엔진 경로만 바꿀 때 재빌드는 필요 없다.
정확한 MAIN/FAST 소스 경로는 JANDI_DUAL_INSTALL.md를 참조한다.

노출의 시작값은 기존 `realsense2_camera/launch/rs_launch.py`의 마지막 parameters 딕셔너리에서
직접 수정한다. 추가 카메라 launch/YAML override는 없다.
확인된 PC/Jetson은 소스 링크 설치로 재빌드 없이 카메라 재시작 때 적용된다.

```bash
ros2 launch realsense2_camera rs_launch.py
# 별도 터미널에서 ROS 환경과 사용할 camera_overlay 설치본을 source한 뒤:
ros2 run camera_overlay depth_color_viewer
```

Jetson 뷰어: http://192.168.0.133:8081, PC: http://localhost:8081.
노출 실시간 변경은 재빌드·카메라 재시작 없이 적용하며 파일을 자동 수정하지 않는다.
