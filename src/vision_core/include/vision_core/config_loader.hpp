#pragma once

#include <string>

#include "vision_core/mission_controller.hpp"

namespace vision_core {

// VISION_CORE_ALGORITHM_CONFIG가 설정되어 있으면 그 경로를, 아니면 설치된
// shared_vision_core의 공통 algorithm YAML 경로를 반환한다.
std::string DefaultAlgorithmConfigPath();

// YAML, ROS parameter override, 직접 C++ 구성 경로가 같은 규칙을 사용하도록
// 완성된 algorithm config 전체를 검증한다. 위반 시 std::runtime_error를 던진다.
void ValidateAlgorithmConfig(const MissionControllerConfig &config);

// YAML의 모든 필수 키를 읽는다. 파일, 키, 타입이 잘못되면
// 일부가 0으로 남은 config를 사용하지 않고 std::runtime_error를 던진다.
MissionControllerConfig LoadAlgorithmConfig(const std::string &path);

// 기본 공통 YAML을 읽는다. 파일이 없거나 잘못되면 예외를 던진다.
MissionControllerConfig LoadDefaultAlgorithmConfig();

} // namespace vision_core
