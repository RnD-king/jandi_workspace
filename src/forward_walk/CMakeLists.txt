

cmake_minimum_required(VERSION 3.5)
project(forward_walk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 이 줄을 추가하여 ROS 2 패키지 기능을 사용
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)  # 다이나믹셀 SDK 의존성
find_package(Eigen3 REQUIRED)         # Eigen3 의존성
find_package(std_msgs REQUIRED)  # std_msgs 패키지 의존성 추가
find_package(tf2 REQUIRED)



# 메시지 타입과 서비스가 필요하면 이 줄을 추가
find_package(rosidl_default_generators REQUIRED)



find_package(vision REQUIRED)

# 타겟에 대한 포함 디렉토리
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)


# P2P JSON parser/player도 main_node 실행 파일에 함께 컴파일한다.
# JSON 파싱은 header-only Boost.PropertyTree를 사용하므로 별도 링크 라이브러리는 없다.
add_executable(main_node
  src/main.cpp
  src/p2p_motion_player.cpp
  src/dynamixel.cpp
)

# 실행 파일에 필요한 ROS 2 라이브러리 링크
ament_target_dependencies(main_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  dynamixel_sdk
  Eigen3
  tf2  # tf2 의존성 추가
  vision
)

# ===== 새 센서 노드 =====
add_executable(sensor_node
  src/sensor.cpp
)
target_include_directories(sensor_node PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(sensor_node
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  Eigen3
)

# 설치 설정
install(TARGETS main_node
  sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY motions/
  DESTINATION share/${PROJECT_NAME}/motions
)

install(
  DIRECTORY missions/
  DESTINATION share/${PROJECT_NAME}/missions
)


install(PROGRAMS
  scripts/plot_leg_torque.py
  DESTINATION lib/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_executable(executor_logic_test
    tests/executor_logic_test.cpp
    src/p2p_motion_player.cpp
    src/dynamixel.cpp
  )
  ament_target_dependencies(executor_logic_test dynamixel_sdk Eigen3)
  target_compile_definitions(executor_logic_test PRIVATE
    FORWARD_WALK_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME executor_logic_test COMMAND executor_logic_test)
  # Compile the real MainNode against an in-memory Dxl, never the device driver.
  add_executable(executor_integration_test
    tests/executor_integration_test.cpp src/p2p_motion_player.cpp)
  ament_target_dependencies(executor_integration_test
    rclcpp ament_index_cpp vision Eigen3)
  target_compile_options(executor_integration_test PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/tests/fake_dynamixel.hpp)
  add_test(NAME executor_integration_test COMMAND executor_integration_test)
  set_tests_properties(executor_integration_test PROPERTIES
    TIMEOUT 20 ENVIRONMENT "ROS_DOMAIN_ID=87;ROS_LOCALHOST_ONLY=1")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME executor_source_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/source_contract_test.py)
endif()

ament_package()
