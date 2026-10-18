cmake_minimum_required(VERSION 3.16)
project(shared_vision_core VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(yaml-cpp REQUIRED)

add_library(shared_vision_core SHARED
  src/config_loader.cpp
  src/coordinate_rectifier.cpp
  src/cruise_selector.cpp
  src/line_detection_extractor.cpp
  src/line_feature_extractor.cpp
  src/line_p2p_controller.cpp
  src/line_plan.cpp
  src/line_mission_entry.cpp
  src/object_target_extractor.cpp
  src/object_association_tracker.cpp
  src/ball_controller.cpp
  src/hurdle_controller.cpp
  src/goal_controller.cpp
  src/control_command.cpp
  src/mission_controller.cpp
)
add_library(shared_vision_core::shared_vision_core ALIAS shared_vision_core)

target_include_directories(shared_vision_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(shared_vision_core PRIVATE yaml-cpp)
target_compile_definitions(shared_vision_core PRIVATE
  VISION_CORE_DEFAULT_ALGORITHM_CONFIG=\"${CMAKE_INSTALL_FULL_DATADIR}/${PROJECT_NAME}/config/vision_algorithm.yaml\"
)

install(TARGETS shared_vision_core
  EXPORT shared_vision_coreTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY config/ DESTINATION
  ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/shared_vision_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/shared_vision_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/shared_vision_core
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/shared_vision_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(EXPORT shared_vision_coreTargets
  FILE shared_vision_coreTargets.cmake
  NAMESPACE shared_vision_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/shared_vision_core
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/shared_vision_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/shared_vision_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/shared_vision_core
)

if(BUILD_TESTING)
  # Fixed scenario data keeps logic tests independent of operator tuning.
  set(VISION_CORE_TEST_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/mission_scenarios.yaml")
  add_executable(shared_vision_core_down_search_hold_test tests/down_search_hold_test.cpp)
  target_link_libraries(shared_vision_core_down_search_hold_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_down_search_hold_test COMMAND shared_vision_core_down_search_hold_test)
  set_tests_properties(shared_vision_core_down_search_hold_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  add_executable(shared_vision_core_mission_oct16_regression_test tests/mission_oct16_regression_test.cpp)
  target_link_libraries(shared_vision_core_mission_oct16_regression_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_mission_oct16_regression_test COMMAND shared_vision_core_mission_oct16_regression_test)
  set_tests_properties(shared_vision_core_mission_oct16_regression_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  add_executable(shared_vision_core_mission_revision_test tests/mission_revision_test.cpp)
  target_link_libraries(shared_vision_core_mission_revision_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_mission_revision_test COMMAND shared_vision_core_mission_revision_test)
  set_tests_properties(shared_vision_core_mission_revision_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  add_executable(shared_vision_core_staged_mission_contract_test tests/staged_mission_contract_test.cpp)
  target_link_libraries(shared_vision_core_staged_mission_contract_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_staged_mission_contract_test COMMAND shared_vision_core_staged_mission_contract_test)
  set_tests_properties(shared_vision_core_staged_mission_contract_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  add_executable(shared_vision_core_approach_stance_regression_test tests/approach_stance_regression_test.cpp)
  target_link_libraries(shared_vision_core_approach_stance_regression_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_approach_stance_regression_test COMMAND shared_vision_core_approach_stance_regression_test)
  set_tests_properties(shared_vision_core_approach_stance_regression_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  add_executable(shared_vision_core_review_regression_test tests/review_regression_test.cpp)
  target_link_libraries(shared_vision_core_review_regression_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_review_regression_test COMMAND shared_vision_core_review_regression_test)
  set_tests_properties(shared_vision_core_review_regression_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  add_executable(shared_vision_core_unified_sequence_test tests/unified_sequence_test.cpp)
  target_link_libraries(shared_vision_core_unified_sequence_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_unified_sequence_test COMMAND shared_vision_core_unified_sequence_test)
  set_tests_properties(shared_vision_core_unified_sequence_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  add_executable(shared_vision_core_config_loader_test
    tests/config_loader_test.cpp
  )
  target_link_libraries(shared_vision_core_config_loader_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_config_loader_test
    COMMAND shared_vision_core_config_loader_test
            ${VISION_CORE_TEST_CONFIG}
  )

  add_executable(shared_vision_core_ball_controller_test
    tests/ball_controller_test.cpp
  )
  target_link_libraries(shared_vision_core_ball_controller_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_ball_controller_test
    COMMAND shared_vision_core_ball_controller_test
  )

  add_executable(shared_vision_core_hurdle_controller_test
    tests/hurdle_controller_test.cpp
  )
  target_link_libraries(shared_vision_core_hurdle_controller_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_hurdle_controller_test
    COMMAND shared_vision_core_hurdle_controller_test
  )

  add_executable(shared_vision_core_goal_controller_test
    tests/goal_controller_test.cpp
  )
  target_link_libraries(shared_vision_core_goal_controller_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_goal_controller_test
    COMMAND shared_vision_core_goal_controller_test
  )

  add_executable(shared_vision_core_line_detection_stability_test
    tests/line_detection_stability_test.cpp
  )
  target_link_libraries(shared_vision_core_line_detection_stability_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_line_detection_stability_test
    COMMAND shared_vision_core_line_detection_stability_test
  )

  add_executable(shared_vision_core_control_command_test
    tests/control_command_test.cpp
  )
  target_link_libraries(shared_vision_core_control_command_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_control_command_test
    COMMAND shared_vision_core_control_command_test
  )

  add_executable(shared_vision_core_line_p2p_controller_test
    tests/line_p2p_controller_test.cpp
  )
  target_link_libraries(shared_vision_core_line_p2p_controller_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_line_p2p_controller_test
    COMMAND shared_vision_core_line_p2p_controller_test
  )

  add_executable(shared_vision_core_mission_controller_test
    tests/mission_controller_test.cpp
  )
  target_link_libraries(shared_vision_core_mission_controller_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_mission_controller_test
    COMMAND shared_vision_core_mission_controller_test
  )

  add_executable(shared_vision_core_perception_pipeline_test
    tests/perception_pipeline_test.cpp
  )
  target_link_libraries(shared_vision_core_perception_pipeline_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_perception_pipeline_test
    COMMAND shared_vision_core_perception_pipeline_test
  )

  add_executable(shared_vision_core_object_association_tracker_test
    tests/object_association_tracker_test.cpp
  )
  target_link_libraries(shared_vision_core_object_association_tracker_test
    PRIVATE shared_vision_core
  )
  add_test(
    NAME shared_vision_core_object_association_tracker_test
    COMMAND shared_vision_core_object_association_tracker_test
  )

  set_tests_properties(
    shared_vision_core_config_loader_test
    shared_vision_core_ball_controller_test
    shared_vision_core_hurdle_controller_test
    shared_vision_core_goal_controller_test
    shared_vision_core_line_detection_stability_test
    shared_vision_core_control_command_test
    shared_vision_core_line_p2p_controller_test
    shared_vision_core_mission_controller_test
    shared_vision_core_perception_pipeline_test
    shared_vision_core_object_association_tracker_test
    PROPERTIES ENVIRONMENT
      "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}"
  )
  # Logic tests always use the committed scenario fixture, including default
  # controller constructors. Robot nodes keep their production default path.
  add_executable(shared_vision_core_line_plan_entry_test tests/line_plan_entry_test.cpp)
  target_link_libraries(shared_vision_core_line_plan_entry_test PRIVATE shared_vision_core)
  add_test(NAME shared_vision_core_line_plan_entry_test COMMAND shared_vision_core_line_plan_entry_test)
  set_tests_properties(shared_vision_core_line_plan_entry_test PROPERTIES ENVIRONMENT
    "VISION_CORE_ALGORITHM_CONFIG=${VISION_CORE_TEST_CONFIG}")
  get_property(_logic_tests DIRECTORY PROPERTY TESTS)
  set_tests_properties(${_logic_tests} PROPERTIES LABELS "logic")

  add_executable(shared_vision_core_runtime_config_check tests/runtime_config_check.cpp)
  target_link_libraries(shared_vision_core_runtime_config_check PRIVATE shared_vision_core yaml-cpp)
  add_test(NAME shared_vision_core_runtime_config_check
    COMMAND shared_vision_core_runtime_config_check
      ${CMAKE_CURRENT_SOURCE_DIR}/config/vision_algorithm.yaml)
  set_tests_properties(shared_vision_core_runtime_config_check PROPERTIES LABELS "config")
endif()
