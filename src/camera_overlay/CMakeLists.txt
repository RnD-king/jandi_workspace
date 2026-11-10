cmake_minimum_required(VERSION 3.8)
project(camera_overlay)

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(Threads REQUIRED)

# YOLO TensorRT dependencies
find_package(vision REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ============================================================
# Shared Vision Core
# ============================================================

# Installed standalone shared_vision_core library.
#
# Default:
#   /home/noh/vision_core/install
#
# Can be overridden through VISION_CORE_ROOT or CMake.

set(
  _SHARED_VISION_CORE_DEFAULT
  "$ENV{HOME}/vision_core/install"
)

if(
  DEFINED ENV{VISION_CORE_ROOT}
  AND NOT "$ENV{VISION_CORE_ROOT}" STREQUAL ""
)
  set(
    _SHARED_VISION_CORE_DEFAULT
    "$ENV{VISION_CORE_ROOT}"
  )
endif()

set(
  SHARED_VISION_CORE_ROOT
  "${_SHARED_VISION_CORE_DEFAULT}"
  CACHE PATH
  "shared_vision_core installation prefix"
)

list(
  PREPEND CMAKE_PREFIX_PATH
  "${SHARED_VISION_CORE_ROOT}"
)

find_package(
  shared_vision_core
  CONFIG
  REQUIRED
)

# ============================================================
# Existing Camera Overlay Node
# ============================================================

add_executable(camera_overlay_node
  src/camera_overlay_node.cpp
)

target_compile_features(camera_overlay_node PRIVATE cxx_std_17)

target_compile_options(camera_overlay_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

target_include_directories(camera_overlay_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_overlay_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  cv_bridge
)

target_link_libraries(camera_overlay_node
  ${OpenCV_LIBS}
)

# ============================================================
# Existing Click Depth Node
# ============================================================

add_executable(click_depth_node
  src/click_depth_node.cpp
)

target_compile_features(click_depth_node PRIVATE cxx_std_17)

target_compile_options(click_depth_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

target_include_directories(click_depth_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(click_depth_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  cv_bridge
  message_filters
)

target_link_libraries(click_depth_node
  ${OpenCV_LIBS}
)

# ============================================================
# Existing YOLO TensorRT Library
# ============================================================

# Find existing vision package installation prefix.
#
# vision_DIR is normally:
# <prefix>/share/vision/cmake

get_filename_component(
  VISION_PREFIX
  "${vision_DIR}/../../.."
  ABSOLUTE
)

# Existing YOLO TensorRT header
find_path(
  VISION_YOLO_INCLUDE_DIR
  NAMES vision/yolo_trt_engine.hpp
  HINTS
    "${VISION_PREFIX}/include"
  REQUIRED
)

# Existing vision_core YOLO library
find_library(
  VISION_YOLO_LIBRARY
  NAMES vision_core
  HINTS
    "${VISION_PREFIX}/lib/vision"
  REQUIRED
)

# ============================================================
# TensorRT
# ============================================================

# TensorRT headers
find_path(
  TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/local/include
  REQUIRED
)

# TensorRT runtime library
find_library(
  TENSORRT_NVINFER
  NAMES nvinfer
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
  REQUIRED
)

# TensorRT plugin library
find_library(
  TENSORRT_NVINFER_PLUGIN
  NAMES nvinfer_plugin
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
  REQUIRED
)

# ============================================================
# Existing Ball Pixel Pin Node
# ============================================================

add_executable(ball_pixel_pin_node
  src/ball_pixel_pin_node.cpp
)

target_compile_features(ball_pixel_pin_node PRIVATE cxx_std_17)

target_compile_options(ball_pixel_pin_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

target_include_directories(ball_pixel_pin_node PRIVATE
  include
  ${VISION_YOLO_INCLUDE_DIR}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDAToolkit_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(ball_pixel_pin_node
  rclcpp
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(ball_pixel_pin_node
  ${VISION_YOLO_LIBRARY}
  ${OpenCV_LIBS}
  ${TENSORRT_NVINFER}
  ${TENSORRT_NVINFER_PLUGIN}
  CUDA::cudart
)

# Runtime library path for the existing YOLO library.
set_target_properties(ball_pixel_pin_node PROPERTIES
  BUILD_RPATH "${VISION_PREFIX}/lib/vision"
  INSTALL_RPATH "${VISION_PREFIX}/lib/vision"
)

# ============================================================
# NEW: Motion Calibration Node
# ============================================================
#
# Modes:
#
#   ball
#     BEFORE / AFTER ball center (u,v)
#     Result: delta_u, delta_v [pixel]
#
#   hurdle
#     BEFORE / AFTER LINE Offset / Heading
#     Result: delta_offset [pixel]
#             delta_heading [degree]
#
#   goal
#     BEFORE / AFTER goal rim X/Z
#     Result: delta_x, delta_z [meter]
#
# Initialization:
#
#   1. Send DEFAULT_POSE_MODE (Action 2)
#   2. Wait for actual DONE
#   3. Send mode-specific camera request
#   4. Wait for actual camera DONE
#   5. Begin measurement
#
# Test motions are executed separately through RQT.
#
# No CSV output.
# ============================================================

add_executable(motion_calibration_node
  src/motion_calibration_node.cpp
)

target_compile_features(
  motion_calibration_node
  PRIVATE cxx_std_17
)

target_compile_options(
  motion_calibration_node
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_include_directories(
  motion_calibration_node
  PRIVATE
    include
    ${VISION_YOLO_INCLUDE_DIR}
    ${TENSORRT_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(
  motion_calibration_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  ament_index_cpp
  vision
)

target_link_libraries(
  motion_calibration_node

  # Existing YOLO TensorRT engine
  ${VISION_YOLO_LIBRARY}

  # Shared mission/geometry algorithms
  shared_vision_core::shared_vision_core

  # OpenCV
  ${OpenCV_LIBS}

  # TensorRT
  ${TENSORRT_NVINFER}
  ${TENSORRT_NVINFER_PLUGIN}

  # CUDA runtime
  CUDA::cudart
)

# Runtime library paths.
#
# libvision_core.so:
#   <vision_prefix>/lib/vision
#
# libshared_vision_core.so:
#   <shared_vision_core_prefix>/lib

set_target_properties(
  motion_calibration_node
  PROPERTIES

  BUILD_RPATH
    "${VISION_PREFIX}/lib/vision;${SHARED_VISION_CORE_ROOT}/lib"

  INSTALL_RPATH
    "${VISION_PREFIX}/lib/vision;${SHARED_VISION_CORE_ROOT}/lib"
)

# ============================================================
# Existing Tests
# ============================================================

foreach(debug_target camera_overlay_node click_depth_node ball_pixel_pin_node motion_calibration_node)
  target_link_libraries(${debug_target} vision::vision_debug_mjpeg)
endforeach()

if(BUILD_TESTING)

  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME camera_overlay_http_debug_view_test
    COMMAND /usr/bin/python3 -s ${CMAKE_CURRENT_SOURCE_DIR}/tests/http_debug_view_test.py
      $<TARGET_FILE:click_depth_node>)
  set_tests_properties(camera_overlay_http_debug_view_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=89;ROS_LOCALHOST_ONLY=1" TIMEOUT 30)

  add_test(NAME camera_overlay_depth_color_test
    COMMAND /usr/bin/python3 -s ${CMAKE_CURRENT_SOURCE_DIR}/tests/depth_color_test.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/depth_color_viewer.py)
  set_tests_properties(camera_overlay_depth_color_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=90;ROS_LOCALHOST_ONLY=1" TIMEOUT 40)

  # ----------------------------------------------------------
  # Depth Measurement Test
  # ----------------------------------------------------------

  add_executable(camera_overlay_depth_measurement_test
    tests/depth_measurement_test.cpp
  )

  target_compile_features(
    camera_overlay_depth_measurement_test
    PRIVATE cxx_std_17
  )

  target_compile_options(
    camera_overlay_depth_measurement_test
    PRIVATE -Wall -Wextra -Wpedantic
  )

  target_include_directories(
    camera_overlay_depth_measurement_test
    PRIVATE
      include
      ${OpenCV_INCLUDE_DIRS}
  )

  target_link_libraries(
    camera_overlay_depth_measurement_test
    ${OpenCV_LIBS}
  )

  add_test(
    NAME camera_overlay_depth_measurement_test
    COMMAND camera_overlay_depth_measurement_test
  )

  # ----------------------------------------------------------
  # Goal Measurement Test
  # ----------------------------------------------------------

  add_executable(camera_overlay_goal_measurement_test
    tests/goal_measurement_test.cpp
  )

  target_compile_features(
    camera_overlay_goal_measurement_test
    PRIVATE cxx_std_17
  )

  target_compile_options(
    camera_overlay_goal_measurement_test
    PRIVATE -Wall -Wextra -Wpedantic
  )

  target_include_directories(
    camera_overlay_goal_measurement_test
    PRIVATE
      include
      ${OpenCV_INCLUDE_DIRS}
  )

  target_link_libraries(
    camera_overlay_goal_measurement_test
    ${OpenCV_LIBS}
  )

  add_test(
    NAME camera_overlay_goal_measurement_test
    COMMAND camera_overlay_goal_measurement_test
  )

endif()

# ============================================================
# Installation
# ============================================================

install(
  TARGETS
    camera_overlay_node
    click_depth_node
    ball_pixel_pin_node
    motion_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md HTTP_DEBUG_STREAMING.md DEPTH_COLOR_VIEWER.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/depth_color_viewer.py
  DESTINATION lib/${PROJECT_NAME} RENAME depth_color_viewer)
ament_package()
