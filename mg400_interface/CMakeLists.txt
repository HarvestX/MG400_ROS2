cmake_minimum_required(VERSION 3.8)
project(mg400_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Error Message Generator ===========================================
ament_auto_add_library(
  error_msg_generator
    STATIC
      ./src/error_msg_generator.cpp)
target_link_libraries(error_msg_generator stdc++fs jsoncpp)
set_property(TARGET error_msg_generator PROPERTY POSITION_INDEPENDENT_CODE ON)
# End Error Message Generator =======================================

# Response Parser ===================================================
ament_auto_add_library(
  response_parser
    STATIC
      ./src/response_parser.cpp)
set_property(TARGET response_parser PROPERTY POSITION_INDEPENDENT_CODE ON)
# End Response Parser ===============================================

# Joint Handler =====================================================
ament_auto_add_library(
  joint_handler
    STATIC
      ./src/joint_handler.cpp)
set_property(TARGET joint_handler PROPERTY POSITION_INDEPENDENT_CODE ON)
# End Joint Handler =================================================

# Tcp Socket Handler ================================================
ament_auto_add_library(
  tcp_interface
    STATIC
      ./src/tcp_interface/dashboard_tcp_interface.cpp
      ./src/tcp_interface/motion_tcp_interface.cpp
      ./src/tcp_interface/realtime_feedback_tcp_interface.cpp
      ./src/tcp_interface/tcp_socket_handler.cpp)
set_property(TARGET tcp_interface PROPERTY POSITION_INDEPENDENT_CODE ON)
# End Tcp Socket Handler ============================================

# ===================================================================
ament_auto_add_library(
  command_utils
    STATIC
      ./src/command_utils/move_jog.cpp)
set_property(TARGET command_utils PROPERTY POSITION_INDEPENDENT_CODE ON)
# ===================================================================

# ===================================================================
ament_auto_add_library(
  commander
    STATIC
      ./src/commander/dashboard_commander.cpp
      ./src/commander/motion_commander.cpp)
set_property(TARGET commander PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(commander command_utils)
# ===================================================================

# Example ===========================================================
ament_auto_add_executable(
  show_realtime_data
    ./example/show_realtime_data.cpp)
target_link_libraries(show_realtime_data tcp_interface)
# End Example =======================================================

# Example ===========================================================
ament_auto_add_executable(
  commander_check
    ./example/commander_check.cpp)
target_link_libraries(commander_check tcp_interface commander)
# End Example =======================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gtest(test_error_msg_generator test/src/test_error_msg_generator.cpp)
  target_link_libraries(test_error_msg_generator error_msg_generator)

  ament_add_gtest(test_response_parser test/src/test_response_parser.cpp)
  target_link_libraries(test_response_parser response_parser)

  ament_add_gmock(test_motion_commander test/src/commander/test_motion_commander.cpp)
  target_link_libraries(test_motion_commander commander)

  ament_add_gmock(test_dashboard_commander test/src/commander/test_dashboard_commander.cpp)
  target_link_libraries(test_dashboard_commander commander)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    resources
)
