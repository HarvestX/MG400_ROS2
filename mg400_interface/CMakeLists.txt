cmake_minimum_required(VERSION 3.8)
project(mg400_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================================
set(TARGET ${PROJECT_NAME})
ament_auto_add_library(
  ${TARGET}
    STATIC
      ./src/command_utils/move_jog.cpp
      ./src/commander/dashboard_commander.cpp
      ./src/commander/motion_commander.cpp
      ./src/commander/response_parser.cpp
      ./src/error_msg_generator.cpp
      ./src/joint_handler.cpp
      ./src/mg400_interface.cpp
      ./src/tcp_interface/dashboard_tcp_interface.cpp
      ./src/tcp_interface/motion_tcp_interface.cpp
      ./src/tcp_interface/realtime_feedback_tcp_interface.cpp
      ./src/tcp_interface/tcp_socket_handler.cpp)
set_property(TARGET ${TARGET} PROPERTY POSITION_INDEPENDENT_CODE ON)
# ===================================================================

# Example ===========================================================
ament_auto_add_executable(
  show_realtime_data
    ./example/show_realtime_data.cpp)
target_link_libraries(show_realtime_data ${TARGET})
# End Example =======================================================

# Example ===========================================================
ament_auto_add_executable(
  commander_check
    ./example/commander_check.cpp)
target_link_libraries(commander_check ${TARGET})
# End Example =======================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  set(TEST_TARGETS
    test_error_msg_generator
    test_joint_handler)
  foreach(TARGET ${TEST_TARGETS})
    ament_add_gtest(${TARGET} test/src/${TARGET}.cpp)
    target_link_libraries(${TARGET} ${PROJECT_NAME})
  endforeach()

  set(TEST_TARGETS
    test_response_parser
    test_motion_commander
    test_dashboard_commander)
  foreach(TARGET ${TEST_TARGETS})
    ament_add_gmock(${TARGET} test/src/commander/${TARGET}.cpp)
    target_link_libraries(${TARGET} ${PROJECT_NAME})
  endforeach()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    resources
)
