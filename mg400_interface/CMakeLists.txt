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

# Joint Handler =====================================================
ament_auto_add_library(
  joint_handler
    STATIC
      ./src/joint_handler.cpp)
set_property(TARGET joint_handler PROPERTY POSITION_INDEPENDENT_CODE ON)
# End Joint Handler =================================================

# Tcp Socket Handler ================================================
ament_auto_add_library(
  ${PROJECT_NAME}
    STATIC
      ./src/realtime_commander.cpp
      ./src/dashboard_commander.cpp
      ./src/tcp_socket_handler.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
# End Tcp Socket Handler ============================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_error_msg_generator test/src/test_error_msg_generator.cpp)
  target_link_libraries(test_error_msg_generator error_msg_generator)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    resources
)
