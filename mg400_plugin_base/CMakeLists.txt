cmake_minimum_required(VERSION 3.8)
project(mg400_plugin_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_motion_api_plugin_base
    test/src/test_motion_api_plugin_base.cpp)
  target_include_directories(
    test_motion_api_plugin_base
    PRIVATE include)
  ament_target_dependencies(
    test_motion_api_plugin_base
    mg400_interface
    mg400_msgs
    rclcpp)
endif()

ament_export_dependencies(rclcpp)
ament_export_dependencies(pluginlib)
ament_export_dependencies(mg400_interface)
ament_export_dependencies(mg400_msgs)
ament_auto_package()
