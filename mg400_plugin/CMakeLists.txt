cmake_minimum_required(VERSION 3.8)
project(mg400_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB DASHBOARD_API_SRCS ./src/dashboard_api/*.cpp)
ament_auto_add_library(
  mg400_dashboard_api_plugin
  SHARED
    ${DASHBOARD_API_SRCS})
pluginlib_export_plugin_description_file(
  mg400_plugin_base dashboard_api_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()