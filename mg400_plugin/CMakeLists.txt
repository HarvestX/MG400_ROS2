cmake_minimum_required(VERSION 3.8)
project(mg400_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


set(TARGETS
  dashboard_api
  motion_api)

foreach(TARGET ${TARGETS})
  file(GLOB MY_SRCS ./src/${TARGET}/*.cpp)
  ament_auto_add_library(
    ${PROJECT_NAME}_${TARGET}
    SHARED
      ${MY_SRCS}
      src/tf_manager.cpp)
  pluginlib_export_plugin_description_file(
    mg400_plugin_base ${TARGET}_plugins.xml)
endforeach()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_move_jog_stop_waiter
    test/test_move_jog_stop_waiter.cpp)
  target_include_directories(
    test_move_jog_stop_waiter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_move_jog_stop_waiter mg400_interface)
endif()

ament_auto_package()
