cmake_minimum_required(VERSION 3.8)
project(mg400_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_AUTOMOC ON)

set(include_files
  include/mg400_rviz_plugin/panel_mg400_controller.hpp)

foreach(header "${include_files}")
  qt5_wrap_cpp(include_wrapped "${header}")
endforeach()

file(GLOB src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
set(MY_LIB_NAME ${PROJECT_NAME})
ament_auto_add_library(${MY_LIB_NAME} SHARED ${src_files} ${include_wrapped})

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()