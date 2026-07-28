cmake_minimum_required(VERSION 3.8)
project(mg400_operation_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mg400_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(
  mg400_operation_gui
  include/mg400_operation_gui/main_window.hpp
  src/main.cpp
  src/main_window.cpp)

target_include_directories(mg400_operation_gui PRIVATE include)
ament_target_dependencies(
  mg400_operation_gui
  geometry_msgs
  mg400_msgs
  rclcpp)
target_link_libraries(mg400_operation_gui Qt5::Widgets)

set_target_properties(
  mg400_operation_gui PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON)

install(TARGETS mg400_operation_gui DESTINATION lib/${PROJECT_NAME})

ament_package()
