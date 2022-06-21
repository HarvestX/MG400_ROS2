cmake_minimum_required(VERSION 3.8)
project(mg400_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Joint State Publisher GUI =========================================
ament_auto_add_executable(
  joint_state_publisher_gui
    ./src/joint_state_publisher_gui/main.cpp
    ./src/joint_state_publisher_gui/main_window.cpp)
target_include_directories(
  joint_state_publisher_gui PRIVATE include ${Qt5Widgets_INCLUDE_DIRS})
target_link_libraries(joint_state_publisher_gui Qt5::Widgets)
# End Joint State Publisher GUI =====================================

# ===================================================================
ament_auto_add_library(service_node SHARED src/service_node.cpp)
rclcpp_components_register_node(
  service_node
    PLUGIN "mg400_node::ServiceNode"
    EXECUTABLE service_node_exec)
# ===================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
