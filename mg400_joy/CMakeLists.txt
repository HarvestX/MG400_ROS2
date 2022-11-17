cmake_minimum_required(VERSION 3.8)
project(mg400_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Joy Interface Node ================================================
set(TARGET mg400_joy_interface_node)
ament_auto_add_library(
  ${TARGET}
    SHARED
      src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "mg400_joy::MG400JoyInterfaceNode"
    EXECUTABLE ${TARGET}_exec)
# End Joy Interface Node ============================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
