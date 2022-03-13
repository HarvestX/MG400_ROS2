cmake_minimum_required(VERSION 3.8)
project(mg400_control)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mg400_msgs REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp sensor_msgs mg400_msgs)

# Extra package
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Joint State Publsher GUI Node =====================================
set(TARGET joint_state_publisher_gui)
add_executable(${TARGET})
target_sources(
  ${TARGET} #
  PRIVATE src/${TARGET}/main.cpp #
  PRIVATE src/${TARGET}/main_window.cpp #
)
target_include_directories(
  ${TARGET} PRIVATE include ${Qt5Widgets_INCLUDE_DIRS} #
)
target_link_libraries(${TARGET} Qt5::Widgets)
ament_target_dependencies(
  ${TARGET} #
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Append excutable to list
set(THIS_CREATED_EXEC_LIST ${THIS_CREATED_EXEC_LIST} ${TARGET})
# End Joint State Publisher GUI Node ================================

# MG400 Control Node ================================================
set(TARGET mg400_control)
set(LIB_TARGET mg400_control_component)

# Create library
add_library(${LIB_TARGET} STATIC)
# cmake-format: off
target_sources(
  ${LIB_TARGET} #
  PRIVATE
    src/${TARGET}/component.cpp
    src/${TARGET}/commander.cpp
    src/${TARGET}/tcp_socket.cpp
)
# cmake-format: on
target_include_directories(${LIB_TARGET} PRIVATE include)
ament_target_dependencies(${LIB_TARGET} ${THIS_PACKAGE_INCLUDE_DEPENDS})
set_property(TARGET ${LIB_TARGET} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Create Node
add_executable(${TARGET})
# cmake-format: off
target_sources(
  ${TARGET}
  PRIVATE
    src/${TARGET}/main.cpp
)
# cmake-format: on
target_link_libraries(${TARGET} ${LIB_TARGET})
target_include_directories(${TARGET} PRIVATE include)
ament_target_dependencies(${TARGET} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Append library and excutable to list
set(THIS_CREATED_LIBRARY_LIST ${THIS_CREATED_LIBRARY_LIST} ${LIB_TARGET})
set(THIS_CREATED_EXEC_LIST ${THIS_CREATED_EXEC_LIST} ${TARGET})
# End MG400 Control Node ============================================

# Install targets ===================================================
install(TARGETS ${THIS_CREATED_EXEC_LIST} DESTINATION lib/${PROJECT_NAME})
# End Install targets ===============================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
