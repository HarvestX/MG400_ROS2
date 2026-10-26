cmake_minimum_required(VERSION 3.8)
project(mg400_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)
file(GLOB action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*.action)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files} ${srv_files} ${action_files}
  DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
