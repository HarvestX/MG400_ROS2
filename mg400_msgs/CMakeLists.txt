cmake_minimum_required(VERSION 3.8)
project(mg400_msgs)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
    "srv/ResetRobot.srv"
    "srv/ClearError.srv"
    "srv/DisableRobot.srv"
    "srv/EnableRobot.srv"
    "srv/ToolDOExecute.srv"
    "srv/MoveJog.srv"
    "srv/MovJ.srv"
    "srv/MovL.srv"
    "srv/JointMovJ.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
