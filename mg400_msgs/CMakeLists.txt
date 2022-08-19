cmake_minimum_required(VERSION 3.8)
project(mg400_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(srv_files
    "srv/AccJ.srv"
    "srv/AccL.srv"
    "srv/ClearError.srv"
    "srv/DisableRobot.srv"
    "srv/EnableRobot.srv"
    "srv/JointMovJ.srv"
    "srv/MoveJog.srv"
    "srv/MovJ.srv"
    "srv/MovL.srv"
    "srv/ResetRobot.srv"
    "srv/SpeedFactor.srv"
    "srv/SpeedJ.srv"
    "srv/SpeedL.srv"
    "srv/ToolDOExecute.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
