cmake_minimum_required(VERSION 3.8)
project(mg400_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(msg_files
    "msg/RobotMode.msg"
    "msg/EndPose.msg"
    "msg/EndPoseStamped.msg")

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
    "srv/Tool.srv"
    "srv/ToolDOExecute.srv"
)

set(action_files "action/MovJ.action")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
