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

set(msg_files "msg/RobotStatus.msg" "msg/ToolVectorActual.msg")

set(srv_files
    "srv/AccJ.srv"
    "srv/AccL.srv"
    "srv/AOExecute.srv"
    "srv/AO.srv"
    "srv/Arch.srv"
    "srv/Arc.srv"
    "srv/Circle.srv"
    "srv/ClearError.srv"
    "srv/GetErrorID.srv"
    "srv/ContinueScript.srv"
    "srv/CP.srv"
    "srv/DisableRobot.srv"
    "srv/DOExecute.srv"
    "srv/DO.srv"
    "srv/EmergencyStop.srv"
    "srv/EnableRobot.srv"
    "srv/GetHoldRegs.srv"
    "srv/JointMovJ.srv"
    "srv/Jump.srv"
    "srv/LimZ.srv"
    "srv/MoveJog.srv"
    "srv/MovJ.srv"
    "srv/MovL.srv"
    "srv/PauseScript.srv"
    "srv/Payload.srv"
    "srv/PowerOn.srv"
    "srv/RelMovJ.srv"
    "srv/RelMovL.srv"
    "srv/ResetRobot.srv"
    "srv/RobotMode.srv"
    "srv/RunScript.srv"
    "srv/ServoJ.srv"
    "srv/ServoP.srv"
    "srv/SetArmOrientation.srv"
    "srv/SetCollisionLevel.srv"
    "srv/SetHoldRegs.srv"
    "srv/SetObstacleAvoid.srv"
    "srv/SetSafeSkin.srv"
    "srv/SpeedFactor.srv"
    "srv/SpeedJ.srv"
    "srv/SpeedL.srv"
    "srv/StartFCTrace.srv"
    "srv/StartPath.srv"
    "srv/StartTrace.srv"
    "srv/StopScript.srv"
    "srv/Sync.srv"
    "srv/ToolDOExecute.srv"
    "srv/ToolDO.srv"
    "srv/Tool.srv"
    "srv/User.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
