cmake_minimum_required(VERSION 3.8)
project(mg400_tools)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)

set(MG400_COMMON_SOURCE_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../mg400_common/include)
set(MG400_MSGS_GENERATED_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../install/mg400_msgs/include/mg400_msgs)
set(MG400_MSGS_LOCAL_TYPESUPPORT_CPP_LIBRARY
  ${CMAKE_CURRENT_SOURCE_DIR}/../install/mg400_msgs/lib/libmg400_msgs__rosidl_typesupport_cpp.so)

# command_queue_client ============================
set(TARGET command_queue_client)
ament_auto_add_executable(${TARGET} src/${TARGET}.cpp)
target_link_libraries(${TARGET} yaml-cpp)
unset(TARGET)

# validate_kinematics_with_solutions =============
set(TARGET validate_kinematics_with_solutions)
ament_auto_add_executable(${TARGET} src/${TARGET}.cpp)
ament_target_dependencies(${TARGET} mg400_common mg400_msgs)
target_include_directories(
  ${TARGET} PRIVATE
  ${MG400_COMMON_SOURCE_INCLUDE_DIR}
  ${MG400_MSGS_GENERATED_INCLUDE_DIR})
if(TARGET mg400_msgs::mg400_msgs__rosidl_typesupport_cpp)
  target_link_libraries(${TARGET} mg400_msgs::mg400_msgs__rosidl_typesupport_cpp)
elseif(TARGET mg400_msgs__rosidl_typesupport_cpp)
  target_link_libraries(${TARGET} mg400_msgs__rosidl_typesupport_cpp)
endif()
if(EXISTS ${MG400_MSGS_LOCAL_TYPESUPPORT_CPP_LIBRARY})
  target_link_libraries(${TARGET} ${MG400_MSGS_LOCAL_TYPESUPPORT_CPP_LIBRARY})
endif()
unset(TARGET)

# mg400_kinematics_cli ============================
set(TARGET mg400_kinematics_cli)
ament_auto_add_executable(${TARGET} src/${TARGET}.cpp)
ament_target_dependencies(${TARGET} mg400_common)
target_include_directories(${TARGET} PRIVATE ${MG400_COMMON_SOURCE_INCLUDE_DIR})
unset(TARGET)

# install config files ============================
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
