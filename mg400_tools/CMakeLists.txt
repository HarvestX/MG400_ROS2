cmake_minimum_required(VERSION 3.8)
project(mg400_tools)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake_python REQUIRED)
find_package(yaml-cpp REQUIRED)

# command_queue_client ============================
set(TARGET command_queue_client)
ament_auto_add_executable(${TARGET} src/${TARGET}.cpp)
target_link_libraries(${TARGET} yaml-cpp)
unset(TARGET)

# validate_kinematics_with_solutions =============
set(TARGET validate_kinematics_with_solutions)
ament_auto_add_executable(${TARGET} src/${TARGET}.cpp)
unset(TARGET)

# mg400_kinematics_cli ============================
set(TARGET mg400_kinematics_cli)
ament_auto_add_executable(${TARGET} src/${TARGET}.cpp)
unset(TARGET)

# Python modules and commands =====================
ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
    scripts/param_identify
    scripts/command_sender
    scripts/compare_positive_inverse_solution
  DESTINATION lib/${PROJECT_NAME}
)

# Configuration and examples ======================
install(
  DIRECTORY config examples
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_param_identify test/test_param_identify.py)
endif()

ament_auto_package()
