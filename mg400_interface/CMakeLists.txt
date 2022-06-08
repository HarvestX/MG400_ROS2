cmake_minimum_required(VERSION 3.8)
project(mg400_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# ===================================================================
ament_auto_add_library(
  ${PROJECT_NAME}
    STATIC
      ./src/error_msg_generator.cpp)
target_link_libraries(${PROJECT_NAME} stdc++fs jsoncpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
# ===================================================================

# ===================================================================
ament_auto_add_executable(
  show_error
    ./examples/show_error.cpp
)
target_link_libraries(show_error ${PROJECT_NAME})
# ===================================================================


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  set(TARGET test_error_msg_generator)
  ament_add_gtest(${TARGET} test/src/${TARGET}.cpp)
  target_link_libraries(${TARGET} ${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    resources
)
