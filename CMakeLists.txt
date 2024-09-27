cmake_minimum_required(VERSION 3.8)
project(diff_wheel_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

find_library(PIGPIOD_LIBRARY pigpiod_if2)
include_directories(/usr/local/include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(diff_motion src/diff_motion.cpp)
ament_target_dependencies(diff_motion rclcpp example_interfaces geometry_msgs)

target_link_libraries(diff_motion ${PIGPIOD_LIBRARY})

install(TARGETS
  diff_motion
  DESTINATION lib/${PROJECT_NAME})

ament_package()