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

# Find gpiod library
find_path(GPIOD_INCLUDE_DIR gpiod.hpp
    PATHS /usr/include /usr/local/include)
find_library(GPIOD_LIBRARY gpiod
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
find_library(GPIODCXX_LIBRARY gpiodcxx
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

if(NOT GPIOD_INCLUDE_DIR OR NOT GPIOD_LIBRARY OR NOT GPIODCXX_LIBRARY)
    message(FATAL_ERROR "Could not find gpiod library. Please install libgpiod-dev.")
endif()

find_library(PIGPIOD_LIBRARY pigpiod_if2)
include_directories(${GPIOD_INCLUDE_DIR} /usr/local/include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(diff_motion src/diff_motion.cpp)
ament_target_dependencies(diff_motion rclcpp example_interfaces geometry_msgs)
target_link_libraries(diff_motion ${PIGPIOD_LIBRARY})

# Add MotionTest node
add_executable(motion_test src/test_node_cpp.cpp)
ament_target_dependencies(motion_test rclcpp)
target_link_libraries(motion_test ${GPIOD_LIBRARY} ${GPIODCXX_LIBRARY})

install(TARGETS
  diff_motion
  motion_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()