cmake_minimum_required(VERSION 3.8)
project(diff_wheel_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find lgpio library
find_path(LGPIO_INCLUDE_DIR lgpio.h
  PATHS 
    /usr/include 
    /usr/local/include 
    /usr/include/lgpio 
    ${CMAKE_SOURCE_DIR}/external/lgpio
  PATH_SUFFIXES lgpio
)

find_library(LGPIO_LIBRARY
  NAMES lgpio
  PATHS 
    /usr/lib 
    /usr/local/lib 
    /usr/lib/x86_64-linux-gnu 
    ${CMAKE_SOURCE_DIR}/external/lgpio
)

if(NOT LGPIO_INCLUDE_DIR OR NOT LGPIO_LIBRARY)
  message(FATAL_ERROR "Could not find lgpio library. Please install lgpio or specify its location.")
else()
  message(STATUS "Found lgpio: ${LGPIO_LIBRARY}")
  message(STATUS "lgpio include dir: ${LGPIO_INCLUDE_DIR}")
endif()

include_directories(${LGPIO_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add diff_motion node
add_executable(diff_motion src/diff_motion.cpp)
ament_target_dependencies(diff_motion rclcpp example_interfaces geometry_msgs)
target_link_libraries(diff_motion ${LGPIO_LIBRARY})

# Add MotionTest node
add_executable(motion_test src/test_node_cpp.cpp)
ament_target_dependencies(motion_test rclcpp)
target_link_libraries(motion_test ${LGPIO_LIBRARY})

# Add gpio_free_node
add_executable(gpio_free_node src/gpio_free.cpp)
ament_target_dependencies(gpio_free_node rclcpp)
target_link_libraries(gpio_free_node ${LGPIO_LIBRARY})


install(TARGETS
  diff_motion
  motion_test
  gpio_free_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()