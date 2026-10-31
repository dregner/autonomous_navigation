cmake_minimum_required(VERSION 3.22.1)
project(autonomous_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(autonomous_navigation_msgs REQUIRED)

include_directories(include)

add_executable(path_generator src/path_generator.cpp)
add_executable(trajectory_controller src/trajectory_controller.cpp)

ament_target_dependencies(path_generator
rclcpp
sensor_msgs
mavros_msgs
geometry_msgs
autonomous_navigation_msgs
nav_msgs
std_srvs
tf2_geometry_msgs
Eigen3
)

ament_target_dependencies(trajectory_controller
rclcpp
sensor_msgs
mavros_msgs
geometry_msgs
autonomous_navigation_msgs
nav_msgs
std_srvs
tf2_geometry_msgs
Eigen3
)

install(TARGETS path_generator trajectory_controller
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
DESTINATION include)

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ROS-free unit tests of the PID / PWM mapping helpers (include/pid.hpp)
  add_executable(test_pid test/test_pid.cpp)
  target_include_directories(test_pid PRIVATE include)
  add_test(NAME test_pid COMMAND test_pid)
endif()

ament_package()
