cmake_minimum_required(VERSION 3.8)
project(drone_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create executables
add_executable(simple_drone_monitor src/simple_drone_monitor.cpp)
add_executable(test_flight src/test_flight.cpp)

# Target dependencies
ament_target_dependencies(simple_drone_monitor
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_target_dependencies(test_flight
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  simple_drone_monitor
  test_flight
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()