cmake_minimum_required(VERSION 3.8)
project(balance_robot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(balance_interface REQUIRED)

# 2. Header include directories
include_directories(
  include
  ${serial_INCLUDE_DIRS} # Ensure the serial-library headers are included
)

# 3. Declare executables
add_executable(balance_robot_node
  src/balance_robot.cpp
  src/quaternion_solution.cpp
)
add_executable(balance_keyboard_control
  src/balance_keyboard_control.cpp
)

# 4. Link libraries
ament_target_dependencies(balance_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  balance_interface
  tf2_geometry_msgs
  # serial
)
ament_target_dependencies(balance_keyboard_control
  rclcpp
  geometry_msgs
)

# Explicitly link the serial library
target_link_libraries(balance_robot_node ${serial_LIBRARIES})

# 5. Installation rules
# Install executables
install(TARGETS 
  balance_robot_node
  balance_keyboard_control
  DESTINATION lib/${PROJECT_NAME})

# Install the launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
