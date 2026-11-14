cmake_minimum_required(VERSION 3.8)
project(balance_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable C++17 support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(balance_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Qt5 configuration
# Automatically generate moc, uic, rcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Declare source files
set(QT_SOURCES
  src/main.cpp
  src/main_window.cpp
  src/main_window_teleop.cpp
  src/main_window_ssh.cpp
  src/main_window_pid.cpp
  src/qnode.cpp
)

set(QT_HEADERS
  include/balance_gui/main_window.hpp
  include/balance_gui/qnode.hpp
)

set(QT_UIS
  ui/main_window.ui
)

# Generate the executable
add_executable(${PROJECT_NAME} 
  ${QT_SOURCES} 
  ${QT_HEADERS} 
  ${QT_UIS}
)

# Link ROS 2 and other package dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
  balance_interface
  std_srvs
  rclcpp_action
  nav2_msgs
)

# Link the Qt5 and OpenCV libraries
target_link_libraries(${PROJECT_NAME} 
  Qt5::Core 
  Qt5::Gui 
  Qt5::Widgets
  ${OpenCV_LIBRARIES}
)

# Installation rules
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config images
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL 
)

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
