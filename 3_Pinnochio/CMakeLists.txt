cmake_minimum_required(VERSION 3.8)
project(ros2_pinocchio_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinocchio REQUIRED)

# Create the executable
add_executable(robot_model_node src/robot_model_node.cpp)

# Specify dependencies
target_include_directories(robot_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link against required libraries
ament_target_dependencies(robot_model_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  pinocchio
)

# Install the executable
install(TARGETS robot_model_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 