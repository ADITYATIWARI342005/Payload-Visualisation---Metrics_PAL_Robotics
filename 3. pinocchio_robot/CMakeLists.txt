cmake_minimum_required(VERSION 3.8)
project(pinocchio_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)

add_executable(pinocchio_node src/pinocchio_node.cpp src/kinematics.cpp src/jacobian.cpp src/collision_check.cpp)
target_include_directories(pinocchio_node PRIVATE include)
target_link_libraries(pinocchio_node pinocchio::pinocchio)

install(TARGETS pinocchio_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
