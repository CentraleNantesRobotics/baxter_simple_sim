cmake_minimum_required(VERSION 3.5)
project(baxter_simple_sim)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 pkgs
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(simulator
    src/sim_node.cpp
    src/sim.cpp
    src/arm_io.cpp)
target_include_directories(simulator PRIVATE include ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(simulator
    rclcpp sensor_msgs baxter_core_msgs urdf kdl_parser robot_state_publisher)

install(TARGETS simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
 
ament_package()
