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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Jacobian.srv" DEPENDENCIES builtin_interfaces)

if("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

add_executable(simulator
    src/sim_node.cpp
    src/sim.cpp
    src/arm_io.cpp)
target_include_directories(simulator PRIVATE include ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(simulator
    rclcpp sensor_msgs baxter_core_msgs urdf kdl_parser robot_state_publisher ament_index_cpp)
if(${LEGACY_IDL})
    rosidl_target_interfaces(simulator ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(simulator "${cpp_typesupport_target}")
endif()

install(TARGETS simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
 
ament_package()
