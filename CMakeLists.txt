cmake_minimum_required(VERSION 3.0.2)
project(baxter_simple_sim)

find_package(Eigen3 REQUIRED)
find_package(catkin
  REQUIRED
  COMPONENTS
  roscpp
  kdl_parser
  sensor_msgs
  baxter_core_msgs
)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_executable(ik_service
    src/ik_service.cpp
    include/baxter_simple_sim/baxter_cppkdl.h src/baxter_cppkdl.cpp)
    
target_link_libraries(ik_service ${catkin_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  baxter_core_msgs
  DEPENDS
  kdl_parser
  INCLUDE_DIRS include
)

install(TARGETS ik_service
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


catkin_install_python(PROGRAMS ros2_bridge.py simple_sim.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
