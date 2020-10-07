cmake_minimum_required(VERSION 3.0.2)
project(baxter_simple_sim)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS bridge DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS bridge.py simple_sim.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
