cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nexus_base
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS roscpp
)

include_directories(
 ./include
 ${catkin_INCLUDE_DIRS}
)

add_executable (aerial_widowx_joint_control src/main_aerial_widowx_joint_control.cpp src/aerial_widowx_joint_control.cpp)
target_link_libraries (aerial_widowx_joint_control ${catkin_LIBRARIES})

add_executable (aerial_widowx_joint_state src/aerial_widowx_joint_state.cpp)
target_link_libraries (aerial_widowx_joint_state ${catkin_LIBRARIES})
