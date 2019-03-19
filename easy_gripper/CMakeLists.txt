cmake_minimum_required(VERSION 2.8.3)
project(easy_gripper)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros)

add_message_files(
  FILES
  GripCommand.msg
  WrenchArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nexus_base
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS roscpp gazebo_ros
)

include_directories(
 ./include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(easy_gripper SHARED src/easy_gripper.cpp)
target_link_libraries(easy_gripper ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(easy_gripper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
