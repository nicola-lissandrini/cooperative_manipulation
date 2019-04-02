cmake_minimum_required(VERSION 2.8.3)
project(prediction_printer)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    message_generation
    aerial_mpc)

generate_messages (
    DEPENDENCIES
    std_msgs
    geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES nexus_base
  CATKIN_DEPENDS aerial_mpc
  DEPENDS roscpp gazebo_ros message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(prediction_printer SHARED src/prediction_printer.cpp)
target_link_libraries(prediction_printer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(prediction_printer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(prediction_printer mpc_wrapper_generate_messages_cpp)
