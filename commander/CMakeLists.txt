cmake_minimum_required(VERSION 2.8.3)
project(commander)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_message_files()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package (CATKIN_DEPENDS message_runtime)

add_executable (commander src/main_commander.cpp src/commander.cpp)
target_link_libraries (commander ${catkin_LIBRARIES})

add_dependencies(commander ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



