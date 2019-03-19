cmake_minimum_required(VERSION 2.8.3)
project(mpc_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation)
find_package(ACADO REQUIRED)

include_directories(
 ${PROJECT_SOURCE_DIR}/src/export_MPC
 ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/include
 include
 ${catkin_INCLUDE_DIRS}
 ${ACADO_INCLUDE_DIRS}
)

add_message_files(
  FILES
  InputDataMsg.msg
  OutputDataMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package (CATKIN_DEPENDS message_runtime)
add_definitions (-DLINUX -D__CODE_GENERATION__)
add_library (${PROJECT_NAME}_qpoases3 STATIC
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/Bounds.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/Constraints.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/Indexlist.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/Matrices.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/MessageHandling.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/Options.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/Flipper.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/Utils.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/QProblem.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/QProblemB.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/qpoases3/src/OQPinterface.c)

add_library (${PROJECT_NAME}_acado_solver STATIC
    ${PROJECT_SOURCE_DIR}/src/export_MPC/acado_solver.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/acado_qpoases3_interface.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/acado_integrator.c
    ${PROJECT_SOURCE_DIR}/src/export_MPC/acado_auxiliary_functions.c)

target_link_libraries (${PROJECT_NAME}_acado_solver ${PROJECT_NAME}_qpoases3 m)

add_executable (mpc_wrapper_test src/mpc_wrapper.cpp src/main_mpc_wrapper.cpp src/mpc.cpp)
target_link_libraries (mpc_wrapper_test ${catkin_LIBRARIES} qpoases3 acado_solver rt ${ACADO_SHARED_LIBRARIES})

add_executable (${PROJECT_NAME}_simple_publisher src/simple_publisher.cpp)
target_link_libraries (${PROJECT_NAME}_simple_publisher ${catkin_LIBRARIES})

add_dependencies(mpc_wrapper_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_simple_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

