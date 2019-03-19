cmake_minimum_required(VERSION 2.8.3)
project(ground_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
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
  Obstacle.msg
  ObstaclesArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package (CATKIN_DEPENDS message_runtime)
add_definitions (-DLINUX -D__CODE_GENERATION__)
add_library (qpoases3 STATIC
    src/export_MPC/qpoases3/src/Bounds.c
    src/export_MPC/qpoases3/src/Constraints.c
    src/export_MPC/qpoases3/src/Indexlist.c
    src/export_MPC/qpoases3/src/Matrices.c
    src/export_MPC/qpoases3/src/MessageHandling.c
    src/export_MPC/qpoases3/src/Options.c
    src/export_MPC/qpoases3/src/Flipper.c
    src/export_MPC/qpoases3/src/Utils.c
    src/export_MPC/qpoases3/src/QProblem.c
    src/export_MPC/qpoases3/src/QProblemB.c
    src/export_MPC/qpoases3/src/OQPinterface.c)

add_library (acado_solver STATIC
    src/export_MPC/acado_solver.c
    src/export_MPC/acado_qpoases3_interface.c
    src/export_MPC/acado_integrator.c
    src/export_MPC/acado_auxiliary_functions.c)

target_link_libraries (acado_solver qpoases3 m)

add_executable (ground_mpc src/mpc_wrapper.cpp src/main_mpc_wrapper.cpp src/mpc.cpp)
target_link_libraries (ground_mpc ${catkin_LIBRARIES} qpoases3 acado_solver rt ${ACADO_SHARED_LIBRARIES})

add_executable (ground_input_interface src/input_interface.cpp src/main_input_interface.cpp)
target_link_libraries (ground_input_interface ${catkin_LIBRARIES} rt)

add_executable (ground_output_interface src/output_interface.cpp src/main_output_interface.cpp)
target_link_libraries (ground_output_interface ${catkin_LIBRARIES} rt)

add_executable (joint_measurer src/joint_measurer.cpp)
target_link_libraries (joint_measurer ${catkin_LIBRARIES})

add_executable (traj src/basic_reference_gen.cpp)
target_link_libraries (traj ${catkin_LIBRARIES})

add_executable (obstacle_detector src/main_obstacle_detector.cpp src/obstacle_detector.cpp)
target_link_libraries (obstacle_detector ${catkin_LIBRARIES})

add_dependencies(ground_mpc ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ground_input_interface ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(ground_output_interface ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(obstacle_detector ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
