cmake_minimum_required(VERSION 2.8.3)
project(aerial_mpc)

add_compile_options(-std=c++11)

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
  Obstacle.msg
  ObstaclesArray.msg
  Trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package (CATKIN_DEPENDS message_runtime)
add_definitions (-DLINUX -D__CODE_GENERATION__)
add_library (${PROJECT_NAME}_qpoases3 STATIC
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

add_library (${PROJECT_NAME}_acado_solver STATIC
    src/export_MPC/acado_solver.c
    src/export_MPC/acado_qpoases3_interface.c
    src/export_MPC/acado_integrator.c
    src/export_MPC/acado_auxiliary_functions.c)

target_link_libraries (${PROJECT_NAME}_acado_solver ${PROJECT_NAME}_qpoases3 m)

add_executable (aerial_mpc src/mpc_wrapper src/main_mpc_wrapper.cpp src/mpc.cpp)
target_link_libraries (aerial_mpc ${catkin_LIBRARIES} ${PROJECT_NAME}_qpoases3 ${PROJECT_NAME}_acado_solver rt ${ACADO_SHARED_LIBRARIES})

add_executable (aerial_input_interface src/main_input_interface.cpp src/input_interface.cpp)
target_link_libraries (aerial_input_interface ${catkin_LIBRARIES})

add_executable (aerial_output_interface src/main_output_interface.cpp src/output_interface.cpp)
target_link_libraries (aerial_output_interface ${catkin_LIBRARIES})

add_executable (aerial_velocity_integrator src/main_velocity_integrator.cpp src/velocity_integrator.cpp src/pid.cpp)
target_link_libraries (aerial_velocity_integrator ${catkin_LIBRARIES})

add_executable (traj_pub src/traj_input.cpp)
target_link_libraries (traj_pub ${catkin_LIBRARIES})

add_executable (aerial_forward_kinematics_estimator src/forward_kinematics_estimator.cpp)
target_link_libraries (aerial_forward_kinematics_estimator  ${catkin_LIBRARIES})

add_executable (aerial_trajectory_filter src/trajectory_filter.cpp)
target_link_libraries (aerial_trajectory_filter ${catkin_LIBRARIES})

add_dependencies(aerial_input_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aerial_output_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aerial_mpc  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aerial_forward_kinematics_estimator  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aerial_trajectory_filter  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

