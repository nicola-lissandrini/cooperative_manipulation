# cooperative_manipulation
Source code for cooperative manipulation task for aerial-ground robots with MPC, master degree project at KTH - Royal Institute of Technology.

## Installing
Dependencies:
* `rotors_simulator` from ETHZ ASL https://github.com/ethz-asl/rotors_simulator
* `uav_control` from KTH SML https://github.com/KTH-SML/uav_control (need permissions), `dev` branch
For the simulations, Gazebo 9 is recommended, some packages may not work on previous versions. If you don't need simulations, try excluding `easy_gripper` and `prediction_printer` from build.


Important note: the code requires the modified version of `fa_controller.py` in the package `uav_control` which is not updated yet. 

## Running the experiment

To run the main launch file:
`roslaunch commander aerial_ground_coop.launch`
with arg `arena`:
* `gazebo` (default): run the experiment in Gazebo simulation.
* `sml`: run the experiment in the SML arena.

To initialize the experiment:
`rostopic pub -1 /world/command/commander std_msgs/Int8 "data: 0"`
this will cause the aerial robot to take off and then both will reach a predefined configuration (set in cooperative_arena.yaml, see below) nearby the object.

To start the cooperative transportation, give a setpoint for the object:
`rostopic pub -1 /world/command/object_pose geometry_msgs/Pose "<desired_pose>"`
The desired pose refers to the object center and is in the world frame.
The default and recommended value for the object orientation quaternion is the same as the starting one, `[0, 0, 0, 1]` (x, y, z, w) by default.

After the experiment is done, to land the drone:
`rostopic pub -1 /world/command/commander std_msgs/Int8 "data: 1"`
This command can be sent any time to abort the experiment and land the aerial robot safely.
This will finalize the experiment.

### Input and outputs connection in KTH-SML

Input requirements from Qualisys:

* `/nexus1/odom` (`nav_msgs/Odometry`): pose of nexus base, the position should be aligned with the center and x axis should be facing the front of the base.
* `/nexus1_end_effector/odom` (`nav_msgs/Odometry`): pose of ground end effector. No particular requirements on the frame displacement, the configuration will affect the "nearby" position relative to the object and nothing else.
* `/popeye/odom`  (`nav_msgs/Odometry`): pose of the base of the aerial robot.
The end effector of the aerial robot is estimated via forward kinematics.
* `/human/odom` (`nav_msgs/Odometry`): pose of the object. This is for the initial approach step, it is not required to physical have a mocap object since it is not used during the task. It can be substituted by manually publishing a constant initial pose in which the object is located. This can be done  by running `publish_human.sh`.

 Input requirements from robots:
 * `/arm/joints_poses` (`std_msgs/Float32MultiArray`):  joints values for aerial robot
 * `/widowx_arm1/joints_poses` (`std_msgs/Float32MultiArray`):  joints values for ground robot

 Output requirements from robots:
 * `/command/roll_pitch_yawrate_thrust` (`mav_msgs/RollPitchYawrateThrust`) aerial rpy controller.
* `/arm/velocity` (`std_msgs/Float32MultiArray`): aerial arm velocity inputs.
* `/cmdvel1` (`geometry_msgs/Twist`): ground base controller.
* `/widowx_arm1/velocity_setpoint`(`std_msgs/Float32MultiArray`): ground arm velocity inputs.

### Setting the obstacles

The obstacles are set manually in the configuration file `ground_mpc/config.sml/obstacles.yaml`.
Current version supports 2 spherical obstacles, defined in the array `obstacle_detector/obstacles`. The format is inteded to define ellipses with the field `axes` of each element.
However, only the first element is currently considered and stands for the radius value. 2nd and 3rd values are ignored. Units in meters, world frame. To use only one obstacle just set an unreachable position for the second.

### Topics for monitoring

* `/aerial_tracking_error`: norm of position tracking error for aerial robot. Checkout this topic to make sure the UAV reached the setpoint after approaching step and before entering the object setpoint.
* `/ground/mpc/control`, `/neo11/mpc/control`: predicted trajectory and control input for ground and aerial robots MPCs. Useful fields: `objVal` current value of objective cost function. `cpuTime` should be always below the sample time 0.1s. `solverStatus` different from zero if errors occured solving the QP. Check console info in the main launchfile output for details.
* `/ground/mpc/input`, `/neo11/mpc/input`: input data for ground and aerial robots MPCs. Contains info regarding current state, running reference and terminal reference, onlineData. 
Everything in the state should be different from zero. If there are zeros it means that some input data is missing.
* `/popeye_end_effector/odom`: estimated position/velocity for aerial robot end effector via direct kinematics. Remember this is the controlled quantity by MPC.
* Input topics from Qualisys and robots.

### Tests for individual control

The MPC receives poses in the topics:
* `/ground/cmd_pose/object_pose` to control the ground end effector.
* `/neo1/command/object_pose` to control the aerial end effector.
Note that `object_pose` here actually means `end_effector` and has nothing to do with the object. This is a legacy from a first version in which the control frame was that of the object which was considered at a fixed pose relative to the end effector.

The orientation must be choosen carefully, as the end effector frame is rotated w.r.t. the world frame. The best practice is to first position the end effector with the desired orientation in any position, read the quaternion from `/nexus1_end_effector/odom` or `/popeye_end_effector/odom`, and then set the latter as input reference in the ros message.
This is not required in the final experiment as the relative transformations are automatically processed by the experiment commander node.