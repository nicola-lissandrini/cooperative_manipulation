# cooperative_manipulation
Source code for cooperative manipulation task for aerial-ground robots with MPC, master degree project at KTH - Royal Institute of Technology.

## Installing
Dependencies:
* `rotors_simulator` from ETHZ ASL https://github.com/ethz-asl/rotors_simulator
* `uav_control` from KTH SML https://github.com/KTH-SML/uav_control (need permissions), `dev` branch
For the simulations, Gazebo 9 is recommended, some packages may not work on previous versions. If you don't need simulations, try excluding `easy_gripper` and `prediction_printer` from build.

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

After the experiment is done, to land the drone:
`rostopic pub -1 /world/command/commander std_msgs/Int8 "data: 1"`
This will finalize the experiment.

### Input and outputs connection in SML
Inputs:
Requirements from MOCAP:

* `/nexus1/odom`: pose of nexus base, the position should be aligned with the center and x axis should be facing the front of the base.
* `/nexus1_end_effector`: pose of ground end effector. No particular requirements on the frame displacement, the configuration will affect the "nearby" position relative to the object and nothing else.
* `popeye`: pose of the base of the aerial robot.
The end effector of the aerial robot is estimated via forward kinematics.

