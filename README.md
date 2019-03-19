# cooperative_manipulation
Source code for cooperative manipulation task for aerial-ground robots with MPC, master degree project at KTH - Royal Institute of Technology.

## Installing
Dependencies:
* `rotors_simulator` from ETHZ ASL https://github.com/ethz-asl/rotors_simulator
* `uav_control` from KTH SML https://github.com/KTH-SML/uav_control (need permissions), `dev` branch
For the simulations, Gazebo 9 is recommended, some packages may not work on previous versions. If you don't need simulations, try excluding `easy_gripper` and `prediction_printer` from build.