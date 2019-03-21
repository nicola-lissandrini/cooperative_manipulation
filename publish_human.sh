#!/bin/bash

rostopic pub -r 100 /human/odom nav_msgs/Odometry "pose:
  pose:
    position:
      x: 0.148474791348
      y: -1.41467897346
      z: 0.35
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"
