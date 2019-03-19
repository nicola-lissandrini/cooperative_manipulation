#!/bin/bash

master_ip=$(cat ~/current_user)

export ROS_MASTER_URI=http://$master_ip:11311
export ROS_IP=12.0.4.1
export ROS_HOSTNAME=12.0.4.1










