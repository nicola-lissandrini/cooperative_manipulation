#ifndef TRAJECTORY_FILTER_H
#define TRAJECTORY_FILTER_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <aerial_mpc/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

class MPCTrajectoryFilter
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber trajSub;
	ros::Subscriber velSub;
	ros::Publisher commandPub;
  ros::Publisher refPub;
	aerial_mpc::Trajectory traj;
	geometry_msgs::Twist vel;

	double sampleTime;

	void initParams ();
	void initROS ();
	void sendCommand ();
	nav_msgs::Odometry filtered ();

public:
	MPCTrajectoryFilter ();

	int spin ();
	void trajCallback (const aerial_mpc::Trajectory &newTraj);
	void velCallback (const geometry_msgs::TwistStamped &newVel);
};

#endif // TRAJECTORY_FILTER_H
