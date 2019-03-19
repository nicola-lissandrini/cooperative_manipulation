#include "trajectory_filter.h"
#include <geometry_msgs/PoseStamped.h>

using namespace ros;
using namespace std;
using namespace XmlRpc;

MPCTrajectoryFilter::MPCTrajectoryFilter ()
{
	initParams ();
	initROS ();
}

int MPCTrajectoryFilter::spin()
{
	while (ok ()) {
		sendCommand ();

		spinOnce ();
		rate->sleep ();
	}

	return 0;
}

void MPCTrajectoryFilter::trajCallback(const aerial_mpc::Trajectory &newTraj) {
	traj = newTraj;
}

void MPCTrajectoryFilter::velCallback(const geometry_msgs::TwistStamped &newVel) {
	vel = newVel.twist;
}

#define FINITE_DIFF(traj, i, k) ((traj.trajectory[i+1].position.k - traj.trajectory[i-1].position.k)/(2*sampleTime))

nav_msgs::Odometry MPCTrajectoryFilter::filtered ()
{
	nav_msgs::Odometry poseTwist;
	geometry_msgs::Pose targetPose;

	targetPose = traj.trajectory[1];

	poseTwist.pose.pose = targetPose;

	// 1st order difference
	poseTwist.twist.twist.linear.x = FINITE_DIFF (traj, 1, x);
	poseTwist.twist.twist.linear.y = FINITE_DIFF (traj, 1, y);
	poseTwist.twist.twist.linear.z = FINITE_DIFF (traj, 1, z);
	poseTwist.twist.twist.angular.z = vel.angular.z;

	return poseTwist;
}

void MPCTrajectoryFilter::sendCommand ()
{
	nav_msgs::Odometry poseTwistCommand;

	if (traj.trajectory.size () == 0)
		// Empty message. Skip
		return;
	// Apply velocity estimation filter
	poseTwistCommand = filtered ();
	commandPub.publish (poseTwistCommand);
}

void MPCTrajectoryFilter::initParams ()
{
	try {
		nh.getParam ("trajectory_filter", params);
		sampleTime = (double) params["mpc_sample_time"];
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s", e.getMessage ().c_str ());
	}
}

void MPCTrajectoryFilter::initROS ()
{
	rate = new Rate (double (params["rate"]));

	trajSub = nh.subscribe ((string) params["ref_trajectory_topic"], 1, &MPCTrajectoryFilter::trajCallback, this);
	velSub = nh.subscribe ((string) params["ref_vel_topic"], 1, &MPCTrajectoryFilter::velCallback, this);
	commandPub = nh.advertise<nav_msgs::Odometry> ((string) params["pose_command_topic"], 1);
}

int main (int argc, char *argv[])
{
	init (argc, argv, "trajectory_filter");

	MPCTrajectoryFilter trajFilter;

	return trajFilter.spin ();
}
