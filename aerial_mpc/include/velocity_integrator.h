#ifndef VELOCITY_INTEGRATOR_H
#define VELOCITY_INTEGRATOR_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include "pid.h"

#define CONTROL_NO 4

class VelocityIntegrator
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber velSub;
	ros::Subscriber stateSub;
	ros::Publisher velPub;

	Pid *pid;

	int seq;
	bool started;
	double sampleTime;
	nav_msgs::Odometry currentOdometry;
	geometry_msgs::Twist currentVelMsg;

	void initParams ();
	void initROS ();
	void sendInput ();
	Eigen::VectorXd getStateVelocity ();
	Eigen::VectorXd getRef ();
	geometry_msgs::Twist controlToMsg (const Eigen::VectorXd &control);

public:
	VelocityIntegrator();

	int spin ();
	inline bool isStarted () {
		return started;
	}
	inline void start () {
		started = true;
	}
	void setVelocity (const geometry_msgs::Twist &velMsg);
	void setOdometry (const nav_msgs::Odometry &poseMsg);

	static void velocityCallback (const geometry_msgs::Twist &velMsg);
	static void stateCallback (const nav_msgs::Odometry &odometryMsg);
};

extern VelocityIntegrator *velocityIntegrator;

#endif // VELOCITY_INTEGRATOR_H
