#ifndef FORWARD_KINEMATICS_ESTIMATOR_H
#define FORWARD_KINEMATICS_ESTIMATOR_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define JOINTS_NO 2

class ForwardKinematicsEstimator
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber baseSub;
	ros::Subscriber jointsSub;
	ros::Publisher eefPub;

	double l1a, l1b, l2, lbx, lby, lbz;

	tf::Pose currBase;
	std::vector<double> currJoints;

	void initParams ();
	void initRos ();
	void computeAndSend ();
	tf::Vector3 forwardPosition (double j1, double j2,
								 double xv, double yv, double zv, double psi);
	tf::Matrix3x3 forwardOrientation (double j1, double j2,
									   double xv, double yv, double zv, double psi);

	void baseCallback (const nav_msgs::Odometry &baseOdom);
	void jointsCallback (const std_msgs::Float32MultiArray &joints);

public:
	ForwardKinematicsEstimator();

	int spin ();
};

#endif // FORWARD_KINEMATICS_ESTIMATOR_H
