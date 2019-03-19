#ifndef OUTPUT_INTERFACE_H
#define OUTPUT_INTERFACE_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include "aerial_mpc/Trajectory.h"

#include "mpc_wrapper/OutputDataMsg.h"

#define JOINTS_N 5

class OutputInterface
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber controlSub;
	ros::Publisher basePub;
	ros::Publisher jointsPub;
	ros::Publisher trajPub;
	double satValBase, satValJoints;

	struct ControlMsgs {
		geometry_msgs::Twist baseVel;
		std_msgs::Float32MultiArray jointsVel;
	} controlMsg;
	aerial_mpc::Trajectory prediction;

	void initParams ();
	void initROS ();
	void sendMessages ();
	float sat(float val, bool base);

	static geometry_msgs::Twist buildTwistMsg (const mpc_wrapper::OutputDataMsg &controlMsg);
	static std_msgs::Float32MultiArray buildJointsMsg (const mpc_wrapper::OutputDataMsg &controlMsg);
	static aerial_mpc::Trajectory buildPredictionMsg (const mpc_wrapper::OutputDataMsg &controlMsg);

public:
	OutputInterface ();

	int spin ();
	void setControlMsg (const geometry_msgs::Twist &baseVel, const std_msgs::Float32MultiArray &jointsVel);
	void setPrediction (const aerial_mpc::Trajectory &newPrediction);

	static void controlCallback (const mpc_wrapper::OutputDataMsg &mpcOutputMsg);


	XmlRpc::XmlRpcValue getParams() const;
};

extern OutputInterface *outputInterface;

#endif // OUTPUT_INTERFACE_H
