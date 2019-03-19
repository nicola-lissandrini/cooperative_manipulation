#ifndef AERIAL_WIDOWX_JOINT_STATE_H
#define AERIAL_WIDOWX_JOINT_STATE_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

#define JOINT_N 2
#define JOINT_MSG_N 2

class WidowxJointState
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	ros::Subscriber jointsSub;
	ros::Publisher jointsPub;
	XmlRpc::XmlRpcValue params;
	std::vector<float> currState;
	bool pub;

	void initROS ();
	void initParams ();
	void sendMsg ();
	std::string getJointName (int i);

public:
	WidowxJointState();

	int spin ();
	void jointMsgCallback (const sensor_msgs::JointState &jointMsg);
};

#endif // AERIAL_WIDOWX_JOINT_STATE_H
