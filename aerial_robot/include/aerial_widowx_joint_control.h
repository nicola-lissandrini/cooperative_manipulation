#ifndef WIDOWX_JOINT_CONTROL_H
#define WIDOWX_JOINT_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>

#define WIDOWX_JOINT_NO 2
#define WIDOWX_JOINT_SUB "/neo11/command/joints"
#define WIDOWX_JOINT_PUB_PRE "/neo11/command/joint"
#define WIDOWX_JOINT_PUB_POST ""
#define WIDOWX_INTERFACE_RATE 100

typedef std_msgs::Float32MultiArray ArrayControlMsg;
typedef std_msgs::Float64 SingleControlMsg;

#define QUA ROS_INFO("qua");
#define QUAN(n) ROS_INFO("qua %d",n);

class WidowXJointControl
{
	ros::NodeHandle nh;
	ros::Subscriber arrayControlSub;
	ros::Rate rate;
	std::vector<ros::Publisher> jointsPub;
	std::vector<float> jointControlValues;

	void initJointsPublishers ();
	void publishAll ();
	std::string getTopicName (int i);

public:
	WidowXJointControl ();

	int spin ();
	inline void setValues (std::vector<float> values);

	static void arrayControlCallback (const ArrayControlMsg &newControl);
};


#endif // WIDOWX_JOINT_CONTROL_H
