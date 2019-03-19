#ifndef POPEYE_INTERFACE_H
#define POPEYE_INTERFACE_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <geometry_msgs/TwistStamped.h>

class PopeyeInterface
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber velSub;
	ros::Publisher velPub;

	void initParams ();
	void initROS ();

public:
	PopeyeInterface();
};

#endif // POPEYE_INTERFACE_H
