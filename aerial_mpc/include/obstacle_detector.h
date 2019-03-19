#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <XmlRpc.h>

class ObstacleDetector
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Publisher obstPub;

	void initParams ();
	void initROS ();
	void sendObstacleData ();

public:
	ObstacleDetector ();

	int spin ();
};

#endif // OBSTACLE_DETECTOR_H
