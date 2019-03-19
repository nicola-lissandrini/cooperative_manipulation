#include "obstacle_detector.h"
#include "ground_mpc/ObstaclesArray.h"

using namespace ros;
using namespace std;
using namespace XmlRpc;

ObstacleDetector::ObstacleDetector()
{
	initParams ();
	initROS ();
}

void ObstacleDetector::initParams ()
{
	try {
		nh.getParam ("obstacle_detector", params);
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s", e.getMessage ().c_str ());
	}
}

void ObstacleDetector::initROS ()
{
	rate = new Rate (double (params["rate"]));

	obstPub = nh.advertise<ground_mpc::ObstaclesArray> ((string) params["obstacle_topic"], 1) ;
}

void ObstacleDetector::sendObstacleData ()
{
	ground_mpc::ObstaclesArray obstaclesMsg;
	XmlRpcValue obstacles = params["obstacles"];

	for (int i = 0; i < obstacles.size (); i++) {
		ground_mpc::Obstacle currMsg;
		currMsg.center.x = (double) obstacles[i]["center"][0];
		currMsg.center.y = (double) obstacles[i]["center"][1];
		currMsg.center.z = (double) obstacles[i]["center"][2];
		currMsg.axes.x = (double) obstacles[i]["axes"][0];
		currMsg.axes.y = (double) obstacles[i]["axes"][1];
		currMsg.axes.z = (double) obstacles[i]["axes"][2];

		obstaclesMsg.obstacles.push_back (currMsg);
	}

	obstPub.publish (obstaclesMsg);
}

int ObstacleDetector::spin ()
{
	while (ok ()) {
		sendObstacleData ();

		spinOnce ();
		rate->sleep ();
	}

	return 0;
}

