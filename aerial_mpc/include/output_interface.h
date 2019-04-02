#ifndef OUTPUT_INTERFACE_H
#define OUTPUT_INTERFACE_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include "mpc_wrapper/OutputDataMsg.h"
#include "aerial_mpc/Trajectory.h"
#include "acado_common.h"

#define JOINTS_N 2
#define WINDOW_N ACADO_N

class OutputInterface
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber controlSub;
	ros::Publisher basePub;
	ros::Publisher jointsPub;
  ros::Publisher trajPub;
	aerial_mpc::Trajectory prediction;

	double satVal;
	bool started;

	struct ControlMsgs {
		geometry_msgs::TwistStamped baseVel;
		std_msgs::Float32MultiArray jointsVel;
	} controlMsg;

	void initParams ();
	void initROS ();
	void sendControl ();
	double sat(double val);
	void setPrediction (const aerial_mpc::Trajectory &newPrediction);

public:
	OutputInterface ();

	int spin ();
	void setControlMsg (const geometry_msgs::TwistStamped &baseVel, const std_msgs::Float32MultiArray &jointsVel);
	aerial_mpc::Trajectory buildPredictionMsg (const mpc_wrapper::OutputDataMsg &controlMsg);
	inline void start () {
		started = true;
	}
	inline bool isStarted () {
		return started;
	}

	static void controlCallback (const mpc_wrapper::OutputDataMsg &mpcOutputMsg);
	XmlRpc::XmlRpcValue getParams() const {
		return params;
	}



};

extern OutputInterface *outputInterface;

#endif // OUTPUT_INTERFACE_H
