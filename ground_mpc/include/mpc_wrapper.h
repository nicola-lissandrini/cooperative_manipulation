#ifndef MPC_WRAPPER_H
#define MPC_WRAPPER_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <iostream>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>

#include "mpc.h"
#include "mpc_wrapper/InputDataMsg.h"
#include "mpc_wrapper/OutputDataMsg.h"

// If SYNC = 1 the input data is latched by the wrapper
#define SYNC 1

#define WARN_ILLEGAL_STATUS(s) {ROS_ERROR ("Illegal status condition occurred\n MPC current status is %s", GET_MPC_STATUS_STR (mpc.statusInfo ()));}

class MPCWrapper
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber inputSub;
	ros::Subscriber referenceSub;
	ros::Publisher controlPub;

	// MPC object
	MPC mpc;
	// Latch input data - only in synchronous mode
	MPC::InputData currentInputData;
	MPC::OutputData mpcOutput;

	void initParams ();
	void initROS ();
	void checkForSolverErrors (int err);

	void sendControl ();

public:
	MPCWrapper ();

	int spin ();

	void updateInputData (const MPC::InputData &inputData);

	// Static callbacks
	static void inputCallback (const mpc_wrapper::InputDataMsg &inputDataMsg);
};


extern MPCWrapper *mpcWrapper;

#endif // MPC_WRAPPER_H
