#include "output_interface.h"

#include <tf/tf.h>

using namespace ros;
using namespace std;
using namespace mpc_wrapper;
using namespace XmlRpc;
using namespace tf;

#include "mpc.h"

OutputInterface::OutputInterface()
{
	initParams ();
	initROS ();
}

int OutputInterface::spin()
{
	while (ok ()) {
		sendMessages ();
		
		spinOnce ();
		rate->sleep ();
	}
	
	return 0;
}

XmlRpc::XmlRpcValue OutputInterface::getParams() const {
	return params;
}

void OutputInterface::initParams ()
{
	try {
		nh.getParam ("mpc_output_interface", params);
		
		satValBase = (double) params["sat_base"];
		satValJoints = (double) params["sat_joints"];
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s", e.getMessage ().c_str ());
	}
}

void OutputInterface::initROS ()
{
	rate = new Rate (double (params["rate"]));
	
	controlSub = nh.subscribe ((string) params["mpc_control_topic"], 1, controlCallback);
	
	basePub = nh.advertise<geometry_msgs::Twist> ((string) params["base_cmd_vel_topic"], 1);
	jointsPub = nh.advertise<std_msgs::Float32MultiArray> ((string) params["joints_cmd_vel_topic"], 1);
	trajPub = nh.advertise<aerial_mpc::Trajectory> ((string) params["prediction_topic"], 1);
}

void OutputInterface::setControlMsg (const geometry_msgs::Twist &baseVel, const std_msgs::Float32MultiArray &jointsVel) {
	controlMsg.baseVel = baseVel;
	controlMsg.jointsVel = jointsVel;
}

void OutputInterface::setPrediction (const aerial_mpc::Trajectory &newPrediction) {
	prediction = newPrediction;
}

float OutputInterface::sat (float val, bool base) {
	float satVal = base ? satValBase : satValJoints;
	return (abs(val) > satVal? satVal * val/abs(val): val);
}

geometry_msgs::Twist OutputInterface::buildTwistMsg (const OutputDataMsg &controlMsg)
{
	const float baseLinScale = (double) outputInterface->getParams()["cmd_scale"]["lin_base_vel"];
	const float baseAngScale = (double) outputInterface->getParams()["cmd_scale"]["ang_base_vel"];
	const float x = controlMsg.currentControl.data[4];
	const float y = controlMsg.currentControl.data[5];
	const float psi = controlMsg.currentControl.data[6];

	geometry_msgs::Twist twistControl;

	twistControl.linear.x = outputInterface->sat(x, true) * baseLinScale;
	twistControl.linear.y = outputInterface->sat(y, true) * baseLinScale;
	twistControl.linear.z = 0;
	twistControl.angular.x = 0;
	twistControl.angular.y = 0;
	twistControl.angular.z = outputInterface->sat(psi, false) * baseAngScale;

	return twistControl;
}

template<typename T>
vector<T> vectorFromParam (XmlRpcValue &param)
{
	vector<T> vec;

	ROS_ASSERT (param.getType () == XmlRpcValue::TypeArray);

	vec.resize (param.size ());

	for (int i = 0; i < param.size (); i++) {
		vec[i] = (T) param[i];
	}

	return vec;
}

std_msgs::Float32MultiArray OutputInterface::buildJointsMsg (const OutputDataMsg &controlMsg)
{
	const float jointScale = (double) outputInterface->getParams ()["cmd_scale"]["joint_vel"];
	const vector<int> velocitySigns = vectorFromParam<int> (outputInterface->getParams ()["joint_signs"]);
	const float j1 = controlMsg.currentControl.data[0] * velocitySigns[0];
	const float j2 = controlMsg.currentControl.data[1] * velocitySigns[1];
	const float j3 = controlMsg.currentControl.data[2] * velocitySigns[2];
	const float j4 = controlMsg.currentControl.data[3] * velocitySigns[3];
	const float j5 = 0; // not controlled -- hardcoded 0

	std_msgs::Float32MultiArray jointsControl;

	jointsControl.layout.dim.resize (1);
	jointsControl.layout.dim[0].label = "Joints";
	jointsControl.layout.dim[0].size = JOINTS_N;
	jointsControl.layout.dim[0].stride = JOINTS_N;
	jointsControl.data = {outputInterface->sat(j1, false) * jointScale,
						  outputInterface->sat(j2, false) * jointScale,
						  outputInterface->sat(j3, false) * jointScale,
						  outputInterface->sat(j4, false) * jointScale,
						  outputInterface->sat(j5, false) * jointScale,
						  0};

	return jointsControl;
}

aerial_mpc::Trajectory OutputInterface::buildPredictionMsg (const OutputDataMsg &controlMsg)
{
	const int samples = controlMsg.stateTrajectory.layout.dim[1].size;
	const int states = controlMsg.stateTrajectory.layout.dim[0].size;
	aerial_mpc::Trajectory prediction;

	prediction.trajectory.resize (samples);

	for (int i = 0; i < samples; i++) {
		Point currPos(controlMsg.stateTrajectory.data[i * states],
				controlMsg.stateTrajectory.data[i * states + 1],
				controlMsg.stateTrajectory.data[i * states + 2]);

		Matrix3x3 currRot(controlMsg.stateTrajectory.data[i * states + 3],
				controlMsg.stateTrajectory.data[i * states + 4],
				controlMsg.stateTrajectory.data[i * states + 5],
				controlMsg.stateTrajectory.data[i * states + 6],
				controlMsg.stateTrajectory.data[i * states + 7],
				controlMsg.stateTrajectory.data[i * states + 8],
				controlMsg.stateTrajectory.data[i * states + 9],
				controlMsg.stateTrajectory.data[i * states + 10],
				controlMsg.stateTrajectory.data[i * states + 11]);
		currRot = currRot.transpose (); // switch from row major to column major
		Quaternion q;
		currRot.getRotation (q);

		pointTFToMsg (currPos, prediction.trajectory[i].position);
		quaternionTFToMsg (q, prediction.trajectory[i].orientation);
	}

	return prediction;
}

void OutputInterface::controlCallback (const OutputDataMsg &controlMsg)
{
	std_msgs::Float32MultiArray jointsControl;
	geometry_msgs::Twist twistControl;
	aerial_mpc::Trajectory prediction;

	jointsControl.data = {0, 0, 0, 0, 0, 0};
	
	if (controlMsg.solverStatus != 0) {
		ROS_INFO ("OUTPUT");
		ROS_WARN ("Unsolved QP. Supplying zero command");
		outputInterface->setControlMsg (twistControl, jointsControl);
		return;
	}

	twistControl = buildTwistMsg (controlMsg);
	jointsControl = buildJointsMsg (controlMsg);
	prediction = buildPredictionMsg (controlMsg);
	
	outputInterface->setControlMsg (twistControl,
									jointsControl);
	outputInterface->setPrediction (prediction);
}

void OutputInterface::sendMessages ()
{
	basePub.publish (controlMsg.baseVel);
	if (controlMsg.jointsVel.data.size () > 0)
		jointsPub.publish (controlMsg.jointsVel);
	trajPub.publish (prediction);
}





















