#include "output_interface.h"
#include "aerial_mpc/Trajectory.h"
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
		if (started)
			sendControl ();

		spinOnce ();
		rate->sleep ();
	}

	return 0;
}

void OutputInterface::initParams ()
{
	try {
		nh.getParam ("mpc_output_interface", params);

		satVal = (double) params["sat"];
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s", e.getMessage ().c_str ());
	}
}

void OutputInterface::initROS ()
{
	rate = new Rate (double (params["rate"]));

	controlSub = nh.subscribe ((string) params["mpc_control_topic"], 1, controlCallback);

	basePub = nh.advertise<geometry_msgs::TwistStamped> ((string) params["base_cmd_vel_topic"], 1);
	jointsPub = nh.advertise<std_msgs::Float32MultiArray> ((string) params["joints_cmd_vel_topic"], 1);
  trajPub = nh.advertise<aerial_mpc::Trajectory> ((string) params["prediction_topic"], 1);
}

void OutputInterface::setControlMsg (const geometry_msgs::TwistStamped &baseVel, const std_msgs::Float32MultiArray &jointsVel) {
	controlMsg.baseVel = baseVel;
	controlMsg.jointsVel = jointsVel;
}

double OutputInterface::sat (double val) {
	return (abs(val) > satVal? satVal * val/abs(val): val);
}

void OutputInterface::setPrediction (const aerial_mpc::Trajectory &newPrediction) {
	prediction = newPrediction;
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
void OutputInterface::controlCallback (const OutputDataMsg &controlMsg)
{
	const float baseLinScale = (double) outputInterface->getParams()["cmd_scale"]["lin_base_vel"];
	const float baseAngScale = (double) outputInterface->getParams()["cmd_scale"]["ang_base_vel"];
	const float jointScale = (double) outputInterface->getParams ()["cmd_scale"]["joint_vel"];
	const vector<int> velocitySigns = vectorFromParam<int> (outputInterface->getParams ()["joint_signs"]);

	geometry_msgs::TwistStamped twistControl;
	std_msgs::Float32MultiArray jointsControl;
	aerial_mpc::Trajectory prediction;

	if (!outputInterface->isStarted ())
		outputInterface->start ();

	const float j1 = controlMsg.currentControl.data[0]  * velocitySigns[0];
	const float j2 = controlMsg.currentControl.data[1]  * velocitySigns[1];
	const float x = controlMsg.currentControl.data[2];
	const float y = controlMsg.currentControl.data[3];
	const float z = controlMsg.currentControl.data[4];
	const float psi = controlMsg.currentControl.data[5];

	if (controlMsg.solverStatus != 0) {
		ROS_WARN ("Unsolved QP. Supplying zero command");
		outputInterface->setControlMsg (twistControl, jointsControl);
		return;
	}

	twistControl.twist.linear.x = outputInterface->sat(x) * baseLinScale;
	twistControl.twist.linear.y = outputInterface->sat(y) * baseLinScale;
	twistControl.twist.linear.z = outputInterface->sat (z) * baseLinScale;
	twistControl.twist.angular.x = 0;
	twistControl.twist.angular.y = 0;
	twistControl.twist.angular.z = outputInterface->sat(psi) * baseAngScale;

	jointsControl.layout.dim.resize (1);
	jointsControl.layout.dim[0].label = "Joints";
	jointsControl.layout.dim[0].size = JOINTS_N;
	jointsControl.layout.dim[0].stride = JOINTS_N;
	jointsControl.data = {outputInterface->sat(j1) *  jointScale,	// METTI STA ROBA NELLO YAML IDIOTA
						  outputInterface->sat(j2) *  jointScale};

	prediction = outputInterface->buildPredictionMsg (controlMsg);

	outputInterface->setControlMsg (twistControl,
									jointsControl);
	outputInterface->setPrediction (prediction);
}

void OutputInterface::sendControl ()
{
	basePub.publish (controlMsg.baseVel);
	jointsPub.publish (controlMsg.jointsVel);
	trajPub.publish (prediction);
}

aerial_mpc::Trajectory OutputInterface::buildPredictionMsg (const OutputDataMsg &controlMsg)
{
	const int samples = controlMsg.stateTrajectory.layout.dim[1].size;
	const int states = controlMsg.stateTrajectory.layout.dim[0].size;
	aerial_mpc::Trajectory prediction;

	prediction.trajectory.resize (samples);

	for (int i = 0; i < samples; i++) {
		// The trajectory we are interested to
		// publish in the aerial case is the one of the vehicle, not
		// the end effector
		Point currPos(controlMsg.stateTrajectory.data[i * states + 14],
				controlMsg.stateTrajectory.data[i * states + 15],
				controlMsg.stateTrajectory.data[i * states + 16]);

		Quaternion q;
		q.setRPY (0,0, controlMsg.stateTrajectory.data[i * states + 17]);

		pointTFToMsg (currPos, prediction.trajectory[i].position);
		quaternionTFToMsg (q, prediction.trajectory[i].orientation);
	}

	return prediction;
}


















