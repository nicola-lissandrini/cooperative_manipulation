#include "velocity_integrator.h"

using namespace ros;
using namespace std;
using namespace tf;
using namespace XmlRpc;
using namespace Eigen;

VelocityIntegrator::VelocityIntegrator():
	started(false),
	seq(0)
{
	initParams ();
	initROS ();
}

void VelocityIntegrator::initParams ()
{
	try {
		nh.getParam ("velocity_integrator", params);

		sampleTime = 1 / double (params["rate"]);

		pid = new Pid (CONTROL_NO, sampleTime,
				double (params["pid"]["p"]),
				double (params["pid"]["i"]),
				double (params["pid"]["d"]));
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s\n",e.getMessage ().c_str ());
	}
}

void VelocityIntegrator::initROS ()
{
	rate = new Rate ((double) params["rate"]);

	velSub = nh.subscribe ((string) params["velocity_reference_topic"], 1, velocityCallback);
	stateSub = nh.subscribe ((string) params["state_topic"], 1, stateCallback);
	velPub = nh.advertise<geometry_msgs::Twist> ((string) params["velocity_output_topic"], 1);
}

void VelocityIntegrator::velocityCallback (const geometry_msgs::Twist &velMsg)
{
	if (!velocityIntegrator->isStarted ()) {
		velocityIntegrator->start ();
		ROS_WARN ("PID STARTED");
	}
	velocityIntegrator->setVelocity (velMsg);
}

void VelocityIntegrator::setVelocity (const geometry_msgs::Twist &velMsg) {
	currentVelMsg = velMsg;
}

void VelocityIntegrator::setOdometry (const nav_msgs::Odometry &poseMsg) {
	currentOdometry = poseMsg;
}

void VelocityIntegrator::stateCallback (const nav_msgs::Odometry &odometryMsg) {
	velocityIntegrator->setOdometry (odometryMsg);
}

void VelocityIntegrator::sendInput ()
{
	geometry_msgs::Twist inputMsg;
	VectorXd state = getStateVelocity ();
	VectorXd ref = getRef ();
	VectorXd control = pid->getControl (ref, state);

	inputMsg = controlToMsg (control);

	velPub.publish (inputMsg);
}

VectorXd VelocityIntegrator::getStateVelocity()
{
	VectorXd stateVel(CONTROL_NO);

	stateVel(0) = currentOdometry.twist.twist.linear.x;
	stateVel(1) = currentOdometry.twist.twist.linear.y;
	stateVel(2) = currentOdometry.twist.twist.linear.z;
	stateVel(3) = currentOdometry.twist.twist.angular.z;

	return stateVel;
}

VectorXd VelocityIntegrator::getRef ()
{
	VectorXd ref(CONTROL_NO);

	ref(0) = currentVelMsg.linear.x;
	ref(1) = currentVelMsg.linear.y;
	ref(2) = currentVelMsg.linear.z;
	ref(3) = currentVelMsg.angular.z;

	return ref;
}

geometry_msgs::Twist VelocityIntegrator::controlToMsg(const VectorXd &control)
{
	geometry_msgs::Twist controlMsg;

	controlMsg.linear.x = control(0);
	controlMsg.linear.y = control(1);
	controlMsg.linear.z = currentVelMsg.linear.z;
	controlMsg.angular.z = control(3);

	return controlMsg;
}

int VelocityIntegrator::spin ()
{
	while (ok ()) {
		if (started)
			sendInput ();
		spinOnce ();
		rate->sleep ();
	}

	return 0;
}



























