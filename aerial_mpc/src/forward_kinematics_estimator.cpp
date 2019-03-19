#include "forward_kinematics_estimator.h"

using namespace std;
using namespace ros;
using namespace XmlRpc;
using namespace tf;

void ForwardKinematicsEstimator::initParams()
{
	try {
		nh.getParam ("forward_kinematics_estimator", params);

		l1a = (double) params["params"]["l1a"];
		l1b = (double) params["params"]["l1b"];
		l2  = (double) params["params"]["l2"];
		lbx = (double) params["params"]["lbx"];
		lby = (double) params["params"]["lby"];
		lbz = (double) params["params"]["lbz"];

		currJoints.resize (JOINTS_NO);
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s\n",e.getMessage ().c_str ());
	}
}

ForwardKinematicsEstimator::ForwardKinematicsEstimator()
{
	initParams ();
	initRos ();

	currJoints[0] = 0;
	currJoints[1] = 0;
}

void ForwardKinematicsEstimator::initRos ()
{
	rate = new Rate (double (params["rate"]));

	baseSub = nh.subscribe ((string) params["base_state_topic"], 1, &ForwardKinematicsEstimator::baseCallback, this);
	jointsSub = nh.subscribe ((string) params["joints_state_topic"], 1, &ForwardKinematicsEstimator::jointsCallback, this);

	eefPub = nh.advertise<nav_msgs::Odometry> ((string) params["end_effector_topic"], 1);
}

void ForwardKinematicsEstimator::baseCallback (const nav_msgs::Odometry &baseOdom) {
	poseMsgToTF (baseOdom.pose.pose, currBase);
}

void ForwardKinematicsEstimator::jointsCallback (const std_msgs::Float32MultiArray &joints)
{
	if (joints.data.size () != JOINTS_NO) {
		ROS_ERROR("ForwardKinematicsEstimator: wrong joints msg size. Skipping.");
		return;
	}

	for (int i = 0; i < JOINTS_NO; i++)
		currJoints[i] = joints.data[i];
}

Vector3 ForwardKinematicsEstimator::forwardPosition (double j1, double j2,
													 double xv, double yv, double zv, double psi)
{
	double eex, eey, eez;
	eex = xv+lbx*cos(psi)-lby*sin(psi)+cos(psi)*(l1b*cos(j1)+l1a*sin(j1))+l2*cos(psi)*cos(j1)*cos(j2)-l2*cos(psi)*sin(j1)*sin(j2);
	eey = yv+lby*cos(psi)+lbx*sin(psi)+sin(psi)*(l1b*cos(j1)+l1a*sin(j1))+l2*cos(j1)*cos(j2)*sin(psi)-l2*sin(psi)*sin(j1)*sin(j2);
	eez = lbz+zv+l2*sin(j1+j2)-l1a*cos(j1)+l1b*sin(j1);

	return Vector3 (eex, eey, eez);
}

Matrix3x3 ForwardKinematicsEstimator::forwardOrientation (double j1, double j2,
														  double xv, double yv, double zv, double psi)
{
	return Matrix3x3 (cos(j1 + j2)*cos(psi), sin(j1 + j2)*cos(psi), -sin(psi),
					  cos(j1 + j2)*sin(psi), sin(j1 + j2)*sin(psi),  cos(psi),
							   sin(j1 + j2),         -cos(j1 + j2),         0);
}

void ForwardKinematicsEstimator::computeAndSend ()
{
	geometry_msgs::Pose eefPoseMsg;
	nav_msgs::Odometry odometryMsg;

	tf::Vector3 basePos = currBase.getOrigin ();
	tf::Matrix3x3 baseRot;
	tf::Vector3 eefPos;
	tf::Matrix3x3 eefRot;
	tf::Quaternion eefQuat;

	double r, p, psi;
	double j1 = currJoints[0] * double (params["joint_signs"][0]);
	double j2 = currJoints[1]* double (params["joint_signs"][1]);

	baseRot.setRotation (currBase.getRotation ());
	baseRot.getEulerYPR (psi, p, r);

	eefPos = forwardPosition (j1, j2, basePos.x (), basePos.y (), basePos.z (), psi);
	eefRot = forwardOrientation (j1, j2, basePos.x (), basePos.y (), basePos.z (), psi);
	eefRot.getRotation (eefQuat);

	pointTFToMsg (eefPos, eefPoseMsg.position);
	quaternionTFToMsg (eefQuat, eefPoseMsg.orientation);

	odometryMsg.pose.pose =  eefPoseMsg;

	eefPub.publish (odometryMsg);
}

int ForwardKinematicsEstimator::spin ()
{
	while (ok ()) {
		computeAndSend ();
		spinOnce ();
		rate->sleep ();
	}

	return 0;
}

int main (int argc, char *argv[])
{
	init (argc, argv, "forward_kinematic_estimator");
	ForwardKinematicsEstimator fke;

	return fke.spin ();
}
